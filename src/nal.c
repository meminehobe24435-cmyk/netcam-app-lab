/*
 * nal.c -- FU-A style media fragmentation and reassembly
 * ======================================================
 *
 * FRAGMENT FORMAT (RFC 6184 section 5.8, simplified subset)
 *
 *   Fragment payload = [FU indicator][FU header][fragment bytes ...]
 *
 *   FU indicator   0 1 2 3 4 5 6 7
 *                 +-+-+-+-+-+-+-+-+
 *                 |F|NRI|  Type   |   Type = 28 (FU-A)
 *                 +-+-+-+-+-+-+-+-+
 *   FU header      0 1 2 3 4 5 6 7
 *                 +-+-+-+-+-+-+-+-+
 *                 |S|E|R|  Type   |   S = start of the fragmented unit
 *                 +-+-+-+-+-+-+-+-+   E = end of the fragmented unit
 *                                     R = reserved, MUST be 0
 *                                     Type = the original NAL unit type
 *
 *   Reassembled NAL header byte = (FU indicator & 0xE0) | (FU header & 0x1F),
 *   followed by the concatenation of all fragment bytes.  We reconstruct the
 *   real header byte explicitly rather than trusting the sender to repeat it.
 *
 *   A NAL unit that fits in a single packet is transmitted UNFRAGMENTED:
 *   the RTP payload is the NAL unit itself (first byte = real NAL header,
 *   type != 28).  That path is exercised by the tests because a receiver
 *   that assumes "everything is FU-A" is a classic bug.
 *
 * REASSEMBLY RULES -- never emit half a frame
 *
 *   1. A unit starts at S and ends at E.  Fragments in between are appended
 *      in arrival order.
 *   2. A fragment with neither S nor E is only valid while a unit is in
 *      progress AND the last seen fragment was also a middle one.
 *   3. A duplicate middle fragment (same "which fragment am I" position) is
 *      ignored and counted -- appending it would corrupt the byte stream.
 *   4. A new S while a unit is in progress means the previous unit lost its
 *      tail: the whole previous unit is DISCARDED and counted as truncated.
 *   5. An E fragment that does not follow a valid middle fragment is refused
 *      with NTC_NAL_ERR_GAP.  There are two such shapes, and both mean the
 *      same thing to the caller -- one frame is lost:
 *        (a) fragments arrived with the S fragment missing entirely, and
 *        (b) two middle fragments arrived back to back, which proves an E
 *            was lost in between.
 *      In both cases every buffered fragment of the unit is dropped and the
 *      unit is counted once in `units_dropped`: we never emit half a frame.
 *
 * The "which fragment am I" position is tracked as `expected_frag_index`,
 * which increments once per accepted middle fragment.  That is enough to
 * detect a duplicated middle fragment and an out-of-order middle fragment
 * without carrying a bitmap, and it is what the tests assert on.
 */

#include "netcam.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* classification                                                      */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_nal_classify_payload(const uint8_t *payload, size_t len,
                                     const uint8_t **unit_out,
                                     size_t *unit_len_out,
                                     ntc_nal_fragment_info_t *frag_out)
{
    uint8_t type;

    if (payload == NULL || len == 0) {
        return NTC_ERR_INVAL;
    }
    if (unit_out != NULL) { *unit_out = NULL; }
    if (unit_len_out != NULL) { *unit_len_out = 0; }
    if (frag_out != NULL) { memset(frag_out, 0, sizeof(*frag_out)); }

    type = (uint8_t)(payload[0] & 0x1Fu);
    if (type != (uint8_t)NTC_NAL_TYPE_FU_A) {
        /* A complete, unfragmented NAL unit (or a non-FU packet type, which
         * this project deliberately does not support: no STAP-A, no MTAP). */
        if (type == 0u || type == 31u) {
            return NTC_NAL_ERR_BAD_TYPE;
        }
        if (unit_out != NULL) { *unit_out = payload; }
        if (unit_len_out != NULL) { *unit_len_out = len; }
        return NTC_OK;
    }

    if (len < 3u) {
        return NTC_NAL_ERR_BAD_FU; /* FU-A needs indicator+header+1 byte */
    }
    if (frag_out != NULL) {
        uint8_t fu = payload[1];
        frag_out->start = (fu & 0x80u) ? 1 : 0;
        frag_out->end = (fu & 0x40u) ? 1 : 0;
        frag_out->type = (int)(fu & 0x1Fu);
        frag_out->nri = (int)((payload[0] >> 5) & 0x03u);
        frag_out->data_len = len - 2u;
        frag_out->data = payload + 2u;
    }
    return NTC_OK;
}

/* ------------------------------------------------------------------ */
/* fragmentation (sender side)                                         */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_nal_fragment(const uint8_t *src, size_t src_len,
                              size_t frag_payload, uint8_t *out,
                              size_t out_cap, size_t *out_len,
                              size_t *frag_count_out)
{
    uint8_t type;
    size_t used = 0;
    size_t frags = 0;

    if (src == NULL || out == NULL || out_len == NULL || src_len == 0) {
        return NTC_ERR_INVAL;
    }
    if (frag_payload < 3u) {
        return NTC_ERR_RANGE; /* must leave room for indicator+header+data */
    }
    type = (uint8_t)(src[0] & 0x1Fu);
    if (type == 0u || type == 31u) {
        return NTC_NAL_ERR_BAD_TYPE;
    }
    if (out_cap < 2u) {
        return NTC_ERR_NO_ROOM;
    }
    /* Reserve the 2-byte total length prefix. */
    used = 2u;

    if (src_len <= frag_payload - 1u) {
        /* Fits in one packet: emit the NAL unit unfragmented. */
        if (used + 2u + src_len > out_cap) {
            return NTC_ERR_NO_ROOM;
        }
        out[used++] = (uint8_t)(src_len >> 8);
        out[used++] = (uint8_t)(src_len & 0xFFu);
        memcpy(out + used, src, src_len);
        used += src_len;
        frags = 1;
    } else {
        /*
         * Fragmented case.  The NAL unit's first byte IS the NAL header, and
         * the reassembler rebuilds that byte from the FU indicator and FU
         * header.  The fragment data therefore starts at src[1]: emitting
         * src[0] as data as well (which an earlier revision did) grows the
         * reassembled unit by one byte for every fragmented NAL unit.
         */
        size_t chunk = frag_payload - 2u;
        size_t off = 1;

        while (off < src_len) {
            size_t take = src_len - off;
            uint8_t indicator;
            uint8_t fu_hdr;
            int first = (off == 1u);
            int last;

            if (take > chunk) {
                take = chunk;
            }
            last = (off + take >= src_len);
            indicator = (uint8_t)((src[0] & 0xE0u) |
                                  (uint8_t)NTC_NAL_TYPE_FU_A);
            fu_hdr = (uint8_t)(((first ? 1u : 0u) << 7) |
                               ((last ? 1u : 0u) << 6) | type);
            if (used + 2u + 2u + take > out_cap) {
                return NTC_ERR_NO_ROOM;
            }
            out[used++] = (uint8_t)((2u + take) >> 8);
            out[used++] = (uint8_t)((2u + take) & 0xFFu);
            out[used++] = indicator;
            out[used++] = fu_hdr;
            memcpy(out + used, src + off, take);
            used += take;
            off += take;
            frags++;
        }
    }

    if (used > 0xFFFFu) {
        return NTC_ERR_NO_ROOM;
    }
    out[0] = (uint8_t)(used >> 8);
    out[1] = (uint8_t)(used & 0xFFu);
    *out_len = used;
    if (frag_count_out != NULL) { *frag_count_out = frags; }
    return NTC_OK;
}

/* ------------------------------------------------------------------ */
/* reassembly (receiver side)                                          */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_nal_reasm_init(ntc_nal_reasm_t *r)
{
    if (r == NULL) {
        return NTC_ERR_INVAL;
    }
    memset(r, 0, sizeof(*r));
    return NTC_OK;
}

/* Discard whatever unit is buffered, counting why. */
static void ntc_nal_discard(ntc_nal_reasm_t *r, int reason_truncated)
{
    if (r->in_progress || r->unit_len > 0) {
        r->stats.units_dropped++;
        if (reason_truncated) {
            r->stats.dropped_truncated++;
        } else {
            r->stats.dropped_missing_frag++;
        }
    }
    r->in_progress = 0;
    r->unit_len = 0;
    r->last_frag_started = 0;
    r->frags_seen = 0;
    r->expected_frag_index = 0;
    r->timestamp_valid = 0;
}

ntc_status_t ntc_nal_reasm_push_ex(ntc_nal_reasm_t *r, const uint8_t *payload,
                                   size_t len, uint32_t timestamp,
                                   long frag_index, int *completed_out)
{
    const uint8_t *unit = NULL;
    size_t unit_len = 0;
    ntc_nal_fragment_info_t frag;
    ntc_status_t st;

    if (completed_out != NULL) { *completed_out = 0; }
    if (r == NULL || payload == NULL || len == 0) {
        return NTC_ERR_INVAL;
    }
    r->stats.fragments_in++;
    r->frag_index = frag_index;

    st = ntc_nal_classify_payload(payload, len, &unit, &unit_len, &frag);
    if (st != NTC_OK) {
        return st;
    }

    if (unit != NULL) {
        /* Unfragmented single-packet NAL unit. */
        if (r->in_progress) {
            /* The previous unit never got its tail. */
            ntc_nal_discard(r, 1);
        }
        if (unit_len > sizeof(r->last_unit)) {
            r->stats.units_dropped++;
            r->stats.dropped_overflow++;
            return NTC_NAL_ERR_NO_ROOM;
        }
        memcpy(r->last_unit, unit, unit_len);
        r->last_unit_len = unit_len;
        r->last_unit_type = (uint8_t)(unit[0] & 0x1Fu);
        r->stats.units_complete++;
        r->stats.single_packet_units++;
        r->stats.bytes_reassembled += unit_len;
        if (completed_out != NULL) { *completed_out = 1; }
        return NTC_OK;
    }

    /* FU-A path. */
    if (frag.type == 0 || frag.type == 31) {
        ntc_nal_discard(r, 0);
        return NTC_NAL_ERR_BAD_TYPE;
    }

    if (frag.start) {
        if (r->in_progress) {
            /* A new unit started before the previous one ended: the previous
             * unit lost its tail.  Drop it, never concatenate. */
            ntc_nal_discard(r, 1);
        }
        if (frag_index >= 0 && frag_index != 0) {
            /* The start fragment claims to be fragment 0 but the caller says
             * otherwise: the stream is inconsistent. */
            ntc_nal_discard(r, 0);
            return NTC_NAL_ERR_SEQUENCE;
        }
        r->in_progress = 1;
        r->unit_type = (uint8_t)frag.type;
        r->unit_nri = (uint8_t)frag.nri;
        r->unit_len = 0;
        r->frags_seen = 0;
        r->expected_frag_index = 0;
        r->last_accepted_index = (frag_index >= 0) ? frag_index : 0;
        r->last_frag_started = 1;
        r->timestamp = timestamp;
        r->timestamp_valid = 1;
        /* The real NAL header byte is rebuilt from both FU bytes. */
        r->unit[r->unit_len++] =
            (uint8_t)((payload[0] & 0xE0u) | (uint8_t)frag.type);
        if (r->unit_len + frag.data_len > NTC_NAL_MAX_UNIT) {
            ntc_nal_discard(r, 0);
            r->stats.dropped_overflow++;
            return NTC_NAL_ERR_NO_ROOM;
        }
    } else {
        if (!r->in_progress) {
            /* A middle/end fragment with no start: the start fragment was
             * lost, so this fragment cannot belong to anything.  Report the
             * gap so the caller counts a damaged frame. */
            r->stats.dropped_missing_frag++;
            return NTC_NAL_ERR_GAP;
        }
        if (r->timestamp_valid && r->timestamp != timestamp) {
            /* Different frame's timestamp while ours is unfinished: the rest
             * of our unit never arrived. */
            r->stats.dropped_missing_frag++;
            ntc_nal_discard(r, 0);
            return NTC_NAL_ERR_GAP;
        }
        if (frag_index >= 0) {
            long expected_next = r->last_accepted_index + 1;
            if (frag_index < expected_next) {
                /* A fragment we already have: appending it again would
                 * duplicate bytes in the middle of the unit. */
                r->stats.duplicate_frags++;
                return NTC_OK;
            }
            if (frag_index > expected_next) {
                /* The fragments in between never arrived.  The unit cannot be
                 * rebuilt from what we hold, so drop all of it -- this is the
                 * "one lost fragment kills the whole frame" rule, and it is
                 * the reason real stacks pass the RTP sequence number in. */
                r->stats.out_of_order_frags +=
                    (uint64_t)(frag_index - expected_next);
                r->gaps_detected++;
                r->stats.dropped_missing_frag++;
                ntc_nal_discard(r, 0);
                return NTC_NAL_ERR_GAP;
            }
            r->last_accepted_index = frag_index;
        } else if (!r->last_frag_started) {
            /* Without a position we cannot see a missing middle fragment;
            * this branch only catches a sender that emits two middle
            * fragments with no end fragment, which is malformed anyway. */
            r->stats.dropped_missing_frag++;
            ntc_nal_discard(r, 0);
            return NTC_NAL_ERR_GAP;
        }
        r->frags_seen++;
        r->expected_frag_index++;
        if (!frag.end) {
            r->last_frag_started = 0;
        }
    }

    if (r->unit_len + frag.data_len > NTC_NAL_MAX_UNIT) {
        ntc_nal_discard(r, 0);
        r->stats.dropped_overflow++;
        return NTC_NAL_ERR_NO_ROOM;
    }
    memcpy(r->unit + r->unit_len, frag.data, frag.data_len);
    r->unit_len += frag.data_len;

    if (frag.end) {
        memcpy(r->last_unit, r->unit, r->unit_len);
        r->last_unit_len = r->unit_len;
        r->last_unit_type = r->unit_type;
        r->stats.units_complete++;
        r->stats.bytes_reassembled += r->unit_len;
        r->in_progress = 0;
        r->unit_len = 0;
        r->last_frag_started = 0;
        r->frags_seen = 0;
        r->expected_frag_index = 0;
        if (completed_out != NULL) { *completed_out = 1; }
    }
    return NTC_OK;
}

ntc_status_t ntc_nal_reasm_push(ntc_nal_reasm_t *r, const uint8_t *payload,
                                size_t len, uint32_t timestamp,
                                int *completed_out)
{
    return ntc_nal_reasm_push_ex(r, payload, len, timestamp, -1, completed_out);
}

ntc_status_t ntc_nal_reasm_flush(ntc_nal_reasm_t *r)
{
    if (r == NULL) {
        return NTC_ERR_INVAL;
    }
    ntc_nal_discard(r, 1);
    return NTC_OK;
}

ntc_status_t ntc_nal_reasm_stats(const ntc_nal_reasm_t *r,
                                 ntc_nal_stats_t *out)
{
    if (r == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    *out = r->stats;
    return NTC_OK;
}
