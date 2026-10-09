/*
 * rtp.c -- RTP packetisation, receiver reorder buffer, loss/duplicate
 *          detection, and the simulated network impairment model.
 * ==================================================================
 *
 * RTP FIXED HEADER (RFC 3550), 12 bytes when CC == 0:
 *
 *    0                   1                   2                   3
 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |V=2|P|X|  CC   |M|     PT      |       sequence number         |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |                           timestamp                           |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |                             SSRC                              |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *   V  version (2)      P  padding      X  header extension
 *   CC CSRC count       M  marker (last packet of an access unit)
 *   PT payload type (96 = dynamic, H.264)   seq: 16-bit, wraps
 *   timestamp: 32-bit, 90 kHz for video      SSRC: 32-bit stream id
 *   All multi-byte fields big endian.  Our packer always emits
 *   V=2, P=0, X=0, CC=0.
 *
 * REORDER BUFFER (receiver side)
 *
 *   expected_seq is the next sequence number we still owe the consumer.
 *   An arriving packet is:
 *     - a duplicate  if its seq was already seen        -> counted, dropped
 *     - late         if seq < expected_seq (we moved on) -> counted, dropped
 *     - in order     if seq == expected_seq               -> delivered now
 *     - out of order otherwise                            -> buffered
 *
 *   After every insertion we drain: while the slot at expected_seq is held,
 *   deliver it and advance.  If the number of buffered packets reaches
 *   reorder_window we flush (that is the "window pressure" path); if an
 *   arriving packet is more than reorder_window ahead of expected_seq we
 *   also flush what we have, which is exactly how a too-small window turns
 *   merely-late packets into declared losses.
 *
 *   A flush walks from expected_seq to the highest buffered seq; sequence
 *   numbers with no slot are declared lost (counted in `lost`), and the
 *   buffered ones are delivered in ascending order.  Everything below
 *   expected_seq must *not* be counted as lost, so the walk starts at
 *   expected_seq by construction.
 *
 * WRAPAROUND
 *   Sequence numbers are 16-bit and wrap.  All comparisons go through
 *   ntc_rtp_seq_lt/gt, which use the "distance less than 32768" rule:
 *       lt(a,b) <=> (int16_t)(a - b) < 0
 *   This is only meaningful for distances < 32768, which is why the
 *   reorder window is capped at 512.
 */

#include "netcam.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* sequence arithmetic                                                 */
/* ------------------------------------------------------------------ */

/*
 * RFC 1982 style serial number comparison for 16-bit RTP sequence numbers.
 *
 * DEFINITION (this is the convention the whole project uses)
 *   ntc_rtp_seq_lt(a, b) is true when a is OLDER than b, i.e. when the
 *   forward distance from a to b is a positive number of steps smaller than
 *   half the sequence space:
 *
 *       d = (uint16_t)(b - a)
 *       lt(a, b)  <=>  0 < d < 0x8000
 *
 *   So 1 < 2, 0 < 1, 0 < 65535 (0 is the oldest value before the wrap),
 *   65534 < 65535, and 32767 < 0 is still true while 32768 vs 0 is
 *   ambiguous (neither lt nor gt).
 *
 *   The subtraction order is the whole ballgame: writing `(uint16_t)(a - b)`
 *   inverts every comparison, and the receiver then classifies the next
 *   expected packet as an ancient duplicate -- a bug that was introduced
 *   twice during development and is now locked down by tests that pin both
 *   wrap directions and both boundary distances, plus a brute-force oracle
 *   check over 2000 sampled pairs.
 *
 *   The result is only meaningful for pairs closer than half the space,
 *   which is why the reorder window is capped far below 32768.
 */
int ntc_rtp_seq_lt(uint16_t a, uint16_t b)
{
    uint16_t d = (uint16_t)(b - a);
    return (d != 0u) && (d < 0x8000u);
}

int ntc_rtp_seq_gt(uint16_t a, uint16_t b)
{
    /* a > b is exactly lt(b, a) under modular ordering. */
    return ntc_rtp_seq_lt(b, a);
}

uint16_t ntc_rtp_seq_diff(uint16_t a, uint16_t b)
{
    return (uint16_t)((uint16_t)(b - a));
}

/* ------------------------------------------------------------------ */
/* header pack / parse                                                 */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_rtp_write_header(uint8_t *buf, size_t cap, int marker,
                                  uint8_t payload_type, uint16_t seq,
                                  uint32_t timestamp, uint32_t ssrc,
                                  size_t *written)
{
    if (buf == NULL) {
        return NTC_ERR_INVAL;
    }
    if (cap < NTC_RTP_HEADER_LEN) {
        if (written != NULL) { *written = NTC_RTP_HEADER_LEN; }
        return NTC_ERR_NO_ROOM;
    }
    buf[0] = (uint8_t)((2u << 6));              /* V=2 P=0 X=0 CC=0 */
    buf[1] = (uint8_t)(((marker ? 1u : 0u) << 7) | (payload_type & 0x7Fu));
    buf[2] = (uint8_t)(seq >> 8);
    buf[3] = (uint8_t)(seq & 0xFFu);
    buf[4] = (uint8_t)(timestamp >> 24);
    buf[5] = (uint8_t)((timestamp >> 16) & 0xFFu);
    buf[6] = (uint8_t)((timestamp >> 8) & 0xFFu);
    buf[7] = (uint8_t)(timestamp & 0xFFu);
    buf[8] = (uint8_t)(ssrc >> 24);
    buf[9] = (uint8_t)((ssrc >> 16) & 0xFFu);
    buf[10] = (uint8_t)((ssrc >> 8) & 0xFFu);
    buf[11] = (uint8_t)(ssrc & 0xFFu);
    if (written != NULL) { *written = NTC_RTP_HEADER_LEN; }
    return NTC_OK;
}

ntc_status_t ntc_rtp_parse(const uint8_t *buf, size_t len,
                           ntc_rtp_packet_t *pkt)
{
    size_t hdr;
    size_t payload_len;

    if (buf == NULL || pkt == NULL) {
        return NTC_ERR_INVAL;
    }
    memset(pkt, 0, sizeof(*pkt));
    if (len < NTC_RTP_HEADER_LEN) {
        return NTC_RTP_ERR_SHORT;
    }
    pkt->version = (uint8_t)((buf[0] >> 6) & 0x03u);
    pkt->padding = (uint8_t)((buf[0] >> 5) & 0x01u);
    pkt->extension = (uint8_t)((buf[0] >> 4) & 0x01u);
    pkt->csrc_count = (uint8_t)(buf[0] & 0x0Fu);
    pkt->marker = (uint8_t)((buf[1] >> 7) & 0x01u);
    pkt->payload_type = (uint8_t)(buf[1] & 0x7Fu);
    pkt->seq = (uint16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    pkt->timestamp = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16) |
                     ((uint32_t)buf[6] << 8) | (uint32_t)buf[7];
    pkt->ssrc = ((uint32_t)buf[8] << 24) | ((uint32_t)buf[9] << 16) |
                ((uint32_t)buf[10] << 8) | (uint32_t)buf[11];

    if (pkt->version != 2u) {
        return NTC_RTP_ERR_VERSION;
    }
    hdr = NTC_RTP_HEADER_LEN + (size_t)pkt->csrc_count * 4u;
    if (hdr > len) {
        return NTC_RTP_ERR_CSRC;
    }
    if (pkt->extension) {
        /* Extension header: 16-bit profile + 16-bit length in 32-bit words. */
        size_t ext_len;
        if (hdr + 4u > len) {
            return NTC_ERR_PARSE;
        }
        ext_len = ((size_t)buf[hdr + 2] << 8) | (size_t)buf[hdr + 3];
        hdr += 4u + ext_len * 4u;
        if (hdr > len) {
            return NTC_ERR_PARSE;
        }
    }
    payload_len = len - hdr;
    if (pkt->padding) {
        size_t pad = buf[len - 1];
        if (pad == 0u || pad > payload_len) {
            return NTC_ERR_PARSE; /* padding count cannot exceed the payload */
        }
        payload_len -= pad;
    }
    pkt->payload_len = payload_len;
    pkt->payload = buf + hdr;
    return NTC_OK;
}

ntc_status_t ntc_rtp_build(uint8_t *buf, size_t cap, int marker,
                           uint8_t payload_type, uint16_t seq,
                           uint32_t timestamp, uint32_t ssrc,
                           const uint8_t *payload, size_t payload_len,
                           size_t *written)
{
    size_t need = NTC_RTP_HEADER_LEN + payload_len;
    ntc_status_t st;

    if (buf == NULL || (payload == NULL && payload_len > 0)) {
        return NTC_ERR_INVAL;
    }
    if (cap < need) {
        if (written != NULL) { *written = need; }
        return NTC_ERR_NO_ROOM;
    }
    st = ntc_rtp_write_header(buf, cap, marker, payload_type, seq, timestamp,
                              ssrc, NULL);
    if (st != NTC_OK) {
        return st;
    }
    if (payload_len > 0) {
        memcpy(buf + NTC_RTP_HEADER_LEN, payload, payload_len);
    }
    if (written != NULL) { *written = need; }
    return NTC_OK;
}

/* ------------------------------------------------------------------ */
/* sender: chop a frame into RTP datagrams                             */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_rtp_sender_init(ntc_rtp_sender_t *s, uint32_t ssrc,
                                 uint8_t payload_type, size_t max_payload)
{
    if (s == NULL) {
        return NTC_ERR_INVAL;
    }
    if (max_payload < 3 || max_payload > NTC_RTP_MAX_PAYLOAD) {
        return NTC_ERR_RANGE;
    }
    memset(s, 0, sizeof(*s));
    s->ssrc = ssrc;
    s->payload_type = payload_type;
    s->next_seq = (uint16_t)(ssrc & 0xFFFFu); /* deterministic start */
    s->timestamp = 0;
    s->max_payload = max_payload;
    return NTC_OK;
}

ntc_status_t ntc_rtp_send_frame(ntc_rtp_sender_t *s,
                                const uint8_t *const *nal_units,
                                const size_t *nal_lens, size_t count,
                                uint8_t **out_dgrams, size_t *out_lens,
                                size_t dgram_cap, size_t *out_count,
                                uint32_t *out_ts)
{
    size_t produced = 0;
    size_t i;

    if (s == NULL || out_count == NULL) {
        return NTC_ERR_INVAL;
    }
    if (count > 0 && (nal_units == NULL || nal_lens == NULL)) {
        return NTC_ERR_INVAL;
    }
    if (count > 0 && (out_dgrams == NULL || out_lens == NULL)) {
        return NTC_ERR_INVAL;
    }
    *out_count = 0;

    for (i = 0; i < count; i++) {
        const uint8_t *src = nal_units[i];
        size_t len = nal_lens[i];
        size_t hdr_type;

        if (src == NULL) {
            continue;
        }
        if (len == 0) {
            continue;
        }
        hdr_type = (size_t)(src[0] & 0x1Fu);
        if (hdr_type == 0u || hdr_type == 31u) {
            return NTC_NAL_ERR_BAD_TYPE; /* forbidden / reserved */
        }

        if (len <= s->max_payload - 1u) {
            /* Small NAL unit: one packet, payload is the NAL unit itself. */
            uint8_t *dg;
            size_t written = 0;
            int is_last_nal = (i + 1u == count);

            if (produced >= dgram_cap) {
                return NTC_ERR_FULL;
            }
            dg = (uint8_t *)malloc(NTC_RTP_HEADER_LEN + len);
            if (dg == NULL) {
                return NTC_ERR_NOMEM;
            }
            if (ntc_rtp_build(dg, NTC_RTP_HEADER_LEN + len, is_last_nal,
                              s->payload_type, s->next_seq, s->timestamp,
                              s->ssrc, src, len, &written) != NTC_OK) {
                free(dg);
                return NTC_ERR_NO_ROOM;
            }
            out_dgrams[produced] = dg;
            out_lens[produced] = written;
            produced++;
            s->next_seq = (uint16_t)(s->next_seq + 1u);
        } else {
            /*
             * Fragmented path.  The fragment data starts at src[1]: src[0] is
             * the NAL header byte, which the receiver rebuilds from the FU
             * indicator and FU header.  Sending src[0] as data as well (which
             * an earlier revision did) makes every reassembled NAL unit one
             * byte longer than the one that was sent.
             */
            size_t chunk = s->max_payload - 2u; /* 2 bytes of FU headers */
            size_t off = 1;
            uint8_t indicator =
                (uint8_t)((src[0] & 0xE0u) | (uint8_t)NTC_NAL_TYPE_FU_A);
            uint8_t base_type = (uint8_t)(src[0] & 0x1Fu);
            int first = 1;

            while (off < len) {
                size_t take = len - off;
                uint8_t *dg;
                size_t written = 0;
                int last;
                uint8_t fu_hdr;

                if (take > chunk) {
                    take = chunk;
                }
                last = (off + take >= len);
                if (produced >= dgram_cap) {
                    return NTC_ERR_FULL;
                }
                fu_hdr = (uint8_t)(((first ? 1u : 0u) << 7) |
                                   ((last ? 1u : 0u) << 6) | base_type);
                dg = (uint8_t *)malloc(NTC_RTP_HEADER_LEN + 2u + take);
                if (dg == NULL) {
                    return NTC_ERR_NOMEM;
                }
                if (ntc_rtp_write_header(dg, NTC_RTP_HEADER_LEN + 2u + take,
                                         last && (i + 1u == count),
                                         s->payload_type, s->next_seq,
                                         s->timestamp, s->ssrc,
                                         &written) != NTC_OK) {
                    free(dg);
                    return NTC_ERR_NO_ROOM;
                }
                dg[NTC_RTP_HEADER_LEN + 0] = indicator;
                dg[NTC_RTP_HEADER_LEN + 1] = fu_hdr;
                memcpy(dg + NTC_RTP_HEADER_LEN + 2u, src + off, take);
                out_dgrams[produced] = dg;
                out_lens[produced] = NTC_RTP_HEADER_LEN + 2u + take;
                produced++;
                s->next_seq = (uint16_t)(s->next_seq + 1u);
                off += take;
                first = 0;
            }
        }
    }

    if (out_ts != NULL) {
        *out_ts = s->timestamp;
    }
    /* 90 kHz clock, 30 fps -> 3000 ticks per frame.  Not a real capture
     * timer: the simulation steps frames explicitly. */
    s->timestamp += 3000u;
    *out_count = produced;
    return NTC_OK;
}

/* ------------------------------------------------------------------ */
/* receiver reorder buffer                                             */
/* ------------------------------------------------------------------ */
/*
 * Ring layout: index = seq % NTC_RTP_MAX_REORDER.  A slot is live only when
 * in_use is set AND slot.seq == the sequence number we are looking for --
 * otherwise it is a leftover from an earlier revolution of the ring and must
 * be ignored.  Two helper predicates below encapsulate that rule, because
 * getting it wrong was one of the bugs the tests caught (see README).
 */

static size_t ntc_rx_index(uint16_t seq)
{
    return (size_t)(seq % (uint16_t)NTC_RTP_MAX_REORDER);
}

static int ntc_rx_holds(const ntc_rtp_rx_t *rx, uint16_t seq)
{
    size_t i = ntc_rx_index(seq);
    return rx->ring[i].in_use && rx->ring[i].seq == seq;
}

/*
 * ntc_rx_queued() is "this slot holds a packet that has already been moved
 * to the delivery queue".  An earlier revision called this predicate
 * ntc_rx_ready() and negated it in the drain condition, i.e. it drained
 * "while NOT ready", which is never true for the packet that just arrived --
 * so nothing was ever handed to the consumer.  Naming it after the state it
 * actually tests removes the double negative that hid the bug.
 */
static int ntc_rx_queued(const ntc_rtp_rx_t *rx, uint16_t seq)
{
    size_t i = ntc_rx_index(seq);
    return rx->ring[i].in_use && rx->ring[i].seq == seq &&
           rx->ring[i].delivered != 0;
}

static int ntc_rx_seen(const ntc_rtp_rx_t *rx, uint16_t seq)
{
    return rx->seen[ntc_rx_index(seq)] != 0u;
}

static void ntc_rx_mark_seen(ntc_rtp_rx_t *rx, uint16_t seq)
{
    rx->seen[ntc_rx_index(seq)] = 1u;
}

/*
 * NOTE: there is deliberately no "rebuild the seen map from the ring" step.
 * An earlier revision cleared the map after every flush and re-derived it
 * from the ring, which threw away the marks for sequence numbers that had
 * been declared lost -- so the next flush counted them again (510 losses for
 * a 400-packet stream).  The seen map now records exactly one thing: "a
 * packet with this sequence number arrived".  Gaps are remembered separately
 * in `declared`, so neither map can be used to reconstruct the other.
 */

/*
 * Slide the expected window forward when it is more than one ring length
 * away from `seq`.  Everything still in the ring lives in a different
 * revolution and can never be matched again, so it is discarded.
 */
static void ntc_rx_slide(ntc_rtp_rx_t *rx, uint16_t seq)
{
    size_t k;
    rx->held = 0;
    rx->in_flight = 0;
    rx->dropped_stale += rx->queued;
    rx->queued = 0;
    for (k = 0; k < NTC_RTP_MAX_REORDER; k++) {
        rx->ring[k].in_use = 0;
        rx->ring[k].delivered = 0;
    }
    memset(rx->seen, 0, sizeof(rx->seen));
    memset(rx->declared, 0, sizeof(rx->declared));
    memset(rx->accounted, 0, sizeof(rx->accounted));
    rx->expected_seq = seq;
    rx->next_seq = seq;
    rx->expected_valid = 1;
}

/*
 * Release everything the receiver is holding because the window has been
 * exceeded, and declare the missing sequence numbers lost.
 *
 *   from       - first sequence number to walk (the gap starts at next_seq,
 *                or one past the expected packet when that packet is the one
 *                that just arrived out of order).
 *   keep       - a sequence number that must NOT be counted as lost, because
 *                the packet is in the caller's hand right now.  Ignored when
 *                `have_keep` is 0.
 *
 * The walk never spans more than the reorder ring itself.  A buffered packet
 * further than that from `from` cannot be trusted: its ring slot would
 * already have been recycled, so it belongs to an older revolution and is
 * discarded rather than counted.  Without this bound a single stale slot
 * makes the walk declare hundreds of phantom losses (the receiver reported
 * 763 losses for a 400-packet stream before the bound was added).
 */
static void ntc_rx_flush_from(ntc_rtp_rx_t *rx, uint16_t from, uint16_t keep,
                              int have_keep)
{
    uint16_t highest = from;
    uint16_t cap;
    int have = 0;
    uint16_t s;
    size_t k;

    for (k = 0; k < NTC_RTP_MAX_REORDER; k++) {
        uint16_t cand = rx->ring[k].seq;
        if (!rx->ring[k].in_use || rx->ring[k].delivered) {
            continue;
        }
        if (!ntc_rtp_seq_gt(cand, from) && cand != from) {
            continue; /* behind the walk: a different revolution */
        }
        if (ntc_rtp_seq_diff(from, cand) >= NTC_RTP_MAX_REORDER) {
            continue; /* too far ahead to be part of this window */
        }
        if (!have || ntc_rtp_seq_gt(cand, highest)) {
            highest = cand;
            have = 1;
        }
    }
    cap = highest;
    if (have_keep) {
        if (!have || ntc_rtp_seq_gt(keep, highest)) {
            highest = keep;
        }
        /*
         * The arriving packet is not lost and must not be released here: the
         * caller is about to store it and hand it to the consumer in order.
         * The walk therefore stops one short of it.  Releasing it instead
         * made the receiver deliver the same packet twice, which showed up as
         * more delivered packets than were ever sent.
         */
        cap = (uint16_t)(keep - 1u);
    }
    (void)have;

    /*
     * Never declare a loss beyond the highest sequence number that has
     * actually shown up.  A gap only becomes a loss when we have proof that
     * the stream moved past it.  Without this bound a single arrival far
     * ahead declares every number up to that point lost, and the reported
     * loss count can exceed the number of packets the sender ever sent.
     */
    if (ntc_rtp_seq_gt(highest, rx->max_seen)) {
        highest = rx->max_seen;
    }
    if (ntc_rtp_seq_gt(highest, cap)) {
        highest = cap;
    }
    if (ntc_rtp_seq_lt(highest, from)) {
        /* Nothing to walk: the whole gap lies beyond the observed stream. */
        return;
    }
    /* Everything from `from` to `highest` is either released or lost.  A
     * missing number is counted once, ever: `declared` remembers the gaps we
     * have already reported so a later flush walking the same range cannot
     * count them twice. */
    s = from;
    for (;;) {
        size_t i = ntc_rx_index(s);
        if (ntc_rx_holds(rx, s) && !ntc_rx_queued(rx, s)) {
            rx->ring[i].delivered = 1;
            rx->ring[i].state = NTC_RTP_SLOT_DELIVERED;
            rx->held--;
            rx->queued++;
        } else if (!rx->accounted[i] && !rx->declared[i]) {
            /* Never received and never counted before: this gap is a loss.
             * `accounted` makes the bookkeeping exact -- a sequence number
             * enters it exactly once, either as a delivery or as a loss. */
            rx->declared[i] = 1;
            rx->accounted[i] = 1;
            rx->lost++;
        }
        if (s == highest) {
            break;
        }
        s = (uint16_t)(s + 1u);
    }

    /* next_seq is the oldest sequence number we still expect to receive: it
     * must sit strictly after everything the walk above disposed of. */
    rx->next_seq = (uint16_t)(highest + 1u);
    if (ntc_rtp_seq_lt(rx->next_seq, rx->expected_seq)) {
        rx->next_seq = rx->expected_seq;
    }
}

ntc_status_t ntc_rtp_rx_init(ntc_rtp_rx_t *rx, size_t reorder_window)
{
    if (rx == NULL) {
        return NTC_ERR_INVAL;
    }
    if (reorder_window < 1 || reorder_window > NTC_RTP_MAX_REORDER) {
        return NTC_ERR_RANGE;
    }
    memset(rx, 0, sizeof(*rx));
    rx->reorder_window = reorder_window;
    rx->expected_valid = 0;
    rx->ssrc_valid = 0;
    return NTC_OK;
}
ntc_status_t ntc_rtp_rx_push(ntc_rtp_rx_t *rx, const uint8_t *buf, size_t len,
                             int *arrived_out_of_order, int *duplicate_out)
{
    ntc_rtp_packet_t pkt;
    ntc_rtp_packet_t saved;
    uint8_t saved_payload[NTC_RTP_MAX_DATAGRAM];
    ntc_status_t st;
    size_t idx;

    if (rx == NULL || buf == NULL) {
        return NTC_ERR_INVAL;
    }
    if (arrived_out_of_order != NULL) { *arrived_out_of_order = 0; }
    if (duplicate_out != NULL) { *duplicate_out = 0; }

    st = ntc_rtp_parse(buf, len, &pkt);
    if (st != NTC_OK) {
        rx->rejected++;
        return st;
    }
    if (pkt.payload_len > NTC_RTP_MAX_DATAGRAM) {
        rx->rejected++;
        return NTC_ERR_NO_ROOM;
    }

    if (!rx->ssrc_valid) {
        rx->ssrc = pkt.ssrc;
        rx->ssrc_valid = 1;
    } else if (rx->ssrc != pkt.ssrc) {
        /* A new stream id on the same port: restart the buffer, count it. */
        size_t k;
        rx->ssrc_changes++;
        rx->ssrc = pkt.ssrc;
        rx->expected_valid = 0;
        rx->held = 0;
        rx->in_flight = 0;
        rx->queued = 0;
        for (k = 0; k < NTC_RTP_MAX_REORDER; k++) {
            rx->ring[k].in_use = 0;
            rx->ring[k].delivered = 0;
        }
        memset(rx->seen, 0, sizeof(rx->seen));
        memset(rx->declared, 0, sizeof(rx->declared));
        memset(rx->accounted, 0, sizeof(rx->accounted));
    }

    /*
     * The window must be established BEFORE any comparison uses
     * expected_seq: with modular ordering a fresh receiver (expected_seq 0)
     * would otherwise classify the first packet as an ancient one and drop
     * it.
     *
     * Do NOT mark the packet as seen here.  The seen-map is the duplicate
     * detector, and marking the packet before it has been stored makes the
     * duplicate check further down reject the very packet we are holding --
     * the receiver then reports "duplicates" while delivering nothing.  The
     * seen bit is set by the store path below, which is the only place a
     * packet becomes "already seen".
     */
    if (!rx->expected_valid) {
        rx->expected_seq = pkt.seq;
        rx->next_seq = pkt.seq;
        rx->max_seen = pkt.seq;
        rx->expected_valid = 1;
    } else if (ntc_rtp_seq_gt(pkt.seq, rx->max_seen)) {
        rx->max_seen = pkt.seq;
    }

    rx->received++;

    /* Keep a private copy so that a window slide cannot discard the packet
     * we are about to store. */
    saved = pkt;
    if (saved.payload_len > 0) {
        memcpy(saved_payload, pkt.payload, pkt.payload_len);
        saved.payload = saved_payload;
    }

    /*
     * Duplicate detection comes FIRST, and it has to: a re-injected packet is
     * an exact copy of one we already handled, so it must be reported as a
     * duplicate.  Checking "is it older than what we are waiting for?" first
     * would label every re-injection as a late arrival instead, and a caller
     * could then not tell "the network duplicated this" from "this arrived
     * after its gap was declared lost" -- two very different repairs.
     */
    if (ntc_rx_seen(rx, pkt.seq)) {
        /* The sequence number was already counted when it first arrived. */
        rx->duplicates++;
        if (duplicate_out != NULL) { *duplicate_out = 1; }
        return NTC_OK;
    }

    /*
     * Window pressure: the window is exhausted when the gap between the
     * arriving sequence number and what we are waiting for is at least the
     * window.  The comparison is >= and not >, on purpose: a gap exactly
     * equal to the window already fills it, so holding the packet would leave
     * the missing numbers undeclared and reported as zero loss while the
     * receiver silently drops data.
     */
    if (ntc_rtp_seq_diff(rx->next_seq, pkt.seq) >= rx->reorder_window) {
        uint16_t from = rx->next_seq;
        int have_keep = 0;
        uint16_t keep = 0;
        if (pkt.seq == rx->next_seq) {
            /* The packet we were waiting for is the one arriving now with a
             * gap after it: the gap starts at the following number. */
            from = (uint16_t)(rx->next_seq + 1u);
        } else {
            /* The arriving packet is ahead of the window; it is not lost, it
             * is the packet being processed. */
            have_keep = 1;
            keep = pkt.seq;
        }
        ntc_rx_flush_from(rx, from, keep, have_keep);
        rx->flushed_by_pressure++;
    }

    if (ntc_rtp_seq_lt(pkt.seq, rx->next_seq)) {
        /*
         * Older than anything we are still waiting for: the window has
         * moved past it, so it can never be delivered.
         *
         * It is counted as LATE when the loss walk already reported this
         * position as a gap (then it is the same missing packet turning up
         * too late).  If the walk never covered it -- which happens for
         * the first packet of a stream, where the receiver has no evidence
         * that earlier sequence numbers ever belonged to the flow -- then
         * this arrival is the only evidence, and it is counted as a loss.
         * Without that rule the loss counter under-reports the gaps and the
         * two ways of counting disagree.
         */
        size_t li = ntc_rx_index(pkt.seq);
        if (rx->declared[li]) {
            rx->late++;
        } else {
            rx->declared[li] = 1;
            rx->accounted[li] = 1;
            rx->lost++;
        }
        if (arrived_out_of_order != NULL) { *arrived_out_of_order = 1; }
        return NTC_OK;
    }

    /* The arrival survived the flush.  It may be far enough ahead to need
     * the ring itself to be slid forward, and it must be re-read from the
     * saved copy because the flush is allowed to have rewritten the ring. */
    if (ntc_rtp_seq_diff(rx->next_seq, saved.seq) >= NTC_RTP_MAX_REORDER) {
        ntc_rx_slide(rx, saved.seq);
    }
    pkt = saved;

    if (ntc_rtp_seq_gt(pkt.seq, rx->next_seq)) {
        rx->out_of_order++;
        rx->reordered_in++;
        if (arrived_out_of_order != NULL) { *arrived_out_of_order = 1; }
    }

    idx = ntc_rx_index(pkt.seq);
    if (rx->ring[idx].in_use) {
        /* Another revolution's packet occupies the slot: discard it. */
        if (rx->ring[idx].delivered) {
            rx->queued--;
        } else {
            rx->held--;
        }
        rx->in_flight--;
    }
    rx->ring[idx].in_use = 1;
    rx->ring[idx].delivered = 0;
    rx->ring[idx].state = NTC_RTP_SLOT_HELD;
    rx->ring[idx].seq = pkt.seq;
    rx->ring[idx].timestamp = pkt.timestamp;
    rx->ring[idx].marker = pkt.marker;
    rx->ring[idx].payload_len = pkt.payload_len;
    if (pkt.payload_len > 0) {
        memcpy(rx->ring[idx].payload, pkt.payload, pkt.payload_len);
    }
    rx->held++;
    rx->in_flight++;
    ntc_rx_mark_seen(rx, pkt.seq);
    rx->accounted[ntc_rx_index(pkt.seq)] = 1;

    /*
     * In-order drain.  A packet is ready for the consumer when it is the one
     * next_seq points at; the drain therefore compares against next_seq, not
     * against expected_seq (which tracks the oldest packet the consumer has
     * not collected yet).  Conflating the two is what made the first version
     * of this loop never fire: it looked for a held packet at expected_seq,
     * which had already been marked delivered one step earlier.
     */
    /*
     * In-order drain.  A packet is handed to the consumer as soon as it is
     * the one next_seq points at: it is held, and it has not been queued
     * already.
     */
    while (rx->in_flight > 0 && ntc_rx_holds(rx, rx->next_seq) &&
           !ntc_rx_queued(rx, rx->next_seq)) {
        rx->ring[ntc_rx_index(rx->next_seq)].delivered = 1;
        rx->ring[ntc_rx_index(rx->next_seq)].state = NTC_RTP_SLOT_DELIVERED;
        rx->held--;
        rx->queued++;
        rx->next_seq = (uint16_t)(rx->next_seq + 1u);
    }

    /* expected_seq trails the consumer: it is the oldest sequence number
     * that is either still buffered or still queued.  Once the consumer has
     * collected everything, it catches up with next_seq.  Guarded by seq_lt
     * so that a flush (which moves next_seq forward) can never make this
     * loop run backwards or forever. */
    while (ntc_rtp_seq_lt(rx->expected_seq, rx->next_seq) &&
           !ntc_rx_holds(rx, rx->expected_seq)) {
        rx->expected_seq = (uint16_t)(rx->expected_seq + 1u);
    }

    if (rx->held >= rx->reorder_window) {
        size_t lost = 0, moved = 0;
        (void)ntc_rtp_rx_flush(rx, &lost, &moved);
        rx->flushed_by_pressure++;
    }
    return NTC_OK;
}

ntc_status_t ntc_rtp_rx_pop(ntc_rtp_rx_t *rx, ntc_rtp_delivery_t *out)
{
    size_t k;
    size_t best = NTC_RTP_MAX_REORDER;
    if (rx == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    for (k = 0; k < NTC_RTP_MAX_REORDER; k++) {
        if (!rx->ring[k].in_use || !rx->ring[k].delivered) {
            continue;
        }
        if (best == NTC_RTP_MAX_REORDER) {
            best = k;
        } else {
            /* Deliver in ascending sequence order even though the ring is
             * indexed by seq % N; pick the smallest window distance from the
             * ring start we already selected. */
            uint16_t cur = rx->ring[k].seq;
            uint16_t ref = rx->ring[best].seq;
            if (ntc_rtp_seq_lt(cur, ref)) {
                best = k;
            }
        }
    }
    if (best == NTC_RTP_MAX_REORDER) {
        return NTC_ERR_EMPTY;
    }

    out->seq = rx->ring[best].seq;
    out->timestamp = rx->ring[best].timestamp;
    out->marker = rx->ring[best].marker;
    out->payload_len = rx->ring[best].payload_len;
    if (out->payload_len > 0) {
        memcpy(out->payload, rx->ring[best].payload, out->payload_len);
    }
    rx->ring[best].in_use = 0;
    rx->ring[best].delivered = 0;
    rx->ring[best].state = NTC_RTP_SLOT_EMPTY;
    rx->queued--;
    rx->in_flight--;
    rx->delivered++;
    return NTC_OK;
}

ntc_status_t ntc_rtp_rx_flush(ntc_rtp_rx_t *rx, size_t *lost_out,
                              size_t *moved_out)
{
    size_t lost_before;

    if (rx == NULL) {
        return NTC_ERR_INVAL;
    }
    if (!rx->expected_valid) {
        if (lost_out != NULL) { *lost_out = 0; }
        if (moved_out != NULL) { *moved_out = 0; }
        return NTC_OK;
    }
    lost_before = (size_t)rx->lost;
    /* End of stream / marker boundary: no packet is "in hand", so nothing is
     * exempt from being declared lost. */
    ntc_rx_flush_from(rx, rx->next_seq, 0u, 0);
    if (lost_out != NULL) { *lost_out = (size_t)rx->lost - lost_before; }
    if (moved_out != NULL) { *moved_out = rx->queued; }
    return NTC_OK;
}

ntc_status_t ntc_rtp_rx_stats(const ntc_rtp_rx_t *rx, ntc_rtp_stats_t *out)
{
    if (rx == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    out->received = rx->received;
    out->delivered = rx->delivered;
    out->duplicates = rx->duplicates;
    out->lost = rx->lost;
    out->late = rx->late;
    out->reordered_in = rx->reordered_in;
    out->out_of_order = rx->out_of_order;
    out->ssrc_changes = rx->ssrc_changes;
    out->flushed_by_pressure = rx->flushed_by_pressure;
    out->rejected = rx->rejected;
    out->held = rx->held;
    return NTC_OK;
}