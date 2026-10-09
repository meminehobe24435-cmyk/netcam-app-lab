/*
 * record.c -- recording index with time-range retrieval and ring-buffer
 *             (oldest-first) capacity eviction.
 * ==========================================================
 *
 * DATA MODEL
 *
 *   One entry per recorded clip:
 *     [start_ms, end_ms)  half-open interval, start_ms < end_ms
 *     bytes               size on "disk"
 *     reason              bitmap of NTC_MOTION_LINK_* that caused the clip
 *     id                  monotonic, never reused
 *
 *   The table is kept SORTED BY start_ms.  Every operation restores that
 *   invariant, which is what makes retrieval a simple linear scan and keeps
 *   "the oldest segment is segs[0]" true for eviction.
 *
 * INSERT: MERGE-OR-ADD
 *
 *   A motion event that re-triggers every few seconds must not create one
 *   clip per trigger, so a new segment whose start is within
 *   `merge_gap_ms` after the tail of an existing segment is MERGED into it:
 *     seg.end_ms  = max(seg.end_ms, new.end_ms)
 *     seg.bytes  += new.bytes
 *     seg.reason |= new.reason
 *   If that merged segment then overlaps the NEXT segment (possible when the
 *   new clip is long), the two are merged as well and the process repeats
 *   backwards until the invariant holds.  Without that step the table could
 *   end up with overlapping intervals, which makes range queries ambiguous.
 *   A gap larger than merge_gap_ms simply creates a new segment.
 *
 * EVICTION
 *
 *   The store has a byte budget (0 = unlimited).  Before inserting we evict
 *   segs[0] (the oldest) repeatedly until the new segment fits.  The segment
 *   being inserted is never a candidate for eviction, and eviction cannot
 *   free memory below zero bytes.
 *
 * RETRIEVAL
 *
 *   ntc_recorder_query(a, b) returns every segment that INTERSECTS [a, b):
 *       hit  <=>  seg.end_ms > a  &&  seg.start_ms < b
 *   A zero-width window (a == b) therefore matches nothing, which the tests
 *   pin down explicitly.
 */

#include "netcam.h"

#include <string.h>

ntc_status_t ntc_recorder_init(ntc_recorder_t *r, size_t capacity_bytes,
                               double merge_gap_ms)
{
    if (r == NULL) {
        return NTC_ERR_INVAL;
    }
    memset(r, 0, sizeof(*r));
    r->capacity_bytes = capacity_bytes;
    r->merge_gap_ms = (merge_gap_ms < 0.0) ? 0.0 : merge_gap_ms;
    r->next_id = 1;
    return NTC_OK;
}

ntc_status_t ntc_recorder_evict_oldest(ntc_recorder_t *r)
{
    size_t i;

    if (r == NULL) {
        return NTC_ERR_INVAL;
    }
    if (r->count == 0) {
        return NTC_ERR_EMPTY;
    }
    r->stats.evictions++;
    r->stats.evicted_bytes += r->segs[0].bytes;
    if (r->used_bytes >= r->segs[0].bytes) {
        r->used_bytes -= r->segs[0].bytes;
    } else {
        r->used_bytes = 0;
    }
    for (i = 1; i < r->count; i++) {
        r->segs[i - 1] = r->segs[i];
    }
    r->count--;
    memset(&r->segs[r->count], 0, sizeof(r->segs[r->count]));
    r->oldest_id = (r->count > 0) ? r->segs[0].id : 0;
    return NTC_OK;
}

ntc_status_t ntc_recorder_insert(ntc_recorder_t *r, double start_ms,
                                 double end_ms, size_t bytes, uint32_t reason,
                                 uint64_t *id_out)
{
    size_t i;

    if (r == NULL) {
        return NTC_ERR_INVAL;
    }
    if (end_ms < start_ms) {
        r->stats.rejected++;
        return NTC_REC_ERR_TIME_ORDER;
    }
    if (r->capacity_bytes > 0 && bytes > r->capacity_bytes) {
        r->stats.rejected++;
        return NTC_ERR_RANGE; /* would never fit, evicting anything is futile */
    }
    if (bytes == 0) {
        bytes = 1; /* a zero-byte clip still occupies a table slot */
    }
    if (r->count >= NTC_REC_MAX_SEGMENTS && r->capacity_bytes == 0) {
        r->stats.rejected++;
        return NTC_REC_ERR_FULL;
    }

    /* Make room: evict oldest segments until the budget allows the insert. */
    while (r->capacity_bytes > 0 &&
           r->used_bytes + bytes > r->capacity_bytes) {
        if (r->count == 0) {
            r->stats.rejected++;
            return NTC_ERR_RANGE;
        }
        (void)ntc_recorder_evict_oldest(r);
    }
    if (r->count >= NTC_REC_MAX_SEGMENTS) {
        r->stats.rejected++;
        return NTC_REC_ERR_FULL;
    }

    r->stats.inserts++;

    /* Case A: merge into an existing segment (starts inside its tail). */
    for (i = 0; i < r->count; i++) {
        if (start_ms >= r->segs[i].start_ms &&
            start_ms <= r->segs[i].end_ms + r->merge_gap_ms) {
            r->segs[i].end_ms = (end_ms > r->segs[i].end_ms) ? end_ms
                                                             : r->segs[i].end_ms;
            r->segs[i].bytes += bytes;
            r->segs[i].reason |= reason;
            r->used_bytes += bytes;
            r->stats.merges++;
            /* Absorb following segments that the extension now overlaps. */
            while (i + 1 < r->count &&
                   r->segs[i + 1].start_ms <= r->segs[i].end_ms +
                                               r->merge_gap_ms) {
                r->segs[i].end_ms = (r->segs[i + 1].end_ms >
                                     r->segs[i].end_ms)
                                        ? r->segs[i + 1].end_ms
                                        : r->segs[i].end_ms;
                r->segs[i].bytes += r->segs[i + 1].bytes;
                r->segs[i].reason |= r->segs[i + 1].reason;
                r->stats.merges++;
                {
                    size_t k;
                    for (k = i + 1; k + 1 < r->count; k++) {
                        r->segs[k] = r->segs[k + 1];
                    }
                    r->count--;
                    memset(&r->segs[r->count], 0, sizeof(r->segs[r->count]));
                }
            }
            if (id_out != NULL) { *id_out = r->segs[i].id; }
            r->oldest_id = (r->count > 0) ? r->segs[0].id : 0;
            return NTC_OK;
        }
    }

    /* Case B: new segment.  Reject a real overlap rather than corrupting
     * the sorted invariant. */
    for (i = 0; i < r->count; i++) {
        double s = r->segs[i].start_ms;
        double e = r->segs[i].end_ms;
        int overlaps = (end_ms > s) && (start_ms < e);
        if (!overlaps) {
            continue;
        }
        if (start_ms <= e + r->merge_gap_ms) {
            /* handled by case A above; reaching here means the gap rule and
             * the overlap rule disagree, e.g. zero-width windows */
            r->segs[i].end_ms = (end_ms > e) ? end_ms : e;
            r->segs[i].bytes += bytes;
            r->segs[i].reason |= reason;
            r->used_bytes += bytes;
            r->stats.merges++;
            if (id_out != NULL) { *id_out = r->segs[i].id; }
            r->oldest_id = (r->count > 0) ? r->segs[0].id : 0;
            return NTC_OK;
        }
        r->stats.rejected++;
        return NTC_REC_ERR_OVERLAP;
    }

    {
        size_t pos = r->count;
        for (i = 0; i < r->count; i++) {
            if (start_ms < r->segs[i].start_ms) {
                pos = i;
                break;
            }
        }
        for (i = r->count; i > pos; i--) {
            r->segs[i] = r->segs[i - 1];
        }
        memset(&r->segs[pos], 0, sizeof(r->segs[pos]));
        r->segs[pos].id = r->next_id++;
        r->segs[pos].start_ms = start_ms;
        r->segs[pos].end_ms = end_ms;
        r->segs[pos].bytes = bytes;
        r->segs[pos].reason = reason;
        r->segs[pos].in_use = 1;
        r->count++;
        r->used_bytes += bytes;
        if (id_out != NULL) { *id_out = r->segs[pos].id; }
        r->oldest_id = r->segs[0].id;
    }
    return NTC_OK;
}

ntc_status_t ntc_recorder_query(const ntc_recorder_t *r, double start_ms,
                                double end_ms, ntc_rec_segment_t *out,
                                size_t out_cap, size_t *hits,
                                size_t *hits_total)
{
    size_t i;
    size_t n = 0;

    if (r == NULL) {
        return NTC_ERR_INVAL;
    }
    if (hits != NULL) { *hits = 0; }
    if (hits_total != NULL) { *hits_total = 0; }
    if (end_ms < start_ms) {
        return NTC_REC_ERR_TIME_ORDER;
    }
    if (out == NULL && out_cap > 0) {
        return NTC_ERR_INVAL;
    }

    for (i = 0; i < r->count; i++) {
        if (r->segs[i].end_ms > start_ms && r->segs[i].start_ms < end_ms) {
            if (n < out_cap && out != NULL) {
                out[n] = r->segs[i];
            }
            n++;
        }
    }
    if (hits != NULL) { *hits = (n < out_cap) ? n : out_cap; }
    if (hits_total != NULL) { *hits_total = n; }
    return NTC_OK;
}

ntc_status_t ntc_recorder_total_ms(const ntc_recorder_t *r, double *out)
{
    size_t i;
    double total = 0.0;
    if (r == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    for (i = 0; i < r->count; i++) {
        total += r->segs[i].end_ms - r->segs[i].start_ms;
    }
    *out = total;
    return NTC_OK;
}

ntc_status_t ntc_recorder_find(const ntc_recorder_t *r, uint64_t id,
                               ntc_rec_segment_t *out)
{
    size_t i;
    if (r == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    for (i = 0; i < r->count; i++) {
        if (r->segs[i].id == id) {
            *out = r->segs[i];
            return NTC_OK;
        }
    }
    return NTC_REC_ERR_NOTFOUND;
}

ntc_status_t ntc_recorder_stats(const ntc_recorder_t *r, ntc_rec_stats_t *out)
{
    if (r == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    *out = r->stats;
    return NTC_OK;
}
