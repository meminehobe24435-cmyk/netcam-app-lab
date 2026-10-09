/*
 * util.c -- shared helpers: status strings, directories, report files,
 *           deterministic PRNG, simulated network impairment.
 *
 * No stdio for string building (we never rely on the C runtime's text-mode
 * translation, and snprintf formats can vary between libcs); instead the
 * small fixed-width helpers below append into explicit-length buffers.
 */

#include "netcam.h"

#include <stdlib.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#if defined(_WIN32)
#  include <direct.h>   /* _mkdir */
#  ifndef S_IFDIR
#    define S_IFDIR _S_IFDIR
#  endif
#endif

/* ------------------------------------------------------------------ */
/* status strings                                                      */
/* ------------------------------------------------------------------ */

const char *ntc_status_str(ntc_status_t st)
{
    switch (st) {
    case NTC_OK:                       return "OK";
    case NTC_ERR_INVAL:                return "invalid argument";
    case NTC_ERR_NOMEM:                return "out of memory";
    case NTC_ERR_PARSE:                return "parse error";
    case NTC_ERR_RANGE:                return "out of range";
    case NTC_ERR_FULL:                 return "container full";
    case NTC_ERR_EMPTY:                return "container empty";
    case NTC_ERR_NOTFOUND:             return "not found";
    case NTC_ERR_UNSUPPORTED:          return "unsupported";
    case NTC_RTSP_ERR_METHOD:          return "rtsp: unknown method";
    case NTC_RTSP_ERR_VERSION:         return "rtsp: bad version";
    case NTC_RTSP_ERR_CSEQ_MISSING:    return "rtsp: CSeq missing";
    case NTC_RTSP_ERR_CSEQ_BAD:        return "rtsp: CSeq not numeric";
    case NTC_RTSP_ERR_CSEQ_ORDER:      return "rtsp: CSeq out of order";
    case NTC_RTSP_ERR_SESSION_MISSING: return "rtsp: Session missing";
    case NTC_RTSP_ERR_SESSION_INVALID: return "rtsp: Session invalid";
    case NTC_RTSP_ERR_STATE:           return "rtsp: illegal in state";
    case NTC_RTSP_ERR_TRANSPORT:       return "rtsp: bad Transport";
    case NTC_RTSP_ERR_SESSION_TABLE:   return "rtsp: session table full";
    case NTC_RTP_ERR_SHORT:            return "rtp: buffer too short";
    case NTC_RTP_ERR_VERSION:          return "rtp: version != 2";
    case NTC_RTP_ERR_CSRC:             return "rtp: bad CSRC count";
    case NTC_RTP_ERR_PAYLOAD_TYPE:     return "rtp: payload type mismatch";
    case NTC_RTP_ERR_NO_ROOM:          return "rtp: no room";
    case NTC_RTP_ERR_SEQ_MISMATCH:     return "rtp: sequence mismatch";
    case NTC_NAL_ERR_BAD_FU:           return "nal: malformed FU header";
    case NTC_NAL_ERR_SEQUENCE:         return "nal: fragment out of order";
    case NTC_NAL_ERR_GAP:              return "nal: fragment gap";
    case NTC_NAL_ERR_NO_ROOM:          return "nal: reassembly overflow";
    case NTC_NAL_ERR_NOT_STARTED:      return "nal: no unit in progress";
    case NTC_NAL_ERR_BAD_TYPE:         return "nal: forbidden type";
    case NTC_MOT_ERR_FRAME_SIZE:       return "motion: frame size mismatch";
    case NTC_MOT_ERR_CONFIG:           return "motion: bad config";
    case NTC_REC_ERR_FULL:             return "recorder: table full";
    case NTC_REC_ERR_OVERLAP:          return "recorder: overlap";
    case NTC_REC_ERR_TIME_ORDER:       return "recorder: bad time order";
    case NTC_REC_ERR_NOTFOUND:         return "recorder: id not found";
    default:                           return "unknown status";
    }
}

/* ------------------------------------------------------------------ */
/* directories + report files                                          */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_mkdir(const char *dir)
{
    if (dir == NULL) {
        return NTC_ERR_INVAL;
    }
#if defined(_WIN32)
    if (_mkdir(dir) == 0) {
        return NTC_OK;
    }
#else
    if (mkdir(dir, 0777) == 0) {
        return NTC_OK;
    }
#endif
    {
        struct stat st;
        if (stat(dir, &st) == 0 && (st.st_mode & S_IFDIR) != 0) {
            return NTC_OK; /* already there: not an error */
        }
    }
    return NTC_ERR_PARSE;
}

static void ntc_join(char *dst, size_t cap, const char *dir, const char *name)
{
    size_t n = 0;
    size_t i;

    if (cap == 0) {
        return;
    }
    for (i = 0; dir[i] != '\0' && n + 1 < cap; i++) {
        dst[n++] = dir[i];
    }
    if (n + 1 < cap && (n == 0 || dst[n - 1] != '/')) {
        dst[n++] = '/';
    }
    for (i = 0; name[i] != '\0' && n + 1 < cap; i++) {
        dst[n++] = name[i];
    }
    dst[n] = '\0';
}

static FILE *ntc_report_open_common(const char *dir, const char *name,
                                    const char *header)
{
    char path[512];
    FILE *f;

    if (dir == NULL || name == NULL) {
        return NULL;
    }
    (void)ntc_mkdir(dir);
    ntc_join(path, sizeof(path), dir, name);

    /*
     * "wb" deliberately: on Windows a text-mode stream would translate
     * "\n" to "\r\n" and, far worse, treat byte 0x1A as end of file.
     * Everything this project writes is UTF-8/ASCII with explicit "\n".
     */
    f = fopen(path, "wb");
    if (f == NULL) {
        return NULL;
    }
    if (header != NULL) {
        size_t n = strlen(header);
        if (n > 0 && fwrite(header, 1, n, f) != n) {
            fclose(f);
            return NULL;
        }
        if (fputc('\n', f) == EOF) {
            fclose(f);
            return NULL;
        }
    }
    return f;
}

FILE *ntc_report_open_text(const char *dir, const char *name)
{
    return ntc_report_open_common(dir, name, NULL);
}

FILE *ntc_report_open_csv(const char *dir, const char *name,
                          const char *header)
{
    return ntc_report_open_common(dir, name, header);
}

/* ------------------------------------------------------------------ */
/* deterministic PRNG (xorshift64*)                                    */
/* ------------------------------------------------------------------ */

void ntc_rng_seed(ntc_rng_t *rng, uint64_t seed)
{
    if (rng == NULL) {
        return;
    }
    /* 0 is a fixed point of xorshift: never let the state be zero. */
    rng->state = (seed == 0u) ? 0x9E3779B97F4A7C15ull : seed;
}

double ntc_rng_double(ntc_rng_t *rng)
{
    uint64_t x;
    if (rng == NULL) {
        return 0.0;
    }
    x = rng->state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    rng->state = x;
    x = x * 0x2545F4914F6CDD1Dull;
    /* top 53 bits -> [0,1) without ever reaching 1.0 */
    return (double)(x >> 11) / 9007199254740992.0;
}

uint32_t ntc_rng_below(ntc_rng_t *rng, uint32_t n)
{
    double d;
    if (n == 0u) {
        return 0u;
    }
    d = ntc_rng_double(rng);
    return (uint32_t)(d * (double)n) % n;
}

/* ------------------------------------------------------------------ */
/* simulated network                                                   */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_net_profile_default(ntc_net_profile_t *p, uint64_t seed)
{
    if (p == NULL) {
        return NTC_ERR_INVAL;
    }
    p->loss_prob = 0.05;
    p->dup_prob = 0.02;
    p->reorder_prob = 0.20;
    p->burst_gap = 1;
    p->seed = seed;
    return NTC_OK;
}

/*
 * SIMULATED NETWORK IMPAIRMENT
 * ============================
 *
 * The three impairments are applied in a fixed order and each one is
 * expressed as an explicit, separated step.  An earlier revision permuted the
 * caller's array in place with a hand written "shift right" helper and
 * accumulated three aliasing bugs in a row: the move copied the wrong slot,
 * the reorder loop re-inserted the same element repeatedly, and the loss pass
 * nulled a slot before reading it.  The symptom was that the datagram array
 * ended up holding eight references to the same buffer, and the process died
 * with a heap corruption abort when the caller freed it.  The structure below
 * makes that class of bug impossible: every step either keeps a pointer or
 * makes a fresh copy, and no pointer is ever stored twice.
 *
 *   1. LOSS        keep each original buffer with probability 1-loss_prob and
 *                  free the rest.  Survivors keep their relative order, which
 *                  is what the receiver's sequence numbers expect.
 *   2. DUPLICATE   for each survivor, with probability dup_prob insert a fresh
 *                  malloc'ed copy immediately after it.
 *   3. REORDER     walk the resulting arrival sequence; with probability
 *                  reorder_prob, move the element at position k to position
 *                  k+burst_gap by SHIFTING the elements in between one slot
 *                  towards the front.  The element that lands at position k is
 *                  then considered before moving on, so no element is moved
 *                  twice in a row.
 *
 * The result is still a set of distinct buffers, each owned by exactly one
 * array slot, which is what ntc_net_free() relies on.
 */

/* Move the element at `from` to index `to` (> from), shifting the elements in
 * between one slot towards the front.  The slot that held it is overwritten by
 * its neighbour, so no pointer is ever duplicated. */
static void net_move_later(uint8_t **d, size_t *l, size_t from, size_t to)
{
    size_t k;
    uint8_t *buf = d[from];
    size_t len = l[from];

    for (k = from; k < to; k++) {
        d[k] = d[k + 1];
        l[k] = l[k + 1];
    }
    d[to] = buf;
    l[to] = len;
}

void ntc_net_free(uint8_t **dgrams, size_t count)
{
    size_t i;
    if (dgrams == NULL) {
        return;
    }
    for (i = 0; i < count; i++) {
        free(dgrams[i]);
        dgrams[i] = NULL;
    }
}

ntc_status_t ntc_net_apply(ntc_net_profile_t *p, uint8_t **dgrams,
                           size_t *lens, size_t cap, size_t *count_inout,
                           size_t *out_lost, size_t *out_duplicated)
{
    ntc_rng_t rng;
    size_t i;
    size_t lost = 0;
    size_t dup = 0;
    size_t n;
    size_t kept = 0;

    if (p == NULL || dgrams == NULL || lens == NULL || count_inout == NULL) {
        return NTC_ERR_INVAL;
    }
    if (p->loss_prob < 0.0 || p->loss_prob > 1.0 ||
        p->dup_prob < 0.0 || p->dup_prob > 1.0 ||
        p->reorder_prob < 0.0 || p->reorder_prob > 1.0) {
        return NTC_ERR_RANGE;
    }
    if (p->burst_gap < 1u) {
        return NTC_ERR_RANGE;
    }

    ntc_rng_seed(&rng, p->seed);
    n = *count_inout;

    /* ---- step 1: loss -------------------------------------------------- */
    for (i = 0; i < n; i++) {
        if (ntc_rng_double(&rng) < p->loss_prob) {
            free(dgrams[i]);
            dgrams[i] = NULL;
            lens[i] = 0;
            lost++;
            continue;
        }
        if (kept != i) {
            /* Move the survivor to the front.  Read before writing: nulling
             * the source first is what lost the pointer in the old code. */
            uint8_t *keep = dgrams[i];
            size_t keep_len = lens[i];
            dgrams[kept] = keep;
            lens[kept] = keep_len;
            dgrams[i] = NULL;
            lens[i] = 0;
        }
        kept++;
    }
    n = kept;

    /* ---- step 2: duplication ------------------------------------------- */
    for (i = 0; i < n; ) {
        if (ntc_rng_double(&rng) < p->dup_prob) {
            uint8_t *copy;
            size_t k;
            if (n + 1u > cap) {
                *count_inout = n;
                if (out_lost != NULL) { *out_lost = lost; }
                if (out_duplicated != NULL) { *out_duplicated = dup; }
                return NTC_ERR_FULL;
            }
            copy = (uint8_t *)malloc(lens[i]);
            if (copy == NULL) {
                *count_inout = n;
                if (out_lost != NULL) { *out_lost = lost; }
                if (out_duplicated != NULL) { *out_duplicated = dup; }
                return NTC_ERR_NOMEM;
            }
            memcpy(copy, dgrams[i], lens[i]);
            /* Insert the copy right after the original. */
            for (k = n; k > i + 1u; k--) {
                dgrams[k] = dgrams[k - 1u];
                lens[k] = lens[k - 1u];
            }
            dgrams[i + 1u] = copy;
            lens[i + 1u] = lens[i];
            n++;
            dup++;
            i += 2u;
        } else {
            i++;
        }
    }

    /* ---- step 3: reordering -------------------------------------------- */
    for (i = 0; i < n; ) {
        if (n - i > (size_t)p->burst_gap &&
            ntc_rng_double(&rng) < p->reorder_prob) {
            size_t target = i + (size_t)p->burst_gap;
            if (target >= n) {
                target = n - 1u;
            }
            if (target > i) {
                net_move_later(dgrams, lens, i, target);
                i++; /* the element now at i has not been considered yet */
                continue;
            }
        }
        i++;
    }

    *count_inout = n;
    if (out_lost != NULL) { *out_lost = lost; }
    if (out_duplicated != NULL) { *out_duplicated = dup; }
    return NTC_OK;
}
