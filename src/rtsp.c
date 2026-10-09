/*
 * rtsp.c -- RTSP session layer (RFC 2326 subset)
 * ==============================================
 *
 * WIRE FORMAT
 *   Request-Line  = Method SP Request-URI SP "RTSP/1.0" CRLF
 *   Status-Line   = "RTSP/1.0" SP Status-Code SP Reason-Phrase CRLF
 *   message       = *( header CRLF ) CRLF
 *   header        = field-name ":" OWS field-value OWS CRLF
 *   We accept CRLF and bare LF line endings (some clients are sloppy) and
 *   we are case-insensitive on header field names, as the spec requires.
 *
 * WHY CSeq MATTERS
 *   CSeq is the request/response matching token of RTSP and a replay
 *   defence.  Inside one session every request must carry a CSeq strictly
 *   greater than the last one accepted; a repeat or a lower value means a
 *   duplicated/reordered request on the wire or a confused client.  We
 *   reject it with 400 before touching any state, which is what makes the
 *   "CSeq reordering is refused" number in the README measurable.
 *
 * STATE MACHINE (per session)
 *
 *   INIT --SETUP--> READY --PLAY--> PLAYING --PAUSE--> READY
 *     |               |                |
 *     +---------------+----------------+--TEARDOWN--> TORN_DOWN (slot freed)
 *
 *   Allowed table (method x state):
 *     OPTIONS   : INIT READY PLAYING      DESCRIBE : INIT READY PLAYING
 *     SETUP     : INIT only               PLAY     : READY only
 *     PAUSE     : PLAYING only            TEARDOWN : READY PLAYING
 *   Anything else is answered 455 Method Not Valid In This State, an
 *   unknown session is answered 454 Session Not Found.
 */

#include "netcam.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* small text helpers (no snprintf: deterministic across libcs)        */
/* ------------------------------------------------------------------ */

static int ntc_is_digit7(int c) { return c >= '0' && c <= '9'; }

static int ntc_ci_eq(const char *a, const char *b)
{
    size_t i = 0;
    for (;;) {
        char ca = a[i];
        char cb = b[i];
        if (ca >= 'A' && ca <= 'Z') { ca = (char)(ca - 'A' + 'a'); }
        if (cb >= 'A' && cb <= 'Z') { cb = (char)(cb - 'A' + 'a'); }
        if (ca != cb) {
            return 0;
        }
        if (ca == '\0') {
            return 1;
        }
        i++;
    }
}

static void ntc_copy_bounded(char *dst, size_t cap, const char *src,
                             size_t len)
{
    size_t i;
    if (cap == 0) {
        return;
    }
    for (i = 0; i + 1 < cap && i < len; i++) {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

/* Compare a NUL terminated string with a [ptr,len) slice, case-insens. */
static int ntc_ci_eq_n(const char *text, const char *slice, size_t len)
{
    size_t i;
    for (i = 0; i < len; i++) {
        char ca = text[i];
        char cb = slice[i];
        if (ca >= 'A' && ca <= 'Z') { ca = (char)(ca - 'A' + 'a'); }
        if (cb >= 'A' && cb <= 'Z') { cb = (char)(cb - 'A' + 'a'); }
        if (ca != cb) {
            return 0;
        }
    }
    return text[len] == '\0';
}

/* Append helper for building response text. */
static void ntc_append(char *dst, size_t cap, size_t *used, const char *src)
{
    size_t i = 0;
    if (dst == NULL || used == NULL) {
        return;
    }
    while (src[i] != '\0' && *used + 1 < cap) {
        dst[*used] = src[i];
        (*used)++;
        i++;
    }
    if (cap > 0) {
        dst[*used] = '\0';
    }
}

static void ntc_append_dec(char *dst, size_t cap, size_t *used, uint64_t v)
{
    char tmp[24];
    size_t n = 0;
    if (v == 0u) {
        tmp[n++] = '0';
    }
    while (v > 0u && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (int)(v % 10u));
        v /= 10u;
    }
    while (n > 0) {
        char c = tmp[--n];
        if (*used + 1 >= cap) {
            return;
        }
        dst[*used] = c;
        (*used)++;
        if (cap > 0) {
            dst[*used] = '\0';
        }
    }
}

/* Parse an unsigned decimal in [ptr, ptr+len); returns 1 on success. */
static int ntc_parse_u32(const char *ptr, size_t len, uint32_t *out)
{
    uint64_t v = 0;
    size_t i;
    if (len == 0) {
        return 0;
    }
    for (i = 0; i < len; i++) {
        if (!ntc_is_digit7((unsigned char)ptr[i])) {
            return 0;
        }
        v = v * 10u + (uint64_t)(ptr[i] - '0');
        if (v > 0xFFFFFFFFull) {
            return 0;
        }
    }
    *out = (uint32_t)v;
    return 1;
}

/* ------------------------------------------------------------------ */
/* method / state names                                                */
/* ------------------------------------------------------------------ */

/*
 * Method tokens are case SENSITIVE in RTSP: "PLAY" is the method, "play" is
 * not.  Header field names, by contrast, are case insensitive.  An earlier
 * revision of this file compared method tokens with the case-insensitive
 * helper and happily accepted "play"; the test vector caught it.
 */
ntc_rtsp_method_t ntc_rtsp_method_from_text(const char *text)
{
    if (text == NULL) {
        return NTC_RTSP_UNKNOWN_METHOD;
    }
    if (strcmp(text, "OPTIONS") == 0)  { return NTC_RTSP_OPTIONS; }
    if (strcmp(text, "DESCRIBE") == 0) { return NTC_RTSP_DESCRIBE; }
    if (strcmp(text, "SETUP") == 0)    { return NTC_RTSP_SETUP; }
    if (strcmp(text, "PLAY") == 0)     { return NTC_RTSP_PLAY; }
    if (strcmp(text, "PAUSE") == 0)    { return NTC_RTSP_PAUSE; }
    if (strcmp(text, "TEARDOWN") == 0) { return NTC_RTSP_TEARDOWN; }
    return NTC_RTSP_UNKNOWN_METHOD;
}

const char *ntc_rtsp_method_str(ntc_rtsp_method_t m)
{
    switch (m) {
    case NTC_RTSP_OPTIONS:  return "OPTIONS";
    case NTC_RTSP_DESCRIBE: return "DESCRIBE";
    case NTC_RTSP_SETUP:    return "SETUP";
    case NTC_RTSP_PLAY:     return "PLAY";
    case NTC_RTSP_PAUSE:    return "PAUSE";
    case NTC_RTSP_TEARDOWN: return "TEARDOWN";
    default:                return "UNKNOWN";
    }
}

const char *ntc_rtsp_state_str(ntc_rtsp_state_t s)
{
    switch (s) {
    case NTC_RTSP_STATE_INIT:      return "INIT";
    case NTC_RTSP_STATE_READY:     return "READY";
    case NTC_RTSP_STATE_PLAYING:   return "PLAYING";
    case NTC_RTSP_STATE_TORN_DOWN: return "TORN_DOWN";
    default:                       return "?";
    }
}

/* ------------------------------------------------------------------ */
/* server / session management                                         */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_rtsp_server_init(ntc_rtsp_server_t *srv, uint32_t base_id,
                                  uint32_t next_cseq, const char *server_name)
{
    if (srv == NULL) {
        return NTC_ERR_INVAL;
    }
    memset(srv, 0, sizeof(*srv));
    srv->base_id = (base_id == 0u) ? 1u : base_id;
    srv->next_id = srv->base_id;
    srv->next_cseq = (next_cseq == 0u) ? 1u : next_cseq;
    if (server_name != NULL) {
        ntc_copy_bounded(srv->server_name, sizeof(srv->server_name),
                         server_name, strlen(server_name));
    } else {
        ntc_copy_bounded(srv->server_name, sizeof(srv->server_name),
                         "netcam-app-lab", 14);
    }
    return NTC_OK;
}

ntc_status_t ntc_rtsp_find_session(const ntc_rtsp_server_t *srv, uint32_t id,
                                   const ntc_rtsp_session_t **out)
{
    size_t i;
    if (srv == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    for (i = 0; i < NTC_RTSP_MAX_SESSIONS; i++) {
        if (srv->sessions[i].in_use && srv->sessions[i].id == id) {
            *out = &srv->sessions[i];
            return NTC_OK;
        }
    }
    return NTC_ERR_NOTFOUND;
}

ntc_status_t ntc_rtsp_session_count(const ntc_rtsp_server_t *srv, size_t *out)
{
    size_t i;
    size_t n = 0;
    if (srv == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    for (i = 0; i < NTC_RTSP_MAX_SESSIONS; i++) {
        if (srv->sessions[i].in_use) {
            n++;
        }
    }
    *out = n;
    return NTC_OK;
}

ntc_status_t ntc_rtsp_close_session(ntc_rtsp_server_t *srv, uint32_t id)
{
    size_t i;
    if (srv == NULL) {
        return NTC_ERR_INVAL;
    }
    for (i = 0; i < NTC_RTSP_MAX_SESSIONS; i++) {
        if (srv->sessions[i].in_use && srv->sessions[i].id == id) {
            srv->sessions[i].state = NTC_RTSP_STATE_TORN_DOWN;
            srv->sessions[i].in_use = 0;
            srv->stats.sessions_torn_down++;
            return NTC_OK;
        }
    }
    return NTC_ERR_NOTFOUND;
}

/* ------------------------------------------------------------------ */
/* request parsing                                                     */
/* ------------------------------------------------------------------ */

ntc_status_t ntc_rtsp_parse_request(const char *text, size_t len,
                                    ntc_rtsp_request_t *out)
{
    size_t pos = 0;
    size_t line_start;
    int first_line = 1;
    const char *eol;

    if (text == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    memset(out, 0, sizeof(*out));
    out->method = NTC_RTSP_UNKNOWN_METHOD;

    while (pos < len) {
        line_start = pos;
        eol = (const char *)memchr(text + pos, '\n', len - pos);
        if (eol == NULL) {
            /* A request line or header without a line terminator: the
             * message is truncated, so we cannot trust anything in it. */
            return NTC_ERR_PARSE;
        }
        pos = (size_t)(eol - text) + 1;

        /* strip trailing CR */
        {
            size_t llen = (size_t)(eol - text) - line_start;
            if (llen > 0 && text[line_start + llen - 1] == '\r') {
                llen--;
            }
            if (first_line) {
                /* Request-Line = Method SP URI SP Version */
                size_t s1 = 0;
                size_t s2;
                while (s1 < llen && text[line_start + s1] != ' ') {
                    s1++;
                }
                if (s1 == 0 || s1 >= llen || s1 + 1 > NTC_RTSP_MAX_METHOD) {
                    return NTC_ERR_PARSE;
                }
                ntc_copy_bounded(out->method_text, sizeof(out->method_text),
                                 text + line_start, s1);
                s2 = s1 + 1;
                while (s2 < llen && text[line_start + s2] != ' ') {
                    s2++;
                }
                if (s2 >= llen) {
                    return NTC_ERR_PARSE; /* URI without version */
                }
                ntc_copy_bounded(out->uri, sizeof(out->uri),
                                 text + line_start + s1 + 1, s2 - s1 - 1);
                if (s2 + 1 >= llen ||
                    s2 + 1 - (s2 + 1) > NTC_RTSP_MAX_VERSION) {
                    return NTC_ERR_PARSE;
                }
                ntc_copy_bounded(out->version, sizeof(out->version),
                                 text + line_start + s2 + 1,
                                 llen - s2 - 1);
                if (!ntc_ci_eq(out->version, "RTSP/1.0")) {
                    return NTC_RTSP_ERR_VERSION;
                }
                out->method = ntc_rtsp_method_from_text(out->method_text);
                first_line = 0;
                out->header_bytes = pos;
                continue;
            }

            if (llen == 0) {
                /* End of headers: the blank line that terminates them. */
                out->header_bytes = pos;
                return NTC_OK;
            }

            /* header = name ":" OWS value OWS */
            {
                size_t i;
                size_t colon = llen;
                for (i = 0; i < llen; i++) {
                    if (text[line_start + i] == ':') {
                        colon = i;
                        break;
                    }
                }
                if (colon == 0 || colon >= llen || colon >= NTC_RTSP_MAX_NAME) {
                    return NTC_ERR_PARSE;
                }
                {
                    char name[NTC_RTSP_MAX_NAME];
                    size_t vstart = colon + 1;
                    size_t vend = llen;
                    ntc_copy_bounded(name, sizeof(name), text + line_start,
                                     colon);
                    while (vstart < vend &&
                           (text[line_start + vstart] == ' ' ||
                            text[line_start + vstart] == '\t')) {
                        vstart++;
                    }
                    while (vend > vstart &&
                           (text[line_start + vend - 1] == ' ' ||
                            text[line_start + vend - 1] == '\t')) {
                        vend--;
                    }
                    if (out->nheaders < NTC_RTSP_MAX_HEADERS) {
                        ntc_rtsp_header_t *h = &out->headers[out->nheaders];
                        ntc_copy_bounded(h->name, sizeof(h->name), name,
                                         strlen(name));
                        ntc_copy_bounded(h->value, sizeof(h->value),
                                         text + line_start + vstart,
                                         vend - vstart);
                        out->nheaders++;
                    } else {
                        out->header_overflow = 1;
                    }

                    if (ntc_ci_eq(name, "CSeq")) {
                        uint32_t v;
                        if (!ntc_parse_u32(text + line_start + vstart,
                                           vend - vstart, &v)) {
                            return NTC_RTSP_ERR_CSEQ_BAD;
                        }
                        out->cseq = v;
                        out->cseq_present = 1;
                    } else if (ntc_ci_eq(name, "Session")) {
                        /* "Session: 12345678;timeout=60" -> take the number */
                        size_t vlen = vend - vstart;
                        size_t semi = vlen;
                        size_t k;
                        uint32_t v;
                        for (k = 0; k < vlen; k++) {
                            if (text[line_start + vstart + k] == ';') {
                                semi = k;
                                break;
                            }
                        }
                        if (semi > 0 &&
                            ntc_parse_u32(text + line_start + vstart, semi,
                                          &v)) {
                            out->session = v;
                            out->session_present = 1;
                        } else {
                            return NTC_RTSP_ERR_SESSION_INVALID;
                        }
                    } else if (ntc_ci_eq(name, "Transport")) {
                        ntc_copy_bounded(out->transport, sizeof(out->transport),
                                         text + line_start + vstart,
                                         vend - vstart);
                    } else if (ntc_ci_eq(name, "User-Agent")) {
                        ntc_copy_bounded(out->user_agent,
                                         sizeof(out->user_agent),
                                         text + line_start + vstart,
                                         vend - vstart);
                    } else if (ntc_ci_eq(name, "Content-Type")) {
                        ntc_copy_bounded(out->content_type,
                                         sizeof(out->content_type),
                                         text + line_start + vstart,
                                         vend - vstart);
                    } else if (ntc_ci_eq(name, "Content-Length")) {
                        uint32_t v;
                        if (ntc_parse_u32(text + line_start + vstart,
                                          vend - vstart, &v)) {
                            out->content_length = (size_t)v;
                        }
                    }
                }
            }
        }
    }
    /* Ran off the end without the blank line that terminates the headers. */
    return NTC_ERR_PARSE;
}
ntc_status_t ntc_rtsp_parse_transport(const char *value,
                                      ntc_rtsp_transport_t *out)
{
    const char *p;
    if (value == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    memset(out, 0, sizeof(*out));
    out->interleaved_rtp = -1;
    out->interleaved_rtcp = -1;

    for (p = value; *p != '\0'; p++) {
        if (ntc_ci_eq_n("rtp/avp/tcp", p, 11)) {
            out->mode = 1;
            ntc_copy_bounded(out->protocol, sizeof(out->protocol),
                             "RTP/AVP/TCP", 11);
            break;
        }
        if (ntc_ci_eq_n("rtp/avp", p, 7)) {
            out->mode = 0;
            ntc_copy_bounded(out->protocol, sizeof(out->protocol),
                             "RTP/AVP", 7);
            break;
        }
    }
    if (out->protocol[0] == '\0') {
        return NTC_ERR_PARSE;
    }

    p = strstr(value, "client_port=");
    if (p == NULL) {
        p = strstr(value, "client_port =");
    }
    if (p != NULL) {
        const char *q = strchr(p, '=');
        if (q != NULL) {
            uint32_t a = 0;
            uint32_t b = 0;
            const char *sp = q + 1;
            const char *dash = strchr(sp, '-');
            size_t n = 0;
            while (ntc_is_digit7((unsigned char)sp[n])) {
                n++;
            }
            if (n > 0 && ntc_parse_u32(sp, n, &a)) {
                out->has_client_port = 1;
                out->client_rtp_port = (uint16_t)a;
                if (dash != NULL && *dash == '-') {
                    size_t m = 0;
                    while (ntc_is_digit7((unsigned char)dash[1 + m])) {
                        m++;
                    }
                    if (m > 0 && ntc_parse_u32(dash + 1, m, &b)) {
                        out->client_rtcp_port = (uint16_t)b;
                    }
                }
            } else {
                return NTC_ERR_PARSE;
            }
        }
    }

    p = strstr(value, "interleaved=");
    if (p != NULL) {
        const char *sp = p + 12;
        const char *dash = strchr(sp, '-');
        size_t n = 0;
        uint32_t a = 0;
        while (ntc_is_digit7((unsigned char)sp[n])) {
            n++;
        }
        if (n > 0 && ntc_parse_u32(sp, n, &a) && a <= 255u) {
            out->has_interleaved = 1;
            out->interleaved_rtp = (int)a;
            if (dash != NULL && *dash == '-') {
                size_t m = 0;
                uint32_t b = 0;
                while (ntc_is_digit7((unsigned char)dash[1 + m])) {
                    m++;
                }
                if (m > 0 && ntc_parse_u32(dash + 1, m, &b) && b <= 255u) {
                    out->interleaved_rtcp = (int)b;
                }
            }
        }
    }

    if (!out->has_client_port && !out->has_interleaved) {
        return NTC_ERR_PARSE; /* neither UDP nor interleaved transport */
    }
    return NTC_OK;
}

int ntc_rtsp_method_allowed(ntc_rtsp_method_t m, ntc_rtsp_state_t s)
{
    switch (m) {
    case NTC_RTSP_OPTIONS:
    case NTC_RTSP_DESCRIBE:
        return (s == NTC_RTSP_STATE_INIT || s == NTC_RTSP_STATE_READY ||
                s == NTC_RTSP_STATE_PLAYING) ? 1 : 0;
    case NTC_RTSP_SETUP:
        return (s == NTC_RTSP_STATE_INIT) ? 1 : 0;
    case NTC_RTSP_PLAY:
        return (s == NTC_RTSP_STATE_READY) ? 1 : 0;
    case NTC_RTSP_PAUSE:
        return (s == NTC_RTSP_STATE_PLAYING) ? 1 : 0;
    case NTC_RTSP_TEARDOWN:
        return (s == NTC_RTSP_STATE_READY || s == NTC_RTSP_STATE_PLAYING) ? 1 : 0;
    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* request handling: validation order + response construction          */
/* ------------------------------------------------------------------ */

static void ntc_resp_set(ntc_rtsp_response_t *resp, int status,
                         const char *reason, uint32_t cseq, uint32_t session)
{
    memset(resp, 0, sizeof(*resp));
    resp->status = status;
    ntc_copy_bounded(resp->reason, sizeof(resp->reason), reason,
                     strlen(reason));
    resp->cseq = cseq;
    resp->session_id = session;
}

/* Which methods need a Session header to identify the session? */
static int ntc_rtsp_needs_session(ntc_rtsp_method_t m)
{
    return (m == NTC_RTSP_PLAY || m == NTC_RTSP_PAUSE ||
            m == NTC_RTSP_TEARDOWN) ? 1 : 0;
}

ntc_status_t ntc_rtsp_handle(ntc_rtsp_server_t *srv,
                             const ntc_rtsp_request_t *req,
                             double now_ms,
                             ntc_rtsp_response_t *resp)
{
    ntc_rtsp_session_t *sess = NULL;
    size_t i;

    if (srv == NULL || req == NULL || resp == NULL) {
        return NTC_ERR_INVAL;
    }
    srv->stats.requests_total++;

    /* 1. method */
    if (req->method == NTC_RTSP_UNKNOWN_METHOD) {
        srv->stats.rejected_method++;
        ntc_resp_set(resp, 405, "Method Not Allowed", req->cseq, 0);
        return NTC_RTSP_ERR_METHOD;
    }

    /* 2/3. CSeq: present, numeric, strictly increasing */
    if (!req->cseq_present) {
        srv->stats.rejected_cseq_missing++;
        ntc_resp_set(resp, 400, "Bad Request", 0, 0);
        return NTC_RTSP_ERR_CSEQ_MISSING;
    }
    if (req->cseq == 0u) {
        /* CSeq is 1-based in practice; 0 is a nonsense value we refuse. */
        srv->stats.rejected_cseq_bad++;
        ntc_resp_set(resp, 400, "Bad Request", req->cseq, 0);
        return NTC_RTSP_ERR_CSEQ_BAD;
    }

    /* 4/5. session identification */
    if (ntc_rtsp_needs_session(req->method)) {
        if (!req->session_present) {
            srv->stats.rejected_session_missing++;
            ntc_resp_set(resp, 454, "Session Not Found", req->cseq, 0);
            return NTC_RTSP_ERR_SESSION_MISSING;
        }
        for (i = 0; i < NTC_RTSP_MAX_SESSIONS; i++) {
            if (srv->sessions[i].in_use &&
                srv->sessions[i].id == req->session) {
                sess = &srv->sessions[i];
                break;
            }
        }
        if (sess == NULL) {
            srv->stats.rejected_session_invalid++;
            ntc_resp_set(resp, 454, "Session Not Found", req->cseq,
                         req->session);
            return NTC_RTSP_ERR_SESSION_INVALID;
        }
    } else if (req->session_present) {
        /* SETUP carries no session; OPTIONS/DESCRIBE may carry one, and if
         * they do it must exist (otherwise the client is confused). */
        for (i = 0; i < NTC_RTSP_MAX_SESSIONS; i++) {
            if (srv->sessions[i].in_use &&
                srv->sessions[i].id == req->session) {
                sess = &srv->sessions[i];
                break;
            }
        }
        if (sess == NULL) {
            srv->stats.rejected_session_invalid++;
            ntc_resp_set(resp, 454, "Session Not Found", req->cseq,
                         req->session);
            return NTC_RTSP_ERR_SESSION_INVALID;
        }
    }

    /* CSeq monotonicity: checked against the session when we have one. */
    if (sess != NULL && req->cseq <= sess->last_cseq) {
        sess->requests_rejected++;
        srv->stats.rejected_cseq_order++;
        ntc_resp_set(resp, 400, "Bad Request", req->cseq, sess->id);
        return NTC_RTSP_ERR_CSEQ_ORDER;
    }

    /* 6. state legality */
    if (sess != NULL && !ntc_rtsp_method_allowed(req->method, sess->state)) {
        sess->requests_rejected++;
        srv->stats.rejected_state++;
        ntc_resp_set(resp, 455, "Method Not Valid In This State", req->cseq,
                     sess->id);
        return NTC_RTSP_ERR_STATE;
    }

    /* Everything from here on is accepted. */
    switch (req->method) {
    case NTC_RTSP_OPTIONS:
        srv->stats.accepted++;
        ntc_resp_set(resp, 200, "OK", req->cseq,
                     sess != NULL ? sess->id : 0u);
        ntc_copy_bounded(resp->extra_header_name,
                         sizeof(resp->extra_header_name), "Public",
                         strlen("Public"));
        ntc_copy_bounded(resp->extra_header_value,
                         sizeof(resp->extra_header_value),
                         "OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN",
                         strlen("OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, "
                                "TEARDOWN"));
        if (sess != NULL) {
            sess->last_cseq = req->cseq;
            sess->requests_seen++;
        }
        return NTC_OK;

    case NTC_RTSP_DESCRIBE:
        srv->stats.accepted++;
        ntc_resp_set(resp, 200, "OK", req->cseq,
                     sess != NULL ? sess->id : 0u);
        {
            /* A stub, deliberately not a full SDP: we advertise the
             * simulated media so the sim has something honest to print. */
            const char *sdp =
                "v=0\r\n"
                "o=- 0 0 IN IP4 127.0.0.1\r\n"
                "s=netcam-app-lab (simulated)\r\n"
                "t=0 0\r\n"
                "m=video 0 RTP/AVP 96\r\n"
                "a=rtpmap:96 H264/90000\r\n"
                "a=control:trackID=0\r\n";
            size_t n = strlen(sdp);
            if (n >= sizeof(resp->body)) {
                n = sizeof(resp->body) - 1;
            }
            memcpy(resp->body, sdp, n);
            resp->body[n] = '\0';
            resp->body_len = n;
        }
        ntc_copy_bounded(resp->extra_header_name,
                         sizeof(resp->extra_header_name), "Content-Type",
                         strlen("Content-Type"));
        ntc_copy_bounded(resp->extra_header_value,
                         sizeof(resp->extra_header_value),
                         "application/sdp", strlen("application/sdp"));
        if (sess != NULL) {
            sess->last_cseq = req->cseq;
            sess->requests_seen++;
        }
        return NTC_OK;

    case NTC_RTSP_SETUP: {
        ntc_rtsp_transport_t tr;
        int slot = -1;
        uint32_t id;

        /* 7. Transport header is mandatory for SETUP. */
        if (req->transport[0] == '\0' ||
            ntc_rtsp_parse_transport(req->transport, &tr) != NTC_OK) {
            srv->stats.rejected_transport++;
            ntc_resp_set(resp, 400, "Bad Request", req->cseq, 0);
            return NTC_RTSP_ERR_TRANSPORT;
        }

        if (sess == NULL) {
            for (i = 0; i < NTC_RTSP_MAX_SESSIONS; i++) {
                if (!srv->sessions[i].in_use) {
                    slot = (int)i;
                    break;
                }
            }
            if (slot < 0) {
                srv->stats.rejected_session_invalid++;
                ntc_resp_set(resp, 503, "Service Unavailable", req->cseq, 0);
                return NTC_RTSP_ERR_SESSION_TABLE;
            }
            sess = &srv->sessions[slot];
            memset(sess, 0, sizeof(*sess));
            sess->in_use = 1;
            sess->id = srv->next_id++;
            sess->state = NTC_RTSP_STATE_INIT;
            srv->stats.sessions_created++;
        }
        sess->state = NTC_RTSP_STATE_READY;
        sess->last_cseq = req->cseq;
        sess->requests_seen++;
        sess->last_activity_ms = now_ms;
        srv->stats.setps++;
        srv->stats.accepted++;
        id = sess->id;

        ntc_resp_set(resp, 200, "OK", req->cseq, id);
        {
            char tmp[NTC_RTSP_MAX_VALUE];
            size_t u = 0;
            tmp[0] = '\0';
            ntc_append(tmp, sizeof(tmp), &u, "RTP/AVP");
            ntc_append(tmp, sizeof(tmp), &u, tr.mode == 1 ? "/TCP" : "");
            ntc_append(tmp, sizeof(tmp), &u, tr.mode == 1 ? ";unicast;interleaved="
                                                         : ";unicast;client_port=");
            ntc_append_dec(tmp, sizeof(tmp), &u,
                           (uint64_t)(tr.mode == 1 ? (uint32_t)tr.interleaved_rtp
                                                   : (uint32_t)tr.client_rtp_port));
            ntc_append(tmp, sizeof(tmp), &u, "-");
            ntc_append_dec(tmp, sizeof(tmp), &u,
                           (uint64_t)(tr.mode == 1
                                          ? (uint32_t)(tr.interleaved_rtcp < 0
                                                           ? tr.interleaved_rtp + 1
                                                           : tr.interleaved_rtcp)
                                          : (uint32_t)tr.client_rtcp_port));
            ntc_append(tmp, sizeof(tmp), &u, ";server_port=20000-20001");
            ntc_copy_bounded(resp->extra_header_name,
                             sizeof(resp->extra_header_name), "Transport",
                             strlen("Transport"));
            ntc_copy_bounded(resp->extra_header_value,
                             sizeof(resp->extra_header_value), tmp,
                             strlen(tmp));
        }
        return NTC_OK;
    }

    case NTC_RTSP_PLAY:
        sess->state = NTC_RTSP_STATE_PLAYING;
        sess->last_cseq = req->cseq;
        sess->requests_seen++;
        sess->last_activity_ms = now_ms;
        srv->stats.plays++;
        srv->stats.accepted++;
        ntc_resp_set(resp, 200, "OK", req->cseq, sess->id);
        ntc_copy_bounded(resp->extra_header_name,
                         sizeof(resp->extra_header_name), "Range",
                         strlen("Range"));
        ntc_copy_bounded(resp->extra_header_value,
                         sizeof(resp->extra_header_value), "npt=0.000-",
                         strlen("npt=0.000-"));
        return NTC_OK;

    case NTC_RTSP_PAUSE:
        sess->state = NTC_RTSP_STATE_READY;
        sess->last_cseq = req->cseq;
        sess->requests_seen++;
        sess->last_activity_ms = now_ms;
        srv->stats.pauses++;
        srv->stats.accepted++;
        ntc_resp_set(resp, 200, "OK", req->cseq, sess->id);
        return NTC_OK;

    case NTC_RTSP_TEARDOWN: {
        uint32_t id = sess->id;
        sess->last_cseq = req->cseq;
        sess->requests_seen++;
        sess->last_activity_ms = now_ms;
        sess->state = NTC_RTSP_STATE_TORN_DOWN;
        sess->in_use = 0;
        srv->stats.sessions_torn_down++;
        srv->stats.accepted++;
        ntc_resp_set(resp, 200, "OK", req->cseq, id);
        return NTC_OK;
    }

    default:
        srv->stats.rejected_method++;
        ntc_resp_set(resp, 405, "Method Not Allowed", req->cseq, 0);
        return NTC_RTSP_ERR_METHOD;
    }
}

ntc_status_t ntc_rtsp_handle_text(ntc_rtsp_server_t *srv, const char *text,
                                  size_t len, double now_ms,
                                  ntc_rtsp_response_t *resp,
                                  ntc_rtsp_request_t *parsed_or_null)
{
    ntc_rtsp_request_t local;
    ntc_rtsp_request_t *req = (parsed_or_null != NULL) ? parsed_or_null : &local;
    ntc_status_t st;

    if (srv == NULL || text == NULL || resp == NULL) {
        return NTC_ERR_INVAL;
    }
    st = ntc_rtsp_parse_request(text, len, req);
    if (st != NTC_OK) {
        /* A malformed request still deserves 400 with whatever CSeq we saw. */
        ntc_resp_set(resp, 400, "Bad Request", req->cseq, req->session);
        srv->stats.requests_total++;
        if (st == NTC_RTSP_ERR_CSEQ_BAD) {
            srv->stats.rejected_cseq_bad++;
        } else {
            srv->stats.rejected_method++;
        }
        return st;
    }
    return ntc_rtsp_handle(srv, req, now_ms, resp);
}

ntc_status_t ntc_rtsp_write_response(const ntc_rtsp_response_t *resp,
                                     char *buf, size_t cap, size_t *written)
{
    size_t used = 0;

    if (resp == NULL || buf == NULL || cap == 0) {
        if (written != NULL) { *written = 0; }
        return NTC_ERR_INVAL;
    }
    buf[0] = '\0';

    ntc_append(buf, cap, &used, "RTSP/1.0 ");
    {
        char sc[8];
        size_t u = 0;
        int v = resp->status;
        sc[0] = '\0';
        if (v < 0) { v = 0; }
        ntc_append_dec(sc, sizeof(sc), &u, (uint64_t)v);
        ntc_append(buf, cap, &used, sc);
    }
    ntc_append(buf, cap, &used, " ");
    {
        char reason[64];
        ntc_copy_bounded(reason, sizeof(reason),
                         resp->reason[0] != '\0' ? resp->reason : "OK",
                         strlen(resp->reason[0] != '\0' ? resp->reason : "OK"));
        ntc_append(buf, cap, &used, reason);
    }
    ntc_append(buf, cap, &used, "\r\n");

    ntc_append(buf, cap, &used, "CSeq: ");
    {
        char cs[16];
        size_t u = 0;
        cs[0] = '\0';
        ntc_append_dec(cs, sizeof(cs), &u, (uint64_t)resp->cseq);
        ntc_append(buf, cap, &used, cs);
    }
    ntc_append(buf, cap, &used, "\r\n");

    if (resp->session_id != 0u) {
        char cs[16];
        size_t u = 0;
        cs[0] = '\0';
        ntc_append_dec(cs, sizeof(cs), &u, (uint64_t)resp->session_id);
        ntc_append(buf, cap, &used, "Session: ");
        ntc_append(buf, cap, &used, cs);
        ntc_append(buf, cap, &used, "\r\n");
    }
    if (resp->extra_header_name[0] != '\0') {
        ntc_append(buf, cap, &used, resp->extra_header_name);
        ntc_append(buf, cap, &used, ": ");
        ntc_append(buf, cap, &used, resp->extra_header_value);
        ntc_append(buf, cap, &used, "\r\n");
    }
    if (resp->body_len > 0) {
        char cl[24];
        size_t u = 0;
        cl[0] = '\0';
        ntc_append_dec(cl, sizeof(cl), &u, (uint64_t)resp->body_len);
        ntc_append(buf, cap, &used, "Content-Length: ");
        ntc_append(buf, cap, &used, cl);
        ntc_append(buf, cap, &used, "\r\n");
    }
    ntc_append(buf, cap, &used, "\r\n");
    if (resp->body_len > 0) {
        size_t k;
        for (k = 0; k < resp->body_len && used + 1 < cap; k++) {
            buf[used++] = resp->body[k];
        }
        buf[used] = '\0';
    }

    if (used + 1 > cap) {
        if (written != NULL) { *written = cap - 1; }
        return NTC_ERR_NOMEM;
    }
    if (written != NULL) { *written = used; }
    return NTC_OK;
}
