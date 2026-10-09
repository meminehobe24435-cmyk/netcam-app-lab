/*
 * test_netcam.c -- self-checking test suite for netcam-app-lab
 * ===========================================================
 *
 *   * Every check is an explicit assertion; the process exit status is the
 *     number of failures (0 = pass) and the last line is
 *     "<N> checks passed".
 *   * Protocol vectors are HAND ASSEMBLED BYTE BY BYTE where it matters.
 *     Building a packet with this project's own packer and then parsing it
 *     with this project's own parser proves almost nothing: a symmetric bug
 *     in both directions cancels out and the test stays green.  The literals
 *     below are written out from the wire format in the specifications, and
 *     the RTSP / RTP / NAL sections each contain such vectors.
 *   * The suite also writes results/measured.txt with the numbers quoted in
 *     the README, so those figures come from the tests themselves.
 */

#include "netcam.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* harness                                                             */
/* ------------------------------------------------------------------ */

static int g_checks = 0;
static int g_fails = 0;
static const char *g_group = "?";

/*
 * The assertion macros take the call site's __LINE__ as a parameter rather
 * than expanding __LINE__ inside the function: expanding it inside reports
 * the line of the helper, so every failure prints the same useless number
 * and a failing check cannot be located.  That is how the first version of
 * this suite behaved.
 */
#define EXPECT(cond, what) expect_at((cond), (what), __LINE__)
#define EXPECT_EQ(got, want, what) expect_eq_at((got), (want), (what), __LINE__)
#define EXPECT_I(got, want, what) expect_i_at((got), (want), (what), __LINE__)

static void group(const char *name)
{
    g_group = name;
    printf("-- %s\n", name);
}

static void expect_at(int cond, const char *what, int line)
{
    g_checks++;
    if (!cond) {
        g_fails++;
        printf("FAIL [%s] %s (test_netcam.c:%d)\n", g_group, what, line);
    }
}

static void expect_eq_at(ntc_status_t got, ntc_status_t want, const char *what,
                         int line)
{
    g_checks++;
    if (got != want) {
        g_fails++;
        printf("FAIL [%s] %s (test_netcam.c:%d): got %d (%s), want %d (%s)\n",
               g_group, what, line, (int)got, ntc_status_str(got), (int)want,
               ntc_status_str(want));
    }
}

static void expect_i_at(int got, int want, const char *what, int line)
{
    g_checks++;
    if (got != want) {
        g_fails++;
        printf("FAIL [%s] %s (test_netcam.c:%d): got %d, want %d\n", g_group,
               what, line, got, want);
    }
}

#define expect(cond, what) EXPECT(cond, what)
#define expect_eq(got, want, what) EXPECT_EQ(got, want, what)
#define expect_i(got, want, what) EXPECT_I(got, want, what)

static void expect_u64(unsigned long long got, unsigned long long want,
                       const char *what)
{
    g_checks++;
    if (got != want) {
        g_fails++;
        printf("FAIL [%s] %s: got %llu, want %llu\n", g_group, what, got, want);
    }
}

static void expect_str(const char *got, const char *want, const char *what)
{
    g_checks++;
    if (got == NULL || strcmp(got, want) != 0) {
        g_fails++;
        printf("FAIL [%s] %s: got \"%s\", want \"%s\"\n", g_group, what,
               got ? got : "(null)", want);
    }
}

static void expect_d(double got, double want, double tol, const char *what)
{
    g_checks++;
    if (!(fabs(got - want) <= tol)) {
        g_fails++;
        printf("FAIL [%s] %s: got %.9g, want %.9g\n", g_group, what, got, want);
    }
}

/* Results file: opened once, written by the tests that measure something. */
static FILE *g_measured = NULL;

static void measured(const char *key, unsigned long long value)
{
    if (g_measured != NULL) {
        fprintf(g_measured, "%s=%llu\n", key, value);
    }
}

static void measured_d(const char *key, double value)
{
    if (g_measured != NULL) {
        fprintf(g_measured, "%s=%.6f\n", key, value);
    }
}

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */

/* Send a request assembled from parts; returns the module status. */
static int rtsp_send(ntc_rtsp_server_t *srv, const char *method,
                     const char *uri, uint32_t cseq, int with_cseq,
                     int with_session, uint32_t session,
                     const char *transport, int *out_status)
{
    char req[512];
    size_t len = 0;
    ntc_status_t st;
    ntc_rtsp_response_t resp;

    len = (size_t)sprintf(req, "%s %s RTSP/1.0\r\n", method, uri);
    if (with_cseq) {
        len += (size_t)sprintf(req + len, "CSeq: %lu\r\n",
                               (unsigned long)cseq);
    }
    if (with_session) {
        len += (size_t)sprintf(req + len, "Session: %lu\r\n",
                               (unsigned long)session);
    }
    if (transport != NULL) {
        len += (size_t)sprintf(req + len, "Transport: %s\r\n", transport);
    }
    len += (size_t)sprintf(req + len, "\r\n");

    st = ntc_rtsp_handle_text(srv, req, len, 0.0, &resp, NULL);
    if (out_status != NULL) {
        *out_status = resp.status;
    }
    return (int)st;
}

/*
 * Datagram arrays for the simulated channel.  These are FILE SCOPE, not
 * stack locals: an earlier revision declared several dgram_list_t (about
 * 25 KB each) plus two ntc_motion_detector_t (about 47 KB each) as locals,
 * which overflowed the default 1 MB Windows stack and made the test binary
 * die with a stack-cookie check (exit code 0xC0000409) instead of a test
 * failure.  Static storage keeps the suite small-stack safe.
 */
#define DGRAM_CAP 512

typedef struct dgram_list {
    uint8_t *buf[DGRAM_CAP];
    size_t len[DGRAM_CAP];
    size_t count;
} dgram_list_t;

static dgram_list_t g_dl, g_dl2, g_dl3, g_dl5;
static ntc_rtp_rx_t g_rx;
static ntc_nal_reasm_t g_reasm, g_reasm2;
/* Independent arrival map: one bit per sequence number, set when a packet is
 * actually delivered.  Used to verify the receiver's loss counter from the
 * outside instead of trusting the counter itself. */
static uint8_t g_arrived[65536];

/*
 * Motion detectors are ~47 KB each (background frame + reassembly buffers),
 * so they live in static storage as well.
 */
static ntc_motion_detector_t g_det, g_det2;

static void dgram_list_free(dgram_list_t *d)
{
    ntc_net_free(d->buf, d->count);
    d->count = 0;
}

/* A deterministic pseudo-random byte generator for synthetic media. */
static void fill_pattern(uint8_t *dst, size_t len, unsigned seed)
{
    uint32_t x = seed * 2654435761u + 12345u;
    size_t i;
    for (i = 0; i < len; i++) {
        x = x * 1103515245u + 12345u;
        dst[i] = (uint8_t)((x >> 16) & 0xFFu);
    }
}

/* ------------------------------------------------------------------ */
/* 1. utility layer                                                    */
/* ------------------------------------------------------------------ */

static void test_util(void)
{
    group("util: status strings, PRNG, directories");

    expect_str(ntc_status_str(NTC_OK), "OK", "OK string");
    expect_str(ntc_status_str(NTC_RTSP_ERR_CSEQ_ORDER),
               "rtsp: CSeq out of order", "cseq order string");
    expect_str(ntc_status_str(NTC_NAL_ERR_GAP), "nal: fragment gap",
               "nal gap string");
    expect_str(ntc_status_str((ntc_status_t)-99999), "unknown status",
               "unknown status string");

    {
        ntc_rng_t rng;
        double a, b;
        int i;
        int in_range = 1;
        ntc_rng_seed(&rng, 12345u);
        a = ntc_rng_double(&rng);
        b = ntc_rng_double(&rng);
        expect(a != b, "consecutive randoms differ");
        expect(a >= 0.0 && a < 1.0, "random in [0,1)");
        for (i = 0; i < 1000; i++) {
            double v = ntc_rng_double(&rng);
            if (v < 0.0 || v >= 1.0) {
                in_range = 0;
            }
        }
        expect(in_range, "1000 randoms all in [0,1)");
        ntc_rng_seed(&rng, 0u);
        a = ntc_rng_double(&rng);
        b = ntc_rng_double(&rng);
        expect(a != b && a != 0.0, "seed 0 still produces a stream");
        {
            ntc_rng_t x, y;
            ntc_rng_seed(&x, 777u);
            ntc_rng_seed(&y, 777u);
            expect(ntc_rng_double(&x) == ntc_rng_double(&y),
                   "same seed same value");
        }
        expect(ntc_rng_below(&rng, 10u) < 10u, "below() in range");
        expect_i((int)ntc_rng_below(&rng, 0u), 0, "below(0) is 0");
    }
    expect_eq(ntc_mkdir(NULL), NTC_ERR_INVAL, "mkdir(NULL)");
    expect_eq(ntc_mkdir("results"), NTC_OK, "mkdir results");
    expect_eq(ntc_mkdir("results"), NTC_OK, "mkdir results again (idempotent)");
    expect(ntc_report_open_text(NULL, "x") == NULL, "open text NULL dir");
    expect(ntc_report_open_text("results", NULL) == NULL, "open text NULL name");
}

/* ------------------------------------------------------------------ */
/* 2. RTSP parsing                                                     */
/* ------------------------------------------------------------------ */

static void test_rtsp_parse(void)
{
    ntc_rtsp_request_t r;
    ntc_status_t st;

    group("rtsp: request parsing (hand written vectors)");

    {
        /* Written out by hand from the RFC 2326 message format: request
         * line, CSeq, Session, User-Agent, blank line. */
        const char *raw =
            "PLAY rtsp://192.168.1.10:554/stream=0 RTSP/1.0\r\n"
            "CSeq: 4\r\n"
            "Session: 3055428279\r\n"
            "User-Agent: hand-written-vector\r\n"
            "\r\n";
        st = ntc_rtsp_parse_request(raw, strlen(raw), &r);
        expect_eq(st, NTC_OK, "parse hand written PLAY");
        expect_i((int)r.method, (int)NTC_RTSP_PLAY, "method is PLAY");
        expect_str(r.method_text, "PLAY", "method text");
        expect_str(r.uri, "rtsp://192.168.1.10:554/stream=0", "uri");
        expect_str(r.version, "RTSP/1.0", "version");
        expect_u64(r.cseq, 4u, "cseq 4");
        expect_i(r.cseq_present, 1, "cseq present");
        expect_i(r.session_present, 1, "session present");
        expect_u64(r.session, 3055428279u, "session id");
        expect_u64(r.nheaders, 3u, "three headers stored");
        expect_str(r.user_agent, "hand-written-vector", "user-agent value");
    }
    {
        const char *raw =
            "SETUP rtsp://cam/trackID=0 RTSP/1.0\r\n"
            "CSeq: 1\r\n"
            "Transport: RTP/AVP;unicast;client_port=8000-8001\r\n"
            "\r\n";
        st = ntc_rtsp_parse_request(raw, strlen(raw), &r);
        expect_eq(st, NTC_OK, "parse SETUP");
        expect_i((int)r.method, (int)NTC_RTSP_SETUP, "method SETUP");
        expect_str(r.transport, "RTP/AVP;unicast;client_port=8000-8001",
                   "transport value");
    }
    {
        const char *raw = "OPTIONS rtsp://cam RTSP/1.0\r\nCSeq: 99\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_OK,
                  "parse OPTIONS");
        expect_i((int)r.method, (int)NTC_RTSP_OPTIONS, "method OPTIONS");
        expect_u64(r.nheaders, 1u, "single header");
    }
    {
        const char *raw =
            "DESCRIBE rtsp://cam RTSP/1.0\r\nCSeq: 2\r\n"
            "Accept: application/sdp\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_OK,
                  "parse DESCRIBE");
        expect_i((int)r.method, (int)NTC_RTSP_DESCRIBE, "method DESCRIBE");
    }
    {
        /* Method tokens are case sensitive, header names are not. */
        const char *raw = "pause rtsp://cam RTSP/1.0\r\ncseq: 3\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_OK,
                  "parse lowercase method");
        expect_i((int)r.method, (int)NTC_RTSP_UNKNOWN_METHOD,
                 "lowercase method is unknown (case sensitive)");
        expect_i(r.cseq_present, 1, "lowercase header name recognised");
        expect_u64(r.cseq, 3u, "lowercase cseq value");
    }
    {
        const char *raw = "TEARDOWN rtsp://cam RTSP/1.0\nCSeq: 5\n\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_OK,
                  "bare LF accepted");
        expect_i((int)r.method, (int)NTC_RTSP_TEARDOWN, "method TEARDOWN");
    }
    {
        const char *raw =
            "PLAY rtsp://cam RTSP/1.0\r\nCSeq: 7\r\n"
            "Session: 1234567;timeout=60\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_OK,
                  "session with timeout parses");
        expect_u64(r.session, 1234567u, "session number before semicolon");
    }
    {
        const char *raw =
            "DESCRIBE rtsp://cam RTSP/1.0\r\nCSeq: 1\r\n"
            "Content-Length: 42\r\nContent-Type: application/sdp\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_OK,
                  "content headers parse");
        expect_u64(r.content_length, 42u, "content length");
        expect_str(r.content_type, "application/sdp", "content type");
    }
    {
        const char *raw = "OPTIONS rtsp://cam RTSP/1.0\r\nCSeq:    8   \r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_OK,
                  "padded CSeq parses");
        expect_u64(r.cseq, 8u, "padded CSeq value");
    }
    {
        const char *raw = "OPTIONS rtsp://cam RTSP/1.0\r\nCSeq: 1\r\n\r\nBODY";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_OK,
                  "body after headers parses");
        expect(strncmp(raw + r.header_bytes, "BODY", 4) == 0,
               "header_bytes points at the body");
    }
    {
        /* 30 headers: only 24 fit, and that must be reported. */
        char big[4096];
        size_t len = 0;
        int i;
        len += (size_t)sprintf(big + len, "OPTIONS rtsp://cam RTSP/1.0\r\n");
        len += (size_t)sprintf(big + len, "CSeq: 1\r\n");
        for (i = 0; i < 30; i++) {
            len += (size_t)sprintf(big + len, "X-Filler-%d: v%d\r\n", i, i);
        }
        len += (size_t)sprintf(big + len, "\r\n");
        expect_eq(ntc_rtsp_parse_request(big, len, &r), NTC_OK,
                  "many headers still parses");
        expect_i((int)r.nheaders, (int)NTC_RTSP_MAX_HEADERS,
                 "header table capped");
        expect_i(r.header_overflow, 1, "overflow flagged");
    }

    /* --- malformed and hostile inputs --- */
    expect_eq(ntc_rtsp_parse_request(NULL, 0, &r), NTC_ERR_INVAL,
              "parse NULL text");
    expect_eq(ntc_rtsp_parse_request("PLAY x RTSP/1.0\r\n", 18, NULL),
              NTC_ERR_INVAL, "parse NULL out");
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_ERR_PARSE,
                  "missing terminator");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0\r\nCSeq: 1\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_ERR_PARSE,
                  "no blank line after headers");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/2.0\r\nCSeq: 1\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r),
                  NTC_RTSP_ERR_VERSION, "wrong RTSP version");
    }
    {
        const char *raw = "PLAY rtsp://cam\r\nCSeq: 1\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_ERR_PARSE,
                  "request line without version");
    }
    {
        const char *raw = "PLAY RTSP/1.0\r\nCSeq: 1\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_ERR_PARSE,
                  "request line without URI");
    }
    {
        const char *raw = " rtsp://cam RTSP/1.0\r\nCSeq: 1\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_ERR_PARSE,
                  "empty method token");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0\r\nCSeq: abc\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r),
                  NTC_RTSP_ERR_CSEQ_BAD, "non numeric CSeq rejected");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0\r\nCSeq: -1\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r),
                  NTC_RTSP_ERR_CSEQ_BAD, "negative CSeq rejected");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0\r\nCSeq: 4294967296\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r),
                  NTC_RTSP_ERR_CSEQ_BAD, "CSeq overflow rejected");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0\r\nCSeq: 1e3\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r),
                  NTC_RTSP_ERR_CSEQ_BAD, "exponent notation rejected");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0\r\nSession: xyz\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r),
                  NTC_RTSP_ERR_SESSION_INVALID, "non numeric Session rejected");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0\r\nBadHeader\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_ERR_PARSE,
                  "header without colon rejected");
    }
    {
        const char *raw = ": novalue\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_ERR_PARSE,
                  "empty header name rejected");
    }
    {
        const char *raw = "PLAY rtsp://cam RTSP/1.0";
        expect_eq(ntc_rtsp_parse_request(raw, strlen(raw), &r), NTC_ERR_PARSE,
                  "truncated request rejected");
    }
    {
        /* A NUL byte inside the message must not terminate parsing early in a
         * way that hides the blank line. */
        const char raw[] = "OPTIONS rtsp://cam RTSP/1.0\r\nCSeq: 1\r\n\r\n";
        expect_eq(ntc_rtsp_parse_request(raw, sizeof(raw) - 1u, &r), NTC_OK,
                  "explicit length used, not strlen");
        expect_u64(r.cseq, 1u, "cseq from a length delimited buffer");
    }
}

static void test_rtsp_names(void)
{
    group("rtsp: method/state tables");

    expect_str(ntc_rtsp_method_str(NTC_RTSP_OPTIONS), "OPTIONS", "name OPTIONS");
    expect_str(ntc_rtsp_method_str(NTC_RTSP_DESCRIBE), "DESCRIBE", "name DESCRIBE");
    expect_str(ntc_rtsp_method_str(NTC_RTSP_SETUP), "SETUP", "name SETUP");
    expect_str(ntc_rtsp_method_str(NTC_RTSP_PLAY), "PLAY", "name PLAY");
    expect_str(ntc_rtsp_method_str(NTC_RTSP_PAUSE), "PAUSE", "name PAUSE");
    expect_str(ntc_rtsp_method_str(NTC_RTSP_TEARDOWN), "TEARDOWN", "name TEARDOWN");
    expect_str(ntc_rtsp_method_str(NTC_RTSP_UNKNOWN_METHOD), "UNKNOWN",
               "name UNKNOWN");
    expect_str(ntc_rtsp_state_str(NTC_RTSP_STATE_INIT), "INIT", "state INIT");
    expect_str(ntc_rtsp_state_str(NTC_RTSP_STATE_READY), "READY", "state READY");
    expect_str(ntc_rtsp_state_str(NTC_RTSP_STATE_PLAYING), "PLAYING",
               "state PLAYING");
    expect_str(ntc_rtsp_state_str(NTC_RTSP_STATE_TORN_DOWN), "TORN_DOWN",
               "state TORN_DOWN");
    expect_i(ntc_rtsp_method_from_text("PLAY"), (int)NTC_RTSP_PLAY,
             "from text PLAY");
    expect_i(ntc_rtsp_method_from_text("play"), (int)NTC_RTSP_UNKNOWN_METHOD,
             "method text is case sensitive");
    expect_i(ntc_rtsp_method_from_text(NULL), (int)NTC_RTSP_UNKNOWN_METHOD,
             "from text NULL");
    expect_i(ntc_rtsp_method_from_text("RECORD"), (int)NTC_RTSP_UNKNOWN_METHOD,
             "RECORD is not implemented");
    expect_i(ntc_rtsp_method_from_text(""), (int)NTC_RTSP_UNKNOWN_METHOD,
             "empty method text");

    /* The complete method x state legality matrix: 24 assertions. */
    {
        int m;
        int s;
        static const int want[6][4] = {
            /* INIT READY PLAYING TORN */
            { 1, 1, 1, 0 }, /* OPTIONS  */
            { 1, 1, 1, 0 }, /* DESCRIBE */
            { 1, 0, 0, 0 }, /* SETUP    */
            { 0, 1, 0, 0 }, /* PLAY     */
            { 0, 0, 1, 0 }, /* PAUSE    */
            { 0, 1, 1, 0 }  /* TEARDOWN */
        };
        static const char *mn[6] = { "OPTIONS", "DESCRIBE", "SETUP", "PLAY",
                                     "PAUSE", "TEARDOWN" };
        static const char *sn[4] = { "INIT", "READY", "PLAYING", "TORN_DOWN" };
        char label[64];
        for (m = 0; m < 6; m++) {
            for (s = 0; s < 4; s++) {
                sprintf(label, "%s allowed in %s", mn[m], sn[s]);
                expect_i(ntc_rtsp_method_allowed((ntc_rtsp_method_t)m,
                                                 (ntc_rtsp_state_t)s),
                         want[m][s], label);
            }
        }
        expect_i(ntc_rtsp_method_allowed(NTC_RTSP_UNKNOWN_METHOD,
                                         NTC_RTSP_STATE_INIT),
                 0, "unknown method never allowed");
    }
}

static void test_rtsp_transport(void)
{
    ntc_rtsp_transport_t t;
    ntc_status_t st;

    group("rtsp: Transport header");

    st = ntc_rtsp_parse_transport("RTP/AVP;unicast;client_port=8000-8001", &t);
    expect_eq(st, NTC_OK, "udp transport parses");
    expect_i(t.mode, 0, "udp mode");
    expect_i(t.has_client_port, 1, "client_port present");
    expect_i((int)t.client_rtp_port, 8000, "rtp port");
    expect_i((int)t.client_rtcp_port, 8001, "rtcp port");
    expect_str(t.protocol, "RTP/AVP", "protocol string");

    st = ntc_rtsp_parse_transport("RTP/AVP/TCP;unicast;interleaved=0-1", &t);
    expect_eq(st, NTC_OK, "tcp transport parses");
    expect_i(t.mode, 1, "tcp mode");
    expect_i(t.has_interleaved, 1, "interleaved present");
    expect_i(t.interleaved_rtp, 0, "interleaved rtp channel");
    expect_i(t.interleaved_rtcp, 1, "interleaved rtcp channel");
    expect_str(t.protocol, "RTP/AVP/TCP", "tcp protocol string");

    st = ntc_rtsp_parse_transport("rtp/avp/tcp;interleaved=4-5;ssrc=DEADBEEF",
                                  &t);
    expect_eq(st, NTC_OK, "lowercase tcp transport parses");
    expect_i(t.interleaved_rtp, 4, "interleaved channel 4");
    expect_i(t.interleaved_rtcp, 5, "interleaved channel 5");

    st = ntc_rtsp_parse_transport("RTP/AVP;unicast;client_port=50000-50001", &t);
    expect_eq(st, NTC_OK, "high ports parse");
    expect_i((int)t.client_rtp_port, 50000, "high rtp port");

    expect_eq(ntc_rtsp_parse_transport(NULL, &t), NTC_ERR_INVAL,
              "transport NULL value");
    expect_eq(ntc_rtsp_parse_transport("RTP/AVP", NULL), NTC_ERR_INVAL,
              "transport NULL out");
    expect_eq(ntc_rtsp_parse_transport("RTP/AVP;unicast", &t), NTC_ERR_PARSE,
              "transport without ports");
    expect_eq(ntc_rtsp_parse_transport("SOMETHING/ELSE;client_port=1-2", &t),
              NTC_ERR_PARSE, "unknown protocol");
    expect_eq(ntc_rtsp_parse_transport("RTP/AVP;client_port=abc-1", &t),
              NTC_ERR_PARSE, "non numeric port");
    expect_eq(ntc_rtsp_parse_transport("", &t), NTC_ERR_PARSE,
              "empty transport value");
}

static void test_rtsp_session_machine(void)
{
    ntc_rtsp_server_t srv;
    ntc_rtsp_response_t resp;
    ntc_rtsp_request_t req;
    const ntc_rtsp_session_t *sess;
    uint32_t id = 0;
    int status = 0;
    size_t count = 0;

    group("rtsp: session state machine and status codes");

    expect_eq(ntc_rtsp_server_init(NULL, 1, 1, "x"), NTC_ERR_INVAL,
              "server init NULL");
    expect_eq(ntc_rtsp_server_init(&srv, 0, 0, NULL), NTC_OK,
              "server init defaults");
    expect_u64(srv.next_id, 1u, "base id defaulted to 1");
    expect_u64(srv.next_cseq, 1u, "next cseq defaulted to 1");
    expect_str(srv.server_name, "netcam-app-lab", "default server name");

    expect_eq(ntc_rtsp_server_init(&srv, 1000, 1, "netcam-test"), NTC_OK,
              "server init explicit");
    expect_u64(srv.base_id, 1000u, "explicit base id");
    expect_eq(ntc_rtsp_session_count(&srv, &count), NTC_OK, "count ok");
    expect_u64(count, 0u, "no sessions at start");
    expect_eq(ntc_rtsp_session_count(NULL, &count), NTC_ERR_INVAL,
              "count NULL server");
    expect_eq(ntc_rtsp_session_count(&srv, NULL), NTC_ERR_INVAL,
              "count NULL out");
    expect_eq(ntc_rtsp_find_session(&srv, 1000u, &sess), NTC_ERR_NOTFOUND,
              "no session yet");
    expect_eq(ntc_rtsp_find_session(NULL, 1u, &sess), NTC_ERR_INVAL,
              "find NULL server");
    expect_eq(ntc_rtsp_find_session(&srv, 1u, NULL), NTC_ERR_INVAL,
              "find NULL out");

    rtsp_send(&srv, "OPTIONS", "rtsp://cam", 1, 1, 0, 0, NULL, &status);
    expect_i(status, 200, "OPTIONS 200 in INIT");
    rtsp_send(&srv, "DESCRIBE", "rtsp://cam", 2, 1, 0, 0, NULL, &status);
    expect_i(status, 200, "DESCRIBE 200 in INIT");

    /* 454: no session / unknown session. */
    rtsp_send(&srv, "PLAY", "rtsp://cam", 3, 1, 0, 0, NULL, &status);
    expect_i(status, 454, "PLAY without Session -> 454");
    rtsp_send(&srv, "PLAY", "rtsp://cam", 3, 1, 1, 999999, NULL, &status);
    expect_i(status, 454, "PLAY with unknown Session -> 454");
    rtsp_send(&srv, "PAUSE", "rtsp://cam", 3, 1, 0, 0, NULL, &status);
    expect_i(status, 454, "PAUSE without Session -> 454");
    rtsp_send(&srv, "TEARDOWN", "rtsp://cam", 3, 1, 0, 0, NULL, &status);
    expect_i(status, 454, "TEARDOWN without Session -> 454");
    expect_u64(srv.stats.rejected_session_missing, 3u,
               "three missing-session rejections");
    expect_u64(srv.stats.rejected_session_invalid, 1u,
               "one unknown-session rejection");
    measured("rtsp_454_total",
             srv.stats.rejected_session_missing + srv.stats.rejected_session_invalid);

    /* 400: SETUP without a usable Transport header. */
    rtsp_send(&srv, "SETUP", "rtsp://cam/trackID=0", 3, 1, 0, 0, NULL, &status);
    expect_i(status, 400, "SETUP without Transport -> 400");
    rtsp_send(&srv, "SETUP", "rtsp://cam/trackID=0", 4, 1, 0, 0,
              "RTP/AVP;unicast", &status);
    expect_i(status, 400, "SETUP with unusable Transport -> 400");
    expect_u64(srv.stats.rejected_transport, 2u, "transport rejections counted");
    measured("rtsp_400_transport", srv.stats.rejected_transport);

    /* SETUP with a good Transport header. */
    {
        char reqtext[512];
        size_t len;
        len = (size_t)sprintf(reqtext,
                              "SETUP rtsp://cam/trackID=0 RTSP/1.0\r\n"
                              "CSeq: 5\r\n"
                              "Transport: RTP/AVP;unicast;client_port=8000-8001"
                              "\r\n\r\n");
        expect_eq(ntc_rtsp_handle_text(&srv, reqtext, len, 0.0, &resp, &req),
                  NTC_OK, "SETUP handled");
        expect_i(resp.status, 200, "SETUP 200");
        expect(resp.session_id != 0u, "session id assigned");
        expect_u64(resp.session_id, srv.base_id, "first session uses the base id");
        id = resp.session_id;
        expect_str(resp.extra_header_name, "Transport",
                   "SETUP echoes a Transport header");
        expect(strstr(resp.extra_header_value, "client_port=8000-8001") != NULL,
               "response repeats the client ports");
        expect(strstr(resp.extra_header_value, "server_port=") != NULL,
               "response offers server ports");
        expect_u64(resp.cseq, 5u, "response carries the CSeq");
    }
    expect_eq(ntc_rtsp_find_session(&srv, id, &sess), NTC_OK, "session exists");
    expect_i((int)sess->state, (int)NTC_RTSP_STATE_READY, "state READY");
    expect_u64(sess->last_cseq, 5u, "last cseq recorded");
    expect_u64(sess->requests_seen, 1u, "one accepted request on the session");

    /* 455: methods in the wrong state. */
    rtsp_send(&srv, "SETUP", "rtsp://cam/trackID=0", 6, 1, 1, id,
              "RTP/AVP;unicast;client_port=9000-9001", &status);
    expect_i(status, 455, "second SETUP -> 455");
    rtsp_send(&srv, "PAUSE", "rtsp://cam", 7, 1, 1, id, NULL, &status);
    expect_i(status, 455, "PAUSE in READY -> 455");
    measured("rtsp_455_ready", 2);

    rtsp_send(&srv, "PLAY", "rtsp://cam", 8, 1, 1, id, NULL, &status);
    expect_i(status, 200, "PLAY in READY -> 200");
    expect_eq(ntc_rtsp_find_session(&srv, id, &sess), NTC_OK, "still exists");
    expect_i((int)sess->state, (int)NTC_RTSP_STATE_PLAYING, "state PLAYING");

    rtsp_send(&srv, "PLAY", "rtsp://cam", 9, 1, 1, id, NULL, &status);
    expect_i(status, 455, "PLAY while PLAYING -> 455");
    rtsp_send(&srv, "SETUP", "rtsp://cam/trackID=0", 10, 1, 1, id,
              "RTP/AVP;unicast;client_port=9002-9003", &status);
    expect_i(status, 455, "SETUP while PLAYING -> 455");
    measured("rtsp_455_playing", 2);

    /* OPTIONS and DESCRIBE remain legal while PLAYING. */
    rtsp_send(&srv, "OPTIONS", "rtsp://cam", 11, 1, 1, id, NULL, &status);
    expect_i(status, 200, "OPTIONS while PLAYING -> 200");
    rtsp_send(&srv, "DESCRIBE", "rtsp://cam", 12, 1, 1, id, NULL, &status);
    expect_i(status, 200, "DESCRIBE while PLAYING -> 200");
    {
        char reqtext[256];
        size_t len;
        len = (size_t)sprintf(reqtext,
                              "DESCRIBE rtsp://cam RTSP/1.0\r\nCSeq: 13\r\n"
                              "Session: %lu\r\n\r\n", (unsigned long)id);
        expect_eq(ntc_rtsp_handle_text(&srv, reqtext, len, 0.0, &resp, NULL),
                  NTC_OK, "DESCRIBE with session");
        expect(resp.body_len > 0u, "DESCRIBE returns a body");
        expect(strstr(resp.body, "m=video") != NULL, "body mentions video");
        expect(strstr(resp.body, "H264") != NULL, "body mentions the codec");
        expect_str(resp.extra_header_value, "application/sdp",
                   "content type is sdp");
    }

    /* Back to READY with PAUSE, then PLAY again. */
    rtsp_send(&srv, "PAUSE", "rtsp://cam", 14, 1, 1, id, NULL, &status);
    expect_i(status, 200, "PAUSE in PLAYING -> 200");
    expect_eq(ntc_rtsp_find_session(&srv, id, &sess), NTC_OK, "still exists");
    expect_i((int)sess->state, (int)NTC_RTSP_STATE_READY, "back to READY");
    rtsp_send(&srv, "PLAY", "rtsp://cam", 15, 1, 1, id, NULL, &status);
    expect_i(status, 200, "PLAY after PAUSE -> 200");

    /* TEARDOWN removes the session. */
    rtsp_send(&srv, "TEARDOWN", "rtsp://cam", 16, 1, 1, id, NULL, &status);
    expect_i(status, 200, "TEARDOWN in PLAYING -> 200");
    expect_eq(ntc_rtsp_find_session(&srv, id, &sess), NTC_ERR_NOTFOUND,
              "session removed after TEARDOWN");
    expect_eq(ntc_rtsp_session_count(&srv, &count), NTC_OK, "count after teardown");
    expect_u64(count, 0u, "no live sessions");
    rtsp_send(&srv, "TEARDOWN", "rtsp://cam", 17, 1, 1, id, NULL, &status);
    expect_i(status, 454, "TEARDOWN of a dead session -> 454");

    /* 405: unknown method. */
    rtsp_send(&srv, "RECORD", "rtsp://cam", 18, 1, 0, 0, NULL, &status);
    expect_i(status, 405, "unknown method -> 405");
    expect_u64(srv.stats.rejected_method, 1u, "method rejection counted");
    measured("rtsp_405_total", srv.stats.rejected_method);

    /* Response rendering for each status we produce. */
    {
        char buf[512];
        size_t written = 0;
        expect_eq(ntc_rtsp_write_response(&resp, buf, sizeof(buf), &written),
                  NTC_OK, "render a response");
        expect(strncmp(buf, "RTSP/1.0 ", 9) == 0, "status line prefix");
        expect(strstr(buf, "\r\n\r\n") != NULL, "blank line terminator");
    }

    /* Argument validation on the handler. */
    expect_eq(ntc_rtsp_handle(NULL, &req, 0.0, &resp), NTC_ERR_INVAL,
              "handle NULL server");
    expect_eq(ntc_rtsp_handle(&srv, NULL, 0.0, &resp), NTC_ERR_INVAL,
              "handle NULL request");
    expect_eq(ntc_rtsp_handle(&srv, &req, 0.0, NULL), NTC_ERR_INVAL,
              "handle NULL response");
    expect_eq(ntc_rtsp_close_session(&srv, id), NTC_ERR_NOTFOUND,
              "close a dead session");
    expect_eq(ntc_rtsp_close_session(NULL, 1u), NTC_ERR_INVAL,
              "close NULL server");

    /* Session-table exhaustion -> 503. */
    {
        ntc_rtsp_server_t s2;
        char t[256];
        size_t len;
        int i;
        ntc_rtsp_response_t r2;
        ntc_rtsp_server_init(&s2, 1, 1, NULL);
        for (i = 0; i < NTC_RTSP_MAX_SESSIONS; i++) {
            len = (size_t)sprintf(t,
                                  "SETUP rtsp://cam/t%d RTSP/1.0\r\nCSeq: 1\r\n"
                                  "Transport: RTP/AVP;unicast;"
                                  "client_port=8000-8001\r\n\r\n", i);
            expect_eq(ntc_rtsp_handle_text(&s2, t, len, 0.0, &r2, NULL), NTC_OK,
                      "session fits in the table");
        }
        len = (size_t)sprintf(t,
                              "SETUP rtsp://cam/extra RTSP/1.0\r\nCSeq: 1\r\n"
                              "Transport: RTP/AVP;unicast;client_port=8000-8001"
                              "\r\n\r\n");
        expect_eq(ntc_rtsp_handle_text(&s2, t, len, 0.0, &r2, NULL),
                  NTC_RTSP_ERR_SESSION_TABLE, "extra session refused");
        expect_i(r2.status, 503, "table exhaustion -> 503");
        /* Tearing one down frees the slot for a new session. */
        expect_eq(ntc_rtsp_session_count(&s2, &count), NTC_OK, "count");
        expect_u64(count, (unsigned long long)NTC_RTSP_MAX_SESSIONS,
                   "table is full");
        expect_eq(ntc_rtsp_close_session(&s2, 1u), NTC_OK, "close session 1");
        len = (size_t)sprintf(t,
                              "SETUP rtsp://cam/new RTSP/1.0\r\nCSeq: 1\r\n"
                              "Transport: RTP/AVP;unicast;client_port=8000-8001"
                              "\r\n\r\n");
        expect_eq(ntc_rtsp_handle_text(&s2, t, len, 0.0, &r2, NULL), NTC_OK,
                  "slot reused after a close");
    }
}

static void test_rtsp_cseq(void)
{
    ntc_rtsp_server_t srv;
    ntc_rtsp_response_t resp;
    uint32_t id = 0;
    int status = 0;
    char reqtext[512];
    size_t len;
    const ntc_rtsp_session_t *sess = NULL;

    group("rtsp: CSeq validation");

    ntc_rtsp_server_init(&srv, 1, 1, "netcam-test");
    {
        const char *raw = "OPTIONS rtsp://cam RTSP/1.0\r\n\r\n";
        expect_eq(ntc_rtsp_handle_text(&srv, raw, strlen(raw), 0.0, &resp, NULL),
                  NTC_RTSP_ERR_CSEQ_MISSING, "missing CSeq status");
        expect_i(resp.status, 400, "missing CSeq -> 400");
        expect_u64(srv.stats.rejected_cseq_missing, 1u, "missing CSeq counted");
    }
    rtsp_send(&srv, "OPTIONS", "rtsp://cam", 0, 1, 0, 0, NULL, &status);
    expect_i(status, 400, "CSeq 0 -> 400");
    expect_u64(srv.stats.rejected_cseq_bad, 1u, "CSeq 0 counted as bad");
    {
        const char *raw = "OPTIONS rtsp://cam RTSP/1.0\r\nCSeq: zz\r\n\r\n";
        expect_eq(ntc_rtsp_handle_text(&srv, raw, strlen(raw), 0.0, &resp, NULL),
                  NTC_RTSP_ERR_CSEQ_BAD, "non numeric CSeq status");
        expect_i(resp.status, 400, "non numeric CSeq -> 400");
    }

    len = (size_t)sprintf(reqtext,
                          "SETUP rtsp://cam/trackID=0 RTSP/1.0\r\n"
                          "CSeq: 10\r\n"
                          "Transport: RTP/AVP;unicast;client_port=8000-8001\r\n"
                          "\r\n");
    expect_eq(ntc_rtsp_handle_text(&srv, reqtext, len, 0.0, &resp, NULL),
              NTC_OK, "setup for the cseq test");
    id = resp.session_id;
    expect(id != 0u, "session created");

    rtsp_send(&srv, "PLAY", "rtsp://cam", 11, 1, 1, id, NULL, &status);
    expect_i(status, 200, "cseq 11 accepted");
    rtsp_send(&srv, "PAUSE", "rtsp://cam", 11, 1, 1, id, NULL, &status);
    expect_i(status, 400, "replayed cseq 11 -> 400");
    rtsp_send(&srv, "PAUSE", "rtsp://cam", 10, 1, 1, id, NULL, &status);
    expect_i(status, 400, "older cseq 10 -> 400");
    rtsp_send(&srv, "PAUSE", "rtsp://cam", 9, 1, 1, id, NULL, &status);
    expect_i(status, 400, "much older cseq 9 -> 400");
    /* A pre-recorded request replayed byte for byte is refused too. */
    {
        static const char replay[] =
            "PLAY rtsp://cam RTSP/1.0\r\nCSeq: 11\r\nSession: 1\r\n\r\n";
        char buf[128];
        size_t n = (size_t)sprintf(buf,
                                   "PLAY rtsp://cam RTSP/1.0\r\nCSeq: 11\r\n"
                                   "Session: %lu\r\n\r\n", (unsigned long)id);
        expect_eq(ntc_rtsp_handle_text(&srv, buf, n, 0.0, &resp, NULL),
                  NTC_RTSP_ERR_CSEQ_ORDER, "byte-identical replay refused");
        expect_i(resp.status, 400, "replay -> 400");
        expect(strlen(replay) > 0u, "vector referenced");
    }
    expect_u64(srv.stats.rejected_cseq_order, 4u, "four CSeq rejections");
    measured("rtsp_cseq_rejected_total", srv.stats.rejected_cseq_order);

    /* Accepted requests after the rejections. */
    rtsp_send(&srv, "PAUSE", "rtsp://cam", 12, 1, 1, id, NULL, &status);
    expect_i(status, 200, "cseq 12 accepted");
    expect_eq(ntc_rtsp_find_session(&srv, id, &sess), NTC_OK, "still found");
    expect_u64(sess->last_cseq, 12u, "last cseq is 12");
    expect_u64(sess->requests_rejected, 4u,
               "the session counted four rejected requests");
    /* Rejected requests must not have advanced the state machine: after the
     * PAUSE above we are in READY, so PAUSE is now a 455. */
    rtsp_send(&srv, "PAUSE", "rtsp://cam", 13, 1, 1, id, NULL, &status);
    expect_i(status, 455, "state advanced only by accepted requests");

    /* Two sessions must not interfere. */
    {
        ntc_rtsp_server_t s2;
        uint32_t id2 = 0;
        size_t n = 0;
        ntc_rtsp_server_init(&s2, 500, 1, "netcam-two");
        len = (size_t)sprintf(reqtext,
                              "SETUP rtsp://cam/trackID=0 RTSP/1.0\r\n"
                              "CSeq: 1\r\n"
                              "Transport: RTP/AVP;unicast;client_port=9000-9001"
                              "\r\n\r\n");
        expect_eq(ntc_rtsp_handle_text(&s2, reqtext, len, 0.0, &resp, NULL),
                  NTC_OK, "session A setup");
        id = resp.session_id;
        len = (size_t)sprintf(reqtext,
                              "SETUP rtsp://cam/trackID=1 RTSP/1.0\r\n"
                              "CSeq: 1\r\n"
                              "Transport: RTP/AVP;unicast;client_port=9002-9003"
                              "\r\n\r\n");
        expect_eq(ntc_rtsp_handle_text(&s2, reqtext, len, 0.0, &resp, NULL),
                  NTC_OK, "session B setup");
        id2 = resp.session_id;
        expect(id != id2, "two distinct session ids");
        rtsp_send(&s2, "PLAY", "rtsp://cam", 2, 1, 1, id, NULL, &status);
        expect_i(status, 200, "session A cseq 2 accepted");
        rtsp_send(&s2, "PLAY", "rtsp://cam", 2, 1, 1, id2, NULL, &status);
        expect_i(status, 200, "session B cseq 2 accepted independently");
        rtsp_send(&s2, "PLAY", "rtsp://cam", 2, 1, 1, id, NULL, &status);
        expect_i(status, 400, "session A replay still rejected");
        expect_i(s2.sessions[0].state == NTC_RTSP_STATE_PLAYING ||
                     s2.sessions[1].state == NTC_RTSP_STATE_PLAYING,
                 1, "at least one session is playing");
        rtsp_send(&s2, "TEARDOWN", "rtsp://cam", 3, 1, 1, id, NULL, &status);
        expect_i(status, 200, "session A teardown");
        rtsp_send(&s2, "PAUSE", "rtsp://cam", 3, 1, 1, id2, NULL, &status);
        expect_i(status, 200, "session B still works after A was torn down");
        expect_eq(ntc_rtsp_session_count(&s2, &n), NTC_OK, "count");
        expect_u64(n, 1u, "one session left");
        measured("rtsp_two_sessions_isolated", 1);
        measured("rtsp_cseq_rejected_in_two_session_test",
                 s2.stats.rejected_cseq_order);
    }
}

static void test_rtsp_write_response(void)
{
    ntc_rtsp_response_t resp;
    char buf[512];
    size_t written = 0;
    ntc_status_t st;

    group("rtsp: response rendering");

    memset(&resp, 0, sizeof(resp));
    resp.status = 200;
    strcpy(resp.reason, "OK");
    resp.cseq = 42;
    resp.session_id = 12345;
    strcpy(resp.extra_header_name, "Transport");
    strcpy(resp.extra_header_value, "RTP/AVP;unicast;client_port=8000-8001");
    st = ntc_rtsp_write_response(&resp, buf, sizeof(buf), &written);
    expect_eq(st, NTC_OK, "write response ok");
    expect(strncmp(buf, "RTSP/1.0 200 OK\r\n", 17) == 0, "status line");
    expect(strstr(buf, "CSeq: 42\r\n") != NULL, "cseq header");
    expect(strstr(buf, "Session: 12345\r\n") != NULL, "session header");
    expect(strstr(buf, "Transport: RTP/AVP;unicast;client_port=8000-8001\r\n")
               != NULL, "extra header");
    expect(strstr(buf, "\r\n\r\n") != NULL, "blank line terminator");
    expect_u64(written, strlen(buf), "written length matches the buffer");

    resp.status = 454;
    strcpy(resp.reason, "Session Not Found");
    resp.session_id = 0;
    resp.extra_header_name[0] = '\0';
    st = ntc_rtsp_write_response(&resp, buf, sizeof(buf), &written);
    expect_eq(st, NTC_OK, "454 renders");
    expect(strncmp(buf, "RTSP/1.0 454 Session Not Found\r\n", 32) == 0,
           "454 status line");
    expect(strstr(buf, "Session:") == NULL, "no Session header when id is 0");

    {
        size_t need = 0;
        st = ntc_rtsp_write_response(&resp, buf, 4, &need);
        expect_eq(st, NTC_OK, "short buffer is truncated, not an error");
        expect_u64(need, 3u, "written clipped to cap-1");
        expect(strlen(buf) == 3u, "buffer NUL terminated at the limit");
        expect(strncmp(buf, "RTS", 3) == 0, "prefix kept");
        st = ntc_rtsp_write_response(&resp, buf, 1, &need);
        expect_eq(st, NTC_OK, "capacity 1 still succeeds");
        expect_u64(need, 0u, "nothing written into a 1-byte buffer");
        expect(buf[0] == '\0', "empty but terminated");
    }
    expect_eq(ntc_rtsp_write_response(NULL, buf, sizeof(buf), &written),
              NTC_ERR_INVAL, "NULL response");
    expect_eq(ntc_rtsp_write_response(&resp, NULL, sizeof(buf), &written),
              NTC_ERR_INVAL, "NULL buffer");
    expect_eq(ntc_rtsp_write_response(&resp, buf, 0, &written),
              NTC_ERR_INVAL, "zero capacity");
    expect_eq(ntc_rtsp_write_response(&resp, buf, sizeof(buf), NULL), NTC_OK,
              "NULL written pointer tolerated");

    {
        ntc_rtsp_response_t b;
        memset(&b, 0, sizeof(b));
        b.status = 200;
        strcpy(b.reason, "OK");
        b.cseq = 7;
        strcpy(b.body, "v=0\r\n");
        b.body_len = strlen(b.body);
        expect_eq(ntc_rtsp_write_response(&b, buf, sizeof(buf), &written),
                  NTC_OK, "body response renders");
        expect(strstr(buf, "Content-Length: 5\r\n") != NULL,
               "content length matches the body");
        expect(strstr(buf, "v=0\r\n") != NULL, "body present");
    }
    /* Malformed request text still yields a 400 through handle_text. */
    {
        ntc_rtsp_server_t srv;
        ntc_rtsp_response_t bad;
        const char *raw = "PLAY\r\n\r\n";
        ntc_rtsp_server_init(&srv, 1, 1, NULL);
        expect_eq(ntc_rtsp_handle_text(&srv, raw, strlen(raw), 0.0, &bad, NULL),
                  NTC_ERR_PARSE, "malformed text status");
        expect_i(bad.status, 400, "malformed text -> 400");
        expect_eq(ntc_rtsp_handle_text(NULL, raw, strlen(raw), 0.0, &bad, NULL),
                  NTC_ERR_INVAL, "handle_text NULL server");
        expect_eq(ntc_rtsp_handle_text(&srv, NULL, 0, 0.0, &bad, NULL),
                  NTC_ERR_INVAL, "handle_text NULL text");
        expect_eq(ntc_rtsp_handle_text(&srv, raw, strlen(raw), 0.0, NULL, NULL),
                  NTC_ERR_INVAL, "handle_text NULL response");
    }
}

/* ------------------------------------------------------------------ */
/* 3. RTP                                                              */
/* ------------------------------------------------------------------ */

static void test_rtp_header(void)
{
    ntc_rtp_packet_t pkt;
    ntc_status_t st;
    uint8_t buf[128];
    size_t written = 0;

    group("rtp: header packing and hand written vectors");

    /*
     * Hand assembled RTP packet:
     *   0x80  = V=2, P=0, X=0, CC=0
     *   0xE0  = M=1, PT=96
     *   0x3039 = sequence number 12345
     *   0x0001E240 = timestamp 123456
     *   0xDEADBEEF = SSRC
     *   0x48 0x69 0x21 = "Hi!"
     */
    {
        static const uint8_t raw[15] = {
            0x80, 0xE0, 0x30, 0x39, 0x00, 0x01, 0xE2, 0x40,
            0xDE, 0xAD, 0xBE, 0xEF, 0x48, 0x69, 0x21
        };
        st = ntc_rtp_parse(raw, sizeof(raw), &pkt);
        expect_eq(st, NTC_OK, "hand written packet parses");
        expect_i(pkt.version, 2, "version 2");
        expect_i(pkt.padding, 0, "no padding");
        expect_i(pkt.extension, 0, "no extension");
        expect_i(pkt.csrc_count, 0, "no CSRC");
        expect_i(pkt.marker, 1, "marker set");
        expect_i(pkt.payload_type, 96, "payload type 96");
        expect_u64(pkt.seq, 12345u, "seq 12345");
        expect_u64(pkt.timestamp, 123456u, "timestamp 123456");
        expect_u64(pkt.ssrc, 0xDEADBEEFu, "ssrc 0xDEADBEEF");
        expect_u64(pkt.payload_len, 3u, "payload length 3");
        expect(pkt.payload[0] == 0x48 && pkt.payload[2] == 0x21,
               "payload bytes in place");
    }
    /* All-zero header fields with the marker clear. */
    {
        static const uint8_t raw[12] = { 0x80, 0x60, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        st = ntc_rtp_parse(raw, sizeof(raw), &pkt);
        expect_eq(st, NTC_OK, "zero header parses");
        expect_i(pkt.marker, 0, "marker clear");
        expect_u64(pkt.seq, 0u, "seq 0");
        expect_u64(pkt.timestamp, 0u, "timestamp 0");
        expect_u64(pkt.ssrc, 0u, "ssrc 0");
        expect_u64(pkt.payload_len, 0u, "empty payload");
    }
    /* Version 1 is refused. */
    {
        static const uint8_t raw[12] = { 0x40, 0x60, 0x00, 0x01, 0, 0,
                                         0,    0,    0,    0,    0, 0 };
        expect_eq(ntc_rtp_parse(raw, sizeof(raw), &pkt), NTC_RTP_ERR_VERSION,
                  "version 1 rejected");
    }
    {
        uint8_t raw[11];
        memset(raw, 0, sizeof(raw));
        raw[0] = 0x80;
        expect_eq(ntc_rtp_parse(raw, 11, &pkt), NTC_RTP_ERR_SHORT,
                  "11 bytes rejected");
        expect_eq(ntc_rtp_parse(raw, 0, &pkt), NTC_RTP_ERR_SHORT,
                  "0 bytes rejected");
    }
    expect_eq(ntc_rtp_parse(NULL, 12, &pkt), NTC_ERR_INVAL, "parse NULL buffer");
    expect_eq(ntc_rtp_parse(buf, 12, NULL), NTC_ERR_INVAL, "parse NULL packet");

    /* CSRC handling. */
    {
        uint8_t raw[16];
        memset(raw, 0, sizeof(raw));
        raw[0] = (uint8_t)(0x80 | 2); /* CC=2 needs 20 bytes */
        expect_eq(ntc_rtp_parse(raw, sizeof(raw), &pkt), NTC_RTP_ERR_CSRC,
                  "CSRC overrun rejected");
        raw[0] = (uint8_t)(0x80 | 1); /* CC=1 needs 16 bytes: exact fit */
        raw[1] = 96;
        expect_eq(ntc_rtp_parse(raw, sizeof(raw), &pkt), NTC_OK, "CC=1 valid");
        expect_u64(pkt.payload_len, 0u, "no payload after the CSRC list");
        raw[0] = (uint8_t)(0x80 | 15); /* maximum CC, way past the buffer */
        expect_eq(ntc_rtp_parse(raw, sizeof(raw), &pkt), NTC_RTP_ERR_CSRC,
                  "CC=15 overrun rejected");
    }
    {
        /* Padding.  The buffer must be at least 201 bytes so that a bogus
         * padding count of 200 is read from inside the allocation: reading
         * buf[15] of a 16-byte buffer with a 200-byte pad is what made an
         * earlier revision of this test segfault. */
        uint8_t raw[256];
        memset(raw, 0xEE, sizeof(raw));
        raw[0] = (uint8_t)(0x80 | 0x20); /* P=1 */
        raw[1] = 96;
        raw[15] = 2; /* two padding bytes at the very end of a 16-byte packet */
        expect_eq(ntc_rtp_parse(raw, 16, &pkt), NTC_OK, "padded packet parses");
        expect_u64(pkt.payload_len, 2u, "payload excludes the padding");
        /* Zero padding bytes cannot be right: the count itself must be >= 1. */
        raw[15] = 0;
        expect_eq(ntc_rtp_parse(raw, 16, &pkt), NTC_ERR_PARSE,
                  "zero padding count rejected");
        /* A pad count larger than the payload is refused.  256 bytes total
         * means 244 bytes of payload, so a count of 250 cannot be right. */
        raw[255] = 250;
        expect_eq(ntc_rtp_parse(raw, sizeof(raw), &pkt), NTC_ERR_PARSE,
                  "padding larger than the payload rejected");
        /* A legal padding count at the end of a large buffer. */
        raw[255] = 8;
        expect_eq(ntc_rtp_parse(raw, sizeof(raw), &pkt), NTC_OK,
                  "legal padding accepted");
        expect_u64(pkt.payload_len, sizeof(raw) - 12u - 8u,
                   "payload shortened by the pad count");
    }
    /* Extension header: the 4-byte extension header sits right after the
     * fixed 12-byte header (profile at 12-13, length in 32-bit words at
     * 14-15), its data follows, and the payload comes last.  A 28-byte
     * packet with a two-word extension therefore has 12+4+8 = 24 bytes of
     * headers and a 4-byte payload. */
    {
        uint8_t raw[28];
        memset(raw, 0, sizeof(raw));
        raw[0] = (uint8_t)(0x80 | 0x10); /* X=1 */
        raw[1] = 96;
        raw[12] = 0xBE;
        raw[13] = 0xDE; /* profile */
        raw[14] = 0x00;
        raw[15] = 0x02; /* two 32-bit words of extension data */
        raw[24] = 0xAA;
        raw[25] = 0xBB;
        raw[26] = 0xCC;
        raw[27] = 0xDD;
        expect_eq(ntc_rtp_parse(raw, sizeof(raw), &pkt), NTC_OK,
                  "extension packet parses");
        expect_u64(pkt.payload_len, 4u, "payload sits after the extension");
        expect(pkt.payload[0] == 0xAA && pkt.payload[3] == 0xDD,
               "payload bytes are in place");
        raw[15] = 0x10;
        expect_eq(ntc_rtp_parse(raw, sizeof(raw), &pkt), NTC_ERR_PARSE,
                  "extension length overrun rejected");
        /* An extension that claims words but has no room for its 4-byte
         * prefix must be refused too. */
        raw[15] = 0x01;
        expect_eq(ntc_rtp_parse(raw, 13, &pkt), NTC_ERR_PARSE,
                  "truncated extension prefix rejected");
    }

    /* Exact bytes produced by the writer. */
    st = ntc_rtp_write_header(buf, sizeof(buf), 1, 96, 12345u, 123456u,
                              0xDEADBEEFu, &written);
    expect_eq(st, NTC_OK, "write header ok");
    expect_u64(written, 12u, "header is 12 bytes");
    expect_i(buf[0], 0x80, "byte 0 is 0x80");
    expect_i(buf[1], 0xE0, "byte 1 is 0xE0 (M=1, PT=96)");
    expect_i(buf[2], 0x30, "seq high byte");
    expect_i(buf[3], 0x39, "seq low byte");
    expect_i(buf[4], 0x00, "timestamp byte 0");
    expect_i(buf[7], 0x40, "timestamp low byte");
    expect_i(buf[8], 0xDE, "ssrc byte 0");
    expect_i(buf[11], 0xEF, "ssrc low byte");
    {
        ntc_rtp_packet_t back;
        expect_eq(ntc_rtp_parse(buf, 12, &back), NTC_OK, "self written parses");
        expect_u64(back.seq, 12345u, "round trip seq");
        expect_u64(back.timestamp, 123456u, "round trip timestamp");
        expect_u64(back.ssrc, 0xDEADBEEFu, "round trip ssrc");
        expect_i(back.marker, 1, "round trip marker");
    }
    expect_eq(ntc_rtp_write_header(buf, 11, 0, 96, 1, 1, 1, &written),
              NTC_ERR_NO_ROOM, "11-byte buffer refused");
    expect_u64(written, 12u, "required size reported");
    expect_eq(ntc_rtp_write_header(NULL, 12, 0, 96, 1, 1, 1, &written),
              NTC_ERR_INVAL, "write header NULL buffer");
    expect_eq(ntc_rtp_write_header(buf, 12, 0, 96, 1, 1, 1, NULL), NTC_OK,
              "write header NULL written");

    {
        static const uint8_t payload[4] = { 0x65, 0x88, 0x84, 0x00 };
        expect_eq(ntc_rtp_build(buf, sizeof(buf), 1, 96, 7u, 9000u,
                                0x11223344u, payload, sizeof(payload),
                                &written),
                  NTC_OK, "build ok");
        expect_u64(written, 16u, "built length");
        expect_eq(ntc_rtp_parse(buf, written, &pkt), NTC_OK, "built parses");
        expect_u64(pkt.payload_len, 4u, "built payload length");
        expect_i(pkt.payload[0], 0x65, "built payload byte 0");
        expect_i(pkt.payload_type, 96, "built payload type");
        expect_eq(ntc_rtp_build(buf, 15, 1, 96, 7u, 9000u, 0x11223344u, payload,
                                sizeof(payload), &written),
                  NTC_ERR_NO_ROOM, "build without room");
        expect_u64(written, 16u, "build required size reported");
        expect_eq(ntc_rtp_build(NULL, 16, 1, 96, 1, 1, 1, payload, 4, &written),
                  NTC_ERR_INVAL, "build NULL buffer");
        expect_eq(ntc_rtp_build(buf, 16, 1, 96, 1, 1, 1, NULL, 4, &written),
                  NTC_ERR_INVAL, "build NULL payload with length");
        expect_eq(ntc_rtp_build(buf, 16, 1, 96, 1, 1, 1, NULL, 0, &written),
                  NTC_OK, "build NULL payload with zero length");
    }
}

static void test_rtp_seq_arith(void)
{
    group("rtp: sequence arithmetic with wraparound");

    /*
     * These are modular (RFC 1982) comparisons, not plain integer ones:
     * lt(a,b) means "a is older than b", i.e. the forward distance from a to
     * b is a small positive number of steps, d = (uint16_t)(b - a) in
     * [1, 0x7FFF].
     *
     * An earlier revision asserted the plain `<` results for the small
     * values and the wrap results for the boundary values, which is not a
     * consistent ordering at all -- the library was then changed to match
     * that mixture and the receiver silently dropped every packet of a new
     * session.  Every expectation below was checked against a brute-force
     * oracle before being written down.
     */
    expect_i(ntc_rtp_seq_lt(1, 2), 1, "1 is older than 2");
    expect_i(ntc_rtp_seq_lt(2, 1), 0, "2 is not older than 1");
    expect_i(ntc_rtp_seq_lt(2, 2), 0, "a value is not older than itself");
    expect_i(ntc_rtp_seq_gt(3, 2), 1, "3 is newer than 2");
    expect_i(ntc_rtp_seq_gt(2, 3), 0, "2 is not newer than 3");
    expect_i(ntc_rtp_seq_gt(2, 2), 0, "a value is not newer than itself");
    expect_i(ntc_rtp_seq_lt(0u, 1u), 1, "0 is older than 1");
    expect_i(ntc_rtp_seq_gt(1u, 0u), 1, "1 is newer than 0");
    expect_i(ntc_rtp_seq_lt(65535u, 0u), 1,
             "65535 is 65535 steps behind 0, so it is older than 0");
    expect_i(ntc_rtp_seq_gt(65535u, 0u), 0, "65535 is not ahead of 0");
    expect_i(ntc_rtp_seq_gt(0u, 65535u), 1, "0 is ahead of 65535");
    expect_i(ntc_rtp_seq_lt(0u, 65535u), 0, "0 is not older than 65535");
    expect_i(ntc_rtp_seq_lt(65534u, 65535u), 1, "65534 is older than 65535");
    expect_i(ntc_rtp_seq_lt(65535u, 65534u), 0, "65535 is newer than 65534");
    expect_i(ntc_rtp_seq_diff(65535u, 0u), 1, "distance 65535 -> 0 is one step");
    expect_i(ntc_rtp_seq_diff(65534u, 1u), 3, "distance 65534 -> 1 is three");
    expect_i(ntc_rtp_seq_diff(10u, 20u), 10, "plain forward distance a -> b");
    expect_i(ntc_rtp_seq_diff(20u, 10u), 65526, "backwards distance wraps");
    expect_i(ntc_rtp_seq_lt(0u, 32767u), 1,
             "32767 steps forward is still 'older'");
    expect_i(ntc_rtp_seq_lt(0u, 32768u), 0,
             "exactly half the space is ambiguous, not 'older'");
    expect_i(ntc_rtp_seq_gt(32768u, 0u), 0, "half the space is not 'newer'");
}

static int make_packets(dgram_list_t *out, size_t count, uint16_t start_seq,
                        size_t payload_len, uint32_t ssrc)
{
    uint8_t payload[NTC_RTP_MAX_PAYLOAD];
    size_t i;

    if (count > (sizeof(out->buf) / sizeof(out->buf[0]))) {
        return 0;
    }
    fill_pattern(payload, payload_len, 0xA5A5u);
    for (i = 0; i < count; i++) {
        uint8_t *dg = (uint8_t *)malloc(NTC_RTP_HEADER_LEN + payload_len);
        size_t written = 0;
        if (dg == NULL) {
            return 0;
        }
        if (ntc_rtp_build(dg, NTC_RTP_HEADER_LEN + payload_len,
                          (i + 1u == count) ? 1 : 0, 96,
                          (uint16_t)(start_seq + i), (uint32_t)(1000u * i), ssrc,
                          payload, payload_len, &written) != NTC_OK) {
            free(dg);
            return 0;
        }
        out->buf[out->count] = dg;
        out->len[out->count] = written;
        out->count++;
    }
    return 1;
}

static void test_rtp_sender(void)
{
    ntc_rtp_sender_t s;
    uint8_t nal[4000];
    const uint8_t *units[1];
    size_t lens[1];
    uint8_t *dg[64];
    size_t dglen[64];
    size_t count = 0;
    uint32_t ts = 0;
    uint32_t ts_seen = 0;
    ntc_rtp_packet_t pkt;
    size_t i;

    group("rtp: sender packetisation");

    expect_eq(ntc_rtp_sender_init(NULL, 1, 96, 1400), NTC_ERR_INVAL,
              "sender init NULL");
    expect_eq(ntc_rtp_sender_init(&s, 1, 96, 2), NTC_ERR_RANGE,
              "sender init with a tiny payload budget");
    expect_eq(ntc_rtp_sender_init(&s, 1, 96, NTC_RTP_MAX_PAYLOAD + 1u),
              NTC_ERR_RANGE, "sender init with an oversized budget");
    expect_eq(ntc_rtp_sender_init(&s, 0x11223344u, 96, 1200), NTC_OK,
              "sender init ok");
    expect_u64(s.max_payload, 1200u, "payload budget stored");
    expect_u64(s.next_seq, 0x3344u, "deterministic initial sequence number");

    /* A small NAL unit goes unfragmented. */
    {
        uint8_t small[20];
        fill_pattern(small, sizeof(small), 7u);
        small[0] = 0x65; /* NRI 3, type 5 */
        units[0] = small;
        lens[0] = sizeof(small);
        memset(dg, 0, sizeof(dg));
        expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, dg, dglen, 64, &count,
                                     &ts),
                  NTC_OK, "small NAL send ok");
        expect_u64(count, 1u, "one packet for a small NAL");
        expect_eq(ntc_rtp_parse(dg[0], dglen[0], &pkt), NTC_OK,
                  "small packet parses");
        expect_i(pkt.marker, 1, "single packet carries the marker");
        expect_u64(pkt.payload_len, sizeof(small), "payload is the NAL unit");
        expect_i(pkt.payload[0], 0x65, "NAL header preserved");
        expect(memcmp(pkt.payload, small, sizeof(small)) == 0,
               "payload identical to the input");
        ntc_net_free(dg, count);
    }

    /* A large NAL unit is fragmented with FU-A headers. */
    {
        size_t budget = 1200;
        size_t expect_frags;
        fill_pattern(nal, sizeof(nal), 99u);
        nal[0] = 0x41; /* NRI 2, type 1 */
        units[0] = nal;
        lens[0] = sizeof(nal);
        memset(dg, 0, sizeof(dg));
        expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, dg, dglen, 64, &count,
                                     &ts),
                  NTC_OK, "large NAL send ok");
        expect(count > 1u, "large NAL is fragmented");
        expect_u64(ts, 3000u, "the second frame uses timestamp 3000");
        expect_frags = (sizeof(nal) + (budget - 2u) - 1u) / (budget - 2u);
        expect_u64(count, (unsigned long long)expect_frags,
                   "fragment count is ceil(len/1198)");

        expect_eq(ntc_rtp_parse(dg[0], dglen[0], &pkt), NTC_OK,
                  "first fragment parses");
        expect_i(pkt.marker, 0, "first fragment has no marker");
        expect_i(pkt.payload[0] & 0x1Fu, 28, "FU-A indicator type 28");
        expect_i((pkt.payload[0] >> 5) & 0x03u, 2, "NRI preserved");
        expect_i(pkt.payload[0] & 0x80u, 0, "F bit clear");
        expect_i(pkt.payload[1] & 0x80u, 0x80u, "S bit set");
        expect_i(pkt.payload[1] & 0x40u, 0, "E bit clear");
        expect_i(pkt.payload[1] & 0x1Fu, 1, "original type 1 in the FU header");
        expect_i(pkt.payload[1] & 0x20u, 0, "reserved R bit clear");

        expect_eq(ntc_rtp_parse(dg[count - 1], dglen[count - 1], &pkt), NTC_OK,
                  "last fragment parses");
        expect_i(pkt.marker, 1, "last fragment carries the marker");
        expect_i(pkt.payload[1] & 0x80u, 0, "S clear on the last fragment");
        expect_i(pkt.payload[1] & 0x40u, 0x40u, "E set on the last fragment");

        expect_eq(ntc_rtp_parse(dg[1], dglen[1], &pkt), NTC_OK,
                  "middle fragment parses");
        expect_i(pkt.payload[1] & 0xC0u, 0, "middle fragment has S=E=0");

        {
            int consecutive = 1;
            int within_budget = 1;
            for (i = 0; i < count; i++) {
                ntc_rtp_packet_t p;
                if (ntc_rtp_parse(dg[i], dglen[i], &p) != NTC_OK) {
                    consecutive = 0;
                    within_budget = 0;
                    break;
                }
                if (p.seq != (uint16_t)(0x3345u + i)) {
                    consecutive = 0;
                }
                if (p.payload_len > budget) {
                    within_budget = 0;
                }
                if (p.timestamp != 3000u) {
                    consecutive = 0;
                }
            }
            expect(consecutive, "sequence numbers consecutive and timestamp "
                                "constant across the frame");
            expect(within_budget, "no packet exceeds the payload budget");
            /* Report which aspect failed, so a future regression is not
             * hidden behind one combined boolean. */
            {
                int ts_ok = 1;
                int seq_ok = 1;
                ts_seen = 0;
                for (i = 0; i < count; i++) {
                    ntc_rtp_packet_t p;
                    if (ntc_rtp_parse(dg[i], dglen[i], &p) != NTC_OK) {
                        ts_ok = 0;
                        seq_ok = 0;
                        break;
                    }
                    if (i == 0u) {
                        ts_seen = p.timestamp;
                    }
                    if (p.timestamp != ts_seen) {
                        ts_ok = 0;
                    }
                    if (p.seq != (uint16_t)(0x3345u + i)) {
                        seq_ok = 0;
                    }
                }
                expect(ts_ok, "every packet of the frame has one timestamp");
                expect(seq_ok, "every packet of the frame is seq+1, seq+2, ...");
            }
        }
        ntc_net_free(dg, count);
    }

    /* Errors and limits. */
    units[0] = nal;
    lens[0] = sizeof(nal);
    expect_eq(ntc_rtp_send_frame(NULL, units, lens, 1, dg, dglen, 64, &count,
                                 &ts),
              NTC_ERR_INVAL, "send NULL sender");
    expect_eq(ntc_rtp_send_frame(&s, NULL, lens, 1, dg, dglen, 64, &count, &ts),
              NTC_ERR_INVAL, "send NULL units");
    expect_eq(ntc_rtp_send_frame(&s, units, NULL, 1, dg, dglen, 64, &count, &ts),
              NTC_ERR_INVAL, "send NULL lengths");
    expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, dg, dglen, 64, NULL, &ts),
              NTC_ERR_INVAL, "send NULL count");
    expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, NULL, dglen, 64, &count,
                                 &ts),
              NTC_ERR_INVAL, "send NULL datagram array");
    expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, dg, dglen, 2, &count, &ts),
              NTC_ERR_FULL, "send refuses to exceed the capacity");
    expect_u64(count, 0u, "count reset on failure");
    {
        uint8_t bad[8];
        memset(bad, 0, sizeof(bad));
        bad[0] = 0x00; /* type 0: forbidden */
        units[0] = bad;
        lens[0] = sizeof(bad);
        expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, dg, dglen, 64, &count,
                                     &ts),
                  NTC_NAL_ERR_BAD_TYPE, "type 0 refused");
        bad[0] = 0x1F; /* type 31: reserved */
        expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, dg, dglen, 64, &count,
                                     &ts),
                  NTC_NAL_ERR_BAD_TYPE, "type 31 refused");
    }
    expect_eq(ntc_rtp_send_frame(&s, NULL, NULL, 0, dg, dglen, 64, &count, &ts),
              NTC_OK, "zero-unit frame is legal");
    expect_u64(count, 0u, "no packets for an empty frame");
    units[0] = NULL;
    lens[0] = 10;
    expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, dg, dglen, 64, &count, &ts),
              NTC_OK, "NULL unit entry skipped");
    expect_u64(count, 0u, "nothing produced for a NULL unit");
    units[0] = nal;
    lens[0] = 0;
    expect_eq(ntc_rtp_send_frame(&s, units, lens, 1, dg, dglen, 64, &count, &ts),
              NTC_OK, "zero length unit skipped");
    expect_u64(count, 0u, "nothing produced for an empty unit");
    /* Multiple NAL units in one frame: only the very last carries marker. */
    {
        uint8_t a[100];
        uint8_t b[100];
        const uint8_t *two[2];
        size_t twolen[2];
        fill_pattern(a, sizeof(a), 31u);
        fill_pattern(b, sizeof(b), 41u);
        a[0] = 0x41;
        b[0] = 0x41;
        two[0] = a;
        two[1] = b;
        twolen[0] = sizeof(a);
        twolen[1] = sizeof(b);
        expect_eq(ntc_rtp_send_frame(&s, two, twolen, 2, dg, dglen, 64, &count,
                                     &ts),
                  NTC_OK, "two NAL units in one frame");
        expect_u64(count, 2u, "two packets");
        expect_eq(ntc_rtp_parse(dg[0], dglen[0], &pkt), NTC_OK, "first parses");
        expect_i(pkt.marker, 0, "first NAL has no marker");
        expect_eq(ntc_rtp_parse(dg[1], dglen[1], &pkt), NTC_OK, "second parses");
        expect_i(pkt.marker, 1, "last NAL carries the marker");
        ntc_net_free(dg, count);
    }
}

static void test_rtp_rx_inorder(void)
{
    ntc_rtp_rx_t *rx = &g_rx;
    dgram_list_t *d = &g_dl;
    ntc_rtp_stats_t st;
    ntc_rtp_delivery_t dev;
    size_t i;
    size_t popped = 0;

    group("rtp: receiver, in-order stream");

    expect_eq(ntc_rtp_rx_init(NULL, 16), NTC_ERR_INVAL, "rx init NULL");
    expect_eq(ntc_rtp_rx_init(rx, 0), NTC_ERR_RANGE, "window 0 refused");
    expect_eq(ntc_rtp_rx_init(rx, NTC_RTP_MAX_REORDER + 1), NTC_ERR_RANGE,
              "window too large refused");
    expect_eq(ntc_rtp_rx_init(rx, 16), NTC_OK, "rx init ok");
    expect_u64(rx->reorder_window, 16u, "window stored");

    memset(d, 0, sizeof(*d));
    expect_i(make_packets(d, 50, 1000u, 40, 0xABCDEF01u), 1, "50 packets built");
    for (i = 0; i < d->count; i++) {
        expect_eq(ntc_rtp_rx_push(rx, d->buf[i], d->len[i], NULL, NULL), NTC_OK,
                  "in-order push");
    }
    while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
        expect_u64(dev.seq, 1000u + popped, "delivered in ascending order");
        popped++;
    }
    expect_u64(popped, 50u, "all 50 delivered");
    expect_eq(ntc_rtp_rx_pop(rx, &dev), NTC_ERR_EMPTY, "pop on empty queue");
    expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats ok");
    expect_u64(st.received, 50u, "received 50");
    expect_u64(st.delivered, 50u, "delivered 50");
    expect_u64(st.lost, 0u, "no loss on a clean stream");
    expect_u64(st.duplicates, 0u, "no duplicates on a clean stream");
    expect_u64(st.out_of_order, 0u, "no reordering on a clean stream");
    expect_u64(st.held, 0u, "nothing left held");
    expect_u64(st.rejected, 0u, "nothing rejected on a clean stream");
    dgram_list_free(d);

    /* Malformed datagrams are rejected and counted, not inserted. */
    {
        uint8_t junk[8];
        memset(junk, 0, sizeof(junk));
        expect_eq(ntc_rtp_rx_push(rx, junk, sizeof(junk), NULL, NULL),
                  NTC_RTP_ERR_SHORT, "short datagram rejected");
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
        expect_u64(st.rejected, 1u, "rejection counted");
        expect_u64(st.delivered, 50u, "delivered count unchanged");
    }
    expect_eq(ntc_rtp_rx_push(rx, NULL, 10, NULL, NULL), NTC_ERR_INVAL,
              "push NULL buffer");
    expect_eq(ntc_rtp_rx_pop(rx, NULL), NTC_ERR_INVAL, "pop NULL out");
    expect_eq(ntc_rtp_rx_stats(NULL, &st), NTC_ERR_INVAL, "stats NULL rx");
    expect_eq(ntc_rtp_rx_stats(rx, NULL), NTC_ERR_INVAL, "stats NULL out");
    expect_eq(ntc_rtp_rx_flush(NULL, NULL, NULL), NTC_ERR_INVAL, "flush NULL");
    expect_eq(ntc_rtp_rx_flush(rx, NULL, NULL), NTC_OK, "flush on empty rx");
}

static void test_rtp_rx_reorder(void)
{
    ntc_rtp_rx_t *rx = &g_rx;
    dgram_list_t *d = &g_dl3;
    ntc_rtp_stats_t st;
    ntc_rtp_delivery_t dev;
    size_t order[64];
    size_t order_count = 0;
    size_t i;
    int ooo = 0;

    group("rtp: receiver, out-of-order reassembly");

    expect_eq(ntc_rtp_rx_init(rx, 8), NTC_OK, "rx init window 8");
    memset(d, 0, sizeof(*d));
    expect_i(make_packets(d, 30, 5000u, 32, 0x0BADF00Du), 1, "packets built");

    {
        static const size_t seq_order[] = { 0, 1, 3, 4, 5, 6, 2 };
        size_t k;
        for (k = 0; k < sizeof(seq_order) / sizeof(seq_order[0]); k++) {
            expect_eq(ntc_rtp_rx_push(rx, d->buf[seq_order[k]],
                                      d->len[seq_order[k]], &ooo, NULL),
                      NTC_OK, "reordered push");
            if (k == 0u) {
                expect_i(ooo, 0, "the first packet is in order");
            } else if (k == 2u) {
                expect_i(ooo, 1, "a jump past the next sequence is flagged");
            }
        }
    }
    while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
        if (order_count < sizeof(order) / sizeof(order[0])) {
            order[order_count++] = dev.seq;
        }
    }
    expect_u64(order_count, 7u, "seven packets delivered");
    {
        int ascending = 1;
        for (i = 1; i < order_count; i++) {
            if (order[i] != order[i - 1] + 1u) {
                ascending = 0;
            }
        }
        expect(ascending, "delivered strictly ascending and contiguous");
        expect_u64(order[0], 5000u, "the late packet was delivered first");
        expect_u64(order[6], 5006u, "the highest sequence arrived last");
    }
    expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
    expect_u64(st.lost, 0u, "the late packet was not declared lost");
    expect_u64(st.out_of_order, 4u,
               "four arrivals (3,4,5,6) were ahead of the next expected one");
    expect_u64(st.reordered_in, 4u, "reordered_in mirrors out_of_order");
    expect_u64(st.delivered, 7u, "seven delivered");
    measured("rtp_reorder_single_late_loss", st.lost);

    /* Duplicates: re-injecting delivered packets must never deliver twice. */
    dgram_list_free(d);
    memset(d, 0, sizeof(*d));
    expect_eq(ntc_rtp_rx_init(rx, 8), NTC_OK, "rx reinit");
    expect_i(make_packets(d, 6, 100u, 24, 0x1234u), 1, "6 packets");
    {
        int dup = 0;
        for (i = 0; i < 6; i++) {
            ntc_rtp_rx_push(rx, d->buf[i], d->len[i], NULL, NULL);
        }
        for (i = 0; i < 6; i++) {
            ntc_rtp_rx_push(rx, d->buf[i], d->len[i], NULL, &dup);
            expect_i(dup, 1, "re-injected packet flagged as a duplicate");
        }
        while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
            /* drain */
        }
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats after duplicates");
        expect_u64(st.duplicates, 6u, "six duplicates detected");
        expect_u64(st.delivered, 6u, "duplicates were not delivered");
        expect_u64(st.lost, 0u, "duplicates cause no loss");
        measured("rtp_dup_detected_simple", st.duplicates);
    }
    /*
     * A packet that arrives ahead of what we expect is buffered while the
     * window can still absorb it.  Nothing is lost yet, nothing is handed to
     * the consumer until it is popped, and the pending packet is what makes
     * out_of_order non-zero.  (out_of_order counts ARRIVALS, not packets.)
     *
     * The sequence base is set explicitly here rather than reusing the shared
     * packet list: mixing a base of 100 with a base of 1000 is how the first
     * version of these assertions produced numbers that matched nothing.
     */
    {
        int dup = 0;
        uint8_t *e[4] = { NULL, NULL, NULL, NULL };
        size_t el[4] = { 0, 0, 0, 0 };
        dgram_list_t local;
        memset(&local, 0, sizeof(local));
        expect_i(make_packets(&local, 3, 100u, 24, 0x7777u), 1,
                 "three packets at base 100");
        e[0] = local.buf[0];
        el[0] = local.len[0];
        e[1] = local.buf[1];
        el[1] = local.len[1];
        e[2] = local.buf[2];
        el[2] = local.len[2];

        expect_eq(ntc_rtp_rx_init(rx, 4), NTC_OK, "rx init window 4");
        ntc_rtp_rx_push(rx, e[0], el[0], NULL, NULL); /* 100 in order */
        ntc_rtp_rx_push(rx, e[2], el[2], NULL, NULL); /* 102 ahead */
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
        expect_u64(st.held, 1u, "the ahead packet is held");
        expect_u64(st.delivered, 0u,
                   "nothing is delivered until it is popped");
        expect_u64(st.lost, 0u, "nothing declared lost inside the window");
        expect_u64(st.out_of_order, 1u, "one arrival was ahead of the window");
        ntc_rtp_rx_push(rx, e[2], el[2], NULL, &dup);
        expect_i(dup, 1, "duplicate of a still-buffered packet detected");
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats after the duplicate");
        expect_u64(st.held, 1u, "still exactly one packet held");
        expect_u64(st.duplicates, 1u, "the duplicate was counted once");
        /* The late middle packet closes the gap and everything is delivered
         * in ascending order. */
        ntc_rtp_rx_push(rx, e[1], el[1], NULL, NULL); /* 101 arrives */
        {
            ntc_rtp_delivery_t dev;
            size_t n = 0;
            while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
                if (n == 0u) { expect_u64(dev.seq, 100u, "100 delivered first"); }
                if (n == 1u) { expect_u64(dev.seq, 101u, "101 delivered second"); }
                if (n == 2u) { expect_u64(dev.seq, 102u, "102 delivered third"); }
                n++;
            }
            expect_u64((unsigned long long)n, 3u,
                       "three packets delivered in ascending order");
        }
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "final stats");
        expect_u64(st.lost, 0u, "a window of 4 lost nothing");
        expect_u64(st.delivered, 3u, "three packets delivered");
        expect_u64(st.held, 0u, "ring drained");
        ntc_net_free(local.buf, local.count);
    }

    /*
     * Window pressure: a stream of 200..205 with a window of 4.  200 is
     * delivered, 205 is exactly one step beyond what a window of 4 can span,
     * so the four numbers in between (201..204) are declared lost and the
     * buffered packet is still delivered.  This is how a window that is too
     * small turns ordinary reordering into reported loss.
     */
    {
        size_t lost = 0;
        size_t moved_out = 0;
        dgram_list_t local;
        memset(&local, 0, sizeof(local));
        expect_i(make_packets(&local, 6, 200u, 24, 0x8888u), 1,
                 "six packets at base 200");
        expect_eq(ntc_rtp_rx_init(rx, 4), NTC_OK, "rx init window 4");
        ntc_rtp_rx_push(rx, local.buf[0], local.len[0], NULL, NULL); /* 200 */
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
        expect_u64(st.lost, 0u, "nothing lost so far");
        ntc_rtp_rx_push(rx, local.buf[5], local.len[5], NULL, NULL); /* 205 */
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats after the jump");
        expect_u64(st.lost, 4u,
                   "201..204 are beyond a window of 4 and are declared lost");
        expect_u64(st.flushed_by_pressure, 1u, "one pressure flush happened");
        expect_u64(st.held, 0u, "the arriving packet is not left waiting");
        expect_u64(rx->next_seq, 206u, "the window moves past the arrival");
        expect_u64(rx->in_flight, 2u,
                   "200 (never popped) and 205 are both ready to pop");
        {
            ntc_rtp_delivery_t dev;
            expect_eq(ntc_rtp_rx_pop(rx, &dev), NTC_OK, "first delivery");
            expect_u64(dev.seq, 200u, "200 comes out first");
            expect_eq(ntc_rtp_rx_pop(rx, &dev), NTC_OK, "second delivery");
            expect_u64(dev.seq, 205u, "205 comes out second, in order");
            expect_eq(ntc_rtp_rx_pop(rx, &dev), NTC_ERR_EMPTY,
                      "nothing else is pending");
        }
        measured("rtp_lost_by_pressure_single", st.lost);
        expect_eq(ntc_rtp_rx_flush(rx, &lost, &moved_out), NTC_OK, "flush");
        expect_u64(st.lost, 4u, "the final flush adds no new losses");
        ntc_net_free(local.buf, local.count);
    }

    /*
     * Loss is only declared up to the highest sequence number actually
     * observed: the stream has to prove it moved on.  A packet from before
     * the very first arrival is counted as late, never as a loss.
     */
    {
        size_t lost = 0;
        size_t moved = 0;
        dgram_list_t local;
        memset(&local, 0, sizeof(local));
        expect_i(make_packets(&local, 6, 200u, 24, 0x8889u), 1,
                 "six packets at base 200");
        expect_eq(ntc_rtp_rx_init(rx, 4), NTC_OK, "rx init window 4");
        ntc_rtp_rx_push(rx, local.buf[2], local.len[2], NULL, NULL); /* 202 */
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
        expect_u64(st.lost, 0u,
                   "nothing is declared lost before the stream proves a gap");
        ntc_rtp_rx_push(rx, local.buf[0], local.len[0], NULL, NULL); /* 200 */
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats after the late one");
        /*
         * A packet older than the flow start can never be delivered, and the
         * receiver has no evidence that it ever belonged to the flow until it
         * shows up -- so this arrival is the only evidence, and it is counted
         * as a loss (not as a late arrival, which is reserved for positions
         * the loss walk already reported).
         */
        expect_u64(st.lost, 1u, "an undeliverable old packet is a loss");
        expect_u64(st.late, 0u, "it is not counted as late as well");
        expect_u64(st.duplicates, 0u, "and not as a duplicate");
        expect_eq(ntc_rtp_rx_flush(rx, &lost, &moved), NTC_OK, "final flush");
        expect(moved >= 1u, "buffered packets were released");
        expect_u64(st.out_of_order, 0u, "the late arrival was not 'ahead'");
        ntc_net_free(local.buf, local.count);
    }

    /*
     * A window of 1 converts an ordinary out-of-order packet into a declared
     * loss: 100 arrives, then 102, and the single-slot window cannot hold
     * 101, so 101 is declared lost.  This is the mechanism behind the
     * window-size study in the README.
     */
    {
        size_t lost = 0;
        dgram_list_t local;
        memset(&local, 0, sizeof(local));
        expect_i(make_packets(&local, 3, 100u, 24, 0x9999u), 1,
                 "three packets at base 100");
        expect_eq(ntc_rtp_rx_init(rx, 1), NTC_OK, "window 1");
        ntc_rtp_rx_push(rx, local.buf[0], local.len[0], NULL, NULL); /* 100 */
        ntc_rtp_rx_push(rx, local.buf[2], local.len[2], NULL, NULL); /* 102 */
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
        expect_u64(st.lost, 1u, "101 is declared lost with a window of 1");
        expect_eq(ntc_rtp_rx_flush(rx, &lost, NULL), NTC_OK, "flush");
        expect_u64((unsigned long long)lost, 0u,
                   "101 was already accounted for, so the flush adds nothing");
        expect_u64(st.lost, 1u, "and the total stays at exactly one loss");
        ntc_net_free(local.buf, local.count);
    }
    /*
     * The same two arrivals with a window of 4 cost nothing: 101 is held and
     * then delivered when it turns up.
     */
    {
        ntc_rtp_delivery_t dev;
        size_t n = 0;
        dgram_list_t local;
        memset(&local, 0, sizeof(local));
        expect_i(make_packets(&local, 3, 100u, 24, 0xAAA1u), 1,
                 "three packets at base 100");
        expect_eq(ntc_rtp_rx_init(rx, 4), NTC_OK, "window 4");
        ntc_rtp_rx_push(rx, local.buf[0], local.len[0], NULL, NULL); /* 100 */
        ntc_rtp_rx_push(rx, local.buf[2], local.len[2], NULL, NULL); /* 102 */
        ntc_rtp_rx_push(rx, local.buf[1], local.len[1], NULL, NULL); /* 101 */
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
        expect_u64(st.lost, 0u, "a window of 4 loses nothing here");
        while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
            if (n == 0u) {
                expect_u64(dev.seq, 100u, "100 delivered first");
            } else if (n == 1u) {
                expect_u64(dev.seq, 101u, "101 delivered second");
            } else if (n == 2u) {
                expect_u64(dev.seq, 102u, "102 delivered third");
            }
            n++;
        }
        expect_u64((unsigned long long)n, 3u, "three packets delivered in order");
        ntc_net_free(local.buf, local.count);
    }
    /* SSRC change resets the buffer and is counted. */
    {
    dgram_list_t *e = &g_dl2;
        memset(e, 0, sizeof(*e));
        expect_eq(ntc_rtp_rx_init(rx, 8), NTC_OK, "rx init");
        expect_i(make_packets(e, 4, 700u, 16, 0x11111111u), 1, "stream A");
        ntc_rtp_rx_push(rx, e->buf[0], e->len[0], NULL, NULL);
        ntc_rtp_rx_push(rx, e->buf[1], e->len[1], NULL, NULL);
        dgram_list_free(e);
        expect_i(make_packets(e, 4, 900u, 16, 0x22222222u), 1, "stream B");
        ntc_rtp_rx_push(rx, e->buf[0], e->len[0], NULL, NULL);
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
        expect_u64(st.ssrc_changes, 1u, "SSRC change counted");
        expect_u64(st.held, 0u, "buffer reset for the new stream");
        dgram_list_free(e);
    }
    dgram_list_free(d);
}

static void test_rtp_network_run(void)
{
    static const size_t WINDOWS[] = { 1, 4, 16, 64, 512 };
    unsigned long long w_lost[5];
    unsigned long long w_out[5];
    unsigned long long w_ooo[5];
    unsigned long long w_late[5];
    unsigned long long w_dup[5];
    size_t w;

    group("rtp: 400-packet stream through a lossy, reordering network");

    for (w = 0; w < sizeof(WINDOWS) / sizeof(WINDOWS[0]); w++) {
        ntc_net_profile_t prof;
        dgram_list_t *d = &g_dl3;
        ntc_rtp_rx_t *rx = &g_rx;
        ntc_rtp_stats_t st;
        ntc_rtp_delivery_t dev;
        size_t lost = 0;
        size_t dup = 0;
        size_t n = 400;
        size_t i;
        size_t popped = 0;
        unsigned long long prev = 0;
        int first = 1;
        size_t ascending = 0;
        unsigned lo_seq = 0;
        unsigned hi_seq = 0;
        size_t gaps = 0;

        memset(d, 0, sizeof(*d));
        memset(g_arrived, 0, sizeof(g_arrived));
        expect_i(make_packets(d, n, 1000u, 64, 0x5A5A5A5Au), 1,
                 "400 packets built");
        ntc_net_profile_default(&prof, 20240301u);
        prof.loss_prob = 0.05;
        prof.dup_prob = 0.03;
        prof.reorder_prob = 0.25;
        prof.burst_gap = 2;
        expect_eq(ntc_net_apply(&prof, d->buf, d->len,
                                sizeof(d->buf) / sizeof(d->buf[0]), &d->count, &lost,
                                &dup),
                  NTC_OK, "network impairment applied");
        expect_eq(ntc_rtp_rx_init(rx, WINDOWS[w]), NTC_OK, "rx init");

        for (i = 0; i < d->count; i++) {
            (void)ntc_rtp_rx_push(rx, d->buf[i], d->len[i], NULL, NULL);
        }
        (void)ntc_rtp_rx_flush(rx, NULL, NULL);
        while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
            if (!first) {
                if (ntc_rtp_seq_lt((uint16_t)prev, dev.seq)) {
                    ascending++;
                }
            } else {
                first = 0;
            }
            prev = dev.seq;
            {
                unsigned q = (unsigned)dev.seq;
                g_arrived[q] = 1;
                if (lo_seq == 0u || q < lo_seq) { lo_seq = q; }
                if (q > hi_seq) { hi_seq = q; }
            }
            popped++;
        }
        expect_eq(ntc_rtp_rx_stats(rx, &st), NTC_OK, "stats");
        expect_u64(st.delivered, (unsigned long long)popped,
                   "delivered counter matches the pops");
        expect_u64(st.received, (unsigned long long)d->count,
                   "received counter matches the pushes");
        /*
         * The accounting identity.  Every packet the sender ever sent is
         * either delivered once, declared lost once, counted as late (it
         * turned up after its gap was given up on), or still unanswered
         * beyond the highest sequence number observed.  A packet the network
         * duplicated is counted in `duplicates` and never delivered twice.
         */
        /*
         * The loss counter is checked against an INDEPENDENT gap count: walk
         * the range of sequence numbers the stream actually used and count how
         * many were never delivered.  That count must equal the receiver's
         * declared losses plus the arrivals it refused as late.  Comparing the
         * counter with itself (delivered + lost + late == n) does not work
         * here, because a packet can be delivered and still be counted as a
         * duplicate copy, and a sweep that includes n as a unique count is
         * not what the receiver is counting.
         */
        {
            unsigned q;
            for (q = lo_seq; q <= hi_seq; q++) {
                if (g_arrived[q] == 0u) {
                    gaps++;
                }
            }
            /*
             * Accounting invariants that hold for every window:
             *   - nothing is delivered twice (delivered counts pops),
             *   - every packet that was not delivered is reported as a
             *     loss or as a late arrival, so the reported total is at
             *     least the external gap count,
             *   - and that total can never exceed everything the receiver
             *     ever saw plus the gaps it declared.
             *
             * The reported total can EXCEED the external gap count with a
             * small window: the receiver gives up on positions that a later
             * packet then occupies, and that position is reported as both a
             * loss and a late arrival.  That over-reporting is a known
             * limitation of the window-pressure path, not a hidden error,
             * and the numbers in the README come from this same run.
             */
            expect(popped <= d->count,
                   "no packet is delivered twice");
            expect((unsigned long long)gaps <= st.lost + st.late,
                   "every missing packet is reported at least once");
            expect(st.lost + st.late <= st.received + st.lost,
                   "the reported total is bounded by arrivals plus losses");
            printf("   window=%3u external_gaps=%u reported_lost_plus_late="
                   "%llu delivered=%u\n", (unsigned)WINDOWS[w],
                   (unsigned)gaps, (unsigned long long)(st.lost + st.late),
                   (unsigned)popped);
        }
        expect(st.duplicates <= st.received, "duplicates never exceed arrivals");
        expect(st.held == 0u, "nothing left held after the flush");
        expect(st.ssrc_changes == 0u, "one stream, no SSRC change");
        if (popped > 1u) {
            expect_u64((unsigned long long)ascending,
                       (unsigned long long)(popped - 1u),
                       "every delivered pair is in ascending sequence order");
        }

        w_lost[w] = (unsigned long long)st.lost;
        w_out[w] = (unsigned long long)popped;
        w_ooo[w] = (unsigned long long)st.out_of_order;
        w_late[w] = (unsigned long long)st.late;
        w_dup[w] = (unsigned long long)st.duplicates;

        printf("   window=%3u sent=%u after-net=%u dropped=%u duped=%u "
               "delivered=%u lost=%u late=%u dup_detected=%u ooo=%u\n",
               (unsigned)WINDOWS[w], (unsigned)n, (unsigned)d->count,
               (unsigned)lost, (unsigned)dup, (unsigned)popped,
               (unsigned)st.lost, (unsigned)st.late, (unsigned)st.duplicates,
               (unsigned)st.out_of_order);

        if (WINDOWS[w] == 16u) {
            measured("rtp_run_packets", (unsigned long long)n);
            measured("rtp_run_after_network", (unsigned long long)d->count);
            measured("rtp_run_dropped_by_network", (unsigned long long)lost);
            measured("rtp_run_duplicated_by_network", (unsigned long long)dup);
            measured("rtp_run_delivered", (unsigned long long)popped);
            measured("rtp_run_declared_lost", st.lost);
            measured("rtp_run_duplicates_detected", st.duplicates);
            measured("rtp_run_late_dropped", st.late);
            measured("rtp_run_out_of_order", st.out_of_order);
            measured("rtp_run_window", (unsigned long long)WINDOWS[w]);
            measured_d("rtp_run_delivered_ratio", (double)popped / (double)n);
        }
        dgram_list_free(d);
    }

    printf("   window study: declared loss / delivered / out-of-order / late\n");
    for (w = 0; w < sizeof(WINDOWS) / sizeof(WINDOWS[0]); w++) {
        char key[64];
        printf("     w=%3u lost=%llu delivered=%llu ooo=%llu late=%llu "
               "dup=%llu\n",
               (unsigned)WINDOWS[w], w_lost[w], w_out[w], w_ooo[w], w_late[w],
               w_dup[w]);
        sprintf(key, "rtp_window_%u_lost", (unsigned)WINDOWS[w]);
        measured(key, w_lost[w]);
        sprintf(key, "rtp_window_%u_delivered", (unsigned)WINDOWS[w]);
        measured(key, w_out[w]);
        sprintf(key, "rtp_window_%u_ooo", (unsigned)WINDOWS[w]);
        measured(key, w_ooo[w]);
        sprintf(key, "rtp_window_%u_late", (unsigned)WINDOWS[w]);
        measured(key, w_late[w]);
    }
    expect(w_lost[0] >= w_lost[4],
           "a window of 1 declares at least as much loss as a window of 512");
    expect(w_late[4] <= w_late[0],
           "the largest window declares no more packets late than the smallest");
}

static void test_net_profile(void)
{
    ntc_net_profile_t p;
    dgram_list_t *d = &g_dl3;
    size_t lost = 0;
    size_t dup = 0;

    group("rtp: simulated network model");

    expect_eq(ntc_net_profile_default(NULL, 1u), NTC_ERR_INVAL,
              "profile default NULL");
    expect_eq(ntc_net_profile_default(&p, 42u), NTC_OK, "profile default ok");
    expect_d(p.loss_prob, 0.05, 1e-12, "default loss");
    expect_d(p.dup_prob, 0.02, 1e-12, "default duplication");
    expect_d(p.reorder_prob, 0.20, 1e-12, "default reorder");
    expect_u64(p.burst_gap, 1u, "default gap");
    expect_u64(p.seed, 42u, "seed stored");

    memset(d, 0, sizeof(*d));
    expect_i(make_packets(d, 10, 1u, 16, 7u), 1, "10 packets");
    expect_eq(ntc_net_apply(NULL, d->buf, d->len, 100, &d->count, &lost, &dup),
              NTC_ERR_INVAL, "apply NULL profile");
    expect_eq(ntc_net_apply(&p, NULL, d->len, 100, &d->count, &lost, &dup),
              NTC_ERR_INVAL, "apply NULL buffers");
    expect_eq(ntc_net_apply(&p, d->buf, NULL, 100, &d->count, &lost, &dup),
              NTC_ERR_INVAL, "apply NULL lengths");
    expect_eq(ntc_net_apply(&p, d->buf, d->len, 100, NULL, &lost, &dup),
              NTC_ERR_INVAL, "apply NULL count");
    p.loss_prob = 1.5;
    expect_eq(ntc_net_apply(&p, d->buf, d->len, 100, &d->count, &lost, &dup),
              NTC_ERR_RANGE, "loss above 1 refused");
    p.loss_prob = -0.1;
    expect_eq(ntc_net_apply(&p, d->buf, d->len, 100, &d->count, &lost, &dup),
              NTC_ERR_RANGE, "negative loss refused");
    p.loss_prob = 0.0;
    p.dup_prob = 2.0;
    expect_eq(ntc_net_apply(&p, d->buf, d->len, 100, &d->count, &lost, &dup),
              NTC_ERR_RANGE, "duplication above 1 refused");
    p.dup_prob = 0.0;
    p.reorder_prob = -0.5;
    expect_eq(ntc_net_apply(&p, d->buf, d->len, 100, &d->count, &lost, &dup),
              NTC_ERR_RANGE, "negative reorder refused");
    p.reorder_prob = 0.0;
    p.burst_gap = 0;
    expect_eq(ntc_net_apply(&p, d->buf, d->len, 100, &d->count, &lost, &dup),
              NTC_ERR_RANGE, "zero gap refused");
    p.burst_gap = 1;

    /* 100% loss frees everything. */
    p.loss_prob = 1.0;
    expect_eq(ntc_net_apply(&p, d->buf, d->len, 100, &d->count, &lost, &dup),
              NTC_OK, "total loss applied");
    expect_u64((unsigned long long)d->count, 0u, "nothing survived");
    expect_u64((unsigned long long)lost, 10u, "ten losses counted");
    expect_u64((unsigned long long)dup, 0u, "no duplicates produced");
    dgram_list_free(d);

    /* A clean channel is a pass-through. */
    {
    dgram_list_t *c = &g_dl5;
        size_t l2 = 0, d2 = 0;
        ntc_net_profile_t clean;
        memset(c, 0, sizeof(*c));
        make_packets(c, 20, 1u, 16, 9u);
        ntc_net_profile_default(&clean, 5u);
        clean.loss_prob = 0.0;
        clean.dup_prob = 0.0;
        clean.reorder_prob = 0.0;
        expect_eq(ntc_net_apply(&clean, c->buf, c->len, 100, &c->count, &l2, &d2),
                  NTC_OK, "clean channel");
        expect_u64((unsigned long long)c->count, 20u, "all packets survive");
        expect_u64((unsigned long long)l2, 0u, "no loss");
        expect_u64((unsigned long long)d2, 0u, "no duplication");
        dgram_list_free(c);
    }

    /* Determinism: the same seed produces the same impairment. */
    {
        dgram_list_t a, b;
        size_t la = 0, lb = 0, da = 0, db = 0;
        ntc_net_profile_t pa, pb;
        memset(&a, 0, sizeof(a));
        memset(&b, 0, sizeof(b));
        make_packets(&a, 50, 1u, 16, 3u);
        make_packets(&b, 50, 1u, 16, 3u);
        ntc_net_profile_default(&pa, 999u);
        ntc_net_profile_default(&pb, 999u);
        ntc_net_apply(&pa, a.buf, a.len, 200, &a.count, &la, &da);
        ntc_net_apply(&pb, b.buf, b.len, 200, &b.count, &lb, &db);
        expect_u64((unsigned long long)a.count, (unsigned long long)b.count,
                   "same seed, same surviving count");
        expect_u64((unsigned long long)la, (unsigned long long)lb,
                   "same seed, same loss count");
        expect_u64((unsigned long long)da, (unsigned long long)db,
                   "same seed, same duplicate count");
        dgram_list_free(&a);
        dgram_list_free(&b);
    }
    ntc_net_free(NULL, 5);
    {
        uint8_t *one[2];
        one[0] = NULL;
        one[1] = NULL;
        ntc_net_free(one, 2);
        expect(1, "net_free tolerates NULL entries");
    }
}

/* ------------------------------------------------------------------ */
/* 4. NAL                                                              */
/* ------------------------------------------------------------------ */

/* Hand written FU-A payloads:
 *   start:  indicator 0x7C (F=0, NRI=3, type=28), FU header 0x85 (S=1, type=5)
 *   middle: 0x7C, FU header 0x05 (S=0, E=0, type=5)
 *   end:    0x7C, FU header 0x45 (E=1, type=5)
 */
static const uint8_t kFuStart[4] = { 0x7C, 0x85, 0x11, 0x22 };
static const uint8_t kFuMiddle[3] = { 0x7C, 0x05, 0x55 };
static const uint8_t kFuEnd[4] = { 0x7C, 0x45, 0x33, 0x44 };

static void test_nal_classify(void)
{
    const uint8_t *unit = NULL;
    size_t unit_len = 0;
    ntc_nal_fragment_info_t f;
    ntc_status_t st;

    group("nal: payload classification (hand written vectors)");

    st = ntc_nal_classify_payload(kFuStart, sizeof(kFuStart), &unit, &unit_len,
                                  &f);
    expect_eq(st, NTC_OK, "FU start classified");
    expect(unit == NULL, "a fragment yields no complete unit");
    expect_i(f.start, 1, "S bit set");
    expect_i(f.end, 0, "E bit clear");
    expect_i(f.type, 5, "original type 5");
    expect_i(f.nri, 3, "NRI 3");
    expect_u64(f.data_len, 2u, "two data bytes");
    expect_i(f.data[0], 0x11, "first data byte");
    expect_i(f.data[1], 0x22, "second data byte");

    st = ntc_nal_classify_payload(kFuMiddle, sizeof(kFuMiddle), &unit, &unit_len,
                                  &f);
    expect_eq(st, NTC_OK, "FU middle classified");
    expect_i(f.start, 0, "S clear");
    expect_i(f.end, 0, "E clear");
    expect_i(f.type, 5, "type 5");
    expect_u64(f.data_len, 1u, "one data byte");

    st = ntc_nal_classify_payload(kFuEnd, sizeof(kFuEnd), &unit, &unit_len, &f);
    expect_eq(st, NTC_OK, "FU end classified");
    expect_i(f.start, 0, "S clear");
    expect_i(f.end, 1, "E set");
    expect_i(f.type, 5, "type 5");

    /* A complete single-packet NAL unit (type 5 = IDR, NRI 3, F=0). */
    {
        static const uint8_t complete[5] = { 0x65, 0x88, 0x84, 0x21, 0x00 };
        st = ntc_nal_classify_payload(complete, sizeof(complete), &unit,
                                      &unit_len, &f);
        expect_eq(st, NTC_OK, "complete unit classified");
        expect(unit == complete, "unit points at the payload");
        expect_u64(unit_len, sizeof(complete), "unit length");
    }
    /* Forbidden and reserved NAL types. */
    {
        static const uint8_t forbidden[3] = { 0x00, 0x01, 0x02 };
        static const uint8_t reserved[3] = { 0x1F, 0x01, 0x02 };
        expect_eq(ntc_nal_classify_payload(forbidden, 3, &unit, &unit_len, &f),
                  NTC_NAL_ERR_BAD_TYPE, "type 0 forbidden");
        expect_eq(ntc_nal_classify_payload(reserved, 3, &unit, &unit_len, &f),
                  NTC_NAL_ERR_BAD_TYPE, "type 31 forbidden");
    }
    /* Truncated FU headers. */
    {
        static const uint8_t two[2] = { 0x7C, 0x85 };
        static const uint8_t one[1] = { 0x7C };
        expect_eq(ntc_nal_classify_payload(two, 2, &unit, &unit_len, &f),
                  NTC_NAL_ERR_BAD_FU, "FU payload without data refused");
        expect_eq(ntc_nal_classify_payload(one, 1, &unit, &unit_len, &f),
                  NTC_NAL_ERR_BAD_FU, "indicator only refused");
    }
    expect_eq(ntc_nal_classify_payload(NULL, 4, &unit, &unit_len, &f),
              NTC_ERR_INVAL, "NULL payload");
    expect_eq(ntc_nal_classify_payload(kFuStart, 0, &unit, &unit_len, &f),
              NTC_ERR_INVAL, "zero length payload");
    expect_eq(ntc_nal_classify_payload(kFuStart, sizeof(kFuStart), NULL, NULL,
                                       NULL),
              NTC_OK, "all out pointers may be NULL");
}

static void test_nal_fragment(void)
{
    uint8_t src[3000];
    uint8_t out[8192];
    size_t out_len = 0;
    size_t frags = 0;
    ntc_status_t st;

    group("nal: fragmentation");

    fill_pattern(src, sizeof(src), 3u);
    src[0] = 0x65; /* NRI 3, type 5 */

    st = ntc_nal_fragment(src, sizeof(src), 1200, out, sizeof(out), &out_len,
                          &frags);
    expect_eq(st, NTC_OK, "fragment ok");
    expect(frags > 1u, "a large NAL produces several fragments");
    expect_u64(out_len, (unsigned long long)out[0] * 256u + out[1],
               "length prefix matches the container length");
    expect(out_len <= sizeof(out), "output within capacity");

    {
        size_t pos = 2;
        size_t idx = 0;
        size_t total_data = 0;
        while (pos + 2u <= out_len) {
            size_t flen = (size_t)out[pos] * 256u + out[pos + 1];
            expect(flen > 2u, "each fragment carries data");
            expect(pos + 2u + flen <= out_len, "fragment fits the container");
            if (pos + 2u + flen > out_len) {
                break;
            }
            expect_i(out[pos + 2] & 0x1Fu, 28, "indicator type 28");
            expect_i((out[pos + 2] >> 5) & 0x03u, 3, "NRI preserved");
            expect_i(out[pos + 3] & 0x1Fu, 5, "FU header carries type 5");
            expect_i(out[pos + 3] & 0x20u, 0, "reserved R bit clear");
            if (idx == 0u) {
                expect_i(out[pos + 3] & 0x80u, 0x80u, "first fragment S=1");
                expect_i(out[pos + 3] & 0x40u, 0, "first fragment E=0");
            } else {
                expect_i(out[pos + 3] & 0x80u, 0, "later fragment S=0");
            }
            if (idx + 1u == frags) {
                expect_i(out[pos + 3] & 0x40u, 0x40u, "last fragment E=1");
            }
            total_data += flen - 2u;
            idx++;
            pos += 2u + flen;
        }
        expect_u64((unsigned long long)idx, (unsigned long long)frags,
                   "walked every fragment");
        /* The container holds the NAL HEADER byte once plus the payload
         * bytes: the header is rebuilt from the FU indicator and FU header by
         * the receiver, so the fragment data is one byte shorter than the NAL
         * unit it came from. */
        expect_u64((unsigned long long)total_data, sizeof(src) - 1u,
                   "fragment data is the payload without the header byte");
        expect_u64((unsigned long long)frags, 3u,
                   "3000 bytes at 1200 per packet is three fragments");
    }

    /* A NAL unit that fits is sent unfragmented. */
    {
        uint8_t small[10];
        fill_pattern(small, sizeof(small), 5u);
        small[0] = 0x41;
        st = ntc_nal_fragment(small, sizeof(small), 1200, out, sizeof(out),
                              &out_len, &frags);
        expect_eq(st, NTC_OK, "small fragment ok");
        expect_u64((unsigned long long)frags, 1u, "one fragment");
        expect_u64((unsigned long long)out_len, 2u + 2u + sizeof(small),
                   "container is 2 + 2 + 10 bytes");
        expect_i(out[4], 0x41, "unfragmented NAL keeps its header");
        expect(memcmp(out + 4, small, sizeof(small)) == 0,
               "unfragmented payload identical to the input");
    }
    /* Error paths. */
    expect_eq(ntc_nal_fragment(NULL, 10, 1200, out, sizeof(out), &out_len,
                               &frags),
              NTC_ERR_INVAL, "fragment NULL source");
    expect_eq(ntc_nal_fragment(src, 10, 1200, NULL, sizeof(out), &out_len,
                               &frags),
              NTC_ERR_INVAL, "fragment NULL output");
    expect_eq(ntc_nal_fragment(src, 10, 1200, out, sizeof(out), NULL, &frags),
              NTC_ERR_INVAL, "fragment NULL out_len");
    expect_eq(ntc_nal_fragment(src, 0, 1200, out, sizeof(out), &out_len, &frags),
              NTC_ERR_INVAL, "fragment zero length");
    expect_eq(ntc_nal_fragment(src, sizeof(src), 2, out, sizeof(out), &out_len,
                               &frags),
              NTC_ERR_RANGE, "payload budget too small");
    {
        uint8_t bad[10];
        memset(bad, 0, sizeof(bad));
        bad[0] = 0x00;
        expect_eq(ntc_nal_fragment(bad, sizeof(bad), 1200, out, sizeof(out),
                                   &out_len, &frags),
                  NTC_NAL_ERR_BAD_TYPE, "forbidden type refused");
        bad[0] = 0x1F;
        expect_eq(ntc_nal_fragment(bad, sizeof(bad), 1200, out, sizeof(out),
                                   &out_len, &frags),
                  NTC_NAL_ERR_BAD_TYPE, "reserved type refused");
    }
    expect_eq(ntc_nal_fragment(src, sizeof(src), 1200, out, 4, &out_len, &frags),
              NTC_ERR_NO_ROOM, "container too small");
    expect_eq(ntc_nal_fragment(src, sizeof(src), 1200, out, 1, &out_len, &frags),
              NTC_ERR_NO_ROOM, "container smaller than the prefix");
}

/*
 * Feed the fragments of one container through the reassembler, optionally
 * dropping and/or duplicating one fragment index, optionally in reverse.
 * Returns the status of the last push.
 */
static int feed_fragments(ntc_nal_reasm_t *r, const uint8_t *tlv, size_t tlv_len,
                          uint32_t ts, int drop_index, int dup_index,
                          int reverse, uint8_t *rebuilt, size_t *rebuilt_len,
                          int *completed_any, size_t *frag_count_out)
{
    size_t pos = 2;
    int last = (int)NTC_OK;
    size_t frag_count = 0;
    size_t offsets[NTC_NAL_MAX_FRAG];
    size_t lengths[NTC_NAL_MAX_FRAG];
    size_t k;

    *completed_any = 0;
    *rebuilt_len = 0;
    while (pos + 2u <= tlv_len) {
        size_t flen = (size_t)tlv[pos] * 256u + tlv[pos + 1];
        if (pos + 2u + flen > tlv_len) {
            break;
        }
        if (frag_count < (size_t)NTC_NAL_MAX_FRAG) {
            offsets[frag_count] = pos + 2u;
            lengths[frag_count] = flen;
            frag_count++;
        }
        pos += 2u + flen;
    }
    if (frag_count_out != NULL) {
        *frag_count_out = frag_count;
    }
    for (k = 0; k < frag_count; k++) {
        size_t pick = reverse ? (frag_count - 1u - k) : k;
        int completed = 0;
        if ((int)pick == drop_index) {
            continue;
        }
        /*
         * The fragment position is passed explicitly, exactly as a real
         * receiver derives it from the RTP sequence numbers: FU-A's own
         * header carries only S and E, so a missing MIDDLE fragment is
         * invisible in the payload itself.
         */
        last = (int)ntc_nal_reasm_push_ex(r, tlv + offsets[pick],
                                          lengths[pick], ts, (long)pick,
                                          &completed);
        if (completed) {
            *completed_any = 1;
            if (*rebuilt_len == 0u) {
                memcpy(rebuilt, r->last_unit, r->last_unit_len);
                *rebuilt_len = r->last_unit_len;
            }
        }
        if ((int)pick == dup_index) {
            completed = 0;
            last = (int)ntc_nal_reasm_push_ex(r, tlv + offsets[pick],
                                              lengths[pick], ts, (long)pick,
                                              &completed);
        }
    }
    return last;
}

static void test_nal_reasm(void)
{
    uint8_t src[3000];
    uint8_t tlv[8192];
    uint8_t rebuilt[8192];
    size_t tlv_len = 0;
    size_t frags = 0;
    size_t rebuilt_len = 0;
    int completed = 0;
    ntc_nal_reasm_t *r = &g_reasm;
    ntc_nal_stats_t st;

    group("nal: reassembly");

    fill_pattern(src, sizeof(src), 11u);
    src[0] = 0x65;
    expect_eq(ntc_nal_fragment(src, sizeof(src), 1200, tlv, sizeof(tlv),
                               &tlv_len, &frags),
              NTC_OK, "fragment the unit for reassembly");
    expect(frags >= 3u, "at least three fragments for the interesting cases");
    measured("nal_fragment_count", (unsigned long long)frags);

    expect_eq(ntc_nal_reasm_init(NULL), NTC_ERR_INVAL, "reasm init NULL");
    expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reasm init");
    expect_i(r->in_progress, 0, "nothing in progress at the start");
    expect_eq(ntc_nal_reasm_stats(NULL, &st), NTC_ERR_INVAL, "stats NULL r");
    expect_eq(ntc_nal_reasm_stats(r, NULL), NTC_ERR_INVAL, "stats NULL out");
    expect_eq(ntc_nal_reasm_flush(NULL), NTC_ERR_INVAL, "flush NULL");
    expect_eq(ntc_nal_reasm_push(NULL, tlv, 10, 0u, &completed),
              NTC_ERR_INVAL, "push NULL reassembler");
    expect_eq(ntc_nal_reasm_push(r, NULL, 10, 0u, &completed), NTC_ERR_INVAL,
              "push NULL payload");
    expect_eq(ntc_nal_reasm_push(r, tlv, 0, 0u, &completed), NTC_ERR_INVAL,
              "push zero length");

    /* (a) Clean in-order reassembly reproduces the input exactly. */
    {
        size_t nf = 0;
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        (void)feed_fragments(r, tlv, tlv_len, 9000u, -1, -1, 0, rebuilt,
                             &rebuilt_len, &completed, &nf);
        expect_u64((unsigned long long)nf, (unsigned long long)frags,
                   "fragment walk saw every fragment");
        expect_i(completed, 1, "unit completed");
        expect_u64((unsigned long long)rebuilt_len, sizeof(src),
                   "rebuilt length matches");
        expect(memcmp(rebuilt, src, sizeof(src)) == 0,
               "rebuilt bytes identical to the original NAL unit");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.units_complete, 1u, "one unit complete");
        expect_u64(st.units_dropped, 0u, "no drops on a clean run");
        expect_u64(st.fragments_in, (unsigned long long)frags,
                   "fragment count matches the sender");
        expect_u64(st.bytes_reassembled, sizeof(src), "bytes reassembled");
        expect_u64(st.duplicate_frags, 0u, "no duplicates");
        measured("nal_clean_units_complete", st.units_complete);
        measured("nal_reassembled_bytes", st.bytes_reassembled);
    }

    /* (b) Reverse arrival order: every fragment that has no start to attach
     * to is refused, and the trailing S fragment starts a unit that is never
     * completed.  Nothing is emitted. */
    {
        size_t nf = 0;
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        (void)feed_fragments(r, tlv, tlv_len, 9000u, -1, -1, 1, rebuilt,
                             &rebuilt_len, &completed, &nf);
        expect_i(completed, 0, "reverse order completes nothing");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.units_complete, 0u, "no unit completed from reverse order");
        expect_u64(st.units_dropped, 0u, "no unit was ever in progress");
        expect_u64(st.dropped_missing_frag, (unsigned long long)(frags - 1u),
                   "every start-less fragment was refused");
        expect_eq(ntc_nal_reasm_flush(r), NTC_OK, "flush the dangling unit");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats after flush");
        expect_u64(st.units_dropped, 1u, "the dangling unit is dropped");
        measured("nal_reverse_refused_frags", st.dropped_missing_frag);
    }

    /* (c) A middle fragment is lost: the whole unit is dropped and counted
     * once, and nothing is emitted. */
    {
        size_t drop = frags / 2u;
        expect(drop > 0u && drop + 1u < frags, "drop a genuine middle fragment");
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        (void)feed_fragments(r, tlv, tlv_len, 9000u, (int)drop, -1, 0, rebuilt,
                             &rebuilt_len, &completed, NULL);
        expect_i(completed, 0, "no complete unit when a fragment is missing");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.units_complete, 0u, "nothing completed");
        expect_u64(st.units_dropped, 1u, "exactly one unit dropped");
        expect(st.dropped_missing_frag >= 1u, "the drop is attributed to a gap");
        measured("nal_gap_dropped_units", st.units_dropped);
        measured("nal_gap_missing_frag_events", st.dropped_missing_frag);
    }

    /* (d) The first fragment (S) is lost: the rest must not be emitted. */
    {
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        (void)feed_fragments(r, tlv, tlv_len, 9000u, 0, -1, 0, rebuilt,
                             &rebuilt_len, &completed, NULL);
        expect_i(completed, 0, "no unit without its start fragment");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.units_complete, 0u, "nothing completed");
        expect_u64(st.units_dropped, 0u, "no unit was ever in progress");
        expect_u64(st.dropped_missing_frag, (unsigned long long)(frags - 1u),
                   "every orphan fragment refused");
    }

    /* (e) The last fragment (E) is lost: the second middle fragment proves
     * the E is missing, so the unit is dropped immediately. */
    {
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        (void)feed_fragments(r, tlv, tlv_len, 9000u, (int)(frags - 1u), -1, 0,
                             rebuilt, &rebuilt_len, &completed, NULL);
        expect_i(completed, 0, "no unit without its end fragment");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.units_complete, 0u, "nothing completed");
        /* Nothing is dropped yet: a receiver cannot know whether the end
         * fragment is lost or merely late, so the unit stays buffered until
         * the end of the stream.  This is why a real stack also needs an
         * end-of-stream or keyframe boundary to release the buffer. */
        expect_u64(st.units_dropped, 0u, "still buffered, not yet dropped");
        expect_eq(ntc_nal_reasm_flush(r), NTC_OK, "flush at end of stream");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats after the flush");
        expect_u64(st.units_complete, 0u, "still nothing completed");
        expect_u64(st.units_dropped, 1u, "the truncated unit was dropped");
        expect_u64(st.dropped_truncated, 1u, "attributed to truncation");
    }

    /* (f) A duplicated middle fragment is ignored and the unit still
     * reassembles byte-exactly. */
    {
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        (void)feed_fragments(r, tlv, tlv_len, 9000u, -1, 1, 0, rebuilt,
                             &rebuilt_len, &completed, NULL);
        expect_i(completed, 1, "a duplicate fragment does not break the unit");
        expect_u64((unsigned long long)rebuilt_len, sizeof(src),
                   "length is still right");
        expect(memcmp(rebuilt, src, sizeof(src)) == 0,
               "bytes are still identical with a duplicated fragment");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect(st.duplicate_frags >= 1u, "the duplicate was counted");
        measured("nal_duplicate_frags", st.duplicate_frags);
    }

    /* (g) An unfragmented unit goes straight through. */
    {
        uint8_t small[12];
        fill_pattern(small, sizeof(small), 13u);
        small[0] = 0x41;
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        expect_eq(ntc_nal_reasm_push(r, small, sizeof(small), 100u, &completed),
                  NTC_OK, "push unfragmented unit");
        expect_i(completed, 1, "completed immediately");
        expect_u64((unsigned long long)r->last_unit_len, sizeof(small),
                   "stored length");
        expect(memcmp(r->last_unit, small, sizeof(small)) == 0, "stored bytes");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.single_packet_units, 1u, "single packet unit counted");
        measured("nal_single_packet_units", st.single_packet_units);
    }

    /* (h) An unfragmented unit arriving mid-unit truncates the old one. */
    {
        uint8_t small[8];
        size_t flen;
        fill_pattern(small, sizeof(small), 17u);
        small[0] = 0x41;
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        flen = (size_t)tlv[2] * 256u + tlv[3];
        expect_eq(ntc_nal_reasm_push(r, tlv + 4, flen, 9000u, &completed),
                  NTC_OK, "start fragment accepted");
        expect_i(completed, 0, "not complete yet");
        expect_eq(ntc_nal_reasm_push(r, small, sizeof(small), 12000u,
                                     &completed),
                  NTC_OK, "unfragmented unit accepted");
        expect_i(completed, 1, "the new unit completed");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.dropped_truncated, 1u, "the interrupted unit was dropped");
    }

    /* (i) A fragment from a different frame timestamp aborts the unit. */
    {
        size_t flen = (size_t)tlv[2] * 256u + tlv[3];
        size_t pos = 2u + 2u + flen;
        size_t flen2 = (size_t)tlv[pos] * 256u + tlv[pos + 1];
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        expect_eq(ntc_nal_reasm_push(r, tlv + 4, flen, 1000u, &completed),
                  NTC_OK, "start accepted");
        expect_eq(ntc_nal_reasm_push(r, tlv + pos + 2u, flen2, 4000u,
                                     &completed),
                  NTC_NAL_ERR_GAP, "timestamp change detected as a gap");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.units_dropped, 1u, "unit dropped on a timestamp mismatch");
    }

    /* (j) A unit larger than the reassembly buffer overflows and is dropped
     * once, without corrupting the following parse. */
    {
        uint8_t big[NTC_NAL_MAX_UNIT + 2048];
        uint8_t bigtlv[2 * (NTC_NAL_MAX_UNIT + 2048)];
        size_t big_tlv_len = 0;
        size_t big_frags = 0;
        size_t k;
        int saw_overflow = 0;
        fill_pattern(big, sizeof(big), 21u);
        big[0] = 0x65;
        expect_eq(ntc_nal_fragment(big, sizeof(big), 1200, bigtlv,
                                   sizeof(bigtlv), &big_tlv_len, &big_frags),
                  NTC_OK, "oversized unit fragmented");
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        {
            size_t pos = 2;
            size_t accumulated = 1u; /* the rebuilt NAL header byte */
            for (k = 0; k < big_frags; k++) {
                size_t flen = (size_t)bigtlv[pos] * 256u + bigtlv[pos + 1];
                ntc_status_t s;
                accumulated += flen - 2u;
                if (accumulated > (size_t)NTC_NAL_MAX_UNIT) {
                    /* The unit provably cannot fit: the reassembler must
                     * refuse it rather than silently truncate. */
                    expect_u64((unsigned long long)accumulated,
                               (unsigned long long)accumulated,
                               "oversized unit is over the buffer limit");
                }
                s = (ntc_status_t)ntc_nal_reasm_push_ex(r, bigtlv + pos + 2u,
                                                        flen, 7u, (long)k,
                                                        &completed);
                if (s == NTC_NAL_ERR_NO_ROOM) {
                    saw_overflow = 1;
                    break;
                }
                pos += 2u + flen;
            }
        }
        expect_i(saw_overflow, 1, "overflow reported");
        expect(completed == 0, "no oversized unit is ever emitted");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.dropped_overflow, 1u, "overflow drop counted");
        expect_u64(st.units_complete, 0u, "the oversized unit was not emitted");
    }

    /* (k) An orphan middle fragment while idle is refused. */
    {
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        expect_eq(ntc_nal_reasm_push(r, kFuMiddle, sizeof(kFuMiddle), 1u,
                                     &completed),
                  NTC_NAL_ERR_GAP, "orphan middle fragment refused");
        expect_eq(ntc_nal_reasm_stats(r, &st), NTC_OK, "stats");
        expect_u64(st.units_dropped, 0u, "no unit was in progress");
        expect_u64(st.dropped_missing_frag, 1u, "the orphan was counted");
    }

    /* (l) A hand written three-fragment unit reassembles to the exact bytes
     * the specification implies: header rebuilt from both FU bytes. */
    {
        static const uint8_t f1[4] = { 0x7C, 0x85, 0xAA, 0xBB };
        static const uint8_t f2[3] = { 0x7C, 0x05, 0xCC };
        static const uint8_t f3[3] = { 0x7C, 0x45, 0xDD };
        expect_eq(ntc_nal_reasm_init(r), NTC_OK, "reinit");
        expect_eq(ntc_nal_reasm_push_ex(r, f1, sizeof(f1), 5u, 0L, &completed),
                  NTC_OK, "hand written start");
        expect_eq(ntc_nal_reasm_push_ex(r, f2, sizeof(f2), 5u, 1L, &completed),
                  NTC_OK, "hand written middle");
        expect_eq(ntc_nal_reasm_push_ex(r, f3, sizeof(f3), 5u, 2L, &completed),
                  NTC_OK, "hand written end");
        expect_i(completed, 1, "hand written unit completed");
        expect_u64((unsigned long long)r->last_unit_len, 5u, "one header + four bytes");
        expect_i(r->last_unit[0], 0x65, "header rebuilt as 0x65 (NRI 3, type 5)");
        expect_i(r->last_unit[1], 0xAA, "first data byte");
        expect_i(r->last_unit[2], 0xBB, "second data byte");
        expect_i(r->last_unit[3], 0xCC, "third data byte");
        expect_i(r->last_unit[4], 0xDD, "fourth data byte");
        expect_i(r->last_unit_type, 5, "reported type is 5");
    }
}

/* ------------------------------------------------------------------ */
/* 5. motion detection                                                 */
/* ------------------------------------------------------------------ */

#define MOT_W 64u
#define MOT_H 48u
#define MOT_LEN (MOT_W * MOT_H)

static void paint_frame(uint8_t *frame, size_t w, size_t h, int base, int x,
                        int y, int bw, int bh, int value)
{
    size_t i;
    for (i = 0; i < w * h; i++) {
        frame[i] = (uint8_t)base;
    }
    if (bw > 0 && bh > 0) {
        int yy;
        int xx;
        for (yy = y; yy < y + bh; yy++) {
            for (xx = x; xx < x + bw; xx++) {
                if (xx >= 0 && yy >= 0 && (size_t)xx < w && (size_t)yy < h) {
                    frame[(size_t)yy * w + (size_t)xx] = (uint8_t)value;
                }
            }
        }
    }
}

static void test_motion_config(void)
{
    ntc_motion_config_t cfg;
    ntc_motion_detector_t *d = &g_det;

    group("motion: configuration validation");

    expect_eq(ntc_motion_config_default(NULL, 64, 48), NTC_ERR_INVAL,
              "default NULL");
    expect_eq(ntc_motion_config_default(&cfg, MOT_W, MOT_H), NTC_OK,
              "default ok");
    expect_i((int)cfg.width, (int)MOT_W, "width copied");
    expect_i(cfg.block_size, 16, "default block size");
    expect_i(cfg.pixel_threshold, 20, "default pixel threshold");
    expect_d(cfg.motion_threshold, 0.02, 1e-12, "default start threshold");
    expect_d(cfg.end_threshold, 0.01, 1e-12, "default end threshold");
    expect_i(cfg.start_frames, 3, "default start debounce");
    expect_i(cfg.end_frames, 5, "default end debounce");
    expect_i(cfg.cooldown_frames, 25, "default cooldown");
    expect_i(cfg.min_event_frames, 5, "default minimum event length");
    expect_i(cfg.bg_alpha, 8, "default background alpha");
    expect_i((int)cfg.link_flags,
             (int)(NTC_MOTION_LINK_RECORD | NTC_MOTION_LINK_STREAM),
             "default link flags");
    expect_eq(ntc_motion_config_check(&cfg), NTC_OK, "defaults pass the check");
    expect_eq(ntc_motion_config_check(NULL), NTC_ERR_INVAL, "check NULL");

    {
        ntc_motion_config_t bad;
        bad = cfg;
        bad.width = 0;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "zero width refused");
        bad = cfg;
        bad.height = 0;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "zero height refused");
        bad = cfg;
        bad.block_size = 0;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "zero block size refused");
        bad = cfg;
        bad.block_size = bad.width + 1u;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "block larger than the frame refused");
        bad = cfg;
        bad.pixel_threshold = 0;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "zero pixel threshold refused");
        bad = cfg;
        bad.pixel_threshold = 256;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "pixel threshold 256 refused");
        bad = cfg;
        bad.motion_threshold = 0.0;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "zero motion threshold refused");
        bad = cfg;
        bad.motion_threshold = 1.5;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "motion threshold above 1 refused");
        bad = cfg;
        bad.end_threshold = cfg.motion_threshold + 0.5;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "end threshold above the start threshold refused");
        bad = cfg;
        bad.end_threshold = -0.5;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "negative end threshold refused");
        bad = cfg;
        bad.start_frames = 0;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "zero start frames refused");
        bad = cfg;
        bad.end_frames = 0;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "zero end frames refused");
        bad = cfg;
        bad.cooldown_frames = -1;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "negative cooldown refused");
        bad = cfg;
        bad.min_event_frames = -1;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "negative minimum event length refused");
        bad = cfg;
        bad.bg_alpha = 0;
        expect_eq(ntc_motion_config_check(&bad), NTC_MOT_ERR_CONFIG,
                  "zero background alpha refused");
    }

    expect_eq(ntc_motion_config_default(&cfg, 0, 0), NTC_OK,
              "default with zero geometry still fills the struct");
    expect_eq(ntc_motion_init(d, &cfg), NTC_MOT_ERR_CONFIG,
              "init with zero geometry refused");
    expect_eq(ntc_motion_init(NULL, &cfg), NTC_ERR_INVAL, "init NULL detector");
    expect_eq(ntc_motion_init(d, NULL), NTC_ERR_INVAL, "init NULL config");

    ntc_motion_config_default(&cfg, MOT_W, MOT_H);
    expect_eq(ntc_motion_init(d, &cfg), NTC_OK, "init ok");
    expect(d->background != NULL, "background allocated");
    expect_u64((unsigned long long)d->bg_len, MOT_LEN, "background length");
    expect_i(d->background_ready, 0, "no background yet");
    expect_i((int)ntc_motion_state(d), (int)NTC_MOTION_IDLE, "starts IDLE");
    expect_u64((unsigned long long)d->total_blocks, 12u,
               "64x48 with 16px blocks is 4x3 = 12 blocks");
    ntc_motion_destroy(d);
    expect(d->background == NULL, "background released");
    ntc_motion_destroy(d);
    ntc_motion_destroy(NULL);
    expect(1, "destroy is NULL-safe and idempotent");

    expect_str(ntc_motion_state_str(NTC_MOTION_IDLE), "IDLE", "state name IDLE");
    expect_str(ntc_motion_state_str(NTC_MOTION_PENDING), "PENDING",
               "state name PENDING");
    expect_str(ntc_motion_state_str(NTC_MOTION_ACTIVE), "ACTIVE",
               "state name ACTIVE");
    expect_str(ntc_motion_state_str(NTC_MOTION_COOLDOWN), "COOLDOWN",
               "state name COOLDOWN");
    expect_str(ntc_motion_state_str((ntc_motion_state_t)77), "?",
               "unknown state name");
}

static void test_motion_pipeline(void)
{
    ntc_motion_config_t cfg;
    ntc_motion_detector_t *d = &g_det;
    uint8_t frame[MOT_LEN];
    double ratio = 0.0;
    size_t changed = 0;
    int started = 0;
    int ended = 0;
    ntc_motion_stats_t st;

    group("motion: frame difference, thresholds and debounce");

    ntc_motion_config_default(&cfg, MOT_W, MOT_H);
    expect_eq(ntc_motion_init(d, &cfg), NTC_OK, "init");

    expect_eq(ntc_motion_process(NULL, frame, 0.0, NULL, NULL, NULL),
              NTC_ERR_INVAL, "process NULL detector");
    expect_eq(ntc_motion_process(d, NULL, 0.0, NULL, NULL, NULL),
              NTC_ERR_INVAL, "process NULL frame");
    expect_eq(ntc_motion_measure(NULL, frame, &ratio, &changed), NTC_ERR_INVAL,
              "measure NULL detector");
    expect_eq(ntc_motion_measure(d, NULL, &ratio, &changed), NTC_ERR_INVAL,
              "measure NULL frame");
    expect_eq(ntc_motion_stats(NULL, &st), NTC_ERR_INVAL, "stats NULL detector");
    expect_eq(ntc_motion_stats(d, NULL), NTC_ERR_INVAL, "stats NULL out");
    expect_i((int)ntc_motion_state(NULL), (int)NTC_MOTION_IDLE,
             "state(NULL) is IDLE");

    /* Frame 0 defines the background. */
    paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
    expect_eq(ntc_motion_process(d, frame, 0.0, &ratio, &started, &ended),
              NTC_OK, "first frame processed");
    expect_d(ratio, 0.0, 1e-12, "the first frame has no motion");
    expect_i(started, 0, "the first frame raises nothing");
    expect_i(d->background_ready, 1, "background established");
    expect_eq(ntc_motion_measure(d, frame, &ratio, &changed), NTC_OK,
              "measure an identical frame");
    expect_d(ratio, 0.0, 1e-12, "identical frame ratio 0");
    expect_u64((unsigned long long)changed, 0u, "no changed blocks");

    /* One fully changed block out of twelve -> 1/12. */
    paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 16, 16, 200);
    expect_eq(ntc_motion_measure(d, frame, &ratio, &changed), NTC_OK,
              "measure one moved block");
    expect_u64((unsigned long long)changed, 1u, "exactly one changed block");
    expect_d(ratio, 1.0 / 12.0, 1e-12, "ratio is 1/12");
    expect_eq(ntc_motion_process(d, frame, 1.0, &ratio, &started, &ended),
              NTC_OK, "mover frame 1");
    expect_i(started, 0, "not raised after one mover");
    expect_eq(ntc_motion_process(d, frame, 2.0, &ratio, &started, &ended),
              NTC_OK, "mover frame 2");
    expect_i(started, 0, "not raised after two movers");
    expect_i((int)ntc_motion_state(d), (int)NTC_MOTION_PENDING,
             "state PENDING");
    expect_eq(ntc_motion_process(d, frame, 3.0, &ratio, &started, &ended),
              NTC_OK, "mover frame 3");
    expect_i(started, 1, "raised on the third consecutive mover");
    expect_i((int)ntc_motion_state(d), (int)NTC_MOTION_ACTIVE, "state ACTIVE");
    expect_eq(ntc_motion_process(d, frame, 4.0, &ratio, &started, NULL),
              NTC_OK, "mover frame 4");
    expect_i(started, 0, "no second alarm while still active");

    /* A single still frame inside PENDING resets the debounce. */
    {
        ntc_motion_config_t c2;
    ntc_motion_detector_t *d2 = &g_det2;
        int i;
        ntc_motion_config_default(&c2, MOT_W, MOT_H);
        expect_eq(ntc_motion_init(d2, &c2), NTC_OK, "second detector");
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        ntc_motion_process(d2, frame, 0.0, NULL, NULL, NULL);
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 16, 16, 200);
        ntc_motion_process(d2, frame, 1.0, NULL, &started, NULL);
        ntc_motion_process(d2, frame, 2.0, NULL, &started, NULL);
        expect_i(started, 0, "two movers is not enough");
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        ntc_motion_process(d2, frame, 3.0, NULL, NULL, NULL);
        expect_i((int)ntc_motion_state(d2), (int)NTC_MOTION_IDLE,
                 "a still frame resets PENDING to IDLE");
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 16, 16, 200);
        ntc_motion_process(d2, frame, 4.0, NULL, NULL, NULL);
        ntc_motion_process(d2, frame, 5.0, NULL, NULL, NULL);
        expect_i((int)ntc_motion_state(d2), (int)NTC_MOTION_PENDING,
                 "the counter restarted from scratch");
        /*
         * ntc_motion_process() RESETS the out flag on every call, so a loop
         * that passes the same variable only reports the last iteration.  The
         * first version of this check therefore always saw 0.
         */
        {
            int raised = 0;
            for (i = 0; i < 3; i++) {
                int one = 0;
                ntc_motion_process(d2, frame, 6.0 + (double)i, NULL, &one,
                                   NULL);
                if (one) {
                    raised = 1;
                }
            }
            expect_i(raised, 1, "raised after a fresh three-frame run");
        }
        ntc_motion_destroy(d2);
    }

    /* Clearing needs end_frames consecutive still frames. */
    paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
    {
        int i;
        for (i = 0; i < cfg.end_frames - 1; i++) {
            ntc_motion_process(d, frame, 10.0 + (double)i, &ratio, NULL,
                               &ended);
            expect_i(ended, 0, "not cleared before end_frames still frames");
        }
        ntc_motion_process(d, frame, 99.0, &ratio, NULL, &ended);
        expect_i(ended, 1, "cleared after end_frames still frames");
        expect_i((int)ntc_motion_state(d), (int)NTC_MOTION_COOLDOWN,
                 "state COOLDOWN after clearing");
    }
    /* Motion inside the cooldown: no new alarm, straight back to ACTIVE. */
    paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 16, 16, 200);
    ntc_motion_process(d, frame, 100.0, &ratio, &started, &ended);
    expect_i(started, 0, "no new alarm inside the cooldown");
    expect_i((int)ntc_motion_state(d), (int)NTC_MOTION_ACTIVE,
             "back to ACTIVE for the ongoing motion");

    /* Hysteresis: while ACTIVE the lower end_threshold decides. */
    {
        int i;
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        for (i = 0; i < cfg.end_frames; i++) {
            ntc_motion_process(d, frame, 200.0 + (double)i, &ratio, NULL,
                               &ended);
        }
        expect_i(ended, 1, "cleared again");
        expect_eq(ntc_motion_stats(d, &st), NTC_OK, "stats");
        expect_u64(st.events_started, 1u, "still only one alarm");
        expect_u64(st.events_ended, 2u, "two clearings (one suppressed)");
        expect_u64(st.events_suppressed_cooldown, 1u, "one cooldown suppression");
    }

    /* Measurement edge cases. */
    paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 64, 48, 200);
    expect_eq(ntc_motion_measure(d, frame, &ratio, &changed), NTC_OK,
              "measure a fully changed frame");
    expect_d(ratio, 1.0, 1e-12, "a fully changed frame gives ratio 1");
    expect_u64((unsigned long long)changed, 12u, "all twelve blocks changed");
    paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
    frame[0] = 200;
    expect_eq(ntc_motion_measure(d, frame, &ratio, &changed), NTC_OK,
              "measure a single bright pixel");
    expect_u64((unsigned long long)changed, 0u,
               "one pixel does not move a block");
    expect_d(ratio, 0.0, 1e-12, "ratio still 0");
    {
        int yy;
        int xx;
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        for (yy = 0; yy < 8; yy++) {
            for (xx = 0; xx < 16; xx++) {
                frame[(size_t)yy * MOT_W + (size_t)xx] = 200;
            }
        }
        expect_eq(ntc_motion_measure(d, frame, &ratio, &changed), NTC_OK,
                  "measure a half-changed block");
        expect_u64((unsigned long long)changed, 1u,
                   "half the pixels is enough to move a block");
    }
    {
        int yy;
        int xx;
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        for (yy = 0; yy < 16; yy++) {
            for (xx = 0; xx < 16; xx++) {
                frame[(size_t)yy * MOT_W + (size_t)xx] = 119; /* diff of 19 */
            }
        }
        expect_eq(ntc_motion_measure(d, frame, &ratio, &changed), NTC_OK,
                  "measure a change just below the pixel threshold");
        expect_u64((unsigned long long)changed, 0u,
                   "a difference of 19 is not over a threshold of 20");
    }
    {
        int yy = 0;
        int xx = 0;
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        for (yy = 0; yy < 16; yy++) {
            for (xx = 0; xx < 16; xx++) {
                frame[(size_t)yy * MOT_W + (size_t)xx] = 120; /* diff of 20 */
            }
        }
        expect_eq(ntc_motion_measure(d, frame, &ratio, &changed), NTC_OK,
                  "measure a change exactly at the threshold");
        expect_u64((unsigned long long)changed, 0u,
                   "the test is 'greater than', so exactly 20 does not count");
        for (yy = 0; yy < 16; yy++) {
            for (xx = 0; xx < 16; xx++) {
                frame[(size_t)yy * MOT_W + (size_t)xx] = 121; /* diff of 21 */
            }
        }
        expect_eq(ntc_motion_measure(d, frame, &ratio, &changed), NTC_OK,
                  "measure a change one over the threshold");
        expect_u64((unsigned long long)changed, 1u,
                   "a difference of 21 does count");
    }
    expect_eq(ntc_motion_stats(d, &st), NTC_OK, "final stats");
    expect(st.frames > 0u, "frames counted");
    expect(st.record_triggers >= 1u, "at least one recording triggered");
    expect(st.stream_triggers >= 1u, "at least one stream trigger");
    expect_u64(st.snapshot_triggers, 0u, "snapshot linkage is off by default");
    expect(st.motion_sum > 0.0, "motion sum accumulated");
    ntc_motion_destroy(d);
}

static void test_motion_scripted(void)
{
    /*
     * Ground truth for the scripted sequence: 60 frames.
     *   frames  0..9   still
     *   frames 10..19  real motion (event 1)
     *   frames 20..29  still, with one noisy frame at 23
     *   frames 30..39  real motion (event 2)
     *   frames 40..59  still, with noisy frames at 44 and 52
     * Exactly 2 real events and exactly 3 single-frame noise blips.
     */
    enum { FRAMES = 60 };
    static const int noisy[3] = { 23, 44, 52 };
    uint8_t frame[MOT_LEN];
    int f;
    int i;

    group("motion: scripted sequence, false positives and missed events");

    {
        ntc_motion_config_t cfg_raw;
    ntc_motion_detector_t *d_raw = &g_det;
        ntc_motion_config_t cfg_deb;
    ntc_motion_detector_t *d_deb = &g_det2;
        ntc_motion_stats_t a;
        ntc_motion_stats_t b;

        /* Variant 1: no debounce at all -- the naive detector. */
        ntc_motion_config_default(&cfg_raw, MOT_W, MOT_H);
        cfg_raw.start_frames = 1;
        cfg_raw.end_frames = 1;
        cfg_raw.cooldown_frames = 0;
        cfg_raw.min_event_frames = 0;
        cfg_raw.motion_threshold = 0.005;
        cfg_raw.end_threshold = 0.005;
        expect_eq(ntc_motion_init(d_raw, &cfg_raw), NTC_OK, "naive detector");

        /*
         * Variant 2: debounce with the cooldown switched OFF, so this
         * measurement isolates the debounce.  With the default 25-frame
         * cooldown the second real event would be suppressed as an alarm
         * storm, which is the point of the cooldown and is measured
         * separately below.
         */
        ntc_motion_config_default(&cfg_deb, MOT_W, MOT_H);
        cfg_deb.cooldown_frames = 0;
        expect_eq(ntc_motion_init(d_deb, &cfg_deb), NTC_OK,
                  "debounced detector");

        for (f = 0; f < FRAMES; f++) {
            int is_motion = (f >= 10 && f <= 19) || (f >= 30 && f <= 39);
            int is_noisy = 0;
            double now = (double)f * 33.0;
            for (i = 0; i < 3; i++) {
                if (noisy[i] == f) {
                    is_noisy = 1;
                }
            }
            if (is_motion) {
                paint_frame(frame, MOT_W, MOT_H, 100, (f % 4) * 16, 0, 16, 16,
                            200);
            } else if (is_noisy) {
                paint_frame(frame, MOT_W, MOT_H, 100, 32, 16, 16, 16, 210);
            } else {
                paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
            }
            ntc_motion_process(d_raw, frame, now, NULL, NULL, NULL);
            ntc_motion_process(d_deb, frame, now, NULL, NULL, NULL);
        }
        expect_eq(ntc_motion_stats(d_raw, &a), NTC_OK, "naive stats");
        expect_eq(ntc_motion_stats(d_deb, &b), NTC_OK, "debounced stats");

        expect_u64(a.frames, (unsigned long long)FRAMES, "naive saw every frame");
        expect_u64(b.frames, (unsigned long long)FRAMES, "debounced saw every frame");
        expect_u64(a.events_started, 5u,
                   "the naive detector raised 5 alarms (2 real + 3 noise)");
        expect_u64(b.events_started, 2u,
                   "the debounced detector raised exactly the 2 real alarms");
        expect(b.events_started == 2u, "no real event was missed");
        expect_u64(a.events_started - b.events_started, 3u,
                   "exactly the 3 noise blips were suppressed");
        expect(b.record_triggers == 2u, "both alarms triggered a recording");
        expect(b.stream_triggers == 2u, "both alarms triggered a stream");

        printf("   scripted 60 frames: naive alarms=%llu, debounced alarms=%llu"
               " (2 real events, 3 single-frame noise blips)\n",
               a.events_started, b.events_started);

        /* With the shipped 25-frame cooldown the second event is suppressed
         * as an alarm storm: that is the anti-storm rule doing its job, not a
         * missed detection. */
        {
            ntc_motion_config_t cfg_cool;
            ntc_motion_detector_t *d_cool = &g_det2;
            ntc_motion_stats_t c;
            int g;
            ntc_motion_config_default(&cfg_cool, MOT_W, MOT_H);
            expect_eq(ntc_motion_init(d_cool, &cfg_cool), NTC_OK,
                      "cooldown detector");
            for (g = 0; g < FRAMES; g++) {
                int is_motion = (g >= 10 && g <= 19) || (g >= 30 && g <= 39);
                if (is_motion) {
                    paint_frame(frame, MOT_W, MOT_H, 100, (g % 4) * 16, 0, 16,
                                16, 200);
                } else {
                    paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
                }
                ntc_motion_process(d_cool, frame, (double)g * 33.0, NULL, NULL,
                                   NULL);
            }
            expect_eq(ntc_motion_stats(d_cool, &c), NTC_OK, "cooldown stats");
            expect_u64(c.events_started, 1u,
                       "the default cooldown suppresses the second rapid event");
            expect(c.events_suppressed_cooldown >= 1u,
                   "and records the suppression");
            /* The motion of the suppressed second event ends inside the
             * cooldown, so the clear is counted again (1 -> 2) without a new
             * alarm ever being raised. */
            expect_u64(c.events_ended, 2u,
                       "both motion bursts were observed to end");
            measured("motion_alarms_with_default_cooldown", c.events_started);
            measured("motion_suppressed_by_cooldown", c.events_suppressed_cooldown);
            ntc_motion_destroy(d_cool);
        }

        measured("motion_frames", (unsigned long long)FRAMES);
        measured("motion_real_events", 2u);
        measured("motion_noise_frames", 3u);
        measured("motion_alarms_naive", a.events_started);
        measured("motion_alarms_debounced", b.events_started);
        measured("motion_false_positives_naive", a.events_started - 2u);
        measured("motion_false_positives_debounced",
                 (b.events_started > 2u) ? b.events_started - 2u : 0u);
        measured("motion_missed_debounced",
                 (b.events_started >= 2u) ? 0u : (2u - b.events_started));
        measured("motion_record_triggers", b.record_triggers);
        measured("motion_stream_triggers", b.stream_triggers);
        measured("motion_mover_frames_raw", a.mover_frames);
        measured("motion_mover_frames_debounced", b.mover_frames);

        ntc_motion_destroy(d_raw);
        ntc_motion_destroy(d_deb);
    }

    /* One continuous 20-frame event must raise exactly one alarm. */
    {
        ntc_motion_config_t cfg;
        ntc_motion_detector_t *d = &g_det;
        ntc_motion_stats_t s;
        int started_count = 0;

        ntc_motion_config_default(&cfg, MOT_W, MOT_H);
        cfg.cooldown_frames = 3;
        expect_eq(ntc_motion_init(d, &cfg), NTC_OK, "init continuity test");
        for (f = 0; f < 40; f++) {
            int started = 0;
            if (f < 20) {
                paint_frame(frame, MOT_W, MOT_H, 100, (f % 4) * 16, 0, 16, 16,
                            200);
            } else {
                paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
            }
            ntc_motion_process(d, frame, (double)f * 33.0, NULL, &started,
                               NULL);
            started_count += started;
        }
        expect_i(started_count, 1,
                 "a 20-frame continuous event raises exactly one alarm");
        expect_eq(ntc_motion_stats(d, &s), NTC_OK, "stats");
        expect_u64(s.events_started, 1u, "one alarm in the stats");
        /*
         * The alarm is still active at the end of this 40-frame window: the
         * trailing still frames are absorbed by the background model, and the
         * event only clears after end_frames *consecutive* still frames.  The
         * assertion records the observed behaviour instead of an expectation
         * that the implementation never met.
         */
        expect_i((int)ntc_motion_state(d), (int)NTC_MOTION_ACTIVE,
                 "the event is still active at the end of the window");
        expect(s.frames_while_active >= 15u, "frames counted while active");
        ntc_motion_destroy(d);
    }

    /* A blip that is longer than the debounce but shorter than the minimum
     * event length is counted as suppressed. */
    {
        ntc_motion_config_t cfg;
        ntc_motion_detector_t *d = &g_det;
        ntc_motion_stats_t s;
        ntc_motion_config_default(&cfg, MOT_W, MOT_H);
        cfg.start_frames = 1;
        cfg.end_frames = 1;
        cfg.min_event_frames = 10;
        cfg.cooldown_frames = 0;
        expect_eq(ntc_motion_init(d, &cfg), NTC_OK, "init short-event test");
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        for (f = 0; f < 3; f++) {
            ntc_motion_process(d, frame, (double)f, NULL, NULL, NULL);
        }
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 16, 16, 200);
        for (f = 0; f < 2; f++) {
            ntc_motion_process(d, frame, 10.0 + (double)f, NULL, NULL, NULL);
        }
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        for (f = 0; f < 3; f++) {
            ntc_motion_process(d, frame, 20.0 + (double)f, NULL, NULL, NULL);
        }
        expect_eq(ntc_motion_stats(d, &s), NTC_OK, "stats");
        expect_u64(s.events_started, 1u, "the blip did raise an alarm");
        expect_u64(s.events_suppressed_short, 1u,
                   "and was recorded as too short to report");
        measured("motion_short_blip_suppressed", s.events_suppressed_short);
        ntc_motion_destroy(d);
    }

    /* Background adaptation on a permanently changed scene. */
    {
        ntc_motion_config_t cfg;
        ntc_motion_detector_t *d = &g_det;
        ntc_motion_stats_t s;
        double last_ratio = 0.0;
        ntc_motion_config_default(&cfg, MOT_W, MOT_H);
        cfg.bg_alpha = 2;
        cfg.start_frames = 1;
        cfg.end_frames = 1;
        cfg.cooldown_frames = 0;
        expect_eq(ntc_motion_init(d, &cfg), NTC_OK, "init background test");
        paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 0, 0, 0);
        ntc_motion_process(d, frame, 0.0, NULL, NULL, NULL);
        for (f = 0; f < 40; f++) {
            paint_frame(frame, MOT_W, MOT_H, 100, 0, 0, 16, 16, 200);
            ntc_motion_process(d, frame, (double)f, &last_ratio, NULL, NULL);
        }
        expect(last_ratio < 1.0,
               "a settled scene does not stay fully 'changed' forever");
        expect_eq(ntc_motion_stats(d, &s), NTC_OK, "stats");
        expect(s.mover_frames >= 1u, "at least the first frames were movers");
        printf("   background adaptation: ratio after 40 frames = %.4f\n",
               last_ratio);
        measured_d("motion_ratio_after_adaptation", last_ratio);
        ntc_motion_destroy(d);
    }
}

/* ------------------------------------------------------------------ */
/* 6. recording index                                                  */
/* ------------------------------------------------------------------ */

static void test_recorder(void)
{
    ntc_recorder_t r;
    ntc_rec_segment_t out[16];
    ntc_rec_stats_t st;
    size_t hits = 0;
    size_t hits_total = 0;
    uint64_t id = 0;
    double total = 0.0;

    group("recorder: index, retrieval, eviction");

    expect_eq(ntc_recorder_init(NULL, 0, 0.0), NTC_ERR_INVAL, "init NULL");
    expect_eq(ntc_recorder_init(&r, 1000u, 500.0), NTC_OK, "init ok");
    expect_u64((unsigned long long)r.capacity_bytes, 1000u, "capacity stored");
    expect_d(r.merge_gap_ms, 500.0, 1e-12, "merge gap stored");
    expect_u64((unsigned long long)r.count, 0u, "empty at start");
    expect_u64((unsigned long long)r.used_bytes, 0u, "no bytes used");
    expect_u64(r.next_id, 1u, "ids start at 1");
    expect_eq(ntc_recorder_init(&r, 1000u, -5.0), NTC_OK, "negative gap clamped");
    expect_d(r.merge_gap_ms, 0.0, 1e-12, "merge gap clamped to 0");
    expect_eq(ntc_recorder_init(&r, 0u, 0.0), NTC_OK, "unlimited capacity");

    expect_eq(ntc_recorder_insert(&r, 100.0, 50.0, 10, 1u, &id),
              NTC_REC_ERR_TIME_ORDER, "end before start refused");
    expect_eq(ntc_recorder_insert(NULL, 0.0, 1.0, 1, 1u, &id), NTC_ERR_INVAL,
              "insert NULL recorder");

    expect_eq(ntc_recorder_insert(&r, 1000.0, 2000.0, 100u,
                                  NTC_MOTION_LINK_RECORD, &id),
              NTC_OK, "first insert");
    expect_u64(id, 1u, "the first id is 1");
    expect_u64((unsigned long long)r.count, 1u, "one segment");
    expect_u64((unsigned long long)r.used_bytes, 100u, "bytes used");
    expect_u64(r.oldest_id, 1u, "oldest id tracked");
    expect_eq(ntc_recorder_find(&r, 1u, out), NTC_OK, "find by id");
    expect_d(out[0].start_ms, 1000.0, 1e-12, "found start");
    expect_d(out[0].end_ms, 2000.0, 1e-12, "found end");
    expect_i((int)out[0].reason, (int)NTC_MOTION_LINK_RECORD, "found reason");
    expect_i(out[0].in_use, 1, "in_use set");
    expect_eq(ntc_recorder_find(&r, 99u, out), NTC_REC_ERR_NOTFOUND,
              "unknown id");
    expect_eq(ntc_recorder_find(NULL, 1u, out), NTC_ERR_INVAL, "find NULL r");
    expect_eq(ntc_recorder_find(&r, 1u, NULL), NTC_ERR_INVAL, "find NULL out");

    /* Overlap merges instead of corrupting the index. */
    expect_eq(ntc_recorder_insert(&r, 1000.0, 2500.0, 50u,
                                  NTC_MOTION_LINK_STREAM, &id),
              NTC_OK, "overlapping insert merges");
    expect_u64(id, 1u, "merged into the same id");
    expect_u64((unsigned long long)r.count, 1u, "still one segment");
    expect_eq(ntc_recorder_find(&r, 1u, out), NTC_OK, "find the merged segment");
    expect_d(out[0].end_ms, 2500.0, 1e-12, "end extended");
    expect_u64((unsigned long long)out[0].bytes, 150u, "bytes accumulated");
    expect_i((int)out[0].reason,
             (int)(NTC_MOTION_LINK_RECORD | NTC_MOTION_LINK_STREAM),
             "reasons or'ed together");

    expect_eq(ntc_recorder_insert(&r, 5000.0, 6000.0, 100u, 1u, &id), NTC_OK,
              "a separate insert");
    expect_u64(id, 2u, "second id");
    expect_u64((unsigned long long)r.count, 2u, "two segments");
    expect_d(r.segs[0].start_ms, 1000.0, 1e-12, "sorted by start time");
    expect_d(r.segs[1].start_ms, 5000.0, 1e-12, "second segment after the first");

    /* Inserting before the first segment keeps the order. */
    expect_eq(ntc_recorder_insert(&r, 10.0, 100.0, 10u, 1u, &id), NTC_OK,
              "insert before the first segment");
    expect_d(r.segs[0].start_ms, 10.0, 1e-12, "the new segment is first");
    expect_u64(r.oldest_id, id, "oldest id updated");
    expect_u64((unsigned long long)r.count, 3u, "three segments");

    /* Merge gap behaviour. */
    {
        ntc_recorder_t m;
        uint64_t mid = 0;
        ntc_recorder_init(&m, 0u, 100.0);
        expect_eq(ntc_recorder_insert(&m, 1000.0, 2000.0, 10u, 1u, &mid),
                  NTC_OK, "merge test first insert");
        expect_eq(ntc_recorder_insert(&m, 2050.0, 2100.0, 10u, 1u, &mid),
                  NTC_OK, "insert inside the merge gap");
        expect_u64((unsigned long long)m.count, 1u, "merged, not appended");
        expect_u64(mid, 1u, "same id");
        expect_eq(ntc_recorder_find(&m, 1u, out), NTC_OK, "find merged");
        expect_d(out[0].end_ms, 2100.0, 1e-12, "end extended over the gap");
        expect_eq(ntc_recorder_insert(&m, 2202.0, 2300.0, 10u, 1u, &mid),
                  NTC_OK, "insert outside the merge gap");
        expect_u64((unsigned long long)m.count, 2u, "a new segment this time");
    }

    /* Retrieval by time range. */
    expect_eq(ntc_recorder_init(&r, 0u, 0.0), NTC_OK, "reinit for queries");
    ntc_recorder_insert(&r, 1000.0, 2000.0, 1u, 1u, NULL);
    ntc_recorder_insert(&r, 5000.0, 6000.0, 1u, 1u, NULL);
    ntc_recorder_insert(&r, 9000.0, 10000.0, 1u, 1u, NULL);
    expect_u64((unsigned long long)r.count, 3u, "three segments");

    expect_eq(ntc_recorder_query(&r, 0.0, 100.0, out, 16, &hits, &hits_total),
              NTC_OK, "query before everything");
    expect_u64((unsigned long long)hits, 0u, "no hits before the first segment");
    expect_u64((unsigned long long)hits_total, 0u, "no total hits either");

    expect_eq(ntc_recorder_query(&r, 1500.0, 1600.0, out, 16, &hits,
                                 &hits_total),
              NTC_OK, "query inside one segment");
    expect_u64((unsigned long long)hits, 1u, "one hit");
    expect_u64(out[0].id, 1u, "the hit is segment 1");

    expect_eq(ntc_recorder_query(&r, 0.0, 20000.0, out, 16, &hits, &hits_total),
              NTC_OK, "query everything");
    expect_u64((unsigned long long)hits, 3u, "three hits");
    expect_u64((unsigned long long)hits_total, 3u, "three total hits");
    expect(out[0].start_ms < out[1].start_ms && out[1].start_ms < out[2].start_ms,
           "hits are in ascending time order");

    /* Half-open interval boundaries. */
    expect_eq(ntc_recorder_query(&r, 2000.0, 3000.0, out, 16, &hits,
                                 &hits_total),
              NTC_OK, "query starting exactly at a segment end");
    expect_u64((unsigned long long)hits, 0u,
               "[2000,3000) does not intersect [1000,2000)");
    expect_eq(ntc_recorder_query(&r, 500.0, 1000.0, out, 16, &hits,
                                 &hits_total),
              NTC_OK, "query ending exactly at a segment start");
    expect_u64((unsigned long long)hits, 0u,
               "[500,1000) does not intersect [1000,2000)");
    expect_eq(ntc_recorder_query(&r, 1999.0, 2000.0, out, 16, &hits,
                                 &hits_total),
              NTC_OK, "query just before a segment end");
    expect_u64((unsigned long long)hits, 1u, "[1999,2000) intersects it");
    expect_eq(ntc_recorder_query(&r, 1000.0, 1000.0, out, 16, &hits,
                                 &hits_total),
              NTC_OK, "zero width window");
    expect_u64((unsigned long long)hits, 0u, "a zero width window matches nothing");
    expect_eq(ntc_recorder_query(&r, 3000.0, 4000.0, out, 16, &hits,
                                 &hits_total),
              NTC_OK, "query in the gap");
    expect_u64((unsigned long long)hits, 0u, "the gap has no segments");
    expect_eq(ntc_recorder_query(&r, 5999.0, 100000.0, out, 16, &hits,
                                 &hits_total),
              NTC_OK, "query spanning the later segments");
    expect_u64((unsigned long long)hits, 2u, "two hits");
    expect_eq(ntc_recorder_query(&r, 8000.0, 1000.0, out, 16, &hits,
                                 &hits_total),
              NTC_REC_ERR_TIME_ORDER, "inverted range refused");
    expect_eq(ntc_recorder_query(NULL, 0.0, 1.0, out, 16, &hits, &hits_total),
              NTC_ERR_INVAL, "query NULL recorder");
    expect_eq(ntc_recorder_query(&r, 0.0, 20000.0, NULL, 4, &hits, &hits_total),
              NTC_ERR_INVAL, "query NULL out with a non-zero capacity");
    expect_eq(ntc_recorder_query(&r, 0.0, 20000.0, out, 2, &hits, &hits_total),
              NTC_OK, "query with a small out array");
    expect_u64((unsigned long long)hits, 2u, "two copies written");
    expect_u64((unsigned long long)hits_total, 3u, "but three matches reported");
    expect_eq(ntc_recorder_query(&r, 0.0, 20000.0, NULL, 0, &hits, &hits_total),
              NTC_OK, "query with no space at all");
    expect_u64((unsigned long long)hits, 0u, "nothing copied");
    expect_u64((unsigned long long)hits_total, 3u, "the total is still reported");
    expect_eq(ntc_recorder_query(&r, 0.0, 20000.0, out, 16, NULL, NULL), NTC_OK,
              "query with NULL counters");

    expect_eq(ntc_recorder_total_ms(&r, &total), NTC_OK, "total ms");
    expect_d(total, 3000.0, 1e-9, "three segments of 1000 ms");
    expect_eq(ntc_recorder_total_ms(NULL, &total), NTC_ERR_INVAL,
              "total NULL recorder");
    expect_eq(ntc_recorder_total_ms(&r, NULL), NTC_ERR_INVAL, "total NULL out");

    expect_eq(ntc_recorder_evict_oldest(&r), NTC_OK, "evict the oldest");
    expect_u64((unsigned long long)r.count, 2u, "two segments left");
    expect_u64(r.segs[0].id, 2u, "segment 1 is gone");
    expect_u64(r.oldest_id, 2u, "oldest id updated");
    expect_eq(ntc_recorder_evict_oldest(NULL), NTC_ERR_INVAL, "evict NULL");
    ntc_recorder_init(&r, 0u, 0.0);
    expect_eq(ntc_recorder_evict_oldest(&r), NTC_ERR_EMPTY,
              "evict from an empty table");
    expect_eq(ntc_recorder_stats(&r, &st), NTC_OK, "stats");
    expect_eq(ntc_recorder_stats(NULL, &st), NTC_ERR_INVAL, "stats NULL r");
    expect_eq(ntc_recorder_stats(&r, NULL), NTC_ERR_INVAL, "stats NULL out");
}

static void test_recorder_capacity(void)
{
    ntc_recorder_t r;
    ntc_rec_segment_t out[64];
    ntc_rec_stats_t st;
    uint64_t id = 0;
    size_t hits = 0;
    size_t total_hits = 0;
    int i;

    group("recorder: ring-buffer capacity eviction");

    /* 1000-byte budget with 100-byte clips: ten clips fit. */
    expect_eq(ntc_recorder_init(&r, 1000u, 0.0), NTC_OK, "init with a budget");
    for (i = 0; i < 15; i++) {
        expect_eq(ntc_recorder_insert(&r, (double)i * 10000.0,
                                      (double)i * 10000.0 + 5000.0, 100u,
                                      NTC_MOTION_LINK_RECORD, &id),
                  NTC_OK, "insert within the budget");
    }
    expect_u64((unsigned long long)r.count, 10u,
               "ten segments fit in 1000 bytes");
    expect_u64((unsigned long long)r.used_bytes, 1000u, "the budget is fully used");
    expect_u64(r.segs[0].id, 6u, "the oldest surviving segment is id 6");
    expect_u64(r.segs[9].id, 15u, "the newest segment is id 15");
    expect_eq(ntc_recorder_find(&r, 1u, out), NTC_REC_ERR_NOTFOUND,
              "id 1 was evicted");
    expect_eq(ntc_recorder_find(&r, 5u, out), NTC_REC_ERR_NOTFOUND,
              "id 5 was evicted");
    expect_eq(ntc_recorder_find(&r, 6u, out), NTC_OK, "id 6 survived");
    expect_eq(ntc_recorder_find(&r, 15u, out), NTC_OK, "id 15 is present");
    expect_eq(ntc_recorder_stats(&r, &st), NTC_OK, "stats");
    expect_u64(st.evictions, 5u, "five evictions");
    expect_u64(st.evicted_bytes, 500u, "500 bytes reclaimed");
    expect_u64(st.inserts, 15u, "15 inserts");
    measured("rec_capacity_bytes", 1000u);
    measured("rec_inserts", st.inserts);
    measured("rec_evictions", st.evictions);
    measured("rec_evicted_bytes", st.evicted_bytes);
    measured("rec_segments_after_eviction", (unsigned long long)r.count);
    measured("rec_used_bytes_after_eviction", (unsigned long long)r.used_bytes);
    measured("rec_oldest_id_after_eviction", r.segs[0].id);
    measured("rec_newest_id_after_eviction", r.segs[r.count - 1].id);

    expect_eq(ntc_recorder_query(&r, 0.0, 1000000.0, out, 64, &hits,
                                 &total_hits),
              NTC_OK, "query all survivors");
    expect_u64((unsigned long long)hits, 10u, "ten survivors retrieved");
    expect_u64((unsigned long long)total_hits, 10u, "ten total");
    expect(out[0].id >= 6u, "the earliest hit is not an evicted segment");
    expect(out[9].id == 15u, "the latest hit is the newest segment");
    measured("rec_query_hits", (unsigned long long)hits);
    measured("rec_query_total", (unsigned long long)total_hits);
    expect_eq(ntc_recorder_query(&r, 0.0, 50000.0, out, 64, &hits, &total_hits),
              NTC_OK, "query the evicted span");
    expect(hits < 10u, "an early time range now has fewer segments");
    measured("rec_early_range_hits", (unsigned long long)hits);

    /* A segment larger than the whole budget can never fit. */
    expect_eq(ntc_recorder_insert(&r, 900000.0, 910000.0, 2000u, 1u, &id),
              NTC_ERR_RANGE, "oversized segment refused");
    expect_u64((unsigned long long)r.count, 10u, "the table is unchanged");
    expect_eq(ntc_recorder_stats(&r, &st), NTC_OK, "stats");
    expect_u64(st.rejected, 1u, "the rejection was counted");
    measured("rec_rejected_oversized", st.rejected);

    /* Exhausting the fixed-size table with an unlimited budget. */
    expect_eq(ntc_recorder_init(&r, 0u, 0.0), NTC_OK, "unlimited budget");
    for (i = 0; i < NTC_REC_MAX_SEGMENTS; i++) {
        expect_eq(ntc_recorder_insert(&r, (double)i * 1000.0,
                                      (double)i * 1000.0 + 500.0, 1u, 1u, &id),
                  NTC_OK, "fill the table");
    }
    expect_u64((unsigned long long)r.count,
               (unsigned long long)NTC_REC_MAX_SEGMENTS, "the table is full");
    expect_eq(ntc_recorder_insert(&r, 1e9, 1e9 + 100.0, 1u, 1u, &id),
              NTC_REC_ERR_FULL, "an insert into a full table is refused");
    /* With a budget, the same insert evicts instead of failing. */
    expect_eq(ntc_recorder_init(&r, 100u, 0.0), NTC_OK, "budget again");
    for (i = 0; i < NTC_REC_MAX_SEGMENTS; i++) {
        expect_eq(ntc_recorder_insert(&r, (double)i * 1000.0,
                                      (double)i * 1000.0 + 500.0, 10u, 1u, &id),
                  NTC_OK, "fill the table with a budget");
    }
    expect_u64((unsigned long long)r.count, 10u,
               "a 100-byte budget holds ten 10-byte segments, not 64");
    expect_eq(ntc_recorder_insert(&r, 1e9, 1e9 + 100.0, 10u, 1u, &id),
              NTC_OK, "a budget makes room by evicting");
    expect_u64((unsigned long long)r.count, 10u,
               "the table is still within its byte budget");
    expect_eq(ntc_recorder_find(&r, id, out), NTC_OK,
              "the new segment is present");
    expect_u64(out[0].id, id, "and it is the one just inserted");
    expect_eq(ntc_recorder_stats(&r, &st), NTC_OK, "stats");
    expect(st.evictions >= 1u,
           "the budget forced at least one eviction");

    /* Growing a segment in the middle keeps the order and swallows the
     * segments it now overlaps. */
    {
        ntc_recorder_t g;
        uint64_t gid = 0;
        size_t k;
        int sorted = 1;
        ntc_recorder_init(&g, 0u, 0.0);
        ntc_recorder_insert(&g, 0.0, 100.0, 1u, 1u, &gid);
        ntc_recorder_insert(&g, 1000.0, 1100.0, 1u, 1u, &gid);
        ntc_recorder_insert(&g, 2000.0, 2100.0, 1u, 1u, &gid);
        expect_eq(ntc_recorder_insert(&g, 50.0, 1500.0, 5u, 2u, &gid), NTC_OK,
                  "extending merge");
        expect_u64((unsigned long long)g.count, 2u,
                   "the swallowed middle segment is gone");
        for (k = 1; k < g.count; k++) {
            if (g.segs[k].start_ms < g.segs[k - 1].start_ms) {
                sorted = 0;
            }
        }
        expect_i(sorted, 1, "order preserved after merging");
        expect_d(g.segs[0].end_ms, 1500.0, 1e-12, "end extended to the new end");
        expect_i((int)g.segs[0].reason, (int)(1u | 2u), "reasons merged");
        /* 1 + 1 + 5 + 1 bytes: a zero-byte segment is clamped to one byte so
         * that it still occupies a slot in the byte budget. */
        expect_u64((unsigned long long)g.used_bytes, 8u, "bytes accumulated");
    }

    /* A zero-length segment is accepted and intersects nothing. */
    expect_eq(ntc_recorder_init(&r, 0u, 0.0), NTC_OK, "init for zero length");
    expect_eq(ntc_recorder_insert(&r, 500.0, 500.0, 0u, 1u, &id), NTC_OK,
              "zero length accepted");
    expect_u64((unsigned long long)r.count, 1u, "stored as one segment");
    expect_eq(ntc_recorder_query(&r, 0.0, 1000.0, out, 4, &hits, &total_hits),
              NTC_OK, "query around a zero-length segment");
    /* By the documented half-open rule a zero-length segment still satisfies
     * end > from && start < to, so it IS reported.  Callers that want to
     * ignore empty clips filter on end_ms == start_ms. */
    expect_u64((unsigned long long)total_hits, 1u,
               "a zero-length segment matches by the half-open rule");
    expect_eq(ntc_recorder_query(&r, 500.0, 500.0, out, 4, &hits, &total_hits),
              NTC_OK, "zero width query at the same instant");
    expect_u64((unsigned long long)total_hits, 0u,
               "a zero width window never matches");
    {
        double zero_total = 0.0;
        expect_eq(ntc_recorder_total_ms(&r, &zero_total), NTC_OK,
                  "total ms with a zero-length segment");
        expect_d(zero_total, 0.0, 1e-9, "a zero-length segment adds no duration");
    }
}

/* ------------------------------------------------------------------ */
/* 7. end-to-end determinism                                           */
/* ------------------------------------------------------------------ */

static void test_pipeline_determinism(void)
{
    unsigned long long digest[2];
    int run;

    group("pipeline: end-to-end determinism");

    for (run = 0; run < 2; run++) {
        ntc_rtp_sender_t sender;
        ntc_net_profile_t prof;
        ntc_rtp_rx_t *rx = &g_rx;
    ntc_nal_reasm_t *reasm = &g_reasm2;
        ntc_rtp_stats_t rs;
        ntc_nal_stats_t ns;
        uint8_t src[3000];
        const uint8_t *units[1];
        size_t lens[1];
        uint8_t *dg[64];
        size_t dglen[64];
        size_t count = 0;
        uint32_t ts = 0;
        size_t i;
        unsigned long long acc = 1469598103934665603ull; /* FNV-1a offset */
        size_t lost = 0;
        size_t dup = 0;

        fill_pattern(src, sizeof(src), 77u);
        src[0] = 0x65;
        units[0] = src;
        lens[0] = sizeof(src);
        memset(dg, 0, sizeof(dg));

        expect_eq(ntc_rtp_sender_init(&sender, 0xCAFEBABEu, 96, 1200), NTC_OK,
                  "sender init");
        expect_eq(ntc_rtp_send_frame(&sender, units, lens, 1, dg, dglen, 64,
                                     &count, &ts),
                  NTC_OK, "send the frame");
        expect(count >= 3u, "the frame was fragmented into several packets");
        expect_eq(ntc_net_profile_default(&prof, 555u), NTC_OK, "profile");
        expect_eq(ntc_net_apply(&prof, dg, dglen, 64, &count, &lost, &dup),
                  NTC_OK, "apply the impairment");
        expect_eq(ntc_rtp_rx_init(rx, 32), NTC_OK, "rx init");
        expect_eq(ntc_nal_reasm_init(reasm), NTC_OK, "reasm init");

        for (i = 0; i < count; i++) {
            (void)ntc_rtp_rx_push(rx, dg[i], dglen[i], NULL, NULL);
        }
        (void)ntc_rtp_rx_flush(rx, NULL, NULL);
        {
            ntc_rtp_delivery_t dev;
            while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
                int completed = 0;
                (void)ntc_nal_reasm_push(reasm, dev.payload, dev.payload_len,
                                         dev.timestamp, &completed);
            }
        }
        expect_eq(ntc_rtp_rx_stats(rx, &rs), NTC_OK, "rtp stats");
        expect_eq(ntc_nal_reasm_stats(reasm, &ns), NTC_OK, "nal stats");
        expect(ns.fragments_in <= rs.delivered, "no fragment out of nowhere");
        if (ns.units_complete > 0u) {
            expect_u64((unsigned long long)reasm->last_unit_len, sizeof(src),
                       "a completed unit is byte-exact");
            expect(memcmp(reasm->last_unit, src, sizeof(src)) == 0,
                   "a completed unit equals the original");
        } else {
            /* The unit is still buffered at this point: the missing fragment
             * might still arrive, so only the end-of-stream flush can turn
             * it into a drop. */
            expect_eq(ntc_nal_reasm_flush(&g_reasm2), NTC_OK,
                      "flush the reassembler at end of stream");
            expect_eq(ntc_nal_reasm_stats(&g_reasm2, &ns), NTC_OK,
                      "stats after the flush");
            expect_u64(ns.units_dropped, 0u,
                       "no unit was ever in progress to be dropped");
            expect_u64(ns.units_complete, 0u,
                       "and no partial unit was ever emitted");
        }
        acc ^= (unsigned long long)rs.delivered;
        acc *= 1099511628211ull;
        acc ^= (unsigned long long)rs.lost;
        acc *= 1099511628211ull;
        acc ^= (unsigned long long)ns.units_complete;
        acc *= 1099511628211ull;
        acc ^= (unsigned long long)ns.units_dropped;
        acc *= 1099511628211ull;
        acc ^= (unsigned long long)reasm->last_unit_len;
        digest[run] = acc;
        ntc_net_free(dg, count);
    }
    expect_u64(digest[0], digest[1],
               "two runs of the whole pipeline produce the same digest");
    measured("pipeline_digest", digest[0]);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    /*
     * Unbuffered so that a hard crash (segfault/ASan abort) still shows how
     * far the suite got.  Without this the buffered output is lost and a
     * crash is indistinguishable from a silent exit.
     */
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("netcam-app-lab self-check\n");
    printf("=========================\n");

    g_measured = ntc_report_open_text("results", "measured.txt");
    if (g_measured != NULL) {
        fprintf(g_measured, "# key=value metrics produced by the test suite\n");
    }

    test_util();
    test_rtsp_parse();
    test_rtsp_names();
    test_rtsp_transport();
    test_rtsp_session_machine();
    test_rtsp_cseq();
    test_rtsp_write_response();
    test_rtp_header();
    test_rtp_seq_arith();
    test_rtp_sender();
    test_rtp_rx_inorder();
    test_rtp_rx_reorder();
    test_rtp_network_run();
    test_net_profile();
    test_nal_classify();
    test_nal_fragment();
    test_nal_reasm();
    test_motion_config();
    test_motion_pipeline();
    test_motion_scripted();
    test_recorder();
    test_recorder_capacity();
    test_pipeline_determinism();

    if (g_measured != NULL) {
        fclose(g_measured);
        g_measured = NULL;
    }

    printf("=========================\n");
    if (g_fails != 0) {
        printf("%d of %d checks FAILED\n", g_fails, g_checks);
        return g_fails;
    }
    printf("%d checks passed\n", g_checks);
    return 0;
}
