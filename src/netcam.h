/*
 * netcam.h -- netcam-app-lab public interface (the delivery contract)
 * ==================================================================
 *
 * netcam-app-lab simulates the APPLICATION / PROTOCOL layer of a network
 * camera (IPC) on a host PC:
 *
 *   RTSP session setup -> RTP packetisation -> network (loss / reorder /
 *   duplicate) -> receiver reordering & NAL reassembly -> motion detection
 *   alarm state machine -> recording index with ring-buffer eviction.
 *
 * SCOPE / HONEST BOUNDARIES (see README.md for the long version)
 * -------------------------------------------------------------
 *   * Pure logic + numeric simulation. There is NO camera sensor, NO ISP,
 *     NO real H.264/HEVC encoder, NO sockets, NO real network, NO RTCP.
 *   * The protocols implemented are a *common subset* of the public specs:
 *       - RTSP: request parsing for OPTIONS / DESCRIBE / SETUP / PLAY /
 *         PAUSE / TEARDOWN, CSeq bookkeeping, Session handling and a
 *         session state machine. No full SDP negotiation, no auth, no
 *         SRTP, no ONVIF.
 *       - RTP: fixed 12-byte header (V/P/X/CC, M, PT, seq, timestamp,
 *         SSRC) packing/unpacking, jitter/reorder buffer, loss and
 *         duplicate detection. No RTCP SR/RR reports, no feedback.
 *       - NAL: a "FU-A style" fragmentation scheme (1-byte FU indicator +
 *         1-byte FU header with S/E/R/Type bits) over the RTP payload.
 *         This is *not* an H.264 conformance implementation: no SPS/PPS
 *         parsing, no real slice syntax, no decoder.
 *   * Not a product: host-only, single threaded, no realtime or memory
 *     bandwidth validation.
 *
 * CONVENTIONS
 * -----------
 *   * C99, standard library only (string.h / stdio.h / stdlib.h / math.h).
 *   * Every function that can fail returns an ntc_status_t.
 *   * Functions that "return a value" write it through an out pointer and
 *     return a status; out pointers are validated (NULL -> NTC_ERR_INVAL).
 *   * All structs are plain old data, no hidden heap ownership except where
 *     a create/init function explicitly says so.
 *   * Unsigned sequence numbers wrap modulo 2^32 and are handled with
 *     "RFC 1982 style" wraparound-safe comparison (see ntc_rtp_seq_lt).
 */

#ifndef NETCAM_H
#define NETCAM_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Library version                                                      */
/* ------------------------------------------------------------------ */

#define NTC_VERSION_MAJOR 1
#define NTC_VERSION_MINOR 0
#define NTC_VERSION_PATCH 0

/* ------------------------------------------------------------------ */
/* Status codes                                                         */
/*                                                                      */
/*   0        NTC_OK                                                    */
/*   -1..-99   generic / argument errors                                */
/*   -100..   RTSP                                                      */
/*   -200..   RTP                                                       */
/*   -300..   NAL                                                       */
/*   -400..   motion                                                    */
/*   -500..   recorder                                                  */
/* ------------------------------------------------------------------ */

typedef enum ntc_status {
    NTC_OK                      = 0,

    NTC_ERR_INVAL               = -1,  /* NULL pointer / out-of-range arg  */
    NTC_ERR_NOMEM               = -2,  /* internal capacity exhausted      */
    NTC_ERR_PARSE               = -3,  /* malformed input text            */
    NTC_ERR_RANGE               = -4,  /* value outside allowed range     */
    NTC_ERR_FULL                = -5,  /* container full, caller must drain*/
    NTC_ERR_EMPTY               = -6,  /* container empty                 */
    NTC_ERR_NOTFOUND            = -7,  /* lookup miss                     */
    NTC_ERR_UNSUPPORTED         = -8,  /* syntactically fine, not in scope*/
    NTC_ERR_NO_ROOM             = -9,  /* caller-supplied buffer too small*/

    /* RTSP ---------------------------------------------------------- */
    NTC_RTSP_ERR_METHOD         = -100, /* unknown / unsupported method    */
    NTC_RTSP_ERR_VERSION        = -101, /* not RTSP/1.0                    */
    NTC_RTSP_ERR_CSEQ_MISSING   = -102, /* no CSeq header                  */
    NTC_RTSP_ERR_CSEQ_BAD        = -103,/* CSeq not a number               */
    NTC_RTSP_ERR_CSEQ_ORDER     = -104, /* CSeq <= last seen (replay/oor)  */
    NTC_RTSP_ERR_SESSION_MISSING= -105, /* method requires Session header   */
    NTC_RTSP_ERR_SESSION_INVALID= -106, /* Session header failed validation*/
    NTC_RTSP_ERR_STATE           = -107,/* method illegal in this state    */
    NTC_RTSP_ERR_TRANSPORT      = -108, /* missing/invalid Transport header*/
    NTC_RTSP_ERR_SESSION_TABLE  = -109, /* session table full              */

    /* RTP ---------------------------------------------------------- */
    NTC_RTP_ERR_SHORT           = -200, /* buffer < 12 bytes               */
    NTC_RTP_ERR_VERSION         = -201, /* version field != 2              */
    NTC_RTP_ERR_CSRC            = -202, /* CC > 15 or header overruns buf  */
    NTC_RTP_ERR_PAYLOAD_TYPE    = -203, /* PT mismatch with expectation    */
    NTC_RTP_ERR_NO_ROOM         = -204, /* caller buffer too small         */
    NTC_RTP_ERR_SEQ_MISMATCH    = -205, /* buffer logic: unexpected seq    */

    /* NAL ---------------------------------------------------------- */
    NTC_NAL_ERR_BAD_FU          = -300, /* FU header / indicator malformed */
    NTC_NAL_ERR_SEQUENCE        = -301, /* fragment arrived out of order   */
    NTC_NAL_ERR_GAP             = -302, /* fragment missing -> frame drop  */
    NTC_NAL_ERR_NO_ROOM         = -303, /* reassembly buffer too small     */
    NTC_NAL_ERR_NOT_STARTED     = -304, /* no unit in progress             */
    NTC_NAL_ERR_BAD_TYPE        = -305, /* forbidden / reserved NAL type   */

    /* motion ------------------------------------------------------- */
    NTC_MOT_ERR_FRAME_SIZE      = -400, /* dims mismatch with state        */
    NTC_MOT_ERR_CONFIG          = -401, /* nonsensical config values       */

    /* recorder ----------------------------------------------------- */
    NTC_REC_ERR_FULL            = -500, /* segment table full              */
    NTC_REC_ERR_OVERLAP         = -501, /* new segment overlaps an old one */
    NTC_REC_ERR_TIME_ORDER      = -502, /* end_ms < start_ms               */
    NTC_REC_ERR_NOTFOUND        = -503  /* segment id not present          */
} ntc_status_t;

/* ------------------------------------------------------------------ */
/* Small shared helpers                                                 */
/* ------------------------------------------------------------------ */

/* Human readable name of a status code; never NULL, never allocates. */
const char *ntc_status_str(ntc_status_t st);

/* ------------------------------------------------------------------ */
/* Module 1: RTSP session layer                       (src/rtsp.c)     */
/* ------------------------------------------------------------------ */
/*
 * WIRE FORMAT (RFC 2326 subset)
 *
 *   Request-Line  = Method SP Request-URI SP "RTSP/1.0" CRLF
 *   Status-Line   = "RTSP/1.0" SP Status-Code SP Reason-Phrase CRLF
 *   message       = *( header CRLF ) CRLF [ body ]
 *   header        = field-name ":" OWS field-value OWS CRLF
 *
 * Headers we care about:
 *   CSeq: <uint>            -- REQUIRED on every request; MUST increase
 *                              strictly monotonically inside a session.
 *                              A replayed or out-of-order CSeq is rejected
 *                              with 400 Bad Request (see ntc_rtsp_handle).
 *   Session: <uint>[;timeout=n]  -- REQUIRED for PLAY/PAUSE/TEARDOWN.
 *   Transport: RTP/AVP/TCP;unicast;interleaved=0-1
 *              RTP/AVP;unicast;client_port=8000-8001
 *
 * SESSION STATE MACHINE
 *
 *   INIT --SETUP(aggregate ok)--> READY --PLAY--> PLAYING
 *     ^                             ^               |
 *     |                             +-----PAUSE-----+
 *     +-------------TEARDOWN--------+ (from any state)
 *
 *   DESCRIBE and OPTIONS are legal in every state.
 *   SETUP     is legal only in INIT.
 *   PLAY      is legal only in READY.
 *   PAUSE     is legal only in PLAYING.
 *   TEARDOWN  is legal in READY and PLAYING (not in INIT).
 *   Anything else -> 455 Method Not Valid In This State.
 *   Unknown Session -> 454 Session Not Found.
 *   CSeq problem    -> 400 Bad Request.
 */

#define NTC_RTSP_MAX_METHOD 16
#define NTC_RTSP_MAX_URI 256
#define NTC_RTSP_MAX_VERSION 16
#define NTC_RTSP_MAX_HEADERS 24
#define NTC_RTSP_MAX_NAME 32
#define NTC_RTSP_MAX_VALUE 256
#define NTC_RTSP_MAX_SESSIONS 8
#define NTC_RTSP_MAX_SESSION_STR 64

typedef enum ntc_rtsp_method {
    NTC_RTSP_OPTIONS = 0,
    NTC_RTSP_DESCRIBE,
    NTC_RTSP_SETUP,
    NTC_RTSP_PLAY,
    NTC_RTSP_PAUSE,
    NTC_RTSP_TEARDOWN,
    NTC_RTSP_UNKNOWN_METHOD
} ntc_rtsp_method_t;

typedef enum ntc_rtsp_state {
    NTC_RTSP_STATE_INIT = 0,
    NTC_RTSP_STATE_READY,
    NTC_RTSP_STATE_PLAYING,
    NTC_RTSP_STATE_TORN_DOWN
} ntc_rtsp_state_t;

typedef struct ntc_rtsp_header {
    char name[NTC_RTSP_MAX_NAME];
    char value[NTC_RTSP_MAX_VALUE];
} ntc_rtsp_header_t;

/*
 * Parsed RTSP request, fully owned by value (no pointers into the caller's
 * buffer), so the request text may be freed after ntc_rtsp_parse_request().
 *
 *   cseq       - 0 when absent or unparsable; cseq_present says which.
 *   session    - numeric session id, 0 when absent.
 *   transport  - raw Transport header value; parsed by
 *                ntc_rtsp_parse_transport() on demand.
 *   nheaders   - number of headers actually stored (<= NTC_RTSP_MAX_HEADERS).
 *   header_overflow - 1 if the message had more headers than we could store.
 */
typedef struct ntc_rtsp_request {
    ntc_rtsp_method_t method;
    char method_text[NTC_RTSP_MAX_METHOD];
    char uri[NTC_RTSP_MAX_URI];
    char version[NTC_RTSP_MAX_VERSION];

    uint32_t cseq;
    int cseq_present;

    uint32_t session;
    int session_present;

    char transport[NTC_RTSP_MAX_VALUE];

    char user_agent[NTC_RTSP_MAX_VALUE];
    char content_type[NTC_RTSP_MAX_VALUE];

    ntc_rtsp_header_t headers[NTC_RTSP_MAX_HEADERS];
    size_t nheaders;
    int header_overflow;

    size_t content_length;
    size_t header_bytes; /* offset just past the CRLF CRLF terminator  */
} ntc_rtsp_request_t;

/*
 * Session record.  Created by the first SETUP that carries CSeq-only
 * (no Session header); identified by `id` afterwards.  `last_cseq` is the
 * highest CSeq accepted so far; a new request must satisfy cseq > last_cseq
 * (strict monotonic), otherwise it is rejected with NTC_RTSP_ERR_CSEQ_ORDER.
 */
typedef struct ntc_rtsp_session {
    uint32_t id;
    int in_use;
    ntc_rtsp_state_t state;
    uint32_t last_cseq;
    size_t requests_seen;
    size_t requests_rejected;
    double last_activity_ms; /* caller supplied clock, see ntc_rtsp_play */
} ntc_rtsp_session_t;

/*
 * Session manager (table of NTC_RTSP_MAX_SESSIONS sessions).
 *   next_id    - monotonically increasing session id source, starts at base.
 *   next_cseq  - the manager's own next outbound CSeq (responses/keepalive).
 *   stats      - accepted/rejected counters per reason, used by tests+sim.
 */
typedef struct ntc_rtsp_stats {
    size_t requests_total;
    size_t accepted;
    size_t rejected_cseq_order;
    size_t rejected_cseq_missing;
    size_t rejected_cseq_bad;
    size_t rejected_session_missing;
    size_t rejected_session_invalid;
    size_t rejected_state;     /* 455 */
    size_t rejected_transport; /* 461-ish, we answer 400 */
    size_t rejected_method;    /* 405 */
    size_t sessions_created;
    size_t sessions_torn_down;
    size_t plays;
    size_t pauses;
    size_t setps;
} ntc_rtsp_stats_t;

typedef struct ntc_rtsp_server {
    ntc_rtsp_session_t sessions[NTC_RTSP_MAX_SESSIONS];
    uint32_t next_id;
    uint32_t base_id;
    uint32_t next_cseq; /* outbound */
    ntc_rtsp_stats_t stats;
    char server_name[NTC_RTSP_MAX_VALUE];
} ntc_rtsp_server_t;

/*
 * Result of handling one request: the status line we would put on the wire.
 *   reason  - literal reason phrase, e.g. "OK", "Session Not Found".
 *   status  - numeric code, 200 / 400 / 404 / 405 / 454 / 455 / 461.
 *   session_id - session assigned by SETUP, else the request's session.
 *   body    - optional response body (DESCRIBE SDP-like stub), may be "".
 */
typedef struct ntc_rtsp_response {
    int status;
    char reason[64];
    uint32_t cseq;
    uint32_t session_id;
    char body[512];
    size_t body_len;
    char extra_header_name[NTC_RTSP_MAX_NAME];
    char extra_header_value[NTC_RTSP_MAX_VALUE];
} ntc_rtsp_response_t;

/*
 * Parsed Transport header.
 *   mode            - 0 = UDP (client_port), 1 = TCP interleaved.
 *   client_rtp_port / client_rtcp_port - UDP mode, 0 when absent.
 *   interleaved_rtp / interleaved_rtcp - TCP mode, -1 when absent.
 *   has_client_port / has_interleaved   - presence flags.
 */
typedef struct ntc_rtsp_transport {
    int mode;
    char protocol[32]; /* "RTP/AVP" or "RTP/AVP/TCP" */
    int has_client_port;
    uint16_t client_rtp_port;
    uint16_t client_rtcp_port;
    int has_interleaved;
    int interleaved_rtp;
    int interleaved_rtcp;
} ntc_rtsp_transport_t;

/*
 * Initialise a session manager.
 *   base_id    - first session id handed out (0 is remapped to 1).
 *   next_cseq  - first outbound CSeq (0 is remapped to 1).
 *   server_name- copied into the Server header; NULL -> "netcam-app-lab".
 * Returns NTC_ERR_INVAL if srv is NULL.
 */
ntc_status_t ntc_rtsp_server_init(ntc_rtsp_server_t *srv, uint32_t base_id,
                                  uint32_t next_cseq, const char *server_name);

/*
 * Parse an RTSP request from `text` (need not be NUL terminated; `len`
 * bytes are examined).  Accepts CRLF or bare LF line endings.
 *
 * Populates *out on success.  Returns:
 *   NTC_OK                   parsed; method_text may still be unknown.
 *   NTC_ERR_INVAL            out == NULL.
 *   NTC_ERR_PARSE            request line malformed / no terminator.
 *   NTC_RTSP_ERR_VERSION     version token is not "RTSP/1.0".
 *   NTC_RTSP_ERR_CSEQ_BAD    CSeq present but not a decimal number.
 *   NTC_RTSP_ERR_METHOD      method token empty or too long.
 *
 * Note: an *unknown but well formed* method yields
 * out->method == NTC_RTSP_UNKNOWN_METHOD and NTC_OK here; the 405 decision
 * is made by ntc_rtsp_handle() so that parsing and policy stay separable.
 */
ntc_status_t ntc_rtsp_parse_request(const char *text, size_t len,
                                    ntc_rtsp_request_t *out);

/* Map a method token to the enum; NTC_RTSP_UNKNOWN_METHOD if not recognised. */
ntc_rtsp_method_t ntc_rtsp_method_from_text(const char *text);

/* Canonical method token, e.g. "PLAY"; "UNKNOWN" for the unknown enum. */
const char *ntc_rtsp_method_str(ntc_rtsp_method_t m);

/* Canonical state name: "INIT" / "READY" / "PLAYING" / "TORN_DOWN". */
const char *ntc_rtsp_state_str(ntc_rtsp_state_t s);

/* Parse a "Transport:" value.  Returns NTC_ERR_PARSE / NTC_ERR_INVAL. */
ntc_status_t ntc_rtsp_parse_transport(const char *value,
                                     ntc_rtsp_transport_t *out);

/*
 * Stateless policy question: is `m` allowed in state `s`?
 * Returns 1 (allowed) or 0 (not allowed).  Unknown method -> 0.
 */
int ntc_rtsp_method_allowed(ntc_rtsp_method_t m, ntc_rtsp_state_t s);

/*
 * Validate a request against a session and produce the response.
 *
 * Order of validation (the first failure wins, which is what an IPC that
 * wants deterministic behaviour should do):
 *   1. unknown method                          -> 405 + stats.rejected_method
 *   2. CSeq missing                            -> 400
 *   3. CSeq not strictly greater than last     -> 400  (this is the
 *      "CSeq reordering / replay" rejection)  -> stats.rejected_cseq_order
 *   4. method needs a session but none given   -> 454
 *   5. session id unknown                      -> 454
 *   6. method illegal in the session state     -> 455
 *   7. SETUP without a usable Transport header -> 400
 *
 * On success the session is updated (state transition, last_cseq, counters)
 * and the response is filled in.  `now_ms` is only stored for bookkeeping
 * (no timers are run by this module).
 *
 * `req->session_present` with an id that the caller obtained out of band is
 * still validated against the table.
 */
ntc_status_t ntc_rtsp_handle(ntc_rtsp_server_t *srv,
                             const ntc_rtsp_request_t *req,
                             double now_ms,
                             ntc_rtsp_response_t *resp);

/* Convenience: parse + handle in one call (used by the end-to-end sim). */
ntc_status_t ntc_rtsp_handle_text(ntc_rtsp_server_t *srv, const char *text,
                                  size_t len, double now_ms,
                                  ntc_rtsp_response_t *resp,
                                  ntc_rtsp_request_t *parsed_or_null);

/*
 * Render `resp` as an RTSP response message into `buf` (NUL terminated when
 * there is room).  Returns the number of bytes that the full message needs;
 * if it exceeds cap, *written is set to cap-1 and NTC_ERR_NOMEM is returned.
 * `written` may be NULL.
 */
ntc_status_t ntc_rtsp_write_response(const ntc_rtsp_response_t *resp,
                                     char *buf, size_t cap, size_t *written);

/*
 * Session-table queries.  Both return NTC_OK and write through the out
 * pointer, or NTC_ERR_NOTFOUND / NTC_ERR_INVAL.
 */
ntc_status_t ntc_rtsp_find_session(const ntc_rtsp_server_t *srv, uint32_t id,
                                   const ntc_rtsp_session_t **out);
ntc_status_t ntc_rtsp_session_count(const ntc_rtsp_server_t *srv, size_t *out);

/* Force a session to TORN_DOWN and free its slot.  NTC_ERR_NOTFOUND if
 * the id is not live. */
ntc_status_t ntc_rtsp_close_session(ntc_rtsp_server_t *srv, uint32_t id);

/* ------------------------------------------------------------------ */
/* Module 2: RTP packetisation + receiver reordering   (src/rtp.c)     */
/* ------------------------------------------------------------------ */
/*
 * RTP FIXED HEADER (RFC 3550), 12 bytes when CC == 0
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
 *   |  CC * 32-bit CSRC identifiers (only when CC > 0)              |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * All multi-byte fields are big endian (network byte order).  Our packer
 * always emits X=0, P=0, CC=0, i.e. exactly 12 bytes of header.
 *
 * RECEIVER REORDER BUFFER
 *
 *   Packets are inserted by sequence number.  An entry is "released" (moved
 *   to the delivered list, in ascending seq order) as soon as the buffer
 *   holds a contiguous run starting at expected_seq, or when the buffer is
 *   full (window pressure).  A packet whose seq is further than
 *   `reorder_window` ahead of expected_seq forces a flush of everything
 *   below it, which is the mechanism by which "window too small" turns
 *   late packets into declared losses -- see ntc_rtp_rx_flush.
 *
 * SEQUENCE NUMBER ARITHMETIC
 *
 *   Sequence numbers are 16-bit RTP seq; timestamps and SSRC are 32-bit.
 *   ntc_rtp_seq_lt() compares two uint16_t with wraparound:
 *       lt(a,b) <- ((int16_t)(a - b)) < 0
 *   which is correct as long as the two are within 32767 of each other.
 *   The simulation additionally carries `ext_seq` (a 64-bit monotonically
 *   increasing counter) so that tests can reason about long streams without
 *   writing wrap-aware loops everywhere.
 */

#define NTC_RTP_HEADER_LEN 12
#define NTC_RTP_MAX_PAYLOAD 1400
#define NTC_RTP_MAX_DATAGRAM (NTC_RTP_HEADER_LEN + NTC_RTP_MAX_PAYLOAD)
#define NTC_RTP_DEFAULT_PT 96 /* dynamic payload type for H.264 */
#define NTC_RTP_MAX_REORDER 512

/* Wraparound-safe 16-bit "a < b". */
int ntc_rtp_seq_lt(uint16_t a, uint16_t b);
/* Wraparound-safe 16-bit "a > b". */
int ntc_rtp_seq_gt(uint16_t a, uint16_t b);
/* Wraparound-safe forward distance b - a as an unsigned 16-bit count. */
uint16_t ntc_rtp_seq_diff(uint16_t a, uint16_t b);

typedef struct ntc_rtp_packet {
    uint8_t version;
    uint8_t padding;
    uint8_t extension;
    uint8_t csrc_count;
    uint8_t marker;
    uint8_t payload_type;
    uint16_t seq;
    uint32_t timestamp;
    uint32_t ssrc;
    size_t payload_len;
    const uint8_t *payload; /* points into the caller's buffer */
} ntc_rtp_packet_t;

/*
 * Write a 12-byte RTP header into buf (cap >= 12).
 *   marker - 0/1, payload_type - typically 96, ssrc/seq/timestamp are
 *   emitted big endian.  Always emits V=2,P=0,X=0,CC=0.
 * Returns NTC_OK / NTC_ERR_INVAL / NTC_ERR_NO_ROOM.
 * Does NOT append a payload; use ntc_rtp_build() for that.
 */
ntc_status_t ntc_rtp_write_header(uint8_t *buf, size_t cap, int marker,
                                  uint8_t payload_type, uint16_t seq,
                                  uint32_t timestamp, uint32_t ssrc,
                                  size_t *written);

/*
 * Parse a datagram into *pkt.  `pkt->payload` aliases `buf`, valid as long
 * as `buf` is.  Rejects: < 12 bytes (NTC_RTP_ERR_SHORT), version != 2
 * (NTC_RTP_ERR_VERSION), CC header overrun (NTC_RTP_ERR_CSRC).
 * A P bit set with a padding count that exceeds the remaining payload is
 * reported as NTC_ERR_PARSE (we do not silently mask it).
 */
ntc_status_t ntc_rtp_parse(const uint8_t *buf, size_t len,
                           ntc_rtp_packet_t *pkt);

/*
 * Header + payload into one buffer.  `seq` is emitted verbatim (wraps are
 * the caller's business).  Returns NTC_ERR_NO_ROOM when cap is too small;
 * *written is set to the required size in that case.
 */
ntc_status_t ntc_rtp_build(uint8_t *buf, size_t cap, int marker,
                           uint8_t payload_type, uint16_t seq,
                           uint32_t timestamp, uint32_t ssrc,
                           const uint8_t *payload, size_t payload_len,
                           size_t *written);

/* ------------------------------------------------------------------ */
/* Sender-side packetiser                                              */
/* ------------------------------------------------------------------ */
/*
 * A sender chops one access unit (a "frame" = list of NAL units, each of
 * which is chopped into FU-A fragments) into RTP packets.  Consecutive
 * packets of the same frame share `timestamp`; the last packet of the
 * frame has marker = 1.  Sequence numbers increase by one per packet.
 */
typedef struct ntc_rtp_sender {
    uint32_t ssrc;
    uint8_t payload_type;
    uint16_t next_seq;
    uint32_t timestamp;
    size_t max_payload; /* per-packet payload budget, >= 1 */
} ntc_rtp_sender_t;

ntc_status_t ntc_rtp_sender_init(ntc_rtp_sender_t *s, uint32_t ssrc,
                                 uint8_t payload_type, size_t max_payload);

/*
 * Packetise one frame.
 *   nal_units     - array of `count` byte ranges forming the frame.
 *   out_dgrams    - array of at least `dgram_cap` pointers; on success each
 *                   element points to malloc'ed memory owned by the CALLER
 *                   (free each with free()).  Passed NULL only when
 *                   dgram_cap == 0.
 *   out_lens      - parallel array of lengths.
 *   out_count     - number of datagrams produced.
 *   out_ts        - timestamp used for this frame.
 *
 * The frame is split into fragments of at most (max_payload - 2) bytes,
 * each carrying a 2-byte FU-A header (indicator + FU header).  A NAL unit
 * that fits in max_payload - 1 bytes is sent as a single-packet NAL (no FU
 * header) with its own 1-byte NAL header -- this mirrors the real "small
 * NAL goes unfragmented" rule and is exercised by tests.
 *
 * Returns NTC_ERR_FULL when more than dgram_cap packets are needed (some
 * *dgrams entries may already be allocated; the caller must free them),
 * NTC_ERR_INVAL on bad args.
 */
ntc_status_t ntc_rtp_send_frame(ntc_rtp_sender_t *s,
                                const uint8_t *const *nal_units,
                                const size_t *nal_lens, size_t count,
                                uint8_t **out_dgrams, size_t *out_lens,
                                size_t dgram_cap, size_t *out_count,
                                uint32_t *out_ts);

/* ------------------------------------------------------------------ */
/* Receiver-side reorder buffer                                        */
/* ------------------------------------------------------------------ */
typedef enum ntc_rtp_slot_state {
    NTC_RTP_SLOT_EMPTY = 0,
    NTC_RTP_SLOT_HELD,    /* buffered, waiting for an earlier packet */
    NTC_RTP_SLOT_DELIVERED
} ntc_rtp_slot_state_t;

/*
 * One entry of the reorder ring.  `in_use` says the slot belongs to a packet
 * of the current window; `delivered` says that packet has already been moved
 * to the delivery queue (pop() will hand it out in ascending seq order).
 * slot.state mirrors those two flags for readability in reports.
 */
typedef struct ntc_rtp_slot {
    ntc_rtp_slot_state_t state;
    uint8_t in_use;
    uint8_t delivered;
    uint16_t seq;
    uint32_t timestamp;
    uint8_t marker;
    size_t payload_len;
    uint8_t payload[NTC_RTP_MAX_DATAGRAM];
} ntc_rtp_slot_t;

typedef struct ntc_rtp_rx {
    size_t reorder_window;  /* max packets held out of order, 1..512 */
    uint16_t expected_seq;  /* oldest sequence number not yet handed out   */
    uint16_t next_seq;      /* next sequence number we still expect to get */
    uint16_t max_seen;      /* highest sequence number ever observed       */
    int expected_valid;     /* 0 until the first packet is seen       */
    uint32_t ssrc;
    int ssrc_valid;

    uint64_t received;      /* ntc_rtp_rx_push() calls that parsed OK    */
    uint64_t delivered;     /* packets handed out by ntc_rtp_rx_pop()    */
    uint64_t duplicates;    /* seq already seen -> dropped               */
    uint64_t lost;          /* declared missing during a flush           */
    uint64_t late;          /* arrived after its slot was passed         */
    uint64_t reordered_in;  /* arrivals whose seq > expected_seq         */
    uint64_t out_of_order;  /* same event, counted per packet buffered   */
    uint64_t ssrc_changes;
    uint64_t flushed_by_pressure;
    uint64_t dropped_stale; /* discarded because the window slid past them*/
    uint64_t rejected;      /* parse/ssrc failures                       */

    /* ring of packets, indexed by (seq % NTC_RTP_MAX_REORDER) */
    ntc_rtp_slot_t ring[NTC_RTP_MAX_REORDER];
    size_t held;            /* SLOT_HELD entries: waiting for earlier seqs*/
    size_t queued;          /* delivered-but-not-yet-popped entries      */
    size_t in_flight;       /* held + queued: what the ring really holds */
    /* shadow "already seen" map to detect duplicates and late arrivals */
    uint8_t seen[NTC_RTP_MAX_REORDER];
    /*
     * "already declared lost" map.  A missing sequence number must be
     * counted exactly once no matter how many flushes walk past it: without
     * this, a stream with 5% loss reported 510 losses for 393 arrivals, i.e.
     * more losses than packets.  Cleared only on a stream restart.
     */
    uint8_t declared[NTC_RTP_MAX_REORDER];
    /*
     * "already accounted for" map: set when a sequence number is either
     * received or declared lost.  It is what makes the accounting identity
     *   delivered + declared_lost + late == packets sent
     * hold exactly, instead of a sequence number being counted in the loss
     * walk and again as a late arrival later on.
     */
    uint8_t accounted[NTC_RTP_MAX_REORDER];
} ntc_rtp_rx_t;

/*
 * Reset the receiver.  reorder_window is clamped to [1, NTC_RTP_MAX_REORDER];
 * 0 or an out-of-range value is rejected with NTC_ERR_RANGE.
 */
ntc_status_t ntc_rtp_rx_init(ntc_rtp_rx_t *rx, size_t reorder_window);

/* One delivered packet, as handed back by ntc_rtp_rx_pop(). */
typedef struct ntc_rtp_delivery {
    uint16_t seq;
    uint32_t timestamp;
    uint8_t marker;
    size_t payload_len;
    uint8_t payload[NTC_RTP_MAX_DATAGRAM];
} ntc_rtp_delivery_t;

/*
 * Push a received datagram.
 *   *arrived_out_of_order - optional; set to 1 when the packet's seq was
 *                           greater than the expected one.
 *   *duplicate_out        - optional; set to 1 when this seq was seen before.
 *
 * A duplicate is counted and dropped (NTC_OK, not an error: the network
 * duplicated it, the app just ignores it).  A packet older than
 * expected_seq is counted as `late` if we never saw it, else `duplicate`.
 *
 * The push may release packets: call ntc_rtp_rx_pop() repeatedly until it
 * returns NTC_ERR_EMPTY to drain them.
 */
ntc_status_t ntc_rtp_rx_push(ntc_rtp_rx_t *rx, const uint8_t *buf, size_t len,
                             int *arrived_out_of_order, int *duplicate_out);

/* Pop one delivered packet in sequence order.  NTC_ERR_EMPTY when none. */
ntc_status_t ntc_rtp_rx_pop(ntc_rtp_rx_t *rx, ntc_rtp_delivery_t *out);

/*
 * Force out everything currently held: packets that are still missing below
 * the highest held seq are counted as `lost` and skipped (that is the
 * "gap declared as loss" path), then all held packets are delivered in
 * ascending seq order.
 *   *lost_out  - number of sequence numbers declared lost by this flush.
 *   *moved_out - number of packets moved to the delivery queue.
 * Used at end-of-stream and whenever the sender's marker says the frame
 * boundary has passed.
 */
ntc_status_t ntc_rtp_rx_flush(ntc_rtp_rx_t *rx, size_t *lost_out,
                              size_t *moved_out);

/* Snapshot of the counters above, for reporting. */
typedef struct ntc_rtp_stats {
    uint64_t received;
    uint64_t delivered;
    uint64_t duplicates;
    uint64_t lost;
    uint64_t late;
    uint64_t reordered_in;
    uint64_t out_of_order;
    uint64_t ssrc_changes;
    uint64_t flushed_by_pressure;
    uint64_t rejected;
    size_t held;
} ntc_rtp_stats_t;

ntc_status_t ntc_rtp_rx_stats(const ntc_rtp_rx_t *rx, ntc_rtp_stats_t *out);

/* ------------------------------------------------------------------ */
/* Module 3: NAL fragmentation / reassembly            (src/nal.c)     */
/* ------------------------------------------------------------------ */
/*
 * FU-A STYLE FRAGMENTATION (RFC 6184 section 5.8 shape, simplified)
 *
 *   Fragment i>0 payload = [FU indicator][FU header][fragment bytes...]
 *
 *   FU indicator:   +---------------+
 *                   |F|NRI|  Type   |   Type = 28 (FU-A)
 *                   +---------------+
 *   FU header:      +---------------+
 *                   |S|E|R|  Type   |   S=start, E=end, R=reserved(0),
 *                   +---------------+   Type = original NAL unit type
 *
 *   The original NAL unit is the concatenation of the fragment payloads
 *   that follow each FU header; the original NAL header byte is
 *   (indicator & 0xE0) | (fu_header & 0x1F).
 *
 *   A NAL unit small enough to fit in one packet is sent *unfragmented*:
 *   its payload is just the NAL unit itself (first byte = real NAL header).
 *
 * REASSEMBLY RULES (this is where the bugs live)
 *   * Fragments must arrive with S first, then increasing fragment order,
 *     then E last.  A missing middle fragment invalidates the WHOLE unit:
 *     we drop every buffered fragment of that unit and count one
 *     `frame_dropped` (never emit half a NAL unit).
 *   * A duplicate fragment inside the same unit is ignored and counted.
 *   * A new S while a unit is in progress means the previous unit was
 *     truncated -> drop + count.
 *   * Reassembly capacity is fixed; overflow drops the unit with
 *     NTC_NAL_ERR_NO_ROOM and counts it.
 */

#define NTC_NAL_TYPE_FU_A 28
#define NTC_NAL_MAX_UNIT 8192
#define NTC_NAL_MAX_FRAG 256

typedef struct ntc_nal_fragment_info {
    int start;      /* S bit */
    int end;        /* E bit */
    int type;       /* original NAL type, 5 bits */
    int nri;        /* NRI from the FU indicator */
    size_t data_len;
    const uint8_t *data;
} ntc_nal_fragment_info_t;

/*
 * Look at a payload and classify it:
 *   *unit_out - complete NAL unit bytes (payload as-is) for a single-packet
 *               NAL, else NULL.
 *   *frag_out - FU-A fragment description for a fragmented NAL, else NULL.
 * Returns NTC_OK / NTC_NAL_ERR_BAD_FU (indicator bytes malformed) /
 * NTC_ERR_INVAL.
 */
ntc_status_t ntc_nal_classify_payload(const uint8_t *payload, size_t len,
                                      const uint8_t **unit_out,
                                      size_t *unit_len_out,
                                      ntc_nal_fragment_info_t *frag_out);

/*
 * Build FU-A fragments for one NAL unit into the caller's buffer.
 *   `src`/`src_len` is the whole NAL unit (including its 1-byte header).
 *   `frag_payload` is the max payload bytes per RTP packet; each fragment
 *   consumes 2 bytes of that budget for the FU headers.
 *   `out`/`out_cap` receives a sequence of length-prefixed fragments:
 *   out[0..1] = big-endian total fragment bytes, then for each fragment a
 *   2-byte big-endian length followed by the fragment bytes.
 *   *frag_count_out - number of fragments produced.
 * Returns NTC_ERR_NO_ROOM when out_cap is too small, NTC_ERR_INVAL on args,
 * NTC_NAL_ERR_BAD_TYPE when the NAL header type is forbidden (0 or 31).
 * A NAL unit that fits in frag_payload - 1 bytes still produces ONE
 * fragment-free packet: the data is copied verbatim.
 */
ntc_status_t ntc_nal_fragment(const uint8_t *src, size_t src_len,
                              size_t frag_payload, uint8_t *out,
                              size_t out_cap, size_t *out_len,
                              size_t *frag_count_out);

/*
 * Reassembler state.  One instance handles one stream (one SSRC).
 *
 *   unit[]      - accumulates the NAL unit being reassembled.
 *   unit_len    - bytes accumulated so far.
 *   in_progress - 1 between S and E.
 *   expected_frag_index - fragments seen so far for this unit (used for
 *                         duplicate/out-of-order detection).
 *   last_unit   - the most recently completed unit (valid until the next
 *                 start), so the caller can inspect it after NTC_OK.
 */
typedef struct ntc_nal_stats {
    uint64_t units_complete;
    uint64_t units_dropped;      /* any reason */
    uint64_t dropped_missing_frag;
    uint64_t dropped_overflow;
    uint64_t dropped_truncated;  /* new S / end-of-stream mid-unit */
    uint64_t fragments_in;
    uint64_t duplicate_frags;
    uint64_t out_of_order_frags;
    uint64_t single_packet_units;
    uint64_t bytes_reassembled;
} ntc_nal_stats_t;

typedef struct ntc_nal_reasm {
    uint8_t unit[NTC_NAL_MAX_UNIT];
    size_t unit_len;
    int in_progress;
    int last_frag_started;
    size_t frags_seen;
    size_t expected_frag_index;
    uint8_t unit_type;
    uint8_t unit_nri;
    uint32_t timestamp;
    int timestamp_valid;

    /*
     * Position of the fragment inside its NAL unit, counted from 0.
     *
     * FU-A's own header carries only S and E, so a missing MIDDLE fragment
     * is invisible in the fragment payload itself -- that is why real stacks
     * detect the loss from the RTP sequence numbers, which are consecutive
     * across the fragments of one unit.  ntc_nal_reasm_push() takes the
     * position explicitly for exactly that reason: pass (seq - seq_of_first
     * fragment).  With it the reassembler can prove a gap and drop the unit
     * instead of emitting a NAL unit assembled from non-adjacent bytes; pass
     * -1 when the position is unknown and the check is skipped.
     */
    long frag_index;
    long last_accepted_index;
    uint64_t gaps_detected;
    uint64_t out_of_order_frags;

    uint8_t last_unit[NTC_NAL_MAX_UNIT];
    size_t last_unit_len;
    uint8_t last_unit_type;

    ntc_nal_stats_t stats;
} ntc_nal_reasm_t;

ntc_status_t ntc_nal_reasm_init(ntc_nal_reasm_t *r);

/*
 * Feed one RTP payload (as delivered by the reorder buffer).
 *   payload   - RTP payload: either a whole NAL unit or an FU-A fragment.
 *   timestamp - the RTP timestamp of the packet, used to detect a fragment
 *               belonging to a different frame (must match the unit's).
 *   frag_index- the fragment's position inside its NAL unit, or -1 when it is
 *               not known.  RTP sequence numbers are consecutive across the
 *               fragments of one unit, so a normal caller passes
 *               (seq - first_seq).  With the position known a missing middle
 *               fragment is detected and the whole unit is dropped, which is
 *               the only way to avoid emitting a NAL unit stitched together
 *               from non-adjacent bytes.
 *
 * Returns:
 *   NTC_OK            a complete unit is available in r->last_unit
 *                     (*completed_out = 1) or a fragment was buffered
 *                     (*completed_out = 0).
 *   NTC_NAL_ERR_GAP   a fragment was missing; the whole unit was discarded
 *                     (never a half frame), and the caller should count a
 *                     lost frame.
 *   NTC_NAL_ERR_NO_ROOM   unit exceeded NTC_NAL_MAX_UNIT.
 *   NTC_NAL_ERR_BAD_FU    malformed FU headers.
 *   NTC_ERR_INVAL     NULL args / len == 0.
 */
ntc_status_t ntc_nal_reasm_push_ex(ntc_nal_reasm_t *r, const uint8_t *payload,
                                   size_t len, uint32_t timestamp,
                                   long frag_index, int *completed_out);

/* Convenience wrapper: ntc_nal_reasm_push_ex(r, p, n, ts, -1, out). */
ntc_status_t ntc_nal_reasm_push(ntc_nal_reasm_t *r, const uint8_t *payload,
                                size_t len, uint32_t timestamp,
                                int *completed_out);

/* Drop any unit in progress, counting a truncated drop.  Used at EOS. */
ntc_status_t ntc_nal_reasm_flush(ntc_nal_reasm_t *r);

/* Copy the stats out (returns NTC_OK / NTC_ERR_INVAL). */
ntc_status_t ntc_nal_reasm_stats(const ntc_nal_reasm_t *r,
                                 ntc_nal_stats_t *out);

/* ------------------------------------------------------------------ */
/* Module 4: motion detection + alarm state machine    (src/motion.c)  */
/* ------------------------------------------------------------------ */
/*
 * PIPELINE
 *
 *   frame(t) --+--> per-block abs diff vs background
 *              |    metric = (changed_blocks / total_blocks)  in [0,1]
 *              +--> threshold: metric >= motion_threshold ?
 *                   yes -> "frame mover", no -> "frame still"
 *
 *   DEBOUNCE  : an event starts only after `start_frames` consecutive
 *               mover frames; it ends only after `end_frames` consecutive
 *               still frames.  This kills single-frame sensor noise.
 *   HYSTERESIS: the end condition may use a lower threshold
 *               (end_threshold <= motion_threshold) so a slow-moving
 *               object does not chatter.
 *
 * ALARM STATE MACHINE
 *
 *   IDLE --start_frames movers--> PENDING --*--> ACTIVE --> (end_frames
 *        stills) --> COOLDOWN --> IDLE (or straight back to ACTIVE if
 *        motion resumes inside the cooldown)
 *
 *   COOLDOWN implements the minimum quiet period between two events
 *   (alarm suppression / anti-storm).  `min_event_frames` additionally
 *   suppresses events that would be shorter than a minimum duration.
 *
 * BACKGROUND MODEL: running average, updated only on still frames:
 *     bg = bg + (cur - bg) / bg_alpha      (bg_alpha >= 1, integer steps)
 *   Frame 0 initialises the background.
 *
 * LINKAGE flags: when an event starts, the detector records which
 * downstream actions it asked for (record / stream / snapshot) based on
 * ntc_motion_config.link_flags, so the sim can count "alarms that
 * triggered a recording".
 */

typedef enum ntc_motion_state {
    NTC_MOTION_IDLE = 0,
    NTC_MOTION_PENDING,
    NTC_MOTION_ACTIVE,
    NTC_MOTION_COOLDOWN
} ntc_motion_state_t;

typedef struct ntc_motion_config {
    size_t width;            /* frame width in pixels, >= 1 */
    size_t height;           /* frame height in pixels, >= 1 */
    size_t block_size;       /* block edge in pixels, >= 1 */
    int pixel_threshold;     /* per-pixel abs diff to count as changed, 1..255 */
    double motion_threshold; /* changed-block ratio to call a frame a mover, (0,1] */
    double end_threshold;    /* ratio used while ACTIVE, <= motion_threshold */
    int start_frames;        /* consecutive movers to raise, >= 1 */
    int end_frames;          /* consecutive stills to clear, >= 1 */
    int cooldown_frames;     /* quiet frames after clear, >= 0 */
    int min_event_frames;    /* events shorter than this are suppressed, >= 0 */
    int bg_alpha;            /* background adaptation divisor, 1 = freeze bg */
    uint32_t link_flags;     /* NTC_MOTION_LINK_* */
} ntc_motion_config_t;

#define NTC_MOTION_LINK_NONE 0u
#define NTC_MOTION_LINK_RECORD 1u
#define NTC_MOTION_LINK_STREAM 2u
#define NTC_MOTION_LINK_SNAPSHOT 4u

typedef struct ntc_motion_stats {
    uint64_t frames;
    uint64_t mover_frames;
    uint64_t still_frames;
    uint64_t events_raw;      /* state transitions into PENDING that got
                               * a mover frame, i.e. without debounce */
    uint64_t events_started;  /* alarms actually raised (after debounce) */
    uint64_t events_ended;
    uint64_t events_suppressed_short;
    uint64_t events_suppressed_cooldown;
    uint64_t record_triggers;
    uint64_t stream_triggers;
    uint64_t snapshot_triggers;
    uint64_t frames_while_active;
    double motion_sum;        /* sum of per-frame motion ratios */
} ntc_motion_stats_t;

typedef struct ntc_motion_detector {
    ntc_motion_config_t cfg;
    ntc_motion_state_t state;

    uint8_t *background;   /* width*height grey bytes, owned by this struct */
    size_t bg_len;
    int background_ready;

    double last_motion;    /* ratio for the most recent frame */
    size_t last_changed;   /* changed blocks in the most recent frame */
    size_t total_blocks;

    int consecutive_movers;
    int consecutive_stills;
    int event_frames;      /* frames since the event started */
    int cooldown_left;

    double event_start_ms;
    double event_end_ms;

    ntc_motion_stats_t stats;
} ntc_motion_detector_t;

/*
 * Fill a config with documented defaults for the given frame geometry:
 *   block_size 16, pixel_threshold 20, motion_threshold 0.02,
 *   end_threshold 0.01, start_frames 3, end_frames 5, cooldown_frames 25,
 *   min_event_frames 5, bg_alpha 8, link_flags RECORD|STREAM.
 * Returns NTC_ERR_INVAL when cfg is NULL.
 */
ntc_status_t ntc_motion_config_default(ntc_motion_config_t *cfg,
                                       size_t width, size_t height);

/* Validate a config: NTC_MOT_ERR_CONFIG on nonsense. */
ntc_status_t ntc_motion_config_check(const ntc_motion_config_t *cfg);

/*
 * Allocate the detector (background buffer) and validate the config.
 * Returns NTC_ERR_INVAL / NTC_MOT_ERR_CONFIG / NTC_ERR_NOMEM.
 */
ntc_status_t ntc_motion_init(ntc_motion_detector_t *d,
                             const ntc_motion_config_t *cfg);

/* Free internal buffers (idempotent; the struct may be re-inited). */
void ntc_motion_destroy(ntc_motion_detector_t *d);

/*
 * Feed one grey (8-bit) frame of cfg.width * cfg.height bytes.
 *   *motion_out        - changed-block ratio for this frame (optional)
 *   *event_started_out - 1 when this frame raised an alarm
 *   *event_ended_out   - 1 when this frame cleared an alarm
 * Returns NTC_OK, NTC_ERR_INVAL, or NTC_MOT_ERR_FRAME_SIZE.
 */
ntc_status_t ntc_motion_process(ntc_motion_detector_t *d, const uint8_t *frame,
                                double now_ms, double *motion_out,
                                int *event_started_out, int *event_ended_out);

/* The ratio the detector would compute, without touching the state machine
 * or the background model.  Used by tests and by the sim to show the raw
 * signal next to the debounced decision. */
ntc_status_t ntc_motion_measure(const ntc_motion_detector_t *d,
                                const uint8_t *frame, double *ratio_out,
                                size_t *changed_out);

/* Copy stats out.  NTC_ERR_INVAL on NULL. */
ntc_status_t ntc_motion_stats(const ntc_motion_detector_t *d,
                              ntc_motion_stats_t *out);

/* Current state / name, for reporting. */
ntc_motion_state_t ntc_motion_state(const ntc_motion_detector_t *d);
const char *ntc_motion_state_str(ntc_motion_state_t s);

/* ------------------------------------------------------------------ */
/* Module 5: recording index + ring-buffer eviction   (src/record.c)   */
/* ------------------------------------------------------------------ */
/*
 * A recording index holds N segments, each a [start_ms, end_ms) interval
 * with a size in bytes and a reason bitmap.  The store has a byte budget:
 * inserting a segment that would exceed it evicts the OLDEST segments
 * (by start_ms) until it fits.  A single segment larger than the whole
 * budget is rejected with NTC_REC_ERR_TIME_ORDER-free NTC_ERR_RANGE.
 *
 * INVARIANTS
 *   * Segments are kept sorted by start_ms; overlapping inserts whose
 *     overlap is a *contiguous* continuation (new.start_ms <=
 *     last.end_ms + merge_gap_ms) are MERGED into the existing segment,
 *     which is how a motion event that keeps re-triggering becomes one
 *     clip.  Overlap deeper in the table is rejected with
 *     NTC_REC_ERR_OVERLAP.
 *   * Eviction always removes index 0 (the oldest) and never removes the
 *     segment just inserted.
 *
 * RETRIEVAL: ntc_rec_query(start, end) returns every segment that
 * *intersects* [start, end), i.e. seg.end_ms > start && seg.start_ms < end.
 * A zero-length query window (start == end) matches nothing.
 */

#define NTC_REC_MAX_SEGMENTS 64
#define NTC_REC_MAX_QUERY 64

typedef struct ntc_rec_segment {
    uint64_t id;
    double start_ms;
    double end_ms;
    size_t bytes;
    uint32_t reason;   /* NTC_MOTION_LINK_* bitmap */
    int in_use;
} ntc_rec_segment_t;

typedef struct ntc_rec_stats {
    uint64_t inserts;
    uint64_t merges;
    uint64_t evictions;
    uint64_t evicted_bytes;
    uint64_t queries;
    uint64_t query_hits;
    uint64_t rejected;
} ntc_rec_stats_t;

typedef struct ntc_recorder {
    ntc_rec_segment_t segs[NTC_REC_MAX_SEGMENTS];
    size_t count;
    uint64_t next_id;
    size_t capacity_bytes;
    size_t used_bytes;
    double merge_gap_ms;
    uint64_t oldest_id;   /* for tests: id of segs[0], 0 when empty */
    ntc_rec_stats_t stats;
} ntc_recorder_t;

/*
 * Init.  capacity_bytes == 0 means unlimited (segments are then only
 * limited by NTC_REC_MAX_SEGMENTS).  merge_gap_ms is clamped to >= 0.
 */
ntc_status_t ntc_recorder_init(ntc_recorder_t *r, size_t capacity_bytes,
                               double merge_gap_ms);

/*
 * Insert a segment.
 *   *id_out - id assigned (or the id of the segment it merged into).
 * Returns:
 *   NTC_OK              inserted or merged
 *   NTC_REC_ERR_TIME_ORDER  end_ms < start_ms
 *   NTC_REC_ERR_FULL    table full and nothing evictable
 *   NTC_ERR_RANGE       bytes > capacity_bytes
 *   NTC_ERR_INVAL       NULL args
 * Eviction may remove several segments to make room; the just-inserted
 * segment is never evicted.
 */
ntc_status_t ntc_recorder_insert(ntc_recorder_t *r, double start_ms,
                                 double end_ms, size_t bytes, uint32_t reason,
                                 uint64_t *id_out);

/*
 * Query [start_ms, end_ms).
 *   out     - array of at least `out_cap` segments, filled in ascending
 *             start_ms order.
 *   *hits   - number of segments written (never more than out_cap).
 *   *hits_total - number of segments matching, even if > out_cap.
 * Returns NTC_OK (even with 0 hits), NTC_ERR_INVAL on NULL,
 * NTC_REC_ERR_TIME_ORDER when end < start.
 */
ntc_status_t ntc_recorder_query(const ntc_recorder_t *r, double start_ms,
                                double end_ms, ntc_rec_segment_t *out,
                                size_t out_cap, size_t *hits,
                                size_t *hits_total);

/* Total recorded milliseconds (sum of segment durations). */
ntc_status_t ntc_recorder_total_ms(const ntc_recorder_t *r, double *out);

/* Look up one segment by id. */
ntc_status_t ntc_recorder_find(const ntc_recorder_t *r, uint64_t id,
                               ntc_rec_segment_t *out);

/* Remove the oldest segment explicitly (ring-buffer step).  NTC_ERR_EMPTY
 * when there is nothing to remove. */
ntc_status_t ntc_recorder_evict_oldest(ntc_recorder_t *r);

ntc_status_t ntc_recorder_stats(const ntc_recorder_t *r,
                                ntc_rec_stats_t *out);

/* ------------------------------------------------------------------ */
/* Module 6: end-to-end helpers + reporting            (src/sim.c)     */
/* ------------------------------------------------------------------ */
/*
 * Deterministic PRNG (xorshift64*) so every run of `make sim` prints the
 * same numbers, and the README numbers are reproducible.
 */
typedef struct ntc_rng {
    uint64_t state;
} ntc_rng_t;

void ntc_rng_seed(ntc_rng_t *rng, uint64_t seed);
/* Uniform in [0,1). */
double ntc_rng_double(ntc_rng_t *rng);
/* Uniform in [0,n). */
uint32_t ntc_rng_below(ntc_rng_t *rng, uint32_t n);

/* Network impairment profile for the simulated channel. */
typedef struct ntc_net_profile {
    double loss_prob;      /* per-packet drop probability, [0,1] */
    double dup_prob;       /* per-packet duplication probability, [0,1] */
    double reorder_prob;   /* probability a packet is held and swapped with
                            * the next one, [0,1] */
    uint32_t burst_gap;    /* reorder distance in packets, >= 1 */
    uint64_t seed;
} ntc_net_profile_t;

/* Defaults: 5% loss, 2% duplicate, 20% reorder by gap 1. */
ntc_status_t ntc_net_profile_default(ntc_net_profile_t *p, uint64_t seed);

/*
 * Apply the profile to an ordered array of datagrams in place (an array of
 * `count` malloc'ed buffers + lengths, capacity `cap`).  Duplication grows
 * the array (each duplicate is a fresh malloc), reordering permutes it, and
 * loss frees the dropped buffer and shifts the array down.
 *
 *   *count_out - resulting number of datagrams (<= *count_in + cap slack)
 *   out_lost / out_duplicated - counters
 * Returns NTC_ERR_FULL if duplication would exceed `cap`.
 * The caller owns every buffer still present in the array afterwards.
 */
ntc_status_t ntc_net_apply(ntc_net_profile_t *p, uint8_t **dgrams,
                           size_t *lens, size_t cap, size_t *count_inout,
                           size_t *out_lost, size_t *out_duplicated);

/* Free an array of datagrams (safe on NULL entries). */
void ntc_net_free(uint8_t **dgrams, size_t count);

/* ------------------------------------------------------------------ */
/* Reporting: create results/ if needed and write text + CSV           */
/* ------------------------------------------------------------------ */
/*
 * Create `dir` (one level, using mkdir from <sys/stat.h>; no shelling
 * out, so it works the same on Windows/macOS/Linux).  Returns NTC_OK if
 * the directory exists or was created, NTC_ERR_INVAL on NULL.
 */
ntc_status_t ntc_mkdir(const char *dir);

/*
 * Open `dir/name` for TEXT writing (mode "wb": we always write our own
 * "\n" and never rely on the C runtime's text translation).
 * Returns NULL on failure.  The caller fcloses.
 */
FILE *ntc_report_open_text(const char *dir, const char *name);

/* Same, but appends a header row when overwriting a CSV. */
FILE *ntc_report_open_csv(const char *dir, const char *name,
                          const char *header);

#ifdef __cplusplus
}
#endif

#endif /* NETCAM_H */
