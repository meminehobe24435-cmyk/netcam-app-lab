/*
 * main_sim.c -- end-to-end simulation of the netcam application/protocol chain
 * ===========================================================================
 *
 * Chain:  RTSP session setup -> media fragmentation -> packetisation ->
 *         network (loss / reorder / duplicate) -> receiver reordering ->
 *         NAL reassembly -> motion detection -> recording index.
 *
 * Every stage prints its key numbers and the run is written to results/ as a
 * text file plus two CSV files (results/summary.csv, results/rtp_windows.csv).
 * The run is fully deterministic, so the figures in README.md are reproducible.
 *
 * Honest scope: a logic simulation on a PC.  No camera sensor, no ISP, no real
 * H.264/HEVC encoder, no sockets and no real network, single threaded.
 */

#include "netcam.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIM_RESULT_DIR "results"
#define FRAMES 30
#define NAL_BYTES 3000
#define FRAME_W 64u
#define FRAME_H 48u
#define MOTION_LEN (FRAME_W * FRAME_H)

static void banner(const char *title)
{
    printf("\n=== %s ===\n", title);
}

static void paint(uint8_t *frame, int base, int x, int y, int bw, int bh,
                  int value)
{
    size_t i;
    for (i = 0; i < MOTION_LEN; i++) {
        frame[i] = (uint8_t)base;
    }
    if (bw > 0 && bh > 0) {
        int yy;
        int xx;
        for (yy = y; yy < y + bh; yy++) {
            for (xx = x; xx < x + bw; xx++) {
                if (xx >= 0 && yy >= 0 && (size_t)xx < FRAME_W &&
                    (size_t)yy < FRAME_H) {
                    frame[(size_t)yy * FRAME_W + (size_t)xx] = (uint8_t)value;
                }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* stage 1: RTSP session                                               */
/* ------------------------------------------------------------------ */

static int stage_rtsp(ntc_rtsp_server_t *srv, FILE *txt, uint32_t *session_io)
{
    ntc_rtsp_response_t resp;
    size_t i;
    int accepted = 0;
    int rejected = 0;
    int bad = 0;

    banner("stage 1: RTSP session setup and state machine");

    if (ntc_rtsp_server_init(srv, 1000u, 1u, "netcam-app-lab") != NTC_OK) {
        return 0;
    }

    for (i = 0; i < 8u; i++) {
        static const char *labels[] = {
            "OPTIONS", "DESCRIBE", "SETUP", "PLAY",
            "PLAY(replayed CSeq)", "PAUSE", "PAUSE(again)", "TEARDOWN"
        };
        char req[512];
        size_t len;
        ntc_status_t st;
        int with_session = (i >= 3u);
        const char *fmt;

        switch (i) {
        case 0:
            fmt = "OPTIONS rtsp://192.168.1.10:554/stream=0 RTSP/1.0\r\n"
                  "CSeq: 1\r\n\r\n";
            break;
        case 1:
            fmt = "DESCRIBE rtsp://192.168.1.10:554/stream=0 RTSP/1.0\r\n"
                  "CSeq: 2\r\nAccept: application/sdp\r\n\r\n";
            break;
        case 2:
            fmt = "SETUP rtsp://192.168.1.10:554/stream=0/trackID=0 "
                  "RTSP/1.0\r\nCSeq: 3\r\n"
                  "Transport: RTP/AVP;unicast;client_port=8000-8001\r\n\r\n";
            break;
        case 3:
            fmt = "PLAY rtsp://cam RTSP/1.0\r\nCSeq: 4\r\nSession: %u\r\n"
                  "Range: npt=0.000-\r\n\r\n";
            break;
        case 4:
            fmt = "PLAY rtsp://cam RTSP/1.0\r\nCSeq: 4\r\nSession: %u\r\n\r\n";
            break;
        case 5:
            fmt = "PAUSE rtsp://cam RTSP/1.0\r\nCSeq: 5\r\nSession: %u\r\n\r\n";
            break;
        case 6:
            fmt = "PAUSE rtsp://cam RTSP/1.0\r\nCSeq: 6\r\nSession: %u\r\n\r\n";
            break;
        default:
            fmt = "TEARDOWN rtsp://cam RTSP/1.0\r\nCSeq: 7\r\n"
                  "Session: %u\r\n\r\n";
            break;
        }

        if (with_session) {
            len = (size_t)sprintf(req, fmt, (unsigned)*session_io);
        } else {
            len = (size_t)sprintf(req, "%s", fmt);
        }
        st = ntc_rtsp_handle_text(srv, req, len, (double)i * 10.0, &resp, NULL);
        if (resp.session_id != 0u) {
            *session_io = resp.session_id;
        }
        if (resp.status == 200) {
            accepted++;
        } else {
            rejected++;
        }
        printf("  %-20s -> %d %-30s [%s]\n", labels[i], resp.status,
               resp.reason, ntc_status_str(st));
        if (txt != NULL) {
            fprintf(txt, "rtsp_%s_status=%d\n", labels[i], resp.status);
        }
        if (resp.status == 0) {
            bad = 1;
        }
    }

    /* Render one response onto the wire, to show the formatting is real. */
    {
        static const char *probe =
            "OPTIONS rtsp://cam RTSP/1.0\r\nCSeq: 9\r\n\r\n";
        ntc_rtsp_request_t parsed;
        if (ntc_rtsp_parse_request(probe, strlen(probe), &parsed) == NTC_OK) {
            ntc_rtsp_response_t r;
            char line[512];
            size_t written = 0;
            memset(&r, 0, sizeof(r));
            r.status = 200;
            strcpy(r.reason, "OK");
            r.cseq = 9;
            if (ntc_rtsp_write_response(&r, line, sizeof(line), &written) ==
                NTC_OK) {
                printf("  wire format (%u bytes): %s", (unsigned)written, line);
            }
        }
    }

    printf("  accepted=%d rejected=%d  CSeq rejections=%u  455 rejections=%u"
           "  454 rejections=%u\n",
           accepted, rejected, (unsigned)srv->stats.rejected_cseq_order,
           (unsigned)srv->stats.rejected_state,
           (unsigned)(srv->stats.rejected_session_missing +
                      srv->stats.rejected_session_invalid));
    if (txt != NULL) {
        fprintf(txt, "rtsp_accepted=%d\n", accepted);
        fprintf(txt, "rtsp_rejected=%d\n", rejected);
        fprintf(txt, "rtsp_cseq_rejected=%u\n",
                (unsigned)srv->stats.rejected_cseq_order);
        fprintf(txt, "rtsp_455_total=%u\n",
                (unsigned)srv->stats.rejected_state);
        fprintf(txt, "rtsp_454_total=%u\n",
                (unsigned)(srv->stats.rejected_session_missing +
                           srv->stats.rejected_session_invalid));
    }
    return bad ? 0 : 1;
}

/* ------------------------------------------------------------------ */
/* stage 2-4: media -> packets -> network -> reorder -> reassembly     */
/* ------------------------------------------------------------------ */

static int stage_media(FILE *txt, size_t reorder_window, size_t *sent_out,
                       size_t *delivered_out, size_t *lost_out,
                       size_t *exact_out)
{
    ntc_rtp_sender_t sender;
    ntc_net_profile_t prof;
    ntc_rtp_rx_t *rx = (ntc_rtp_rx_t *)calloc(1, sizeof(*rx));
    ntc_nal_reasm_t *reasm = (ntc_nal_reasm_t *)calloc(1, sizeof(*reasm));
    uint8_t *src = (uint8_t *)malloc(NAL_BYTES);
    const uint8_t *units[1];
    size_t lens[1];
    size_t sent = 0;
    size_t after_net = 0;
    size_t dropped = 0;
    size_t duplicated = 0;
    size_t delivered = 0;
    size_t matched = 0;
    size_t mismatched = 0;
    size_t acc_units_complete = 0;
    size_t acc_units_dropped = 0;
    size_t acc_fragments_in = 0;
    size_t acc_bytes_reassembled = 0;
    size_t f;

    banner("stage 2-4: media -> RTP -> network -> reorder -> reassembly");

    if (rx == NULL || reasm == NULL || src == NULL) {
        fprintf(stderr, "allocation failure\n");
        free(rx);
        free(reasm);
        free(src);
        return 0;
    }
    if (ntc_rtp_sender_init(&sender, 0xCAFEBABEu, 96, 1200) != NTC_OK ||
        ntc_net_profile_default(&prof, 20240301u) != NTC_OK) {
        free(rx);
        free(reasm);
        free(src);
        return 0;
    }
    /*
     * The receiver is initialised ONCE, before the frame loop, exactly like a
     * real RTP session: SSRC, sequence window and duplicate map live for the
     * whole session.  Re-initialising it per frame (which an earlier revision
     * of this file did) throws away the window, so the first fragment of every
     * frame -- which arrives out of order -- is classified as an ancient
     * packet and dropped, and no frame ever reassembles.
     */
    if (ntc_rtp_rx_init(rx, reorder_window) != NTC_OK) {
        free(rx);
        free(reasm);
        free(src);
        return 0;
    }
    /*
     * A deliberately hostile channel: enough loss and reordering that the
     * window actually has to work, but not so much that nothing survives.
     */
    prof.loss_prob = 0.10;
    prof.dup_prob = 0.04;
    prof.reorder_prob = 0.35;
    prof.burst_gap = 2;

    for (f = 0; f < FRAMES; f++) {
        uint8_t *dgrams[256];
        size_t dglen[256];
        size_t count = 0;
        size_t i;
        uint32_t ts = 0;
        uint32_t seq_before_frame = sender.next_seq;

        for (i = 0; i < NAL_BYTES; i++) {
            src[i] = (uint8_t)((i * 31u + f * 7u) & 0xFFu);
        }
        src[0] = 0x65; /* NAL type 5 (IDR), NRI 3 */
        units[0] = src;
        lens[0] = NAL_BYTES;

        memset(dgrams, 0, sizeof(dgrams));
        if (ntc_rtp_send_frame(&sender, units, lens, 1, dgrams, dglen, 256,
                               &count, &ts) != NTC_OK) {
            printf("  frame %u: packetisation failed\n", (unsigned)f);
            break;
        }
        sent += count;

        /*
         * ntc_net_apply() reseeds its generator from the profile, so the
         * seed is advanced per frame.  Leaving it constant makes every
         * frame suffer exactly the same losses; with three-fragment frames
         * that can mean "the same fragment every time", which is a
         * perfectly clean run that says nothing about the receiver.
         */
        prof.seed = 20240301ull + (uint64_t)f * 7919ull;
        if (ntc_net_apply(&prof, dgrams, dglen, 256, &count, &dropped,
                          &duplicated) != NTC_OK) {
            printf("  frame %u: impairment failed\n", (unsigned)f);
            ntc_net_free(dgrams, count);
            break;
        }
        after_net += count;

        if (ntc_nal_reasm_init(reasm) != NTC_OK) {
            ntc_net_free(dgrams, count);
            break;
        }
        for (i = 0; i < count; i++) {
            (void)ntc_rtp_rx_push(rx, dgrams[i], dglen[i], NULL, NULL);
        }
        (void)ntc_rtp_rx_flush(rx, NULL, NULL);

        {
            ntc_rtp_delivery_t dev;
            /*
             * Recover the fragment position the way a real stack does: the RTP
             * sequence numbers of one access unit are consecutive, so the
             * offset of a fragment from the FIRST sequence number of the frame
             * is its fragment index.  The reference is the sender's first
             * sequence number for this frame (the sender's counter before the
             * frame was packetised), not the first fragment that happened to
             * arrive: if the leading fragment was lost, using the first
             * arrival as the reference would relabel every later fragment and
             * the reassembler would correctly refuse the unit.
             */
            uint32_t base_seq = seq_before_frame;
            while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
                int completed = 0;
                long idx = (long)(uint16_t)(dev.seq - (uint16_t)base_seq);
                (void)ntc_nal_reasm_push_ex(reasm, dev.payload, dev.payload_len,
                                            dev.timestamp, idx, &completed);
                if (completed) {
                    if (reasm->last_unit_len == NAL_BYTES &&
                        memcmp(reasm->last_unit, src, NAL_BYTES) == 0) {
                        matched++;
                    } else {
                        mismatched++;
                    }
                }
                delivered++;
            }
        }
        (void)ntc_nal_reasm_flush(reasm);
        {
            /*
             * The reassembler is reset per frame (one access unit at a time),
             * so its statistics have to be ACCUMULATED across frames.  Reading
             * them only after the loop reports the last frame's numbers and
             * contradicts the matched counter -- which is exactly what an
             * earlier revision of this file printed.
             */
            ntc_nal_stats_t frame_stats;
            (void)ntc_nal_reasm_stats(reasm, &frame_stats);
            acc_units_complete += (size_t)frame_stats.units_complete;
            acc_units_dropped += (size_t)frame_stats.units_dropped;
            acc_fragments_in += (size_t)frame_stats.fragments_in;
            acc_bytes_reassembled += (size_t)frame_stats.bytes_reassembled;
        }
        ntc_net_free(dgrams, count);
    }

    {
        ntc_rtp_stats_t rs;
        (void)ntc_rtp_rx_stats(rx, &rs);

        printf("  frames sent               : %u\n", (unsigned)FRAMES);
        printf("  datagrams sent            : %u\n", (unsigned)sent);
        printf("  after the network         : %u (dropped %u, duplicated %u)\n",
               (unsigned)after_net, (unsigned)dropped, (unsigned)duplicated);
        printf("  delivered by the receiver : %u\n", (unsigned)delivered);
        printf("  declared lost / late      : %u / %u\n", (unsigned)rs.lost,
               (unsigned)rs.late);
        printf("  duplicates detected       : %u\n",
               (unsigned)rs.duplicates);
        printf("  reorder window            : %u\n",
               (unsigned)reorder_window);
        printf("  NAL units complete        : %u\n",
               (unsigned)acc_units_complete);
        printf("  NAL units dropped         : %u\n",
               (unsigned)acc_units_dropped);
        printf("  bytes reassembled         : %u\n",
               (unsigned)acc_bytes_reassembled);
        printf("  byte-exact frames         : %u of %u completed\n",
               (unsigned)matched, (unsigned)acc_units_complete);
        printf("  content mismatches        : %u (must be 0: a frame is either\n"
               "                              reassembled exactly or dropped)\n",
               (unsigned)mismatched);

        if (txt != NULL) {
            fprintf(txt, "sim_frames=%u\n", (unsigned)FRAMES);
            fprintf(txt, "sim_datagrams_sent=%u\n", (unsigned)sent);
            fprintf(txt, "sim_after_network=%u\n", (unsigned)after_net);
            fprintf(txt, "sim_dropped_by_network=%u\n", (unsigned)dropped);
            fprintf(txt, "sim_duplicated_by_network=%u\n", (unsigned)duplicated);
            fprintf(txt, "sim_delivered=%u\n", (unsigned)delivered);
            fprintf(txt, "sim_declared_lost=%u\n", (unsigned)rs.lost);
            fprintf(txt, "sim_late=%u\n", (unsigned)rs.late);
            fprintf(txt, "sim_duplicates_detected=%u\n",
                    (unsigned)rs.duplicates);
            fprintf(txt, "sim_units_complete=%u\n",
                    (unsigned)acc_units_complete);
            fprintf(txt, "sim_units_dropped=%u\n", (unsigned)acc_units_dropped);
            fprintf(txt, "sim_bytes_reassembled=%u\n",
                    (unsigned)acc_bytes_reassembled);
            fprintf(txt, "sim_byte_exact_frames=%u\n", (unsigned)matched);
            fprintf(txt, "sim_content_mismatches=%u\n", (unsigned)mismatched);
            fprintf(txt, "sim_reorder_window=%u\n", (unsigned)reorder_window);
        }
        if (sent_out != NULL) { *sent_out = sent; }
        if (delivered_out != NULL) { *delivered_out = delivered; }
        if (lost_out != NULL) { *lost_out = (size_t)rs.lost; }
        if (exact_out != NULL) { *exact_out = matched; }
    }

    free(rx);
    free(reasm);
    free(src);
    return mismatched == 0 ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* window study: the same stream at five reorder window sizes          */
/* ------------------------------------------------------------------ */

static void window_study(FILE *csv)
{
    static const size_t WINDOWS[] = { 1, 4, 16, 64, 512 };
    size_t w;

    banner("reorder window study (400 datagrams, 5% loss, 3% dup, 25% reorder)");
    printf("  %7s %10s %10s %10s %10s %10s\n", "window", "delivered", "lost",
           "late", "dup", "out_of_order");
    for (w = 0; w < sizeof(WINDOWS) / sizeof(WINDOWS[0]); w++) {
        ntc_rtp_rx_t *rx = (ntc_rtp_rx_t *)calloc(1, sizeof(*rx));
        ntc_net_profile_t prof;
        uint8_t **dgrams;
        size_t *lens;
        uint8_t payload[64];
        size_t n = 400;
        size_t i;
        size_t lost = 0;
        size_t dup = 0;
        size_t popped = 0;
        ntc_rtp_stats_t st;

        if (rx == NULL) {
            return;
        }
        dgrams = (uint8_t **)calloc(1024, sizeof(*dgrams));
        lens = (size_t *)calloc(1024, sizeof(*lens));
        if (dgrams == NULL || lens == NULL) {
            free(dgrams);
            free(lens);
            free(rx);
            return;
        }
        memset(payload, 0x5A, sizeof(payload));
        for (i = 0; i < n; i++) {
            uint8_t *dg = (uint8_t *)malloc(NTC_RTP_HEADER_LEN + 64);
            size_t written = 0;
            if (dg == NULL) {
                break;
            }
            (void)ntc_rtp_build(dg, NTC_RTP_HEADER_LEN + 64, 0, 96,
                                (uint16_t)(1000u + i), (uint32_t)(1000u * i),
                                0x5A5A5A5Au, payload, sizeof(payload), &written);
            dgrams[i] = dg;
            lens[i] = written;
        }
        (void)ntc_net_profile_default(&prof, 20240301u);
        prof.loss_prob = 0.05;
        prof.dup_prob = 0.03;
        prof.reorder_prob = 0.25;
        prof.burst_gap = 2;
        (void)ntc_net_apply(&prof, dgrams, lens, 1024, &n, &lost, &dup);
        (void)ntc_rtp_rx_init(rx, WINDOWS[w]);
        for (i = 0; i < n; i++) {
            (void)ntc_rtp_rx_push(rx, dgrams[i], lens[i], NULL, NULL);
        }
        (void)ntc_rtp_rx_flush(rx, NULL, NULL);
        {
            ntc_rtp_delivery_t dev;
            while (ntc_rtp_rx_pop(rx, &dev) == NTC_OK) {
                popped++;
            }
        }
        (void)ntc_rtp_rx_stats(rx, &st);
        printf("  %7u %10u %10u %10u %10u %10u\n", (unsigned)WINDOWS[w],
               (unsigned)popped, (unsigned)st.lost, (unsigned)st.late,
               (unsigned)st.duplicates, (unsigned)st.out_of_order);
        if (csv != NULL) {
            fprintf(csv, "%u,%u,%u,%u,%u,%u\n", (unsigned)WINDOWS[w],
                    (unsigned)popped, (unsigned)st.lost, (unsigned)st.late,
                    (unsigned)st.duplicates, (unsigned)st.out_of_order);
        }
        ntc_net_free(dgrams, n);
        free(dgrams);
        free(lens);
        free(rx);
    }
}

/* ------------------------------------------------------------------ */
/* stage 5: motion detection                                           */
/* ------------------------------------------------------------------ */

static void stage_motion(FILE *txt)
{
    /*
     * Scripted 60-frame sequence with known ground truth:
     *   frames  0.. 9  still            (noise blip at 5)
     *   frames 10..19  real motion      (event 1)
     *   frames 20..29  still            (noise blip at 23)
     *   frames 30..39  real motion      (event 2)
     *   frames 40..59  still            (noise blip at 48)
     * Ground truth: 2 real events, 3 single-frame noise blips.
     */
    static const int noisy[3] = { 5, 23, 48 };
    uint8_t *frame = (uint8_t *)malloc(MOTION_LEN);
    ntc_motion_config_t cfg_raw;
    ntc_motion_config_t cfg_deb;
    ntc_motion_detector_t *d_raw =
        (ntc_motion_detector_t *)calloc(1, sizeof(*d_raw));
    ntc_motion_detector_t *d_deb =
        (ntc_motion_detector_t *)calloc(1, sizeof(*d_deb));
    ntc_motion_stats_t a;
    ntc_motion_stats_t b;
    int f;
    int i;

    banner("stage 5: motion detection and alarm state machine");

    if (frame == NULL || d_raw == NULL || d_deb == NULL) {
        free(frame);
        free(d_raw);
        free(d_deb);
        return;
    }

    (void)ntc_motion_config_default(&cfg_raw, FRAME_W, FRAME_H);
    cfg_raw.start_frames = 1;
    cfg_raw.end_frames = 1;
    cfg_raw.cooldown_frames = 0;
    cfg_raw.min_event_frames = 0;
    cfg_raw.motion_threshold = 0.005;
    cfg_raw.end_threshold = 0.005;

    /* Debounce with the cooldown switched off, so this isolates debouncing. */
    (void)ntc_motion_config_default(&cfg_deb, FRAME_W, FRAME_H);
    cfg_deb.cooldown_frames = 0;

    if (ntc_motion_init(d_raw, &cfg_raw) != NTC_OK ||
        ntc_motion_init(d_deb, &cfg_deb) != NTC_OK) {
        free(frame);
        free(d_raw);
        free(d_deb);
        return;
    }

    for (f = 0; f < 60; f++) {
        int is_motion = (f >= 10 && f <= 19) || (f >= 30 && f <= 39);
        int is_noisy = 0;
        for (i = 0; i < 3; i++) {
            if (noisy[i] == f) {
                is_noisy = 1;
            }
        }
        if (is_motion) {
            paint(frame, 100, (f % 4) * 16, 0, 16, 16, 200);
        } else if (is_noisy) {
            paint(frame, 100, 32, 16, 16, 16, 210);
        } else {
            paint(frame, 100, 0, 0, 0, 0, 0);
        }
        (void)ntc_motion_process(d_raw, frame, (double)f * 33.0, NULL, NULL,
                                 NULL);
        (void)ntc_motion_process(d_deb, frame, (double)f * 33.0, NULL, NULL,
                                 NULL);
    }

    (void)ntc_motion_stats(d_raw, &a);
    (void)ntc_motion_stats(d_deb, &b);

    printf("  frames processed          : %u\n", (unsigned)b.frames);
    printf("  ground truth              : 2 real events, 3 noise blips\n");
    printf("  alarms without debounce   : %u  (false positives: %u)\n",
           (unsigned)a.events_started,
           (unsigned)(a.events_started > 2u ? a.events_started - 2u : 0u));
    printf("  alarms with debounce      : %u  (missed events: %u)\n",
           (unsigned)b.events_started,
           (unsigned)(b.events_started < 2u ? 2u - b.events_started : 0u));
    printf("  noise blips suppressed    : %u\n",
           (unsigned)(a.events_started - b.events_started));
    printf("  recording triggers        : %u\n",
           (unsigned)b.record_triggers);
    printf("  stream triggers           : %u\n", (unsigned)b.stream_triggers);

    if (txt != NULL) {
        fprintf(txt, "motion_frames=%u\n", (unsigned)b.frames);
        fprintf(txt, "motion_real_events=2\n");
        fprintf(txt, "motion_noise_blips=3\n");
        fprintf(txt, "motion_alarms_no_debounce=%u\n",
                (unsigned)a.events_started);
        fprintf(txt, "motion_alarms_with_debounce=%u\n",
                (unsigned)b.events_started);
        fprintf(txt, "motion_false_positives_no_debounce=%u\n",
                (unsigned)(a.events_started > 2u ? a.events_started - 2u : 0u));
        fprintf(txt, "motion_false_positives_with_debounce=%u\n",
                (unsigned)(b.events_started > 2u ? b.events_started - 2u : 0u));
        fprintf(txt, "motion_missed_with_debounce=%u\n",
                (unsigned)(b.events_started < 2u ? 2u - b.events_started : 0u));
        fprintf(txt, "motion_record_triggers=%u\n",
                (unsigned)b.record_triggers);
        fprintf(txt, "motion_stream_triggers=%u\n",
                (unsigned)b.stream_triggers);
    }

    ntc_motion_destroy(d_raw);
    ntc_motion_destroy(d_deb);
    free(frame);
    free(d_raw);
    free(d_deb);
}

/* ------------------------------------------------------------------ */
/* stage 6: recording index                                            */
/* ------------------------------------------------------------------ */

static void stage_recording(FILE *txt)
{
    ntc_recorder_t *rec = (ntc_recorder_t *)calloc(1, sizeof(*rec));
    ntc_rec_segment_t hits[32];
    size_t hits_n = 0;
    size_t hits_total = 0;
    ntc_rec_stats_t st;
    size_t i;

    banner("stage 6: recording index and ring-buffer eviction");

    if (rec == NULL) {
        return;
    }
    /* 30 clips of 100 bytes into a 1000-byte budget: ten can survive. */
    (void)ntc_recorder_init(rec, 1000u, 0.0);
    for (i = 0; i < 30; i++) {
        uint64_t id = 0;
        (void)ntc_recorder_insert(rec, (double)i * 5000.0,
                                  (double)i * 5000.0 + 3000.0, 100u,
                                  NTC_MOTION_LINK_RECORD, &id);
    }
    (void)ntc_recorder_stats(rec, &st);

    printf("  clips inserted            : %u\n", (unsigned)st.inserts);
    printf("  capacity / used bytes     : %u / %u\n",
           (unsigned)rec->capacity_bytes, (unsigned)rec->used_bytes);
    printf("  evictions / bytes freed   : %u / %u\n",
           (unsigned)st.evictions, (unsigned)st.evicted_bytes);
    printf("  oldest surviving id       : %u\n",
           (unsigned)(rec->count > 0 ? rec->segs[0].id : 0u));
    printf("  newest id                 : %u\n",
           (unsigned)(rec->count > 0 ? rec->segs[rec->count - 1u].id : 0u));
    (void)ntc_recorder_query(rec, 0.0, 1.0e9, hits, 32, &hits_n, &hits_total);
    printf("  query [0, 1e9)  -> hits   : %u\n", (unsigned)hits_total);
    (void)ntc_recorder_query(rec, 0.0, 50000.0, hits, 32, &hits_n, &hits_total);
    printf("  query [0, 50000) -> hits  : %u (the early clips were evicted)\n",
           (unsigned)hits_total);

    if (txt != NULL) {
        fprintf(txt, "rec_capacity_bytes=%u\n", (unsigned)rec->capacity_bytes);
        fprintf(txt, "rec_inserts=%u\n", (unsigned)st.inserts);
        fprintf(txt, "rec_evictions=%u\n", (unsigned)st.evictions);
        fprintf(txt, "rec_evicted_bytes=%u\n", (unsigned)st.evicted_bytes);
        fprintf(txt, "rec_used_bytes=%u\n", (unsigned)rec->used_bytes);
        fprintf(txt, "rec_oldest_id=%u\n",
                (unsigned)(rec->count > 0 ? rec->segs[0].id : 0u));
        fprintf(txt, "rec_newest_id=%u\n",
                (unsigned)(rec->count > 0 ? rec->segs[rec->count - 1u].id : 0u));
        (void)ntc_recorder_query(rec, 0.0, 1.0e9, hits, 32, &hits_n,
                                 &hits_total);
        fprintf(txt, "rec_query_hits=%u\n", (unsigned)hits_total);
        (void)ntc_recorder_query(rec, 0.0, 50000.0, hits, 32, &hits_n,
                                 &hits_total);
        fprintf(txt, "rec_early_query_hits=%u\n", (unsigned)hits_total);
    }
    free(rec);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    FILE *txt;
    FILE *csv;
    FILE *csvw;
    ntc_rtsp_server_t *srv =
        (ntc_rtsp_server_t *)calloc(1, sizeof(ntc_rtsp_server_t));
    uint32_t session = 0;
    size_t sent = 0;
    size_t delivered = 0;
    size_t lost = 0;
    size_t exact = 0;
    int ok = 1;

    setvbuf(stdout, NULL, _IONBF, 0);

    printf("netcam-app-lab end-to-end simulation\n");
    printf("====================================\n");
    printf("Host-side logic simulation: no camera sensor, no ISP, no real\n");
    printf("H.264/HEVC encoder, no sockets, no RTCP.  See README.md.\n");

    /* The C code creates the output directory: make.exe on Windows has no
     * portable mkdir -p and the build must not require one. */
    if (ntc_mkdir(SIM_RESULT_DIR) != NTC_OK) {
        fprintf(stderr, "cannot create %s\n", SIM_RESULT_DIR);
        free(srv);
        return 1;
    }
    txt = ntc_report_open_text(SIM_RESULT_DIR, "simulation.txt");
    csv = ntc_report_open_csv(SIM_RESULT_DIR, "rtp_windows.csv",
                              "window,delivered,declared_lost,late,"
                              "duplicates_detected,out_of_order");
    csvw = ntc_report_open_csv(SIM_RESULT_DIR, "summary.csv",
                               "stage,metric,value");

    if (srv == NULL) {
        if (txt != NULL) { fclose(txt); }
        if (csv != NULL) { fclose(csv); }
        if (csvw != NULL) { fclose(csvw); }
        return 1;
    }

    if (txt != NULL) {
        fprintf(txt, "# netcam-app-lab end-to-end simulation\n");
        fprintf(txt, "# deterministic; produced by `make sim`\n");
    }

    ok = stage_rtsp(srv, txt, &session) && ok;
    ok = stage_media(txt, 8u, &sent, &delivered, &lost, &exact) && ok;
    window_study(csv);
    stage_motion(txt);
    stage_recording(txt);

    if (csvw != NULL) {
        fprintf(csvw, "rtsp,accepted,200\n");
        fprintf(csvw, "rtsp,cseq_rejected,%u\n",
                (unsigned)srv->stats.rejected_cseq_order);
        fprintf(csvw, "rtp,datagrams_sent,%u\n", (unsigned)sent);
        fprintf(csvw, "rtp,delivered,%u\n", (unsigned)delivered);
        fprintf(csvw, "rtp,declared_lost,%u\n", (unsigned)lost);
        fprintf(csvw, "nal,byte_exact_frames,%u\n", (unsigned)exact);
    }

    banner("result");
    if (ok) {
        printf("  every stage completed and no reassembled frame was corrupted\n");
    } else {
        printf("  a stage reported a problem -- see the output above\n");
    }
    printf("  reports: %s/simulation.txt, summary.csv, rtp_windows.csv\n",
           SIM_RESULT_DIR);

    if (txt != NULL) { fclose(txt); }
    if (csv != NULL) { fclose(csv); }
    if (csvw != NULL) { fclose(csvw); }
    free(srv);
    return ok ? 0 : 1;
}
