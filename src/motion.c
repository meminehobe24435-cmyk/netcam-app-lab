/*
 * motion.c -- motion detection and the alarm state machine
 * =======================================================
 *
 * PIPELINE (all integer arithmetic except the final ratio)
 *
 *   1. FRAME DIFFERENCE
 *        For every pixel:  d = |cur[i] - bg[i]|
 *        A pixel "changed" when d > cfg.pixel_threshold.
 *        A block (cfg.block_size x cfg.block_size) is a "changed block" when
 *        at least half of its pixels changed.  Counting changed blocks
 *        instead of changed pixels makes the metric insensitive to thin
 *        noise speckle, which is why a block-based detector needs a lower
 *        start_frames debounce than a pure pixel-sum detector.
 *
 *   2. METRIC
 *        metric = changed_blocks / total_blocks        in [0,1]
 *        A frame is a "mover" when metric >= cfg.motion_threshold.
 *
 *   3. DEBOUNCE
 *        The detector needs cfg.start_frames consecutive mover frames before
 *        it raises an alarm, and cfg.end_frames consecutive still frames
 *        before it clears one.  The clear test uses the lower
 *        cfg.end_threshold, which is hysteresis: a slow-moving object whose
 *        metric hovers around the threshold does not chatter.
 *
 *   4. ALARM STATE MACHINE
 *
 *        IDLE --mover--> PENDING --start_frames consecutive movers--> ACTIVE
 *          ^                |                                             |
 *          |                +--a still frame here resets to IDLE---------+
 *          |                                                              |
 *          +--cooldown exhausted-- COOLDOWN <--end_frames stills---------+
 *
 *        ACTIVE -> COOLDOWN is where "minimum quiet period" lives: a new
 *        event cannot be raised until cooldown_frames frames have passed,
 *        which is the anti-storm rule.  Events that end up shorter than
 *        min_event_frames are counted as suppressed instead of reported.
 *
 *   5. BACKGROUND MODEL
 *        bg = bg + (cur - bg) / bg_alpha   applied on still frames only, so a
 *        parked object does not get absorbed into the background while the
 *        alarm is active.  bg_alpha = 1 means "bg becomes cur exactly"
 *        (essentially no memory); large bg_alpha means a slow-running model.
 *        Integer division is used deliberately: identical results on every
 *        platform, and no FMA/rounding surprises.
 *
 * LINKAGE
 *        On the rising edge the detector ORs cfg.link_flags into the event
 *        and counts one trigger per set bit (record / stream / snapshot), so
 *        the end-to-end run can report "alarms that actually started a
 *        recording".
 */

#include "netcam.h"

#include <stdlib.h>
#include <string.h>

ntc_status_t ntc_motion_config_default(ntc_motion_config_t *cfg,
                                       size_t width, size_t height)
{
    if (cfg == NULL) {
        return NTC_ERR_INVAL;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->width = width;
    cfg->height = height;
    cfg->block_size = 16;
    cfg->pixel_threshold = 20;
    cfg->motion_threshold = 0.02;
    cfg->end_threshold = 0.01;
    cfg->start_frames = 3;
    cfg->end_frames = 5;
    cfg->cooldown_frames = 25;
    cfg->min_event_frames = 5;
    cfg->bg_alpha = 8;
    cfg->link_flags = NTC_MOTION_LINK_RECORD | NTC_MOTION_LINK_STREAM;
    return NTC_OK;
}

ntc_status_t ntc_motion_config_check(const ntc_motion_config_t *cfg)
{
    if (cfg == NULL) {
        return NTC_ERR_INVAL;
    }
    if (cfg->width == 0 || cfg->height == 0) {
        return NTC_MOT_ERR_CONFIG;
    }
    if (cfg->block_size == 0 || cfg->block_size > cfg->width ||
        cfg->block_size > cfg->height) {
        return NTC_MOT_ERR_CONFIG;
    }
    if (cfg->pixel_threshold <= 0 || cfg->pixel_threshold > 255) {
        return NTC_MOT_ERR_CONFIG;
    }
    if (!(cfg->motion_threshold > 0.0) || cfg->motion_threshold > 1.0) {
        return NTC_MOT_ERR_CONFIG;
    }
    if (cfg->end_threshold < 0.0 ||
        cfg->end_threshold > cfg->motion_threshold) {
        return NTC_MOT_ERR_CONFIG;
    }
    if (cfg->start_frames < 1 || cfg->end_frames < 1) {
        return NTC_MOT_ERR_CONFIG;
    }
    if (cfg->cooldown_frames < 0 || cfg->min_event_frames < 0) {
        return NTC_MOT_ERR_CONFIG;
    }
    if (cfg->bg_alpha < 1) {
        return NTC_MOT_ERR_CONFIG;
    }
    return NTC_OK;
}

ntc_status_t ntc_motion_init(ntc_motion_detector_t *d,
                             const ntc_motion_config_t *cfg)
{
    ntc_status_t st;
    size_t len;
    size_t bx;
    size_t by;

    if (d == NULL || cfg == NULL) {
        return NTC_ERR_INVAL;
    }
    st = ntc_motion_config_check(cfg);
    if (st != NTC_OK) {
        return st;
    }
    memset(d, 0, sizeof(*d));
    d->cfg = *cfg;
    len = cfg->width * cfg->height;
    d->background = (uint8_t *)calloc(len, 1);
    if (d->background == NULL) {
        return NTC_ERR_NOMEM;
    }
    d->bg_len = len;
    d->background_ready = 0;
    d->state = NTC_MOTION_IDLE;
    bx = (cfg->width + cfg->block_size - 1u) / cfg->block_size;
    by = (cfg->height + cfg->block_size - 1u) / cfg->block_size;
    d->total_blocks = bx * by;
    return NTC_OK;
}

void ntc_motion_destroy(ntc_motion_detector_t *d)
{
    if (d == NULL) {
        return;
    }
    free(d->background);
    d->background = NULL;
    d->bg_len = 0;
}

ntc_status_t ntc_motion_measure(const ntc_motion_detector_t *d,
                                const uint8_t *frame, double *ratio_out,
                                size_t *changed_out)
{
    size_t bx_count;
    size_t by_count;
    size_t changed = 0;
    size_t total;

    if (d == NULL || frame == NULL) {
        return NTC_ERR_INVAL;
    }
    if (d->background == NULL || d->bg_len != d->cfg.width * d->cfg.height) {
        return NTC_MOT_ERR_CONFIG;
    }
    bx_count = (d->cfg.width + d->cfg.block_size - 1u) / d->cfg.block_size;
    by_count = (d->cfg.height + d->cfg.block_size - 1u) / d->cfg.block_size;
    total = bx_count * by_count;

    {
        size_t by;
        for (by = 0; by < by_count; by++) {
            size_t bx;
            for (bx = 0; bx < bx_count; bx++) {
                size_t x0 = bx * d->cfg.block_size;
                size_t y0 = by * d->cfg.block_size;
                size_t x1 = x0 + d->cfg.block_size;
                size_t y1 = y0 + d->cfg.block_size;
                size_t pixels = 0;
                size_t hit = 0;
                size_t y;

                if (x1 > d->cfg.width) { x1 = d->cfg.width; }
                if (y1 > d->cfg.height) { y1 = d->cfg.height; }
                for (y = y0; y < y1; y++) {
                    size_t x;
                    size_t row = y * d->cfg.width;
                    for (x = x0; x < x1; x++) {
                        int cur = (int)frame[row + x];
                        int bgv = (int)d->background[row + x];
                        int diff = cur - bgv;
                        if (diff < 0) { diff = -diff; }
                        if (diff > d->cfg.pixel_threshold) {
                            hit++;
                        }
                        pixels++;
                    }
                }
                if (pixels > 0u && hit * 2u >= pixels) {
                    changed++;
                }
            }
        }
    }

    if (changed_out != NULL) { *changed_out = changed; }
    if (ratio_out != NULL) {
        *ratio_out = (total == 0u) ? 0.0
                                   : (double)changed / (double)total;
    }
    return NTC_OK;
}

/* Update the background on a still frame: bg += (cur - bg) / alpha. */
static void ntc_motion_update_background(ntc_motion_detector_t *d,
                                         const uint8_t *frame)
{
    size_t i;
    int alpha = d->cfg.bg_alpha;
    for (i = 0; i < d->bg_len; i++) {
        int cur = (int)frame[i];
        int bgv = (int)d->background[i];
        int step = (cur - bgv);
        /* integer division toward zero; alpha >= 1 guaranteed by config */
        if (alpha > 1) {
            step /= alpha;
        }
        d->background[i] = (uint8_t)(bgv + step);
    }
}

ntc_status_t ntc_motion_process(ntc_motion_detector_t *d, const uint8_t *frame,
                                double now_ms, double *motion_out,
                                int *event_started_out, int *event_ended_out)
{
    double ratio = 0.0;
    size_t changed = 0;
    int mover;
    int started = 0;
    int ended = 0;

    if (event_started_out != NULL) { *event_started_out = 0; }
    if (event_ended_out != NULL) { *event_ended_out = 0; }
    if (d == NULL || frame == NULL) {
        return NTC_ERR_INVAL;
    }
    if (d->background == NULL) {
        return NTC_MOT_ERR_CONFIG;
    }
    if (ntc_motion_measure(d, frame, &ratio, &changed) != NTC_OK) {
        return NTC_MOT_ERR_CONFIG;
    }

    d->stats.frames++;
    d->stats.motion_sum += ratio;
    d->last_motion = ratio;
    d->last_changed = changed;

    if (!d->background_ready) {
        /* First frame defines the background; it is never a "mover". */
        memcpy(d->background, frame, d->bg_len);
        d->background_ready = 1;
        d->stats.still_frames++;
        if (motion_out != NULL) { *motion_out = 0.0; }
        return NTC_OK;
    }

    mover = (ratio >= d->cfg.motion_threshold) ? 1 : 0;
    if (d->state == NTC_MOTION_ACTIVE) {
        mover = (ratio >= d->cfg.end_threshold) ? 1 : 0;
    }

    if (mover) {
        d->stats.mover_frames++;
        d->consecutive_movers++;
        d->consecutive_stills = 0;
    } else {
        d->stats.still_frames++;
        d->consecutive_stills++;
        d->consecutive_movers = 0;
        ntc_motion_update_background(d, frame);
    }

    if (d->state == NTC_MOTION_ACTIVE) {
        d->event_frames++;
        d->stats.frames_while_active++;
        if (!mover && d->consecutive_stills >= d->cfg.end_frames) {
            d->event_end_ms = now_ms;
            d->state = NTC_MOTION_COOLDOWN;
            d->cooldown_left = d->cfg.cooldown_frames;
            ended = 1;
            if (d->event_frames < d->cfg.min_event_frames) {
                /* Too short to be a real event: count it, but the alarm we
                 * already raised stays counted as raised -- we only record
                 * that it was suppressed rather than "reported long". */
                d->stats.events_suppressed_short++;
            }
            d->stats.events_ended++;
        }
    } else if (d->state == NTC_MOTION_PENDING) {
        if (!mover) {
            d->state = NTC_MOTION_IDLE;
        } else if (d->consecutive_movers >= d->cfg.start_frames) {
            d->state = NTC_MOTION_ACTIVE;
            d->event_frames = 0;
            d->event_start_ms = now_ms;
            d->stats.events_started++;
            if (d->cfg.link_flags & NTC_MOTION_LINK_RECORD) {
                d->stats.record_triggers++;
            }
            if (d->cfg.link_flags & NTC_MOTION_LINK_STREAM) {
                d->stats.stream_triggers++;
            }
            if (d->cfg.link_flags & NTC_MOTION_LINK_SNAPSHOT) {
                d->stats.snapshot_triggers++;
            }
            started = 1;
        }
    } else if (d->state == NTC_MOTION_COOLDOWN) {
        if (mover) {
            /* Motion resumed inside the quiet period: suppress a new event
             * (that is the point of the cooldown) and go straight back to
             * ACTIVE so the ongoing motion is not lost. */
            d->state = NTC_MOTION_ACTIVE;
            d->event_frames = 0;
            d->stats.events_suppressed_cooldown++;
        } else if (d->cooldown_left > 0) {
            d->cooldown_left--;
            if (d->cooldown_left == 0) {
                d->state = NTC_MOTION_IDLE;
            }
        } else {
            d->state = NTC_MOTION_IDLE;
        }
    } else { /* IDLE */
        if (mover) {
            d->stats.events_raw++;
            if (d->cfg.start_frames <= 1) {
                d->state = NTC_MOTION_ACTIVE;
                d->event_frames = 0;
                d->event_start_ms = now_ms;
                d->stats.events_started++;
                if (d->cfg.link_flags & NTC_MOTION_LINK_RECORD) {
                    d->stats.record_triggers++;
                }
                if (d->cfg.link_flags & NTC_MOTION_LINK_STREAM) {
                    d->stats.stream_triggers++;
                }
                if (d->cfg.link_flags & NTC_MOTION_LINK_SNAPSHOT) {
                    d->stats.snapshot_triggers++;
                }
                started = 1;
            } else {
                d->state = NTC_MOTION_PENDING;
            }
        }
    }

    if (motion_out != NULL) { *motion_out = ratio; }
    if (event_started_out != NULL) { *event_started_out = started; }
    if (event_ended_out != NULL) { *event_ended_out = ended; }
    return NTC_OK;
}

ntc_status_t ntc_motion_stats(const ntc_motion_detector_t *d,
                              ntc_motion_stats_t *out)
{
    if (d == NULL || out == NULL) {
        return NTC_ERR_INVAL;
    }
    *out = d->stats;
    return NTC_OK;
}

ntc_motion_state_t ntc_motion_state(const ntc_motion_detector_t *d)
{
    if (d == NULL) {
        return NTC_MOTION_IDLE;
    }
    return d->state;
}

const char *ntc_motion_state_str(ntc_motion_state_t s)
{
    switch (s) {
    case NTC_MOTION_IDLE:     return "IDLE";
    case NTC_MOTION_PENDING:  return "PENDING";
    case NTC_MOTION_ACTIVE:   return "ACTIVE";
    case NTC_MOTION_COOLDOWN: return "COOLDOWN";
    default:                  return "?";
    }
}
