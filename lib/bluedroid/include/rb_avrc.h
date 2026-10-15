#pragma once
/* Rockbox: now-playing info for the AVRCP target (GetElementAttributes,
 * GetPlayStatus). Implemented in apps/bluetooth/bt.c. */
#include <stdbool.h>
#include <stdint.h>

#define RB_AVRC_STR_LEN 64 /* keeps a full GetElementAttributes reply small */

typedef struct {
    bool has_track;
    char title[RB_AVRC_STR_LEN];
    char artist[RB_AVRC_STR_LEN];
    char album[RB_AVRC_STR_LEN];
    char genre[RB_AVRC_STR_LEN];
    uint32_t track_no;      /* position in the playlist, 1-based */
    uint32_t total_tracks;
    uint32_t length_ms;
    uint32_t position_ms;
    uint8_t play_status;    /* AVRC_PLAYSTATE_* */
} rb_avrc_track_info_t;

void rb_avrc_get_track_info(rb_avrc_track_info_t *info);
