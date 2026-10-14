/*
 * Copyright 2016 The Android Open Source Project
 * Copyright 2026 Skye Green
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "common/bt_defs.h"
#include "esp_a2dp_api.h"
#include "tick.h"
#include "bluetooth/codecs.h"
#include "bluetooth/bt.h"
#include "a2dp_vendor_ldac_constants.h"
#include "stack/avdt_api.h"

#include "ldacBT.h"
#include "ldacBT_abr.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

//
// Encoder for LDAC Source Codec
//

// Initial EQMID for ABR mode.
#define LDAC_ABR_MODE_EQMID LDACBT_EQMID_SQ

// A2DP LDAC encoder interval in milliseconds
#define A2DP_LDAC_ENCODER_INTERVAL_MS 20
#define A2DP_LDAC_MEDIA_BYTES_PER_FRAME 128

// offset
#define A2DP_LDAC_OFFSET (AVDT_MEDIA_OFFSET + A2DP_LDAC_MPL_HDR_LEN)

typedef struct {
  uint32_t sample_rate;
  uint8_t channel_mode;
  uint8_t bits_per_sample;
  int quality_mode_index;
  int pcm_wlength;
  LDACBT_SMPL_FMT_T pcm_fmt;
} tA2DP_LDAC_ENCODER_PARAMS;

typedef struct {
  float counter;
  uint32_t bytes_per_tick; /* pcm bytes read each media task tick */
  uint64_t last_frame_us;
} tA2DP_LDAC_FEEDING_STATE;

typedef struct {
  uint64_t session_start_us;

  size_t media_read_total_expected_packets;
  size_t media_read_total_expected_reads_count;
  size_t media_read_total_expected_read_bytes;

  size_t media_read_total_dropped_packets;
  size_t media_read_total_actual_reads_count;
  size_t media_read_total_actual_read_bytes;
} a2dP_ldac_encoder_stats_t;

typedef struct {
  uint16_t TxAaMtuSize;
  size_t TxQueueLength;

  bool use_SCMS_T;
  esp_a2d_mcc_t peer_params;
  uint16_t peer_mtu;
  bool peer_is_edr;
  esp_a2d_conn_hdl_t peer;
  uint32_t timestamp;  // Timestamp for the A2DP frames

  HANDLE_LDAC_BT ldac_handle;
  bool has_ldac_handle;  // True if ldac_handle is valid

  HANDLE_LDAC_ABR ldac_abr_handle;
  bool has_ldac_abr_handle;
  int last_ldac_abr_eqmid;
  size_t ldac_abr_adjustments;

  tA2DP_LDAC_ENCODER_PARAMS ldac_encoder_params;
  tA2DP_LDAC_FEEDING_STATE ldac_feeding_state;

  a2dP_ldac_encoder_stats_t stats;
} tA2DP_LDAC_ENCODER_CB;

static bool ldac_abr_loaded = true;  // the library is statically linked

static tA2DP_LDAC_ENCODER_CB a2dP_ldac_encoder_cb;

static void a2dp_vendor_ldac_encoder_update(uint32_t freq, esp_a2d_mcc_t *pref_mcc,
                                            bool* p_restart_input, bool* p_restart_output,
                                            bool* p_config_updated);
static void a2dP_ldac_get_num_frame_iteration(uint8_t* num_of_iterations, uint8_t* num_of_frames,
                                              uint64_t timestamp_us);
static void a2dP_ldac_encode_frames(uint8_t nb_frame);
static bool a2dP_ldac_read_feeding(uint8_t* read_buffer, uint32_t* bytes_read);
static uint16_t adjust_effective_mtu(void);
static void a2dp_vendor_ldac_encoder_cleanup(void);
static void a2dp_vendor_ldac_feeding_reset(void);

unsigned long bt_ldac_sampr[4] =
{
    SAMPR_96,
    SAMPR_88,
    SAMPR_48,
    SAMPR_44,
};

void a2dp_vendor_ldac_encoder_init(struct pcm_sink_caps *caps, const struct a2dp_peer_info* peer, esp_a2d_mcc_t *pref_mcc) {
  a2dp_vendor_ldac_encoder_cleanup();

  int i = 0;
  if (peer->peer_caps->cie.vs_info.mcc.ldac_info.samp_freq & A2DP_LDAC_SAMPLING_FREQ_96000) {
    bt_ldac_sampr[i++] = SAMPR_96;
  }
  if (peer->peer_caps->cie.vs_info.mcc.ldac_info.samp_freq & A2DP_LDAC_SAMPLING_FREQ_88200) {
    bt_ldac_sampr[i++] = SAMPR_88;
  }
  if (peer->peer_caps->cie.vs_info.mcc.ldac_info.samp_freq & A2DP_LDAC_SAMPLING_FREQ_48000) {
    bt_ldac_sampr[i++] = SAMPR_48;
  }
  if (peer->peer_caps->cie.vs_info.mcc.ldac_info.samp_freq & A2DP_LDAC_SAMPLING_FREQ_44100) {
    bt_ldac_sampr[i++] = SAMPR_44;
  }
  caps->samprs = bt_ldac_sampr;
  caps->default_freq = 0;
  caps->num_samprs = i;
  caps->volume_type = PCM_SINK_HWVOL;

  a2dP_ldac_encoder_cb.stats.session_start_us = current_tick;

  a2dP_ldac_encoder_cb.peer_params = *peer->peer_caps;
  a2dP_ldac_encoder_cb.peer_mtu = peer->peer_mtu;
  a2dP_ldac_encoder_cb.peer = peer->conn_hdl;
  a2dP_ldac_encoder_cb.peer_is_edr = peer->is_edr;
  a2dP_ldac_encoder_cb.timestamp = 0;
  a2dP_ldac_encoder_cb.ldac_abr_handle = NULL;
  a2dP_ldac_encoder_cb.has_ldac_abr_handle = false;
  a2dP_ldac_encoder_cb.last_ldac_abr_eqmid = -1;
  a2dP_ldac_encoder_cb.ldac_abr_adjustments = 0;
  a2dP_ldac_encoder_cb.TxQueueLength = 0;

  a2dP_ldac_encoder_cb.use_SCMS_T = false;

  // NOTE: Ignore the restart_input / restart_output flags - this initization
  // happens when the audio session is (re)started.
  bool restart_input = false;
  bool restart_output = false;
  bool config_updated = false;
  a2dp_vendor_ldac_encoder_update(44100, pref_mcc, &restart_input, &restart_output,
                                  &config_updated);
}

// Update the A2DP LDAC encoder.
// |a2dp_codec_config| is the A2DP codec to use for the update.
static void a2dp_vendor_ldac_encoder_update(uint32_t freq, esp_a2d_mcc_t *pref_mcc,
                                            bool* p_restart_input, bool* p_restart_output,
                                            bool* p_config_updated) {
  tA2DP_LDAC_ENCODER_PARAMS* p_encoder_params = &a2dP_ldac_encoder_cb.ldac_encoder_params;

  pref_mcc->losc = A2DP_LDAC_CODEC_LEN;
  pref_mcc->media_type = A2D_MEDIA_TYPE_AUDIO;
  pref_mcc->codec_type = ESP_A2D_MCT_NON_A2DP;
  pref_mcc->cie.vs_info.vendor_id = A2DP_LDAC_VENDOR_ID;
  pref_mcc->cie.vs_info.codec_id = A2DP_LDAC_CODEC_ID;
  pref_mcc->cie.vs_info.mcc.ldac_info.unknown0 = 0;
  pref_mcc->cie.vs_info.mcc.ldac_info.unknown1 = 0;
  pref_mcc->cie.vs_info.mcc.ldac_info.ch_mode = A2DP_LDAC_CHANNEL_MODE_STEREO;
  switch (freq) {
    case 44100:
      pref_mcc->cie.vs_info.mcc.ldac_info.samp_freq = A2DP_LDAC_SAMPLING_FREQ_44100;
      break;
    case 48000:
      pref_mcc->cie.vs_info.mcc.ldac_info.samp_freq = A2DP_LDAC_SAMPLING_FREQ_48000;
      break;
    case 88200:
      pref_mcc->cie.vs_info.mcc.ldac_info.samp_freq = A2DP_LDAC_SAMPLING_FREQ_88200;
      break;
    case 96000:
      pref_mcc->cie.vs_info.mcc.ldac_info.samp_freq = A2DP_LDAC_SAMPLING_FREQ_96000;
      break;
  }

  *p_restart_input = false;
  *p_restart_output = false;
  *p_config_updated = false;

  if (!a2dP_ldac_encoder_cb.has_ldac_handle) {
    a2dP_ldac_encoder_cb.ldac_handle = ldacBT_get_handle();
    if (a2dP_ldac_encoder_cb.ldac_handle == NULL) {
      return;  // TODO: Return an error?
    }
    a2dP_ldac_encoder_cb.has_ldac_handle = true;
  } else {
    ldacBT_close_handle(a2dP_ldac_encoder_cb.ldac_handle);
  }
  assert(a2dP_ldac_encoder_cb.ldac_handle != NULL);

  a2dp_vendor_ldac_feeding_reset();

  // The codec parameters
  p_encoder_params->sample_rate = freq;
  p_encoder_params->channel_mode = A2DP_LDAC_CHANNEL_MODE_STEREO;

  // Set the quality mode index
  int old_quality_mode_index = p_encoder_params->quality_mode_index;
  p_encoder_params->quality_mode_index = A2DP_LDAC_QUALITY_ABR;

  int ldac_eqmid = LDAC_ABR_MODE_EQMID;
  if (p_encoder_params->quality_mode_index == A2DP_LDAC_QUALITY_ABR) {
    if (!ldac_abr_loaded) {
      p_encoder_params->quality_mode_index = A2DP_LDAC_QUALITY_MID;
    } else {
      if (a2dP_ldac_encoder_cb.ldac_abr_handle != NULL) {
      } else {
        a2dP_ldac_encoder_cb.ldac_abr_handle = ldac_ABR_get_handle();
        if (a2dP_ldac_encoder_cb.ldac_abr_handle != NULL) {
          a2dP_ldac_encoder_cb.has_ldac_abr_handle = true;
          a2dP_ldac_encoder_cb.last_ldac_abr_eqmid = -1;
          a2dP_ldac_encoder_cb.ldac_abr_adjustments = 0;
          ldac_ABR_Init(a2dP_ldac_encoder_cb.ldac_abr_handle, A2DP_LDAC_ENCODER_INTERVAL_MS);
        } else {
          p_encoder_params->quality_mode_index = A2DP_LDAC_QUALITY_MID;
        }
      }
    }
  } else {
    ldac_eqmid = p_encoder_params->quality_mode_index;
    if (a2dP_ldac_encoder_cb.has_ldac_abr_handle) {
      ldac_ABR_free_handle(a2dP_ldac_encoder_cb.ldac_abr_handle);
      a2dP_ldac_encoder_cb.ldac_abr_handle = NULL;
      a2dP_ldac_encoder_cb.has_ldac_abr_handle = false;
      a2dP_ldac_encoder_cb.last_ldac_abr_eqmid = -1;
      a2dP_ldac_encoder_cb.ldac_abr_adjustments = 0;
    }
  }

  if (p_encoder_params->quality_mode_index != old_quality_mode_index) {
    *p_config_updated = true;
  }

  p_encoder_params->pcm_wlength = 2;
  // Set the Audio format from pcm_wlength
  p_encoder_params->pcm_fmt = LDACBT_SMPL_FMT_S16;
  if (p_encoder_params->pcm_wlength == 2) {
    p_encoder_params->pcm_fmt = LDACBT_SMPL_FMT_S16;
  } else if (p_encoder_params->pcm_wlength == 3) {
    p_encoder_params->pcm_fmt = LDACBT_SMPL_FMT_S24;
  } else if (p_encoder_params->pcm_wlength == 4) {
    p_encoder_params->pcm_fmt = LDACBT_SMPL_FMT_S32;
  }

  a2dP_ldac_encoder_cb.TxAaMtuSize = adjust_effective_mtu();

  // Initialize the encoder.
  // NOTE: MTU in the initialization must include the AVDT media header size.
  int result = ldacBT_init_handle_encode(a2dP_ldac_encoder_cb.ldac_handle,
                                         a2dP_ldac_encoder_cb.TxAaMtuSize + AVDT_MEDIA_HDR_SIZE,
                                         ldac_eqmid, p_encoder_params->channel_mode,
                                         p_encoder_params->pcm_fmt, p_encoder_params->sample_rate);
  if (result != 0) {
    int err_code = ldacBT_get_error_code(a2dP_ldac_encoder_cb.ldac_handle);
  }
}

void a2dp_vendor_ldac_encoder_cleanup(void) {
  if (a2dP_ldac_encoder_cb.has_ldac_abr_handle) {
    ldac_ABR_free_handle(a2dP_ldac_encoder_cb.ldac_abr_handle);
  }
  if (a2dP_ldac_encoder_cb.has_ldac_handle) {
    ldacBT_free_handle(a2dP_ldac_encoder_cb.ldac_handle);
  }
  memset(&a2dP_ldac_encoder_cb, 0, sizeof(a2dP_ldac_encoder_cb));
}

void a2dp_vendor_ldac_feeding_reset(void) {
  /* By default, just clear the entire state */
  memset(&a2dP_ldac_encoder_cb.ldac_feeding_state, 0,
         sizeof(a2dP_ldac_encoder_cb.ldac_feeding_state));

  a2dP_ldac_encoder_cb.ldac_feeding_state.bytes_per_tick =
          (a2dP_ldac_encoder_cb.ldac_encoder_params.sample_rate *
           16 / 8 *
           2 * A2DP_LDAC_ENCODER_INTERVAL_MS) /
          1000;

}

void a2dp_vendor_ldac_feeding_flush(void) {
  a2dP_ldac_encoder_cb.ldac_feeding_state.counter = 0.0f;
}

uint64_t a2dp_vendor_ldac_get_encoder_interval_ms(void) { return A2DP_LDAC_ENCODER_INTERVAL_MS; }

int a2dp_vendor_ldac_get_effective_frame_size(void) { return a2dP_ldac_encoder_cb.TxAaMtuSize; }

void a2dp_vendor_ldac_send_frames(void) {
  a2dP_ldac_encoder_cb.TxQueueLength = esp_a2d_source_audio_queue_len();
  uint64_t timestamp_us = ((uint64_t) current_tick) * (1000000 / HZ);
  uint8_t nb_frame = 0;
  uint8_t nb_iterations = 0;

  a2dP_ldac_get_num_frame_iteration(&nb_iterations, &nb_frame, timestamp_us);
  if (nb_frame == 0) {
    return;
  }

  for (uint8_t counter = 0; counter < nb_iterations; counter++) {
    if (a2dP_ldac_encoder_cb.has_ldac_abr_handle) {
      int flag_enable = 1;
      int prev_eqmid = a2dP_ldac_encoder_cb.last_ldac_abr_eqmid;
      a2dP_ldac_encoder_cb.last_ldac_abr_eqmid =
              ldac_ABR_Proc(a2dP_ldac_encoder_cb.ldac_handle, a2dP_ldac_encoder_cb.ldac_abr_handle,
                            a2dP_ldac_encoder_cb.TxQueueLength, flag_enable);
      if (prev_eqmid != a2dP_ldac_encoder_cb.last_ldac_abr_eqmid) {
        a2dP_ldac_encoder_cb.ldac_abr_adjustments++;
      }
    }
    // Transcode frame and enqueue
    a2dP_ldac_encode_frames(nb_frame);
  }
}

// Obtains the number of frames to send and number of iterations
// to be used. |num_of_iterations| and |num_of_frames| parameters
// are used as output param for returning the respective values.
static void a2dP_ldac_get_num_frame_iteration(uint8_t* num_of_iterations, uint8_t* num_of_frames,
                                              uint64_t timestamp_us) {
  uint32_t result = 0;
  uint8_t nof = 0;
  uint8_t noi = 1;

  uint32_t pcm_bytes_per_frame = A2DP_LDAC_MEDIA_BYTES_PER_FRAME *
                                 2 *
                                 16 / 8;

  uint32_t us_this_tick = A2DP_LDAC_ENCODER_INTERVAL_MS * 1000;
  uint64_t now_us = timestamp_us;
  if (a2dP_ldac_encoder_cb.ldac_feeding_state.last_frame_us != 0) {
    us_this_tick = (now_us - a2dP_ldac_encoder_cb.ldac_feeding_state.last_frame_us);
  }
  a2dP_ldac_encoder_cb.ldac_feeding_state.last_frame_us = now_us;

  a2dP_ldac_encoder_cb.ldac_feeding_state.counter +=
          (float)a2dP_ldac_encoder_cb.ldac_feeding_state.bytes_per_tick * us_this_tick /
          (A2DP_LDAC_ENCODER_INTERVAL_MS * 1000);

  result = a2dP_ldac_encoder_cb.ldac_feeding_state.counter / pcm_bytes_per_frame;
  a2dP_ldac_encoder_cb.ldac_feeding_state.counter -= result * pcm_bytes_per_frame;
  nof = result;

  *num_of_frames = nof;
  *num_of_iterations = noi;
}

static void a2dP_ldac_encode_frames(uint8_t nb_frame) {
  tA2DP_LDAC_ENCODER_PARAMS* p_encoder_params = &a2dP_ldac_encoder_cb.ldac_encoder_params;
  uint8_t remain_nb_frame = nb_frame;
  uint16_t ldac_frame_size;
  uint8_t read_buffer[LDACBT_MAX_LSU * 4 /* byte/sample */ * 2 /* ch */];

  switch (p_encoder_params->sample_rate) {
    case 176400:
    case 192000:
      ldac_frame_size = 512;  // sample/ch
      break;
    case 88200:
    case 96000:
      ldac_frame_size = 256;  // sample/ch
      break;
    case 44100:
    case 48000:
    default:
      ldac_frame_size = 128;  // sample/ch
      break;
  }

  uint32_t count;
  int32_t encode_count = 0;
  int32_t out_frames = 0;
  int written = 0;

  uint32_t bytes_read = 0;
  while (nb_frame) {
    esp_a2d_audio_buff_t* p_buf = esp_a2d_audio_buff_alloc(BT_DEFAULT_BUFFER_SIZE);
    if (p_buf == NULL) {
      return;
    }
    p_buf->data_len = 1;

    a2dP_ldac_encoder_cb.stats.media_read_total_expected_packets++;

    count = 0;
    do {
      //
      // Read the PCM data and encode it
      //
      uint32_t temp_bytes_read = 0;
      if (a2dP_ldac_read_feeding(read_buffer, &temp_bytes_read)) {
        bytes_read += temp_bytes_read;
        uint8_t* packet = p_buf->data + p_buf->data_len;
        if (a2dP_ldac_encoder_cb.ldac_handle == NULL) {
          a2dP_ldac_encoder_cb.stats.media_read_total_dropped_packets++;
          esp_a2d_audio_buff_free(p_buf);
          return;
        }
        int result =
                ldacBT_encode(a2dP_ldac_encoder_cb.ldac_handle, read_buffer, (int*)&encode_count,
                              packet + count, (int*)&written, (int*)&out_frames);
        if (result != 0) {
          int err_code = ldacBT_get_error_code(a2dP_ldac_encoder_cb.ldac_handle);
          a2dP_ldac_encoder_cb.stats.media_read_total_dropped_packets++;
          esp_a2d_audio_buff_free(p_buf);
          return;
        }
        count += written;
        p_buf->data_len += written;
        nb_frame--;
        p_buf->number_frame += out_frames;  // added a frame to the buffer
      } else {
        a2dP_ldac_encoder_cb.ldac_feeding_state.counter +=
                nb_frame * LDACBT_ENC_LSU * 2 *
                16 / 8;

        // no more pcm to read
        nb_frame = 0;
      }
    } while ((written == 0) && nb_frame);

    if (p_buf->data_len > 1) {
      /*
       * Timestamp of the media packet header represent the TS of the
       * first frame, i.e the timestamp before including this frame.
       */
      p_buf->timestamp = a2dP_ldac_encoder_cb.timestamp;
      p_buf->data[0] = p_buf->number_frame & 0x0F;

      // Timestamp will wrap over to 0 if stream continues on long enough
      // (>25H @ 48KHz). The parameters are promoted to 64bit to ensure that
      // no unsigned overflow is triggered as ubsan is always enabled.
      a2dP_ldac_encoder_cb.timestamp = ((uint64_t)a2dP_ldac_encoder_cb.timestamp +
                                        (p_buf->number_frame * ldac_frame_size)) &
                                       UINT32_MAX;

      uint8_t done_nb_frame = remain_nb_frame - nb_frame;
      remain_nb_frame = nb_frame;
      if (esp_a2d_source_audio_data_send(a2dP_ldac_encoder_cb.peer, p_buf) != ESP_OK) {
        esp_a2d_audio_buff_free(p_buf);
        return;
      }
    } else {
      // NOTE: Unlike the execution path for other codecs, it is normal for
      // LDAC to NOT write encoded data to the last buffer if there wasn't
      // enough data to write to. That data is accumulated internally by
      // the codec and included in the next iteration. Therefore, here we
      // don't increment the "media_read_total_dropped_packets" counter.
      esp_a2d_audio_buff_free(p_buf);
    }
  }
}

static bool a2dP_ldac_read_feeding(uint8_t* read_buffer, uint32_t* bytes_read) {
  uint32_t read_size = LDACBT_ENC_LSU * 2 *
                       16 / 8;

  a2dP_ldac_encoder_cb.stats.media_read_total_expected_reads_count++;
  a2dP_ldac_encoder_cb.stats.media_read_total_expected_read_bytes += read_size;

  /* Read Data from UIPC channel */
  uint32_t nb_byte_read = bt_read_pcm(read_buffer, read_size);
  a2dP_ldac_encoder_cb.stats.media_read_total_actual_read_bytes += nb_byte_read;

  if (nb_byte_read < read_size) {
    if (nb_byte_read == 0) {
      return false;
    }

    /* Fill the unfilled part of the read buffer with silence (0) */
    memset(((uint8_t*)read_buffer) + nb_byte_read, 0, read_size - nb_byte_read);
    nb_byte_read = read_size;
  }
  a2dP_ldac_encoder_cb.stats.media_read_total_actual_reads_count++;

  *bytes_read = nb_byte_read;
  return true;
}

static uint16_t adjust_effective_mtu(void) {
  uint16_t mtu_size = BT_DEFAULT_BUFFER_SIZE - A2DP_LDAC_OFFSET - sizeof(BT_HDR);
  if (mtu_size > a2dP_ldac_encoder_cb.peer_mtu) {
    mtu_size = a2dP_ldac_encoder_cb.peer_mtu;
  }
  return mtu_size;
}

void a2dp_vendor_ldac_set_transmit_queue_length(size_t transmit_queue_length) {
  a2dP_ldac_encoder_cb.TxQueueLength = transmit_queue_length;
}

static bool ldac_is_acceptable(const esp_a2d_mcc_t *peer_caps)
{
    if (peer_caps->losc != A2DP_LDAC_CODEC_LEN
        || peer_caps->media_type != A2D_MEDIA_TYPE_AUDIO
        || peer_caps->codec_type != ESP_A2D_MCT_NON_A2DP
        || peer_caps->cie.vs_info.vendor_id != A2DP_LDAC_VENDOR_ID
        || peer_caps->cie.vs_info.codec_id != A2DP_LDAC_CODEC_ID) {
        return false;
    }
    if (peer_caps->cie.vs_info.mcc.ldac_info.samp_freq & A2DP_LDAC_SAMPLING_FREQ_44100 == 0
        || peer_caps->cie.vs_info.mcc.ldac_info.ch_mode & A2DP_LDAC_CHANNEL_MODE_STEREO == 0
        || peer_caps->cie.vs_info.mcc.ldac_info.unknown0 != 0
        || peer_caps->cie.vs_info.mcc.ldac_info.unknown1 != 0) {
        return false;
    }
    return true;
}

static void ldac_set_freq(uint32_t freq, esp_a2d_mcc_t *pref_mcc) {
  bool restart_input = false;
  bool restart_output = false;
  bool config_updated = false;
  a2dp_vendor_ldac_encoder_update(freq, pref_mcc, &restart_input, &restart_output, &config_updated);
}

static void ldac_debug_info(char * buffer, size_t buffer_len) {
  if (a2dP_ldac_encoder_cb.has_ldac_abr_handle) {
    snprintf(buffer, buffer_len, "ABR eqmid %d sampr %d", a2dP_ldac_encoder_cb.last_ldac_abr_eqmid, a2dP_ldac_encoder_cb.ldac_encoder_params.sample_rate);
  } else {
    snprintf(buffer, buffer_len, "CBR eqmid %d sampr %d", a2dP_ldac_encoder_cb.ldac_encoder_params.quality_mode_index, a2dP_ldac_encoder_cb.ldac_encoder_params.sample_rate);
  }
}

struct a2dp_codec a2dp_codec_ldac = {
    .codec_id = A2D_CODEC_LDAC,
    .is_acceptable = ldac_is_acceptable,
    .init = a2dp_vendor_ldac_encoder_init,
    .deinit = a2dp_vendor_ldac_encoder_cleanup,
    .get_period = a2dp_vendor_ldac_get_encoder_interval_ms,
    .tick = a2dp_vendor_ldac_send_frames,
    .set_freq = ldac_set_freq,
    .feeding_reset = a2dp_vendor_ldac_feeding_reset,
    .feeding_flush = a2dp_vendor_ldac_feeding_flush,
    .debug_info = ldac_debug_info,
};