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
#include "osi/allocator.h"
#include "tick.h"
#include "bluetooth/codecs.h"
#include "bluetooth/bt.h"
#include "a2dp_vendor_aptx_constants.h"
#include "stack/avdt_api.h"

#include "aptXbtenc.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

//
// Encoder for aptX Source Codec
//

// offset
// no RTP header for aptX classic
#define A2DP_APTX_OFFSET (AVDT_MEDIA_OFFSET - AVDT_MEDIA_HDR_SIZE)

#define A2DP_APTX_MAX_PCM_BYTES_PER_READ 4096

typedef struct {
  uint64_t sleep_time_ns;
  uint32_t pcm_reads;
  uint32_t pcm_bytes_per_read;
  uint32_t aptx_bytes;
  uint32_t frame_size_counter;
} tAPTX_FRAMING_PARAMS;

typedef struct {
  uint64_t session_start_us;

  size_t media_read_total_expected_packets;
  size_t media_read_total_expected_reads_count;
  size_t media_read_total_expected_read_bytes;

  size_t media_read_total_dropped_packets;
  size_t media_read_total_actual_reads_count;
  size_t media_read_total_actual_read_bytes;
} a2dp_aptx_encoder_stats_t;

typedef struct {
  bool use_SCMS_T;
  esp_a2d_mcc_t peer_params;
  uint16_t peer_mtu;
  bool peer_is_edr;
  esp_a2d_conn_hdl_t peer;
  uint32_t timestamp;  // Timestamp for the A2DP frames

  uint32_t sample_rate;
  tAPTX_FRAMING_PARAMS framing_params;
  void* aptx_encoder_state;
  a2dp_aptx_encoder_stats_t stats;
} tA2DP_APTX_ENCODER_CB;

static tA2DP_APTX_ENCODER_CB a2dp_aptx_encoder_cb;

static void a2dp_vendor_aptx_encoder_update(uint32_t freq, esp_a2d_mcc_t *pref_mcc,
                                            bool* p_restart_input, bool* p_restart_output,
                                            bool* p_config_updated);
static void aptx_init_framing_params(tAPTX_FRAMING_PARAMS* framing_params);
static void aptx_update_framing_params(tAPTX_FRAMING_PARAMS* framing_params);
static size_t aptx_encode_16bit(tAPTX_FRAMING_PARAMS* framing_params, size_t* data_out_index,
                                uint16_t* data16_in, uint8_t* data_out);
static void a2dp_vendor_aptx_feeding_reset(void);

unsigned long bt_aptx_sampr[2] =
{
    SAMPR_48,
    SAMPR_44,
};

void a2dp_vendor_aptx_encoder_init(struct pcm_sink_caps *caps, const struct a2dp_peer_info* peer, esp_a2d_mcc_t *pref_mcc) {
  memset(&a2dp_aptx_encoder_cb, 0, sizeof(a2dp_aptx_encoder_cb));

  int i = 0;
  if (peer->peer_caps->cie.vs_info.mcc.aptx_info.samp_freq & 0x1) {
    bt_aptx_sampr[i++] = SAMPR_48;
  }
  if (peer->peer_caps->cie.vs_info.mcc.aptx_info.samp_freq & 0x2) {
    bt_aptx_sampr[i++] = SAMPR_44;
  }
  caps->samprs = bt_aptx_sampr;
  caps->default_freq = 0;
  caps->num_samprs = i;
  caps->volume_type = PCM_SINK_HWVOL;

  a2dp_aptx_encoder_cb.stats.session_start_us = current_tick;

  a2dp_aptx_encoder_cb.peer_params = *peer->peer_caps;
  a2dp_aptx_encoder_cb.peer_mtu = peer->peer_mtu;
  a2dp_aptx_encoder_cb.peer = peer->conn_hdl;
  a2dp_aptx_encoder_cb.peer_is_edr = peer->is_edr;
  a2dp_aptx_encoder_cb.timestamp = 0;

  /* aptX encoder config */
  a2dp_aptx_encoder_cb.use_SCMS_T = false;

  a2dp_aptx_encoder_cb.aptx_encoder_state = osi_malloc(SizeofAptxbtenc());
  if (a2dp_aptx_encoder_cb.aptx_encoder_state != NULL) {
    aptxbtenc_init(a2dp_aptx_encoder_cb.aptx_encoder_state, 0);
  }

  // NOTE: Ignore the restart_input / restart_output flags - this initization
  // happens when the audio session is (re)started.
  bool restart_input = false;
  bool restart_output = false;
  bool config_updated = false;
  a2dp_vendor_aptx_encoder_update(44100, pref_mcc, &restart_input, &restart_output,
                                  &config_updated);
}

// Update the A2DP aptX encoder.
// |a2dp_codec_config| is the A2DP codec to use for the update.
static void a2dp_vendor_aptx_encoder_update(uint32_t freq, esp_a2d_mcc_t *pref_mcc,
                                            bool* p_restart_input, bool* p_restart_output,
                                            bool* p_config_updated) {
  *p_restart_input = false;
  *p_restart_output = false;
  *p_config_updated = false;

  pref_mcc->losc = A2DP_APTX_CODEC_LEN;
  pref_mcc->media_type = A2D_MEDIA_TYPE_AUDIO;
  pref_mcc->codec_type = ESP_A2D_MCT_NON_A2DP;
  pref_mcc->cie.vs_info.vendor_id = A2DP_APTX_VENDOR_ID;
  pref_mcc->cie.vs_info.codec_id = A2DP_APTX_CODEC_ID_BLUETOOTH;
  pref_mcc->cie.vs_info.mcc.aptx_info.ch_mode = 0x2;
  switch (freq) {
    case 44100:
      pref_mcc->cie.vs_info.mcc.aptx_info.samp_freq = 0x2;
      a2dp_aptx_encoder_cb.sample_rate = 44100;
      break;
    case 48000:
      pref_mcc->cie.vs_info.mcc.aptx_info.samp_freq = 0x1;
      a2dp_aptx_encoder_cb.sample_rate = 48000;
      break;
  }

  a2dp_vendor_aptx_feeding_reset();
}

void a2dp_vendor_aptx_encoder_cleanup(void) {
  osi_free(a2dp_aptx_encoder_cb.aptx_encoder_state);
  memset(&a2dp_aptx_encoder_cb, 0, sizeof(a2dp_aptx_encoder_cb));
}

//
// Initialize the framing parameters, and set those that don't change
// while streaming (e.g., 'sleep_time_ns').
//
static void aptx_init_framing_params(tAPTX_FRAMING_PARAMS* framing_params) {
  framing_params->sleep_time_ns = 0;
  framing_params->pcm_reads = 0;
  framing_params->pcm_bytes_per_read = 0;
  framing_params->aptx_bytes = 0;
  framing_params->frame_size_counter = 0;

  if (a2dp_aptx_encoder_cb.sample_rate == 48000) {
    if (a2dp_aptx_encoder_cb.use_SCMS_T) {
      framing_params->sleep_time_ns = 13000000;
    } else {
      framing_params->sleep_time_ns = 14000000;
    }
  } else {
    // Assume the sample rate is 44100
    if (a2dp_aptx_encoder_cb.use_SCMS_T) {
      framing_params->sleep_time_ns = 14000000;
    } else {
      framing_params->sleep_time_ns = 15000000;
    }
  }
}

//
// Set frame size and transmission interval needed to stream the required
// sample rate using 2-DH5 packets for aptX and 2-DH3 packets for aptX-LL.
// With SCMS-T enabled we need to reserve room for extra headers added later.
// Packets are always sent at equals time intervals but to achieve the
// required sample rate, the frame size needs to change on occasion.
//
// Also need to specify how many of the required PCM samples are read at a
// time:
//     aptx_bytes = pcm_reads * pcm_bytes_per_read / 4
// and
//     number of aptX samples produced = pcm_bytes_per_read / 16
//
static void aptx_update_framing_params(tAPTX_FRAMING_PARAMS* framing_params) {
  if (a2dp_aptx_encoder_cb.sample_rate == 48000) {
    if (a2dp_aptx_encoder_cb.use_SCMS_T) {
      framing_params->aptx_bytes = 624;
      framing_params->pcm_bytes_per_read = 208;
      framing_params->pcm_reads = 12;
    } else {
      framing_params->aptx_bytes = 672;
      framing_params->pcm_bytes_per_read = 224;
      framing_params->pcm_reads = 12;
    }
  } else {
    // Assume the sample rate is 44100
    if (a2dp_aptx_encoder_cb.use_SCMS_T) {
      if (++framing_params->frame_size_counter < 20) {
        framing_params->aptx_bytes = 616;
        framing_params->pcm_bytes_per_read = 224;
        framing_params->pcm_reads = 11;
      } else {
        framing_params->aptx_bytes = 644;
        framing_params->pcm_bytes_per_read = 368;
        framing_params->pcm_reads = 7;
        framing_params->frame_size_counter = 0;
      }
    } else {
      if (++framing_params->frame_size_counter < 8) {
        framing_params->aptx_bytes = 660;
        framing_params->pcm_bytes_per_read = 240;
        framing_params->pcm_reads = 11;
      } else {
        framing_params->aptx_bytes = 672;
        framing_params->pcm_bytes_per_read = 224;
        framing_params->pcm_reads = 12;
        framing_params->frame_size_counter = 0;
      }
    }
  }
}

void a2dp_vendor_aptx_feeding_reset(void) {
  aptx_init_framing_params(&a2dp_aptx_encoder_cb.framing_params);
}

void a2dp_vendor_aptx_feeding_flush(void) {
  aptx_init_framing_params(&a2dp_aptx_encoder_cb.framing_params);
}

uint64_t a2dp_vendor_aptx_get_encoder_interval_ms(void) {
  return a2dp_aptx_encoder_cb.framing_params.sleep_time_ns / (1000 * 1000);
}

int a2dp_vendor_aptx_get_effective_frame_size(void) {
  return a2dp_aptx_encoder_cb.peer_mtu;
}

void a2dp_vendor_aptx_send_frames(void) {
  tAPTX_FRAMING_PARAMS* framing_params = &a2dp_aptx_encoder_cb.framing_params;

  // Prepare the packet to send
  esp_a2d_audio_buff_t* p_buf = esp_a2d_audio_buff_alloc(BT_DEFAULT_BUFFER_SIZE);
  if (p_buf == NULL) {
    return;
  }
  p_buf->number_frame = 0xFF;

  uint8_t* encoded_ptr = p_buf->data;

  aptx_update_framing_params(framing_params);

  //
  // Read the PCM data and encode it
  //
  uint16_t read_buffer16[A2DP_APTX_MAX_PCM_BYTES_PER_READ / sizeof(uint16_t)];
  uint32_t expected_read_bytes = framing_params->pcm_reads * framing_params->pcm_bytes_per_read;
  size_t encoded_ptr_index = 0;
  size_t pcm_bytes_encoded = 0;
  uint32_t bytes_read = 0;

  a2dp_aptx_encoder_cb.stats.media_read_total_expected_packets++;
  a2dp_aptx_encoder_cb.stats.media_read_total_expected_reads_count++;
  a2dp_aptx_encoder_cb.stats.media_read_total_expected_read_bytes += expected_read_bytes;

  bytes_read = bt_read_pcm((uint8_t*)read_buffer16, expected_read_bytes);
  a2dp_aptx_encoder_cb.stats.media_read_total_actual_read_bytes += bytes_read;
  if (bytes_read < expected_read_bytes) {
    a2dp_aptx_encoder_cb.stats.media_read_total_dropped_packets++;
    esp_a2d_audio_buff_free(p_buf);
    return;
  }
  a2dp_aptx_encoder_cb.stats.media_read_total_actual_reads_count++;

  for (uint32_t reads = 0, offset = 0; reads < framing_params->pcm_reads;
       reads++, offset += (framing_params->pcm_bytes_per_read / sizeof(uint16_t))) {
    pcm_bytes_encoded += aptx_encode_16bit(framing_params, &encoded_ptr_index,
                                           read_buffer16 + offset, encoded_ptr);
  }

  // Compute the number of encoded bytes
  const int COMPRESSION_RATIO = 4;
  size_t encoded_bytes = pcm_bytes_encoded / COMPRESSION_RATIO;
  p_buf->data_len += encoded_bytes;

  // Update the RTP timestamp
  p_buf->timestamp = a2dp_aptx_encoder_cb.timestamp;

  const uint8_t BYTES_PER_FRAME = 2;
  uint32_t rtp_timestamp =
          (pcm_bytes_encoded / 2) / BYTES_PER_FRAME;

  // Timestamp will wrap over to 0 if stream continues on long enough
  // (>25H @ 48KHz). The parameters are promoted to 64bit to ensure that
  // no unsigned overflow is triggered as ubsan is always enabled.
  a2dp_aptx_encoder_cb.timestamp =
          ((uint64_t)a2dp_aptx_encoder_cb.timestamp + rtp_timestamp) & UINT32_MAX;

  if (p_buf->data_len > 0) {
    if (esp_a2d_source_audio_data_send(a2dp_aptx_encoder_cb.peer, p_buf) != ESP_OK) {
      esp_a2d_audio_buff_free(p_buf);
      return;
    }
  } else {
    a2dp_aptx_encoder_cb.stats.media_read_total_dropped_packets++;
    esp_a2d_audio_buff_free(p_buf);
  }
}

static size_t aptx_encode_16bit(tAPTX_FRAMING_PARAMS* framing_params, size_t* data_out_index,
                                uint16_t* data16_in, uint8_t* data_out) {
  size_t pcm_bytes_encoded = 0;
  size_t frame = 0;

  for (size_t aptx_samples = 0; aptx_samples < framing_params->pcm_bytes_per_read / 16;
       aptx_samples++) {
    uint32_t pcmL[4];
    uint32_t pcmR[4];
    uint16_t encoded_sample[2];

    for (size_t i = 0, j = frame; i < 4; i++, j++) {
      pcmL[i] = (uint16_t)*(data16_in + (2 * j));
      pcmR[i] = (uint16_t)*(data16_in + ((2 * j) + 1));
    }

    aptxbtenc_encodestereo(a2dp_aptx_encoder_cb.aptx_encoder_state, &pcmL, &pcmR,
                                &encoded_sample);

    data_out[*data_out_index + 0] = (uint8_t)((encoded_sample[0] >> 8) & 0xff);
    data_out[*data_out_index + 1] = (uint8_t)((encoded_sample[0] >> 0) & 0xff);
    data_out[*data_out_index + 2] = (uint8_t)((encoded_sample[1] >> 8) & 0xff);
    data_out[*data_out_index + 3] = (uint8_t)((encoded_sample[1] >> 0) & 0xff);

    frame += 4;
    pcm_bytes_encoded += 16;
    *data_out_index += 4;
  }

  return pcm_bytes_encoded;
}

static bool aptx_is_acceptable(const esp_a2d_mcc_t *peer_caps)
{
    if (peer_caps->losc != A2DP_APTX_CODEC_LEN
        || peer_caps->media_type != A2D_MEDIA_TYPE_AUDIO
        || peer_caps->codec_type != ESP_A2D_MCT_NON_A2DP
        || peer_caps->cie.vs_info.vendor_id != A2DP_APTX_VENDOR_ID
        || peer_caps->cie.vs_info.codec_id != A2DP_APTX_CODEC_ID_BLUETOOTH) {
        return false;
    }
    if (peer_caps->cie.vs_info.mcc.aptx_info.samp_freq & 0x2 == 0
        || peer_caps->cie.vs_info.mcc.aptx_info.ch_mode & 0x2 == 0) {
        return false;
    }
    return true;
}

static void aptx_set_freq(uint32_t freq, esp_a2d_mcc_t *pref_mcc) {
  bool restart_input = false;
  bool restart_output = false;
  bool config_updated = false;
  a2dp_vendor_aptx_encoder_update(freq, pref_mcc, &restart_input, &restart_output, &config_updated);
}

static void aptx_debug_info(char * buffer, size_t buffer_len) {
  snprintf(buffer, buffer_len, "sampr %d", a2dp_aptx_encoder_cb.sample_rate);
}

struct a2dp_codec a2dp_codec_aptx = {
    .codec_id = A2D_CODEC_APTX,
    .is_acceptable = aptx_is_acceptable,
    .init = a2dp_vendor_aptx_encoder_init,
    .deinit = a2dp_vendor_aptx_encoder_cleanup,
    .get_period = a2dp_vendor_aptx_get_encoder_interval_ms,
    .tick = a2dp_vendor_aptx_send_frames,
    .set_freq = aptx_set_freq,
    .feeding_reset = a2dp_vendor_aptx_feeding_reset,
    .feeding_flush = a2dp_vendor_aptx_feeding_flush,
    .debug_info = aptx_debug_info,
};