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
#include "pcm_sink.h"
#include "tick.h"
#include "bluetooth/codecs.h"
#include "bluetooth/bt.h"
#include "a2dp_vendor_aptx_hd_constants.h"
#include "stack/avdt_api.h"

#include "aptXHDbtenc.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

//
// Encoder for aptX-HD Source Codec
//

// offset
#define A2DP_APTX_HD_OFFSET AVDT_MEDIA_OFFSET

#define A2DP_APTX_HD_MAX_PCM_BYTES_PER_READ 4096

typedef struct {
  uint64_t sleep_time_ns;
  uint32_t pcm_reads;
  uint32_t pcm_bytes_per_read;
  uint32_t aptx_hd_bytes;
  uint32_t frame_size_counter;
} tAPTX_HD_FRAMING_PARAMS;

typedef struct {
  uint64_t session_start_us;

  size_t media_read_total_expected_packets;
  size_t media_read_total_expected_reads_count;
  size_t media_read_total_expected_read_bytes;

  size_t media_read_total_dropped_packets;
  size_t media_read_total_actual_reads_count;
  size_t media_read_total_actual_read_bytes;
} a2dp_aptx_hd_encoder_stats_t;

typedef struct {
  bool use_SCMS_T;
  esp_a2d_mcc_t peer_params;
  uint16_t peer_mtu;
  bool peer_is_edr;
  esp_a2d_conn_hdl_t peer;
  uint32_t timestamp;  // Timestamp for the A2DP frames

  uint32_t sample_rate;
  tAPTX_HD_FRAMING_PARAMS framing_params;
  void* aptx_hd_encoder_state;
  a2dp_aptx_hd_encoder_stats_t stats;
} tA2DP_APTX_HD_ENCODER_CB;

static tA2DP_APTX_HD_ENCODER_CB a2dp_aptx_hd_encoder_cb;

static void a2dp_vendor_aptx_hd_encoder_update(uint32_t freq, esp_a2d_mcc_t *pref_mcc,
                                               bool* p_restart_input, bool* p_restart_output,
                                               bool* p_config_updated);
static void aptx_hd_init_framing_params(tAPTX_HD_FRAMING_PARAMS* framing_params);
static void aptx_hd_update_framing_params(tAPTX_HD_FRAMING_PARAMS* framing_params);
static size_t aptx_hd_encode_24bit(tAPTX_HD_FRAMING_PARAMS* framing_params, size_t* data_out_index,
                                   uint8_t* data_in, uint8_t* data_out);
static void a2dp_vendor_aptx_hd_feeding_reset(void);

unsigned long bt_aptx_hd_sampr[2] =
{
    SAMPR_48,
    SAMPR_44,
};

void a2dp_vendor_aptx_hd_encoder_init(struct pcm_sink_caps *caps, const struct a2dp_peer_info* peer, esp_a2d_mcc_t *pref_mcc) {
  memset(&a2dp_aptx_hd_encoder_cb, 0, sizeof(a2dp_aptx_hd_encoder_cb));

  int i = 0;
  if (peer->peer_caps->cie.vs_info.mcc.aptx_hd_info.samp_freq & 0x1) {
    bt_aptx_hd_sampr[i++] = SAMPR_48;
  }
  if (peer->peer_caps->cie.vs_info.mcc.aptx_hd_info.samp_freq & 0x2) {
    bt_aptx_hd_sampr[i++] = SAMPR_44;
  }
  caps->samprs = bt_aptx_hd_sampr;
  caps->default_freq = 0;
  caps->num_samprs = i;
  caps->volume_type = PCM_SINK_HWVOL;

  a2dp_aptx_hd_encoder_cb.stats.session_start_us = current_tick;

  a2dp_aptx_hd_encoder_cb.peer_params = *peer->peer_caps;
  a2dp_aptx_hd_encoder_cb.peer_mtu = peer->peer_mtu;
  a2dp_aptx_hd_encoder_cb.peer = peer->conn_hdl;
  a2dp_aptx_hd_encoder_cb.peer_is_edr = peer->is_edr;
  a2dp_aptx_hd_encoder_cb.timestamp = 0;

  /* aptX-HD encoder config */
  a2dp_aptx_hd_encoder_cb.use_SCMS_T = false;

  a2dp_aptx_hd_encoder_cb.aptx_hd_encoder_state = osi_malloc(SizeofAptxhdbtenc());
  if (a2dp_aptx_hd_encoder_cb.aptx_hd_encoder_state != NULL) {
    aptxhdbtenc_init(a2dp_aptx_hd_encoder_cb.aptx_hd_encoder_state, 0);
  }

  // NOTE: Ignore the restart_input / restart_output flags - this initization
  // happens when the audio session is (re)started.
  bool restart_input = false;
  bool restart_output = false;
  bool config_updated = false;
  a2dp_vendor_aptx_hd_encoder_update(44100, pref_mcc, &restart_input, &restart_output,
                                     &config_updated);
}

// Update the A2DP aptX-HD encoder.
// |a2dp_codec_config| is the A2DP codec to use for the update.
static void a2dp_vendor_aptx_hd_encoder_update(uint32_t freq, esp_a2d_mcc_t *pref_mcc,
                                               bool* p_restart_input, bool* p_restart_output,
                                               bool* p_config_updated) {
  *p_restart_input = false;
  *p_restart_output = false;
  *p_config_updated = false;

  pref_mcc->losc = A2DP_APTX_HD_CODEC_LEN;
  pref_mcc->media_type = A2D_MEDIA_TYPE_AUDIO;
  pref_mcc->codec_type = ESP_A2D_MCT_NON_A2DP;
  pref_mcc->cie.vs_info.vendor_id = A2DP_APTX_HD_VENDOR_ID;
  pref_mcc->cie.vs_info.codec_id = A2DP_APTX_HD_CODEC_ID_BLUETOOTH;
  pref_mcc->cie.vs_info.mcc.aptx_hd_info.ch_mode = 0x2;
  switch (freq) {
    case 44100:
      pref_mcc->cie.vs_info.mcc.aptx_hd_info.samp_freq = 0x2;
      a2dp_aptx_hd_encoder_cb.sample_rate = 44100;
      break;
    case 48000:
      pref_mcc->cie.vs_info.mcc.aptx_hd_info.samp_freq = 0x1;
      a2dp_aptx_hd_encoder_cb.sample_rate = 48000;
      break;
  }
  pref_mcc->cie.vs_info.mcc.aptx_hd_info.reserved0 = 0;
  pref_mcc->cie.vs_info.mcc.aptx_hd_info.reserved1 = 0;
  pref_mcc->cie.vs_info.mcc.aptx_hd_info.reserved2 = 0;
  pref_mcc->cie.vs_info.mcc.aptx_hd_info.reserved3 = 0;


  a2dp_vendor_aptx_hd_feeding_reset();
}

void a2dp_vendor_aptx_hd_encoder_cleanup(void) {
  osi_free(a2dp_aptx_hd_encoder_cb.aptx_hd_encoder_state);
  memset(&a2dp_aptx_hd_encoder_cb, 0, sizeof(a2dp_aptx_hd_encoder_cb));
}

//
// Initialize the framing parameters, and set those that don't change
// while streaming (e.g., 'sleep_time_ns').
//
static void aptx_hd_init_framing_params(tAPTX_HD_FRAMING_PARAMS* framing_params) {
  framing_params->sleep_time_ns = 0;
  framing_params->pcm_reads = 0;
  framing_params->pcm_bytes_per_read = 0;
  framing_params->aptx_hd_bytes = 0;
  framing_params->frame_size_counter = 0;

  framing_params->sleep_time_ns = 9000000;
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
static void aptx_hd_update_framing_params(tAPTX_HD_FRAMING_PARAMS* framing_params) {
  if (a2dp_aptx_hd_encoder_cb.sample_rate == 48000) {
    framing_params->aptx_hd_bytes = 648;
    framing_params->pcm_bytes_per_read = 16;
    framing_params->pcm_reads = 108;
  } else {
    // Assume the sample rate is 44100

    //
    // Total of 80 iterations:
    // - Iteration 80: packet size 648, with 108 reads of 24 PCM bytes
    // - Iterations 20, 40, 60: packet size 612, with 102 reads of 24 PCM bytes
    // - All other iterations: packet size 594, with 99 reads of 24 PCM bytes
    //
    if (framing_params->frame_size_counter + 1 == 80) {
      framing_params->aptx_hd_bytes = 648;
      framing_params->pcm_bytes_per_read = 16;
      framing_params->pcm_reads = 108;
    } else if (((framing_params->frame_size_counter + 1) % 20) == 0) {
      framing_params->aptx_hd_bytes = 612;
      framing_params->pcm_bytes_per_read = 16;
      framing_params->pcm_reads = 102;
    } else {
      framing_params->aptx_hd_bytes = 594;
      framing_params->pcm_bytes_per_read = 16;
      framing_params->pcm_reads = 99;
    }
    framing_params->frame_size_counter++;
    if (framing_params->frame_size_counter == 80) {
      framing_params->frame_size_counter = 0;
    }
  }
}

void a2dp_vendor_aptx_hd_feeding_reset(void) {
  aptx_hd_init_framing_params(&a2dp_aptx_hd_encoder_cb.framing_params);
}

void a2dp_vendor_aptx_hd_feeding_flush(void) {
  aptx_hd_init_framing_params(&a2dp_aptx_hd_encoder_cb.framing_params);
}

uint64_t a2dp_vendor_aptx_hd_get_encoder_interval_ms(void) {
  return a2dp_aptx_hd_encoder_cb.framing_params.sleep_time_ns / (1000 * 1000);
}

int a2dp_vendor_aptx_hd_get_effective_frame_size(void) {
  return a2dp_aptx_hd_encoder_cb.peer_mtu;
}

void a2dp_vendor_aptx_hd_send_frames(void) {
  tAPTX_HD_FRAMING_PARAMS* framing_params = &a2dp_aptx_hd_encoder_cb.framing_params;

  // Prepare the packet to send
  esp_a2d_audio_buff_t* p_buf = esp_a2d_audio_buff_alloc(BT_DEFAULT_BUFFER_SIZE);
  if (p_buf == NULL) {
    return;
  }
  p_buf->number_frame = 0;

  uint8_t* encoded_ptr = p_buf->data;

  aptx_hd_update_framing_params(framing_params);

  //
  // Read the PCM data and encode it
  //
  uint8_t read_buffer[A2DP_APTX_HD_MAX_PCM_BYTES_PER_READ];
  uint32_t expected_read_bytes = framing_params->pcm_reads * framing_params->pcm_bytes_per_read;
  size_t encoded_ptr_index = 0;
  size_t pcm_bytes_encoded = 0;
  uint32_t bytes_read = 0;

  a2dp_aptx_hd_encoder_cb.stats.media_read_total_expected_packets++;
  a2dp_aptx_hd_encoder_cb.stats.media_read_total_expected_reads_count++;
  a2dp_aptx_hd_encoder_cb.stats.media_read_total_expected_read_bytes += expected_read_bytes;

  bytes_read = bt_read_pcm(read_buffer, expected_read_bytes);
  a2dp_aptx_hd_encoder_cb.stats.media_read_total_actual_read_bytes += bytes_read;
  if (bytes_read < expected_read_bytes) {
    a2dp_aptx_hd_encoder_cb.stats.media_read_total_dropped_packets++;
    esp_a2d_audio_buff_free(p_buf);
    return;
  }
  a2dp_aptx_hd_encoder_cb.stats.media_read_total_actual_reads_count++;

  for (uint32_t reads = 0, offset = 0; reads < framing_params->pcm_reads;
       reads++, offset += framing_params->pcm_bytes_per_read) {
    pcm_bytes_encoded += aptx_hd_encode_24bit(framing_params, &encoded_ptr_index,
                                              read_buffer + offset, encoded_ptr);
  }

  // Compute the number of encoded bytes
  const int COMPRESSION_RATIO = 4;
  size_t encoded_bytes = pcm_bytes_encoded / COMPRESSION_RATIO;
  p_buf->data_len += encoded_bytes;

  // Update the RTP timestamp
  *((uint32_t*)(p_buf + 1)) = a2dp_aptx_hd_encoder_cb.timestamp;
  const uint8_t BYTES_PER_FRAME = 3;
  uint32_t rtp_timestamp =
          (pcm_bytes_encoded / 2) /
          BYTES_PER_FRAME;

  // Timestamp will wrap over to 0 if stream continues on long enough
  // (>25H @ 48KHz). The parameters are promoted to 64bit to ensure that
  // no unsigned overflow is triggered as ubsan is always enabled.
  a2dp_aptx_hd_encoder_cb.timestamp =
          ((uint64_t)a2dp_aptx_hd_encoder_cb.timestamp + rtp_timestamp) & UINT32_MAX;

  if (p_buf->data_len > 0) {
    if (esp_a2d_source_audio_data_send(a2dp_aptx_hd_encoder_cb.peer, p_buf) != ESP_OK) {
      esp_a2d_audio_buff_free(p_buf);
      return;
    }
  } else {
    a2dp_aptx_hd_encoder_cb.stats.media_read_total_dropped_packets++;
    esp_a2d_audio_buff_free(p_buf);
  }
}

static size_t aptx_hd_encode_24bit(tAPTX_HD_FRAMING_PARAMS* framing_params, size_t* data_out_index,
                                   uint8_t* data_in, uint8_t* data_out) {
  size_t pcm_bytes_encoded = 0;
  const uint8_t* p = (const uint8_t*)(data_in);

  for (size_t aptx_hd_samples = 0; aptx_hd_samples < framing_params->pcm_bytes_per_read / 16;
       aptx_hd_samples++) {
    uint32_t pcmL[4];
    uint32_t pcmR[4];
    uint32_t encoded_sample[2];

    // Expand from AUDIO_FORMAT_PCM_16_BIT_PACKED data (2 bytes per sample)
    // into AUDIO_FORMAT_PCM_8_24_BIT (4 bytes per sample).
    for (size_t i = 0; i < 4; i++) {
      pcmL[i] = ((p[0] << 8) | ((((int8_t)p[1]) << 16) & 0xFFFF0000));
      p += 2;
      pcmR[i] = ((p[0] << 8) | ((((int8_t)p[1]) << 16) & 0xFFFF0000));
      p += 2;
    }

    aptxhdbtenc_encodestereo(a2dp_aptx_hd_encoder_cb.aptx_hd_encoder_state, &pcmL, &pcmR,
                                   &encoded_sample);

    uint8_t* encoded_ptr = (uint8_t*)&encoded_sample[0];
    data_out[*data_out_index + 0] = *(encoded_ptr + 2);
    data_out[*data_out_index + 1] = *(encoded_ptr + 1);
    data_out[*data_out_index + 2] = *(encoded_ptr + 0);
    data_out[*data_out_index + 3] = *(encoded_ptr + 6);
    data_out[*data_out_index + 4] = *(encoded_ptr + 5);
    data_out[*data_out_index + 5] = *(encoded_ptr + 4);

    pcm_bytes_encoded += 24;
    *data_out_index += 6;
  }

  return pcm_bytes_encoded;
}

static bool aptx_hd_is_acceptable(const esp_a2d_mcc_t *peer_caps)
{
    if (peer_caps->losc != A2DP_APTX_HD_CODEC_LEN
        || peer_caps->media_type != A2D_MEDIA_TYPE_AUDIO
        || peer_caps->codec_type != ESP_A2D_MCT_NON_A2DP
        || peer_caps->cie.vs_info.vendor_id != A2DP_APTX_HD_VENDOR_ID
        || peer_caps->cie.vs_info.codec_id != A2DP_APTX_HD_CODEC_ID_BLUETOOTH) {
        return false;
    }
    if (peer_caps->cie.vs_info.mcc.aptx_hd_info.samp_freq & 0x2 == 0
        || peer_caps->cie.vs_info.mcc.aptx_hd_info.ch_mode & 0x2 == 0) {
        return false;
    }
    return true;
}

static void aptx_hd_set_freq(uint32_t freq, esp_a2d_mcc_t *pref_mcc) {
  bool restart_input = false;
  bool restart_output = false;
  bool config_updated = false;
  a2dp_vendor_aptx_hd_encoder_update(freq, pref_mcc, &restart_input, &restart_output, &config_updated);
}

static void aptx_hd_debug_info(char * buffer, size_t buffer_len) {
  snprintf(buffer, buffer_len, "sampr %d", a2dp_aptx_hd_encoder_cb.sample_rate);
}

struct a2dp_codec a2dp_codec_aptx_hd = {
    .codec_id = A2D_CODEC_APTX_HD,
    .is_acceptable = aptx_hd_is_acceptable,
    .init = a2dp_vendor_aptx_hd_encoder_init,
    .deinit = a2dp_vendor_aptx_hd_encoder_cleanup,
    .get_period = a2dp_vendor_aptx_hd_get_encoder_interval_ms,
    .tick = a2dp_vendor_aptx_hd_send_frames,
    .set_freq = aptx_hd_set_freq,
    .feeding_reset = a2dp_vendor_aptx_hd_feeding_reset,
    .feeding_flush = a2dp_vendor_aptx_hd_feeding_flush,
    .debug_info = aptx_hd_debug_info,
};