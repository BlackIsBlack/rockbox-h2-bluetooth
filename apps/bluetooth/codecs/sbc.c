/******************************************************************************
 *
 *  Copyright 2016 The Android Open Source Project
 *  Copyright 2009-2012 Broadcom Corporation
 *  Copyright 2026 Skye Green
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include "esp_a2dp_api.h"
#include "tick.h"
#include "bluetooth/codecs.h"
#include "bluetooth/bt.h"
#include "stack/a2d_sbc.h"
#include "stack/avdt_api.h"

#include "sbc_encoder.h"

#include <limits.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Buffer pool */
#define A2DP_SBC_BUFFER_SIZE BT_DEFAULT_BUFFER_SIZE

// A2DP SBC encoder interval in milliseconds.
#define A2DP_SBC_ENCODER_INTERVAL_MS 20

/* High quality quality setting @ 44.1 khz */
#define A2DP_SBC_DEFAULT_BITRATE 328

#define A2DP_SBC_NON_EDR_MAX_RATE 229

#define A2DP_SBC_MAX_PCM_ITER_NUM_PER_TICK 3

#define A2DP_SBC_MAX_HQ_FRAME_SIZE_44_1 119
#define A2DP_SBC_MAX_HQ_FRAME_SIZE_48 115

/* Define the bitrate step when trying to match bitpool value */
#define A2DP_SBC_BITRATE_STEP 5

/* Readability constants */
#define A2DP_SBC_FRAME_HEADER_SIZE_BYTES 4  // A2DP Spec v1.3, 12.4, Table 12.12
#define A2DP_SBC_SCALE_FACTOR_BITS 4        // A2DP Spec v1.3, 12.4, Table 12.13

/* offset */
#define A2DP_HDR_SIZE 1
#define A2DP_SBC_OFFSET (AVDT_MEDIA_OFFSET + A2DP_HDR_SIZE)

#define MAX_PCM_FRAME_NUM_PER_TICK 14

typedef struct {
  uint32_t aa_frame_counter;
  int32_t aa_feed_counter;
  int32_t aa_feed_residue;
  float counter;
  uint32_t bytes_per_tick; /* pcm bytes read each media task tick */
  uint64_t last_frame_us;
} tA2DP_SBC_FEEDING_STATE;

typedef struct {
  uint64_t session_start_us;

  size_t media_read_total_expected_packets;
  size_t media_read_total_expected_reads_count;
  size_t media_read_total_expected_read_bytes;

  size_t media_read_total_dropped_packets;
  size_t media_read_total_actual_reads_count;
  size_t media_read_total_actual_read_bytes;

  size_t media_read_total_expected_frames;
  size_t media_read_total_dropped_frames;
} a2dp_sbc_encoder_stats_t;

typedef struct {
  uint16_t TxAaMtuSize;
  uint8_t tx_sbc_frames;
  esp_a2d_mcc_t peer_params;
  uint16_t peer_mtu;
  bool peer_is_edr;
  esp_a2d_conn_hdl_t peer;
  uint32_t timestamp; /* Timestamp for the A2DP frames */
  SBC_ENC_PARAMS sbc_encoder_params;
  tA2DP_SBC_FEEDING_STATE feeding_state;
  int16_t pcmBuffer[SBC_MAX_PCM_BUFFER_SIZE];

  a2dp_sbc_encoder_stats_t stats;
} tA2DP_SBC_ENCODER_CB;

static tA2DP_SBC_ENCODER_CB a2dp_sbc_encoder_cb;

static void a2dp_sbc_encoder_update(int16_t freq, esp_a2d_mcc_t *pref_mcc, bool* p_restart_input,
                                    bool* p_restart_output, bool* p_config_updated);
static bool a2dp_sbc_read_feeding(uint32_t* bytes);
static void a2dp_sbc_encode_frames(uint8_t nb_frame);
static void a2dp_sbc_get_num_frame_iteration(uint8_t* num_of_iterations, uint8_t* num_of_frames,
                                             uint64_t timestamp_us);
static uint16_t adjust_effective_mtu(void);
static uint8_t calculate_max_frames_per_packet(void);
static uint16_t a2dp_sbc_source_rate(void);
static uint32_t a2dp_sbc_frame_length(void);
static void a2dp_sbc_feeding_reset(void);

unsigned long bt_sbc_sampr[4] =
{
    SAMPR_48,
    SAMPR_44,
    SAMPR_32,
    SAMPR_16,
};

void a2dp_sbc_encoder_init(struct pcm_sink_caps *caps, const struct a2dp_peer_info* peer, esp_a2d_mcc_t *pref_mcc) {
  memset(&a2dp_sbc_encoder_cb, 0, sizeof(a2dp_sbc_encoder_cb));

  int i = 0;
  if (peer->peer_caps->cie.sbc_info.samp_freq | ESP_A2D_SBC_CIE_SF_48K) {
    bt_sbc_sampr[i++] = SAMPR_48;
  }
  if (peer->peer_caps->cie.sbc_info.samp_freq | ESP_A2D_SBC_CIE_SF_44K) {
    bt_sbc_sampr[i++] = SAMPR_44;
  }
  if (peer->peer_caps->cie.sbc_info.samp_freq | ESP_A2D_SBC_CIE_SF_32K) {
    bt_sbc_sampr[i++] = SAMPR_32;
  }
  if (peer->peer_caps->cie.sbc_info.samp_freq | ESP_A2D_SBC_CIE_SF_16K) {
    bt_sbc_sampr[i++] = SAMPR_16;
  }
  caps->samprs = bt_sbc_sampr;
  caps->default_freq = 0;
  caps->num_samprs = i;
  caps->volume_type = PCM_SINK_HWVOL;

  a2dp_sbc_encoder_cb.stats.session_start_us = current_tick;
  a2dp_sbc_encoder_cb.timestamp = 0;
  a2dp_sbc_encoder_cb.peer_params = *peer->peer_caps;
  a2dp_sbc_encoder_cb.peer_mtu = peer->peer_mtu;
  a2dp_sbc_encoder_cb.peer = peer->conn_hdl;
  a2dp_sbc_encoder_cb.peer_is_edr = peer->is_edr;

  // NOTE: Ignore the restart_input / restart_output flags - this initization
  // happens when the audio session is (re)started.
  bool restart_input = false;
  bool restart_output = false;
  bool config_updated = false;
  a2dp_sbc_encoder_update(SBC_sf48000, pref_mcc, &restart_input, &restart_output, &config_updated);
}

// Update the A2DP SBC encoder.
// |a2dp_codec_config| is the A2DP codec to use for the update.
static void a2dp_sbc_encoder_update(int16_t freq, esp_a2d_mcc_t *pref_mcc, bool* p_restart_input,
                                    bool* p_restart_output, bool* p_config_updated) {
  SBC_ENC_PARAMS* p_encoder_params = &a2dp_sbc_encoder_cb.sbc_encoder_params;
  uint16_t s16SamplingFreq;
  int16_t s16BitPool = 0;
  int16_t s16BitRate;
  int16_t s16FrameLen;
  uint8_t protect = 0;
  int min_bitpool;
  int max_bitpool;

  *p_restart_input = false;
  *p_restart_output = false;
  *p_config_updated = false;
  min_bitpool = a2dp_sbc_encoder_cb.peer_params.cie.sbc_info.min_bitpool;
  max_bitpool = a2dp_sbc_encoder_cb.peer_params.cie.sbc_info.max_bitpool;

  int spec_bitpool = freq == SBC_sf44100 ? 53 : 51;

  if (max_bitpool < spec_bitpool) {
    // HACK: Samsung Galaxy Buds2 Pro seems to report a ridiculously low max bitpool of 37.
    // Assuming the sink is otherwise compliant with A2DP v1.4.1, which specifies a minimum max bitpool:
    // > The decoder of the SNK shall support 2 as the Minimum Bitpool Value and High Quality Bitpool value in
    // > Table 4.7 as the Maximum Bitpool Value at a minimum.
    // Clamp the minimum to the one specified in Table 4.7.
    max_bitpool = spec_bitpool;
  }

  pref_mcc->losc = A2D_SBC_INFO_LEN;
  pref_mcc->media_type = A2D_MEDIA_TYPE_AUDIO;
  pref_mcc->codec_type = ESP_A2D_MCT_SBC;
  pref_mcc->cie.sbc_info.ch_mode      = ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
  pref_mcc->cie.sbc_info.block_len    = ESP_A2D_SBC_CIE_BLOCK_LEN_16;
  pref_mcc->cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
  pref_mcc->cie.sbc_info.alloc_mthd   = ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
  pref_mcc->cie.sbc_info.min_bitpool  = min_bitpool;
  pref_mcc->cie.sbc_info.max_bitpool  = max_bitpool;

  // The feeding parameters
  a2dp_sbc_feeding_reset();

  // The codec parameters
  p_encoder_params->s16ChannelMode = SBC_JOINT_STEREO;
  p_encoder_params->s16NumOfSubBands = SUB_BANDS_8;
  p_encoder_params->s16NumOfBlocks = SBC_BLOCK_3;
  p_encoder_params->s16AllocationMethod = SBC_LOUDNESS;
  p_encoder_params->s16SamplingFreq = freq;
  p_encoder_params->s16NumOfChannels = 2;

  if (p_encoder_params->s16SamplingFreq == SBC_sf16000) {
    s16SamplingFreq = 16000;
    pref_mcc->cie.sbc_info.samp_freq    = ESP_A2D_SBC_CIE_SF_16K;
  } else if (p_encoder_params->s16SamplingFreq == SBC_sf32000) {
    s16SamplingFreq = 32000;
    pref_mcc->cie.sbc_info.samp_freq    = ESP_A2D_SBC_CIE_SF_32K;
  } else if (p_encoder_params->s16SamplingFreq == SBC_sf44100) {
    s16SamplingFreq = 44100;
    pref_mcc->cie.sbc_info.samp_freq    = ESP_A2D_SBC_CIE_SF_44K;
  } else {
    s16SamplingFreq = 48000;
    pref_mcc->cie.sbc_info.samp_freq    = ESP_A2D_SBC_CIE_SF_48K;
  }

  // Set the initial target bit rate
  p_encoder_params->u16BitRate = a2dp_sbc_source_rate();

  a2dp_sbc_encoder_cb.TxAaMtuSize = adjust_effective_mtu();

  do {
    if ((p_encoder_params->s16ChannelMode == SBC_JOINT_STEREO) ||
        (p_encoder_params->s16ChannelMode == SBC_STEREO)) {
      s16BitPool = (int16_t)((p_encoder_params->u16BitRate * p_encoder_params->s16NumOfSubBands *
                              1000 / s16SamplingFreq) -
                             ((32 +
                               (4 * p_encoder_params->s16NumOfSubBands *
                                p_encoder_params->s16NumOfChannels) +
                               ((p_encoder_params->s16ChannelMode - 2) *
                                p_encoder_params->s16NumOfSubBands)) /
                              p_encoder_params->s16NumOfBlocks));

      s16FrameLen =
              4 +
              (4 * p_encoder_params->s16NumOfSubBands * p_encoder_params->s16NumOfChannels) / 8 +
              (((p_encoder_params->s16ChannelMode - 2) * p_encoder_params->s16NumOfSubBands) +
               (p_encoder_params->s16NumOfBlocks * s16BitPool)) /
                      8;

      s16BitRate = (8 * s16FrameLen * s16SamplingFreq) /
                   (p_encoder_params->s16NumOfSubBands * p_encoder_params->s16NumOfBlocks * 1000);

      if (s16BitRate > p_encoder_params->u16BitRate) {
        s16BitPool--;
      }

      if (p_encoder_params->s16NumOfSubBands == 8) {
        s16BitPool = (s16BitPool > 255) ? 255 : s16BitPool;
      } else {
        s16BitPool = (s16BitPool > 128) ? 128 : s16BitPool;
      }
    } else {
      s16BitPool = (int16_t)(((p_encoder_params->s16NumOfSubBands * p_encoder_params->u16BitRate *
                               1000) /
                              (s16SamplingFreq * p_encoder_params->s16NumOfChannels)) -
                             (((32 / p_encoder_params->s16NumOfChannels) +
                               (4 * p_encoder_params->s16NumOfSubBands)) /
                              p_encoder_params->s16NumOfBlocks));

      p_encoder_params->s16BitPool = (s16BitPool > (16 * p_encoder_params->s16NumOfSubBands))
                                             ? (16 * p_encoder_params->s16NumOfSubBands)
                                             : s16BitPool;
    }

    if (s16BitPool < 0) {
      s16BitPool = 0;
    }

    if (s16BitPool > max_bitpool) {
      /* Decrease bitrate */
      p_encoder_params->u16BitRate -= A2DP_SBC_BITRATE_STEP;
      /* Record that we have decreased the bitrate */
      protect |= 1;
    } else if (s16BitPool < min_bitpool) {

      /* Increase bitrate */
      uint16_t previous_u16BitRate = p_encoder_params->u16BitRate;
      p_encoder_params->u16BitRate += A2DP_SBC_BITRATE_STEP;
      /* Record that we have increased the bitrate */
      protect |= 2;
      /* Check over-flow */
      if (p_encoder_params->u16BitRate < previous_u16BitRate) {
        protect |= 3;
      }
    } else {
      break;
    }
    /* In case we have already increased and decreased the bitrate, just stop */
    if (protect == 3) {
      break;
    }
  } while (true);

  /* Finally update the bitpool in the encoder structure */
  p_encoder_params->s16BitPool = s16BitPool;

  /* Reset the SBC encoder */
  SBC_Encoder_Init(&a2dp_sbc_encoder_cb.sbc_encoder_params);
  a2dp_sbc_encoder_cb.tx_sbc_frames = calculate_max_frames_per_packet();
}

void a2dp_sbc_encoder_cleanup(void) {
  memset(&a2dp_sbc_encoder_cb, 0, sizeof(a2dp_sbc_encoder_cb));
}

void a2dp_sbc_feeding_reset(void) {
  /* By default, just clear the entire state */
  memset(&a2dp_sbc_encoder_cb.feeding_state, 0, sizeof(a2dp_sbc_encoder_cb.feeding_state));

  SBC_ENC_PARAMS* p_encoder_params = &a2dp_sbc_encoder_cb.sbc_encoder_params;
  uint16_t s16SamplingFreq;

  if (p_encoder_params->s16SamplingFreq == SBC_sf16000) {
    s16SamplingFreq = 16000;
  } else if (p_encoder_params->s16SamplingFreq == SBC_sf32000) {
    s16SamplingFreq = 32000;
  } else if (p_encoder_params->s16SamplingFreq == SBC_sf44100) {
    s16SamplingFreq = 44100;
  } else {
    s16SamplingFreq = 48000;
  }

  a2dp_sbc_encoder_cb.feeding_state.bytes_per_tick =
          (s16SamplingFreq *
           16 / 8 *
           2 * A2DP_SBC_ENCODER_INTERVAL_MS) /
          1000;

}

void a2dp_sbc_feeding_flush(void) {
  a2dp_sbc_encoder_cb.feeding_state.counter = 0.0f;
  a2dp_sbc_encoder_cb.feeding_state.aa_feed_residue = 0;
}

uint64_t a2dp_sbc_get_encoder_interval_ms(void) { return A2DP_SBC_ENCODER_INTERVAL_MS; }

void a2dp_sbc_send_frames(void) {
  uint64_t timestamp_us = ((uint64_t) current_tick) * (1000000 / HZ);
  uint8_t nb_frame = 0;
  uint8_t nb_iterations = 0;

  a2dp_sbc_get_num_frame_iteration(&nb_iterations, &nb_frame, timestamp_us);
  if (nb_frame == 0) {
    return;
  }

  for (uint8_t counter = 0; counter < nb_iterations; counter++) {
    // Transcode frame and enqueue
    a2dp_sbc_encode_frames(nb_frame);
  }
}

// Obtains the number of frames to send and number of iterations
// to be used. |num_of_iterations| and |num_of_frames| parameters
// are used as output param for returning the respective values.
static void a2dp_sbc_get_num_frame_iteration(uint8_t* num_of_iterations, uint8_t* num_of_frames,
                                             uint64_t timestamp_us) {
  uint8_t nof = 0;
  uint8_t noi = 1;

  uint32_t projected_nof = 0;
  uint32_t pcm_bytes_per_frame = a2dp_sbc_encoder_cb.sbc_encoder_params.s16NumOfSubBands *
                                 a2dp_sbc_encoder_cb.sbc_encoder_params.s16NumOfBlocks *
                                 2 *
                                 16 / 8;

  uint32_t us_this_tick = A2DP_SBC_ENCODER_INTERVAL_MS * 1000;
  uint64_t now_us = timestamp_us;
  if (a2dp_sbc_encoder_cb.feeding_state.last_frame_us != 0) {
    us_this_tick = (now_us - a2dp_sbc_encoder_cb.feeding_state.last_frame_us);
  }
  a2dp_sbc_encoder_cb.feeding_state.last_frame_us = now_us;

  a2dp_sbc_encoder_cb.feeding_state.counter +=
          (float)a2dp_sbc_encoder_cb.feeding_state.bytes_per_tick * (float)us_this_tick /
          (A2DP_SBC_ENCODER_INTERVAL_MS * 1000);

  /* Calculate the number of frames pending for this media tick */
  projected_nof = a2dp_sbc_encoder_cb.feeding_state.counter / (float)pcm_bytes_per_frame;
  // Update the stats
  a2dp_sbc_encoder_cb.stats.media_read_total_expected_frames += projected_nof;

  if (projected_nof > MAX_PCM_FRAME_NUM_PER_TICK) {
    // Update the stats
    size_t delta = projected_nof - MAX_PCM_FRAME_NUM_PER_TICK;
    a2dp_sbc_encoder_cb.stats.media_read_total_dropped_frames += delta;

    projected_nof = MAX_PCM_FRAME_NUM_PER_TICK;
  }

  if (a2dp_sbc_encoder_cb.peer_is_edr) {
    if (!a2dp_sbc_encoder_cb.tx_sbc_frames) {
      a2dp_sbc_encoder_cb.tx_sbc_frames = calculate_max_frames_per_packet();
    }

    nof = a2dp_sbc_encoder_cb.tx_sbc_frames;
    if (!nof) {
      nof = projected_nof;
      noi = 1;
    } else {
      if (nof < projected_nof) {
        noi = projected_nof / nof;  // number of iterations would vary
        if (noi > A2DP_SBC_MAX_PCM_ITER_NUM_PER_TICK) {
          noi = A2DP_SBC_MAX_PCM_ITER_NUM_PER_TICK;
          a2dp_sbc_encoder_cb.feeding_state.counter = noi * nof * (float)pcm_bytes_per_frame;
        }
        projected_nof = nof;
      } else {
        noi = 1;  // number of iterations is 1
        nof = projected_nof;
      }
    }
  } else {
    // For BR cases nof will be same as the value retrieved at projected_nof
    if (projected_nof > MAX_PCM_FRAME_NUM_PER_TICK) {
      // Update the stats
      size_t delta = projected_nof - MAX_PCM_FRAME_NUM_PER_TICK;
      a2dp_sbc_encoder_cb.stats.media_read_total_dropped_frames += delta;

      projected_nof = MAX_PCM_FRAME_NUM_PER_TICK;
      a2dp_sbc_encoder_cb.feeding_state.counter =
              (float)noi * (float)projected_nof * (float)pcm_bytes_per_frame;
    }
    nof = projected_nof;
  }
  a2dp_sbc_encoder_cb.feeding_state.counter -= noi * nof * (float)pcm_bytes_per_frame;

  *num_of_frames = nof;
  *num_of_iterations = noi;
}

static void a2dp_sbc_encode_frames(uint8_t nb_frame) {
  SBC_ENC_PARAMS* p_encoder_params = &a2dp_sbc_encoder_cb.sbc_encoder_params;
  uint8_t remain_nb_frame = nb_frame;
  uint16_t blocm_x_subband = p_encoder_params->s16NumOfSubBands * p_encoder_params->s16NumOfBlocks;

  uint8_t last_frame_len = 0;

  while (nb_frame) {
    esp_a2d_audio_buff_t* p_buf = esp_a2d_audio_buff_alloc(A2DP_SBC_BUFFER_SIZE);
    if (p_buf == NULL) {
      return;
    }
    uint32_t bytes_read = 0;

    a2dp_sbc_encoder_cb.stats.media_read_total_expected_packets++;

    do {
      /* Fill allocated buffer with 0 */
      memset(a2dp_sbc_encoder_cb.pcmBuffer, 0,
             blocm_x_subband * p_encoder_params->s16NumOfChannels);
      //
      // Read the PCM data and encode it. If necessary, upsample the data.
      //
      uint32_t num_bytes = 0;
      if (a2dp_sbc_read_feeding(&num_bytes)) {
        uint8_t* output = p_buf->data + p_buf->data_len;
        int16_t* input = a2dp_sbc_encoder_cb.pcmBuffer;
        uint16_t output_len = SBC_Encode(p_encoder_params, input, output);
        last_frame_len = output_len;

        /* Update SBC frame length */
        p_buf->data_len += output_len;
        nb_frame--;
        p_buf->number_frame++;

        bytes_read += num_bytes;
      } else {
        a2dp_sbc_encoder_cb.feeding_state.counter +=
                nb_frame * p_encoder_params->s16NumOfSubBands * p_encoder_params->s16NumOfBlocks *
                2 *
                16 / 8;
        /* no more pcm to read */
        nb_frame = 0;
      }
    } while (((p_buf->data_len + last_frame_len) < a2dp_sbc_encoder_cb.TxAaMtuSize) &&
             (p_buf->number_frame < 0x0F) && nb_frame);

    if (p_buf->data_len) {
      /*
       * Timestamp of the media packet header represent the TS of the
       * first SBC frame, i.e the timestamp before including this frame.
       */
      p_buf->timestamp = a2dp_sbc_encoder_cb.timestamp;

      // Timestamp will wrap over to 0 if stream continues on long enough
      // (>25H @ 48KHz). The parameters are promoted to 64bit to ensure that
      // no unsigned overflow is triggered as ubsan is always enabled.
      a2dp_sbc_encoder_cb.timestamp = ((uint64_t)a2dp_sbc_encoder_cb.timestamp +
                                       (p_buf->number_frame * blocm_x_subband)) &
                                      UINT32_MAX;

      uint8_t done_nb_frame = remain_nb_frame - nb_frame;
      remain_nb_frame = nb_frame;
      if (esp_a2d_source_audio_data_send(a2dp_sbc_encoder_cb.peer, p_buf) != ESP_OK) {
        esp_a2d_audio_buff_free(p_buf);
        return;
      }
    } else {
      a2dp_sbc_encoder_cb.stats.media_read_total_dropped_packets++;
      esp_a2d_audio_buff_free(p_buf);
    }
  }
}

static bool a2dp_sbc_read_feeding(uint32_t* bytes_read) {
  SBC_ENC_PARAMS* p_encoder_params = &a2dp_sbc_encoder_cb.sbc_encoder_params;
  uint16_t blocm_x_subband = p_encoder_params->s16NumOfSubBands * p_encoder_params->s16NumOfBlocks;
  uint32_t read_size;
  uint32_t sbc_sampling = 48000;
  uint32_t src_samples;
  uint16_t bytes_needed = blocm_x_subband * p_encoder_params->s16NumOfChannels *
                          16 / 8;
  static uint16_t up_sampled_buffer[SBC_MAX_NUM_FRAME * SBC_MAX_NUM_OF_BLOCKS *
                                    SBC_MAX_NUM_OF_CHANNELS * SBC_MAX_NUM_OF_SUBBANDS * 2];
  static uint16_t read_buffer[SBC_MAX_NUM_FRAME * SBC_MAX_NUM_OF_BLOCKS * SBC_MAX_NUM_OF_CHANNELS *
                              SBC_MAX_NUM_OF_SUBBANDS];
  uint32_t src_size_used;
  uint32_t dst_size_used;
  bool fract_needed;
  int32_t fract_max;
  int32_t fract_threshold;
  uint32_t nb_byte_read;

  /* Get the SBC sampling rate */
  switch (p_encoder_params->s16SamplingFreq) {
    case SBC_sf48000:
      sbc_sampling = 48000;
      break;
    case SBC_sf44100:
      sbc_sampling = 44100;
      break;
    case SBC_sf32000:
      sbc_sampling = 32000;
      break;
    case SBC_sf16000:
      sbc_sampling = 16000;
      break;
  }

  a2dp_sbc_encoder_cb.stats.media_read_total_expected_reads_count++;
  read_size = bytes_needed - a2dp_sbc_encoder_cb.feeding_state.aa_feed_residue;
  a2dp_sbc_encoder_cb.stats.media_read_total_expected_read_bytes += read_size;
  nb_byte_read = bt_read_pcm(
          ((uint8_t*)a2dp_sbc_encoder_cb.pcmBuffer) +
                  a2dp_sbc_encoder_cb.feeding_state.aa_feed_residue,
          read_size);
  a2dp_sbc_encoder_cb.stats.media_read_total_actual_read_bytes += nb_byte_read;

  *bytes_read = nb_byte_read;
  if (nb_byte_read != read_size) {
      a2dp_sbc_encoder_cb.feeding_state.aa_feed_residue += nb_byte_read;
      return false;
  }
  a2dp_sbc_encoder_cb.stats.media_read_total_actual_reads_count++;
  a2dp_sbc_encoder_cb.feeding_state.aa_feed_residue = 0;
  return true;
}

static uint16_t adjust_effective_mtu(void) {
  uint16_t mtu_size = A2DP_SBC_BUFFER_SIZE - A2DP_SBC_OFFSET - sizeof(BT_HDR);
  if (mtu_size > a2dp_sbc_encoder_cb.peer_mtu) {
    mtu_size = a2dp_sbc_encoder_cb.peer_mtu;
  }
  return mtu_size;
}

static uint8_t calculate_max_frames_per_packet(void) {
  SBC_ENC_PARAMS* p_encoder_params = &a2dp_sbc_encoder_cb.sbc_encoder_params;
  uint16_t result = 0;
  uint32_t frame_len;

  a2dp_sbc_encoder_cb.TxAaMtuSize = adjust_effective_mtu();
  const uint16_t effective_mtu_size = a2dp_sbc_encoder_cb.TxAaMtuSize;

  frame_len = a2dp_sbc_frame_length();

  switch (p_encoder_params->s16SamplingFreq) {
    case SBC_sf44100:
      if (frame_len == 0) {
        frame_len = A2DP_SBC_MAX_HQ_FRAME_SIZE_44_1;
      }
      result = (effective_mtu_size - A2DP_HDR_SIZE) / frame_len;
      break;

    case SBC_sf48000:
      if (frame_len == 0) {
        frame_len = A2DP_SBC_MAX_HQ_FRAME_SIZE_48;
      }
      result = (effective_mtu_size - A2DP_HDR_SIZE) / frame_len;
      break;

    default:
      break;
  }
  return result;
}

static uint16_t a2dp_sbc_source_rate(void) {
  uint16_t rate = A2DP_SBC_DEFAULT_BITRATE;

  /* restrict bitrate if a2dp link is non-edr */
  if (!a2dp_sbc_encoder_cb.peer_is_edr) {
    rate = A2DP_SBC_NON_EDR_MAX_RATE;
  }

  return rate;
}

static uint32_t a2dp_sbc_frame_length(void) {
  SBC_ENC_PARAMS* p_encoder_params = &a2dp_sbc_encoder_cb.sbc_encoder_params;
  uint32_t frame_len = 0;

  switch (p_encoder_params->s16ChannelMode) {
    case SBC_MONO:
    case SBC_DUAL:
      frame_len = A2DP_SBC_FRAME_HEADER_SIZE_BYTES +
                  ((uint32_t)(A2DP_SBC_SCALE_FACTOR_BITS * p_encoder_params->s16NumOfSubBands *
                              p_encoder_params->s16NumOfChannels) /
                   CHAR_BIT) +
                  ((uint32_t)(p_encoder_params->s16NumOfBlocks *
                              p_encoder_params->s16NumOfChannels * p_encoder_params->s16BitPool) /
                   CHAR_BIT);
      break;
    case SBC_STEREO:
      frame_len = A2DP_SBC_FRAME_HEADER_SIZE_BYTES +
                  ((uint32_t)(A2DP_SBC_SCALE_FACTOR_BITS * p_encoder_params->s16NumOfSubBands *
                              p_encoder_params->s16NumOfChannels) /
                   CHAR_BIT) +
                  ((uint32_t)(p_encoder_params->s16NumOfBlocks * p_encoder_params->s16BitPool) /
                   CHAR_BIT);
      break;
    case SBC_JOINT_STEREO:
      frame_len = A2DP_SBC_FRAME_HEADER_SIZE_BYTES +
                  ((uint32_t)(A2DP_SBC_SCALE_FACTOR_BITS * p_encoder_params->s16NumOfSubBands *
                              p_encoder_params->s16NumOfChannels) /
                   CHAR_BIT) +
                  ((uint32_t)(p_encoder_params->s16NumOfSubBands +
                              (p_encoder_params->s16NumOfBlocks * p_encoder_params->s16BitPool)) /
                   CHAR_BIT);
      break;
    default:
      break;
  }
  return frame_len;
}

uint32_t a2dp_sbc_get_bitrate() {
  SBC_ENC_PARAMS* p_encoder_params = &a2dp_sbc_encoder_cb.sbc_encoder_params;
  return p_encoder_params->u16BitRate * 1000;
}

static bool sbc_is_acceptable(const esp_a2d_mcc_t *peer_caps)
{
    if (peer_caps->losc != A2D_SBC_INFO_LEN
        || peer_caps->media_type != A2D_MEDIA_TYPE_AUDIO
        || peer_caps->codec_type != ESP_A2D_MCT_SBC) {
        return false;
    }
    if (peer_caps->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_44K == 0
        || peer_caps->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_48K == 0
        || peer_caps->cie.sbc_info.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO == 0
        || peer_caps->cie.sbc_info.block_len & ESP_A2D_SBC_CIE_BLOCK_LEN_16 == 0
        || peer_caps->cie.sbc_info.num_subbands & ESP_A2D_SBC_CIE_NUM_SUBBANDS_8 == 0
        || peer_caps->cie.sbc_info.alloc_mthd & ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS == 0) {
        return false;
    }
    return true;
}

static void sbc_set_freq(uint32_t freq, esp_a2d_mcc_t *pref_mcc) {
  int32_t sbc_sampling;
  /* Get the SBC sampling rate */
  switch (freq) {
    case SAMPR_48:
      sbc_sampling = SBC_sf48000;
      break;
    case SAMPR_44:
      sbc_sampling = SBC_sf44100;
      break;
    case SAMPR_32:
      sbc_sampling = SBC_sf32000;
      break;
    case SAMPR_16:
      sbc_sampling = SBC_sf16000;
      break;
    default:
      return;
  }
  bool restart_input = false;
  bool restart_output = false;
  bool config_updated = false;
  a2dp_sbc_encoder_update(sbc_sampling, pref_mcc, &restart_input, &restart_output, &config_updated);
}

static void sbc_debug_info(char * buffer, size_t buffer_len) {
  snprintf(buffer, buffer_len, "bitpool %d sampr %d", a2dp_sbc_encoder_cb.sbc_encoder_params.s16BitPool, a2dp_sbc_encoder_cb.sbc_encoder_params.s16SamplingFreq);
}

struct a2dp_codec a2dp_codec_sbc = {
    .codec_id = A2D_CODEC_SBC,
    .is_acceptable = sbc_is_acceptable,
    .init = a2dp_sbc_encoder_init,
    .deinit = a2dp_sbc_encoder_cleanup,
    .get_period = a2dp_sbc_get_encoder_interval_ms,
    .tick = a2dp_sbc_send_frames,
    .set_freq = sbc_set_freq,
    .feeding_reset = a2dp_sbc_feeding_reset,
    .feeding_flush = a2dp_sbc_feeding_flush, 
    .debug_info = sbc_debug_info,
};