#pragma once
#include "esp_a2dp_api.h"
#include "pcm_sink.h"
#include "stack/a2d_codecs.h"

struct a2dp_peer_info {
    uint16_t peer_mtu;
    esp_a2d_conn_hdl_t conn_hdl;
    const esp_a2d_mcc_t *peer_caps;
    bool is_edr;
};

struct a2dp_codec {
    tA2D_CODEC codec_id;
    bool (*is_acceptable)(const esp_a2d_mcc_t *peer_caps);
    void (*init)(struct pcm_sink_caps *caps, const struct a2dp_peer_info* peer, esp_a2d_mcc_t *pref_mcc);
    void (*feeding_reset)(void);
    void (*feeding_flush)(void);
    uint64_t (*get_period)(void);
    void (*tick)(void);
    void (*deinit)(void);
    void (*set_freq)(uint32_t freq, esp_a2d_mcc_t *pref_mcc);
    void (*debug_info)(char * buffer, size_t buffer_len);
};

extern struct a2dp_codec a2dp_codec_sbc;
extern struct a2dp_codec a2dp_codec_ldac;
extern struct a2dp_codec a2dp_codec_aptx;
extern struct a2dp_codec a2dp_codec_aptx_hd;