#define LOGF_ENABLE
#include "action.h"
#include "bt_hci.h"
#include "button.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "list.h"
#include "logf.h"
#include "panic.h"
#include "pcm-internal.h"
#include "pcm.h"
#include "pcm_sink.h"
#include "queue.h"
#include "semaphore.h"
#include "splash.h"
#include "system.h"
#include "tick.h"
#include "tlsf.h"
#include "misc.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BT_MAX_NAME_LEN 64
#define BT_MAX_ENTRIES 64

typedef struct {
    bool valid;
    esp_bd_addr_t addr;
    int8_t rssi;
    char name[BT_MAX_NAME_LEN];
} bt_list_entry_t;

static bt_list_entry_t bt_entries[BT_MAX_ENTRIES + 1];

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);
static int32_t bt_app_a2d_data_cb(uint8_t *buf, int32_t len);

void bluetooth_enable_discover(void)
{
    esp_err_t ret;
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        splashf(0, "Enabling bluetooth...");
        bt_hci_enable();
        esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
        if ((ret = esp_bluedroid_init_with_cfg(&cfg)) != ESP_OK) {
            panicf("%s initialize bluedroid failed: %d", __func__, ret);
        }
        if ((ret = esp_bluedroid_enable()) != ESP_OK) {
            panicf("%s enable bluedroid failed: %d", __func__, ret);
        }
        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
        if ((ret = esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t))) != ESP_OK) {
            panicf("%s set security param failed: %d", __func__, ret);
        }
        esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
        esp_bt_pin_code_t pin_code;
        esp_bt_gap_set_pin(pin_type, 0, pin_code);
        if ((ret = esp_bt_gap_register_callback(bt_app_gap_cb)) != ESP_OK) {
            panicf("%s set gap cb failed: %d", __func__, ret);
        }
        char device_name[32] = "Rockbox";
        int fd = open_utf8(ROCKBOX_DIR "/playername.txt", O_RDONLY);
        if (fd >= 0) {
            read_line(fd, device_name, sizeof(device_name));
            close(fd);
            device_name[sizeof(device_name)-1] = 0;
        }
        if ((ret = esp_bt_gap_set_device_name(device_name)) != ESP_OK) {
            panicf("%s set device name failed: %d", __func__, ret);
        }
        if ((ret = esp_avrc_tg_init()) != ESP_OK) {
            panicf("%s avrcp tg init failed: %d", __func__, ret);
        }
        if ((ret = esp_avrc_tg_register_callback(bt_app_avrc_tg_cb)) != ESP_OK) {
            panicf("%s avrcp tg init failed: %d", __func__, ret);
        }

        esp_avrc_psth_bit_mask_t psth = {0};
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_PLAY);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_PAUSE);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_STOP);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_FORWARD);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_BACKWARD);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_FAST_FORWARD);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_REWIND);
        ret = esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD,
                                                &psth);
        if (ret != ESP_OK) {
            panicf("%s avrcp tg init failed: %d", __func__, ret);
        }

        // esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        // esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set,
        //                                     ESP_AVRC_RN_PLAY_STATUS_CHANGE);
        // ret = esp_avrc_tg_set_rn_evt_cap(&evt_set);
        // if (ret != ESP_OK) {
        //     panicf("%s avrcp tg init failed: %d", __func__, ret);
        // }

        if ((ret = esp_a2d_source_init()) != ESP_OK) {
            panicf("%s a2dp source init failed: %d", __func__, ret);
        }
        if ((ret = esp_a2d_register_callback(bt_app_a2d_cb)) != ESP_OK) {
            panicf("%s a2dp register callback failed: %d", __func__, ret);
        }
        if ((ret = esp_a2d_source_register_data_callback(bt_app_a2d_data_cb)) != ESP_OK) {
            panicf("%s a2dp register data callback failed: %d", __func__, ret);
        }
        if ((ret = esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE)) != ESP_OK) {
            panicf("%s set scan mode failed: %d", __func__, ret);
        }
        esp_bd_addr_t bonded_addrs[BT_MAX_ENTRIES];
        int num_bonded_addrs = BT_MAX_ENTRIES;
        esp_bt_gap_get_bond_device_list(&num_bonded_addrs, bonded_addrs);
        for (int i = 0; i < num_bonded_addrs; i++) {
            memcpy(bt_entries[i].addr, (bonded_addrs + i), ESP_BD_ADDR_LEN);
            size_t len = BT_MAX_NAME_LEN - 1;
            if (esp_bt_gap_get_bond_device_property(bt_entries[i].addr, "RbDeviceName", bt_entries[i].name, &len) == ESP_OK) {
                bt_entries[i].name[len] = 0;
            } else {
                strncpy(bt_entries[i].name, "Unknown Paired Device", BT_MAX_NAME_LEN);
            }
            bt_entries[i].rssi = -128;
            bt_entries[i].valid = true;
        }
        bt_entries[num_bonded_addrs].valid = false;
    }
    if ((ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 5, 0)) != ESP_OK) {
        panicf("%s start discovery failed: %d", __func__, ret);
    }
}

int bt_action_callback(int action, struct gui_synclist *lists)
{
    switch (action) {
        case ACTION_REDRAW: {
            int size = 0;
            while (bt_entries[size].valid && size < BT_MAX_ENTRIES) {
                size++;
            }
            gui_synclist_set_nb_items(lists, size * 2);
            return ACTION_REDRAW;
        }
        case ACTION_STD_OK: {
            bt_list_entry_t *entry = (bt_entries + (lists->selected_item / 2));
            esp_err_t ret;
            if ((ret = esp_a2d_source_connect(entry->addr)) != ESP_OK) {
                panicf("%s a2dp connect failed: %d", __func__, ret);
            }
            return ACTION_NONE;
        }
        case ACTION_STD_CONTEXT: {
            bt_list_entry_t *entry = (bt_entries + (lists->selected_item / 2));
            esp_err_t ret;
            if ((ret = esp_a2d_source_disconnect(entry->addr)) != ESP_OK) {
                panicf("%s a2dp disconnect failed: %d", __func__, ret);
            }
            return ACTION_NONE;
        }
        case ACTION_STD_MENU: {
            esp_bt_gap_cancel_discovery();
            esp_a2d_source_deinit();
            esp_avrc_tg_deinit();
            esp_bluedroid_disable();
            esp_bluedroid_deinit();
            bt_hci_disable();
            return ACTION_STD_CANCEL;
        }
        default:
            return action;
    }
}

const char * bt_getname(int selected_item, void * data, char * buffer, size_t buffer_len)
{
    bt_list_entry_t *entry = (bt_entries + (selected_item / 2));
    if (selected_item % 2 == 0) {
        return entry->name;
    } else {
        if (entry->rssi == -128) {
            snprintf(buffer, buffer_len, ESP_BD_ADDR_STR " (Saved)", ESP_BD_ADDR_HEX(entry->addr));
        } else {
            snprintf(buffer, buffer_len, "%d dBm " ESP_BD_ADDR_STR, entry->rssi, ESP_BD_ADDR_HEX(entry->addr));
        }
        return buffer;
    }
}

bool bt_scan(void)
{
    struct simplelist_info info;


    bluetooth_enable_discover();
    int size = 0;
    while (bt_entries[size].valid && size < BT_MAX_ENTRIES) {
        size++;
    }

    simplelist_info_init(&info, "Bluetooth devices:", size, NULL);
    info.scroll_all = false;
    info.selection_size = 2;
    info.action_callback = bt_action_callback;
    info.get_name = bt_getname;
    return simplelist_show_list(&info);
}

static void process_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
    uint32_t cod = 0;     /* class of device */
    uint8_t *eir = NULL;
    esp_bt_gap_dev_prop_t *p;
    int8_t rssi;

    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            eir = (uint8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(uint8_t *)(p->val);
        default:
            break;
        }
    }

    // /* search for device with MAJOR service class as "rendering" and "audio" in COD */
    // if (!esp_bt_gap_is_valid_cod(cod) ||
    //         !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING) ||
    //         !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_AUDIO)) {
    //     return;
    // }

    if (!esp_bt_gap_is_valid_cod(cod) || eir == NULL) {
        return;
    }

    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    /* get complete or short local name from eir data */
    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }
    if (!rmt_bdname) {
        return; // Don't show nameless devices
    }

    int bdname_len = MIN(rmt_bdname_len, BT_MAX_NAME_LEN - 1);

    int i = 0;
    while (i < BT_MAX_ENTRIES && (bt_entries[i].valid && (memcmp(bt_entries[i].addr, param->disc_res.bda, ESP_BD_ADDR_LEN) != 0))) {
        i++;
    }
    if (i < BT_MAX_ENTRIES) {
        memcpy(bt_entries[i].addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
        memcpy(bt_entries[i].name, rmt_bdname, bdname_len);
        bt_entries[i].name[bdname_len] = '\0';
        bt_entries[i].rssi = rssi;
        bt_entries[i].valid = true;
        button_queue_post(BUTTON_REDRAW, 0);
    }
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            logf("gap %d " ESP_BD_ADDR_STR, event, ESP_BD_ADDR_HEX(param->disc_res.bda));
            process_inquiry_scan_result(param);
            return;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            logf("gap %d %d", event, param->disc_st_chg.state);
            return;
        case ESP_BT_GAP_CFM_REQ_EVT:
            logf("gap %d %d", event, param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_PIN_REQ_EVT: {
            logf("gap %d %d", event, param->pin_req.min_16_digit);
            if (param->pin_req.min_16_digit) {
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '0';
                pin_code[1] = '0';
                pin_code[2] = '0';
                pin_code[3] = '0';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT: {
            logf("gap %d %d", event, param->read_rmt_name.stat);
            if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
                param->read_rmt_name.rmt_name[BT_MAX_NAME_LEN - 1] = 0;
                esp_bt_gap_set_bond_device_property(param->read_rmt_name.bda, "RbDeviceName", param->read_rmt_name.rmt_name, strlen(param->read_rmt_name.rmt_name));
            }
            break;
        }
        default:
            logf("gap %d", event);
            return;
    }
}

extern unsigned char btbuf[];

int bt_heap_info_action_callback(int action, struct gui_synclist *lists)
{
    switch (action) {
        case ACTION_NONE:
            return ACTION_REDRAW;
        default:
            return action;
    }
}

const char * bt_heap_info_getname(int selected_item, void * data, char * buffer, size_t buffer_len)
{
    switch (selected_item) {
        case 0:
#ifndef SIMULATOR
            snprintf(buffer, buffer_len, "heap usage: %d/%d", get_used_size(btbuf), get_max_size(btbuf));
            return buffer;
#else
            return "this is sim build you dummy";
#endif
        case 1: {
            const uint8_t *addr = esp_bt_dev_get_address();
            if (addr) {
                snprintf(buffer, buffer_len, "MAC addr: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(addr));
                return buffer;
            } else {
                return "MAC addr: unknown";
            }
        }
        default:
            return "Unknown item!!";
    }
}

bool bt_heap_info(void)
{
    struct simplelist_info info;

    simplelist_info_init(&info, "Bluetooth debug info:", 2, NULL);
    info.scroll_all = false;
    info.action_callback = bt_heap_info_action_callback;
    info.get_name = bt_heap_info_getname;
    return simplelist_show_list(&info);
}

static esp_a2d_conn_hdl_t conn_hdl;
static bool sampr_switch_ongoing = false;
static struct semaphore sink_control_sem;
static struct event_queue sink_control_queue;
static esp_a2d_mcc_t new_pref_mcc;

static int sink_suspended = 1;

bool bt_sink_suspend(void) {
    if (++sink_suspended == 1) {
        logf("suspending playback: sink locked");
        if (semaphore_wait(&sink_control_sem, TIMEOUT_NOBLOCK) == OBJ_WAIT_SUCCEEDED) {
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
        } else {
            queue_post(&sink_control_queue, 1, ESP_A2D_MEDIA_CTRL_SUSPEND);
        }
        return true;
    }
    return false;
}
bool bt_sink_resume(void) {
    if (--sink_suspended == 0) {
        logf("starting playback: sink unlocked");
        if (semaphore_wait(&sink_control_sem, TIMEOUT_NOBLOCK) == OBJ_WAIT_SUCCEEDED) {
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
        } else {
            queue_post(&sink_control_queue, 1, ESP_A2D_MEDIA_CTRL_START);
        }
        return true;
    }
    return false;
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            logf("a2d %d %d", event, param->conn_stat.state);
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                semaphore_init(&sink_control_sem, 1, 0);
                queue_init(&sink_control_queue, false);
                sink_suspended = 1;
                sampr_switch_ongoing = false;
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
                esp_bt_gap_read_remote_name(param->conn_stat.remote_bda);
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED || param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
                pcm_switch_sink(PCM_SINK_BUILTIN);
            }
            return;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
            logf("a2d %d %d %d", event, param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
            if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) {
                pcm_switch_sink(PCM_SINK_BLUETOOTH);
            } else if (sampr_switch_ongoing && param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_SUSPEND) {
                esp_a2d_source_set_pref_mcc(conn_hdl, &new_pref_mcc);
            }
            struct queue_event ev;
            queue_wait_w_tmo(&sink_control_queue, &ev, TIMEOUT_NOBLOCK);
            if (ev.id == 1) {
                esp_a2d_media_ctrl(ev.data);
            } else if (ev.id == SYS_TIMEOUT) {
                semaphore_release(&sink_control_sem);
            }
            return;
        case ESP_A2D_REPORT_SNK_CODEC_CAPS_EVT: {
            logf("a2d %d %d", event, param->a2d_report_snk_codec_caps_stat.mcc.type);
            esp_a2d_mcc_t *sink_mcc = &param->a2d_report_snk_codec_caps_stat.mcc;
            if (sink_mcc->type == ESP_A2D_MCT_SBC) {
                esp_a2d_mcc_t pref_mcc;
                pref_mcc.type = ESP_A2D_MCT_SBC;
                pref_mcc.cie.sbc_info.samp_freq    = ESP_A2D_SBC_CIE_SF_44K;
                pref_mcc.cie.sbc_info.ch_mode      = ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
                pref_mcc.cie.sbc_info.block_len    = ESP_A2D_SBC_CIE_BLOCK_LEN_16;
                pref_mcc.cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
                pref_mcc.cie.sbc_info.alloc_mthd   = ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
                pref_mcc.cie.sbc_info.min_bitpool  = 3;
                pref_mcc.cie.sbc_info.max_bitpool  = 53;

                conn_hdl = param->a2d_report_snk_codec_caps_stat.conn_hdl;
                esp_err_t ret = esp_a2d_source_set_pref_mcc(param->a2d_report_snk_codec_caps_stat.conn_hdl, &pref_mcc);

                if (ret != ESP_OK) {
                    panicf("%s a2dp source set pref mcc failed: %d", __func__, ret);
                }
            }
            break;
        }
        case ESP_A2D_SRC_SET_PREF_MCC_EVT:
            logf("a2d %d %d", event, param->a2d_set_pref_mcc_stat.set_status);
            if (sampr_switch_ongoing) {
                sampr_switch_ongoing = false;
                logf("starting playback: sampr switch done");
                bt_sink_resume();
            }
            return;
        default:
            logf("a2d %d", event);
        return;
    }
}

static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
    switch (event) {
        case ESP_AVRC_TG_CONNECTION_STATE_EVT:
            logf("avrc-tg %d %d", event, param->conn_stat.connected);
            break;
        case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
            logf("avrc-tg %d %d %d", event, param->rmt_feats.ct_feat_flag, param->rmt_feats.feat_mask);
            break;
        case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
            logf("avrc-tg %d %d %d", event, param->psth_cmd.key_code, param->psth_cmd.key_state);
            if (param->psth_cmd.key_state == 1) {
                switch (param->psth_cmd.key_code) {
                    case ESP_AVRC_PT_CMD_PLAY:
                    case ESP_AVRC_PT_CMD_PAUSE:
                        button_queue_post(BUTTON_MULTIMEDIA_PLAYPAUSE, 0);
                        break;
                    case ESP_AVRC_PT_CMD_STOP:
                        button_queue_post(BUTTON_MULTIMEDIA_STOP, 0);
                        break;
                    case ESP_AVRC_PT_CMD_FORWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_NEXT, 0);
                        break;
                    case ESP_AVRC_PT_CMD_BACKWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_PREV, 0);
                        break;
                    case ESP_AVRC_PT_CMD_REWIND:
                        button_queue_post(BUTTON_MULTIMEDIA_REW, 0);
                        break;
                    case ESP_AVRC_PT_CMD_FAST_FORWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_FFWD, 0);
                        break;
                }
            }
            break;
        case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
            logf("avrc-tg %d %d", event, param->reg_ntf.event_id);
            break;
        default:
            logf("avrc-tg %d", event);
            return;
    }
}

const unsigned long bt_freq_sampr[2] =
{
    SAMPR_44,
    SAMPR_48,
};

const uint8_t bt_freq_sampr_cie[2] =
{
    ESP_A2D_SBC_CIE_SF_44K,
    ESP_A2D_SBC_CIE_SF_48K,
};

static const void *pcm_data_start = NULL;
static size_t  pcm_data_size = 0;
static int     audio_locked = 0;

void bt_sink_init(void) {
}

void bt_sink_postinit(void) {
}

void bt_sink_lock(void) {
    audio_locked++;
}

void bt_sink_unlock(void) {
    audio_locked--;
}

void bt_sink_set_freq(uint16_t freq) {
    sampr_switch_ongoing = true;

    logf("suspending playback: sampr switch start, to %d", freq);

    new_pref_mcc.type = ESP_A2D_MCT_SBC;
    new_pref_mcc.cie.sbc_info.samp_freq    = bt_freq_sampr_cie[freq];
    new_pref_mcc.cie.sbc_info.ch_mode      = ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
    new_pref_mcc.cie.sbc_info.block_len    = ESP_A2D_SBC_CIE_BLOCK_LEN_16;
    new_pref_mcc.cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
    new_pref_mcc.cie.sbc_info.alloc_mthd   = ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
    new_pref_mcc.cie.sbc_info.min_bitpool  = 3;
    new_pref_mcc.cie.sbc_info.max_bitpool  = 53;

    if (!bt_sink_suspend()) {
        esp_a2d_source_set_pref_mcc(conn_hdl, &new_pref_mcc);
    }
}

void bt_sink_play(const void* addr, size_t size) {
    pcm_data_start = addr;
    pcm_data_size = size;
    logf("starting playback: starting to play");
    bt_sink_resume();
    pcm_play_dma_status_callback(PCM_DMAST_STARTED);
}

void bt_sink_stop(void) {
    pcm_data_start = NULL;
    pcm_data_size = 0;
    logf("suspending playback: stopped playing");
    bt_sink_suspend();
}

static bool get_new_buf_maybe(void) {
    bool ret = pcm_play_dma_complete_callback(PCM_DMAST_OK, &pcm_data_start, &pcm_data_size);
    if (ret) {
        pcm_play_dma_status_callback(PCM_DMAST_STARTED);
    }
    return ret;
}

static int32_t bt_app_a2d_data_cb(uint8_t *buf, int32_t len) {
    if (len <= 0 || buf == NULL) {
        return 0;
    }
    size_t written = 0;
    if (pcm_data_start != NULL && audio_locked == 0) {
        while (len > 0 && (pcm_data_size > 0 || get_new_buf_maybe())) {
            size_t sent = MIN(pcm_data_size, len);
            memcpy(buf, pcm_data_start, sent);
            pcm_data_start += sent;
            pcm_data_size -= sent;
            buf += sent;
            len -= sent;
            written += sent;
        }
    } else {
        // Transmit digital silence
        written = len;
        memset(buf, 0, len);
    }
    return written;
}

struct pcm_sink bt_pcm_sink = {
    .caps = {
        .samprs       = bt_freq_sampr,
        .num_samprs   = 2,
        .default_freq = 0,
        .volume_type  = PCM_SINK_HWVOL,
    },
    .ops = {
        .init     = bt_sink_init,
        .postinit = bt_sink_postinit,
        .set_freq = bt_sink_set_freq,
        .lock     = bt_sink_lock,
        .unlock   = bt_sink_unlock,
        .play     = bt_sink_play,
        .stop     = bt_sink_stop,
    },
};