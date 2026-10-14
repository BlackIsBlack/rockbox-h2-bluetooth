#define LOGF_ENABLE
#include "action.h"
#include "asm/thread.h"
#include "bluetooth/codecs.h"
#include "bt_hci.h"
#include "btc_av.h"
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
#include "pcm_sw_volume.h"
#include "queue.h"
#include "semaphore.h"
#include "splash.h"
#include "stack/a2d_api.h"
#include "stack/a2d_sbc.h"
#include "system.h"
#include "thread.h"
#include "tick.h"
#include "tlsf.h"
#include "misc.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdarg.h>
#include "mutex.h"
#include "audio.h"
#include "lang.h"
#include "settings.h"
#include "yesno.h"
#include "pcm_sw_volume.h"
#include "strlcpy.h"

/* Event log in /.rockbox/bt_log.txt. Bluedroid's errors/warnings land here
 * too (esp_log.h). Capped so a warning storm can't fill the card. */
#define BT_LOG_MAX (512 * 1024)
static struct mutex bt_log_mutex;
static int bt_log_fd = -2; /* -2: not opened yet, -1: failed or full */
static long bt_log_size;

void bt_log(const char *fmt, ...)
{
    /* static: Bluedroid threads that log only have ~3 KiB of stack */
    static char buf[256];
    mutex_lock(&bt_log_mutex);
    if (bt_log_fd == -2) {
        bt_log_fd = open(ROCKBOX_DIR "/bt_log.txt", O_WRONLY | O_CREAT | O_APPEND, 0666);
        bt_log_size = bt_log_fd >= 0 ? ffilesize(bt_log_fd) : 0;
        if (bt_log_size > BT_LOG_MAX / 2) { /* start fresh, keep it readable */
            ftruncate(bt_log_fd, 0);
            bt_log_size = 0;
        }
    }
    if (bt_log_fd < 0) {
        mutex_unlock(&bt_log_mutex);
        return;
    }
    int n = snprintf(buf, sizeof(buf), "%ld.%02ld ",
                     current_tick / HZ, current_tick % HZ);
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(buf + n, sizeof(buf) - n, fmt, ap);
    va_end(ap);
    if (n >= (int)sizeof(buf) - 1)
        n = sizeof(buf) - 2;
    if (n > 0 && buf[n - 1] != '\n')
        buf[n++] = '\n';

    if (bt_log_size + n <= BT_LOG_MAX) {
        write(bt_log_fd, buf, n);
        fsync(bt_log_fd); /* survive a panic */
        bt_log_size += n;
    }
    mutex_unlock(&bt_log_mutex);
}

#undef logf
#define logf bt_log

/* Shown as the device list title; set from BT threads, drawn by the UI */
#define BT_TITLE_HINT "Play: connect  Menu: BT off"
static char bt_status[64] = BT_TITLE_HINT;
static volatile bool bt_status_changed;
static volatile bool bt_status_popup;

static void bt_set_status(bool popup, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(bt_status, sizeof(bt_status), fmt, ap);
    va_end(ap);
    bt_log("status: %s", bt_status);
    bt_status_popup = popup;
    bt_status_changed = true;
    button_queue_post(BUTTON_REDRAW, 0);
}

#define BT_MAX_NAME_LEN 64
#define BT_MAX_ENTRIES 64

typedef struct {
    bool valid;
    esp_bd_addr_t addr;
    int8_t rssi;
    char name[BT_MAX_NAME_LEN];
} bt_list_entry_t;

static bt_list_entry_t bt_entries[BT_MAX_ENTRIES + 1];

static struct a2dp_codec *current_codec = NULL;
static esp_a2d_audio_state_t bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;

static esp_bd_addr_t bt_target_addr;   /* device we last tried to connect */
static char bt_target_name[BT_MAX_NAME_LEN];
static bool bt_connected;

static const char *bt_codec_name(void)
{
    if (current_codec == &a2dp_codec_ldac)    return "LDAC";
    if (current_codec == &a2dp_codec_aptx_hd) return "aptX HD";
    if (current_codec == &a2dp_codec_aptx)    return "aptX";
    if (current_codec == &a2dp_codec_sbc)     return "SBC";
    return "?";
}

#define NUM_CODECS 4

static struct a2dp_codec *codec_list[NUM_CODECS] = {
    &a2dp_codec_ldac,
    &a2dp_codec_aptx_hd,
    &a2dp_codec_aptx,
    &a2dp_codec_sbc,
};

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

static struct mutex bt_power_mutex; /* UI vs. startup thread */
static bool bt_avrc_play_status_registered;

#define BT_LAST_FILE ROCKBOX_DIR "/bt_last.dat"

/* Losing the headphones pauses playback, like a wired unplug. Called
 * before audio is switched back to the DAC so nothing leaks out there. */
static void bt_pause_playback(void)
{
    int st = audio_status();
    if ((st & AUDIO_STATUS_PLAY) && !(st & AUDIO_STATUS_PAUSE)) {
        bt_log("pausing playback: bluetooth disconnected");
        audio_pause();
    }
}

bool bt_is_enabled(void)
{
    return esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED;
}

static bool bt_lookup_name(const esp_bd_addr_t addr, char *name, size_t size)
{
    size_t len = size - 1;
    if (esp_bt_gap_get_bond_device_property((uint8_t *)addr, "RbDeviceName", name, &len) != ESP_OK)
        return false;
    name[len] = 0;
    return true;
}

static void bt_save_last(const esp_bd_addr_t addr)
{
    int fd = open(BT_LAST_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) {
        write(fd, addr, ESP_BD_ADDR_LEN);
        close(fd);
    }
}

/* Reconnect to the device we were last connected to, if still paired */
static void bt_connect_last(void)
{
    esp_bd_addr_t addr;
    int fd = open(BT_LAST_FILE, O_RDONLY);
    if (fd < 0)
        return;
    bool ok = read(fd, addr, ESP_BD_ADDR_LEN) == ESP_BD_ADDR_LEN;
    close(fd);
    if (!ok || !bt_lookup_name(addr, bt_target_name, sizeof(bt_target_name)))
        return;
    memcpy(bt_target_addr, addr, ESP_BD_ADDR_LEN);
    bt_log("reconnect " ESP_BD_ADDR_STR " '%s'", ESP_BD_ADDR_HEX(addr), bt_target_name);
    if (esp_a2d_source_connect(addr) == ESP_OK)
        bt_set_status(false, "Connecting to %s...", bt_target_name);
}

static void bt_enable(bool quiet)
{
    esp_err_t ret;
    mutex_lock(&bt_power_mutex);
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        if (!quiet)
            splashf(0, "Enabling bluetooth...");
        bt_log("--- enabling bluetooth ---");
        bt_hci_enable();
        esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
        cfg.sc_en = true;
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

        /* Headphones need our play status for in-ear detection to
         * send pause/play */
        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set,
                                            ESP_AVRC_RN_PLAY_STATUS_CHANGE);
        ret = esp_avrc_tg_set_rn_evt_cap(&evt_set);
        if (ret != ESP_OK) {
            panicf("%s avrcp tg init failed: %d", __func__, ret);
        }

        if ((ret = esp_a2d_source_init()) != ESP_OK) {
            panicf("%s a2dp source init failed: %d", __func__, ret);
        }
        if ((ret = esp_a2d_register_callback(bt_app_a2d_cb)) != ESP_OK) {
            panicf("%s a2dp register callback failed: %d", __func__, ret);
        }
        esp_a2d_mcc_t mcc;
        mcc.losc = A2D_SBC_INFO_LEN;
        mcc.media_type = A2D_MEDIA_TYPE_AUDIO;
        mcc.codec_type = ESP_A2D_MCT_SBC;
        mcc.cie.sbc_info.samp_freq    = ESP_A2D_SBC_CIE_SF_44K | ESP_A2D_SBC_CIE_SF_48K;
        mcc.cie.sbc_info.ch_mode      = ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
        mcc.cie.sbc_info.block_len    = ESP_A2D_SBC_CIE_BLOCK_LEN_16;
        mcc.cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
        mcc.cie.sbc_info.alloc_mthd   = ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
        mcc.cie.sbc_info.min_bitpool  = 3;
        mcc.cie.sbc_info.max_bitpool  = 53;
        if ((ret = esp_a2d_source_register_stream_endpoint(0, &mcc)) != ESP_OK) {
            panicf("%s a2dp register stream endpoint failed: %d", __func__, ret);
        }
        if ((ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE)) != ESP_OK) {
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
    mutex_unlock(&bt_power_mutex);
}

void bt_disable(void)
{
    mutex_lock(&bt_power_mutex);
    if (bt_is_enabled()) {
        if (bt_connected)
            bt_pause_playback();
        /* no AVRCP notifications once the stack is going away */
        bt_avrc_play_status_registered = false;
        /* stop the codec thread before tearing the stack down */
        bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;
        sleep(HZ / 10);
        esp_bt_gap_cancel_discovery();
        esp_a2d_source_deinit();
        esp_avrc_tg_deinit();
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        bt_hci_disable();
        if (current_codec != NULL) {
            current_codec->deinit();
            current_codec = NULL;
        }
        if (pcm_get_current_sink() != &builtin_pcm_sink) {
            pcm_switch_sink(PCM_SINK_BUILTIN);
            setvol();
        }
        bt_log("--- bluetooth off ---");
    }
    bt_connected = false;
    bt_avrc_play_status_registered = false;
    strlcpy(bt_status, BT_TITLE_HINT, sizeof(bt_status));
    mutex_unlock(&bt_power_mutex);
}

/* Settings > Bluetooth Settings > Bluetooth: On/Off */
int bt_toggle(void)
{
    if (bt_is_enabled()) {
        bt_disable();
    } else {
        bt_enable(false);
        bt_connect_last();
    }
    return 0;
}

static void bluetooth_enable_discover(void)
{
    esp_err_t ret;
    bt_enable(false);
    if ((ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 5, 0)) != ESP_OK) {
        bt_set_status(true, "Scan failed (%d)", ret);
    }
}

/* "Enable on Startup": firmware upload takes a few seconds, keep it off
 * the boot path */
static uint32_t bt_start_stack[(16 * 1024) / sizeof(uint32_t)];

static void bt_start_task(void)
{
    bt_enable(true);
    bt_connect_last();
    thread_exit();
}

void bt_init(void)
{
    mutex_init(&bt_log_mutex);
    mutex_init(&bt_power_mutex);
    if (global_settings.bt_on_startup)
        create_thread(bt_start_task, bt_start_stack, sizeof(bt_start_stack), 0,
                      "bt_start" IF_PRIO(, PRIORITY_BACKGROUND));
}

int bt_action_callback(int action, struct gui_synclist *lists)
{
    switch (action) {
        case ACTION_REDRAW: {
            if (bt_status_changed) {
                bt_status_changed = false;
                gui_synclist_set_title(lists, bt_status, Icon_NOICON);
                if (bt_status_popup) {
                    bt_status_popup = false;
                    splash(HZ * 2, bt_status);
                }
            }
            int size = 0;
            while (bt_entries[size].valid && size < BT_MAX_ENTRIES) {
                size++;
            }
            gui_synclist_set_nb_items(lists, size * 2);
            return ACTION_REDRAW;
        }
        case ACTION_STD_OK: {
            bt_list_entry_t *entry = (bt_entries + (lists->selected_item / 2));
            if (!entry->valid)
                return ACTION_NONE;
            esp_err_t ret;
            /* paging while an inquiry runs is unreliable; ESP-IDF's own
             * A2DP source example stops discovery first too */
            esp_bt_gap_cancel_discovery();
            memcpy(bt_target_addr, entry->addr, ESP_BD_ADDR_LEN);
            strlcpy(bt_target_name, entry->name, sizeof(bt_target_name));
            bt_log("connect " ESP_BD_ADDR_STR " '%s'", ESP_BD_ADDR_HEX(entry->addr), entry->name);
            if ((ret = esp_a2d_source_connect(entry->addr)) != ESP_OK) {
                bt_set_status(true, "Connect failed to start (%d)", ret);
                return ACTION_NONE;
            }
            bt_set_status(false, "Connecting to %s...", entry->name);
            return ACTION_NONE;
        }
        case ACTION_STD_CONTEXT: {
            bt_list_entry_t *entry = (bt_entries + (lists->selected_item / 2));
            if (!entry->valid)
                return ACTION_NONE;
            esp_err_t ret;
            if ((ret = esp_a2d_source_disconnect(entry->addr)) != ESP_OK) {
                bt_set_status(true, "Disconnect failed (%d)", ret);
                return ACTION_NONE;
            }
            bt_set_status(false, "Disconnecting...");
            return ACTION_NONE;
        }
        case ACTION_STD_MENU:
            bt_disable();
            return ACTION_STD_CANCEL;
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
        if (bt_connected && memcmp(entry->addr, bt_target_addr, ESP_BD_ADDR_LEN) == 0) {
            snprintf(buffer, buffer_len, "Connected (%s)", bt_codec_name());
        } else if (entry->rssi == -128) {
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

    bt_status_changed = false;
    simplelist_info_init(&info, bt_status, size, NULL);
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
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            logf("gap auth " ESP_BD_ADDR_STR " stat %d", ESP_BD_ADDR_HEX(param->auth_cmpl.bda), param->auth_cmpl.stat);
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
                bt_set_status(false, "Paired, connecting audio...");
            else
                bt_set_status(true, "Pairing failed (%d)", param->auth_cmpl.stat);
            return;
        case ESP_BT_GAP_CFM_REQ_EVT:
            logf("gap %d %d", event, param->cfm_req.num_val);
            /* we're connectable now: only pair with the device the user picked */
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda,
                memcmp(param->cfm_req.bda, bt_target_addr, ESP_BD_ADDR_LEN) == 0);
            break;
        case ESP_BT_GAP_PIN_REQ_EVT: {
            logf("gap %d %d", event, param->pin_req.min_16_digit);
            if (memcmp(param->pin_req.bda, bt_target_addr, ESP_BD_ADDR_LEN) != 0) {
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, false, 0, pin_code);
            } else if (param->pin_req.min_16_digit) {
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

/* Settings > Bluetooth Settings > Forget Devices */
static int bt_forget_count;

static int bt_forget_action(int action, struct gui_synclist *lists)
{
    if (action != ACTION_STD_OK || bt_forget_count == 0)
        return action;
    bt_list_entry_t *e = &bt_entries[gui_synclist_get_sel_pos(lists)];
    if (!yesno_pop(str(LANG_BLUETOOTH_FORGET_CONFIRM)))
        return ACTION_REDRAW;
    if (bt_connected && memcmp(e->addr, bt_target_addr, ESP_BD_ADDR_LEN) == 0)
        esp_a2d_source_disconnect(e->addr);
    bt_log("forget " ESP_BD_ADDR_STR " '%s'", ESP_BD_ADDR_HEX(e->addr), e->name);
    esp_bt_gap_remove_bond_device(e->addr);
    /* drop it from the list */
    memmove(e, e + 1, (char *)&bt_entries[BT_MAX_ENTRIES] - (char *)e);
    bt_entries[BT_MAX_ENTRIES].valid = false;
    bt_forget_count--;
    gui_synclist_set_nb_items(lists, bt_forget_count);
    if (bt_forget_count == 0)
        return ACTION_STD_CANCEL;
    return ACTION_REDRAW;
}

static const char *bt_forget_getname(int item, void *data, char *buf, size_t len)
{
    (void)data; (void)buf; (void)len;
    return bt_entries[item].name;
}

int bt_forget_devices(void)
{
    struct simplelist_info info;
    bt_enable(false); /* paired list lives in the stack's config */
    esp_bt_gap_cancel_discovery();
    /* rebuild bt_entries from the bonded list only */
    esp_bd_addr_t bonded[BT_MAX_ENTRIES];
    int n = BT_MAX_ENTRIES;
    esp_bt_gap_get_bond_device_list(&n, bonded);
    for (int i = 0; i < n; i++) {
        memcpy(bt_entries[i].addr, bonded[i], ESP_BD_ADDR_LEN);
        if (!bt_lookup_name(bonded[i], bt_entries[i].name, BT_MAX_NAME_LEN))
            strlcpy(bt_entries[i].name, "Unknown Paired Device", BT_MAX_NAME_LEN);
        bt_entries[i].rssi = -128;
        bt_entries[i].valid = true;
    }
    bt_entries[n].valid = false;
    bt_forget_count = n;
    if (n == 0) {
        splash(HZ * 2, str(LANG_BLUETOOTH_NO_DEVICES));
        return 0;
    }
    simplelist_info_init(&info, str(LANG_BLUETOOTH_FORGET), n, NULL);
    info.action_callback = bt_forget_action;
    info.get_name = bt_forget_getname;
    simplelist_show_list(&info);
    return 0;
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
        case 2:
            if (current_codec == &a2dp_codec_ldac) {
                return "current codec: LDAC";
            } else if (current_codec == &a2dp_codec_aptx_hd) {
                return "current codec: aptX HD";
            } else if (current_codec == &a2dp_codec_aptx) {
                return "current codec: aptX";
            } else if (current_codec == &a2dp_codec_sbc) {
                return "current codec: SBC";
            } else {
                return "current codec: unknown";
            }
        case 3:
            if (current_codec != NULL && current_codec->debug_info != NULL) {
                strcpy(buffer, "codec info: ");
                current_codec->debug_info(buffer + 12, buffer_len - 12);
                return buffer;
            } else {
                return "codec info: N/A";
            }
        case 4:
            snprintf(buffer, buffer_len, "real sink sampr: %lu", pcm_current_sink_caps()->samprs[pcm_get_current_sink()->configured_freq]);
            return buffer;
        default:
            return "Unknown item!!";
    }
}

bool bt_heap_info(void)
{
    struct simplelist_info info;

    simplelist_info_init(&info, "Bluetooth debug info:", 5, NULL);
    info.scroll_all = false;
    info.action_callback = bt_heap_info_action_callback;
    info.get_name = bt_heap_info_getname;
    return simplelist_show_list(&info);
}


/* The codec thread pulls PCM from the mixer (normally done on the IRQ
 * stack), runs the encoder and calls into Bluedroid; MIPS's 1 KiB
 * DEFAULT_STACK_SIZE overflowed after a while. */
#define BT_CODEC_STACK_SIZE (32 * 1024)
uint32_t bt_codec_stack[BT_CODEC_STACK_SIZE / sizeof(uint32_t)];


static void bt_codec_task(void)
{
    int64_t next_ms = (((int64_t)current_tick) * 1000) / HZ;
    while (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED && bt_current_state == ESP_A2D_AUDIO_STATE_STARTED) {
        uint64_t ticked = 0;
        do {
            if (ticked > (1000 / HZ)) {
                ticked %= (1000 / HZ);
                yield();
            }
            if (current_codec == NULL) {
                thread_exit();
                return;
            }
            current_codec->tick();
            next_ms += current_codec->get_period();
            ticked += current_codec->get_period();
        } while ((next_ms * HZ) / 1000 <= current_tick);
        sleep(((next_ms * HZ) / 1000) - current_tick);
    }
    if (current_codec == NULL) {
        thread_exit();
        return;
    }
    current_codec->feeding_flush();
    thread_exit();
}

int bt_get_status_level(void)
{
    if (!global_settings.bt_show_icon || !bt_is_enabled())
        return 0;
    if (!bt_connected)
        return 1;
    return bt_current_state == ESP_A2D_AUDIO_STATE_STARTED ? 3 : 2;
}

static esp_a2d_conn_hdl_t conn_hdl;
static bool sampr_switch_ongoing = false;
static struct semaphore sink_control_sem;
static struct event_queue sink_control_queue;
static esp_a2d_mcc_t new_pref_mcc;
struct pcm_sink bt_pcm_sink;
static esp_a2d_mcc_t peer_caps;

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
            logf("a2d conn state %d rsn %d " ESP_BD_ADDR_STR, param->conn_stat.state, param->conn_stat.disc_rsn, ESP_BD_ADDR_HEX(param->conn_stat.remote_bda));
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                if (memcmp(bt_target_addr, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN) != 0
                    && !bt_lookup_name(param->conn_stat.remote_bda, bt_target_name, sizeof(bt_target_name)))
                    bt_target_name[0] = 0;
                memcpy(bt_target_addr, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
                bt_save_last(bt_target_addr);
                bt_connected = true;
                bt_set_status(true, "Connected: %s", bt_target_name[0] ? bt_target_name : "headphones");
                semaphore_init(&sink_control_sem, 1, 0);
                queue_init(&sink_control_queue, false);
                sink_suspended = 1;
                sampr_switch_ongoing = false;
                bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;
                esp_bt_gap_read_remote_name(param->conn_stat.remote_bda);
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                if (bt_connected)
                    bt_set_status(true, param->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL ? "Disconnected" : "Connection lost");
                else
                    bt_set_status(true, "Could not connect (see bt_log.txt)");
                bt_connected = false;
                bt_avrc_play_status_registered = false;
            }
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED || param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
                if (current_codec != NULL) {
                    bt_pause_playback();
                    bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;
                    current_codec->deinit();
                    current_codec = NULL;
                    pcm_switch_sink(PCM_SINK_BUILTIN);
                    setvol(); /* back to DAC volume */
                }
            }
            return;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
            logf("a2d %d %d %d", event, param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
            if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) {
                pcm_switch_sink(PCM_SINK_BLUETOOTH);
                setvol(); /* BT uses software volume */
            } else if (sampr_switch_ongoing && param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_SUSPEND) {
                esp_a2d_source_set_pref_mcc(conn_hdl, &new_pref_mcc);
            }
            if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_SUSPEND && param->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;
            } else if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START && param->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                bt_current_state = ESP_A2D_AUDIO_STATE_STARTED;
                bt_hci_log_stop_after(1500); /* setup + first ~10 s of audio */
                bt_set_status(false, "Streaming: %s", bt_codec_name());
                current_codec->feeding_reset();
                create_thread(bt_codec_task, bt_codec_stack, BT_CODEC_STACK_SIZE, 0, "bt_codec" IF_PRIO(, PRIORITY_BLUETOOTH));
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
            if (current_codec == NULL)
                return; /* no usable codec, already disconnecting */
            esp_a2d_mcc_t pref_mcc;
            struct a2dp_peer_info peer_info = {
                .peer_mtu = param->a2d_report_snk_codec_caps_stat.mtu,
                .peer_caps = &peer_caps,
                .conn_hdl = param->a2d_report_snk_codec_caps_stat.conn_hdl,
                .is_edr = btc_av_is_peer_edr(),
            };
            conn_hdl = param->a2d_report_snk_codec_caps_stat.conn_hdl;
            current_codec->init(&bt_pcm_sink.caps, &peer_info, &pref_mcc);
            esp_err_t ret = esp_a2d_source_set_pref_mcc(param->a2d_report_snk_codec_caps_stat.conn_hdl, &pref_mcc);

            if (ret != ESP_OK) {
                panicf("%s a2dp source set pref mcc failed: %d", __func__, ret);
            }
            break;
        }
        case ESP_A2D_REPORT_SNK_ALL_CODEC_CAPS_EVT: {
            logf("a2d %d %d", event, param->a2d_report_snk_all_codec_caps_stat.sep_num);
            bool done = false;
            for (int codec_idx = 0; codec_idx < NUM_CODECS; codec_idx++) {
                current_codec = codec_list[codec_idx];
                for (uint32_t i = 0; i < param->a2d_report_snk_all_codec_caps_stat.sep_num; i++) {
                    esp_a2d_mcc_t *sink_mcc = &param->a2d_report_snk_all_codec_caps_stat.sep_mcc[i].mcc;
                    if (current_codec->is_acceptable(sink_mcc)) {
                        memcpy(&peer_caps, sink_mcc, sizeof(esp_a2d_mcc_t));
                        done = true;
                        break;
                    }
                }
                if (done) {
                    break;
                }
            }
            if (!done) {
                current_codec = NULL;
                bt_set_status(true, "No supported codec on headphones");
                esp_a2d_source_disconnect(bt_target_addr);
                return;
            }
            logf("codec chosen: %s", bt_codec_name());
            break;
        }
        case ESP_A2D_SRC_SET_PREF_MCC_EVT:
            logf("a2d %d %d", event, param->a2d_set_pref_mcc_stat.set_status);
            if (sampr_switch_ongoing) {
                sampr_switch_ongoing = false;
                logf("starting playback: sampr switch done");
                bt_sink_resume();
            } else {
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
            }
            return;
        case ESP_A2D_AUDIO_STATE_EVT:
            logf("a2d %d %d\n", event, param->audio_stat.state);
        default:
            logf("a2d %d\n", event);
        return;
    }
}

static esp_avrc_playback_stat_t bt_play_status(void)
{
    int st = audio_status();
    if (!(st & AUDIO_STATUS_PLAY))
        return ESP_AVRC_PLAYBACK_STOPPED;
    return (st & AUDIO_STATUS_PAUSE) ? ESP_AVRC_PLAYBACK_PAUSED : ESP_AVRC_PLAYBACK_PLAYING;
}

/* tell the headphones we started/stopped (they re-register after each) */
static void bt_notify_play_status(esp_avrc_playback_stat_t st)
{
    if (!bt_avrc_play_status_registered)
        return;
    bt_avrc_play_status_registered = false;
    esp_avrc_rn_param_t rn = { .playback = st };
    logf("avrc-tg notify play status %d", st);
    esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_STATUS_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn);
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
                    /* explicit, never toggle: in-ear detection sends these */
                    case ESP_AVRC_PT_CMD_PLAY:
                        button_queue_post(BUTTON_MULTIMEDIA_PLAY, 0);
                        break;
                    case ESP_AVRC_PT_CMD_PAUSE:
                        button_queue_post(BUTTON_MULTIMEDIA_PAUSE, 0);
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
            if (param->reg_ntf.event_id == ESP_AVRC_RN_PLAY_STATUS_CHANGE) {
                esp_avrc_rn_param_t rn = { .playback = bt_play_status() };
                bt_avrc_play_status_registered = true;
                esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_STATUS_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn);
            }
            break;
        default:
            logf("avrc-tg %d", event);
            return;
    }
}

static const void *pcm_data_start = NULL;
static size_t  pcm_data_size = 0;
static int     audio_locked = 0;
struct pcm_sink bt_pcm_sink;

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

    bt_sink_suspend();
    current_codec->set_freq(bt_pcm_sink.caps.samprs[freq], &new_pref_mcc);

    if (bt_current_state == ESP_A2D_AUDIO_STATE_SUSPEND) {
        esp_a2d_source_set_pref_mcc(conn_hdl, &new_pref_mcc);
    }
}

void bt_sink_play(const void* addr, size_t size) {
    pcm_data_start = addr;
    pcm_data_size = size;
    logf("starting playback: starting to play");
    bt_notify_play_status(ESP_AVRC_PLAYBACK_PLAYING);
    bt_sink_resume();
    pcm_play_dma_status_callback(PCM_DMAST_STARTED);
}

void bt_sink_stop(void) {
    pcm_data_start = NULL;
    pcm_data_size = 0;
    logf("suspending playback: stopped playing");
    bt_notify_play_status((audio_status() & AUDIO_STATUS_PLAY) ? ESP_AVRC_PLAYBACK_PAUSED
                                                                : ESP_AVRC_PLAYBACK_STOPPED);
    bt_sink_suspend();
}

static bool get_new_buf_maybe(void) {
    bool ret = pcm_play_dma_complete_callback(PCM_DMAST_OK, &pcm_data_start, &pcm_data_size);
    if (ret) {
        pcm_play_dma_status_callback(PCM_DMAST_STARTED);
    }
    return ret;
}

size_t bt_read_pcm(uint8_t *buf, size_t len) {
    if (len <= 0 || buf == NULL) {
        return 0;
    }
    size_t written = 0;
    if (pcm_data_start != NULL && audio_locked == 0) {
        while (len > 0 && (pcm_data_size > 0 || get_new_buf_maybe())) {
            size_t sent = MIN(pcm_data_size, len);
            memcpy(buf, pcm_data_start, sent);
            pcm_scale_buffer_16((int16_t *)buf, sent); /* BT volume */
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
        .samprs       = NULL,
        .num_samprs   = -1,
        .default_freq = -1,
        .volume_type  = -1,
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