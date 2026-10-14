/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2026 Skye Green
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#include <stdint.h>
#include "bt_hci.h"
#include "devicedata.h"
#include "esp_bluedroid_hci.h"
#include "esp_bt_main.h"
#include "gpio-ingenic.h"
#include "kernel.h"
#include "panic.h"
#include "thread.h"
#include "uart-x1000.h"
#include "semaphore.h"
#include "bcm_firmware.h"

/* HCI capture to /.rockbox/bt_hci.pcap (open in Wireshark). It runs from
 * BT enable until audio starts streaming, which covers pairing/connecting. */
#define BT_DEBUG_PCAP

#ifdef BT_DEBUG_PCAP
#include "file.h"
#include "mutex.h"
#include "tick.h"

static int hci_log_fd = -1;
static struct mutex hci_log_mutex;
/* packets left before the capture closes; -1 = unlimited (fsync each) */
static int hci_log_budget = -1;

typedef struct {
    uint32_t seconds;
    uint32_t microseconds;
    uint32_t captured_packet_length;
    uint32_t original_packet_length;
    uint32_t direction;
} pcap_header_t;

static void hci_log_packet(uint32_t direction, const uint8_t *data, size_t len)
{
    if (hci_log_fd == -1)
        return;
    mutex_lock(&hci_log_mutex);
    if (hci_log_fd != -1) {
        pcap_header_t header = {
            .seconds = current_tick / HZ,
            .microseconds = (current_tick % HZ) * (1000000 / HZ),
            .captured_packet_length = len + 4,
            .original_packet_length = len + 4,
            .direction = direction,
        };
        write(hci_log_fd, &header, 20);
        write(hci_log_fd, data, len);
        if (hci_log_budget < 0) {
            fsync(hci_log_fd); /* setup phase: survive a panic */
        } else if (--hci_log_budget <= 0) {
            close(hci_log_fd);
            hci_log_fd = -1;
        }
    }
    mutex_unlock(&hci_log_mutex);
}

/* the capture writes to the SD card from this thread; FS calls go deep */
#define HCI_STACK_SIZE (DEFAULT_STACK_SIZE + HCI_RECV_BUF_SIZE + 8192)
#else
#define HCI_STACK_SIZE (DEFAULT_STACK_SIZE + HCI_RECV_BUF_SIZE)
#endif

/* Broadcom UART sleep mode (as the OF's brcm_patchram_plus --enable_lpm):
 * the chip may sleep while HOST_WAKE_BT is low; we raise it before sending
 * and drop it after a second with no traffic. The chip raises BT_WAKE_HOST
 * itself before it sends, and our UART never sleeps, so RX needs nothing. */
static bool hci_lpm;            /* sleep mode accepted by the chip */
static volatile bool hci_awake; /* HOST_WAKE_BT is high */
static volatile long hci_last_activity;

static void hci_lpm_tick(void)
{
    if (hci_awake && TIME_AFTER(current_tick, hci_last_activity + HZ)) {
        hci_awake = false;
        gpio_set_level(GPIO_HOST_WAKE_BT, 0);
    }
}

void hci_send_uart(uint8_t *data, uint16_t len) {
#ifdef BT_DEBUG_PCAP
    hci_log_packet(0x01000000, data, len);
#endif
    hci_last_activity = current_tick;
    if (hci_lpm && !hci_awake) {
        hci_awake = true;
        gpio_set_level(GPIO_HOST_WAKE_BT, 1);
        sleep(1); /* chip wake-up takes a few ms */
    }
    uart_tx(PORT_UART0, data, len);
}

static volatile bool hci_stopping;

/* Receive exactly n bytes; false on timeout or shutdown */
static bool hci_rx_exact(uint8_t *data, size_t n, int tmo)
{
    size_t got = 0;
    while (got < n) {
        size_t r = uart_rx_timeout(PORT_UART0, data + got, n - got, tmo);
        if (r == 0 || hci_stopping)
            return false;
        got += r;
    }
    return true;
}

/* Receive one H4 packet; returns its length, 0 on timeout/shutdown */
static size_t hci_recv_uart_tmo(uint8_t *data, size_t len, int tmo) {
    size_t hdr, expected;
    if (!hci_rx_exact(data, 1, tmo))
        return 0;
    switch (data[0]) {
        case 0x01: hdr = 4; break; /* command */
        case 0x02: hdr = 5; break; /* ACL */
        case 0x03: hdr = 4; break; /* SCO */
        case 0x04: hdr = 3; break; /* event */
        case 0x05: hdr = 5; break; /* ISO */
        default:
            /* the stream is out of sync and can't be recovered */
            panicf("hci got unknown frame type %d", data[0]);
    }
    if (!hci_rx_exact(data + 1, hdr - 1, tmo))
        return 0;
    if (data[0] == 0x04)
        expected = data[2] + 3;
    else if (data[0] == 0x02 || data[0] == 0x05)
        expected = (data[3] | (((size_t) data[4]) << 8)) + 5;
    else
        expected = data[3] + 4;
    if (len < expected)
        panicf("hci recv buffer too small, got %u but wanted at least %u", len, expected);
    if (!hci_rx_exact(data + hdr, expected - hdr, tmo))
        return 0;
    hci_last_activity = current_tick;
#ifdef BT_DEBUG_PCAP
    hci_log_packet(0x00000000, data, expected);
#endif
    return expected;
}

size_t hci_recv_uart(uint8_t *data, size_t len) {
    return hci_recv_uart_tmo(data, len, TIMEOUT_BLOCK);
}

bool hci_check_send_available_uart(void) {
    return true;
}

const esp_bluedroid_hci_driver_callbacks_t *hci_callbacks;
struct semaphore hci_callbacks_sem;

esp_err_t hci_register_host_callback_uart(const esp_bluedroid_hci_driver_callbacks_t *callback) {
    if (hci_callbacks != NULL) {
        return ESP_FAIL;
    }
    hci_callbacks = callback;
    semaphore_release(&hci_callbacks_sem);
    return ESP_OK;
}

#define HCI_RECV_BUF_SIZE 1026

uint32_t hci_stack[HCI_STACK_SIZE / sizeof(uint32_t)];
static unsigned int hci_thread_id;

void bluetooth_hci_task(void)
{
    uint8_t buf[HCI_RECV_BUF_SIZE];

    semaphore_wait(&hci_callbacks_sem, TIMEOUT_BLOCK);

    while (!hci_stopping) {
        size_t recvd = hci_recv_uart(buf, HCI_RECV_BUF_SIZE);
        if (recvd == 0 || hci_stopping ||
            esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED)
            break;
        hci_callbacks->notify_host_recv(buf, recvd);
    }
    hci_callbacks = NULL;
    thread_exit();
}

/* Send an init command and wait for the chip's reply */
static bool hci_init_cmd(const uint8_t *cmd, size_t len, uint8_t *buf)
{
    hci_send_uart((uint8_t *)cmd, len);
    return hci_recv_uart_tmo(buf, HCI_RECV_BUF_SIZE, HZ) != 0;
}

static const uint8_t hci_reset[] = {0x01, 0x03, 0x0c, 0x00};
/* Update_UART_Baud_Rate: explicit 3000000 */
static const uint8_t hci_baud_3m[] = {0x01, 0x18, 0xfc, 0x06, 0x00, 0x00, 0xc0, 0xc6, 0x2d, 0x00};

/* 115200 -> 3 Mbaud, chip first (its reply still comes at the old rate) */
static bool hci_go_fast(uint8_t *buf)
{
    if (!hci_init_cmd(hci_baud_3m, sizeof(hci_baud_3m), buf))
        return false;
    uart_set_baud(PORT_UART0, 3000000);
    sleep(HZ / 100 + 1);
    return true;
}

bool bt_hci_enable(bool capture)
{
    uint8_t buf[HCI_RECV_BUF_SIZE];

    if (device_data.hw_rev != 4)
        return false; /* pins/PMU only known for hw4 */

#ifdef BT_DEBUG_PCAP
    hci_log_budget = -1;
    hci_log_fd = -1;
    mutex_init(&hci_log_mutex);
    if (capture) {
        hci_log_fd = open(ROCKBOX_DIR "/bt_hci.pcap", O_WRONLY | O_CREAT | O_TRUNC, 0666);
        write(hci_log_fd, (uint8_t[]){0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc9, 0x00, 0x00, 0x00}, 24);
    }
#else
    (void)capture;
#endif

    hci_stopping = false;
    hci_lpm = false;
    hci_awake = true;
    gpio_set_level(GPIO_HOST_WAKE_BT, 1); /* awake (ignored until sleep mode is on) */
    uart_init(PORT_UART0, 115200);

    /* Same order as the OF kernel: 32 kHz LPO clock out on PB26 first,
     * then BT_REG_ON. The chip needs time to boot before it answers. */
    gpio_set_function(GPIO_PB(26), GPIOF_DEVICE(0));
    sleep(HZ / 100 + 1);
    gpio_set_level(GPIO_BT_REG_ON_HW4, 1);
    sleep(HZ / 10);

    /* Upload the firmware patch at 3 Mbaud like the OF does
     * (--use_baudrate_for_download): ~3 s at 115200 otherwise */
    if (!hci_init_cmd(hci_reset, sizeof(hci_reset), buf) || !hci_go_fast(buf))
        goto fail;

    // Download_Minidriver
    if (!hci_init_cmd((uint8_t[]){0x01, 0x2e, 0xfc, 0x00}, 4, buf))
        goto fail;
    sleep(HZ / 20);

    for (uint8_t **i = hcd_init_commands; *i != NULL; i++) {
        if (!hci_init_cmd(*i, ((size_t) (*i)[3]) + 4, buf))
            goto fail;
    }

    /* Launch_RAM reboots the chip, back at 115200; Linux btbcm waits
     * 250 ms here too */
    sleep(HZ / 4);
    uart_set_baud(PORT_UART0, 115200);

    if (!hci_init_cmd(hci_reset, sizeof(hci_reset), buf) || !hci_go_fast(buf))
        goto fail;

    /* Sleep mode: UART with BT_WAKE/HOST_WAKE, both active high, 300 ms
     * idle thresholds - the OF's parameters */
    static const uint8_t sleep_mode[] = {0x01, 0x27, 0xfc, 0x0c,
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (hci_init_cmd(sleep_mode, sizeof(sleep_mode), buf) && buf[6] == 0x00) {
        hci_lpm = true;
        hci_last_activity = current_tick;
        tick_add_task(hci_lpm_tick);
    }

    hci_callbacks = NULL;
    semaphore_init(&hci_callbacks_sem, 1, 0);

    esp_bluedroid_hci_driver_operations_t operations = {
        .send = hci_send_uart,
        .check_send_available = hci_check_send_available_uart,
        .register_host_callback = hci_register_host_callback_uart,
    };
    esp_bluedroid_attach_hci_driver(&operations);
    hci_thread_id = create_thread(bluetooth_hci_task, hci_stack, HCI_STACK_SIZE, 0, "bt_hci" IF_PRIO(, PRIORITY_BLUETOOTH));
    return true;

fail:
    hci_thread_id = 0;
    bt_hci_disable();
    return false;
}

void bt_hci_log_stop_after(int packets)
{
#ifdef BT_DEBUG_PCAP
    if (hci_log_fd < 0)
        return;
    mutex_lock(&hci_log_mutex);
    if (packets > 0 && hci_log_budget < 0) {
        hci_log_budget = packets;
    } else if (packets <= 0) {
        close(hci_log_fd);
        hci_log_fd = -1;
    }
    mutex_unlock(&hci_log_mutex);
#endif
}

void bt_hci_disable(void)
{
    if (hci_lpm) {
        tick_remove_task(hci_lpm_tick);
        hci_lpm = false;
    }
    /* stop the RX thread before its stack can be reused */
    hci_stopping = true;
    if (hci_thread_id) {
        semaphore_release(&hci_callbacks_sem); /* if it never got going */
        uart_deinit(PORT_UART0);               /* wakes a pending uart_rx */
        thread_wait(hci_thread_id);
        hci_thread_id = 0;
    } else {
        uart_deinit(PORT_UART0);
    }
    gpio_set_level(GPIO_HOST_WAKE_BT, 0);
    gpio_set_level(GPIO_BT_REG_ON_HW4, 0);
    gpio_set_function(GPIO_PB(26), GPIOF_INPUT);
    bt_hci_log_stop_after(0);
}