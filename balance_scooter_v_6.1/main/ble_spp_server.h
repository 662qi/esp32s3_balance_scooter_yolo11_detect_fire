/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef H_BLESPPSERVER_
#define H_BLESPPSERVER_

#include <stdbool.h>
#include "nimble/ble.h"
#include "modlog/modlog.h"
#include "esp_peripheral.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16                                  0xABF0

/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16                              0xABF1

#define RX_PACKET_MAX_LEN   128

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

extern QueueHandle_t cmd_queue;

#ifdef __cplusplus
}
#endif

void myble_init(void);
void myuart_printf(const char *fmt, ...);

#endif
