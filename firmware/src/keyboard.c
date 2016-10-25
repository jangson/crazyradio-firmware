/**
 * HID keyboard
 * Jason (Hyun-joon Kim)
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "nRF24LU1p.h"
#include "nRF24L01.h"

#include "pinout.h"
#include "utils.h"
#include "radio.h"
#include "usb.h"
#include "led.h"
#include "hal_uart.h"

#include "keyboard.h"

__xdata uint8_t radio_channel = 100;
__xdata uint8_t radio_data_rate = DATA_RATE_2M;
__xdata uint8_t pipo0_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };
__xdata uint8_t pipo1_address[5] = { 0xc2, 0xc2, 0xc2, 0xc2, 0xc2 };
__xdata uint8_t pipo2_5_address[5] = { 0xc3, 0xc4, 0xc5, 0xc5 };

void hid_report_key(uint8_t key)
{
    key = 0;
}

void init_keyboard(void)
{
    radioSetAutoAck(true);
    radioEnableDynamicPayloads();
    radioEnableAckPayload();
    radioSetAddress(pipo0_address);
    radioSetDataRate(radio_data_rate);
    radioSetChannel(radio_channel);
    radioSetMode(RADIO_MODE_PRX);
}

