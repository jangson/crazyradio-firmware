/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyradio firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * radio.h - nRF cpu radio driver
 */

#ifndef __RADIO_H__
#define __RADIO_H__

#include <stdbool.h>

enum radioMode_e
{
  RADIO_MODE_PTX,
  RADIO_MODE_PRX,
};

//High level functions
void radioInit(enum radioMode_e mode);
void radioDeinit();
uint8_t radioSendPacket(__xdata uint8_t *payload, uint8_t len, 
                              __xdata uint8_t *ackPayload, uint8_t *ackLen);
void radioSendPacketNoAck(__xdata uint8_t  *payload, uint8_t  len);
void radioSetChannel(uint8_t channel);
void radioSetDataRate(uint8_t dr);
uint8_t radioGetDataRate();
void radioSetAddress(__xdata uint8_t* address);
void radioSetPower(uint8_t power);
void radioSetArd(uint8_t ard);
void radioSetArc(uint8_t arc);
void radioSetContCarrier(bool contCarrier);
uint8_t radioGetRpd(void);
uint8_t radioGetTxRetry(void);

void radioSetMode(enum radioMode_e mode);

bool radioIsRxEmpty();

//Each function returns the status register
uint8_t radioNop();
uint8_t radioActivate();
uint8_t radioReadReg(uint8_t addr);
uint8_t radioWriteReg(uint8_t addr, uint8_t value);
void radioTxPacket(__xdata uint8_t *payload, uint8_t len);
void radioTxPacketNoAck(__xdata uint8_t *payload, uint8_t len);
void radioAckPacket(uint8_t pipe, __xdata uint8_t* payload, uint8_t len);
uint8_t radioRxPacket(__xdata uint8_t *payload);

#define ARD_RAW 0
#define ARD_PLOAD 0x80

#endif /* __RADIO_H__ */


