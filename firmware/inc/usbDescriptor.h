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
 * usbDescriptors.h : USB descriptors header
 */

#ifndef __USB_DESCRIPTOR_H__
#define __USB_DESCRIPTOR_H__

#include <stdint.h>

//USB descriptor classes
#define DEVICE_DESCRIPTOR 0x01
#define CONFIGURATION_DESCRIPTOR 0x02
#define STRING_DESCRIPTOR 0x03
#define INTERFACE_DESCRIPTOR 0x04
#define ENDPOINT_DESCRIPTOR 0x05

//USB HID descriptor classes
#define HID_DESCRIPTOR 0x21
#define HID_REPORT     0x22


//Descriptors instanciated in usbDescriptor.c
extern __code const uint8_t usbDeviceDescriptor[];
extern __code const uint8_t usbConfigurationDescriptor[57];

#ifdef PPM_JOYSTICK
extern __code const uint8_t usbHidReportDescriptor[32];
#endif
#ifdef PPM_KEYBOARD 
#define HID_REP_DESC_SIZE_KEYBOARD   46
extern __code const uint8_t usbHidReportDescriptorKeyboard[HID_REP_DESC_SIZE_KEYBOARD];
#endif

extern __code char usbStringDescriptor0[4];
extern __code char usbStringDescriptor1[18];
extern __code char usbStringDescriptor2[];

//Offset from the beginning of the configration decr. to the HID descr.
#define USB_HID_DESC_OFFSET 41

#endif /* __USB_DESCRIPTOR_H__ */

