/**
 * HID keyboard
 * Jason (Hyun-joon Kim)
 */

#ifndef __KEYUBOARD_H__
#define __KEYUBOARD_H__ 

#include <stdbool.h>
#include <stdint.h>

void hid_report_key(uint8_t key);
void init_keyboard(void);

#endif /* __KEYUBOARD_H__  */


