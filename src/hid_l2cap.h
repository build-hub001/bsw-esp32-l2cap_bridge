#ifndef _HID_L2CAP_H_
#define _HID_L2CAP_H_

#include "stack/bt_types.h"

#include <esp_bt_main.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>

typedef enum BT_STATUS {
  BT_UNINITIALIZED,
  BT_DISCONNECTED,
  BT_CONNECTING,
  BT_CONNECTED
}enBT_STATUS;

#define HID_L2CAP_MESSAGE_SIZE  8
typedef void (*HID_L2CAP_CALLBACK)(uint8_t *p_msg, size_t length); 

long hid_l2cap_initialize(HID_L2CAP_CALLBACK callback);
long hid_l2cap_connect(BD_ADDR addr);
long hid_l2cap_reconnect(void);
enBT_STATUS hid_l2cap_get_status(void);

#endif
