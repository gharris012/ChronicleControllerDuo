#ifndef BLE_DUO_H
#define BLE_DUO_H

/*
 * BLE scan parameters:
 *     - BLE_SCAN_TYPE
 *           0x00: Passive scanning, no scan request packets shall be sent.(default)
 *           0x01: Active scanning, scan request packets may be sent.
 *           0x02 - 0xFF: Reserved for future use.
 *     - BLE_SCAN_INTERVAL: This is defined as the time interval from when the Controller started its last LE scan until it begins the subsequent LE scan.
 *           Range: 0x0004 to 0x4000
 *           Default: 0x0010 (10 ms)
 *           Time = N * 0.625 msec
 *           Time Range: 2.5 msec to 10.24 seconds
 *     - BLE_SCAN_WINDOW: The duration of the LE scan. The scan window shall be less than or equal to the scan interval.
 *           Range: 0x0004 to 0x4000
 *           Default: 0x0010 (10 ms)
 *           Time = N * 0.625 msec
 *           Time Range: 2.5 msec to 10240 msec
 */

#define BLE_SCAN_TYPE        0x00   // Passive scanning
#define BLE_SCAN_INTERVAL    0x0060 // 60 ms
#define BLE_SCAN_WINDOW      0x0030 // 30 ms

/******************************************************
 *                      Type Define
 ******************************************************/
typedef struct {
  uint16_t  connected_handle;
  uint8_t   addr_type;
  bd_addr_t addr;
  struct {
    gatt_client_service_t service;
    struct {
      gatt_client_characteristic_t chars;
      gatt_client_characteristic_descriptor_t descriptor[2]; // User_descriptor and client charactersitc configuration descriptor.
    } chars[2];
  } service; // Service contains two characteristics and each characteristic contains two descriptors.
} Device_t;

uint32_t ble_advdata_decode(uint8_t type, uint8_t advdata_len, uint8_t *p_advdata, uint8_t *len, uint8_t *p_field_data);
void reportCallback(advertisementReport_t *report);
void ble_scanner_setup();

#endif
