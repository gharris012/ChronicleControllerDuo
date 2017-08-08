
#include "Particle.h"
#include "ble_duo.h"

/**
 * @brief Callback for scanning device.
 *
 * @param[in]  *report
 *
 * Advertising Event Type:
 *     - (0x00)BLE_GAP_ADV_TYPE_ADV_IND          : Connectable undirected.
 *     - (0x01)BLE_GAP_ADV_TYPE_ADV_DIRECT_IND   : Connectable directed.
 *     - (0x02)BLE_GAP_ADV_TYPE_ADV_SCAN_IND     : Scannable undirected.
 *     - (0x03)BLE_GAP_ADV_TYPE_ADV_NONCONN_IND  : Non connectable undirected.
 *     - (0x04)BLE_GAP_ADV_TYPE_SCAN_RSP         : Scan response.
 */
void reportCallback(advertisementReport_t *report) {
  uint8_t index;

  // Addresses
  // Orange: 80 101 131 109 168 63
  // Purple: 80 101 131 109 159 102

/*
    Tilt UUIDs by color
    red    = 'a495bb10c5b14b44b5121370f02d74de'
    green  = 'a495bb20c5b14b44b5121370f02d74de'
    black  = 'a495bb30c5b14b44b5121370f02d74de'
    purple = 'a495bb40c5b14b44b5121370f02d74de'
    orange = 'a495bb50c5b14b44b5121370f02d74de'
    blue   = 'a495bb60c5b14b44b5121370f02d74de'
    yellow = 'a495bb70c5b14b44b5121370f02d74de'
    pink   = 'a495bb80c5b14b44b5121370f02d74de'
*/

  if ( report->advEventType == 0
        && report->peerAddr[0] == 80
        && report->peerAddr[1] == 101
        && report->peerAddr[2] == 131
        && report->peerAddr[3] == 109
        && ( report->advData[9] == 80
            || report->advData[9] == 64
            )
        )
  {
    Serial.println();
    Serial.println("reportCallback: =====");
    Serial.print("The advEventType: ");
    Serial.println(report->advEventType, HEX);
    Serial.print("The peerAddrType: ");
    Serial.println(report->peerAddrType, HEX);

    Serial.print("The peerAddr: ");
    for (index = 0; index < 6; index++) {
      Serial.print(report->peerAddr[index]);
      Serial.print(" ");
    }
    Serial.println(" ");

    Serial.print("The rssi: ");
    Serial.println(report->rssi, DEC);

    // get tilt indicator
    index = 9;
    Serial.print(index);
    Serial.print(": ");
    Serial.print(report->advData[index], HEX);
    Serial.print(" (");
    Serial.print(report->advData[index]);
    Serial.println(")");

    for (index = 22; index < report->advDataLen; index++) {
      Serial.print(index);
      Serial.print(": ");
      Serial.print(report->advData[index], HEX);
      Serial.print(" (");
      Serial.print(report->advData[index]);
      Serial.println(")");
    }
    Serial.println(" ");

    uint16_t tempF = ( report->advData[22] << 8 ) + report->advData[23];
    uint16_t gravity = ( report->advData[24] << 8 ) + report->advData[25];

    if ( report->advData[9] == 80 ) // Orange: 0x50
    {
        Serial.println("Tilt: Orange");
    }
    else if ( report->advData[9] == 64 ) // Purple: 0x40
    {
        Serial.println("Tilt: Purple");
    }
    else
    {
        Serial.println("Tilt: unknown");
    }

    Serial.print("Temp: ");
    Serial.println(tempF);

    Serial.print("Gravity: ");
    Serial.println(gravity);

    Serial.println("=====");
  }
}

/**
 * @brief Setup.
 */
void ble_scanner_setup()
{
  // Open debugger, must befor init().
  //ble.debugLogger(true);
  //ble.debugError(true);
  //ble.enablePacketLogger();

  Serial.println("Setting up BLE Central scanner!");
  // Initialize ble_stack.
  ble.init();

  // Register callback functions.
  ble.onScanReportCallback(reportCallback);

  // Set scan parameters.
  ble.setScanParams(BLE_SCAN_TYPE, BLE_SCAN_INTERVAL, BLE_SCAN_WINDOW);

  // Start scanning.
  ble.startScanning();
  Serial.println("Start scanning ");
}
