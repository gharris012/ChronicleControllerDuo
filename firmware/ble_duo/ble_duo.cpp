
#include "Particle.h"
#include "ble_duo.h"

ble_report_callback_t ble_report_callback;
Logger *bleLogger;


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

    if ( report->advEventType == 0 )
    {
        bleLogger->info("Callback from %#04X %#04X %#04X %#04X %#04X %#04X (%#04X) at %d",
                    report->peerAddr[0], report->peerAddr[1], report->peerAddr[2],
                    report->peerAddr[3], report->peerAddr[4], report->peerAddr[5],
                    report->peerAddrType, report->rssi);

        // look for Tilt UUID
        if ( report->advData[6] == 0xA4
                && report->advData[7] == 0x95
                && report->advData[8] == 0xBB
                && report->advData[10] == 0xC5
                && report->advData[11] == 0xB1
                && report->advData[12] == 0x4B
                && report->advData[13] == 0x44
                && report->advData[14] == 0xB5
                && report->advData[15] == 0x12
                && report->advData[16] == 0x13
                && report->advData[17] == 0x70
                && report->advData[18] == 0xF0
                && report->advData[19] == 0x2D
                && report->advData[20] == 0x74
                && report->advData[21] == 0xDE
           )
        {
            if ( report->advData[9] == 0x50 ) // Orange: 0x50
            {
                bleLogger->info("Tilt: Orange");
            }
            else if ( report->advData[9] == 0x40 ) // Purple: 0x40
            {
                bleLogger->info("Tilt: Purple");
            }
            else
            {
                bleLogger->warn("Tilt: unknown (%#04X)", report->advData[9]);
            }
            bleLogger->trace(" Raw data: %#04X %#04X %#04X %#04X", report->advData[22], report->advData[23],
                         report->advData[24], report->advData[25]);

            short tempF = ( report->advData[22] << 8 ) + report->advData[23];
            short gravity = ( report->advData[24] << 8 ) + report->advData[25];
            bleLogger->info("Temp: %d ; Gravity: %d", tempF, gravity);

            ble_report_callback(report->advData[9], tempF, gravity);
        }
    }
}

void ble_scanner_setup(ble_report_callback_t callback, Logger *logger)
{
    ble_report_callback = callback;
    bleLogger = logger;
    bleLogger->info("Setting up BLE Central scanner!");
    // Initialize ble_stack.
    ble.init();

    // Register callback functions.
    ble.onScanReportCallback(reportCallback);

    // Set scan parameters.
    ble.setScanParams(BLE_SCAN_TYPE, BLE_SCAN_INTERVAL, BLE_SCAN_WINDOW);

    // Start scanning.
    ble.startScanning();
}
