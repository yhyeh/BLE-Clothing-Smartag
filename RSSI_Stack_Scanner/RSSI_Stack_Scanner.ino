/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <bluefruit.h>

/* For a list of EIR data types see:
 *    https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
 *    Matching enum: cores/nRF5/SDK/components/softdevice/s132/headers/ble_gap.h */
int received_cnt = 0;
int first_seq_num;

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  //Serial.println("Bluefruit52 Central ADV Scan Example");
  //Serial.println("------------------------------------\n");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  /* Set the device name */
  Bluefruit.setName("Bluefruit52 Central");

  /* Set the LED interval for blinky pattern on BLUE LED */
  Bluefruit.setConnLedInterval(250);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Filter out packet with a min rssi
   * - Interval = 100 ms, window = 50 ms
   * - Use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  //Bluefruit.Scanner.filterRssi(-80);
  //Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE); // only invoke callback if detect bleuart service
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

  //Serial.println("Scanning ...");
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  PRINT_LOCATION();
  uint8_t len = 0;
  uint8_t buffer[32];
  char addr[18];
  memset(buffer, 0, sizeof(buffer));
  //uint8_t TXRX_Mask = 0xf0;
  //uint8_t ITEMADV_Mask = 0x40;

  // MAC is in little endian --> print reverse

  snprintf(addr, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
    report->peer_addr.addr[5],
    report->peer_addr.addr[4],
    report->peer_addr.addr[3],
    report->peer_addr.addr[2],
    report->peer_addr.addr[1],
    report->peer_addr.addr[0]
  );

  if( Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer))
    && memcmp(buffer, "BFItem", 6) == 0){
  /* Complete Local Name */

    Serial.printf("%s, ", buffer);
    memset(buffer, 0, sizeof(buffer));

    // count received pkt
    received_cnt++;
    Serial.printf("%d, ", received_cnt);

    // timestamp
    Serial.printf("%d, ", millis());

    Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
    Serial.print(", ");

    /* RSSI value */
    Serial.printf("%d, ", report->rssi);


    /* TX Power Level */
    if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, buffer, sizeof(buffer)))
    {
      if(buffer[0] > 120){
        Serial.printf("%i, ", buffer[0]-256);
      }
      else{
        Serial.printf("%i, ", buffer[0]);
      }
      memset(buffer, 0, sizeof(buffer));
    }

    /* channel index*/
    Serial.printf("%d", report->ch_index);

    Serial.println();
    // connect
    //Bluefruit.Central.connect(report);
  }
  else{
    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    //Serial.println("Get other device");
  }
  Bluefruit.Scanner.resume();
}

void connect_callback(uint16_t conn_handle)
{

}

void printUuid16List(uint8_t* buffer, uint8_t len)
{
  Serial.printf("%14s %s", "16-Bit UUID");
  for(int i=0; i<len; i+=2)
  {
    uint16_t uuid16;
    memcpy(&uuid16, buffer+i, 2);
    Serial.printf("%04X ", uuid16);
  }
  Serial.println();
}

void printUuid128List(uint8_t* buffer, uint8_t len)
{
  (void) len;
  Serial.printf("%14s %s", "128-Bit UUID");

  // Print reversed order
  for(int i=0; i<16; i++)
  {
    const char* fm = (i==4 || i==6 || i==8 || i==10) ? "-%02X" : "%02X";
    Serial.printf(fm, buffer[15-i]);
  }

  Serial.println();
}

void loop()
{
  // nothing to do
}
