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

ble_gap_addr_t defaultAddr;
char catalog[20][20];
int catalog_len = 0;
/* For a list of EIR data types see:
 *    https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
 *    Matching enum: cores/nRF5/SDK/components/softdevice/s132/headers/ble_gap.h */

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("User - Central");
  Serial.println("------------------------------------\n");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  defaultAddr = Bluefruit.getAddr();
  Serial.printBufferReverse(defaultAddr.addr, 6, ':');
  Serial.println();
  /* Set the device name */
  //Bluefruit.setName("Bluefruit52 Central");

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
  Bluefruit.Scanner.filterRssi(-25);
  //Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE); // only invoke callback if detect bleuart service
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

  Serial.println("Scanning ...");
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  PRINT_LOCATION();
  uint8_t len = 0;
  uint8_t buffer[32];
  char addr[18];
  memset(buffer, 0, sizeof(buffer));
  uint8_t TXRX_Mask = 0x0f;
  uint8_t CENTADV_Mask = 0x00;

  // MAC is in little endian --> print reverse

  snprintf(addr, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
    report->peer_addr.addr[5],
    report->peer_addr.addr[4],
    report->peer_addr.addr[3],
    report->peer_addr.addr[2],
    report->peer_addr.addr[1],
    report->peer_addr.addr[0]
  );


  /* Complete Local Name */
  if( Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer))
      && memcmp(buffer, "BFItem", 6) == 0){
    if( (report->peer_addr.addr[5] & TXRX_Mask) == CENTADV_Mask )
    {
      char name[14];
      char addr_str[13];
      char addr_str_split[18];
      memcpy(name, buffer, 13);
      name[13] = '\0';
      memcpy(&addr_str_split[0], &buffer[14], 2);
      memcpy(&addr_str_split[3], &buffer[16], 2);
      memcpy(&addr_str_split[6], &buffer[18], 2);
      memcpy(&addr_str_split[9], &buffer[20], 2);
      memcpy(&addr_str_split[12], &buffer[22], 2);
      memcpy(&addr_str_split[15], &buffer[24], 2);
      for(int i=2; i < 17; i+=3){
        addr_str_split[i] = ':';
      }
      addr_str_split[17] =  '\0';

      //Serial.printf("%s, ", name);
      //Serial.printf("%s\n", addr_str_split);

      memset(buffer, 0, sizeof(buffer));
      int addAddrFlag = 0;
      int newAdded = 0;
      for(int i=0; i<catalog_len; i++){
        if(memcmp(&addr_str_split[3], &catalog[i][3], 15)==0){
          addAddrFlag = 1;
          break;
        }
      }
      if (addAddrFlag == 0) {
        memcpy(catalog[catalog_len], addr_str_split, 18);
        catalog_len++;
        addAddrFlag = 1;
        newAdded = 1;

      }

      Serial.println("Catalog (item addr) ======================================");
      for(int i=0; i<catalog_len; i++){
        Serial.printf("%s", catalog[i]);
        Serial.println();
      }
      Serial.println("==========================================================");

      Serial.println();
      // Target: orange illini T-shirt
      if (newAdded == 1 && memcmp(&addr_str_split[3], "50:21:CF:01", 11)==0){
        Serial.printf("****************************************************\n");
        Serial.printf("****************************************************\n");
        Serial.printf("Here is a T-shirt you might like! BOGO only today!\n");
        Serial.printf("****************************************************\n");
        Serial.printf("****************************************************\n");
        Serial.println();

      }
      // timestamp
      //Serial.printf("%d, ", millis());

      //Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
      //Serial.print(", ");

      /* Raw buffer contents *//*
      Serial.printf("%14s %d bytes\n", "PAYLOAD", report->data.len);
      if (report->data.len)
      {
        Serial.printf("%15s", " ");
        Serial.printBuffer(report->data.p_data, report->data.len, '-');
        Serial.println();
      }
      */
      /* RSSI value */
      //Serial.printf("%d, ", report->rssi);

      /* Adv Type *//*
      Serial.printf("%14s ", "ADV TYPE");
      if ( report->type.connectable )
      {
        Serial.print("Connectable ");
      }else
      {
        Serial.print("Non-connectable ");
      }

      if ( report->type.directed )
      {
        Serial.println("directed");
      }else
      {
        Serial.println("undirected");
      }
      */
      /* Shortened Local Name *//*
      if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer)))
      {
        Serial.printf("%14s %s\n", "SHORT NAME", buffer);
        memset(buffer, 0, sizeof(buffer));
      }
      */


      /* TX Power Level */
      if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, buffer, sizeof(buffer)))
      {
        Serial.printf("%i,", buffer[0]);
        memset(buffer, 0, sizeof(buffer));
      }

      /* channel index*/
      //Serial.printf("%d,", report->ch_index);

      /* Apperence */
      /*
      if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_APPEARANCE, buffer, sizeof(buffer)))
      {
        Serial.printf("%d%d", buffer[0], buffer[1]);
        memset(buffer, 0, sizeof(buffer));
      }
      */
      /* Check for UUID16 Complete List *//*
      len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, buffer, sizeof(buffer));
      if ( len )
      {
        printUuid16List(buffer, len);
      }
      */
      /* Check for UUID16 More Available List *//*
      len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE, buffer, sizeof(buffer));
      if ( len )
      {
        printUuid16List(buffer, len);
      }
      */
      /* Check for UUID128 Complete List *//*
      len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, buffer, sizeof(buffer));
      if ( len )
      {
        printUuid128List(buffer, len);
      }
      */
      /* Check for UUID128 More Available List *//*
      len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE, buffer, sizeof(buffer));
      if ( len )
      {
        printUuid128List(buffer, len);
      }
      */
      /* Check for BLE UART UUID *//*
      if ( Bluefruit.Scanner.checkReportForUuid(report, BLEUART_UUID_SERVICE) )
      {
        Serial.printf("%14s %s\n", "BLE UART", "UUID Found!");
      }
      */
      /* Check for DIS UUID *//*
      if ( Bluefruit.Scanner.checkReportForUuid(report, UUID16_SVC_DEVICE_INFORMATION) )
      {
        Serial.printf("%14s %s\n", "DIS", "UUID Found!");
      }
      */
      /* Check for Manufacturer Specific Data *//*
      len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
      if (len)
      {
        Serial.printf("%14s ", "MAN SPEC DATA");
        Serial.printBuffer(buffer, len, '-');
        Serial.println();
        memset(buffer, 0, sizeof(buffer));
      }
      */
      Serial.println();
      // connect
      //Bluefruit.Central.connect(report);
    }
  }
  else{
    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    //Serial.println("Get other device");
  }
  Bluefruit.Scanner.resume();
}

void loop()
{
  // nothing to do
}
