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

#define ADV_TIMEOUT 2
int g_seq_num = 0;
ble_gap_addr_t defaultAddr;
ble_gap_addr_t newAddr;

/* For a list of EIR data types see:
 *    https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
 *    Matching enum: cores/nRF5/SDK/components/softdevice/s132/headers/ble_gap.h */
struct itemRecord{
  uint8_t addr[6];
  uint8_t IDlen;
  uint8_t IDlist[256];
};
itemRecord itemBook[50];
int bookLen = 0;
int advFlag[50];
int firstScanFlag = 1;

int controllerID = 0; // Tshirt
//int controllerID = 1; // Shorts

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Controller - Dual");
  Serial.println("------------------------------------\n");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  defaultAddr = Bluefruit.getAddr();
  Serial.printBufferReverse(defaultAddr.addr, 6, ':');
  Serial.println();
  memcpy(&newAddr, &defaultAddr, sizeof(ble_gap_addr_t));
  // init Addr to zero X0:00:00:00:00:00
  for(int i=0; i<5; i++){
    newAddr.addr[i] = defaultAddr.addr[i] & 0x00;
  }
  newAddr.addr[5] = defaultAddr.addr[5] & 0xf0;
  Bluefruit.setAddr(&newAddr);
  Serial.printBufferReverse(Bluefruit.getAddr().addr, 6, ':');
  Serial.println();

  for(int i=0; i<50; i++){
    itemBook[i].IDlen = 0;
    advFlag[i] = 0;
  }
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
  //Bluefruit.Scanner.filterRssi(-80);
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
  uint8_t ITEMADV_Mask = (uint8_t) controllerID << 2 | 0x01;
  int addAddrFlag = 0;

  // MAC is in little endian --> print reverse
  /*
  snprintf(addr, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
    report->peer_addr.addr[5],
    report->peer_addr.addr[4],
    report->peer_addr.addr[3],
    report->peer_addr.addr[2],
    report->peer_addr.addr[1],
    report->peer_addr.addr[0]
  );
  */

  /* Complete Local Name */
  if( Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer))
      && memcmp(buffer, "BFItem", 6) == 0){
    if( (report->peer_addr.addr[5] & TXRX_Mask) == ITEMADV_Mask )
    {
      //Serial.printf("%s, ", buffer);
      memset(buffer, 0, sizeof(buffer));

      // timestamp
      //Serial.printf("%d, ", millis());


      uint8_t itemAddr[6];
      memcpy(itemAddr, report->peer_addr.addr, 6);
      //Serial.printBufferReverse(itemAddr, 6, ':');
      //Serial.printf(", ");
      itemAddr[5] = itemAddr[5] & 0x0f;
      //itemAddr[0] = itemAddr[0] & 0x00;

      /* RSSI value */
      //Serial.printf("%d, ", report->rssi);
      /* TX Power Level */
      if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, buffer, sizeof(buffer)))
      {
        //Serial.printf("%i,", buffer[0]);
        memset(buffer, 0, sizeof(buffer));
      }

      /* channel index*/
      //Serial.printf("%d,", report->ch_index);
      //Serial.println();

      for(int i=0; i<bookLen; i++){
        if(memcmp(&itemAddr[1], &itemBook[i].addr[1], 5)==0){
          for(int j=0; j<itemBook[i].IDlen; j++){
            if(itemBook[i].IDlist[j] == itemAddr[0]){
              addAddrFlag = 1;
              break;
            }
          }
          if (addAddrFlag == 0){
            itemBook[i].IDlist[itemBook[i].IDlen] = itemAddr[0];
            itemBook[i].IDlen++;
            addAddrFlag = 1;
            break;
          }
        }
      }
      if (addAddrFlag == 0) {
        memcpy(itemBook[bookLen].addr, itemAddr, 6);
        itemBook[bookLen].IDlist[itemBook[bookLen].IDlen] = itemAddr[0];
        itemBook[bookLen].IDlen++;
        bookLen++;
        addAddrFlag = 1;
      }

      Serial.println("Catalog (item addr, # in stock) ==========================");
      for(int i=0; i<bookLen; i++){
        Serial.printBufferReverse(itemBook[i].addr, 6, ':');
        Serial.printf(", %d", itemBook[i].IDlen);
        Serial.println();
      }
      Serial.println("==========================================================");

      Serial.println();

      if (firstScanFlag == 1 && bookLen > 0){
        digitalWrite(LED_RED, HIGH);
        Serial.printf("Advertise to user!\n");
        startAdv(g_seq_num);
        firstScanFlag = 0;
      }
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

void startAdv(int seq_num)
{
  int advFinish = 0;
  for(int i=0; i<bookLen; i++){
    if(advFlag[i] == 0){
      newAddr.addr[0] = itemBook[i].IDlen;
      memcpy(&newAddr.addr[1], &itemBook[i].addr[1], 4);
      advFlag[i] = 1;
      advFinish = 1;
      break;
    }
  }
  if (advFinish == 0){
    for(int i=0; i<bookLen; i++){
      advFlag[i] = 0;
    }
    delay(3000);
    newAddr.addr[0] = itemBook[0].IDlen;
    memcpy(&newAddr.addr[1], &itemBook[0].addr[1], 4);
    advFlag[0] = 1;
    advFinish = 1;
  }

  char newName[25];
  sprintf(newName, "BFItem%7d,%02X%02X%02X%02X%02X%02X", seq_num, newAddr.addr[5],newAddr.addr[4],newAddr.addr[3],newAddr.addr[2],newAddr.addr[1],newAddr.addr[0]);
  Bluefruit.setName((char const *)newName);
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  //Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  //if(Bluefruit.Advertising.addName()) Serial.println("addName Success");
  //Bluefruit.ScanResponse.addFlags(dist);
  //if(Bluefruit.Advertising.addAppearance((uint16_t) seq_num)) Serial.println("addAppearence success");
  /*
  uint8_t* data_addr = Bluefruit.Advertising.getData();
  uint8_t  data_len = Bluefruit.Advertising.count();
  uint8_t* newDataBuf = (uint8_t*) malloc(data_len);
  memcpy(newDataBuf, data_addr, data_len);
  */



  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(244, 244);    // in units of 0.625 ms
  //Bluefruit.Advertising.setIntervalMS(998, 999);    // in units of 1     ms
  //Serial.printf("Interval: %d\n", Bluefruit.Advertising.getInterval());
  Bluefruit.Advertising.setFastTimeout(1);      // number of seconds in fast mode
  Bluefruit.Advertising.start(ADV_TIMEOUT);
  //if(Bluefruit.Advertising.start(ADV_TIMEOUT)) Serial.println("start success");      // Stop advertising entirely after ADV_TIMEOUT seconds
}

/**
 * Callback invoked when advertising is stopped by timeout
 */
void adv_stop_callback(void)
{
  //Serial.println("Advertising stop.");
  digitalWrite(LED_RED, LOW);
  /*
  for(int i = 0; i < 60; i++){
    digitalToggle(LED_RED);
    delay(1000);
  }
  */
  Bluefruit.Advertising.clearData();
  //delay(5000);

  //Serial.println("Advertising is started");
  Serial.printf("Advertise to user!\n");
  digitalWrite(LED_RED, HIGH);
  startAdv(g_seq_num);

  //Serial.println("Press btn to start.");
}

void loop()
{
  // nothing to do
}
