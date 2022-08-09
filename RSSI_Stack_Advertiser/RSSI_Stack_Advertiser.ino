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

/* This sketch demonstrates the Bluefruit.Advertising API(). When powered up,
 * the Bluefruit module will start advertising for ADV_TIMEOUT seconds (by
 * default 30 seconds in fast mode, the remaining time slow mode) and then
 * stop advertising completely. The module will start advertising again if
 * PIN_ADV is grounded.
 */
#include <bluefruit.h>
#include <stdio.h>


#define PIN_ADV       A0
#define BUTTON        7
#define ADV_TIMEOUT   0 // seconds
uint8_t AND_MASK = 0x00;

//////////////////////////////
uint8_t OR_MASK = 0x00;
//////////////////////////////


//int adv_interval_idx = 1;
int g_seq_num = 0;

void setup()
{
  // configure PIN_ADV as input with a pullup (pin is active low)
  pinMode(PIN_ADV, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);
  //pinMode(BUTTON, INPUT);

  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Advanced Advertising Example");
  Serial.println("----------------------------------------\n");

  Bluefruit.begin();

  //////////////////////////////
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  //////////////////////////////

  //Bluefruit.setName("Bluefruit Item");
  ble_gap_addr_t defaultAddr = Bluefruit.getAddr();
  Serial.printBufferReverse(defaultAddr.addr, 6, ':');
  Serial.println();

  ble_gap_addr_t newAddr;
  memcpy(&newAddr, &defaultAddr, sizeof(ble_gap_addr_t));
  newAddr.addr[0] = (defaultAddr.addr[0] & AND_MASK) | OR_MASK;
  Bluefruit.setAddr(&newAddr);
  Serial.printBufferReverse(Bluefruit.getAddr().addr, 6, ':');
  Serial.println();

  // Set up and start advertising
  digitalWrite(LED_RED, HIGH);
  /*
  Serial.println("Advertising will start in 30 sec");
  for(int i = 0; i < 10; i++){
    digitalToggle(LED_RED);
    delay(1000);
  }
  */
  //Serial.println("Advertising is started");
  Serial.printf("seq_num: %d\n", g_seq_num);
  startAdv(g_seq_num);
}

void startAdv(int seq_num)
{
  char newName[25];
  sprintf(newName, "BFItem %9d", seq_num);
  Bluefruit.setName((char const *)newName);
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
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
  Bluefruit.Advertising.setInterval(3198, 3199);    // in units of 0.625 ms
  //Bluefruit.Advertising.setIntervalMS(998, 999);    // in units of 1     ms
  //Serial.printf("Interval: %d\n", Bluefruit.Advertising.getInterval());
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
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
  delay(1000);

  //Serial.println("Advertising is started");
  Serial.printf("seq_num: %d\n", ++g_seq_num);
  digitalWrite(LED_RED, HIGH);
  startAdv(g_seq_num);

  //Serial.println("Press btn to start.");
}
void loop()
{
}
