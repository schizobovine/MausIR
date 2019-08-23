//
// bleumaus.ino
//
// Modified mouse robot toy for my cats. Now with Bluetooth!
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv3.0
//

#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

//
// Command structure: 5 bytes total, 4 data + 1 checksum
//
// byte 0 => '!'
// byte 1 => 'B' (for button)
// byte 2 => '1' (buttons 1-4, up=5, down=6, left=7, right=8)
// byte 3 => '1' (pressed) or '0' (released)
// byte 4 => checksum
//

#define BLE_UART_NAME  ("BleuMaus")
#define DEBUG_SERIAL   (1)

#define CMD_BUFF_SZ       (5)
#define CMD_BYTE_HEADER   (0) // '!'
#define CMD_BYTE_TYPE     (1) // B:button, L:GPS, Q:quaternion, M:mag, A:acc
#define CMD_BYTE_BUTTON   (2)
#define CMD_BYTE_PRESSED  (3)
#define CMD_BYTE_CHECKSUM (4)

#define BUTTON_UP         ('5')
#define BUTTON_DOWN       ('6')
#define BUTTON_LEFT       ('7')
#define BUTTON_RIGHT      ('8')

#define PIN_AIN1          (4)
#define PIN_AIN2          (5)
#define PIN_BIN1          (3)
#define PIN_BIN2          (2)
#define PIN_SLEEP         (28)
#define PIN_FAULT         (29)

// Bluetooth LE service objects
BLEDfu  ble_dfu;  // Over-the-air (OTA) Device Firmware Update (DFU)
BLEDis  ble_dis;  // Device Information Service
BLEUart ble_uart; // UART over BLE
BLEBas  ble_bas;  // BLE battery service

// Command buffer
byte cmd[CMD_BUFF_SZ];

#if (DEBUG_SERIAL)
#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif

//
// checksum algorithm: sum bytes in message, then invert. e.g., '!B51':
//
// 33 + 66 + 53 + 49 => 201 => 0b11001001 => 0b00110110 => 54 => '6'
//
bool verify_checksum() {
    uint8_t counter = 0;
    for (int i=0; i<CMD_BUFF_SZ-1; i++) {
      counter += cmd[i];
    }
    counter = ~counter;
    return (counter == cmd[CMD_BYTE_CHECKSUM]);
}

//
// Dispatch button presses to movement commands
//
void dispatch(char button, char pressed) {
  bool released = false;

  if (pressed == '0') {
    released = true;
  }

  switch (button) {

    case BUTTON_UP:
      DPRINTLN(F("^"));
      break;

    case BUTTON_DOWN:
      DPRINTLN(F("v"));
      break;

    case BUTTON_LEFT:
      DPRINTLN(F("<"));
      break;

    case BUTTON_RIGHT:
      DPRINTLN(F(">"));
      break;

    default:
      DPRINTLN(F("WAT"));
      break;
  }
}

/**
 * Callback invoked when we get a BLE connection.
 * @param conn_handle Connection where event happens.
 */
void connect_callback(uint16_t conn_handle) {
  BLEConnection *con = Bluefruit.Connection(conn_handle);

  // Print out name of central that connected to us
  char central_name[32] = { 0 };
  con->getPeerName(central_name, sizeof(central_name));
  DPRINT(F("Connected to "));
  DPRINTLN(central_name);
}

/**
 * Callback invoked when connection is dropped.
 * @param conn_handle Connection where event happens.
 * @param reason BLE_HCI_STATUS_CODE found in ble_hci.h.
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  DPRINTLN("Disconnected");
}

void setup() {
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);

    // Setup serial connection, hopefully this doesn't stall?
#if (DEBUG_SERIAL)
    Serial.begin(115200);
    while (!Serial) {
      delay(1);
    }
#endif

    DPRINTLN(F("BlueMaus"));
    DPRINTLN(F("ohai"));

    // Initialize motor control pins
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    pinMode(PIN_SLEEP, OUTPUT);
    pinMode(PIN_FAULT, INPUT_PULLUP);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_SLEEP, LOW);

    // das blinkenlights off until connect
    Bluefruit.autoConnLed(true);

    // Config with max bandwidth (at the expense of SoftDevice SRAM)
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

    // NB: All configXXX() functions should be called before here!!
    Bluefruit.begin();
    DPRINTLN(F("Bluefruit: begun"));

    // More basic setup stuff
    Bluefruit.setTxPower(4); // magic?!?!
    Bluefruit.setName(BLE_UART_NAME);
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    DPRINTLN(F("Bluefruit: callbacks set"));

    // Make sure to execute any incoming OTA DFUs
    ble_dfu.begin();
    DPRINTLN(F("Bluefruit: DFU (maybe) done"));

    // Setup DIS
    ble_dis.setManufacturer("Your Mama");
    ble_dis.setModel("BleuMaus v1.0");
    ble_dis.begin();
    DPRINTLN(F("Bluefruit: DIS online"));

    // Setup & start UART service
    ble_uart.begin();
    DPRINTLN(F("Bluefruit: UART started"));

    // Start BLE battery service
    ble_bas.begin();
    ble_bas.write(100);
    DPRINTLN(F("Bluefruit: Battery service started"));
    
    // Setup advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(ble_uart);

    // Name is only sent after discovery because BLE lol
    Bluefruit.ScanResponse.addName();

    // Start advertizing
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244); // in units of 0.625ms
    Bluefruit.Advertising.setFastTimeout(30); // number of seconds in fast mode
    Bluefruit.Advertising.start(0); // never stop advertising

    // Finally
    DPRINTLN(F("setup() done"));

}

void loop() {
  
    // If a command is available, read it, verify the checksum and then
    // dispatch if it passes.
    if (ble_uart.available() >= CMD_BUFF_SZ) {
      for (int i=0; i<CMD_BUFF_SZ; i++) {
        cmd[i] = ble_uart.read();
      }
      if (verify_checksum()) {
        dispatch(cmd[CMD_BYTE_BUTTON], cmd[CMD_BYTE_PRESSED]);
      }
    }

}

// vi: syntax=arduino
