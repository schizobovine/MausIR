//
// bleumaus_controller.ino
//
// Controller for modified mouse robot toy for my cats. Now with Bluetooth!
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_seesaw.h>
#include <bluefruit.h>

#define DEBUG           (true)

#if (DEBUG)
#define DPRINT(...)   do { if (Serial) { Serial.print(__VA_ARGS__);   } } while (0)
#define DPRINTLN(...) do { if (Serial) { Serial.println(__VA_ARGS__); } } while (0)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif

#define BUTTON_RIGHT    (1 << 6)
#define BUTTON_DOWN     (1 << 7)
#define BUTTON_LEFT     (1 << 9)
#define BUTTON_UP       (1 << 10)
#define BUTTON_SEL      (1 << 14)

const uint32_t BUTTON_MASK = 
    BUTTON_RIGHT
  | BUTTON_DOWN
  | BUTTON_LEFT
  | BUTTON_UP
  | BUTTON_SEL
  ;

#define PIN_SEESAW_IRQ  13
#define I2C_ADDR_SEESAW 0x49

#define MIN_X_DELTA 3
#define MIN_Y_DELTA 3

#define HAS_X_MOVED (abs((x) - (last_x)) > MIN_X_DELTA)
#define HAS_Y_MOVED (abs((y) - (last_y)) > MIN_Y_DELTA)

//
// Global objects
//

Adafruit_seesaw joy;
BLEClientUart bleuart;

uint32_t butts = 0; //store buttons pressed in irq handler
int last_x = 0;
int last_y = 0;
int x = 0;
int y = 0;

void bleuart_rx_callback() {
}

/**
 * Invoked when a connection is established.
 * @param handle Connection handle
 */
void connect_callback(uint16_t handle) {
  DPRINTLN(F("Discovering UART..."));
  if (bleuart.discover(handle)) {
    DPRINTLN(F("Found UART, enabling TXD notify"));
    bleuart.enableTXD();
    DPRINTLN(F("Ready to receive!"));
  } else {
    DPRINTLN(F("UART not discovered, disconnecting"));
    Bluefruit.disconnect(handle);
  }
}

/**
 * Invoked when a connection is dropped.
 * @param handle Connection handle
 * @param reason BLE_HCI_STATUS_CODE found in ble_hci.h
 */
void disconnect_callback(uint16_t handle, uint8_t reason) {
  DPRINTLN(F("Disconnected"));
}

/**
 * Invoked when scanner picks up advertising data.
 * @param report Structural advertising data.
 */
void scan_callback(ble_gap_evt_adv_report_t *report) {
  if (Bluefruit.Scanner.checkReportForService(report, bleuart)) {
    DPRINTLN(F("BLE UART service detected, connecting..."));
    Bluefruit.Central.connect(report);
  } else {
    Bluefruit.Scanner.resume();
  }
}

void setup() {

#if (DEBUG)
    Serial.begin(115200);
    while (!Serial) delay(1);
#endif

    // Connect to joystick controller
    Wire.begin();
    if (!joy.begin(I2C_ADDR_SEESAW)) {
      DPRINTLN(F("Seesaw not found?"));
    } else {
      DPRINTLN(F("Seesaw found, version: "));
      DPRINTLN(joy.getVersion());
      joy.pinModeBulk(BUTTON_MASK, INPUT_PULLUP);
      joy.setGPIOInterrupts(BUTTON_MASK, 1);
    }
    pinMode(PIN_SEESAW_IRQ, INPUT);

    // Initialize BLE in Central mode
    Bluefruit.begin(0, 1);
    Bluefruit.setName("BleuMaus Controller");
    Bluefruit.setConnLedInterval(250);

    // Fire up the UART over BLE
    bleuart.beign();
    bleuart.setRxCallback(bleuart_rx_callback);

    // Scan for da maus and try to connect
    Bluefruit.Scanner.setRxCallback(scan_callback);
    Bluefruit.Scanner.restartOnDisconnect(true);
    Bluefruit.Scanner.setInterval(160, 80);
    Bluefruit.Scanner.useActiveScan(false);
    Bluefruit.Scanner.start(0);

    // Attach interrupt handler to joystick IRQ line
    //attachInterrupt(PIN_SEESAW_IRQ, joy_irq, CHANGE);

    // Set LED phasers to GLOW (wat)
    //pinMode(PIN_LED, OUTPUT);
    //digitalWrite(PIN_LED, HIGH);

    Serial.println("setup() done");

}

void loop() {

  if (!digitalRead(PIN_SEESAW_IRQ)) {
    butts = joy.digitalReadBulk(BUTTON_MASK);
    if (!(butts & BUTTON_RIGHT)) {
      DPRINTLN(F("RIGHT"));
    }
    if (!(butts & BUTTON_LEFT)) {
      DPRINTLN(F("LEFT"));
    }
    if (!(butts & BUTTON_UP)) {
      DPRINTLN(F("UP"));
    }
    if (!(butts & BUTTON_DOWN)) {
      DPRINTLN(F("DOWN"));
    }
    if (!(butts & BUTTON_SEL)) {
      DPRINTLN(F("SELECT"));
    }
  }

  x = joy.analogRead(2);
  y = joy.analogRead(3);
  if (HAS_X_MOVED || HAS_Y_MOVED) {
    SerialUSB.print("x: ");
    SerialUSB.print(x);
    SerialUSB.print(" y: ");
    SerialUSB.print(y);
    SerialUSB.println();
    if (ble.isConnected()) {
      ble.print("x ");
      ble.println(x);
      ble.print("y ");
      ble.println(y);
    }
    last_x = x;
    last_y = y;
  }

}

// vi: syntax=arduino
