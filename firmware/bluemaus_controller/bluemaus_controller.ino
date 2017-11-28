//
// bluemaus_controller.ino
//
// Controller for modified mouse robot toy for my cats. Now with Bluetooth!
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include <Arduino.h>
#include <Wire.h>
//#include <Metro.h>
#include <Adafruit_seesaw.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
//#include <EnableInterrupt.h>

//
// UUIDs of service and characteristics
//

// Magical UUIDs for UART because who would need a service to throw generic
// text back and forth in a standard way half the planet uses already? :P

// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E // UART service
// 6E400002-B5A3-F393-E0A9-E50E24DCCA9E // TX characteristic
// 6E400003-B5A3-F393-E0A9-E50E24DCCA9E // RX characteristic

static const uint8_t srv_uuid[] = {
//    0x71, 0x3D, 0x00, 0x00, 0x50, 0x3E, 0x4C, 0x75,
//    0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E
// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E // UART service
  0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
  0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
};

static const uint8_t srv_tx_uuid[] = {
//    0x71, 0x3D, 0x00, 0x03, 0x50, 0x3E, 0x4C, 0x75,
//    0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E
  0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93,
  0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
};

static const uint8_t srv_rx_uuid[] = {
//    0x71, 0x3D, 0x00, 0x02, 0x50, 0x3E, 0x4C, 0x75,
//    0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E
  0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93,
  0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
};

static const uint8_t uart_base_uuid_rev[] = {
    0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA,
    0x75, 0x4C, 0x3E, 0x50, 0x00, 0x00, 0x3D, 0x71
};

//
// Other constants and magic numbers
//

#define BUTTON_RIGHT    (1 << 6)
#define BUTTON_DOWN     (1 << 7)
#define BUTTON_LEFT     (1 << 9)
#define BUTTON_UP       (1 << 10)
#define BUTTON_SEL      (1 << 14)

#define BLE_PRINT_IF_CONNECTED(...) do { \
    if (ble.isConnected()) { \
      ble.print(__VA_ARGS__); \
      ble.flush(); \
    } \
  } while(0)

const uint32_t BUTTON_MASK = 
    BUTTON_RIGHT
  | BUTTON_DOWN
  | BUTTON_LEFT
  | BUTTON_UP
  | BUTTON_SEL
  ;

#define PIN_BLE_RST     4
#define PIN_BLE_IRQ     7
#define PIN_BLE_CS      8
#define PIN_SEESAW_IRQ  13

#define I2C_ADDR_SEESAW 0x49

#define MIN_X_DELTA 3
#define MIN_Y_DELTA 3

#define HAS_X_MOVED (abs((x) - (last_x)) > MIN_X_DELTA)
#define HAS_Y_MOVED (abs((y) - (last_y)) > MIN_Y_DELTA)

//
// Global objects
//

//bool flip = false;
//Metro flipper = Metro(500);
Adafruit_seesaw joy;
Adafruit_BluefruitLE_SPI ble = Adafruit_BluefruitLE_SPI(PIN_BLE_CS, PIN_BLE_IRQ, PIN_BLE_RST);
uint32_t butts = 0; //store buttons pressed in irq handler
int last_x = 0;
int last_y = 0;
int x = 0;
int y = 0;

//
// Helper functions
//

//void timerCallBack() {
//
//    // Flip our glow stick to say we're still here
//    if (flip) {
//        digitalWrite(PIN_LED, HIGH);
//    } else {
//        digitalWrite(PIN_LED, LOW);
//    }
//    flip = !flip; // flop
//
//}

void setup() {

    // Setup serial connection, can't leave this for production tho
    while (!SerialUSB)
      ;
    SerialUSB.begin(115200);

    // Connect to joystick controller
    Wire.begin();
    if (!joy.begin(I2C_ADDR_SEESAW)) {
      SerialUSB.println("Seesaw not found?");
    } else {
      SerialUSB.print("Seesaw found, version: ");
      SerialUSB.println(joy.getVersion());
      joy.pinModeBulk(BUTTON_MASK, INPUT_PULLUP);
      joy.setGPIOInterrupts(BUTTON_MASK, 1);
    }
    pinMode(PIN_SEESAW_IRQ, INPUT);

    // Connect to BLE module
    if (!ble.begin(false)) {
      SerialUSB.println("Bluefruit not found?");
    } else {
      SerialUSB.println("BLE module found");
      ble.info();
      ble.setMode(BLUEFRUIT_MODE_DATA);
    }

    // Attach interrupt handler to joystick IRQ line
    //attachInterrupt(PIN_SEESAW_IRQ, joy_irq, CHANGE);

    // Set LED phasers to GLOW (wat)
    //pinMode(PIN_LED, OUTPUT);
    //digitalWrite(PIN_LED, HIGH);

    SerialUSB.println("setup() done");

}

void loop() {
  bool connected = ble.isConnected();

  if (!digitalRead(PIN_SEESAW_IRQ)) {
    butts = joy.digitalReadBulk(BUTTON_MASK);
    if (!(butts & BUTTON_RIGHT)) {
      SerialUSB.println("RIGHT");
      BLE_PRINT_IF_CONNECTED("R");
    }
    if (!(butts & BUTTON_LEFT)) {
      SerialUSB.println("LEFT");
      BLE_PRINT_IF_CONNECTED("L");
    }
    if (!(butts & BUTTON_UP)) {
      SerialUSB.println("UP");
      BLE_PRINT_IF_CONNECTED("U");
    }
    if (!(butts & BUTTON_DOWN)) {
      SerialUSB.println("DOWN");
      BLE_PRINT_IF_CONNECTED("D");
    }
    if (!(butts & BUTTON_SEL)) {
      SerialUSB.println("SELECT");
      BLE_PRINT_IF_CONNECTED("S");
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
