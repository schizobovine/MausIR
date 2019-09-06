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
#include <Chrono.h>


//
// Debugging macros/settings
//

#define DEBUG_SERIAL      (1)

#if (DEBUG_SERIAL)
#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif

//
// Command structure: 5 bytes total, 4 data + 1 checksum
//
// byte 0 => '!'
// byte 1 => 'B' (for button)
// byte 2 => '1' (buttons 1-4, up=5, down=6, left=7, right=8)
// byte 3 => '1' (pressed) or '0' (released)
// byte 4 => checksum
//

#define CMD_BUFF_SZ       (5)
#define CMD_BYTE_HEADER   (0) // '!'
#define CMD_BYTE_TYPE     (1) // B:button, L:GPS, Q:quaternion, M:mag, A:acc
#define CMD_BYTE_BUTTON   (2)
#define CMD_BYTE_PRESSED  (3)
#define CMD_BYTE_CHECKSUM (4)

#define CMD_DURATION      (200) // in ms, how long should a movement command
                                // "run" before returning to coasting?

#define PWM_SPEED         (127) // Out of 255, how hard to push?

// Is channel A the:
//  LEFT  (true)  -OR-  RIGHT (false)
// wheel?
#define IS_A_LEFT         (false)

// Is "forward" actually "reverse"?
#define REVERSED          (true)

#define BLE_UART_NAME     ("BleuMaus")

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

// Command duration timer
Chrono cmd_timer;

// Motor speed variables
int16_t curr_motor_l = 0;
int16_t curr_motor_r = 0;

/**
 * Dispatch movement commands.
 */
void dispatch_movement() {
    if (curr_motor_l > 0 && curr_motor_r > 0) { //forward
        digitalWrite(PIN_SLEEP, HIGH);
        digitalWrite(PIN_AIN1, (REVERSED) ? LOW : HIGH);
        digitalWrite(PIN_BIN1, (REVERSED) ? LOW : HIGH);
        analogWrite(PIN_AIN2, abs((IS_A_LEFT) ? curr_motor_l : curr_motor_r));
        analogWrite(PIN_BIN2, abs((IS_A_LEFT) ? curr_motor_r : curr_motor_l));
    } else if (curr_motor_l > 0 && curr_motor_r < 0) { // right
        digitalWrite(PIN_SLEEP, HIGH);
        digitalWrite(PIN_AIN1, (IS_A_LEFT) ? HIGH : LOW);
        digitalWrite(PIN_BIN1, (IS_A_LEFT) ? LOW : HIGH);
        analogWrite(PIN_AIN2, abs((IS_A_LEFT) ? curr_motor_l : curr_motor_r));
        analogWrite(PIN_BIN2, abs((IS_A_LEFT) ? curr_motor_r : curr_motor_l));
    } else if (curr_motor_l < 0 && curr_motor_r > 0) { // left
        digitalWrite(PIN_SLEEP, HIGH);
        digitalWrite(PIN_AIN1, (IS_A_LEFT) ? LOW : HIGH);
        digitalWrite(PIN_BIN1, (IS_A_LEFT) ? HIGH : LOW);
        analogWrite(PIN_AIN2, abs((IS_A_LEFT) ? curr_motor_l : curr_motor_r));
        analogWrite(PIN_BIN2, abs((IS_A_LEFT) ? curr_motor_r : curr_motor_l));
    } else if (curr_motor_l < 0 && curr_motor_r < 0) { // reverse
        digitalWrite(PIN_SLEEP, HIGH);
        digitalWrite(PIN_AIN1, (REVERSED) ? HIGH : LOW);
        digitalWrite(PIN_BIN1, (REVERSED) ? HIGH : LOW);
        analogWrite(PIN_AIN2, abs((IS_A_LEFT) ? curr_motor_l : curr_motor_r));
        analogWrite(PIN_BIN2, abs((IS_A_LEFT) ? curr_motor_r : curr_motor_l));
    } else {
        digitalWrite(PIN_SLEEP, LOW);
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, LOW);
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, LOW);
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
    //pinMode(17, OUTPUT);
    //digitalWrite(17, HIGH);

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
    if (ble_uart.available()) {
        if (ble_uart.find("L")) {
            curr_motor_l = (int16_t) ble_uart.parseInt(SKIP_NONE);
            if (ble_uart.find("R")) {
                curr_motor_r = (int16_t) ble_uart.parseInt(SKIP_NONE);
            } else {
                curr_motor_l = curr_motor_r = 0;
            }
        } else {
            curr_motor_l = curr_motor_r = 0;
        }
    }

    // If we are supposed to be moving, do so.
    dispatch_movement();

    // If timer has expired and we've been moving, stop.
    if (cmd_timer.hasPassed(CMD_DURATION)) {
        if (curr_motor_l != 0 || curr_motor_r != 0) {
            curr_motor_l = 0;
            curr_motor_r = 0;
        }
    }

    // Yield to do other stuff.
    delay(10);

}

// vi: syntax=arduino ts=4 sw=4
