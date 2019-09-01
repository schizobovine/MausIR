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
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

//using namespace Adafruit_LittleFS_Namespace;

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
#define CHANNEL_X       (3)
#define CHANNEL_Y       (2)

//const uint32_t BUTTON_MASK = 0xffffffff;
const uint32_t BUTTON_MASK = 
    BUTTON_RIGHT
  | BUTTON_DOWN
  | BUTTON_LEFT
  | BUTTON_UP
  | BUTTON_SEL
  ;

#define PIN_SEESAW_IRQ  13
#define I2C_ADDR_SEESAW 0x49

#define MIN_DELTA 3
#define HAS_MOVED(z, last_z) (abs((z) - (last_z)) > MIN_DELTA)

#define CALIBRATE_FILENAME ("/calibrat.dat")
#define FILE_O_READ (Adafruit_LittleFS_Namespace::FILE_O_READ)
#define FILE_O_WRITE (Adafruit_LittleFS_Namespace::FILE_O_WRITE)

#define DPAD(x, width, pad_char) \
    do { \
        for (size_t _i=width; _i > 0; _i--) { \
            if ((_i == 1 && (x) >= 0) || (abs(x) < pow(10, _i-1))) { \
                DPRINT(pad_char); \
            } \
        } \
    } while (0)

const int16_t MOTOR_SCALE_Y = 512;
const int16_t MOTOR_SCALE_X = 128;

//
// Global objects
//

Adafruit_seesaw joy;
BLEClientUart bleuart;
Adafruit_LittleFS_Namespace::File caldata(InternalFS);

uint32_t curr_butts = 0;
uint32_t last_butts = 0;
int16_t last_x = 0;
int16_t last_y = 0;
int16_t x = 0;
int16_t y = 0;
int16_t motor_l = 0;
int16_t motor_r = 0;
int16_t calibrate_x = 512;
int16_t calibrate_y = 512;

/**
 * Invoked when UART receives data.
 * @param uart_svc Reference to service object where data arrived (should be bleuart).
 */
void bleuart_rx_callback(BLEClientUart &uart_svc) {
  DPRINT(F("RX < "));
  while (uart_svc.available()) {
    DPRINT((char)uart_svc.read());
  }
  DPRINTLN();
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

/**
 * Read in calibration data from save file on internal FS.
 */
void read_calibration_data() {
    if (caldata.open(CALIBRATE_FILENAME, FILE_O_READ)) {
        caldata.read((uint8_t*)&calibrate_x, sizeof(calibrate_x));
        caldata.read((uint8_t*)&calibrate_y, sizeof(calibrate_y));
        caldata.close();
        DPRINT(F("Reading caliration data: x = "));
        DPRINT(calibrate_x);
        DPRINT(F(", y = "));
        DPRINTLN(calibrate_y);
    }
}

/**
 * Write out calibration data to file on internal FS.
 */
void write_calibration_data() {
    if (caldata.open(CALIBRATE_FILENAME, FILE_O_WRITE)) {

        byte buff[4];
        buff[0] = (calibrate_x >> 8) && 0xFF;
        buff[1] = (calibrate_x >> 0) && 0xFF;
        buff[2] = (calibrate_y >> 8) && 0xFF;
        buff[3] = (calibrate_y >> 0) && 0xFF;

        DPRINT(F("Writing caliration data: x = "));
        DPRINT(calibrate_x);
        DPRINT(F(", y = "));
        DPRINTLN(calibrate_y);

        DPAD(buff[0], 2, "0"); DPRINT(buff[0], HEX);
        DPAD(buff[1], 2, "0"); DPRINT(buff[1], HEX);
        DPAD(buff[2], 2, "0"); DPRINT(buff[2], HEX);
        DPAD(buff[3], 2, "0"); DPRINT(buff[3], HEX);
        DPRINTLN();

        //caldata.write((uint8_t*)&calibrate_x, sizeof(calibrate_x));
        //caldata.write((uint8_t*)&calibrate_y, sizeof(calibrate_y));
        caldata.write(buff, sizeof(buff));
        caldata.flush();
        caldata.close();

    }
}

/**
 * Recalibrate joystick assuming current coordinates are the new origin.
 */
void calibrate_joystick() {
    calibrate_x = joy.analogRead(CHANNEL_X);
    calibrate_y = joy.analogRead(CHANNEL_Y);
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

    // Read in calibration data from internal FS
    InternalFS.begin();
    read_calibration_data();

    // Initialize BLE in Central mode
    Bluefruit.begin(0, 1);
    Bluefruit.setName("BleuMaus Controller");
    Bluefruit.setConnLedInterval(250);

    // Fire up the UART over BLE
    bleuart.begin();
    bleuart.setRxCallback(bleuart_rx_callback);

    // Scan for da maus and try to connect
    Bluefruit.Scanner.setRxCallback(scan_callback);
    Bluefruit.Scanner.restartOnDisconnect(true);
    Bluefruit.Scanner.setInterval(160, 80);
    Bluefruit.Scanner.useActiveScan(false);
    Bluefruit.Scanner.start(0);

    DPRINTLN(F("setup() done"));

}

void loop() {

    curr_butts = joy.digitalReadBulk(BUTTON_MASK);
    if (last_butts != curr_butts) {
        last_butts = curr_butts;
        DPRINTLN(curr_butts, BIN);
        if (!(curr_butts & BUTTON_RIGHT)) {
            DPRINTLN(F("RIGHT"));
        }
        if (!(curr_butts & BUTTON_LEFT)) {
            DPRINTLN(F("LEFT"));
        }
        if (!(curr_butts & BUTTON_UP)) {
            DPRINTLN(F("UP"));
        }
        if (!(curr_butts & BUTTON_DOWN)) {
            DPRINTLN(F("DOWN"));
        }
        if (!(curr_butts & BUTTON_SEL)) {
            DPRINTLN(F("SELECT"));
            // TODO add joystick calibration / recentering function here
            calibrate_joystick();
            write_calibration_data();
        }
    }

    // Subtracting 512 to re-center origin at 0,0
    x = joy.analogRead(CHANNEL_X) - calibrate_x;
    y = -(joy.analogRead(CHANNEL_Y) - calibrate_y);

    // If we've seen enough of a change, dispatch new movement command
    if (HAS_MOVED(x, last_x) || HAS_MOVED(y, last_y)) {

        //if (y >= 0) {
        //    motor_l = (x>=0) ? (MOTOR_MAX) : (MOTOR_MAX + x);
        //    motor_r = (x>=0) ? (MOTOR_MAX - x) : (MOTOR_MAX);
        //} else {
        //    motor_l = (x>=0) ? (MOTOR_MAX - x) : (MOTOR_MAX);
        //    motor_r = (x>=0) ? (MOTOR_MAX) : (MOTOR_MAX + x);
        //}

        motor_l  = MOTOR_SCALE_Y * y / MOTOR_SCALE_Y;
        motor_l += MOTOR_SCALE_X * x / MOTOR_SCALE_X;

        motor_r  = MOTOR_SCALE_Y * y / MOTOR_SCALE_Y;
        motor_r -= MOTOR_SCALE_X * x / MOTOR_SCALE_X;

        //if (x < 0) {
        //    motor_l = motor_l + ((x<0) ? x : -x);
        //    motor_r = motor_r + ((x<0) ? -x : x);
        //} else {
        //    motor_l = motor_l + ((x<0) ? x : -x);
        //    motor_r = motor_r + ((x<0) ? -x : x);
        //}

        //if (y < 0) {
        //    motor_l = -motor_l;
        //    motor_r = -motor_r;
        //}

        DPRINT(F( "x: ")); DPAD(x,       4, F(" ")); DPRINT(x);
        DPRINT(F(" y: ")); DPAD(y,       4, F(" ")); DPRINT(y);
        DPRINT(F(" L: ")); DPAD(motor_l, 4, F(" ")); DPRINT(motor_l);
        DPRINT(F(" R: ")); DPAD(motor_r, 4, F(" ")); DPRINT(motor_r);
        DPRINTLN();

        last_x = x;
        last_y = y;

    }

}

// vi: syntax=arduino ts=4 sw=4 expandtab smarttab
