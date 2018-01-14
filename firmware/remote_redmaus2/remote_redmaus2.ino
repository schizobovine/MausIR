//
// remote_redmaus.ino
//
// IR remote control for the modified mouse robot toy for my cats.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>

#include<IRLibSendBase.h>
#include<IRLib_P07_NECx.h>
#include<IRLibCombo.h>

#define PIN_X    A5
#define PIN_Y    A4
#define PIN_IRDA  3
#define PIN_LED  13

// Number of low order bits to shave off the ADC readings before relaying to
// the mouse
#define DROPBITS 2

// Closely related to the calibration values, but not something that will
// (ever)change is the midpoint of 8 bit unsigned integers (oh for the love of
// gods and Turing let that be a safe assumption...)
const int16_t MIDPOINT_X = 127;
const int16_t MIDPOINT_Y = 127;

// Object(s) for processing IR data
IRsend ir;
uint32_t cmd = 0;
uint8_t bits = 16;

// Joystick position data
int16_t last_x = 0;
int16_t last_y = 0;
int16_t curr_x = 0;
int16_t curr_y = 0;

// Joystick calibration data. Represents the midpoint of the ADC value range,
// bit shifted down by 5 to discard the low order bits.
int16_t calibration_x = 128;
int16_t calibration_y = 126;

void setup() {

  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_IRDA, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  last_x = curr_x = calibration_x;
  last_y = curr_y = calibration_y;

#if SKETCH_DEBUG
  if (Serial) {
    Serial.begin(9600);
    Serial.println(F("SQUEAK")); // Say hi
  }
#endif

}

void loop() {

  // Read analog conversion for joystick
  curr_x = (analogRead(PIN_X) >> DROPBITS);
  curr_y = (analogRead(PIN_Y) >> DROPBITS);

  // Re-home coordinates based on calibration settings
  curr_x = curr_x - calibration_x + MIDPOINT_X;
  curr_y = curr_y - calibration_y + MIDPOINT_Y;

  // If updated position, send command(s)
  if (curr_x != last_x || curr_y != last_y) {

    // For debugging, fire visible LED and dump to serial
    digitalWrite(PIN_LED, HIGH);

    // Save last positions
    last_x = curr_x;
    last_y = curr_y;

    // Construct command buffer
    cmd = (((uint16_t)curr_x) & 0xFF) << 8 | (((uint16_t)curr_y) & 0xFF);
    if (cmd == 0) {
      cmd = 0xBEEF;
      bits = 16;
    } else {
      bits = 16;
    }

    // Send command buffer
    ir.send(NECX, cmd, bits);

    // Disable LED since we're done now
    digitalWrite(PIN_LED, LOW);

#if SKETCH_DEBUG
    Serial.print(F("x "));
    Serial.print(curr_x);
    Serial.print(F(" y "));
    Serial.print(curr_y);
    Serial.print(F(" cmd "));
    Serial.print(cmd, HEX);
    Serial.println();
#endif

  }

}
