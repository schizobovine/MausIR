//
// remote_calibrate.ino
//
// Calibration program for the remote control
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include<stdlib.h>
#include<limits.h>

#include "remote_calibrate.h"

#define PIN_X A1
#define PIN_Y A0
#define DROPBITS 2

// Joystick calibration data. Represents the midpoint of the ADC value range,
// bit shifted down by DROPBITS to discard the low order bits.
int16_t calibration_x = 128;
int16_t calibration_y = 126;

// Joystick position data
int16_t x = 0;
int16_t y = 0;
int16_t last_x = 0;
int16_t last_y = 0;

// Joystick stats
int16_t min_x = INT_MAX;
int16_t min_y = INT_MAX;
int16_t max_x = INT_MIN;
int16_t max_y = INT_MIN;
int16_t last_min_x = INT_MAX;
int16_t last_min_y = INT_MAX;
int16_t last_max_x = INT_MIN;
int16_t last_max_y = INT_MIN;

//////////////////////////////////////////////////////////////////////

void setup() {

  //curr_x = calibration_x;
  //last_x = calibration_x;
  //curr_y = calibration_y;
  //last_y = calibration_y;

  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  //analogReadResolution(9);

#if SKETCH_DEBUG
  // Say hi
  Serial.begin(115200);
  //SQUEAK(F("SQUEAK SQUEAK"));
#endif

}

//////////////////////////////////////////////////////////////////////

void loop() {

  // Read analog conversion for joystick but drop some least bits so we've only
  // got a total of ??? values for each axis. Subtracting calibration settings
  // shifts the values so that negative is the lower half of each axis.
  x = analogRead(PIN_X) >> DROPBITS;
  y = analogRead(PIN_Y) >> DROPBITS;

  if (x != last_x || y != last_y) {
    last_x = x;
    last_y = y;

#if SKETCH_DEBUG
    Serial.print(F("\t")); //Serial.print(F("x="));
    Serial.print(x);
    Serial.print(F("\t")); //Serial.print(F(" y="));
    Serial.print(y);
    //Serial.print(F(" min_x="));
    //Serial.print(min_x);
    //Serial.print(F(" min_y="));
    //Serial.print(min_y);
    //Serial.print(F(" max_x="));
    //Serial.print(max_x);
    //Serial.print(F(" max_y="));
    //Serial.print(max_y);
    Serial.println();
#endif

  }

  if (x < min_x) {
    min_x = x;
  }

  if (y < min_y) {
    min_y = y;
  }

  if (x > max_x) {
    max_x = x;
  }

  if (y > max_y) {
    max_y = y;
  }

}

//////////////////////////////////////////////////////////////////////
