//
// sender.ino
//
// Is every Arduino IR library a complete piece of shit? Why the fuck do the
// example programs suck so much? Ugh.
//
// Sends test sequences of coordinates to make sure receiver is getting them
// correctly.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//
#include<Arduino.h>
#include<Metro.h>
#include<MicroView.h>
#include<IRLibAll.h>

#define PIN_IRDA  3
#define PIN_LED   5

#define MIN_X 0
#define MAX_X 255
#define MIN_Y 0
#define MAX_Y 255
#define DELAY_MS 70

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
int16_t reading_x = 0;
int16_t reading_y = 0;
int16_t last_x = 0;
int16_t last_y = 0;
int16_t curr_x = 0;
int16_t curr_y = 0;

// Joystick calibration data. Represents the midpoint of the ADC value range,
// bit shifted down by 5 to discard the low order bits.
int16_t calibration_x = 128;
int16_t calibration_y = 126;

// Timeout for reading joystick data
Metro stickTimer = Metro(DELAY_MS);
uint16_t counter = 0;

void setup() {

  pinMode(PIN_IRDA, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  uView.begin();
  uView.clear(PAGE);
  uView.setCursor(0, 0);
  uView.println("hi");
  uView.display();

#if SKETCH_DEBUG
  if (Serial) {
    Serial.begin(9600);
    Serial.println(F("SQUEAK")); // Say hi
  }
#endif

}

void loop() {

  // Read analog conversion for joystick
  if (stickTimer.check()) {
    //curr_x = (analogRead(PIN_X) >> DROPBITS);
    //curr_y = (analogRead(PIN_Y) >> DROPBITS);
    //reading_x = (reading_x >= MAX_X ? MIN_X : reading_x+1);
    //reading_y = (reading_y >= MAX_Y ? MIN_Y : reading_y+1);
    reading_x = counter & 0xFF;
    reading_y = (counter >> 8) & 0xFF;
    counter++;

    // Re-home coordinates based on calibration settings
    curr_x = reading_x - calibration_x + MIDPOINT_X;
    curr_y = reading_y - calibration_y + MIDPOINT_Y;

    stickTimer.reset();
  }

  // If updated position, send command(s)
  if (curr_x != last_x || curr_y != last_y) {

    // For debugging, fire visible LED and dump to serial
    digitalWrite(PIN_LED, HIGH);

    // Save last positions
    last_x = curr_x;
    last_y = curr_y;

    // Construct command buffer
    cmd = (((uint16_t)curr_x) & 0xFF) << 8 | (((uint16_t)curr_y) & 0xFF);

    // Update screen
    uView.clear(PAGE);
    uView.setCursor(0, 0);
    uView.println(cmd, HEX);
    uView.println(counter, DEC);
    uView.display();

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
