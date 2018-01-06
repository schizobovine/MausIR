//
// remote_redmaus.ino
//
// IR remote control for the modified mouse robot toy for my cats.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include<IRremote.h>
#include<Bounce2.h>

// DEBUG OUTPUT (A LA DEATH OF RATS); set to 0 to disable debugging
#if (0)

#define SKETCH_DEBUG 1
#define SQUEAK(a) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.println((a)); \
} while(0)

#define SQUEAK2(a, b) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.print(F(a)); \
  Serial.println((b)); \
} while(0)

#define SQUEAK3(a, b) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.print(F(a)); \
  Serial.println((b), HEX); \
} while(0)

#else

#define SKETCH_DEBUG 0
#define SQUEAK(a)
#define SQUEAK2(a, b)
#define SQUEAK3(a, b)

#endif

#define PIN_X    A0
#define PIN_Y    A1
#define PIN_IRDA  3
#define PIN_LED  13

// Object(s) for processing IR data
IRsend ir;
Bounce butcal;
uint32_t cmd = 0;

// Joystick position data
int16_t last_x = 0;
int16_t last_y = 0;
int16_t curr_x = 0;
int16_t curr_y = 0;
int16_t delta_x = 0;
int16_t delta_y = 0;

// Joystick calibration data. Represents the midpoint of the ADC value range,
// bit shifted down by 2 to discard the low order bits.
int16_t calibration_x = 128;
int16_t calibration_y = 126;

void setup() {

  //butcal.attach(0, INPUT_PULLUP, 100);
  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_IRDA, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  last_x = curr_x = calibration_x;
  last_y = curr_y = calibration_y;

#if SKETCH_DEBUG
  // Say hi
  Serial.begin(9600);
  SQUEAK(F("SQUEAK SQUEAK"));
#endif

}

void loop() {
#if 0
  digitalWrite(13, HIGH);
  delay(1000);
  ir.sendNEC(0xDEADBEEF, 32);

  digitalWrite(13, LOW);
  delay(1000);
#endif

  // Read analog conversion for joystick
  curr_x = analogRead(PIN_X) >> 2;
  curr_y = analogRead(PIN_Y) >> 2;

  // Reset calibration data if requested
  if (butcal.update() && butcal.rose()) {
    SQUEAK2("(recenter) x = ", curr_x);
    SQUEAK2("(recenter) y = ", curr_y);
    calibration_x = curr_x;
    calibration_y = curr_y;
  }

  // If updated position, send command(s)
  if (curr_x != last_x || curr_y != last_y) {

    // For debugging
    digitalWrite(PIN_LED, HIGH);
    SQUEAK2("curr_x = ", curr_x);
    SQUEAK2("curr_y = ", curr_y);

    // Save last positions
    last_x = curr_x;
    last_y = curr_y;

    // Calculate delta from center
    delta_x = curr_x - calibration_x;
    delta_y = curr_y - calibration_y;
    SQUEAK3("delta_x = ", delta_x);
    SQUEAK3("delta_y = ", delta_y);

    // Construct command buffer using delta
    //SQUEAK3("0) cmd = ", cmd);
    //cmd =  (delta_x >= 0 ? 0x00 : 0xFF) << 24;
    //SQUEAK3("1) cmd = ", cmd);
    cmd = 0;
    cmd |= ((uint32_t)abs(delta_x))      << 16;
    SQUEAK3("2) cmd = ", cmd);
    //cmd |= (delta_y >= 0 ? 0x00 : 0xFF) << 8;
    //SQUEAK3("3) cmd = ", cmd);
    cmd |= ((uint32_t)abs(delta_y))      << 0;

    // Send command buffer
    SQUEAK3("4) cmd = ", cmd);
    ir.sendNEC(cmd, 32);
    digitalWrite(PIN_LED, LOW);

  }

}
