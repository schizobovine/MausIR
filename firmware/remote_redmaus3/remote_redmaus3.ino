//
// remote_redmaus.ino
//
// IR remote control for the modified mouse robot toy for my cats.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include<IRLibAll.h>
#include<Metro.h>
#include<Wire.h>
#include<ArduinoNunchuk.h>
#include<LowPower.h>

// Enables debugging information output on Serial.
// Comment out/undefine to disable debugging.
#define SKETCH_DEBUG 1

// Hardware configuration
#define PIN_IRDA  3
#define PIN_LED  13

// Microseconds between input poll intervals
const int DELAY_US = 70;

// Milliseconds to wait for input data to change (at all) before going to low
// power mode.
const uint32_t SLEEP_US = 5*60*1000*1000;

// Object(s) for processing IR data
IRsend ir;
uint32_t cmd = 0;
uint8_t bits = 32;

// Timers adnd such
Metro pollTimer = Metro(DELAY_US);
Metro sleepTimer = Metro(SLEEP_US);

// Wiichuck controller
ArduinoNunchuk nunchuk = ArduinoNunchuk();

// Joystick position data
int16_t reading_x = 0;
int16_t reading_y = 0;
int16_t last_x = 0;
int16_t last_y = 0;
int16_t curr_x = 0;
int16_t curr_y = 0;

// Joystick calibration data. Numbers are as reported from my 'chuk with stick
// zeroed. Adjust to your setup as needed.
int16_t calibration_x = 127;
int16_t calibration_y = 127;

// Closely related to the calibration values, but not something that will
// (ever)change is the midpoint of 8 bit unsigned integers (oh for the love of
// gods and Turing let that be a safe assumption...)
const int16_t MIDPOINT_X = 127;
const int16_t MIDPOINT_Y = 127;

//////////////////////////////////////////////////////////////////////

void setup() {

  curr_x = calibration_x;
  curr_y = calibration_y;
  last_x = curr_x;
  last_y = curr_y;

  pinMode(PIN_IRDA, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  Wire.begin();
  nunchuk.init();

#if SKETCH_DEBUG
  delay(1000);
  if (Serial) {
    Serial.begin(57600);
    Serial.println(F("SQUEAK")); // Say hi
  }
#endif

}

//////////////////////////////////////////////////////////////////////

void loop() {

  // Only check for new data every so often
  if (pollTimer.check()) {

    nunchuk.update();

    // Re-home coordinates based on calibration settings
    curr_x = nunchuk.analogX - calibration_x + MIDPOINT_X;
    curr_y = nunchuk.analogY - calibration_y + MIDPOINT_Y;

    // Reset poll timer
    pollTimer.reset();

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

    // Reset sleep timer since something wiggled
    sleepTimer.reset();

  //} else if (sleepTimer.check()) {
    //LowPower.idle(
    //  SLEEP_120MS
    //  , ADC_OFF
    //  , TIMER2_ON
    //  , TIMER1_ON
    //  , TIMER0_ON
    //  , SPI_OFF
    //  , (SKETCH_DEBUG ? USART0_ON : USART0_OFF)
    //  , TWI_OFF
    //);
    
    // TODO: BUTTS
    //sleepTimer.reset();
  }

}

//////////////////////////////////////////////////////////////////////
// vi: ts=4 sw=4 expandtab syntax=arduino
