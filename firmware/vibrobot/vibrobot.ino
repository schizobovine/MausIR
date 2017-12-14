//
// vibrobot.ino
//
// Modified mouse robot toy for my cats.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include<IRremote.h>
#include<Adafruit_DotStar.h>
#include<Metro.h>
#include<Bounce2.h>

//
// Pin mapping for Mk2 mouse
//

#define PIN_MOTOR_PH    3
#define PIN_MOTOR_EN    2
#define PIN_IR_RECV_A   0
#define PIN_IR_RECV_B   4
#define PIN_TILT_SW     1
#define PIN_DOTSTAR_DAT 7
#define PIN_DOTSTAR_CLK 8
//#define PIN_LED       13

//
// NEC IR codes for SparkFun's custom IR remote
//

#define NEC_IR_CODE_UP     0x10EFA05F
#define NEC_IR_CODE_DOWN   0x10EF00FF
#define NEC_IR_CODE_LEFT   0x10EF10EF
#define NEC_IR_CODE_RIGHT  0x10EF807F
#define NEC_IR_CODE_CENTER 0x10EF20DF
#define NEC_IR_CODE_A      0x10EFF807
#define NEC_IR_CODE_B      0x10EF7887
#define NEC_IR_CODE_C      0x10EF58A7
#define NEC_IR_CODE_POWER  0x10EFD827
#define NEC_IR_REPEAT      0xFFFFFFFF

//
// How often to poll for IR data
//

#define TIMER_IR_CHECK_USEC 200000
#define TIMER_IR_CHECK_MS   200

//
// Tilting at switches
//

#define TILT_SWITCH_DEBOUNCE_MS 10

//
// PWM limits
//

#define PWM_MIN 0
#define PWM_MAX 255
#define PWM_INC_SMALL 32
#define PWM_DEC_SMALL -(PWM_INC_SMALL)
#define PWM_INC_LARGE 64
#define PWM_DEC_LARGE -(PWM_INC_LARGE)
#define PWM_BOUND(x) (constrain((x), PWM_MIN, PWM_MAX))
#define PWM_SLOW 63
#define PWM_FAST 127

#define SQUEAK(msg, what, how) \
  do { \
    Serial.print(F("SQUEAK ")); \
    Serial.print(F(msg " ")); \
    Serial.println((what), (how)); \
    Serial.flush(); \
  } while (0)

//
// Globals
//

// IR data receiver & decoder controllers

IRrecv recv_a(PIN_IR_RECV_A);
//IRrecv recv_b(PIN_IR_RECV_B);
decode_results a;//, b;

// Built-in LED we need to turn off to save power

Adafruit_DotStar glowy = Adafruit_DotStar(
  1,           // num pixels
  PIN_DOTSTAR_DAT, // data pin
  PIN_DOTSTAR_CLK, // clock pin
  DOTSTAR_BRG  // color order
);

// Speed & motor variables

int speed = 0;
int delta_v = 0;
int last_speed = 0;
int last_delta_v = 0;

// "Twitch mode," based on the tilt switch inside, will cause the mouse to wake
// up and move for a random interval until it's bumped again.

Bounce butt = Bounce();
bool twitch = false;

//void dump_decode(decode_results *results) {
//  Serial.print(millis());
//  Serial.print(F(" SQUEAK type:"));
//  Serial.print(results->decode_type, DEC);
//  Serial.print(F(" bits:"));
//  Serial.print(results->bits, DEC);
//  Serial.print(F(" hex:"));
//  Serial.print(results->value, HEX);
//  if (results->overflow)
//    Serial.print(F(" Overflow!"));
//  Serial.println();
//}

void dispatch_decode(decode_results *results) {

  // Ignore non-NEC IR codes
  if (results->decode_type != NEC) {
    return;
  }

  // Fart out command received and store for repeated command checks later
  if (results == &a) {
    SQUEAK("A<", results->value, HEX);
  } else {
    SQUEAK("B<", results->value, HEX);
  }

  // Default to no change in speed
  delta_v = 0;

  switch (results->value) {

    case NEC_IR_CODE_A:       // Go (slowly)
      delta_v = PWM_SLOW - speed;
      break;

    case NEC_IR_CODE_B:       // Go (fast)
      delta_v = PWM_FAST - speed;
      break;

    case NEC_IR_CODE_C:       // (De-)activate "twitch mode"
      twitch = !twitch;
      break;

    case NEC_IR_CODE_RIGHT:   // Increase speed a little
      delta_v = PWM_INC_SMALL;
      break;

    case NEC_IR_CODE_LEFT:    // Decrease speed a little
      delta_v = PWM_DEC_SMALL;
      break;

    case NEC_IR_CODE_UP:      // Increase speed a lot
      delta_v = PWM_INC_LARGE;
      break;

    case NEC_IR_CODE_DOWN:    // Decrease speed a lot
      delta_v = PWM_DEC_LARGE;
      break;

    case NEC_IR_CODE_POWER:
    case NEC_IR_CODE_CENTER:  // Stop
      delta_v = 0 - speed;
      break;

    case NEC_IR_REPEAT:       // Repeat last change
      delta_v = last_delta_v;
      break;

    default:                  // Ignore other codes
      //SQUEAK("WAT", results->value, HEX);
      break;
  }

  last_speed = speed;
  last_delta_v = delta_v;

  if (delta_v != 0) {
    SQUEAK("delta_v", delta_v, DEC);
    speed = PWM_BOUND(speed + delta_v);
  }

  if (last_speed != speed) {
    analogWrite(PIN_MOTOR_EN, speed);
    SQUEAK("speed", speed, DEC);
  }
  digitalWrite(PIN_LED, (speed > 0) ? HIGH : LOW);

}

// Async routine to check for newly received IR data and dispatch any commands
// found
void check_for_ir_data() {
  if (recv_a.decode(&a)) {
    recv_a.resume();
    //dump_decode(&a);
    dispatch_decode(&a);
  }

  //if (recv_b.decode(&b)) {
  //  recv_b.resume();
  //  //dump_decode(&b);
  //  dispatch_decode(&b);
  //}
}

void setup() {

  // Set motor direction (whatever)
  pinMode(PIN_MOTOR_PH, OUTPUT);
  digitalWrite(PIN_MOTOR_PH, LOW);

  // Set motor speed (0)
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, LOW);

  // Set LED phasers to GLOW (wat)
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // But disable the photon torpedoes (WAT)
  glowy.begin();
  glowy.setPixelColor(0, 0);
  glowy.show();

  // Start IR receivers
  recv_a.enableIRIn();
  //recv_b.enableIRIn();

  // Prime the prime number pump
  randomSeed(analogRead(0));

  // Hold on to your butts, people
  butt.attach(PIN_TILT_SW, TILT_SWITCH_DEBOUNCE_MS, INPUT_PULLUP);

  // Say hi
  if (Serial) {
    Serial.begin(9600);
    Serial.println(F("SQUEAK SQUEAK"));
  }


}

void loop() {

  if (recv_a.decode(&a)) {
    recv_a.resume(); // Immediately resume looking for input
    dispatch_decode(&a);
  }

  if (butt.update()) {
    SQUEAK("OH NOES MY BUTT", millis(), DEC);
  }

  // I tried to use the WDT instead but it was a bundle of fail to get all the
  // libraries working together. Until then, 200ms poll interval and 19mA
  // powerdraw (sigh).
  delay(200);

}
