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
#include<Metro.h>
#include<TinyDebugSerial.h>

#define PIN_MOTOR   1
#define PIN_IR_RECV 4
//#define PIN_LED       13

// For SparkFun's custom IR remote
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

// How often to poll for IR data
#define TIMER_IR_CHECK_USEC 200000
#define TIMER_IR_CHECK_MS   200

// PWM limits
#define PWM_MIN 0
#define PWM_MAX 128
#define PWM_INC 32
#define PWM_DEC PWM_INC

TinyDebugSerial Serial = TinyDebugSerial();

IRrecv irrecv(PIN_IR_RECV);

decode_results results;

uint8_t speed = 0;

Metro tock = Metro(200);

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

  // Dispatch based on incoming IR command
  switch (results->value) {

    // Go (slowly)
    case NEC_IR_CODE_A:
      speed = 32;
      break;

    // Go (fast)
    case NEC_IR_CODE_B:
      speed = 64;
      break;

    // Go (really fucking fast)
    case NEC_IR_CODE_C:
      speed = 128;
      break;

    // Increase speed one level
    case NEC_IR_CODE_UP:
    case NEC_IR_CODE_RIGHT:
      speed = constrain(speed + PWM_INC, PWM_MIN, PWM_MAX);
      break;

    // Decrease speed one level
    case NEC_IR_CODE_LEFT:
    case NEC_IR_CODE_DOWN:
      speed = constrain(speed - PWM_DEC, PWM_MIN, PWM_MAX);
      break;

    // Stop
    case NEC_IR_CODE_POWER:
    case NEC_IR_CODE_CENTER:
      speed = 0;
      break;

    // Ignore other codes
    case NEC_IR_REPEAT:
    default:
      Serial.print(F("SQUEAK WAT "));
      Serial.println(results->value, HEX);
      return;
      break;
  }

  analogWrite(PIN_MOTOR, speed);
  Serial.print(F("SQUEAK "));
  Serial.println(speed);

}

// Async routine to check for newly received IR data and dispatch any commands found
void check_for_ir_data() {
  if (irrecv.decode(&results)) {
    irrecv.resume();
    //dump_decode(&results);
    dispatch_decode(&results);
  }
}

void setup() {

  // Set motor PWM off
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);

  // Set LED phasers to GLOW (wat)
  //pinMode(PIN_LED, OUTPUT);
  //digitalWrite(PIN_LED, LOW);

  // Start IR receivers
  pinMode(PIN_IR_RECV, INPUT);
  irrecv.enableIRIn();

  Serial.begin(9600);
  Serial.println("hi");

}

void loop() {

  if (tock.check()) {
    check_for_ir_data();
    tock.reset();
  }

  // I tried to use the WDT instead but it was a bundle of fail to get all the
  // libraries working together. Until then, 200ms poll interval and 19mA
  // powerdraw (sigh).
  //delay(200);
}
