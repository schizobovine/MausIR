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

#define DOTSTAR_DAT 7
#define DOTSTAR_CLK 8

#define PIN_MOTOR_PH  0
#define PIN_MOTOR_EN  2
#define PIN_IR_RECV_A 3
#define PIN_IR_RECV_B 4
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

IRrecv recv_a(PIN_IR_RECV_A);
IRrecv recv_b(PIN_IR_RECV_B);

decode_results a, b;

Adafruit_DotStar glowy = Adafruit_DotStar(
  1,           // num pixels
  DOTSTAR_DAT, // data pin
  DOTSTAR_CLK, // clock pin
  DOTSTAR_BRG  // color order
);

int speed = 0;

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
      break;
  }

  // Set new speed and LED status
  if (speed > 0) {
    analogWrite(PIN_MOTOR_PH, speed);
    digitalWrite(PIN_LED, HIGH);
    Serial.print(F("SQUEAK "));
    Serial.println(speed);
  } else {
    analogWrite(PIN_MOTOR_PH, 0);
    digitalWrite(PIN_LED, LOW);
    Serial.println(F("SQUEAK STOP"));
  }

}

// Async routine to check for newly received IR data and dispatch any commands found
void check_for_ir_data() {
  if (recv_a.decode(&a)) {
    recv_a.resume();
    //dump_decode(&a);
    dispatch_decode(&a);
  }

  if (recv_b.decode(&b)) {
    recv_b.resume();
    //dump_decode(&b);
    dispatch_decode(&b);
  }
}

void setup() {

  // Set motor direction (whatever)
  pinMode(PIN_MOTOR_PH, OUTPUT);
  digitalWrite(PIN_MOTOR_PH, LOW);

  // Set motor speed (0)
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, HIGH);

  // Set LED phasers to GLOW (wat)
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // But disable the photon torpedoes (WAT)
  glowy.begin();
  glowy.setPixelColor(0, 0);
  glowy.show();

  // Start IR receivers
  recv_a.enableIRIn();
  recv_b.enableIRIn();

}

void loop() {

  if (tock.check()) {
    check_for_ir_data();
    tock.reset();
  }

  // I tried to use the WDT instead but it was a bundle of fail to get all the
  // libraries working together. Until then, 200ms poll interval and 19mA
  // powerdraw (sigh).
  delay(200);
}
