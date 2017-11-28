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

// I think
// A <=> RIGHT
// B <=> LEFT

#define PIN_MOTOR_AIN1 3
#define PIN_MOTOR_AIN2 5
#define PIN_MOTOR_BIN1 6
#define PIN_MOTOR_BIN2 10
#define PIN_IR_RECV 2
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
#define PWM_MIN -127
#define PWM_MAX 128
#define PWM_ABS_MIN 0
#define PWM_ABS_MAX 255
#define PWM_INC 32
#define PWM_DEC PWM_INC

#define DUTY(x) (map((x), PWM_MIN, PWM_MAX, PWM_ABS_MIN, PWM_ABS_MAX))
#define SQUEAK(a) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.println((a)); \
} while(0)
#define SQUEAK2(a, b) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.print(F(a)); \
  Serial.println((b)); \
} while(0)

IRrecv recv(PIN_IR_RECV);
Metro tock = Metro(200);

decode_results results;

// Current & last speed settings
int curr_pwm_a = 0;
int curr_pwm_b = 0;
int last_pwm_a = 0;
int last_pwm_b = 0;

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

    case NEC_IR_CODE_A:
    case NEC_IR_CODE_B:
    case NEC_IR_CODE_C:
      break;

    // Increase speed one level for channel a
    case NEC_IR_CODE_RIGHT:
      last_pwm_a = curr_pwm_a;
      curr_pwm_a = constrain(curr_pwm_a + PWM_INC, PWM_MIN, PWM_MAX);
      break;

    // Decrease speed one level for channel a
    case NEC_IR_CODE_LEFT:
      last_pwm_a = curr_pwm_a;
      curr_pwm_a = constrain(curr_pwm_a - PWM_DEC, PWM_MIN, PWM_MAX);
      break;

    // Increase speed one level for channel b
    case NEC_IR_CODE_UP:
      last_pwm_b = curr_pwm_b;
      curr_pwm_b = constrain(curr_pwm_b + PWM_INC, PWM_MIN, PWM_MAX);
      break;

    // Decrease speed one level for channel b
    case NEC_IR_CODE_DOWN:
      last_pwm_b = curr_pwm_b;
      curr_pwm_b = constrain(curr_pwm_b - PWM_DEC, PWM_MIN, PWM_MAX);
      break;

    // Stop
    case NEC_IR_CODE_POWER:
    case NEC_IR_CODE_CENTER:
      last_pwm_a = curr_pwm_a;
      curr_pwm_a = 0;
      last_pwm_b = curr_pwm_b;
      curr_pwm_b = 0;
      break;

    // Ignore other codes
    case NEC_IR_REPEAT:
    default:
      SQUEAK2("WAT ", results->value);
      break;
  }

  if (curr_pwm_a != last_pwm_a) {
    if (curr_pwm_a > 0) { // forward/coast
      analogWrite(PIN_MOTOR_AIN1, DUTY(curr_pwm_a));
      digitalWrite(PIN_MOTOR_AIN2, LOW);
      SQUEAK2("A (fwd) = ", DUTY(curr_pwm_a));
    } else { // reverse/coast
      digitalWrite(PIN_MOTOR_AIN1, LOW);
      analogWrite(PIN_MOTOR_AIN2, DUTY(curr_pwm_a));
      SQUEAK2("A (rev) = ", DUTY(curr_pwm_a));
    }
  } else {
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
    SQUEAK(F("A STOP"));
  }

  if (curr_pwm_b != last_pwm_b) {
    if (curr_pwm_b > 0) { // forward/coast
      analogWrite(PIN_MOTOR_BIN1, DUTY(curr_pwm_b));
      digitalWrite(PIN_MOTOR_BIN2, LOW);
      SQUEAK2(" B (fwd) = ", DUTY(curr_pwm_b));
    } else { // reverse/coast
      digitalWrite(PIN_MOTOR_BIN1, LOW);
      analogWrite(PIN_MOTOR_BIN2, DUTY(curr_pwm_b));
      SQUEAK2(" B (rev) = ", DUTY(curr_pwm_b));
    }
  } else {
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    digitalWrite(PIN_MOTOR_BIN2, LOW);
    SQUEAK(F("B STOP"));
  }

}

// Async routine to check for newly received IR data and dispatch any commands found
void check_for_ir_data() {
  if (recv.decode(&results)) {
    recv.resume();
    //dump_decode(&results;
    dispatch_decode(&results);
  }
}

void setup() {

  // Set motor pins as outputs
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_AIN2, OUTPUT);
  pinMode(PIN_MOTOR_BIN1, OUTPUT);
  pinMode(PIN_MOTOR_BIN2, OUTPUT);

  // Set motors to freewheel
  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, LOW);
  digitalWrite(PIN_MOTOR_BIN1, LOW);
  digitalWrite(PIN_MOTOR_BIN2, LOW);

  // Start IR receivers
  recv.enableIRIn();

  // Say hi
  while (!Serial)
    ;
  Serial.begin(9600);
  SQUEAK(F("B STOP"));

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
