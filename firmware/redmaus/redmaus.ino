//
// redmaus.ino
//
// Modified mouse robot toy for my cats. IR remote controlled.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include<IRremote.h>
#include<Metro.h>

//
// I think
//
// A <=> RIGHT
// B <=> LEFT
//

//
// DRV8835 Cheat Sheet
//

// +-------------------------------------------------------------------+
// | MODE=0 (default)                                                  |
// +-------+-------+--------+--------+---------------------------------+
// |  xIN1 |  xIN2 |  xOUT1 |  xOUT2 | Description                     |
// +-------+-------+--------+--------+---------------------------------+
// |    0  |    0  |  OPEN  |  OPEN  | coast (output floating)         |
// +-------+-------+--------+--------+---------------------------------+
// |  PWM  |    0  |  PWM   |   LOW  | forward/coast at PWM%           |
// +-------+-------+--------+--------+---------------------------------+
// |    0  |  PWM  |   LOW  |  PWM   | reverse/coast at PWM%           |
// +-------+-------+--------+--------+---------------------------------+
// |    1  |  PWM  |  ~PWM  |   LOW  | forward/brake at (100 - PWM%)   |
// +-------+-------+--------+--------+---------------------------------+
// |  PWM  |    1  |   LOW  |  ~PWM  | reverse/brake at (100 - PWM%)   |
// +-------+-------+--------+--------+---------------------------------+
// |    1  |    1  |   LOW  |   LOW  | brake low (output short to GND) |
// +-------+-------+--------+--------+---------------------------------+

// +-------------------------------------------------------------+
// | MODE=1 (pulled high with 10K)                               |
// +-----+-----+-------+-------+---------------------------------+
// | xPH | xEN | xOUT1 | xOUT2 | Description                     |
// +-----+-----+-------+-------+---------------------------------+
// |  0  | PWM |  PWM  |   L   | forward/brake at PWM%           |
// +-----+-----+-------+-------+---------------------------------+
// |  1  | PWM |   L   |  PWM  | reverse/brake at PWM%           |
// +-----+-----+-------+-------+---------------------------------+
// |  x  |  0  |   L   |   L   | brake low (output short to GND) |
// +-----+-----+-------+-------+---------------------------------+

// Pin mappings

#define PIN_IR_RECV    2
#define PIN_MOTOR_PH_R 3
#define PIN_MOTOR_EN_R 5
#define PIN_MOTOR_PH_L 10
#define PIN_MOTOR_EN_L 6

// PWM parameter to control speed

#define SPEED 128

// Motor control macros because I like horrifying people

#define FORWARD 0
#define REVERSE 1

#define MOTOR_R(pwm, dir) do { \
  digitalWrite(PIN_MOTOR_PH_R, (dir)); \
  analogWrite(PIN_MOTOR_EN_R, (pwm)); \
} while (0)

#define MOTOR_L(pwm, dir) do { \
  digitalWrite(PIN_MOTOR_PH_L, (dir)); \
  analogWrite(PIN_MOTOR_EN_L, (pwm)); \
} while (0)

// IR codes from SparkFun's custom IR remote

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

// DEBUG OUTPUT (A LA DEATH OF RATS)

#define SQUEAK(a) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.println((a)); \
} while(0)

#define SQUEAK2(a, b) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.print(F(a)); \
  Serial.println((b)); \
} while(0)

// Objects for processing IR data
IRrecv recv(PIN_IR_RECV);
decode_results results;

// Command timer, reset each time a new IR command comes in; once countdown
// hits zero, resume "default" action which is parked.
const uint32_t CMD_LEN_MS = 200;
Metro timer = Metro(CMD_LEN_MS);

// Decode incoming IR command and return next action
void decode_cmd(decode_results *results) {

  // Ignore non-NEC IR codes
  if (results->decode_type != NEC) {
    SQUEAK2("WAT (type) ", results->decode_type);
    SQUEAK2("WAT (value) ", results->value);
    return;
  }

  // Dispatch based on incoming IR command
  switch (results->value) {

    case NEC_IR_CODE_LEFT:      // Turn left
      SQUEAK(F("LEFT"));
      MOTOR_L(SPEED, REVERSE);
      MOTOR_R(SPEED, FORWARD);
      timer.reset();
      break;

    case NEC_IR_CODE_RIGHT:     // Turn right
      SQUEAK(F("RIGHT"));
      MOTOR_L(SPEED, FORWARD);
      MOTOR_R(SPEED, REVERSE);
      timer.reset();
      break;

    case NEC_IR_CODE_UP:        // Go forwards
      SQUEAK(F("FWD"));
      MOTOR_L(SPEED, FORWARD);
      MOTOR_R(SPEED, FORWARD);
      timer.reset();
      break;

    case NEC_IR_CODE_DOWN:      // Go backwards
      SQUEAK(F("REV"));
      MOTOR_L(SPEED, REVERSE);
      MOTOR_R(SPEED, REVERSE);
      timer.reset();
      break;

    case NEC_IR_CODE_POWER:
    case NEC_IR_CODE_CENTER:    // Stop
      MOTOR_L(0, FORWARD);
      MOTOR_R(0, FORWARD);
      SQUEAK(F("STOP"));
      break;

    case NEC_IR_REPEAT:         // Keep doing last action
      timer.reset();
      SQUEAK(F("KEEP FIRING, ASSHOLES!"));
      break;

    default:                    // Ignore other codes
      SQUEAK2("WAT ", results->value);
      break;
  }

}

void setup() {

  // Set motor pins as outputs
  pinMode(PIN_MOTOR_PH_R, OUTPUT);
  pinMode(PIN_MOTOR_EN_R, OUTPUT);
  pinMode(PIN_MOTOR_PH_L, OUTPUT);
  pinMode(PIN_MOTOR_EN_L, OUTPUT);

  // Set motors to parked
  MOTOR_L(0, FORWARD);
  MOTOR_R(0, FORWARD);

  // Start IR receivers
  pinMode(PIN_IR_RECV, INPUT);
  recv.enableIRIn();

  // Say hi
  Serial.begin(9600);
  SQUEAK(F("SQUEAK SQUEAK"));

}

void loop() {

  // Check if new IR command has been sent
  if (recv.decode(&results)) {
    recv.resume(); //immediately resume looking for input
    //dump_decode(&results;
    decode_cmd(&results);
  }

  // Otherwise, check if our command timer has expired and reset the motors to
  // coast
  if (timer.check()) {
    digitalWrite(PIN_MOTOR_PH_R, LOW);
    analogWrite(PIN_MOTOR_EN_R, LOW);
    digitalWrite(PIN_MOTOR_PH_L, LOW);
    analogWrite(PIN_MOTOR_EN_L, LOW);
  }

  // I tried to use the WDT instead but it was a bundle of fail to get all the
  // libraries working together. Until then, 200ms poll interval and 19mA
  // powerdraw (sigh).
  delay(200);

}
