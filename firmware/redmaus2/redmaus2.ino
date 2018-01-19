//
// redmaus.ino
//
// Modified mouse robot toy for my cats. IR remote controlled.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include<IRLibAll.h>
#include<Metro.h>
#include<LowPower.h>
#include<util/crc16.h>

//////////////////////////////////////////////////////////////////////

//
// A <=> LEFT
// B <=> RIGHT
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

//////////////////////////////////////////////////////////////////////

// Pin mappings

#define PIN_IR_RECV    2
#define PIN_MOTOR_PH_R 10 //3
#define PIN_MOTOR_EN_R 6  //5
#define PIN_MOTOR_PH_L 3  //10
#define PIN_MOTOR_EN_L 5  //6

// PWM parameter to control speed

#define SPEEDS_STRAIGHT 255

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
#define SKETCH_DEBUG 1

// Objects for processing IR data
IRdecode decoder;
IRrecv recv(PIN_IR_RECV);

// Command timer, reset each time a new IR command comes in; once countdown
// hits zero, resume "default" action which is parked.
const uint32_t CMD_LEN_MS = 200;
Metro timer = Metro(CMD_LEN_MS);
bool running = false;

// Decomposed command as vectors for X and Y
uint16_t curr_x = 0;
uint16_t curr_y = 0;

// Controls how fast fwd/backwards we go
#define SPEED_SLOW 128
#define SPEED_FAST 255
uint8_t curr_speed = SPEED_SLOW;

// Controls turn rate
#define SPEED_TURN 64

//////////////////////////////////////////////////////////////////////

// Decode command into global vars and return if CRC16 is valid
bool decode(uint32_t cmd) {
  uint16_t crc_given, crc_calc;
  uint8_t x, y;

  curr_y = cmd & 0xFF;
  curr_x = (cmd >> 8) & 0xFF;
  crc_given = (cmd >> 16) & 0xFFFF;

  crc_calc = _crc_ccitt_update(crc_calc, curr_x);
  crc_calc = _crc_ccitt_update(crc_calc, curr_y);

  return (crc_given == crc_calc);
}

//////////////////////////////////////////////////////////////////////

// Decode incoming IR command and return next action
void decode_cmd(uint32_t cmd) {

#if 0
  // Ignore non-NEC IR codes
  if (results->decode_type != NEC) {
    SQUEAK2("WAT (type) ", results->decode_type);
    SQUEAK2("WAT (value) ", results->value);
    return;
  }

  // Dispatch based on incoming IR command
  switch (results->value) {

    case NEC_IR_CODE_A:         // Go slow
      SQUEAK(F("SLOW"));
      curr_speed = SPEED_SLOW;
      break;

    case NEC_IR_CODE_B:         // Go fast
      SQUEAK(F("FAST"));
      curr_speed = SPEED_FAST;
      break;

    case NEC_IR_CODE_LEFT:      // Turn left
      SQUEAK(F("LEFT"));
      MOTOR_L(SPEED_TURN, REVERSE);
      MOTOR_R(SPEED_TURN, FORWARD);
      timer.reset();
      running = true;
      break;

    case NEC_IR_CODE_RIGHT:     // Turn right
      SQUEAK(F("RIGHT"));
      MOTOR_L(SPEED_TURN, FORWARD);
      MOTOR_R(SPEED_TURN, REVERSE);
      timer.reset();
      running = true;
      break;

    case NEC_IR_CODE_UP:        // Go forwards
      SQUEAK(F("FWD"));
      MOTOR_L(curr_speed, FORWARD);
      MOTOR_R(curr_speed, FORWARD);
      timer.reset();
      running = true;
      break;

    case NEC_IR_CODE_DOWN:      // Go backwards
      SQUEAK(F("REV"));
      MOTOR_L(curr_speed, REVERSE);
      MOTOR_R(curr_speed, REVERSE);
      timer.reset();
      running = true;
      break;

    case NEC_IR_CODE_POWER:
    case NEC_IR_CODE_CENTER:    // Stop
      MOTOR_L(0, FORWARD);
      MOTOR_R(0, FORWARD);
      SQUEAK(F("STOP"));
      running = false;
      break;

    case NEC_IR_REPEAT:         // Keep doing last action
      if (running) {
        timer.reset();
        SQUEAK(F("KEEP FIRING, ASSHOLES!"));
      }
      break;

    default:                    // Ignore other codes
      SQUEAK2("WAT ", results->value);
      break;
  }
#endif

}

//////////////////////////////////////////////////////////////////////

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
#if SKETCH_DEBUG
  Serial.begin(57600);
  Serial.println(F("SQUEAK SQUEAK"));
#endif

}

//////////////////////////////////////////////////////////////////////

void loop() {

  // Check if new IR command has been sent
  if (recv.getResults()) {
    bool valid = false;

    decoder.decode();
    //if (decoder.value != 0) {
      valid = decode(decoder.value);

#if SKETCH_DEBUG
      Serial.print(F("proto "));
      Serial.print(decoder.protocolNum);
      Serial.print(F(" addr "));
      Serial.print(decoder.address);
      Serial.print(F(" val "));
      Serial.print(decoder.value, HEX);
      Serial.print(F(" bits "));
      Serial.print(decoder.bits);
      Serial.print(F(" x "));
      Serial.print(curr_x, HEX);
      Serial.print(F(" y "));
      Serial.print(curr_y, HEX);
      if (valid) {
        Serial.println(F(" OK"));
      } else {
        Serial.println(F(" BAD"));
      }
#endif
    }

    recv.enableIRIn(); //resume looking for input
  //}

  // Otherwise, check if our command timer has expired and reset the motors to
  // coast
  if (timer.check()) {
    digitalWrite(PIN_MOTOR_PH_R, LOW);
    analogWrite(PIN_MOTOR_EN_R, LOW);
    digitalWrite(PIN_MOTOR_PH_L, LOW);
    analogWrite(PIN_MOTOR_EN_L, LOW);
  }

  // If we're still running a command, don't go into standby yet
  // Otherwise, go into low power mode
  if (!running) {
    LowPower.idle(
      SLEEP_120MS
      , ADC_OFF
      , TIMER2_ON
      , TIMER1_ON
      , TIMER0_ON
      , SPI_OFF
      , (SKETCH_DEBUG ? USART0_ON : USART0_OFF)
      , TWI_OFF
    );
  }

}

//////////////////////////////////////////////////////////////////////
