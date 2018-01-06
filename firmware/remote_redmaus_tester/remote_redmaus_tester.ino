//
// remote_redmaus_tester.ino
//
// IR remote control for the modified mouse robot toy for my cats.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include<IRremote.h>
#include<Bounce2.h>
#include<MicroView.h>

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

// Object(s) for processing IR data
Bounce butt = Bounce();
IRrecv irrecv(2);
decode_results irdata;

// Joystick position data
uint16_t last_x = 0;
uint16_t last_y = 0;
uint16_t curr_x = 0;
uint16_t curr_y = 0;

bool paused = false;

void setup() {

  irrecv.enableIRIn();

  uView.begin();
  uView.clear(PAGE);
  uView.setCursor(0, 0);
  uView.println("waiting");
  uView.display();

  pinMode(2, INPUT);
  butt.attach(3, INPUT_PULLUP, 100);

#if SKETCH_DEBUG
  // Say hi
  Serial.begin(9600);
  SQUEAK(F("SQUEAK SQUEAK"));
#endif

}

void loop() {

  // If we've gotten a new command, decode it
  if (!paused && irrecv.decode(&irdata)) {

    // Valid code
    if (irdata.decode_type == NEC || irdata.decode_type == WHYNTER) {

      if (irdata.value != 0xFFFFFFFF) { // NEC repeat code
        last_x = curr_x;
        last_y = curr_y;
        curr_x = (uint16_t)(((irdata.value & 0xFFFF0000) >> 16) & 0xFFFF);
        curr_y = (uint16_t)(((irdata.value & 0x0000FFFF) >>  0) & 0xFFFF);
      } else {
        curr_x = last_x;
        curr_y = last_y;
      }

      if (curr_x != 0 || curr_y != 0 || last_x != 0 || last_y != 0) {
        SQUEAK2("delta_x = ", curr_x);
        SQUEAK2("delta_y = ", curr_y);
      }

      uView.clear(PAGE);
      uView.setCursor(0, 0);
      uView.println(irdata.value, HEX);
      uView.println(irdata.decode_type, HEX);
      uView.print("dx "); uView.println(curr_x);
      uView.print("dy "); uView.println(curr_y);
      uView.display();

    // Invalid code, display debug info
    } else {
      uView.clear(PAGE);
      uView.setCursor(0, 0);
      uView.println(irdata.value, HEX);
      uView.println(irdata.decode_type, HEX);
      uView.println("wat");
      uView.display();
    }

    irrecv.resume();

  }


  if (butt.update() && butt.fell()) {
    paused = !paused;
    if (paused) {
      uView.setCursor(0, 32);
      uView.println("paused");
    } else {
      uView.rectFill(0, 32, 64, 8, BLACK, NORM);
    }
    uView.display();
  }

}
