//
// remote_calibrate.h
//
// Calibration program for the remote control
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#ifndef _REMOTE_CALIBRATE_H
#define _REMOTE_CALIBRATE_H


// DEBUG OUTPUT (A LA DEATH OF RATS); set to 0 to disable debugging
#if (1)

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

#define SQUEAK2HEX(a, b) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.print(F(a)); \
  Serial.println((b), HEX); \
} while(0)

#define SQUEAK3(a, b, c) do { \
  Serial.print(F("SQUEAK ")); \
  Serial.print(F(a)); \
  Serial.print((b)); \
  Serial.print(F(" ")); \
  Serial.println((c)); \
} while(0)

#else

#define SKETCH_DEBUG 0
#define SQUEAK(a)
#define SQUEAK2(a, b)
#define SQUEAK2HEX(a, b)
#define SQUEAK3(a, b, c)

#endif

#endif
