//
// reciever.ino
//
// Is every Arduino IR library a complete piece of shit? Why the fuck do the
// example programs suck so much? Ugh.
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include<IRLibAll.h>
#include<util/crc16.h>

#define PIN_IRDA 2

IRdecode  decoder;
//IRrecvPCI recv(PIN_IRDA);
IRrecv recv(PIN_IRDA);
uint16_t buff[RECV_BUF_LENGTH];

bool verify(uint32_t cmd) {
  uint16_t crc_given, crc_calc;
  uint8_t x, y;

  y = cmd & 0xFF;
  x = (cmd >> 8) & 0xFF;
  crc_given = (cmd >> 16) & 0xFFFF;

  crc_calc = _crc_ccitt_update(crc_calc, x);
  crc_calc = _crc_ccitt_update(crc_calc, y);

  return (crc_given == crc_calc);
}

void setup() {

  pinMode(PIN_IRDA, INPUT);
  //recv.enableAutoResume(buff);
  recv.enableIRIn();

  Serial.begin(115200);
  Serial.println(F("SQUEAK"));

}

void loop() {

  // Check if new IR command has been sent
  if (recv.getResults()) {
    decoder.decode();
    //decoder.dumpResults(false);
    Serial.print(F("proto "));
    Serial.print(decoder.protocolNum);
    Serial.print(F(" addr "));
    Serial.print(decoder.address);
    Serial.print(F(" val "));
    Serial.print(decoder.value, HEX);
    Serial.print(F(" bits "));
    Serial.print(decoder.bits);
    if (verify(decoder.value)) {
      Serial.println(F(" OK"));
    } else {
      Serial.println(F(" BAD"));
    }
    recv.enableIRIn();
  }

}
