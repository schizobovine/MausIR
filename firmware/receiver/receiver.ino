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

#define PIN_IRDA 2

IRdecode  decoder;
//IRrecvPCI recv(PIN_IRDA);
IRrecv recv(PIN_IRDA);
uint16_t buff[RECV_BUF_LENGTH];

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
    Serial.println(decoder.bits);
    recv.enableIRIn();
  }

}
