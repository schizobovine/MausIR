//
// bluemaus.ino
//
// Modified mouse robot toy for my cats. Now with Bluetooth!
//
// Author:  Sean Caulfield <sean@yak.net>
// License: GPLv2.0
//

#include<Arduino.h>
#include <nRF5x_BLE_API.h>

#define PIN_MOTOR_A_PH  0
#define PIN_MOTOR_A_EN  1
#define PIN_MOTOR_B_PH  2
#define PIN_MOTOR_B_EN  3
#define PIN_LED         13

//
// UUIDs of service and characteristics
//


// Magical UUIDs for UART because who would need a service to throw generic
// text back and forth in a standard way half the planet uses already? :P
// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E // UART service
// 6E400002-B5A3-F393-E0A9-E50E24DCCA9E // TX characteristic
// 6E400003-B5A3-F393-E0A9-E50E24DCCA9E // RX characteristic

static const uint8_t srv_uuid[] = {
//    0x71, 0x3D, 0x00, 0x00, 0x50, 0x3E, 0x4C, 0x75,
//    0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E
// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E // UART service
  0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
  0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
};

static const uint8_t srv_tx_uuid[] = {
//    0x71, 0x3D, 0x00, 0x03, 0x50, 0x3E, 0x4C, 0x75,
//    0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E
  0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93,
  0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
};

static const uint8_t srv_rx_uuid[] = {
//    0x71, 0x3D, 0x00, 0x02, 0x50, 0x3E, 0x4C, 0x75,
//    0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E
  0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93,
  0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
};

static const uint8_t uart_base_uuid_rev[] = {
    0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA,
    0x75, 0x4C, 0x3E, 0x50, 0x00, 0x00, 0x3D, 0x71
};

//
// Other constants and magic numbers
//

//#define TXRX_BUF_LEN    20
static const size_t CHAR_RX_VAL_BUFSZ = 4;
static const size_t TXRX_BUF_LEN      = 20;
static const int BLE_TX_POWER         = 4;
static const int BLE_ADVERT_TIMEOUT   = 0;   // seconds
static const int BLE_ADVERT_INTERVAL  = 160; // multiples of 0.625ms (wat)
static const char SHORT_LOCAL_NAME[]  = "TXRX";
static const char DEVICE_NAME[]       = "BlueMaus";
static const uint8_t RESPONSE[]       = "OK";

//
// Global objects
//

BLE    ble;
Ticker tock;

uint8_t tx_value[TXRX_BUF_LEN] = {0};
uint8_t rx_value[TXRX_BUF_LEN] = {0};
bool flip = false;

//
// BLE characteristics
//

GattCharacteristic char_tx(
    srv_tx_uuid,
    tx_value,
    1,
    TXRX_BUF_LEN,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE
        | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE
);

GattCharacteristic char_recv(
    srv_rx_uuid,
    rx_value,
    1,
    TXRX_BUF_LEN,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

GattCharacteristic *uartChars[] = {&char_tx, &char_recv};

//
// Unify characteristics into a service
//

GattService uartService(
    srv_uuid,
    uartChars,
    sizeof(uartChars) / sizeof(GattCharacteristic *)
);

//
// Disconnect callback, restarts advertising
//
void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  Serial.println("Disconnected!");
  Serial.println("Restarting the advertising process");
  ble.startAdvertising();
}

//
// Callback when we receive (?) bytes from the attached client.
//
void gattServerWriteCallBack(const GattWriteCallbackParams *Handler) {
  uint8_t buf[TXRX_BUF_LEN];
  uint16_t index;
  uint16_t bytesRead = TXRX_BUF_LEN;

  if (Handler->handle == char_tx.getValueAttribute().getHandle()) {
    ble.readCharacteristicValue(char_tx.getValueAttribute().getHandle(), buf, &bytesRead);
    Serial.print("bytesRead: ");
    Serial.println(bytesRead, HEX);
    for(index=0; index<bytesRead; index++) {
      Serial.print(buf[index], HEX);
      Serial.print(" ");
    }
    Serial.println("");

    /* DO DISPATCH HERE */

    // Write back a response
    ble.updateCharacteristicValue(
        char_recv.getValueAttribute().getHandle(),
        RESPONSE,
        sizeof(RESPONSE)
    );

  }
}

// Timer callback

void timerCallBack() {
    //uint8_t buf[CHAR_RX_VAL_BUFSZ];
    //uint32_t now = millis();

    // Update the "receive" characteristic with the time
    //buf[0] = 'f';
    //buf[1] = 'o';
    //buf[2] = 'o';
    //buf[3] = '\0';
    //ble.updateCharacteristicValue(
    //    char_recv.getValueAttribute().getHandle(),
    //    buf,
    //    CHAR_RX_VAL_BUFSZ
    //);

    // Flip our glow stick to say we're still here
    if (flip) {
        digitalWrite(PIN_LED, HIGH);
    } else {
        digitalWrite(PIN_LED, LOW);
    }
    flip = !flip; // flop

}

void setup() {

    // Setup serial connection, hopefully this doesn't stall?
    Serial.begin(115200);

    // Setup BLE and callbacks
    ble.init();
    ble.onDisconnection(disconnectionCallBack);
    ble.onDataWritten(gattServerWriteCallBack);

    // Setup advertising info
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.accumulateAdvertisingPayload(
        GapAdvertisingData::SHORTENED_LOCAL_NAME,
        (const uint8_t *)SHORT_LOCAL_NAME,
        sizeof(SHORT_LOCAL_NAME) - 1
    );
    ble.accumulateAdvertisingPayload(
        GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
        (const uint8_t *)uart_base_uuid_rev,
        sizeof(uart_base_uuid_rev)
    );
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.setAdvertisingInterval(BLE_ADVERT_INTERVAL);
    ble.setAdvertisingTimeout(BLE_ADVERT_TIMEOUT);

    // Register UART service
    ble.addService(uartService);

    ble.setDeviceName((const uint8_t *)DEVICE_NAME);
    ble.setTxPower(BLE_TX_POWER);

    // Start advertising
    ble.startAdvertising();

    // Set LED phasers to GLOW (wat)
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Setup timer (every 200ms)
    //tock.attach_us(timerCallBack, 200000);
    tock.attach(timerCallBack, 1);

    Serial.println("setup() done");

}

void loop() {
    ble.waitForEvent();
}

// vi: syntax=arduino
