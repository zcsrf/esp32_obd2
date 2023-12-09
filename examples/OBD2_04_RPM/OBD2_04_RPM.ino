// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <esp32_can.h> // the ESP32_OBD2 library depends on the https://github.com/collin80/esp32_can and https://github.com/collin80/can_common CAN libraries
#include <esp32_obd2.h>

#define CAN_RX_PIN  13
#define CAN_TX_PIN  14


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("OBD2 RPM"));

  CAN0.setCANPins((gpio_num_t)CAN_RX_PIN, (gpio_num_t)CAN_TX_PIN);
  while (true) {
    Serial.print(F("Attempting to connect to OBD2 CAN bus ... "));

    if (!OBD2.begin()) {
      Serial.println(F("failed!"));

      delay(1000);
    } else {
      Serial.println(F("success"));
      break;
    }
  }

  Serial.println();
}

void loop() {

  Serial.print(OBD2.pidName(ENGINE_RPM));
  Serial.print(" = ");
  Serial.print(OBD2.pidRead(ENGINE_RPM));
  Serial.print(OBD2.pidUnits(ENGINE_RPM));
  Serial.println();

  delay(1000);
}
