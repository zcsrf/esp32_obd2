// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <esp32_can.h> // the ESP32_OBD2 library depends on the https://github.com/collin80/esp32_can and https://github.com/collin80/can_common CAN libraries
#include <esp32_obd2.h>

#define CAN_RX_PIN  13
#define CAN_TX_PIN  14

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("OBD2 data printer"));

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
  Serial.print("VIN = ");
  Serial.println(OBD2.vinRead());
  Serial.print("ECU Name = ");
  Serial.println(OBD2.ecuNameRead());
  Serial.println();
}

void loop() {
  // loop through PIDs 0 to 95, reading and printing the values
  for (int pid = 0; pid < 96; pid++) {
    processPid(pid);
  }
  Serial.println();

  // wait 5 seconds before next run
  delay(5000);
}

void processPid(int pid) {
  if (!OBD2.pidSupported(pid)) {
    // PID not supported, continue to next one ...
    return;
  }

  // print PID name
  Serial.print(OBD2.pidName(pid));
  Serial.print(F(" = "));

  if (OBD2.pidValueRaw(pid)) {
    // read the raw PID value
    unsigned long pidRawValue = OBD2.pidReadRaw(pid);

    Serial.print(F("0x"));
    Serial.print(pidRawValue, HEX);
  } else {
    // read the PID value
    float pidValue = OBD2.pidRead(pid);

    if (isnan(pidValue)) {
      Serial.print("error");
    } else {
      // print value with units

      Serial.print(pidValue);
      Serial.print(F(" "));
      Serial.print(OBD2.pidUnits(pid));
    }
  }

  Serial.println();
}

