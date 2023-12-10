#include <esp32_can.h>  // https://github.com/collin80/esp32_can and https://github.com/collin80/can_common CAN libraries
#include <esp32_obd2.h>

#define CAN_RX_PIN  13
#define CAN_TX_PIN  14

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.print("Starting OBD2 RPM... ");

  CAN0.setCANPins((gpio_num_t)CAN_RX_PIN, (gpio_num_t)CAN_TX_PIN);
  
  if (!OBD2.begin()) {
    Serial.println("failed!");
  }
  else {
    Serial.println("succeded");
  }
}

void loop() {
  Serial.print(OBD2.pidName(ENGINE_RPM));
  Serial.print(" = ");
  Serial.print(OBD2.pidRead(ENGINE_RPM));
  Serial.print(OBD2.pidUnits(ENGINE_RPM));
  Serial.println();
  delay(1000);
}
