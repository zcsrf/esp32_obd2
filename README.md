# Arduino OBD2

An Arduino library for reading OBD-II data from your car over CAN bus. 


Rewritten from sandeepmistry/arduino-OBD2 to instead depend on the ESP32 CAN libraries https://github.com/collin80/esp32_can and https://github.com/collin80/can_common that support ESP32 with TWAI. This means it works with newer ESP32 variants like ESP32-S3, ESP32-C6 and so on.

