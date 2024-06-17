# Arduino ESP32 OBD2

An Arduino library for reading OBD-II data from your car over CAN bus. 

Based on the MagnusThome/esp32_obd2, where I added some specific PID for BMW N47 (DDE7.1)

Rewritten from sandeepmistry/arduino-OBD2 to instead depend on the ESP32 CAN libraries https://github.com/collin80/esp32_can and https://github.com/collin80/can_common that support ESP32 with TWAI. This means it works with newer ESP32 variants like ESP32-S3, ESP32-C6 and so on.
  
## Install  

Download, unzip and place folder and contents in your Arduino libraries folder

Check MagnusThome/esp32_obd2 for more info (https://github.com/MagnusThome/esp32_obd2)
