// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <math.h>
#include <esp32_can.h>
#include "esp32_obd2.h"

#define PROGMEM

OBD2Class::OBD2Class() : _responseTimeout(OBD2_DEFAULT_TIMEOUT),
                         _lastPidResponseMillis(0)
{}

OBD2Class::~OBD2Class()
{
}

int OBD2Class::begin()
{
  if (!CAN0.begin(CAN_BPS_500K))
  {
    return 0;
  }
  // Our can uses only standard addressing
    _useExtendedAddressing = true;
  CAN0.watchFor(0x7e8);
  CAN0.watchFor(0x18daf110);

}

void OBD2Class::end()
{
  //  CAN0.end();
}

bool OBD2Class::pidValueRaw(uint8_t pid)
{
  switch (pid)
  {
  case PIDS_SUPPORT_01_20:                                // raw
  case MONITOR_STATUS_SINCE_DTCS_CLEARED:                 // raw
  case FREEZE_DTC:                                        // raw
  case PIDS_SUPPORT_21_40:                                // raw
  case PIDS_SUPPORT_41_60:                                // raw
  case MONITOR_STATUS_THIS_DRIVE_CYCLE:                   // raw
  case FUEL_SYSTEM_STATUS:                                // raw
  case COMMANDED_SECONDARY_AIR_STATUS:                    // raw
  case OBD_STANDARDS_THIS_VEHICLE_CONFORMS_TO:            // raw
  case OXYGEN_SENSORS_PRESENT_IN_2_BANKS:                 // raw
  case OXYGEN_SENSORS_PRESENT_IN_4_BANKS:                 // raw
  case AUXILIARY_INPUT_STATUS:                            // raw
  case FUEL_TYPE:                                         // raw
  case EMISSION_REQUIREMENT_TO_WHICH_VEHICLE_IS_DESIGNED: // raw
    return true;

  default:
    return (pid > 0x5f);
  }
}

float OBD2Class::pidRead(uint8_t pid)
{

#define A value[0]
#define B value[1]
#define C value[2]
#define D value[3]
  uint8_t value[4];

  if (!pidRead(0x01, pid, &value, sizeof(value)))
  {
    return NAN;
  }

  switch (pid)
  {
  default:
  case PIDS_SUPPORT_01_20:                // raw
  case MONITOR_STATUS_SINCE_DTCS_CLEARED: // raw
  case FREEZE_DTC:                        // raw
  case PIDS_SUPPORT_21_40:                // raw
  case PIDS_SUPPORT_41_60:                // raw
  case MONITOR_STATUS_THIS_DRIVE_CYCLE:   // raw
    // NOTE: return value can lose precision!
    return ((uint32_t)A << 24 | (uint32_t)B << 16 | (uint32_t)C << 8 | (uint32_t)D);

  case FUEL_SYSTEM_STATUS: // raw
  case RUN_TIME_SINCE_ENGINE_START:
  case DISTANCE_TRAVELED_WITH_MIL_ON:
  case DISTANCE_TRAVELED_SINCE_CODES_CLEARED:
  case TIME_RUN_WITH_MIL_ON:
  case ENGINE_REF_TORQUE:
  case TIME_SINCE_TROUBLE_CODES_CLEARED:
    return (A * 256.0 + B);

  case CALCULATED_ENGINE_LOAD:
  case THROTTLE_POSITION:
  case COMMANDED_EGR:
  case COMMANDED_EVAPORATIVE_PURGE:
  case FUEL_TANK_LEVEL_INPUT:
  case RELATIVE_THROTTLE_POSITION:
  case ABSOLUTE_THROTTLE_POSITION_B:
  case ABSOLUTE_THROTTLE_POSITION_C:
  case ABSOLUTE_THROTTLE_POSITION_D:
  case ABSOLUTE_THROTTLE_POSITION_E:
  case ABSOLUTE_THROTTLE_POSITION_F:
  case COMMANDED_THROTTLE_ACTUATOR:
  case ETHANOL_FUEL_PERCENTAGE:
  case RELATIVE_ACCELERATOR_PEDAL_POSITTION:
  case HYBRID_BATTERY_PACK_REMAINING_LIFE:
    return (A / 2.55);

  case COMMANDED_SECONDARY_AIR_STATUS:                    // raw
  case OBD_STANDARDS_THIS_VEHICLE_CONFORMS_TO:            // raw
  case OXYGEN_SENSORS_PRESENT_IN_2_BANKS:                 // raw
  case OXYGEN_SENSORS_PRESENT_IN_4_BANKS:                 // raw
  case AUXILIARY_INPUT_STATUS:                            // raw
  case FUEL_TYPE:                                         // raw
  case EMISSION_REQUIREMENT_TO_WHICH_VEHICLE_IS_DESIGNED: // raw
    return (A);

  case OXYGEN_SENSOR_1_SHORT_TERM_FUEL_TRIM:
  case OXYGEN_SENSOR_2_SHORT_TERM_FUEL_TRIM:
  case OXYGEN_SENSOR_3_SHORT_TERM_FUEL_TRIM:
  case OXYGEN_SENSOR_4_SHORT_TERM_FUEL_TRIM:
  case OXYGEN_SENSOR_5_SHORT_TERM_FUEL_TRIM:
  case OXYGEN_SENSOR_6_SHORT_TERM_FUEL_TRIM:
  case OXYGEN_SENSOR_7_SHORT_TERM_FUEL_TRIM:
  case OXYGEN_SENSOR_8_SHORT_TERM_FUEL_TRIM:
    return ((B / 1.28) - 100.0);
    break;

  case ENGINE_COOLANT_TEMPERATURE:
  case AIR_INTAKE_TEMPERATURE:
  case AMBIENT_AIR_TEMPERATURE:
  case ENGINE_OIL_TEMPERATURE:
    return (A - 40.0);

  case SHORT_TERM_FUEL_TRIM_BANK_1:
  case LONG_TERM_FUEL_TRIM_BANK_1:
  case SHORT_TERM_FUEL_TRIM_BANK_2:
  case LONG_TERM_FUEL_TRIM_BANK_2:
  case EGR_ERROR:
    return ((A / 1.28) - 100.0);

  case FUEL_PRESSURE:
    return (A * 3.0);

  case INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
  case VEHICLE_SPEED:
  case WARM_UPS_SINCE_CODES_CLEARED:
  case ABSOLULTE_BAROMETRIC_PRESSURE:
    return (A);

  case ENGINE_RPM:
    return ((A * 256.0 + B) / 4.0);

  case TIMING_ADVANCE:
    return ((A / 2.0) - 64.0);

  case MAF_AIR_FLOW_RATE:
    return ((A * 256.0 + B) / 100.0);

  case FUEL_RAIL_PRESSURE:
    return ((A * 256.0 + B) * 0.079);

  case FUEL_RAIL_GAUGE_PRESSURE:
  case FUEL_RAIL_ABSOLUTE_PRESSURE:
    return ((A * 256.0 + B) * 10.0);

  case OXYGEN_SENSOR_1_FUEL_AIR_EQUIVALENCE_RATIO:
  case OXYGEN_SENSOR_2_FUEL_AIR_EQUIVALENCE_RATIO:
  case OXYGEN_SENSOR_3_FUEL_AIR_EQUIVALENCE_RATIO:
  case OXYGEN_SENSOR_4_FUEL_AIR_EQUIVALENCE_RATIO:
  case OXYGEN_SENSOR_5_FUEL_AIR_EQUIVALENCE_RATIO:
  case OXYGEN_SENSOR_6_FUEL_AIR_EQUIVALENCE_RATIO:
  case OXYGEN_SENSOR_7_FUEL_AIR_EQUIVALENCE_RATIO:
  case OXYGEN_SENSOR_8_FUEL_AIR_EQUIVALENCE_RATIO:
  case 0x34:
  case 0x35:
  case 0x36:
  case 0x37:
  case 0x38:
  case 0x39:
  case 0x3a:
  case 0x3b:
    return (((A * 256.0 + B) * 2.0) / 65536.0);

  case EVAP_SYSTEM_VAPOR_PRESSURE:
    return (((int16_t)(A * 256.0 + B)) / 4.0);

  case CATALYST_TEMPERATURE_BANK_1_SENSOR_1:
  case CATALYST_TEMPERATURE_BANK_2_SENSOR_1:
  case CATALYST_TEMPERATURE_BANK_1_SENSOR_2:
  case CATALYST_TEMPERATURE_BANK_2_SENSOR_2:
    return (((A * 256.0 + B) / 10.0) - 40.0);

  case CONTROL_MODULE_VOLTAGE:
    return ((A * 256.0 + B) / 1000.0);

  case ABSOLUTE_LOAD_VALUE:
    return ((A * 256.0 + B) / 2.55);

  case FUEL_AIR_COMMANDED_EQUIVALENCE_RATE:
    return (2.0 * (A * 256.0 + B) / 65536.0);

  case ABSOLUTE_EVAP_SYSTEM_VAPOR_PRESSURE:
    return ((A * 256.0 + B) / 200.0);

  case 0x54:
    return ((A * 256.0 + B) - 32767.0);

  case FUEL_INJECTION_TIMING:
    return (((A * 256.0 + B) / 128.0) - 210.0);

  case ENGINE_FUEL_RATE:
    return ((A * 256.0 + B) / 20.0);
  }
}

String OBD2Class::vinRead()
{
  char vin[18];

  memset(vin, 0x00, sizeof(vin));

  if (!pidRead(0x09, 0x02, vin, 17))
  {
    // failed
    return "";
  }

  return vin;
}

uint32_t OBD2Class::pidReadRaw(uint8_t pid)
{

#define A value[0]
#define B value[1]
#define C value[2]
#define D value[3]
  uint8_t value[4];

  if (!pidRead(0x01, pid, &value, sizeof(value)))
  {
    return 0;
  }

  switch (pid)
  {
  case COMMANDED_SECONDARY_AIR_STATUS:
  case OBD_STANDARDS_THIS_VEHICLE_CONFORMS_TO:
  case OXYGEN_SENSORS_PRESENT_IN_2_BANKS:
  case OXYGEN_SENSORS_PRESENT_IN_4_BANKS:
  case AUXILIARY_INPUT_STATUS:
  case FUEL_TYPE:
  case EMISSION_REQUIREMENT_TO_WHICH_VEHICLE_IS_DESIGNED:
    return (A);

  case FUEL_SYSTEM_STATUS:
    return ((uint32_t)A << 8 | (uint32_t)B);

  default:
    return ((uint32_t)A << 24 | (uint32_t)B << 16 | (uint32_t)C << 8 | (uint32_t)D);
  }
}

float OBD2Class::pidBmw(uint16_t pid)
{
#define A value[0]
#define B value[1]
#define C value[2]
#define D value[3]
  uint8_t value[4];
  uint32_t pid_a = ((uint32_t)pid) | ((uint32_t)0x010 << 16);
  if (!pidBmwRead(0x2C, pid_a, &value, sizeof(value)))
  {
    return NAN;
  }
  switch (pid)
  {
  case ADV_ENGINE_RPM:
    return (A * 256 + B) * 0.5;
  case ENGINE_TORQUE:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.114443) - 2500;
  case THROTTLE_VALVE:
    return (((uint32_t)A) * 0.392157);
  case INJECTION_VOLUME:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.003052) - 100;
  case BOOST_PRESSURE_ABS:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.091554);
  case AMBIENT_PRESSURE_ABS:
    return (A * 256 + B) * 0.030518;
  case BATTERY_VOLTAGE:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.389105);
  case AMBIENT_TEMPERATURE:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.1) - 273.14;
  case DPF_TEMPERATURE:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.031281) - 50;
  case FUEL_LITERS:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.001907);
  case DPF_PRESSURE_DIFF:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.045777) - 1000;
  case DPF_ASH_WEIGHT:
  case DPF_SOOT_WEIGHT:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.015259);
  case DPF_REGEN_COUNT:
    return (((uint32_t)A << 8 | (uint32_t)B)*10);
  case DPF_LIFETIME:
    return (((uint32_t)A << 8 | (uint32_t)B));
  case COOLANT_TEMPERATURE:
  case OIL_TEMPERATURE:
  case BOOST_TEMPERATURE:
    return (((uint32_t)A << 8 | (uint32_t)B) * 0.01) - 100;
  case DPF_REGEN_STATUS:
  default:
    return ((uint32_t)A << 24 | (uint32_t)B << 16 | (uint32_t)C << 8 | (uint32_t)D);
  }
}

uint32_t OBD2Class::pidBmwRaw(uint16_t pid)
{
#define A value[0]
#define B value[1]
#define C value[2]
#define D value[3]
  uint8_t value[4];

  uint32_t pid_a = ((uint32_t)pid) | ((uint32_t)0x010 << 16);
  if (!pidBmwRead(0x2C, pid_a, &value, sizeof(value)))
  {
    return 0;
  }

  return ((uint32_t)A << 24 | (uint32_t)B << 16 | (uint32_t)C << 8 | (uint32_t)D);
}

String OBD2Class::ecuNameRead()
{
  char ecuName[21];

  memset(ecuName, 0x00, sizeof(ecuName));

  if (!pidRead(0x09, 0x0a, ecuName, 20))
  {
    // failed
    return "";
  }

  return ecuName;
}

void OBD2Class::setTimeout(unsigned long timeout)
{
  _responseTimeout = timeout;
}

int OBD2Class::pidBmwRead(uint8_t mode, uint32_t pid, void *data, int length)
{
  // we changed, from 60 to 10... our ECU is a bit faster...
  // make sure at least 60 ms have passed since the last response
  unsigned long lastResponseDelta = millis() - _lastPidResponseMillis;
  if (lastResponseDelta < 5)
  {
    delay(5 - lastResponseDelta);
  }

  CAN_FRAME outgoing;
  outgoing.id = 0x7df;
  outgoing.length = 8;
  outgoing.extended = 0;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x04; // length (mode + pids)
  outgoing.data.uint8[1] = mode;
  outgoing.data.uint8[2] = (uint8_t)(pid >> 16); // 0x10
  outgoing.data.uint8[3] = (uint8_t)(pid >> 8);  // 0x01
  outgoing.data.uint8[4] = (uint8_t)pid;         // 0x2C
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;

  /*Serial.print("Frame to Send: ");
  for (int i = 0; i < 8; i++)
  {
    Serial.print(outgoing.data.uint8[i], HEX);
    Serial.print(".");
  }*/

  // Attemp to send frame
  for (int retries = 5; retries > 0; retries--)
  {
    if (CAN0.sendFrame(outgoing))
    {
      // send success
      break;
    }
    else if (retries <= 1)
    {
      return 0;
    }
  }

  bool splitResponse = (length > 5);

  CAN_FRAME incoming;
  for (unsigned long start = millis(); (millis() - start) < _responseTimeout;)
  {
    if (CAN0.read(incoming) != 0)
    {
      _lastPidResponseMillis = millis();
      if (!splitResponse && incoming.data.uint8[1] == (mode | 0x40) && incoming.data.uint8[2] == (uint8_t)(pid >> 16))
      {
        for (uint8_t i = 0; i < length; i++)
        {
          ((uint8_t *)data)[i] = incoming.data.uint8[i + 3];
        }
        return length;
      }

      // Is multiple packets
      if (incoming.data.uint8[0] == 0x10)
      {
        int read = 0;

        // Get first packet
        //////// Why only read three of the remaining six bytes in the first packet:
        //////// int read = CAN.readBytes((uint8_t*)data, 3);
        while (read < 3)
        {
          ((uint8_t *)data)[read] = incoming.data.uint8[read + 2];
          read++;
        }

        // Loop through rest of multiple packets
        for (int pck = 0; read < length; pck++)
        {
          delay(60);
          // send the request for the next chunk
          outgoing.data.uint8[0] = 0x30;
          CAN0.sendFrame(outgoing);
          // wait for (proper) response
          while (CAN0.read(incoming) != 0 || incoming.data.uint8[0] != (0x21 + pck))
          {
            // Serial.print(".");
          }; // correct sequence number

          // Something recieved
          for (uint8_t i = 0; i < 7 && read < length; i++)
          {
            ((uint8_t *)data)[read++] = incoming.data.uint8[i + 1];
            /*for (int i = 0; i < 8; i++)
            {
              Serial.print(incoming.data.uint8[i], HEX);
              Serial.print(".");
            }*/
          }
        }

        _lastPidResponseMillis = millis();
        return read;
      }
    }
  }
  return 0;
}

int OBD2Class::pidRead(uint8_t mode, uint8_t pid, void *data, int length)
{
  // we changed, from 60 to 10... our ECU is a bit faster...
  // make sure at least 60 ms have passed since the last response
  unsigned long lastResponseDelta = millis() - _lastPidResponseMillis;
  if (lastResponseDelta < 5)
  {
    delay(5 - lastResponseDelta);
  }

  CAN_FRAME outgoing;
  outgoing.id = 0x7df;
  outgoing.length = 8;
  outgoing.extended = 0;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x02;
  outgoing.data.uint8[1] = mode;
  outgoing.data.uint8[2] = pid;
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;

  for (int retries = 5; retries > 0; retries--)
  {
    if (CAN0.sendFrame(outgoing))
    {
      // send success
      break;
    }
    else if (retries <= 1)
    {
      return 0;
    }
  }

  bool splitResponse = (length > 5);

  CAN_FRAME incoming;
  for (unsigned long start = millis(); (millis() - start) < _responseTimeout;)
  {
    if (CAN0.read(incoming) != 0)
    {
      _lastPidResponseMillis = millis();

      if (!splitResponse && incoming.data.uint8[1] == (mode | 0x40) && incoming.data.uint8[2] == pid)
      {
        for (uint8_t i = 0; i < length; i++)
        {
          ((uint8_t *)data)[i] = incoming.data.uint8[i + 3];
        }
        return length;
      }

      // Is multiple packets
      if (incoming.data.uint8[0] == 0x10)
      {
        int read = 0;

        // Get first packet
        //////// Why only read three of the remaining six bytes in the first packet:
        //////// int read = CAN.readBytes((uint8_t*)data, 3);
        while (read < 3)
        {
          ((uint8_t *)data)[read] = incoming.data.uint8[read + 2];
          read++;
        }

        // Loop through rest of multiple packets
        for (int pck = 0; read < length; pck++)
        {
          delay(60);

          // send the request for the next chunk
          outgoing.data.uint8[0] = 0x30;
          CAN0.sendFrame(outgoing);

          // wait for (proper) response
          while (CAN0.read(incoming) != 0 || incoming.data.uint8[0] != (0x21 + pck))
            ; // correct sequence number

          // Something recieved
          for (uint8_t i = 0; i < 7 && read < length; i++)
          {
            ((uint8_t *)data)[read++] = incoming.data.uint8[i + 1];
          }
        }

        _lastPidResponseMillis = millis();
        return read;
      }
    }
  }
  return 0;
}

OBD2Class OBD2;
