// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <math.h>
#include <esp32_can.h>
#include "esp32_obd2.h"

#define PROGMEM

OBD2Class::OBD2Class() : _responseTimeout(OBD2_DEFAULT_TIMEOUT),
                         _lastPidResponseMillis(0)
{
}

OBD2Class::~OBD2Class()
{
}

int OBD2Class::begin()
{
  if (!CAN0.begin(CAN_BPS_500K))
  {
    return 0;
  }
  _useExtendedAddressing = true;

  // Standard OBD2 response IDs
  CAN0.watchFor(0x7E8);
  CAN0.watchFor(0x18DAF110);

  // BMW-FAST on CAN: all traffic (requests + responses) lives in 0x600–0x6FF
  // Tester sends on 0x6F1, ECUs respond on 0x600|their_addr (e.g. 0x678, 0x612)
  // Also covers 0x130 (terminal/clamp status broadcast)
  CAN0.watchForRange(0x600, 0x6FF);
  //CAN0.watchFor(0x130);

  return 1;
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

  // Precompute 16-bit and 32-bit values once
  uint16_t raw16 = ((uint16_t)A << 8) | B;
  uint32_t raw32 = ((uint32_t)A << 24) | ((uint32_t)B << 16) | ((uint32_t)C << 8) | D;

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
    return (float)raw32;

  case FUEL_SYSTEM_STATUS: // raw
  case RUN_TIME_SINCE_ENGINE_START:
  case DISTANCE_TRAVELED_WITH_MIL_ON:
  case DISTANCE_TRAVELED_SINCE_CODES_CLEARED:
  case TIME_RUN_WITH_MIL_ON:
  case ENGINE_REF_TORQUE:
  case TIME_SINCE_TROUBLE_CODES_CLEARED:
    return (float)raw16;

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
    return (A / 2.55f);

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
    return ((B / 1.28f) - 100.0f);

  case ENGINE_COOLANT_TEMPERATURE:
  case AIR_INTAKE_TEMPERATURE:
  case AMBIENT_AIR_TEMPERATURE:
  case ENGINE_OIL_TEMPERATURE:
    return (A - 40.0f);

  case SHORT_TERM_FUEL_TRIM_BANK_1:
  case LONG_TERM_FUEL_TRIM_BANK_1:
  case SHORT_TERM_FUEL_TRIM_BANK_2:
  case LONG_TERM_FUEL_TRIM_BANK_2:
  case EGR_ERROR:
    return ((A / 1.28f) - 100.0f);

  case FUEL_PRESSURE:
    return (A * 3.0f);

  case INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
  case VEHICLE_SPEED:
  case WARM_UPS_SINCE_CODES_CLEARED:
  case ABSOLULTE_BAROMETRIC_PRESSURE:
    return (float)A;

  case ENGINE_RPM:
    return (raw16 / 4.0f);

  case TIMING_ADVANCE:
    return ((A / 2.0f) - 64.0f);

  case MAF_AIR_FLOW_RATE:
    return (raw16 / 100.0f);

  case FUEL_RAIL_PRESSURE:
    return (raw16 * 0.079f);

  case FUEL_RAIL_GAUGE_PRESSURE:
  case FUEL_RAIL_ABSOLUTE_PRESSURE:
    return (raw16 * 10.0f);

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
    return ((raw16 * 2.0f) / 65536.0f);

  case EVAP_SYSTEM_VAPOR_PRESSURE:
    return (((int16_t)raw16) / 4.0f);

  case CATALYST_TEMPERATURE_BANK_1_SENSOR_1:
  case CATALYST_TEMPERATURE_BANK_2_SENSOR_1:
  case CATALYST_TEMPERATURE_BANK_1_SENSOR_2:
  case CATALYST_TEMPERATURE_BANK_2_SENSOR_2:
    return ((raw16 / 10.0f) - 40.0f);

  case CONTROL_MODULE_VOLTAGE:
    return (raw16 / 1000.0f);

  case ABSOLUTE_LOAD_VALUE:
    return (raw16 / 2.55f);

  case FUEL_AIR_COMMANDED_EQUIVALENCE_RATE:
    return (2.0f * raw16 / 65536.0f);

  case ABSOLUTE_EVAP_SYSTEM_VAPOR_PRESSURE:
    return (raw16 / 200.0f);

  case 0x54:
    return ((float)raw16 - 32767.0f);

  case FUEL_INJECTION_TIMING:
    return ((raw16 / 128.0f) - 210.0f);

  case ENGINE_FUEL_RATE:
    return (raw16 / 20.0f);
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

  // Precompute commonly used values
  uint16_t raw16 = ((uint16_t)A << 8) | B;
  uint32_t raw32 = ((uint32_t)A << 24) | ((uint32_t)B << 16) | ((uint32_t)C << 8) | D;

  switch (pid)
  {
  case COMMANDED_SECONDARY_AIR_STATUS:
  case OBD_STANDARDS_THIS_VEHICLE_CONFORMS_TO:
  case OXYGEN_SENSORS_PRESENT_IN_2_BANKS:
  case OXYGEN_SENSORS_PRESENT_IN_4_BANKS:
  case AUXILIARY_INPUT_STATUS:
  case FUEL_TYPE:
  case EMISSION_REQUIREMENT_TO_WHICH_VEHICLE_IS_DESIGNED:
    return (uint32_t)A;

  case FUEL_SYSTEM_STATUS:
    return raw16;

  default:
    return raw32;
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

  // Precompute 16-bit and 32-bit values once
  uint16_t raw16 = ((uint16_t)A << 8) | B;
  uint32_t raw32 = ((uint32_t)A << 24) | ((uint32_t)B << 16) | ((uint32_t)C << 8) | D;

  switch (pid)
  {
  case VEHICLE_SPEED_ALT:
    return raw16 * 0.004578f;
  case ACC_FUEL_AMOUNT:
  case ADV_ENGINE_RPM:
    return raw16 * 0.5f;
  case ENGINE_TORQUE:
  case ENGINE_TORQUE_ALT:
    return (raw16 * 0.114443f) - 2500.0f;
  case THROTTLE_VALVE:
    return A * 0.392157f;
  case INJECTION_VOLUME:
  case INJECTION_QUANTITY:
    return (raw16 * 0.003052f) - 100.0f;
  case BOOST_PRESSURE_ABS:
    return raw16 * 0.091554f;
  case AMBIENT_PRESSURE_ABS:
    return raw16 * 0.030518f;
  case BATTERY_VOLTAGE:
    return raw16 * 0.389105f;
  case AMBIENT_TEMPERATURE:
    return (raw16 * 0.1f) - 273.14f;
  case DPF_TEMPERATURE:
    return (raw16 * 0.031281f) - 50.0f;
  case FUEL_LITERS:
    return raw16 * 0.001907f;
  case FUEL_LITERS_ALT:
    return raw16 * 0.01f;
  case DPF_PRESSURE_DIFF:
    return (raw16 * 0.045777f) - 1000.0f;
  case DPF_ASH_WEIGHT:
  case DPF_SOOT_WEIGHT:
    return raw16 * 0.015259f;
  case VEHICLE_ACC:
    return (raw16 * 0.001f) - 32.767f;
  case DPF_REGEN_COUNT:
  case TOTAL_FLOW_INJ:
  case DPF_REGEN_REQ:
    return (float)raw16;
  case DPF_LIFETIME:
    return raw16 * 10.0f;
  case COOLANT_TEMPERATURE:
  case OIL_TEMPERATURE:
  case BOOST_TEMPERATURE:
    return (raw16 * 0.01f) - 100.0f;
  case DPF_REGEN_STATUS:
  default:
    return (float)raw32;
  }
}

// Returns Raw 8-bit value for PIDs that return 1 byte (THROTTLE_VALVE for example)
// Returns 0xFF on failure
uint8_t OBD2Class::pidBmwA(uint16_t pid)
{
  uint8_t value[4];
  uint32_t pid_a = ((uint32_t)pid) | ((uint32_t)0x010 << 16);
  if (!pidBmwRead(0x2C, pid_a, &value, sizeof(value)))
    return 0xFF;
  return value[0];
}

// Returns Raw 16-bit value for PIDs that return 2 bytes (most of them)
// Returns 0xFFFF on failure
uint16_t OBD2Class::pidBmwAB(uint16_t pid)
{
  uint8_t value[4];
  uint32_t pid_a = ((uint32_t)pid) | ((uint32_t)0x010 << 16);
  if (!pidBmwRead(0x2C, pid_a, &value, sizeof(value)))
    return 0xFFFF;
  return ((uint16_t)value[0] << 8) | value[1];
}

// Returns Raw 32-bit value for PIDs that return 4 bytes (DPF_REGEN_STATUS default case)
// Returns 0xFFFFFFFF on failure
uint32_t OBD2Class::pidBmwABCD(uint16_t pid)
{
  uint8_t value[4];
  uint32_t pid_a = ((uint32_t)pid) | ((uint32_t)0x010 << 16);
  if (!pidBmwRead(0x2C, pid_a, &value, sizeof(value)))
    return 0xFFFFFFFF;
  return ((uint32_t)value[0] << 24) | ((uint32_t)value[1] << 16) | ((uint32_t)value[2] << 8) | (uint32_t)value[3];
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

  // Precompute 32-bit value
  uint32_t raw32 = ((uint32_t)A << 24) | ((uint32_t)B << 16) | ((uint32_t)C << 8) | (uint32_t)D;
  return raw32;
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

int OBD2Class::pidBmwRead(uint8_t mode, uint32_t pid, void *data, int length, uint32_t *response_id)
{
  // optional output parameter: clear at start
  if (response_id)
    *response_id = 0;

  unsigned long lastResponseDelta = millis() - _lastPidResponseMillis;
  if (lastResponseDelta < OBD2_MIN_REQUEST_GAP_MS)
  {
    delay(OBD2_MIN_REQUEST_GAP_MS - lastResponseDelta);
  }

  CAN_FRAME outgoing;
  outgoing.id = 0x7df;
  //outgoing.id = 0x7df;
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
        if (response_id) *response_id = incoming.id;
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
          // wait a bit before asking for the next chunk; tuned smaller via macro
          delay(OBD2_FLOWCTRL_CHUNK_DELAY_MS);
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
        if (response_id) *response_id = incoming.id;
        return read;
      }
    }
  }
  return 0;
}

int OBD2Class::readBmwUdsDid(uint8_t module_addr, uint16_t did, uint8_t *dest_buffer, int max_length)
{
  // BMW-FAST on CAN: tester sends on 0x6F1 (0x600 | 0xF1), ECU address goes in data[0]
  // ECU responds on 0x600 | module_addr, with data[0] == 0xF1
  uint32_t response_can_id = 0x600 | module_addr;

  CAN_FRAME outgoing;
  outgoing.id       = 0x6F1; // 0x600 | tester address (0xF1)
  outgoing.length   = 8;
  outgoing.extended = 0;
  outgoing.rtr      = 0;

  // BMW-FAST frame layout: [target_addr] [PCI] [service] [params...]
  outgoing.data.uint8[0] = module_addr;            // Target ECU address
  outgoing.data.uint8[1] = 0x03;                   // PCI: 3 bytes follow
  outgoing.data.uint8[2] = 0x22;                   // Service: ReadDataByIdentifier
  outgoing.data.uint8[3] = (uint8_t)(did >> 8);    // DID High
  outgoing.data.uint8[4] = (uint8_t)(did & 0xFF);  // DID Low
  outgoing.data.uint8[5] = 0x00;
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;

  if (!CAN0.sendFrame(outgoing))
  {
    return -1;
  }

  CAN_FRAME incoming;
  unsigned long start = millis();

  int expected_payload_length = 0;
  int current_index = 0;
  bool receiving_multiframe = false;

  while ((millis() - start) < 500)
  {
    if (CAN0.read(incoming) != 0)
    {
      // Filter: from target ECU (0x600 | module_addr), addressed back to tester (0xF1)
      if (incoming.id == response_can_id && incoming.data.uint8[0] == 0xF1)
      {
        uint8_t pci = incoming.data.uint8[1];

        // --- CASE 1: Single Frame ---
        // Layout: [0xF1] [0x0N] [0x62] [DID_H] [DID_L] [Data0...]
        if ((pci & 0xF0) == 0x00)
        {
          if (incoming.data.uint8[2] == 0x62 &&
              incoming.data.uint8[3] == (uint8_t)(did >> 8) &&
              incoming.data.uint8[4] == (uint8_t)(did & 0xFF))
          {
            expected_payload_length = (pci & 0x0F) - 3; // subtract service + 2 DID bytes

            for (int i = 0; i < expected_payload_length && i < max_length; i++)
            {
              dest_buffer[i] = incoming.data.uint8[5 + i];
            }
            return expected_payload_length;
          }
        }

        // --- CASE 2: Multi-Frame First Frame ---
        // Layout: [0xF1] [0x1N] [Len_L] [0x62] [DID_H] [DID_L] [Data0] [Data1]
        else if ((pci & 0xF0) == 0x10)
        {
          if (incoming.data.uint8[3] == 0x62 &&
              incoming.data.uint8[4] == (uint8_t)(did >> 8) &&
              incoming.data.uint8[5] == (uint8_t)(did & 0xFF))
          {
            receiving_multiframe = true;

            expected_payload_length = (((pci & 0x0F) << 8) | incoming.data.uint8[2]) - 3;

            if (max_length > 0) dest_buffer[0] = incoming.data.uint8[6];
            if (max_length > 1) dest_buffer[1] = incoming.data.uint8[7];
            current_index = 2;

            // Send Flow Control — goes to ECU (0x6F1 -> ECU), so use our TX id
            CAN_FRAME flowControl;
            flowControl.id             = 0x6F1;
            flowControl.length         = 8;
            flowControl.data.uint8[0]  = module_addr; // target
            flowControl.data.uint8[1]  = 0x30;        // FC: ContinueToSend
            flowControl.data.uint8[2]  = 0x00;        // block size 0 = send all
            flowControl.data.uint8[3]  = 0x00;        // min separation time
            for (int i = 4; i < 8; i++)
              flowControl.data.uint8[i] = 0x00;
            CAN0.sendFrame(flowControl);

            start = millis();
          }
        }

        // --- CASE 3: Multi-Frame Consecutive Frame ---
        // Layout: [0xF1] [0x2X] [Data...] x6
        else if (receiving_multiframe && (pci & 0xF0) == 0x20)
        {
          for (int i = 2; i < 8; i++)
          {
            if (current_index < expected_payload_length && current_index < max_length)
            {
              dest_buffer[current_index++] = incoming.data.uint8[i];
            }
          }

          if (current_index >= expected_payload_length)
          {
            return current_index; // SUCCESS
          }
          start = millis();
        }
      }
    }
  }
  return -1; // Timeout
}

int OBD2Class::readBmwKwp2000(uint8_t module_addr, uint16_t pid, bool is_two_byte_pid, uint8_t *dest_buffer, int max_length)
{
  // BMW-FAST on CAN: tester sends on 0x6F1, ECU address in data[0]
  // ECU responds on 0x600 | module_addr, with data[0] == 0xF1
  uint32_t response_can_id = 0x600 | module_addr;

  CAN_FRAME outgoing;
  outgoing.id       = 0x6F1; // 0x600 | tester (0xF1)
  outgoing.length   = 8;
  outgoing.extended = 0;
  outgoing.rtr      = 0;

  uint8_t pid_h = (uint8_t)(pid >> 8);
  uint8_t pid_l = (uint8_t)(pid & 0xFF);

  // BMW-FAST frame layout: [target_addr] [PCI] [service] [params...]
  outgoing.data.uint8[0] = module_addr; // Target ECU address

  if (is_two_byte_pid)
  {
    outgoing.data.uint8[1] = 0x03; // PCI: 3 bytes follow
    outgoing.data.uint8[2] = 0x21; // Service: ReadDataByLocalIdentifier
    outgoing.data.uint8[3] = pid_h;
    outgoing.data.uint8[4] = pid_l;
    outgoing.data.uint8[5] = 0x00;
  }
  else
  {
    outgoing.data.uint8[1] = 0x02; // PCI: 2 bytes follow
    outgoing.data.uint8[2] = 0x21; // Service: ReadDataByLocalIdentifier
    outgoing.data.uint8[3] = pid_l;
    outgoing.data.uint8[4] = 0x00;
    outgoing.data.uint8[5] = 0x00;
  }
  outgoing.data.uint8[6] = 0x00;
  outgoing.data.uint8[7] = 0x00;

  if (!CAN0.sendFrame(outgoing))
    return -1;

  CAN_FRAME incoming;
  unsigned long start = millis();

  int expected_payload_length = 0;
  int current_index = 0;
  bool receiving_multiframe = false;

  while ((millis() - start) < 500)
  {
    if (CAN0.read(incoming) != 0)
    {
      // Filter: from ECU (0x600 | module_addr), data[0] == 0xF1 (addressed to us)
      if (incoming.id == response_can_id && incoming.data.uint8[0] == 0xF1)
      {
        uint8_t pci = incoming.data.uint8[1];

        // --- CASE 1: Single Frame ---
        // Layout: [0xF1] [0x0N] [0x61] [PID...] [Data...]
        if ((pci & 0xF0) == 0x00)
        {
          if (incoming.data.uint8[2] == 0x61)
          {
            bool match = false;
            uint8_t data_start = 0;

            if (is_two_byte_pid && incoming.data.uint8[3] == pid_h && incoming.data.uint8[4] == pid_l)
            {
              match = true;
              data_start = 5;
            }
            else if (!is_two_byte_pid && incoming.data.uint8[3] == pid_l)
            {
              match = true;
              data_start = 4;
            }

            if (match)
            {
              expected_payload_length = (pci & 0x0F) - (is_two_byte_pid ? 3 : 2);

              for (int i = 0; i < expected_payload_length && i < max_length; i++)
              {
                dest_buffer[i] = incoming.data.uint8[data_start + i];
              }
              return expected_payload_length;
            }
          }
        }

        // --- CASE 2: Multi-Frame First Frame ---
        // Layout: [0xF1] [0x1N] [Len_L] [0x61] [PID...] [Data...]
        else if ((pci & 0xF0) == 0x10)
        {
          if (incoming.data.uint8[3] == 0x61)
          {
            bool match = false;
            uint8_t data_start = 0;

            if (is_two_byte_pid && incoming.data.uint8[4] == pid_h && incoming.data.uint8[5] == pid_l)
            {
              match = true;
              data_start = 6;
            }
            else if (!is_two_byte_pid && incoming.data.uint8[4] == pid_l)
            {
              match = true;
              data_start = 5;
            }

            if (match)
            {
              receiving_multiframe = true;

              expected_payload_length = (((pci & 0x0F) << 8) | incoming.data.uint8[2]) - (is_two_byte_pid ? 3 : 2);

              for (int i = data_start; i < 8; i++)
              {
                if (current_index < max_length)
                {
                  dest_buffer[current_index++] = incoming.data.uint8[i];
                }
              }

              // Flow Control — send from tester (0x6F1) to ECU
              CAN_FRAME flowControl;
              flowControl.id            = 0x6F1;
              flowControl.length        = 8;
              flowControl.data.uint8[0] = module_addr; // target
              flowControl.data.uint8[1] = 0x30;        // FC: ContinueToSend
              flowControl.data.uint8[2] = 0x00;        // block size 0 = send all
              flowControl.data.uint8[3] = 0x00;        // min separation time
              for (int i = 4; i < 8; i++)
                flowControl.data.uint8[i] = 0x00;
              CAN0.sendFrame(flowControl);

              start = millis();
            }
          }
        }

        // --- CASE 3: Multi-Frame Consecutive Frame ---
        else if (receiving_multiframe && (pci & 0xF0) == 0x20)
        {
          for (int i = 2; i < 8; i++)
          {
            if (current_index < expected_payload_length && current_index < max_length)
            {
              dest_buffer[current_index++] = incoming.data.uint8[i];
            }
          }

          if (current_index >= expected_payload_length)
          {
            return current_index; // SUCCESS
          }
          start = millis();
        }
      }
    }
  }
  return -1; // Timeout
}

int OBD2Class::ioControlBmwKwp2000(uint8_t module_addr, uint16_t local_id, bool is_two_byte_id, uint8_t control_option, uint8_t *dest_buffer, int max_length)
{
  control_option = 0x01; // FORCE reportCurrentState — never risk actuator control

  // BMW-FAST on CAN: tester sends on 0x6F1, ECU address in data[0]
  // ECU responds on 0x600 | module_addr, with data[0] == 0xF1
  uint32_t response_can_id = 0x600 | module_addr;

  CAN_FRAME outgoing;
  outgoing.id       = 0x6F1; // 0x600 | tester (0xF1)
  outgoing.length   = 8;
  outgoing.extended = 0;
  outgoing.rtr      = 0;

  uint8_t id_h = (uint8_t)(local_id >> 8);
  uint8_t id_l = (uint8_t)(local_id & 0xFF);

  // BMW-FAST frame layout: [target_addr] [PCI] [service] [id...] [control_option]
  outgoing.data.uint8[0] = module_addr; // Target ECU address

  if (is_two_byte_id)
  {
    outgoing.data.uint8[1] = 0x04; // PCI: 4 bytes follow
    outgoing.data.uint8[2] = 0x30; // Service: InputOutputControlByLocalIdentifier
    outgoing.data.uint8[3] = id_h;
    outgoing.data.uint8[4] = id_l;
    outgoing.data.uint8[5] = control_option;
    outgoing.data.uint8[6] = 0x00;
    outgoing.data.uint8[7] = 0x00;
  }
  else
  {
    outgoing.data.uint8[1] = 0x03; // PCI: 3 bytes follow
    outgoing.data.uint8[2] = 0x30; // Service: InputOutputControlByLocalIdentifier
    outgoing.data.uint8[3] = id_l;
    outgoing.data.uint8[4] = control_option;
    outgoing.data.uint8[5] = 0x00;
    outgoing.data.uint8[6] = 0x00;
    outgoing.data.uint8[7] = 0x00;
  }

  if (!CAN0.sendFrame(outgoing))
    return -1;

  CAN_FRAME incoming;
  unsigned long start = millis();

  int expected_payload_length = 0;
  int current_index = 0;
  bool receiving_multiframe = false;

  while ((millis() - start) < 500)
  {
    if (CAN0.read(incoming) != 0)
    {
      
      // Filter: from ECU (0x600 | module_addr), data[0] == 0xF1 (addressed to us)
      if (incoming.id == response_can_id && incoming.data.uint8[0] == 0xF1)
      {
        uint8_t pci = incoming.data.uint8[1];

        // --- CASE 1: Single Frame ---
        // Layout: [0xF1] [0x0N] [0x70] [id...] [option_echo] [Data...]
        if ((pci & 0xF0) == 0x00)
        {
          if (incoming.data.uint8[2] == 0x70)
          {
            bool match = false;
            uint8_t data_start = 0;

            if (is_two_byte_id &&
                incoming.data.uint8[3] == id_h &&
                incoming.data.uint8[4] == id_l &&
                incoming.data.uint8[5] == control_option)
            {
              match = true;
              data_start = 6;
            }
            else if (!is_two_byte_id &&
                     incoming.data.uint8[3] == id_l &&
                     incoming.data.uint8[4] == control_option)
            {
              match = true;
              data_start = 5;
            }

            if (match)
            {
              expected_payload_length = (pci & 0x0F) - (is_two_byte_id ? 4 : 3);

              for (int i = 0; i < expected_payload_length && i < max_length; i++)
              {
                dest_buffer[i] = incoming.data.uint8[data_start + i];
              }
              return expected_payload_length;
            }
          }
        }

        // --- CASE 2: Multi-Frame First Frame ---
        // Layout: [0xF1] [0x1N] [Len_L] [0x70] [id...] [option_echo] [Data...]
        else if ((pci & 0xF0) == 0x10)
        {
          if (incoming.data.uint8[3] == 0x70)
          {
            bool match = false;
            uint8_t data_start = 0;

            if (is_two_byte_id &&
                incoming.data.uint8[4] == id_h &&
                incoming.data.uint8[5] == id_l &&
                incoming.data.uint8[6] == control_option)
            {
              match = true;
              data_start = 7;
            }
            else if (!is_two_byte_id &&
                     incoming.data.uint8[4] == id_l &&
                     incoming.data.uint8[5] == control_option)
            {
              match = true;
              data_start = 6;
            }

            if (match)
            {
              receiving_multiframe = true;

              expected_payload_length = (((pci & 0x0F) << 8) | incoming.data.uint8[2]) - (is_two_byte_id ? 4 : 3);

              for (int i = data_start; i < 8; i++)
              {
                if (current_index < max_length)
                {
                  dest_buffer[current_index++] = incoming.data.uint8[i];
                }
              }

              // Flow Control — send from tester (0x6F1) to ECU
              CAN_FRAME flowControl;
              flowControl.id            = 0x6F1;
              flowControl.length        = 8;
              flowControl.data.uint8[0] = module_addr; // target
              flowControl.data.uint8[1] = 0x30;        // FC: ContinueToSend
              flowControl.data.uint8[2] = 0x00;        // block size 0 = send all
              flowControl.data.uint8[3] = 0x00;        // min separation time
              for (int i = 4; i < 8; i++)
                flowControl.data.uint8[i] = 0x00;
              CAN0.sendFrame(flowControl);

              start = millis();
            }
          }
        }

        // --- CASE 3: Multi-Frame Consecutive Frame ---
        else if (receiving_multiframe && (pci & 0xF0) == 0x20)
        {
          for (int i = 2; i < 8; i++)
          {
            if (current_index < expected_payload_length && current_index < max_length)
            {
              dest_buffer[current_index++] = incoming.data.uint8[i];
            }
          }

          if (current_index >= expected_payload_length)
          {
            return current_index; // SUCCESS
          }
        }
        else if (!receiving_multiframe && (pci & 0xF0) == 0x20)
        {
          Serial.printf("[IOCtrl] WARNING: ConsecFrame arrived but no FirstFrame matched! pci=0x%02X\n", pci);
        }
      }
      else if (incoming.id >= 0x600 && incoming.id <= 0x6FF)
      {
        // Other 0x6xx bus traffic — log so we can see what else is on the bus
        Serial.printf("[IOCtrl] OTHER 0x6xx  ID=0x%03X  [0]=0x%02X  [1]=0x%02X\n",
          incoming.id, incoming.data.uint8[0], incoming.data.uint8[1]);
      }
    }
  }

  return -1; // Timeout
}

int OBD2Class::pidRead(uint8_t mode, uint8_t pid, void *data, int length)
{
  // we changed, from 60 to 10... our ECU is a bit faster
  // make sure a minimal gap passes between successive requests
  unsigned long lastResponseDelta = millis() - _lastPidResponseMillis;
  if (lastResponseDelta < OBD2_MIN_REQUEST_GAP_MS)
  {
    delay(OBD2_MIN_REQUEST_GAP_MS - lastResponseDelta);
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
          delay(OBD2_FLOWCTRL_CHUNK_DELAY_MS);
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