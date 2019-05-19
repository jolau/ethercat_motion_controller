//
// Created by jolau on 27.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP

#include <cstdint>
#include <varileg_lowlevel_controller/entities/DeviceState.hpp>
#include "string"
#include "varileg_lowlevel_controller/entities/OperatingMode.hpp"
#include "varileg_lowlevel_controller/entities/HomingState.hpp"
#include "varileg_lowlevel_controller/entities/HomingMethod.hpp"

namespace varileg_lowlevel_controller {
/**
 * Service Data Object, as defined in EPOS Firmware specifications.
 */
struct SDO {
  const std::string name; ///< Describes what SDO does. Usually title as in EPOS Firmware specifications.
  const uint16_t index;
  const uint8_t subindex;
};

/**
 * Controlword as specified in EPOS firmware specifications. Consists of #word and #mask to only change the necessary bits.
 */
struct Controlword {
  uint16_t word;
  uint16_t mask;

  /**
   * Apply this controlword on existing controlword. Changes only the bits in #mask with the #word.
   * @param controlword Existing controlword to be changed.
   */
  void apply(uint16_t &controlword) const {
    controlword = (controlword & ~mask) | (word & mask);
  }
};

/**
 * Statusword as specified in EPOS firmware specifications. Consists of #word and #mask to compare the necessary bits.
 */
struct Statusword {
  uint16_t word;
  uint16_t mask;

  /**
   * Check if this statusword is active, by checking if word equals the masked statusword.
   * @param statusword Statusword to be checked.
   * @return true if statusword is active.
   */
  bool isActive(const uint16_t &statusword) const {
    return (statusword & mask) == word;
  };
};

/**
 * Representation of EPOS firmware specifications. Displays only the needed part of it.
 */
namespace EposCommandLibrary {
namespace SDOs {
const SDO MODES_OF_OPERATION = {"Modes of operation", 0x6060, 0x00};
const SDO MODES_OF_OPERATION_DISPLAY = {"Modes of operation display", 0x6061, 0x00};
const SDO NODE_ID = {"Node-ID", 0x2000, 0x00};
const SDO INTERPOLATION_TIME_PERIOD_VALUE = {"Interpolation time period value", 0x60C2, 0x01};
const SDO OUTPUT_CURRENT_LIMIT = {"Motor Current Limit in mA", 0x3001, 0x02};

const SDO FOLLOWING_ERROR_WINDOW = {"Following error window", 0x6065, 0x00};
const SDO HOME_OFFSET_MOVE_DISTANCE = {"Home offset move distance", 0x30B1, 0x00};
const SDO QUICK_STOP_DECELERATION = {"Quick stop deceleration", 0x6085, 0x00};
const SDO HOMING_SPEED_ZERO_SEARCH = {"Homing speed for zero search (index)", 0x6099, 0x02};
const SDO HOMING_ACCELERATION = {"Homing acceleration", 0x609A, 0x00};
const SDO HOME_POSITION = {"Home position", 0x30B0, 0x00};
const SDO HOMING_METHOD = {"Homing method", 0x6098, 0x00};

const SDO SOFTWARE_MIN_POSITION_LIMIT = {"Software min position limit", 0x607D, 0x01};
const SDO SOFTWARE_MAX_POSITION_LIMIT = {"Software max position limit", 0x607D, 0x02};
const SDO MAX_MOTOR_SPEED = {"Max motor speed", 0x6080, 0x00};
}

namespace Controlwords {
const Controlword SHUTDOWN = {0x0006, 0x0087};
const Controlword SWITCH_ON = {0x0007, 0x0087};
const Controlword SWITCH_ON_ENABLE_OP = {0x000F, 0x008F};
const Controlword DISABLE_VOLTAGE = {0x0000, 0x0082};
const Controlword QUICK_STOP = {0x0002, 0x0086};
const Controlword DISABLE_OP = {0x0007, 0x008F};
const Controlword ENABLE_OP = {0x000F, 0x008F};
const Controlword FAULT_RESET = {0x0080, 0x0080};

const Controlword HOMING_OP_START = {0x0010, 0x0010};
const Controlword HOMING_HALT_ENABLE = {0x0100, 0x0100};
const Controlword HOMING_HALT_DISABLE = {0x0000, 0x0100};
}

namespace Statuswords {
const Statusword NOT_READY_TO_SWITCH_ON = {0x0000, 0x006F};
const Statusword SWITCH_ON_DISABLED = {0x0040, 0x006F};
const Statusword READY_TO_SWITCH_ON = {0x0021, 0x006F};
const Statusword SWITCHED_ON = {0x0023, 0x006F};
const Statusword OP_ENABLED = {0x0027, 0x006F};
const Statusword QUICK_STOP_ACTIVE = {0x0007, 0x006F};
const Statusword FAULT_REACTION_ACTIVE = {0x000F, 0x006F};
const Statusword FAULT = {0x0008, 0x006F};

const Statusword HOMING_IN_PROGRESS = {0x0000, 0x3400};
const Statusword HOMING_INTERRUPTED = {0x0400, 0x3400};
const Statusword HOMING_SUCCESSFUL = {0x1000, 0x3000};
const Statusword HOMING_ERROR = {0x2000, 0x3000};
}

namespace EposOperatingMode {
/**
 * Mapping between enum and command byte.
 */
const std::map<OperatingMode, uint8_t> OPERATING_MODE = {
    {OperatingMode::PPM, 1},
    {OperatingMode::PVM, 3},
    {OperatingMode::HMM, 6},
    {OperatingMode::CSP, 8},
    {OperatingMode::CSV, 9},
    {OperatingMode::CST, 10}
};

/**
 * Convert byte coming from EPOS to #OperatingMode.
 * @param operatingModeCommand byte coming from EPOS
 * @return corresponding #OperatingMode. If byte is unknown, OperatingMode::UNKNOWN is returned.
 */
static OperatingMode toOperatingMode(const uint8_t &operatingModeCommand) {
  for (const auto &it : OPERATING_MODE) {
    if (it.second == operatingModeCommand) {
      return it.first;
    }
  }

  MELO_ERROR_STREAM("Unknown Operating Mode Command: " << operatingModeCommand);
  return OperatingMode::UNKNOWN;
}

/**
 * Convert #OperatingMode to byte as understood by EPOS.
 * @param operatingMode to be converted
 * @return command byte. If OperatingMode unknown, return 0.
 */
static uint8_t toOperatingModeCommand(const OperatingMode &operatingMode) {
  const auto &it = OPERATING_MODE.find(operatingMode);
  if (it == OPERATING_MODE.end()) {
    return 0;
  }
  return it->second;
}
};

namespace EposDeviceState {
/**
 * Mapping between #DeviceState and its #Statusword.
 */
const std::map<DeviceState, Statusword> DEVICE_STATE_STATUSWORD_MAP
    {
        {DeviceState::STATE_NOT_READY_TO_SWITCH_ON,
         Statuswords::NOT_READY_TO_SWITCH_ON},
        {DeviceState::STATE_SWITCH_ON_DISABLED,
         Statuswords::SWITCH_ON_DISABLED},
        {DeviceState::STATE_READY_TO_SWITCH_ON,
         Statuswords::READY_TO_SWITCH_ON},
        {DeviceState::STATE_SWITCHED_ON,
         Statuswords::SWITCHED_ON},
        {DeviceState::STATE_OP_ENABLED,
         Statuswords::OP_ENABLED},
        {DeviceState::STATE_QUICK_STOP_ACTIVE,
         Statuswords::QUICK_STOP_ACTIVE},
        {DeviceState::STATE_FAULT_REACTION_ACTIVE,
         Statuswords::FAULT_REACTION_ACTIVE},
        {DeviceState::STATE_FAULT, Statuswords::FAULT}
    };

/**
 * Convert #Statusword to corresponding #DeviceState.
 * @param statusword to be converted.
 * @return corresponding DeviceState. If statusword unknown, return DeviceState::STATE_UNKNOWN
 */
static DeviceState toDeviceState(const uint16_t &statusword) {
  for (const auto &it : DEVICE_STATE_STATUSWORD_MAP) {
    if (it.second.isActive(statusword)) {
      MELO_DEBUG_STREAM("Mapping Statusword code " << std::bitset<16>(statusword) << " to state " << Enum::toString(it.first));
      return it.first;
    }
  }

  MELO_ERROR_STREAM("Unknown Statusword: " << statusword);
  return DeviceState::STATE_UNKNOWN;
}
}

namespace EposHomingState {
/**
 * Mapping between #HomingState and its #Statusword.
 */
const std::map<HomingState, Statusword> HOMING_STATE_STATUSWORD_MAP{
    {HomingState::HOMING_IN_PROGRESS, Statuswords::HOMING_IN_PROGRESS},
    {HomingState::HOMING_INTERRUPTED, Statuswords::HOMING_INTERRUPTED},
    {HomingState::HOMING_SUCCESSFUL, Statuswords::HOMING_SUCCESSFUL},
    {HomingState::HOMING_ERROR, Statuswords::HOMING_ERROR}
};

/**
 * Convert #Statusword to corresponding #HomingState
 * @param statusword to be converted.
 * @return corresponding HomingState. If statusword unknown, return HomingState::STATE_UNKNOWN
 */
static HomingState toHomingState(const uint16_t &statusword) {
  for (const auto &it : HOMING_STATE_STATUSWORD_MAP) {
    if (it.second.isActive(statusword)) {
      MELO_DEBUG_STREAM("Mapping Statusword code " << std::bitset<16>(statusword) << " to state " << Enum::toString(it.first));
      return it.first;
    }
  }

  MELO_ERROR_STREAM("Unknown Statusword: " << statusword);
  return HomingState::UNKNOWN;
}
}

namespace EposHomingMethod {
/**
 * Mapping between #HomingMethod and its command byte.
 */
// TODO: add all homing methods
const std::map<HomingMethod, int8_t> HOMING_METHOD_COMMAND_MAP{
    {HomingMethod::INDEX_POSITIVE_SPEED, 34},
    {HomingMethod::INDEX_NEGATIVE_SPEED, 33}
};

/**
 * Convert #HomingMethod to its command byte.
 * @param homingMethod to be converted
 * @return command byte. Return 0 if #HomingMethod unknown.
 */
static uint8_t toHomingMethodCommand(const HomingMethod &homingMethod) {
  const auto &it = HOMING_METHOD_COMMAND_MAP.find(homingMethod);
  if (it == HOMING_METHOD_COMMAND_MAP.end()) {
    return 0;
  }
  return it->second;
}
}

}
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP
