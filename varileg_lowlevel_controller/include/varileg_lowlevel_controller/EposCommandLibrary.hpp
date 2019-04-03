//
// Created by jolau on 27.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP

#include <cstdint>
#include "string"
#include "OperatingMode.hpp"

namespace varileg_lowlevel_controller {
struct SDO {
  const std::string name;
  const uint16_t index;
  const uint8_t subindex;
};

struct Controlword {
  uint16_t word;
  uint16_t mask;

  void apply(uint16_t &controlword) const {
    controlword = (controlword & ~mask) | (word & mask);
  }
};

struct Statusword {
  uint16_t word;
  uint16_t mask;

  bool isActive(const uint16_t &statusword) const {
    return (statusword & mask) == word;
  };
};

namespace EposCommandLibrary {
namespace EposOperatingMode {
  const std::map<OperatingMode, uint8_t> OPERATING_MODE = {
      {OperatingMode::HOMING, 0x06},
      {OperatingMode::CSP, 0x08}
  };

  static OperatingMode toOperatingMode(const uint8_t &operatingModeCommand) {
    for (const auto &it : OPERATING_MODE) {
      if (it.second == operatingModeCommand) {
        return it.first;
      }
    }

    MELO_ERROR_STREAM("Unknown Operating Mode Command: " << operatingModeCommand);
    return OperatingMode::UNKNOWN;
  }

  static uint8_t toOperatingModeCommand(const OperatingMode &operatingMode) {
    const auto &it =  OPERATING_MODE.find(operatingMode);
    if(it == OPERATING_MODE.end()) {
      return 0;
    }
    return it->second;
  }
};

const int8_t HOMING_METHOD_INDEX_POSITIVE_SPEED = 34;
const int8_t HOMING_METHOD_INDEX_NEGATIVE_SPEED = 33;

namespace SDOs {
const SDO MODES_OF_OPERATION = {"Modes of operation", 0x6060, 0x00};
const SDO MODES_OF_OPERATION_DISPLAY = {"Modes of operation display", 0x6061, 0x00};
const SDO NODE_ID = {"Node-ID", 0x2000, 0x00};
const SDO INTERPOLATION_TIME_PERIOD_VALUE = {"Interpolation time period value", 0x60C2, 0x01};

const SDO FOLLOWING_ERROR_WINDOW = {"Following error window", 0x6065, 0x00};
const SDO HOME_OFFSET_MOVE_DISTANCE = {"Home offset move distance", 0x30B1, 0x00};
const SDO QUICK_STOP_DECELERATION = {"Quick stop deceleration", 0x6085, 0x00};
const SDO HOMING_SPEED_ZERO_SEARCH = {"Homing speed for zero search (index)", 0x6099, 0x02};
const SDO HOMING_ACCELERATION = {"Homing acceleration", 0x609A, 0x00};
const SDO HOME_POSITION = {"Home position", 0x30B0, 0x00};
const SDO HOMING_METHOD = {"Homing method", 0x6098, 0x00};
}

namespace Controlwords {
const Controlword SHUTDOWN =            {0x0006, 0x0087};
const Controlword SWITCH_ON =           {0x0007, 0x0087};
const Controlword SWITCH_ON_ENABLE_OP = {0x000F, 0x008F};
const Controlword DISABLE_VOLTAGE =     {0x0000, 0x0082};
const Controlword QUICK_STOP =          {0x0002, 0x0086};
const Controlword DISABLE_OP =          {0x0007, 0x008F};
const Controlword ENABLE_OP =           {0x000F, 0x008F};
const Controlword FAULT_RESET =         {0x0080, 0x0080};

const Controlword HOMING_OP_START =     {0x0008, 0x0008};
const Controlword HOMING_HALT_ENABLE =  {0x0080, 0x0080};
const Controlword HOMING_HALT_DISABLE = {0x0000, 0x0080};
}

namespace Statuswords {
const Statusword NOT_READY_TO_SWITCH_ON   = {0x0000, 0x006F};
const Statusword SWITCH_ON_DISABLED       = {0x0040, 0x006F};
const Statusword READY_TO_SWITCH_ON       = {0x0021, 0x006F};
const Statusword SWITCHED_ON              = {0x0023, 0x006F};
const Statusword OP_ENABLED               = {0x0027, 0x006F};
const Statusword QUICK_STOP_ACTIVE        = {0x0007, 0x006F};
const Statusword FAULT_REACTION_ACTIVE    = {0x000F, 0x006F};
const Statusword FAULT                    = {0x0008, 0x006F};

const Statusword HOMING_IN_PROGRESS       = {0x0000, 0x1A00};
const Statusword HOMING_INTERRUPTED       = {0x0200, 0x1A00};
const Statusword HOMING_SUCCESSFUL        = {0x0800, 0x1800};
const Statusword HOMING_ERROR             = {0x1000, 0x1800};
}


}
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP
