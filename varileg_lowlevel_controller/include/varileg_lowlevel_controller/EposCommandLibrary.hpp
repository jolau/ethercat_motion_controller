//
// Created by jolau on 27.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP

#include <cstdint>
#include "string"

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
namespace SDOs {
const SDO MODES_OF_OPERATION = {"Modes of operation", 0x6060, 0x00};
const SDO MODES_OF_OPERATION_DISPLAY = {"Modes of operation display", 0x6061, 0x00};
const SDO NODE_ID = {"Node-ID", 0x2000, 0x00};
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
}


}
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSSDOLIBRARY_HPP
