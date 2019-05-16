//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSSTARTUPCONFIG_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSSTARTUPCONFIG_HPP

#include <cstdint>
#include "PositionUnitConverter.hpp"

struct EposStartupConfig {
  uint8_t interpolationTimePeriod;
  uint32_t motorCurrentLimit;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSSTARTUPCONFIG_HPP
