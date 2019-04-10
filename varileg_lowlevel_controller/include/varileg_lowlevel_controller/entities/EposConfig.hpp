//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSCONFIG_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSCONFIG_HPP

#include <cstdint>
#include "PositionUnitConverter.hpp"

struct EposConfig {
  int32_t minPositionLimitInc;
  int32_t maxPositionLimitInc;
  uint32_t maxMotorSpeedRpm;
  PositionUnitConverter primaryEncoderConverter;
  PositionUnitConverter secondaryEncoderConverter;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSCONFIG_HPP
