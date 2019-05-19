//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSSTARTUPCONFIG_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSSTARTUPCONFIG_HPP

#include <cstdint>
#include "PositionUnitConverter.hpp"

namespace varileg_lowlevel_controller {
/**
 * Configuration which is applied on Startup of EPOS slave. Is not Joint Specific.
 */
struct EposStartupConfig {
  uint8_t interpolationTimePeriod; ///< cycle time of position update loop has to match
  uint32_t motorCurrentLimit; ///< in mA
};
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSSTARTUPCONFIG_HPP
