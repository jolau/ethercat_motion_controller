//
// Created by jolau on 17.05.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_JOINTSPECIFICATIONS_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_JOINTSPECIFICATIONS_HPP

#include <memory>
#include "EncoderCrosschecker.hpp"
#include "PositionUnitConverter.hpp"

namespace varileg_lowlevel_controller {

/**
 * Collection of hardware specific specification of a Joint.
 */
struct JointSpecifications {
  PositionUnitConverter primaryEncoderConverter{1};
  PositionUnitConverter secondaryEncoderConverter{1};
  EncoderCrosschecker encoderCrosschecker{0, 0};
  double homeOffset{0};
  double minPositionLimit{0};
  double maxPositionLimit{0};
};
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_JOINTSPECIFICATIONS_HPP
