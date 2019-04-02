//
// Created by jolau on 29.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EXTENDEDJOINTSTATE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EXTENDEDJOINTSTATE_HPP

#include "MotorControllerState.hpp"

namespace varileg_lowlevel_controller {

struct ExtendedJointState {
  MotorControllerState motorControllerState;
  float position;
};

}
#endif //VARILEG_LOWLEVEL_CONTROLLER_EXTENDEDJOINTSTATE_HPP
