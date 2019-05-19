//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_JOINTSTATE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_JOINTSTATE_HPP

namespace varileg_lowlevel_controller {
/**
 * Status of Joint as read out from EPOS.
 */
struct JointState {
  double position; ///< Homed position of RSF encoder. In radian.
  double primaryPosition; ///< Not homed position of RSF encoder. In radian.
  double secondaryPosition; ///< Not homed position of MILE encoder. In radian.
  double velocity; //< in rpm
  double torque; //< in per thousand of “Motor rated torque”
};
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_JOINTSTATE_HPP
