//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_JOINTTRAJECTORY_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_JOINTTRAJECTORY_HPP

namespace varileg_lowlevel_controller {
/**
 * Target to be sent to EPOS. Only use one member, corresponding to selected drive mode.
 */
struct JointTrajectory {
  double position; ///< in radian
  double velocity; //< in rpm
  double torque; //< in per thousand of “Motor rated torque”
};
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_JOINTTRAJECTORY_HPP
