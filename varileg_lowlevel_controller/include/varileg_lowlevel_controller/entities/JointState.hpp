//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_JOINTSTATE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_JOINTSTATE_HPP

struct JointState {
  double position; // homed position of RSF encoder
  double primaryPosition; // not homed position of RSF encoder
  double secondaryPosition; // not homed position of MILE encoder
  double positionDifference; //RSF - MILE encoder
  double velocity;
  double torque;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_JOINTSTATE_HPP
