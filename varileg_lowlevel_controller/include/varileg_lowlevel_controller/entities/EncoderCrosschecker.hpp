//
// Created by jolau on 13.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP

namespace varileg_lowlevel_controller {

class EncoderCrosschecker {
 public:
  virtual bool check(double primaryPosition, double secondaryPosition) = 0;
};

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP
