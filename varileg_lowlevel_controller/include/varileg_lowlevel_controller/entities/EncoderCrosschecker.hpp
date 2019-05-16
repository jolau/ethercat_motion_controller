//
// Created by jolau on 13.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP

namespace varileg_lowlevel_controller {

/**
 * Interface for Encoder Crosschecker between the primary and secondary position.
 */
class EncoderCrosschecker {
 public:
  /**
   * Crosscheck both encoder position.
   * @param primaryPosition in radian
   * @param secondaryPosition in radian
   * @return true if check is in bounds
   */
  virtual bool check(double primaryPosition, double secondaryPosition) = 0;
};

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP
