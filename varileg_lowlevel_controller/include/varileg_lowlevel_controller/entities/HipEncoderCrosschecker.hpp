//
// Created by wertlin on 16.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

namespace varileg_lowlevel_controller {
/**
 * Crosschecker specific for Hip (no spring, but with belt).
 */
class HipEncoderCrosschecker : public EncoderCrosschecker {
 public:
  HipEncoderCrosschecker(double errorMargin) : errorMargin_(errorMargin) {
  }
  /**
   * Checks if both encoders don't differ more than #errorMargin_
   * @param primaryPosition in radian
   * @param secondaryPosition in radian
   * @return true if within #errorMargin_
   */
  virtual bool check(double primaryPosition, double secondaryPosition) override;
 private:
  double errorMargin_;  ///< in radian
};
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP
