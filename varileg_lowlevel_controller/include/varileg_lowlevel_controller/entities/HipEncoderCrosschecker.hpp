//
// Created by wertlin on 16.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

namespace varileg_lowlevel_controller {
class HipEncoderCrosschecker : public EncoderCrosschecker {
 public:
  HipEncoderCrosschecker(double errorMargin) : errorMargin_(errorMargin) {
  }
  virtual bool check(double primaryPosition, double secondaryPosition) override;
 private:
  double errorMargin_;
};
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP
