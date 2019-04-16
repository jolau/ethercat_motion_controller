//
// Created by jolau on 13.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

class HipEncoderCrosschecker : public EncoderCrosschecker {
 public:
  HipEncoderCrosschecker(double errorMargin) : errorMargin_(errorMargin) {
  }
  virtual bool check(double primaryPosition, double secondaryPosition) override;
 private:
  double errorMargin_;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP
