//
// Created by jolau on 13.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

class HipEncoderCrosschecker : public EncoderCrosschecker {
 public:
  HipEncoderCrosschecker(double errorMargin) : errorMargin(errorMargin) {}
  virtual bool check(double primaryPosition, double secondaryPosition) override;
 private:
  double errorMargin;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_HIPENCODERCROSSCHECKER_HPP
