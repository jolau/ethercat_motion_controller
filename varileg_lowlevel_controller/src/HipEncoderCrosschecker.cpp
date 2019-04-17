//
// Created by wertlin on 16.04.19.
//

#include "varileg_lowlevel_controller/entities/HipEncoderCrosschecker.hpp"
#include <cmath>

bool HipEncoderCrosschecker::check(double primaryPosition, double secondaryPosition) {
  return std::abs(primaryPosition - secondaryPosition) < errorMargin_;
}