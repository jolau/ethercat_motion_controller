//
// Created by wertlin on 16.04.19.
//

#include "varileg_lowlevel_controller/entities/HipEncoderCrosschecker.hpp"
#include <cmath>

bool HipEncoderCrosschecker::check(double primaryPosition, double secondaryPosition) {
  if(std::abs((double) primaryPosition - secondaryPosition) < errorMargin_)return true;
  else return false;
}