#include "varileg_lowlevel_controller/entities/KneeEncoderCrosschecker.hpp"
#include <cmath>

namespace varileg_lowlevel_controller {
bool KneeEncoderCrosschecker::check(double primaryPosition, double secondaryPosition) {
  if(primaryPosition - secondaryPosition > 0){
    return (primaryPosition - secondaryPosition) < errorMarginPositive_;
  }else{
    return (secondaryPosition - primaryPosition) < errorMarginNegative_;
  }
}
}