#ifndef VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

namespace varileg_lowlevel_controller {
class EncoderCrosschecker  {
 public:
  EncoderCrosschecker(double errorMarginPositive, double errorMarginNegative) : errorMarginPositive_(errorMarginPositive), errorMarginNegative_(errorMarginNegative) {
  }
  bool check(double primaryPosition, double secondaryPosition) {
    if(primaryPosition - secondaryPosition > 0){
      return (primaryPosition - secondaryPosition) < errorMarginPositive_;
    }else{
      return (secondaryPosition - primaryPosition) < errorMarginNegative_;
    }
  };
 private:
  double errorMarginPositive_;
  double errorMarginNegative_;
};
}
#endif //VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP
