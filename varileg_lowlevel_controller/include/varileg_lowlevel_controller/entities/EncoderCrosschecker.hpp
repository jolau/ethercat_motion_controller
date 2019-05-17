#ifndef VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_ENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

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
