#ifndef VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

namespace varileg_lowlevel_controller {
class KneeEncoderCrosschecker : public EncoderCrosschecker {
 public:
  KneeEncoderCrosschecker(double errorMarginPositive, double errorMarginNegative) : errorMarginPositive_(errorMarginPositive), errorMarginNegative_(errorMarginNegative) {
  }
  virtual bool check(double primaryPosition, double secondaryPosition) override;
 private:
  double errorMarginPositive_;
  double errorMarginNegative_;
};
}
#endif //VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP
