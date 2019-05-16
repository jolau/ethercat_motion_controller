#ifndef VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

namespace varileg_lowlevel_controller {
/**
 * Crosschecker specific for the knee joint (spring in one direction).
 */
class KneeEncoderCrosschecker : public EncoderCrosschecker {
 public:
  KneeEncoderCrosschecker(double errorMarginPositive, double errorMarginNegative) : errorMarginPositive_(errorMarginPositive), errorMarginNegative_(errorMarginNegative) {
  }
  /**
 * Checks if both encoders don't differ more than a margin. Differentiates between positive and negative direction because of the spring clutch.
 * @param primaryPosition in radian
 * @param secondaryPosition in radian
 * @return true if within #errorMargin_
 */
  virtual bool check(double primaryPosition, double secondaryPosition) override;
 private:
  double errorMarginPositive_; ///< in radian
  double errorMarginNegative_; ///< in radian
};
}
#endif //VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP
