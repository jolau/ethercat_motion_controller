#ifndef VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

namespace varileg_lowlevel_controller {
class KneeEncoderCrosschecker : public EncoderCrosschecker {
 public:
  KneeEncoderCrosschecker(double errorMarginUp, double errorMarginDown) : errorMarginUp_(errorMarginUp), errorMarginDown_(errorMarginDown) {
  }
  virtual bool check(double primaryPosition, double secondaryPosition) override;
 private:
  double errorMarginUp_;
  double errorMarginDown_;
};
}
#endif //VARILEG_LOWLEVEL_CONTROLLER_KNEEENCODERCROSSCHECKER_HPP
