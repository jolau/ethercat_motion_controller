//
// Created by wertlin on 16.04.19.
//



#ifndef VARILEG_LOWLEVEL_CONTROLLER_NOENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_NOENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

namespace varileg_lowlevel_controller {

/**
 * Mock crosschecker which always return true.
 */
class NoEncoderCrosschecker : public EncoderCrosschecker {
 public:
  NoEncoderCrosschecker(){}
  /**
   * ALWAYS returns true
   * @param primaryPosition
   * @param secondaryPosition
   * @return alway true
   */
  virtual bool check(double primaryPosition, double secondaryPosition) override { return true; };
};

}
#endif //VARILEG_LOWLEVEL_CONTROLLER_NOENCODERCROSSCHECKER_HPP
