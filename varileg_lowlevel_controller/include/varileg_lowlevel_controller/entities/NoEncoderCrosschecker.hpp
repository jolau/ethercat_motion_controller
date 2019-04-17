//
// Created by wertlin on 16.04.19.
//



#ifndef VARILEG_LOWLEVEL_CONTROLLER_NOENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_NOENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

namespace varileg_lowlevel_controller {

class NoEncoderCrosschecker : public EncoderCrosschecker {
 public:
  NoEncoderCrosschecker(){}
  virtual bool check(double primaryPosition, double secondaryPosition) override;
};

}
#endif //VARILEG_LOWLEVEL_CONTROLLER_NOENCODERCROSSCHECKER_HPP
