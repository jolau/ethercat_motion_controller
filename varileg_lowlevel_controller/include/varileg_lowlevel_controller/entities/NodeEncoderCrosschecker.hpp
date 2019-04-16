//
// Created by wertlin on 16.04.19.
//



#ifndef VARILEG_LOWLEVEL_CONTROLLER_NODEENCODERCROSSCHECKER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_NODEENCODERCROSSCHECKER_HPP

#include "EncoderCrosschecker.hpp"

class NodeEncoderCrosschecker : public EncoderCrosschecker {
 public:
  NodeEncoderCrosschecker(){}
  virtual bool check(double primaryPosition, double secondaryPosition) override;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_NODEENCODERCROSSCHECKER_HPP
