//
// Created by jolau on 28.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSEXAMPLENODE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSEXAMPLENODE_HPP

#include "any_node/Node.hpp"

#include "soem_interface/EthercatBusBase.hpp"
#include "std_msgs/String.h"
#include "varileg_lowlevel_controller/EposEthercatSlaveManager.hpp"

namespace varileg_lowlevel_controller {
namespace examples {
class EposExampleNode : public any_node::Node {
 public:
  EposExampleNode() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
  EposExampleNode(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh), eposEthercatSlaveManager_(std::make_shared<EposEthercatSlaveManager>()) {
    bus_ = std::make_shared<soem_interface::EthercatBusBase>("ens9");
  }

  ~EposExampleNode() override {
  }

  // these two functions need to be implemented
  bool init() override;
  void cleanup() override;

  // this function implementation is optional (default is empty)
  void preCleanup() override;

  bool update(const any_worker::WorkerEvent &event);

  void subscriberCallback(const std_msgs::StringConstPtr &msg);

 private:
  soem_interface::EthercatBusBasePtr bus_;
  std::vector<soem_interface::EthercatSlaveBasePtr> slaves_;
  EposEthercatSlaveManagerPtr eposEthercatSlaveManager_;
  bool goUp = true;
};
}
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSEXAMPLENODE_HPP
