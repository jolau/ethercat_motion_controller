#include "any_node/Node.hpp"

#include <std_msgs/String.h>

#include "VarilegEthercatBusManager.hpp"
#include "EposEthercatSlave.hpp"
#include "soem_interface/EthercatBusBase.hpp"
#include "EposEthercatSlaveManager.hpp"

namespace varileg_lowlevel_controller {
class EthercatNode : public any_node::Node {
 public:
  EthercatNode() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
  EthercatNode(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh), busManager_(std::make_shared<VarilegEthercatBusManager>()), eposEthercatSlaveManager_(std::make_shared<EposEthercatSlaveManager>()) {
  }

  ~EthercatNode() override {
  }

  // these two functions need to be implemented
  bool init() override;
  void cleanup() override;

  // this function implementation is optional (default is empty)
  void preCleanup() override;

  bool update(const any_worker::WorkerEvent &event);

  void subscriberCallback(const std_msgs::StringConstPtr &msg);

 private:
  VarilegEthercatBusManagerPtr busManager_;
  std::map<std::string, std::vector<soem_interface::EthercatSlaveBasePtr>> slavesOfBusesMap_;
  EposEthercatSlaveManagerPtr eposEthercatSlaveManager_;

  void setupBusManager();
};

}
