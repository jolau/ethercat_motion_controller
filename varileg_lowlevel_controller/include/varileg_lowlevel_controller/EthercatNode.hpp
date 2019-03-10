#include "any_node/Node.hpp"

#include <std_msgs/String.h>

namespace varileg_lowlevel_controller {
class EthercatNode : public any_node::Node {
public:
    EthercatNode() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
    EthercatNode(any_node::Node::NodeHandlePtr nh): any_node::Node(nh)
  {
  }

  ~EthercatNode() override
  {
  }

  // these two functions need to be implemented
  bool init() override;
  void cleanup() override;

  // this function implementation is optional (default is empty)
  void preCleanup() override;

  bool update(const any_worker::WorkerEvent& event);

  void subscriberCallback(const std_msgs::StringConstPtr &msg);
};

}
