#include "any_node/Node.hpp"

#include <std_msgs/String.h>

#include "VarilegEthercatBusManager.hpp"
#include "EposEthercatSlave.hpp"
#include "soem_interface/EthercatBusBase.hpp"
#include "EposEthercatSlaveManager.hpp"

#include "varileg_msgs/ExtendedJointStates.h"
#include "varileg_msgs/ExtendedDeviceStates.h"
#include "varileg_msgs/ExtendedJointTrajectories.h"

#include "actionlib/server/simple_action_server.h"
#include "varileg_msgs/DeviceStateAction.h"
#include "varileg_msgs/HomingAction.h"

namespace varileg_lowlevel_controller {
class EthercatNode : public any_node::Node {
 public:
  EthercatNode() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
  EthercatNode(any_node::Node::NodeHandlePtr nh)
      : any_node::Node(nh),
        busManager_(std::make_shared<VarilegEthercatBusManager>()),
        eposEthercatSlaveManager_(std::make_shared<EposEthercatSlaveManager>()),
        deviceStateActionServer_(*nh, "DeviceState", boost::bind (&EthercatNode::deviceStateCallback, this, _1),false),
        homingActionServer_(*nh, "Homing" , boost::bind(&EthercatNode::homingCallback, this, _1),false)
        {
    //Action Server
    deviceStateActionServer_.start();
    homingActionServer_.start();
  }

  ~EthercatNode() override {
  }

  // these two functions need to be implemented
  bool init() override;
  void cleanup() override;

  // this function implementation is optional (default is empty)
  void preCleanup() override;

  bool update(const any_worker::WorkerEvent &event);

  //Subscriber
  void jointTrajectoriesCallback(const varileg_msgs::ExtendedJointTrajectoriesConstPtr &msg);

  //Action Server
  void deviceStateCallback(const varileg_msgs::DeviceStateGoalConstPtr &goal);
  void homingCallback(const varileg_msgs::HomingGoalConstPtr &goal);

 private:
  VarilegEthercatBusManagerPtr busManager_;
  std::map<std::string, std::vector<soem_interface::EthercatSlaveBasePtr>> slavesOfBusesMap_;
  EposEthercatSlaveManagerPtr eposEthercatSlaveManager_;
  bool goUp = true;

  //Publisher and subscribers
  ros::Subscriber jointTrajectoriesSubscriber_;
  ros::Publisher jointStatesPublisher_;
  ros::Publisher deviceStatePublisher_;

  //Action Servers
  actionlib::SimpleActionServer <varileg_msgs::DeviceStateAction>  deviceStateActionServer_;
  actionlib::SimpleActionServer <varileg_msgs::HomingAction> homingActionServer_;

  //Client Messages
  varileg_msgs::ExtendedDeviceStates extendedDeviceStates_;
  varileg_msgs::ExtendedJointStates extendedJointStates_;

  //Action Messages
  varileg_msgs::DeviceStateFeedback deviceStateFeedback_;
  varileg_msgs::DeviceStateResult deviceStateResult_;

  varileg_msgs::HomingFeedback homingFeedback_;
  varileg_msgs::HomingResult homingResult_;

  void setupBusManager();
};

}
