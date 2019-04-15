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
#include "varileg_msgs/HomingAction.h"
#include "varileg_msgs/SetOperatingMode.h"
#include "varileg_msgs/SetDeviceState.h"

namespace varileg_lowlevel_controller {
class EthercatNode : public any_node::Node {
 public:
  EthercatNode() = delete; // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
  EthercatNode(any_node::Node::NodeHandlePtr nh)
      : any_node::Node(nh),
        busManager_(std::make_shared<VarilegEthercatBusManager>()),
        eposEthercatSlaveManager_(std::make_shared<EposEthercatSlaveManager>()),
        homingActionServerHipRight_(*nh_, "homing/hip_right" , boost::bind(&EthercatNode::homingCallbackHipRight, this, _1),false),
        homingActionServerHipLeft_(*nh_, "homing/hip_left" , boost::bind(&EthercatNode::homingCallbackHipLeft, this, _1),false),
        homingActionServerKneeRight_(*nh_, "homing/knee_right" , boost::bind(&EthercatNode::homingCallbackKneeRight, this, _1),false),
        homingActionServerKneeLeft_(*nh_, "homing/knee_left" , boost::bind(&EthercatNode::homingCallbackKneeLeft, this, _1),false),
        isStopped(false)
        {
    //Action Server
          homingActionServerHipRight_.start();
          homingActionServerHipLeft_.start();
          homingActionServerKneeRight_.start();
          homingActionServerKneeLeft_.start();
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
  void homingCallback(actionlib::SimpleActionServer<varileg_msgs::HomingAction> &homingActionServer, const std::string &name, const varileg_msgs::HomingGoalConstPtr &goal);
  void homingCallbackHipRight (const varileg_msgs::HomingGoalConstPtr &goal);
  void homingCallbackHipLeft (const varileg_msgs::HomingGoalConstPtr &goal);
  void homingCallbackKneeRight (const varileg_msgs::HomingGoalConstPtr &goal);
  void homingCallbackKneeLeft (const varileg_msgs::HomingGoalConstPtr &goal);

  // Service
  bool setOperatingModeCallback(varileg_msgs::SetOperatingModeRequest& request, varileg_msgs::SetOperatingModeResponse& response);
  bool setDeviceStateCallback(varileg_msgs::SetDeviceStateRequest& request, varileg_msgs::SetDeviceStateResponse& response);

 private:
  VarilegEthercatBusManagerPtr busManager_;
  std::map<std::string, std::vector<soem_interface::EthercatSlaveBasePtr>> slavesOfBusesMap_;
  EposEthercatSlaveManagerPtr eposEthercatSlaveManager_;
  bool goUp = true;
  std::atomic_bool isStopped;

  //Publisher and subscribers
  ros::Subscriber jointTrajectoriesSubscriber_;
  ros::Publisher jointStatesPublisher_;
  ros::Publisher deviceStatePublisher_;

  // Sevices
  ros::ServiceServer deviceStateServiceServer_;
  ros::ServiceServer operatingModeServiceServer_;

  //Action Servers
  actionlib::SimpleActionServer <varileg_msgs::HomingAction> homingActionServerHipRight_;
  actionlib::SimpleActionServer <varileg_msgs::HomingAction> homingActionServerHipLeft_;
  actionlib::SimpleActionServer <varileg_msgs::HomingAction> homingActionServerKneeRight_;
  actionlib::SimpleActionServer <varileg_msgs::HomingAction> homingActionServerKneeLeft_;

  void setupBusManager();
};

}
