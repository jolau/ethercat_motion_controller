#pragma once

#include "any_node/Node.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include "actionlib/server/simple_action_server.h"

#include "varileg_msgs/ExtendedJointStates.h"
#include "varileg_msgs/ExtendedDeviceStates.h"
#include "varileg_msgs/ExtendedJointTrajectories.h"

#include "varileg_msgs/DeviceStateAction.h"
#include "varileg_msgs/HomingAction.h"

namespace lowlvl_communication 
{
  class ExampleNode : public any_node::Node 
  {
    private:
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

    public:
    ExampleNode() = delete;  
    ExampleNode(any_node::Node::NodeHandlePtr nh);
    ~ExampleNode() override{}
    bool init() override;
    void cleanup() override;
    void preCleanup() override;
    bool update(const any_worker::WorkerEvent& event);
    void subscriberCallback(const std_msgs::Float32ConstPtr &msg);
    
    //Subscriber
    void jointTrajectoriesCallback(const varileg_msgs::ExtendedJointTrajectoriesConstPtr &msg);

    //Action Server
    void deviceStateCallback(const varileg_msgs::DeviceStateGoalConstPtr &goal);
    void homingCallback(const varileg_msgs::HomingGoalConstPtr &goal);
  };

}
