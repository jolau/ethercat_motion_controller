#pragma once

#include "any_node/Node.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include "actionlib/server/simple_action_server.h"

#include "varileg_msgs/ExtendedJointStates.h"
#include "varileg_msgs/ExtendedDeviceState.h"
#include "varileg_msgs/ExtendedJointTrajectory.h"

#include "varileg_msgs/DeviceStateAction.h"
#include "varileg_msgs/HomingAction.h"

namespace lowlvl_communication 
{
  class ExampleNode : public any_node::Node 
  {
    private:
    //Publisher and subscribers
    ros::Subscriber sub_joint_trajectory_;
    ros::Publisher pub_joint_state_;
    ros::Publisher pub_device_state_;
      
    //Action Servers
    actionlib::SimpleActionServer <varileg_msgs::DeviceStateAction>  as_device_state_;
    actionlib::SimpleActionServer <varileg_msgs::HomingAction> as_homing_;
      
    //Client Messages
    varileg_msgs::ExtendedDeviceState msg_device_state_;
    varileg_msgs::ExtendedJointStates msg_joint_state_;
    
    //Action Messages
    varileg_msgs::DeviceStateFeedback feedback_device_state_;
    varileg_msgs::DeviceStateResult result_device_state_;

    varileg_msgs::HomingFeedback feedback_homing_;
    varileg_msgs::HomingResult result_homing_;

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
    void jointTrajectoryCb (const varileg_msgs::ExtendedJointTrajectoryConstPtr &msg);

    //Action Server
    void deviceStateCb (const varileg_msgs::DeviceStateGoalConstPtr &goal);
    void homingCb (const varileg_msgs::HomingGoalConstPtr &goal);
  };

}
