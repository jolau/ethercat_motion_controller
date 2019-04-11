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
    ros::Subscriber sub_joint_trajectory_;
    ros::Publisher pub_joint_state_;
    ros::Publisher pub_device_state_;
      
    //Action Servers
    actionlib::SimpleActionServer <varileg_msgs::DeviceStateAction>  as_device_state_hip_right_;
    actionlib::SimpleActionServer <varileg_msgs::DeviceStateAction>  as_device_state_hip_left_;
    actionlib::SimpleActionServer <varileg_msgs::DeviceStateAction>  as_device_state_knee_right_;
    actionlib::SimpleActionServer <varileg_msgs::DeviceStateAction>  as_device_state_knee_left_;
    
    actionlib::SimpleActionServer <varileg_msgs::HomingAction> as_homing_hip_right_;
    actionlib::SimpleActionServer <varileg_msgs::HomingAction> as_homing_hip_left_;
    actionlib::SimpleActionServer <varileg_msgs::HomingAction> as_homing_knee_right_;
    actionlib::SimpleActionServer <varileg_msgs::HomingAction> as_homing_knee_left_;
    
      
    //Client Messages
    varileg_msgs::ExtendedDeviceStates msg_device_state_;
    varileg_msgs::ExtendedJointStates msg_joint_state_;
    
    
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
    void jointTrajectoryCb (const varileg_msgs::ExtendedJointTrajectoriesConstPtr &msg);

    //Action Server
    void deviceStateCbHipRight (const varileg_msgs::DeviceStateGoalConstPtr &goal);
    void deviceStateCbHipLeft (const varileg_msgs::DeviceStateGoalConstPtr &goal);
    void deviceStateCbKneeRight (const varileg_msgs::DeviceStateGoalConstPtr &goal);
    void deviceStateCbKneeLeft (const varileg_msgs::DeviceStateGoalConstPtr &goal);
    
    void homingCbHipRight (const varileg_msgs::HomingGoalConstPtr &goal);
    void homingCbHipLeft (const varileg_msgs::HomingGoalConstPtr &goal);
    void homingCbKneeRight (const varileg_msgs::HomingGoalConstPtr &goal);
    void homingCbKneeLeft (const varileg_msgs::HomingGoalConstPtr &goal);
  };

}
