#pragma once

#include "any_node/Node.hpp"
#include <std_msgs/String.h>
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
    typedef actionlib::SimpleActionServer <varileg_msgs::DeviceStateAction>  DeviceState;
    
    //Client Messages
    varileg_msgs::ExtendedDeviceState msg_device_state_;
    
    //Action Messages
    varileg_msgs::DeviceStateFeedback device_state_feedback_;

    public:
    ExampleNode() = delete;  
    ExampleNode(any_node::Node::NodeHandlePtr nh): any_node::Node(nh){}
    ~ExampleNode() override{}
    bool init() override;
    void cleanup() override;
    void preCleanup() override;
    bool update(const any_worker::WorkerEvent& event);
    void subscriberCallback(const std_msgs::StringConstPtr &msg);
    
    //Subscriber
    void jointTrajectoryCb (const varileg_msgs::ExtendedJointTrajectoryConstPtr &msg);

    //Action Server
    void deviceStateCb (const varileg_msgs::DeviceStateGoalConstPtr &goal, DeviceState* as);
    void homingCb (const varileg_msgs::HomingActionGoalConstPtr &goal);
  };

}
