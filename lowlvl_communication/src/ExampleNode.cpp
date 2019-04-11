#include "lowlvl_communication/ExampleNode.hpp"

namespace lowlvl_communication {

ExampleNode::ExampleNode(any_node::Node::NodeHandlePtr nh): any_node::Node(nh), 
   as_device_state_hip_right_(*this->nh_, "DeviceStateHipRight", boost::bind (&ExampleNode::deviceStateCbHipRight, this, _1),false),
   as_device_state_hip_left_(*this->nh_, "DeviceStateHipLeft", boost::bind (&ExampleNode::deviceStateCbHipLeft, this, _1),false),
   as_device_state_knee_right_(*this->nh_, "DeviceStateKneeRight", boost::bind (&ExampleNode::deviceStateCbKneeRight, this, _1),false),
   as_device_state_knee_left_(*this->nh_, "DeviceStateKneeLeft", boost::bind (&ExampleNode::deviceStateCbKneeLeft, this, _1),false),
   
   as_homing_hip_right_(*this->nh_.get(), "HomingHipRight" , boost::bind(&ExampleNode::homingCbHipRight, this, _1),false),
   as_homing_hip_left_(*this->nh_.get(), "HomingHipLeft" , boost::bind(&ExampleNode::homingCbHipLeft, this, _1),false),
   as_homing_knee_right_(*this->nh_.get(), "HomingKneeRight" , boost::bind(&ExampleNode::homingCbKneeRight, this, _1),false),
   as_homing_knee_left_(*this->nh_.get(), "HomingKneeLeft" , boost::bind(&ExampleNode::homingCbKneeLeft, this, _1),false) 
{
  //Action Server
  as_device_state_hip_right_.start();
  as_device_state_hip_left_.start();
  as_device_state_knee_right_.start();
  as_device_state_knee_left_.start();

  as_homing_hip_right_.start();
  as_homing_hip_left_.start();
  as_homing_knee_right_.start();
  as_homing_knee_left_.start();

  //Publisher
  pub_joint_state_= advertise<varileg_msgs::ExtendedJointStates>("joint_states","joint_states",1);
  pub_device_state_= advertise<varileg_msgs::ExtendedDeviceStates>("device_states","device_states",1);
}
   
bool ExampleNode::init() 
{
  constexpr double defaultWorkerTimeStep = 3.0;
  constexpr int priority = 10;
  double workerTimeStep = param<double>("time_step", defaultWorkerTimeStep);
  addWorker("exampleNode::updateWorker", workerTimeStep, &ExampleNode::update, this, priority);
  constexpr unsigned int defaultQueueSize = 1;

  ros::Publisher my_publisher = advertise<std_msgs::String>("my_publisher_name", "/default_publisher_topic", defaultQueueSize);
  ros::Subscriber my_subscriber = subscribe("my_subscriber_name", "/default_subscriber_topic", defaultQueueSize, &ExampleNode::subscriberCallback, this);
  
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ////////////// New stuff                                                          ////////////
  /////////////////////////////////////////////////////////////////////////////////////////////
  
  //Subscriber
  sub_joint_trajectory_ = subscribe("joint_trajectories", "joint_trajectories", defaultQueueSize, &ExampleNode::jointTrajectoryCb, this);
   
  return true;
}

//Subscriber
void ExampleNode::jointTrajectoryCb (const varileg_msgs::ExtendedJointTrajectoriesConstPtr &msg)
{
  MELO_INFO("Received jointTrajectoryCb Topic");
  msg->name[0];
  msg->position[0];
  /* @Jonas
  Fülle die Einträge mit Daten
  */
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Device State Action Server                                                              /////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExampleNode::deviceStateCbHipRight (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  varileg_msgs::DeviceStateFeedback feedback_device_state;
  varileg_msgs::DeviceStateResult result_device_state;

  MELO_INFO("Received deviceStateCb Action");
  goal->target_device_state;
  
  /* @Jonas
  feedback_device_state.current_device_state = [...]; 
  as_device_state_hip_right_.publishFeedback(feedback_device_state);
  result_device_state.successful = [...];
  result_device_state.reached_device_state = [...];
  */
  
  as_device_state_hip_right_.setSucceeded(result_device_state);
}

void ExampleNode::deviceStateCbHipLeft (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  varileg_msgs::DeviceStateFeedback feedback_device_state;
  varileg_msgs::DeviceStateResult result_device_state;

  MELO_INFO("Received deviceStateCb Action");
  goal->target_device_state;
  
  /* @Jonas
  feedback_device_state.current_device_state = [...]; 
  as_device_state_hip_left_.publishFeedback(feedback_device_state);
  result_device_state.successful = [...];
  result_device_state.reached_device_state = [...];
  */
  
  as_device_state_hip_left_.setSucceeded(result_device_state);
}

void ExampleNode::deviceStateCbKneeRight (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  varileg_msgs::DeviceStateFeedback feedback_device_state;
  varileg_msgs::DeviceStateResult result_device_state;

  MELO_INFO("Received deviceStateCb Action");
  goal->target_device_state;
  
  /* @Jonas
  feedback_device_state.current_device_state = [...]; 
  as_device_state_knee_right_.publishFeedback(feedback_device_state);
  result_device_state.successful = [...];
  result_device_state.reached_device_state = [...];
  */
  
  as_device_state_knee_right_.setSucceeded(result_device_state);
}

void ExampleNode::deviceStateCbKneeLeft (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  varileg_msgs::DeviceStateFeedback feedback_device_state;
  varileg_msgs::DeviceStateResult result_device_state;

  MELO_INFO("Received deviceStateCb Action");
  goal->target_device_state;
  
  /* @Jonas
  feedback_device_state.current_device_state = [...]; 
  as_device_state_knee_left_.publishFeedback(feedback_device_state);
  result_device_state.successful = [...];
  result_device_state.reached_device_state = [...];
  */
  
  as_device_state_knee_left_.setSucceeded(result_device_state);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Homing Action Server                                                                    /////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExampleNode::homingCbHipRight (const varileg_msgs::HomingGoalConstPtr &goal)
{
  varileg_msgs::HomingFeedback feedback_homing;
  varileg_msgs::HomingResult result_homing;
  MELO_INFO("Received homingCb Action");

  /* @Jonas
  goal -> ?
  feedback_homing.interrupted = [...];
  as_homing_hip_right_.publishFeedback(feedback_homing);
  result_homing.successful = [...];
  */   
  as_homing_hip_right_.setSucceeded(result_homing); 
}

void ExampleNode::homingCbHipLeft (const varileg_msgs::HomingGoalConstPtr &goal)
{
  varileg_msgs::HomingFeedback feedback_homing;
  varileg_msgs::HomingResult result_homing;
  MELO_INFO("Received homingCb Action");

  /* @Jonas
  goal -> ?
  feedback_homing.interrupted = [...];
  as_homing_hip_left_.publishFeedback(feedback_homing);
  result_homing.successful = [...];
  */   
  as_homing_hip_left_.setSucceeded(result_homing); 
}

void ExampleNode::homingCbKneeRight (const varileg_msgs::HomingGoalConstPtr &goal)
{
  varileg_msgs::HomingFeedback feedback_homing;
  varileg_msgs::HomingResult result_homing;
  MELO_INFO("Received homingCb Action");

  /* @Jonas
  goal -> ?
  feedback_homing.interrupted = [...];
  as_homing_knee_right_.publishFeedback(feedback_homing);
  result_homing.successful = [...];
  */   
  as_homing_knee_right_.setSucceeded(result_homing); 
}

void ExampleNode::homingCbKneeLeft (const varileg_msgs::HomingGoalConstPtr &goal)
{
  varileg_msgs::HomingFeedback feedback_homing;
  varileg_msgs::HomingResult result_homing;
  MELO_INFO("Received homingCb Action");

  /* @Jonas
  goal -> ?
  feedback_homing.interrupted = [...];
  as_homing_knee_left_.publishFeedback(feedback_homing);
  result_homing.successful = [...];
  */   
  as_homing_knee_left_.setSucceeded(result_homing); 
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Publisher                                                                               /////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ExampleNode::update(const any_worker::WorkerEvent& event)
{  
  pub_joint_state_.publish(msg_joint_state_);
  pub_device_state_.publish(msg_device_state_);
  
  return true;
}
////////////////////////////////////////////////////////////////////////
/////////// Old stuff                                          ////////
//////////////////////////////////////////////////////////////////////

void ExampleNode::cleanup()
{
  MELO_INFO("cleanup called");
}

void ExampleNode::preCleanup()
{
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
}

void ExampleNode::subscriberCallback(const std_msgs::Float32ConstPtr &msg)
{
  // called asynchrounously when ros messages arrive for the subscriber created in init() function
  MELO_INFO("received ros message: %f", msg->data);
}

}
