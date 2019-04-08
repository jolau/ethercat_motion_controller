#include "lowlvl_communication/ExampleNode.hpp"

namespace lowlvl_communication {

ExampleNode::ExampleNode(any_node::Node::NodeHandlePtr nh): any_node::Node(nh), 
   as_device_state_(*this->nh_, "DeviceState", boost::bind (&ExampleNode::deviceStateCb, this, _1),false),
   as_homing_(*this->nh_.get(), "Homing" , boost::bind(&ExampleNode::homingCb, this, _1),false)    
{
  //Action Server
  as_device_state_.start();
  as_homing_.start();

  //Publisher
  pub_joint_state_= advertise<varileg_msgs::ExtendedJointStates>("joint_state","joint_state",1);
  pub_device_state_= advertise<varileg_msgs::ExtendedDeviceState>("device_state","device_state",1);
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
  sub_joint_trajectory_ = subscribe("joint_trajectory", "joint_trajectory", defaultQueueSize, &ExampleNode::jointTrajectoryCb, this);
   
  return true;
}

//Subscriber
void ExampleNode::jointTrajectoryCb (const varileg_msgs::ExtendedJointTrajectoryConstPtr &msg)
{
  MELO_INFO("Received jointTrajectoryCb Topic");
  msg->name[0];
  msg->position[0];
  /* @Jonas
  Fülle die Einträge mit Daten
  */
}
//Action Server
void ExampleNode::deviceStateCb (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  MELO_INFO("Received deviceStateCb Action");
  goal->target_device_state;
  
  /* @Jonas
  feedback_device_state_.current_device_state = [...]; 
  as->publishFeedback(feedback_device_state_);
  result_device_state_.successful = [...];
  result_device_state_.reached_device_state = [...];
  */
  
  as_device_state_.setSucceeded(result_device_state_);
}
//Action Server
void ExampleNode::homingCb (const varileg_msgs::HomingGoalConstPtr &goal)
{
  MELO_INFO("Received homingCb Action");

  /* @Jonas
  goal -> ?
  feedback_homing_.interrupted = [...];
  as->publishFeedback(feedback_homing_);
  result_homing_.successful = [...];
  */   
  as_homing_.setSucceeded(result_homing_); 
}
//Publisher
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
