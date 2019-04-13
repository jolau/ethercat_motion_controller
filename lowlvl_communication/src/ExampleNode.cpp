#include "lowlvl_communication/ExampleNode.hpp"

namespace lowlvl_communication {

ExampleNode::ExampleNode(any_node::Node::NodeHandlePtr nh): any_node::Node(nh), 
   deviceStateActionServer_(*nh, "DeviceState", boost::bind (&ExampleNode::deviceStateCallback, this, _1),false),
   homingActionServer_(*nh, "Homing" , boost::bind(&ExampleNode::homingCallback, this, _1),false)
{
  //Action Server
  deviceStateActionServer_.start();
  homingActionServer_.start();
}
   
bool ExampleNode::init() 
{
  constexpr unsigned int defaultQueueSize = 1;
  //Publisher
  jointStatesPublisher_= advertise<varileg_msgs::ExtendedJointStates>("joint_state","joint_state",1);
  deviceStatePublisher_= advertise<varileg_msgs::ExtendedDeviceStates>("device_state","device_state",1);

  //Subscriber
  jointTrajectoriesSubscriber_ = subscribe("joint_trajectory", "joint_trajectory", defaultQueueSize,
                                          &ExampleNode::jointTrajectoriesCallback, this);


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
void ExampleNode::jointTrajectoriesCallback(const varileg_msgs::ExtendedJointTrajectoriesConstPtr &msg)
{
  MELO_INFO("Received jointTrajectoriesCallback Topic");
  msg->name[0];
  msg->position[0];
  /* @Jonas
  Fülle die Einträge mit Daten
  */
}
//Action Server
void ExampleNode::deviceStateCallback(const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  MELO_INFO("Received deviceStateCallback Action");
  goal->target_device_state;
  
  /* @Jonas
  deviceStateFeedback_.current_device_state = [...];
  deviceStateActionServer_.publishFeedback(deviceStateFeedback_);
  deviceStateResult_.successful = [...];
  deviceStateResult_.reached_device_state = [...];
  */
  
  deviceStateActionServer_.setSucceeded(deviceStateResult_);
}
//Action Server
void ExampleNode::homingCallback(const varileg_msgs::HomingGoalConstPtr &goal)
{
  MELO_INFO("Received homingCallback Action");

  /* @Jonas
  goal -> ?
  homingFeedback_.interrupted = [...];
  homingActionServer_.publishFeedback(homingFeedback_);
  homingResult_.successful = [...];
   homingActionServer_.isPreemptedRequested();
  */   
  homingActionServer_.setSucceeded(homingResult_);
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
  jointStatesPublisher_.publish(extendedJointStates_);
  deviceStatePublisher_.publish(extendedDeviceStates_);

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
