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

  ///////////////////////////////////////////////////////////////////////////////////////////////
  ////////////// New stuff                                                          ////////////
  /////////////////////////////////////////////////////////////////////////////////////////////


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
//Publisher
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

}
