#include "lowlvl_communication/ExampleNode.hpp"

namespace lowlvl_communication {


ExampleNode::ExampleNode(any_node::Node::NodeHandlePtr nh): any_node::Node(nh), 
   deviceStateActionServerHipRight_(*nh_, "DeviceStateHipRight", boost::bind (&ExampleNode::deviceStateCallbackHipRight, this, _1),false),
   deviceStateActionServerHipLeft_(*nh_, "DeviceStateHipLeft", boost::bind (&ExampleNode::deviceStateCallbackHipLeft, this, _1),false),
   deviceStateActionServerKneeRight_(*nh_, "DeviceStateKneeRight", boost::bind (&ExampleNode::deviceStateCallbackKneeRight, this, _1),false),
   deviceStateActionServerKneeLeft_(*nh_, "DeviceStateKneeLeft", boost::bind (&ExampleNode::deviceStateCallbackKneeLeft, this, _1),false),
   
   homingActionServerHipRight_(*nh_, "HomingHipRight" , boost::bind(&ExampleNode::homingCallbackHipRight, this, _1),false),
   homingActionServerHipLeft_(*nh_, "HomingHipLeft" , boost::bind(&ExampleNode::homingCallbackHipLeft, this, _1),false),
   homingActionServerKneeRight_(*nh_, "HomingKneeRight" , boost::bind(&ExampleNode::homingCallbackKneeRight, this, _1),false),
   homingActionServerKneeLeft_(*nh_, "HomingKneeLeft" , boost::bind(&ExampleNode::homingCallbackKneeLeft, this, _1),false) 
{
  //Action Server
  deviceStateActionServerHipRight_.start();
  deviceStateActionServerHipLeft_.start();
  deviceStateActionServerKneeRight_.start();
  deviceStateActionServerKneeLeft_.start();

  homingActionServerHipRight_.start();
  homingActionServerHipLeft_.start();
  homingActionServerKneeRight_.start();
  homingActionServerKneeLeft_.start();
  
  //Publisher
  jointStatesPublisher_= advertise<varileg_msgs::ExtendedJointStates>("joint_states","joint_states",1);
  deviceStatePublisher_= advertise<varileg_msgs::ExtendedDeviceStates>("device_states","device_states",1);

}
   
bool ExampleNode::init() 
{
  constexpr unsigned int defaultQueueSize = 1;
  
  //Subscriber
  jointTrajectoriesSubscriber_ = subscribe("joint_trajectories", "joint_trajectories", defaultQueueSize,
                                          &ExampleNode::jointTrajectoriesCallback, this);


  constexpr double defaultWorkerTimeStep = 3.0;
  constexpr int priority = 10;
  double workerTimeStep = param<double>("time_step", defaultWorkerTimeStep);
  addWorker("exampleNode::updateWorker", workerTimeStep, &ExampleNode::update, this, priority);
  
 
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Device State Action Server                                                              /////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExampleNode::deviceStateCallbackHipRight (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  varileg_msgs::DeviceStateFeedback deviceStateFeedback;
  varileg_msgs::DeviceStateResult deviceStateResult;

  MELO_INFO("Received deviceStateCallback Action");
  goal->target_device_state;
  
  /* @Jonas
  deviceStateFeedback.current_device_state = [...]; 
  deviceStateActionServerHipRight_.publishFeedback(feedback_device_state);
  deviceStateResult.successful = [...];
  deviceStateResult.reached_device_state = [...];
  */
  
  deviceStateActionServerHipRight_.setSucceeded(deviceStateResult);
}

void ExampleNode::deviceStateCallbackHipLeft (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  varileg_msgs::DeviceStateFeedback deviceStateFeedback;
  varileg_msgs::DeviceStateResult deviceStateResult;

  MELO_INFO("Received deviceStateCallback Action");
  goal->target_device_state;
  
  /* @Jonas
  deviceStateFeedback.current_device_state = [...]; 
  deviceStateActionServerHipLeft_.publishFeedback(feedback_device_state);
  deviceStateResult.successful = [...];
  deviceStateResult.reached_device_state = [...];
  */
  
  deviceStateActionServerHipLeft_.setSucceeded(deviceStateResult);
}

void ExampleNode::deviceStateCallbackKneeRight (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  varileg_msgs::DeviceStateFeedback deviceStateFeedback;
  varileg_msgs::DeviceStateResult deviceStateResult;

  MELO_INFO("Received deviceStateCallback Action");
  goal->target_device_state;
  
  /* @Jonas
  deviceStateFeedback.current_device_state = [...]; 
  deviceStateActionServerKneeRight_.publishFeedback(feedback_device_state);
  deviceStateResult.successful = [...];
  deviceStateResult.reached_device_state = [...];
  */
  
  deviceStateActionServerKneeRight_.setSucceeded(deviceStateResult);
}

void ExampleNode::deviceStateCallbackKneeLeft (const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  varileg_msgs::DeviceStateFeedback deviceStateFeedback;
  varileg_msgs::DeviceStateResult deviceStateResult;

  MELO_INFO("Received deviceStateCallback Action");
  goal->target_device_state;
  
  /* @Jonas
  deviceStateFeedback.current_device_state = [...]; 
  deviceStateActionServerKneeLeft_.publishFeedback(feedback_device_state);
  deviceStateResult.successful = [...];
  deviceStateResult.reached_device_state = [...];
  */
  
  deviceStateActionServerKneeLeft_.setSucceeded(deviceStateResult);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Homing Action Server                                                                    /////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExampleNode::homingCallbackHipRight (const varileg_msgs::HomingGoalConstPtr &goal)
{
  varileg_msgs::HomingFeedback homingFeedback;
  varileg_msgs::HomingResult homingResult;
  MELO_INFO("Received homingCallback Action");

  /* @Jonas
  goal -> ?
  homingFeedback.interrupted = [...];
  homingActionServerHipRight_.publishFeedback(feedback_homing);
  homingResult.successful = [...];
  */   
  homingActionServerHipRight_.setSucceeded(homingResult); 
}

void ExampleNode::homingCallbackHipLeft (const varileg_msgs::HomingGoalConstPtr &goal)
{
  varileg_msgs::HomingFeedback homingFeedback;
  varileg_msgs::HomingResult homingResult;
  MELO_INFO("Received homingCallback Action");

  /* @Jonas
  goal -> ?
  homingFeedback.interrupted = [...];
  homingActionServerHipLeft_.publishFeedback(feedback_homing);
  homingResult.successful = [...];
  */   
  homingActionServerHipLeft_.setSucceeded(homingResult); 
}

void ExampleNode::homingCallbackKneeRight (const varileg_msgs::HomingGoalConstPtr &goal)
{
  varileg_msgs::HomingFeedback homingFeedback;
  varileg_msgs::HomingResult homingResult;
  MELO_INFO("Received homingCallback Action");

  /* @Jonas
  goal -> ?
  homingFeedback.interrupted = [...];
  homingActionServerKneeRight_.publishFeedback(feedback_homing);
  homingResult.successful = [...];
  */   
  homingActionServerKneeRight_.setSucceeded(homingResult); 
}

void ExampleNode::homingCallbackKneeLeft (const varileg_msgs::HomingGoalConstPtr &goal)
{
  varileg_msgs::HomingFeedback homingFeedback;
  varileg_msgs::HomingResult homingResult;
  MELO_INFO("Received homingCallback Action");

  /* @Jonas
  goal -> ?
  homingFeedback.interrupted = [...];
  homingActionServerKneeLeft_.publishFeedback(feedback_homing);
  homingResult.successful = [...];
  */   
  homingActionServerKneeLeft_.setSucceeded(homingResult);
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
