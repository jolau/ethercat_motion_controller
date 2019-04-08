#include "lowlvl_communication/ExampleNode.hpp"

namespace lowlvl_communication {

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
  sub_joint_trajectory_ = subscribe("joint_trajectory", "/default_subscriber_topic", defaultQueueSize, &ExampleNode::jointTrajectoryCb, this);

  //Action Server
  DeviceState  as_device_state_(*this->nh_.get(), "DeviceState", boost::bind (&ExampleNode::deviceStateCb, this, _1, &as_device_state_),false);

  

  return true;
}

//Subscriber
void ExampleNode::jointTrajectoryCb (const varileg_msgs::ExtendedJointTrajectoryConstPtr &msg)
{
    MELO_INFO("Any nodes sind scheiße");
}
//Action Server
void ExampleNode::deviceStateCb (const varileg_msgs::DeviceStateGoalConstPtr &goal, DeviceState* as)
{
  MELO_INFO("Hello world");
  goal->target_device_state;
  as->setSucceeded();
}
//Action Server
void ExampleNode::homingCb (const varileg_msgs::HomingActionGoalConstPtr &goal)
{
    MELO_INFO("Any nodes sind immer noch scheiße");
}


////////////////////////////////////////////////////////////////////////
/////////// Old stuff                                          ////////
//////////////////////////////////////////////////////////////////////

void ExampleNode::cleanup()
{
  MELO_INFO("cleanup called");
}

bool ExampleNode::update(const any_worker::WorkerEvent& event)
{
  MELO_INFO("update called");
  return true;
}

void ExampleNode::preCleanup()
{
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
}

void ExampleNode::subscriberCallback(const std_msgs::StringConstPtr &msg)
{
  // called asynchrounously when ros messages arrive for the subscriber created in init() function
  MELO_INFO("received ros message: %s", msg->data.c_str());
}



}
