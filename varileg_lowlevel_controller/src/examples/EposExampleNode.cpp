//
// Created by jolau on 28.03.19.
//

#include <varileg_lowlevel_controller/examples/EposExampleNode.hpp>

bool varileg_lowlevel_controller::examples::EposExampleNode::init() {
  MELO_INFO("init called");

  constexpr double defaultWorkerTimeStep = .01;
  double workerTimeStep = param<double>("time_step", defaultWorkerTimeStep);
  constexpr int priority = 10;

  std::map<std::string, int> joint2eposMap;
  joint2eposMap.insert(std::make_pair("hip_left", 3));
 // joint2eposMap.insert(std::make_pair("knee_left", 2));
  eposEthercatSlaveManager_->setJointName2NodeIdMap(joint2eposMap);

  EposEthercatSlavePtr eposEthercatSlaveOne = std::make_shared<EposEthercatSlave>("epos1", bus_, 1);
  slaves_.push_back(eposEthercatSlaveOne);

  //EposEthercatSlavePtr eposEthercatSlaveTwo = std::make_shared<EposEthercatSlave>("epos2", bus_, 2);
  //slaves_.push_back(eposEthercatSlaveTwo);

  if(!bus_->startup(slaves_)){
    MELO_ERROR("Startup of bus failed.");
    return false;
  }

  eposEthercatSlaveOne->writeInterpolationTimePeriod(workerTimeStep);

  bus_->setState(EC_STATE_OPERATIONAL);

  if(!eposEthercatSlaveManager_->addEposEthercatSlave(eposEthercatSlaveOne)) {
    MELO_ERROR("Could not add epos one to manager.")
    return false;
  };

  eposEthercatSlaveManager_->setEncoderConfig("hip_left", {3983.96653}, {-346321.156}, EncoderCrosschecker());

 /* if(!eposEthercatSlaveManager_->addEposEthercatSlave(eposEthercatSlaveTwo)) {
    MELO_ERROR("Could add epos two to manager.")
    return false;
  };*/


  addWorker("eposExampleNode::updateWorker", workerTimeStep, &EposExampleNode::update, this, priority);

  // if you encounter an error in the init function and wish to shut down the node, you can return false
  return true;
}

void varileg_lowlevel_controller::examples::EposExampleNode::preCleanup() {
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
}

void varileg_lowlevel_controller::examples::EposExampleNode::cleanup() {
  // this function is called when the node is requested to shut down, _after_ the ros spinners and workers were stopped
  // no need to stop workers which are started with addWorker(..) function
  MELO_INFO("cleanup called");

  bus_->shutdown(slaves_);
}

bool varileg_lowlevel_controller::examples::EposExampleNode::update(const any_worker::WorkerEvent &event) {
  // called by the worker which is automatically set up if rosparam standalone == True.
  // The frequency is defined in the time_step rosparam.
  MELO_INFO("update called");

  bus_->receiveInbox();

  eposEthercatSlaveManager_->readAllInboxes();

  varileg_msgs::ExtendedDeviceStates extendedDeviceStates = eposEthercatSlaveManager_->getExtendedDeviceStates();
  varileg_msgs::ExtendedJointStates extendedJointStates = eposEthercatSlaveManager_->getExtendedJointStates();
  
  varileg_msgs::ExtendedJointTrajectories extendedJointTrajectories;

  MELO_INFO_STREAM("states size" << extendedJointStates.name.size())

  for (int i = 0; i < extendedJointStates.name.size(); ++i) {
    std::string name = extendedJointStates.name[i];
    MELO_INFO_STREAM(name << i);

      MELO_INFO_STREAM(name << ": Actual Position: " << extendedJointStates.position[i] << " PrimaryPosition: " << extendedJointStates.primary_position[i] << " SecondaryPosition: " << extendedJointStates.secondary_position[i] << " diff: " << (extendedJointStates.primary_position[i] - extendedJointStates.secondary_position[i]));
    if(extendedDeviceStates.device_state[i].state == varileg_msgs::DeviceState::STATE_OP_ENABLED) {
      extendedJointTrajectories.name.push_back(name);

      double position = 0;
      if(goUp) {
        if (extendedJointStates.position[i] >= 1.57079) {
          goUp = false;
        }
          
        position = 1.6;
      } else {
        if(extendedJointStates.position[i] <= 0) {
          goUp = true;
        }

        position = -0.1;
      }

      extendedJointTrajectories.position.push_back(position);

      MELO_INFO_STREAM(name << ": Send Target Position: " << extendedJointTrajectories.position[i] << " goUP: " << goUp);
    } else {
      MELO_INFO_STREAM(name << ": Enabling Drive");
      varileg_msgs::DeviceState deviceState;
      deviceState.state = varileg_msgs::DeviceState::STATE_OP_ENABLED;

     eposEthercatSlaveManager_->setDeviceState(name, deviceState);
    }
  }

  eposEthercatSlaveManager_->setExtendedJointTrajectories(extendedJointTrajectories);

  eposEthercatSlaveManager_->writeAllOutboxes();
  bus_->sendOutbox();

  return true;
}

void varileg_lowlevel_controller::examples::EposExampleNode::subscriberCallback(const std_msgs::StringConstPtr &msg) {
  // called asynchrounously when ros messages arrive for the subscriber created in init() function
  MELO_INFO("received ros message: %s", msg->data.c_str());
}