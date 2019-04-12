#include "varileg_lowlevel_controller/EthercatNode.hpp"

namespace varileg_lowlevel_controller {

bool EthercatNode::init() {
  MELO_INFO("init called");
  constexpr unsigned int defaultQueueSize = 1;
  //Publisher
  jointStatesPublisher_= advertise<varileg_msgs::ExtendedJointStates>("joint_state","joint_state",1);
  deviceStatePublisher_= advertise<varileg_msgs::ExtendedDeviceStates>("device_state","device_state",1);

  //Subscriber
  jointTrajectoriesSubscriber_ = subscribe("joint_trajectory", "/joint_trajectory", defaultQueueSize,
                                           &EthercatNode::jointTrajectoriesCallback, this);

  constexpr double defaultWorkerTimeStep = .005;
  constexpr int priority = 10;
  double workerTimeStep = param<double>("time_step", defaultWorkerTimeStep);

  auto jointName2NodeIdMap = param<std::map<std::string, int>>("epos_mapping", {{"knee_right" , 4}});
  eposEthercatSlaveManager_->setJointName2NodeIdMap(jointName2NodeIdMap);

  for (auto &pair : jointName2NodeIdMap) {
    MELO_INFO_STREAM("key: " << pair.first << " value: " << pair.second)
  }

  setupBusManager();

  MELO_INFO_STREAM("before bus startup");
  if(!busManager_->startupAllBuses(slavesOfBusesMap_, true)) {
    MELO_ERROR("Startup of all buses failed.");
    return false;
  }
  MELO_INFO_STREAM("after bus startup");

  // setup epos manager
  for(auto it : slavesOfBusesMap_) {
    for(soem_interface::EthercatSlaveBasePtr slave : it.second) {
      EposEthercatSlavePtr eposEthercatSlavePtr = std::dynamic_pointer_cast<EposEthercatSlave>(slave);
      if(!eposEthercatSlaveManager_->addEposEthercatSlave(eposEthercatSlavePtr)) {
        return false;
      }
    }
  }

  int interpolationTimePeriod = workerTimeStep * 1000;
  eposEthercatSlaveManager_->writeAllInterpolationTimePeriod(interpolationTimePeriod);

  eposEthercatSlaveManager_->setEncoderConverters("knee_right", {3983.96653}, {-346321.156});

  addWorker("ethercatNode::updateWorker", workerTimeStep, &EthercatNode::update, this, priority);

  // if you encounter an error in the init function and wish to shut down the node, you can return false
  return true;
}

void EthercatNode::setupBusManager() {
  std::string leftBusName = param<std::string>("left_bus_name", "ens9");
  std::string rightBusName = param<std::string>("right_bus_name", "eth1");

  soem_interface::EthercatBusBasePtr leftBus = std::make_shared<soem_interface::EthercatBusBase>(leftBusName);
  busManager_->addEthercatBus(leftBus);

  std::vector<soem_interface::EthercatSlaveBasePtr> leftBusEthercatSlaves;
  leftBusEthercatSlaves.push_back(std::make_shared<EposEthercatSlave>("epos_left_1", leftBus, 1));
  //leftBusEthercatSlaves.push_back(std::make_shared<EposEthercatSlave>("epos_left_2", leftBus, 2));
  slavesOfBusesMap_.insert(std::make_pair(leftBusName, leftBusEthercatSlaves));

 /* soem_interface::EthercatBusBasePtr rightBus = std::make_shared<soem_interface::EthercatBusBase>(rightBusName);
  busManager_->addEthercatBus(rightBus);

  std::vector<soem_interface::EthercatSlaveBasePtr> rightBusEthercatSlaves(2);
  rightBusEthercatSlaves.push_back(std::make_shared<EposEthercatSlave>("epos_right_1", rightBus, 1));
  //rightBusEthercatSlaves.push_back(std::make_shared<EposEthercatSlave>("epos_right_2", rightBus, 2));
  slavesOfBusesMap_.insert(std::make_pair(rightBusName, rightBusEthercatSlaves));*/
}

void EthercatNode::cleanup() {
  // this function is called when the node is requested to shut down, _after_ the ros spinners and workers were stopped
  // no need to stop workers which are started with addWorker(..) function
  MELO_INFO("cleanup called");

  busManager_->shutdownAllBuses(slavesOfBusesMap_);
}

bool EthercatNode::update(const any_worker::WorkerEvent &event) {
  // called by the worker which is automatically set up if rosparam standalone == True.
  // The frequency is defined in the time_step rosparam.
  MELO_INFO("update called");

  busManager_->receiveAllBusBuffers();

  eposEthercatSlaveManager_->readAllInboxes();

  varileg_msgs::ExtendedDeviceStates extendedDeviceStates = eposEthercatSlaveManager_->getExtendedDeviceStates();
  varileg_msgs::ExtendedJointStates extendedJointStates = eposEthercatSlaveManager_->getExtendedJointStates();

  jointStatesPublisher_.publish(extendedJointStates);
  deviceStatePublisher_.publish(extendedDeviceStates);

 /* varileg_msgs::ExtendedJointTrajectories extendedJointTrajectories;

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

  eposEthercatSlaveManager_->setExtendedJointTrajectories(extendedJointTrajectories);*/

  eposEthercatSlaveManager_->writeAllOutboxes();

  busManager_->sendAllBusBuffers();

  return true;
}

//Subscriber
void EthercatNode::jointTrajectoriesCallback(const varileg_msgs::ExtendedJointTrajectoriesConstPtr &msg)
{
  MELO_INFO("Received jointTrajectoriesCallback Topic");

  eposEthercatSlaveManager_->setExtendedJointTrajectories(*msg);
}

//Action Server
void EthercatNode::deviceStateCallback(const varileg_msgs::DeviceStateGoalConstPtr &goal)
{
  MELO_INFO_STREAM("Received deviceStateCallback Action Goal:" << static_cast<int>(goal->target_device_state.state));

  ros::Rate rate(100);

  const std::string jointName = "knee_right";
  eposEthercatSlaveManager_->setDeviceState(jointName, goal->target_device_state);

  varileg_msgs::DeviceState currentDeviceState = eposEthercatSlaveManager_->getDeviceState(jointName);
  while(goal->target_device_state.state != currentDeviceState.state && eposEthercatSlaveManager_->isDeviceStateReachable(jointName)) {
    if(deviceStateActionServer_.isPreemptRequested() || !ros::ok()) {
      deviceStateActionServer_.setPreempted();
      deviceStateResult_.successful = false;
      break;
    }

    currentDeviceState = eposEthercatSlaveManager_->getDeviceState(jointName);

    MELO_INFO_STREAM("callback: target: " << static_cast<int>(goal->target_device_state.state) << " current: " << static_cast<int>(currentDeviceState.state) << " reachable: " << eposEthercatSlaveManager_->isDeviceStateReachable(jointName));

    deviceStateFeedback_.current_device_state = currentDeviceState;
    deviceStateActionServer_.publishFeedback(deviceStateFeedback_);

    rate.sleep();
  }

  if(!eposEthercatSlaveManager_->isDeviceStateReachable(jointName)) {
    deviceStateResult_.successful = false;
  } else {
    deviceStateResult_.successful = goal->target_device_state.state == currentDeviceState.state;
  }

  deviceStateResult_.reached_device_state = currentDeviceState;

  deviceStateActionServer_.setSucceeded(deviceStateResult_);
}

//Action Server
void EthercatNode::homingCallback(const varileg_msgs::HomingGoalConstPtr &goal)
{
  MELO_INFO_STREAM("Received homingCallback Action Goal: " << static_cast<int>(goal->mode));

  ros::Rate rate(100);

  const std::string jointName = "knee_right";
  varileg_msgs::OperatingMode operatingMode;

  operatingMode.mode = varileg_msgs::OperatingMode::MODE_HMM;
  eposEthercatSlaveManager_->writeOperatingMode(jointName, operatingMode);

  eposEthercatSlaveManager_->writeHomingMethod(jointName, goal->mode);

  eposEthercatSlaveManager_->setHomingState(jointName, HomingState::HOMING_IN_PROGRESS);

/*  HomingState currentHomingState;
  do {
    rate.sleep();

    currentHomingState = eposEthercatSlaveManager_->getHomingState(jointName);

    homingFeedback_.interrupted = currentHomingState == HomingState::HOMING_INTERRUPTED;
    homingActionServer_.publishFeedback(homingFeedback_);
  } while(currentHomingState != HomingState::HOMING_SUCCESSFUL);

  homingResult_.successful = currentHomingState == HomingState::HOMING_SUCCESSFUL;
  homingActionServer_.setSucceeded(homingResult_);

  operatingMode.mode = varileg_msgs::OperatingMode::MODE_CSP;
  eposEthercatSlaveManager_->writeOperatingMode(jointName, operatingMode);*/
}

void EthercatNode::preCleanup() {
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
}


}
