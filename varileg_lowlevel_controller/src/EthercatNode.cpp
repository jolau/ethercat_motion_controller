#include "varileg_lowlevel_controller/EthercatNode.hpp"

namespace varileg_lowlevel_controller {

bool EthercatNode::init() {
  MELO_INFO("init called");
  constexpr unsigned int defaultQueueSize = 1;
  //Publisher
  jointStatesPublisher_ = advertise<varileg_msgs::ExtendedJointStates>("joint_states", "joint_states", 1);
  deviceStatePublisher_ = advertise<varileg_msgs::ExtendedDeviceStates>("device_states", "device_states", 1);

  //Subscriber
  jointTrajectoriesSubscriber_ = subscribe("joint_trajectories", "joint_trajectories", defaultQueueSize,
                                           &EthercatNode::jointTrajectoriesCallback, this);
  // Services
  deviceStateServiceServer_ = advertiseService("set_device_state", "set_device_state", &EthercatNode::setDeviceStateCallback, this);
  operatingModeServiceServer_ = advertiseService("set_operating_mode", "set_operating_mode", &EthercatNode::setOperatingModeCallback, this);

  constexpr int priority = 10;
  double workerTimeStep = param<double>("time_step", 0.01);

  auto jointName2NodeIdMap =
      param<std::map<std::string, int>>("epos_mapping", {{"hip_left", 1}, {"hip_right", 2}, {"knee_left", 5}, {"knee_right", 3}});
  eposEthercatSlaveManager_->setJointName2NodeIdMap(jointName2NodeIdMap);
  for (auto &pair : jointName2NodeIdMap) {
    MELO_INFO_STREAM("NodeIDMap key: " << pair.first << " value: " << pair.second)
  }

  EposStartupConfig eposStartupConfig;
  eposStartupConfig.interpolationTimePeriod = workerTimeStep * 1000 * param<double>("epos_interpolation_factor", 1);
  eposStartupConfig.motorCurrentLimit = param<int>("motor_current", 2000);
  setupBusManager(eposStartupConfig);

  if (!busManager_->startupAllBuses(slavesOfBusesMap_, true)) {
    MELO_ERROR("Startup of all buses failed.");
    return false;
  }

  // setup epos manager
  for (auto it : slavesOfBusesMap_) {
    for (soem_interface::EthercatSlaveBasePtr slave : it.second) {
      EposEthercatSlavePtr eposEthercatSlavePtr = std::dynamic_pointer_cast<EposEthercatSlave>(slave);
      if (!eposEthercatSlaveManager_->addEposEthercatSlave(eposEthercatSlavePtr)) {
        return false;
      }
    }
  }

  // worker has to be started as soon as possible as bus is started up
  addWorker("ethercatNode::updateWorker", workerTimeStep, &EthercatNode::update, this, priority);

  for (auto &pair : jointName2NodeIdMap) {
    std::string jointName = pair.first;
    eposEthercatSlaveManager_->setJointSpecifications(jointName, loadJointSpecifications(jointName));
  }

  // if you encounter an error in the init function and wish to shut down the node, you can return false
  return true;
}

JointSpecifications EthercatNode::loadJointSpecifications(std::string jointName) {
  JointSpecifications jointSpecifications;

  jointSpecifications.primaryEncoderConverter = {param<double>(jointName + "/primary_conversion_factor", 1.0)};
  jointSpecifications.secondaryEncoderConverter = {param<double>(jointName + "/secondary_conversion_factor", 1.0)};
  jointSpecifications.encoderCrosschecker = EncoderCrosschecker(param<double>(
      jointName + "/crosscheck_margin_positive",
      1.0), param<double>(jointName + "/crosscheck_margin_negative", 1.0));
  jointSpecifications.homeOffset = param<double>(jointName + "/home_offset", 0);
  jointSpecifications.minPositionLimit = param<double>(jointName + "/min_position_limit", 0);
  jointSpecifications.maxPositionLimit = param<double>(jointName + "/max_position_limit", 0);

  return jointSpecifications;
}

void EthercatNode::setupBusManager(EposStartupConfig config) {
  std::string leftBusName = param<std::string>("left_bus_name", "enp4s0");
  std::string rightBusName = param<std::string>("right_bus_name", "enp3s0");
  MELO_INFO_STREAM("after param; left bus name: " << leftBusName);

  soem_interface::EthercatBusBasePtr leftBus = std::make_shared<soem_interface::EthercatBusBase>(leftBusName);
  busManager_->addEthercatBus(leftBus);

  std::vector<soem_interface::EthercatSlaveBasePtr> leftBusEthercatSlaves;
  leftBusEthercatSlaves.push_back(std::make_shared<EposEthercatSlave>("epos_left_1", leftBus, 1, config));
  leftBusEthercatSlaves.push_back(std::make_shared<EposEthercatSlave>("epos_left_2", leftBus, 2, config));
  slavesOfBusesMap_.insert(std::make_pair(leftBusName, leftBusEthercatSlaves));

  soem_interface::EthercatBusBasePtr rightBus = std::make_shared<soem_interface::EthercatBusBase>(rightBusName);
  busManager_->addEthercatBus(rightBus);

  std::vector<soem_interface::EthercatSlaveBasePtr> rightBusEthercatSlaves;
  rightBusEthercatSlaves.push_back(std::make_shared<EposEthercatSlave>("epos_right_1", rightBus, 1, config));
  rightBusEthercatSlaves.push_back(std::make_shared<EposEthercatSlave>("epos_right_2", rightBus, 2, config));
  slavesOfBusesMap_.insert(std::make_pair(rightBusName, rightBusEthercatSlaves));
}

void EthercatNode::preCleanup() {
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
  isStopped = true;
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
  MELO_DEBUG_STREAM("update called");

  busManager_->receiveAllBusBuffers();

  eposEthercatSlaveManager_->readAllInboxes();

  // TODO: put publisher stuff in own thread for better speed
  varileg_msgs::ExtendedDeviceStates extendedDeviceStates = eposEthercatSlaveManager_->getExtendedDeviceStates();
  varileg_msgs::ExtendedJointStates extendedJointStates = eposEthercatSlaveManager_->getExtendedJointStates();
  jointStatesPublisher_.publish(extendedJointStates);
  deviceStatePublisher_.publish(extendedDeviceStates);

  eposEthercatSlaveManager_->writeAllOutboxes();

  busManager_->sendAllBusBuffers();

  return true;
}

//Subscriber
void EthercatNode::jointTrajectoriesCallback(const varileg_msgs::ExtendedJointTrajectoriesConstPtr &msg) {
  MELO_DEBUG_STREAM("Received jointTrajectoriesCallback Topic");

  eposEthercatSlaveManager_->setExtendedJointTrajectories(*msg);
}

bool EthercatNode::setOperatingModeCallback(varileg_msgs::SetOperatingModeRequest &request,
                                            varileg_msgs::SetOperatingModeResponse &response) {
  if (eposEthercatSlaveManager_->getDeviceState(request.name).state == varileg_msgs::DeviceState::STATE_SWITCH_ON_DISABLED) {
    eposEthercatSlaveManager_->setOperatingMode(request.name, request.operating_mode);
    response.success = true;
  } else {
    response.success = false;
    response.message = "Device State is not SWITCH ON DISABLED";
  }
  return true;
}

bool EthercatNode::setDeviceStateCallback(varileg_msgs::SetDeviceStateRequest &request, varileg_msgs::SetDeviceStateResponse &response) {
  MELO_INFO_STREAM("Set Device State Callback: " << request.name << " : " << request.target_device_state);
  bool stateReachable = eposEthercatSlaveManager_->setDeviceState(request.name, request.target_device_state);
  response.is_device_state_reachable = stateReachable;
  response.current_device_state = eposEthercatSlaveManager_->getDeviceState(request.name);
  return true;
}

//Action Server
void EthercatNode::homingCallback(actionlib::SimpleActionServer<varileg_msgs::HomingAction> &homingActionServer,
                                  const std::string &name,
                                  const varileg_msgs::HomingGoalConstPtr &goal) {
  MELO_INFO_STREAM("Received homingCallback Action Goal: " << static_cast<int>(goal->mode));

  bool success = true;
  varileg_msgs::DeviceState deviceState;

  ros::Rate rate(50);

  varileg_msgs::HomingResult homingResult;
  if (eposEthercatSlaveManager_->getDeviceState(name).state != varileg_msgs::DeviceState::STATE_OP_ENABLED) {
    homingResult.success = false;
    homingResult.message = "Wrong Device State";
    homingActionServer.setSucceeded(homingResult);
    return;
  }

  if (eposEthercatSlaveManager_->getOperatingMode(name) != OperatingMode::HMM) {
    homingResult.success = false;
    homingResult.message = "Wrong Operating Mode";
    homingActionServer.setSucceeded(homingResult);
    return;
  }

  eposEthercatSlaveManager_->writeHomingMethod(name, goal->mode);

  rate.sleep();

  eposEthercatSlaveManager_->setHomingState(name, HomingState::HOMING_IN_PROGRESS);

  varileg_msgs::HomingFeedback homingFeedback;

  HomingState currentHomingState;
  do {
    if (homingActionServer.isPreemptRequested() || isStopped) {
      MELO_INFO_STREAM(name << "'s Homing Action: Preempted");
      homingActionServer.setPreempted();
      success = false;
      break;
    }
    currentHomingState = eposEthercatSlaveManager_->getHomingState(name);

    homingFeedback.interrupted = (currentHomingState == HomingState::HOMING_INTERRUPTED);
    homingActionServer.publishFeedback(homingFeedback);

    rate.sleep();
  } while (currentHomingState != HomingState::HOMING_SUCCESSFUL && currentHomingState != HomingState::HOMING_ERROR);

  eposEthercatSlaveManager_->setHomingState(name, HomingState::HOMING_INTERRUPTED);

  if (success) {
    homingResult.success = (currentHomingState == HomingState::HOMING_SUCCESSFUL);
    homingResult.message = Enum::toString(currentHomingState);
    homingActionServer.setSucceeded(homingResult);
  }
}

void EthercatNode::homingCallbackHipRight(const varileg_msgs::HomingGoalConstPtr &goal) {
  homingCallback(homingActionServerHipRight_, "hip_right", goal);
}

void EthercatNode::homingCallbackHipLeft(const varileg_msgs::HomingGoalConstPtr &goal) {
  homingCallback(homingActionServerHipLeft_, "hip_left", goal);
}

void EthercatNode::homingCallbackKneeRight(const varileg_msgs::HomingGoalConstPtr &goal) {
  homingCallback(homingActionServerKneeRight_, "knee_right", goal);
}

void EthercatNode::homingCallbackKneeLeft(const varileg_msgs::HomingGoalConstPtr &goal) {
  homingCallback(homingActionServerKneeLeft_, "knee_left", goal);
}

}
