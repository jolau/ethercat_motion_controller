#include "varileg_lowlevel_controller/EthercatNode.hpp"
#include "varileg_lowlevel_controller_msgs/MotorControllerState.h"
#include "varileg_lowlevel_controller_msgs/ExtendedJointState.h"

namespace varileg_lowlevel_controller {

bool EthercatNode::init() {
  varileg_lowlevel_controller_msgs::MotorControllerState motorControllerState;
  motorControllerState.state = 3;
  varileg_lowlevel_controller_msgs::ExtendedJointState jointState;
  jointState.motor_controller_state[0].state = varileg_lowlevel_controller_msgs::MotorControllerState::STATE_SWITCH_ON_DISABLED;

  MELO_INFO("init called");

  jointName2NodeIdMap_ = param<std::map<std::string, int>>("epos_mapping", std::map<std::string, int>());

  for (auto &pair : jointName2NodeIdMap_) {
    MELO_INFO_STREAM("key: " << pair.first << " value: " << pair.second)
  }

  std::array<EposEthercatSlavePtr, 4> unmappedEposEthercatSlaves = setupBusManager();

  MELO_INFO_STREAM("before bus startup");
  if(!busManager_->startupAllBuses(true)) {
    MELO_ERROR("Startup of all buses failed.");
    return false;
  }
  MELO_INFO_STREAM("after bus startup");

  for(EposEthercatSlavePtr eposEthercatSlave : unmappedEposEthercatSlaves) {
    if(!mapEpos2Joint(eposEthercatSlave)) {
      return false;
    }
  }

  constexpr double defaultWorkerTimeStep = .01;
  constexpr int priority = 10;
  double workerTimeStep = param<double>("time_step", defaultWorkerTimeStep);
  addWorker("ethercatNode::updateWorker", workerTimeStep, &EthercatNode::update, this, priority);

  // if you encounter an error in the init function and wish to shut down the node, you can return false
  return true;
}

bool EthercatNode::mapEpos2Joint(EposEthercatSlavePtr &eposEthercatSlave) {
  uint8_t nodeId = eposEthercatSlave->readNodeId();

  for (auto &pair : jointName2NodeIdMap_) {
    if (pair.second == nodeId) {
      eposEthercatSlaves_.insert(std::make_pair(pair.first, eposEthercatSlave));
      return true;
    }
  }

  MELO_ERROR_STREAM("Unmapped EPOS EtherCAT slave with NodeID found: " << nodeId);
  return false;
}

std::array<EposEthercatSlavePtr, 4> EthercatNode::setupBusManager() {
  std::string leftBusName = param<std::string>("left_bus_name", "ens9");
  std::string rightBusName = param<std::string>("right_bus_name", "eth1");

  std::array<EposEthercatSlavePtr, 4> unmappedEposEthercatSlaves;

  soem_interface::EthercatBusBasePtr leftBus = std::make_shared<soem_interface::EthercatBusBase>(leftBusName);
  unmappedEposEthercatSlaves[0] = std::make_shared<EposEthercatSlave>("epos_left_1", leftBus, 1);
  leftBus->addSlave(unmappedEposEthercatSlaves[0]);
  unmappedEposEthercatSlaves[1] = std::make_shared<EposEthercatSlave>("epos_left_2", leftBus, 2);
  leftBus->addSlave(unmappedEposEthercatSlaves[1]);
  busManager_->addEthercatBus(leftBus);

  soem_interface::EthercatBusBasePtr rightBus = std::make_shared<soem_interface::EthercatBusBase>(rightBusName);
  unmappedEposEthercatSlaves[2] = std::make_shared<EposEthercatSlave>("epos_right_1", rightBus, 1);
  rightBus->addSlave(unmappedEposEthercatSlaves[2]);
  unmappedEposEthercatSlaves[3] = std::make_shared<EposEthercatSlave>("epos_right_2", rightBus, 2);
  rightBus->addSlave(unmappedEposEthercatSlaves[3]);
  busManager_->addEthercatBus(rightBus);

  return unmappedEposEthercatSlaves;
}

void EthercatNode::cleanup() {
  // this function is called when the node is requested to shut down, _after_ the ros spinners and workers were stopped
  // no need to stop workers which are started with addWorker(..) function
  MELO_INFO("cleanup called");

  busManager_->shutdownAllBuses();
}

bool EthercatNode::update(const any_worker::WorkerEvent &event) {
  // called by the worker which is automatically set up if rosparam standalone == True.
  // The frequency is defined in the time_step rosparam.
  MELO_INFO("update called");

  //ethercat_bus_->updateRead();

  //ethercat_bus_->updateWrite();

  return true;
}

void EthercatNode::preCleanup() {
  // this function is called when the node is requested to shut down, _before_ the ros spinners and workers are beeing stopped
  MELO_INFO("preCleanup called");
}

void EthercatNode::subscriberCallback(const std_msgs::StringConstPtr &msg) {
  // called asynchrounously when ros messages arrive for the subscriber created in init() function
  MELO_INFO("received ros message: %s", msg->data.c_str());
}

}
