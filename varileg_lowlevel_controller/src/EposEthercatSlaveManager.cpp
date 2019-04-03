//
// Created by jolau on 27.03.19.
//

#include "varileg_lowlevel_controller/EposEthercatSlaveManager.hpp"

namespace varileg_lowlevel_controller {
EposEthercatSlaveManager::~EposEthercatSlaveManager() {

}

bool EposEthercatSlaveManager::addEposEthercatSlave(varileg_lowlevel_controller::EposEthercatSlavePtr eposEthercatSlave) {
  if (!eposEthercatSlave->isStartedUp()) {
    MELO_ERROR_STREAM("Bus resp. slave: " << eposEthercatSlave->getName() << " were not started up.")
    return false;
  }

  uint8_t nodeId = eposEthercatSlave->readNodeId();

  for (auto &pair : jointName2NodeIdMap_) {
    if (pair.second == nodeId) {
      MELO_INFO_STREAM("Mapped " << pair.first << " to epos " << eposEthercatSlave->getName());
      eposEthercatSlaves_.insert(std::make_pair(pair.first, eposEthercatSlave));
      return true;
    }
  }

  MELO_ERROR_STREAM("Unmapped EPOS EtherCAT slave with NodeID found: " << static_cast<int>(nodeId));
  return false;
}

void EposEthercatSlaveManager::setJointName2NodeIdMap(const std::map<std::string, int> &jointName2NodeIdMap) {
  jointName2NodeIdMap_ = jointName2NodeIdMap;
}

void EposEthercatSlaveManager::updateWriteAll(const varileg_lowlevel_controller_msgs::ExtendedJointStates &extendedJointStates) {
  assert(extendedJointStates.name.size() == eposEthercatSlaves_.size());

  for (int i = 0; i < extendedJointStates.name.size(); ++i) {
    ExtendedJointState extendedJointState;
    extendedJointState.motorControllerState = static_cast<MotorControllerState>(extendedJointStates.motor_controller_state[i]);
    extendedJointState.position = extendedJointStates.position[i];

    EposEthercatSlavePtr eposEthercatSlavePtr = eposEthercatSlaves_.at(extendedJointStates.name[i]);
    eposEthercatSlavePtr->setSendJointState(extendedJointState);
    eposEthercatSlavePtr->writeOutbox();
  }
}

varileg_lowlevel_controller_msgs::ExtendedJointStates varileg_lowlevel_controller::EposEthercatSlaveManager::updateReadAll() {
  varileg_lowlevel_controller_msgs::ExtendedJointStates extendedJointStates;

  for(const auto &it : eposEthercatSlaves_) {
    EposEthercatSlavePtr  eposEthercatSlavePtr = it.second;

    eposEthercatSlavePtr->readInbox();

    ExtendedJointState extendedJointState = eposEthercatSlavePtr->getReceiveJointState();

    extendedJointStates.name.push_back(it.first);
    extendedJointStates.position.push_back(extendedJointState.position);
    extendedJointStates.motor_controller_state.push_back(static_cast<int8_t>(extendedJointState.motorControllerState));
  }

  return extendedJointStates;
}

}