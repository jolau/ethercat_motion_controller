//
// Created by jolau on 27.03.19.
//

#include "varileg_lowlevel_controller/EposEthercatSlaveManager.hpp"

bool varileg_lowlevel_controller::EposEthercatSlaveManager::addEposEthercatSlave(varileg_lowlevel_controller::EposEthercatSlavePtr eposEthercatSlave) {
  if(!eposEthercatSlave->isStartedUp()) {
    MELO_ERROR_STREAM("Bus resp. slave: " << eposEthercatSlave->getName() << " were not started up.")
    return false;
  }

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

void varileg_lowlevel_controller::EposEthercatSlaveManager::setJointName2NodeIdMap(const std::map<std::string, int> &jointName2NodeIdMap) {
  jointName2NodeIdMap_ = jointName2NodeIdMap;
}
