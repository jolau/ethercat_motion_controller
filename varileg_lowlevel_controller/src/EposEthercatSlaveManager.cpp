//
// Created by jolau on 27.03.19.
//

#include <varileg_lowlevel_controller/ConversionTraits.hpp>
#include "varileg_lowlevel_controller/EposEthercatSlaveManager.hpp"

namespace varileg_lowlevel_controller {
EposEthercatSlaveManager::~EposEthercatSlaveManager() {

}

bool EposEthercatSlaveManager::addEposEthercatSlave(varileg_lowlevel_controller::EposEthercatSlavePtr eposEthercatSlave) {
  std::lock_guard<std::mutex> lock(mutex_);

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

EposEthercatSlavePtr EposEthercatSlaveManager::getEposEthercatSlave(const std::string &name) const {
  const auto &it = eposEthercatSlaves_.find(name);
  if (it == eposEthercatSlaves_.end()) {
    return nullptr;
  }
  return it->second;
}

double EposEthercatSlaveManager::getJointOffset(const std::string &name) const {
  const auto &it = jointOffsetMap_.find(name);
  if (it == jointOffsetMap_.end()) {
    return 0;
  }
  return it->second;
}

void EposEthercatSlaveManager::setJointName2NodeIdMap(const std::map<std::string, int> &jointName2NodeIdMap) {
  std::lock_guard<std::mutex> lock(mutex_);

  jointName2NodeIdMap_ = jointName2NodeIdMap;
}

void EposEthercatSlaveManager::setJointOffsetMap(const std::map<std::string, double> &jointOffsetMap) {
  std::lock_guard<std::mutex> lock(mutex_);

  jointOffsetMap_ = jointOffsetMap;
}

void EposEthercatSlaveManager::writeAllOutboxes() {
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto &it : eposEthercatSlaves_) {
    EposEthercatSlavePtr eposEthercatSlavePtr = it.second;
    eposEthercatSlavePtr->writeOutbox();
  }
}

void EposEthercatSlaveManager::readAllInboxes() {
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto &it : eposEthercatSlaves_) {
    EposEthercatSlavePtr eposEthercatSlavePtr = it.second;
    eposEthercatSlavePtr->readInbox();
  }
}

varileg_msgs::ExtendedDeviceStates EposEthercatSlaveManager::getExtendedDeviceStates() {
  std::lock_guard<std::mutex> lock(mutex_);

  varileg_msgs::ExtendedDeviceStates extendedDeviceStates;

  //resizeExtendedDeviceStates(extendedDeviceStates, eposEthercatSlaves_.size());

  for (const auto &it : eposEthercatSlaves_) {
    EposEthercatSlavePtr eposEthercatSlavePtr = it.second;

    extendedDeviceStates.name.push_back(it.first);

    varileg_msgs::DeviceState
        deviceStateRos = ConversionTraits<DeviceState, varileg_msgs::DeviceState>::convert(eposEthercatSlavePtr->getReceiveDeviceState());
    extendedDeviceStates.device_state.push_back(deviceStateRos);

    varileg_msgs::OperatingMode operatingModeRos =
        ConversionTraits<OperatingMode, varileg_msgs::OperatingMode>::convert(eposEthercatSlavePtr->getReceiveOperatingMode());
    extendedDeviceStates.operating_mode.push_back(operatingModeRos);
  }

  return extendedDeviceStates;
}

void EposEthercatSlaveManager::resizeExtendedDeviceStates(varileg_msgs::ExtendedDeviceStates &extendedDeviceStates,
                                                          const unsigned long &size) {
  extendedDeviceStates.name.resize(size);
  extendedDeviceStates.device_state.resize(size);
  extendedDeviceStates.device_error.resize(size);
  extendedDeviceStates.operating_mode.resize(size);
}

varileg_msgs::ExtendedJointStates EposEthercatSlaveManager::getExtendedJointStates() {
  std::lock_guard<std::mutex> lock(mutex_);

  varileg_msgs::ExtendedJointStates extendedJointStates;

  MELO_INFO_STREAM(eposEthercatSlaves_.size());

 // resizeExtendedJointStates(extendedJointStates, eposEthercatSlaves_.size());

  for (const auto &it : eposEthercatSlaves_) {
    std::string name = it.first;
    EposEthercatSlavePtr eposEthercatSlavePtr = it.second;
    
    double offset = getJointOffset(name);

    MELO_INFO_STREAM("slave: " << name);
    extendedJointStates.name.push_back(name);

    JointState jointState = eposEthercatSlavePtr->getReceiveJointState();
    extendedJointStates.position.push_back(jointState.position + offset);
    extendedJointStates.primary_position.push_back(jointState.primaryPosition);
    extendedJointStates.secondary_position.push_back(jointState.secondaryPosition);
    extendedJointStates.velocity.push_back(jointState.velocity);
    extendedJointStates.torque.push_back(jointState.torque);
  }

  return extendedJointStates;
}

void EposEthercatSlaveManager::resizeExtendedJointStates(varileg_msgs::ExtendedJointStates &extendedJointStates,
                                                         const unsigned long &size) {
  extendedJointStates.name.resize(size);
  extendedJointStates.position.resize(size);
  extendedJointStates.primary_position.resize(size);
  extendedJointStates.secondary_position.resize(size);
  extendedJointStates.velocity.resize(size);
  extendedJointStates.torque.resize(size);
}

void EposEthercatSlaveManager::setExtendedJointTrajectories(const varileg_msgs::ExtendedJointTrajectories &extendedJointTrajectories) {
  std::lock_guard<std::mutex> lock(mutex_);
  const unsigned long nameSize = extendedJointTrajectories.name.size();

  if(nameSize != eposEthercatSlaves_.size()) {
    MELO_WARN_STREAM("ExtendedJointTrajectories does not address all EposEthercatSlaves.");
  }

  assert(nameSize == extendedJointTrajectories.position.size());

  for (int i = 0; i < nameSize; ++i) {
    std::string name = extendedJointTrajectories.name[i];
    EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
    MELO_INFO_STREAM(name);
    if (!eposEthercatSlavePtr) {
      MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
      // TODO: change back!
      continue;//return;
    }
    double offset = getJointOffset(name);

    MELO_INFO_STREAM("position: " << extendedJointTrajectories.position.size());

    JointTrajectory jointTrajectory;
    jointTrajectory.position = extendedJointTrajectories.position[i] - offset;
    eposEthercatSlavePtr->setSendJointTrajectory(jointTrajectory);
  }
}

HomingState EposEthercatSlaveManager::getHomingState(const std::string &name) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return HomingState::UNKNOWN;
  }

  return eposEthercatSlavePtr->getReceiveHomingState();
}

OperatingMode EposEthercatSlaveManager::getOperatingMode(const std::string &name) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return OperatingMode::UNKNOWN;
  }

  return eposEthercatSlavePtr->getReceiveOperatingMode();
}

void EposEthercatSlaveManager::setHomingState(const std::string &name, const HomingState &homingState) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return;
  }

  eposEthercatSlavePtr->setSendHomingState(homingState);
}

void EposEthercatSlaveManager::setDeviceState(const std::string &name, const varileg_msgs::DeviceState &deviceStateRos) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return;
  }

  DeviceState deviceState = ConversionTraits<DeviceState, varileg_msgs::DeviceState>::convert(deviceStateRos);
  eposEthercatSlavePtr->setSendDeviceState(deviceState);
}

const bool EposEthercatSlaveManager::isDeviceStateReachable(const std::string &name) const {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return false;
  }

  return eposEthercatSlavePtr->isDeviceStateReachable();
}

varileg_msgs::DeviceState EposEthercatSlaveManager::getDeviceState(const std::string &name) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    varileg_msgs::DeviceState deviceState;
    deviceState.state = varileg_msgs::DeviceState::STATE_UNKNOWN;
    return deviceState;
  }

  return ConversionTraits<DeviceState, varileg_msgs::DeviceState>::convert(eposEthercatSlavePtr->getReceiveDeviceState());
}

void EposEthercatSlaveManager::setEncoderConfig(const std::string &name,
                                                PositionUnitConverter primaryEncoderConverter,
                                                PositionUnitConverter secondaryEncoderConverter,
                                                std::unique_ptr<EncoderCrosschecker> encoderCrosschecker) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return;
  }

  eposEthercatSlavePtr->setPrimaryEncoderConverter(primaryEncoderConverter);
  eposEthercatSlavePtr->setSecondaryEncoderConverter(secondaryEncoderConverter);
  eposEthercatSlavePtr->setEncoderCrosschecker(std::move(encoderCrosschecker));
  MELO_INFO_STREAM("Slave: " << name << " primary converter: " << primaryEncoderConverter.getConversionFactor() << " secondary converter: " << secondaryEncoderConverter.getConversionFactor());
}

bool EposEthercatSlaveManager::writeSetup(const std::string &name, const EposConfig eposConfig) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return false;
  }

  return eposEthercatSlavePtr->writeSetup(eposConfig);
}

void EposEthercatSlaveManager::setOperatingMode(const std::string &name, const varileg_msgs::OperatingMode &operatingModeRos) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return;
  }

  OperatingMode operatingMode = ConversionTraits<OperatingMode, varileg_msgs::OperatingMode>::convert(operatingModeRos);
  eposEthercatSlavePtr->setSendOperatingMode(operatingMode);
}

bool EposEthercatSlaveManager::writeHomingMethod(const std::string &name, const varileg_msgs::HomingGoal::_mode_type &homingMode) {
  std::lock_guard<std::mutex> lock(mutex_);

  EposEthercatSlavePtr eposEthercatSlavePtr = getEposEthercatSlave(name);
  if (!eposEthercatSlavePtr) {
    MELO_ERROR_STREAM("Epos Slave with name " << name << " does not exist!")
    return false;
  }

  HomingMethod homingMethod;
  switch (homingMode) {
    case varileg_msgs::HomingGoal::MODE_NEGATIVE_DIRECTION:
      homingMethod = HomingMethod::INDEX_NEGATIVE_SPEED;
      break;
    case varileg_msgs::HomingGoal::MODE_POSITIVE_DIRECTION:
      homingMethod = HomingMethod::INDEX_POSITIVE_SPEED;
      break;
    default:
      MELO_ERROR_STREAM("Homing Mode does not exist: " << homingMode);
      return false;
  }

  return eposEthercatSlavePtr->writeHomingMethod(homingMethod);
}

bool EposEthercatSlaveManager::writeAllInterpolationTimePeriod(uint8_t timePeriod) {
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto &it : eposEthercatSlaves_) {
    EposEthercatSlavePtr eposEthercatSlavePtr = it.second;
    if(!eposEthercatSlavePtr->writeInterpolationTimePeriod(timePeriod)) {
      MELO_ERROR_STREAM(it.first << ": Could not set Interpolation TimePeriod");
      return false;
    }
  }

  return true;
}

}