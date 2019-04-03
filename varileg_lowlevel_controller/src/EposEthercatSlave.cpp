//
// Created by jolau on 05.03.19.
//

#include <varileg_lowlevel_controller/EposEthercatSlave.hpp>

#include "varileg_lowlevel_controller/EposEthercatSlave.hpp"

namespace varileg_lowlevel_controller {

EposEthercatSlave::EposEthercatSlave(const std::string &name,
                                     const soem_interface::EthercatBusBasePtr &bus,
                                     const uint32_t address) : soem_interface::EthercatSlaveBase(bus, address),
                                                               name_(name) {
  pdoInfo_.rxPdoId_ = 0x1600;
  pdoInfo_.txPdoId_ = 0x1200;
  pdoInfo_.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo_.txPdoSize_ = sizeof(TxPdo);
}

std::string EposEthercatSlave::getName() const {
  return name_;
}

soem_interface::EthercatSlaveBase::PdoInfo EposEthercatSlave::getCurrentPdoInfo() const {
  return pdoInfo_;
}

bool EposEthercatSlave::startup() {
  bus_->setState(EC_STATE_PRE_OP);
  if (!bus_->waitForState(EC_STATE_PRE_OP)) {
    MELO_ERROR_STREAM(name_ << ": not entered PRE OP");
    return false;
  }

  if(!writeOperatingMode(OperatingMode::CSP)){
    MELO_ERROR_STREAM(name_ << ": Could not set CSP mode.")
    return false;
  }

  isStartedUp_ = true;

  return true;
}

void EposEthercatSlave::readInbox() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  TxPdo txPdo;

  bus_->readTxPdo(address_, txPdo);

  receiveJointState_.motorControllerState = getMotorControllerState(txPdo.StatusWord);
  receiveJointState_.position = txPdo.PositionActualValue;

  MELO_INFO_STREAM(name_ << ": RSF Encoder: " << ((float) txPdo.PositionActualValue)  << " and MILE Encoder: " << ((float) txPdo.PositionSecondEncoder) << " diff: " << (txPdo.PositionActualValue - txPdo.PositionSecondEncoder));
  float correctedRsfPosition = (float) txPdo.PositionActualValue / 40181;
  float correctedMilePosition = (float) -1 * txPdo.PositionSecondEncoder / 2176000;
  MELO_INFO_STREAM(name_ << ": RSF Encoder: " << correctedRsfPosition << " and MILE Encoder: " << correctedMilePosition << " diff: " << (correctedMilePosition - correctedRsfPosition))
}

void EposEthercatSlave::writeOutbox() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  RxPdo rxPdo;

  uint16_t controlWord = 0;

  applyNextStateTransition(controlWord, receiveJointState_.motorControllerState, sendJointState_.motorControllerState);
  MELO_INFO_STREAM("Current state: " << static_cast<int>(receiveJointState_.motorControllerState) << " target state: " << static_cast<int>(sendJointState_.motorControllerState) << " next controlword " << std::bitset<16>(controlWord));

  rxPdo.ControlWord = controlWord;
  rxPdo.TargetPosition = sendJointState_.position;

  bus_->writeRxPdo(address_, rxPdo);
}

void EposEthercatSlave::shutdown() {
  RxPdo rxPdo;
  rxPdo.ControlWord = 0;
  bus_->writeRxPdo(address_, rxPdo);
}

bool EposEthercatSlave::writeOperatingMode(const OperatingMode &operatingMode) {
  uint8_t operatingModeCommand = EposCommandLibrary::EposOperatingMode::toOperatingModeCommand(operatingMode);

  // set mode to CSP
  if(!writeSDO(EposCommandLibrary::SDOs::MODES_OF_OPERATION, operatingModeCommand, true)) {
    MELO_ERROR_STREAM(name_ << ": Could not set CSP mode.")
    return false;
  }

  if(operatingMode != readOperatingMode()) {
    MELO_ERROR_STREAM("Operating mode could not be set");
    return false;
  }

  return true;
}

OperatingMode EposEthercatSlave::readOperatingMode() {
  int8_t mode = 0;

  readSDO(EposCommandLibrary::SDOs::MODES_OF_OPERATION_DISPLAY, mode, false);

  return EposCommandLibrary::EposOperatingMode::toOperatingMode(mode);
}

uint8_t EposEthercatSlave::readNodeId() {
  uint8_t nodeId = 0;
  if(!readSDO(EposCommandLibrary::SDOs::NODE_ID, nodeId, false)) {
    MELO_ERROR_STREAM("Could not read NodeID of " << name_);
  }
  return nodeId;
}

bool EposEthercatSlave::writeInterpolationTimePeriod(uint8_t timePeriod) {
  return writeSDO(EposCommandLibrary::SDOs::INTERPOLATION_TIME_PERIOD_VALUE, timePeriod, true);
}

template<typename Value>
bool EposEthercatSlave::writeSDO(const SDO &sdo, const Value value, const bool completeAccess) {
  if(!sendSdoWrite(sdo.index, sdo.subindex, completeAccess, value)) {
    MELO_ERROR_STREAM("Write SDO: " << sdo.name << " with value: " << value << " didn't work.")
    return false;
  }
  return true;
}

template<typename Value>
bool EposEthercatSlave::readSDO(const SDO &sdo, Value &value, const bool completeAccess) {
  if(!sendSdoRead(sdo.index, sdo.subindex, completeAccess, value)) {
    MELO_ERROR_STREAM("Read SDO: " << sdo.name << " didn't work.")
    return false;
  }
  return true;
}

bool EposEthercatSlave::applyNextStateTransition(uint16_t &controlword,
                                                 const MotorControllerState &currentState,
                                                 const MotorControllerState &targetState) {
  switch (currentState) {
    case MotorControllerState::STATE_UNKNOWN:
      return false;
    case MotorControllerState::STATE_NOT_READY_TO_SWITCH_ON: MELO_INFO_STREAM(
        "Doing nothing, will be autotransfered to state SWITCH ON DISABLED.")
      return true;
    case MotorControllerState::STATE_SWITCH_ON_DISABLED:
      switch (targetState) {
        case MotorControllerState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
        case MotorControllerState::STATE_READY_TO_SWITCH_ON:
        case MotorControllerState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::SHUTDOWN.apply(controlword);
          return true;
      }
      break;
    case MotorControllerState::STATE_READY_TO_SWITCH_ON:
      switch (targetState) {
        case MotorControllerState::STATE_READY_TO_SWITCH_ON:
          EposCommandLibrary::Controlwords::SHUTDOWN.apply(controlword);
          return true;
        case MotorControllerState::STATE_SWITCHED_ON:
          EposCommandLibrary::Controlwords::SWITCH_ON.apply(controlword);
          return true;
        case MotorControllerState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::SWITCH_ON_ENABLE_OP.apply(controlword);
          return true;
        case MotorControllerState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
      }
      break;
    case MotorControllerState::STATE_SWITCHED_ON:
      switch (targetState) {
        case MotorControllerState::STATE_SWITCHED_ON:
          EposCommandLibrary::Controlwords::SWITCH_ON.apply(controlword);
          return true;
        case MotorControllerState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::ENABLE_OP.apply(controlword);
          return true;
        case MotorControllerState::STATE_READY_TO_SWITCH_ON:
          EposCommandLibrary::Controlwords::SHUTDOWN.apply(controlword);
          return true;
        case MotorControllerState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
      }
      break;
    case MotorControllerState::STATE_OP_ENABLED:
      switch (targetState) {
        case MotorControllerState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::ENABLE_OP.apply(controlword);
          return true;
        case MotorControllerState::STATE_QUICK_STOP_ACTIVE:
          EposCommandLibrary::Controlwords::QUICK_STOP.apply(controlword);
          return true;
        case MotorControllerState::STATE_SWITCHED_ON:
          EposCommandLibrary::Controlwords::DISABLE_OP.apply(controlword);
          return true;
        case MotorControllerState::STATE_READY_TO_SWITCH_ON:
          EposCommandLibrary::Controlwords::SHUTDOWN.apply(controlword);
          return true;
        case MotorControllerState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
      }
      break;
    case MotorControllerState::STATE_QUICK_STOP_ACTIVE:
      switch (targetState) {
        case MotorControllerState::STATE_QUICK_STOP_ACTIVE:
          EposCommandLibrary::Controlwords::QUICK_STOP.apply(controlword);
          return true;
        case MotorControllerState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::ENABLE_OP.apply(controlword);
          return true;
        case MotorControllerState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
      }
      break;
    case MotorControllerState::STATE_FAULT_REACTION_ACTIVE: MELO_INFO_STREAM("Doing nothing, will be autotransfered to state FAULT.")
      return true;
    case MotorControllerState::STATE_FAULT:
      switch (targetState) {
        case MotorControllerState::STATE_FAULT:
          return true;
        case MotorControllerState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::FAULT_RESET.apply(controlword);
          return true;
      }
      break;
  }

  MELO_ERROR_STREAM(
      "Target State: " << static_cast<int>(targetState) << " can't be reached from current State: " << static_cast<int>(currentState));
  return false;
}

void EposEthercatSlave::setSendJointState(const ExtendedJointState &sendJointState) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  sendJointState_ = sendJointState;
}

const ExtendedJointState &EposEthercatSlave::getReceiveJointState() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return receiveJointState_;
}

MotorControllerState EposEthercatSlave::getMotorControllerState(const uint16_t &statusword) {
  for (const auto &it : STATE_STATUSWORD_MAP) {
    if (it.second.isActive(statusword)) {
      MELO_INFO_STREAM("Mapping Statusword code " << std::bitset<16>(statusword) << " to state " << static_cast<int>(it.first));
      return it.first;
    }
  }

  MELO_ERROR_STREAM("Unknown Statusword: " << statusword);
  return MotorControllerState::STATE_UNKNOWN;
}


}