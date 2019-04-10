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

  if (!writeOperatingMode(OperatingMode::CSP)) {
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
  receiveDeviceState_ = EposCommandLibrary::EposDeviceState::toDeviceState(txPdo.statusWord);

  switch (currentOperatingMode_) {
    case OperatingMode::CSP: {
      receiveJointState_.position = primaryEncoderConverter_.toRad(txPdo.positionActualValue);
      receiveJointState_.primaryPosition = primaryEncoderConverter_.toRad(txPdo.positionPrimaryEncoder);
      receiveJointState_.secondaryPosition = secondaryEncoderConverter_.toRad(txPdo.positionSecondaryEncoder);
      receiveJointState_.velocity = txPdo.velocityActualValue;
      receiveJointState_.torque = txPdo.torqueActualValue;

     /* MELO_INFO_STREAM(
          name_ << ": RSF Encoder: " << ((float) txPdo.positionActualValue) << " and MILE Encoder: "
                << ((float) txPdo.PositionSecondEncoder)
                << " diff: " << (txPdo.positionActualValue - txPdo.PositionSecondEncoder));
      float correctedRsfPosition = (float) txPdo.positionActualValue / 40181;
      float correctedMilePosition = (float) -1 * txPdo.PositionSecondEncoder / 2176000;
      MELO_INFO_STREAM(name_ << ": RSF Encoder: " << correctedRsfPosition << " and MILE Encoder: " << correctedMilePosition << " diff: "
                             << (correctedMilePosition - correctedRsfPosition))*/
      break;
    }
    case OperatingMode::HMM: {
      HomingState newHomingState = EposCommandLibrary::EposHomingState::toHomingState(txPdo.statusWord);

      // Only if state has changed compared to last cycle
      if(newHomingState != receiveHomingState_ && newHomingState == HomingState::HOMING_SUCCESSFUL) {
        // TODO: set new mile home position
      }

      receiveHomingState_ = newHomingState;
      break;
    }
  }
}

void EposEthercatSlave::writeOutbox() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  RxPdo rxPdo;

  uint16_t controlWord = 0;
  applyNextDeviceStateTransition(controlWord, receiveDeviceState_, sendDeviceState_);
  MELO_INFO_STREAM("Current state: " << Enum::toString(receiveDeviceState_) << " target state: "
                                     << Enum::toString(sendDeviceState_) << " next controlword "
                                     << std::bitset<16>(controlWord));

  switch (currentOperatingMode_) {
    case OperatingMode::CSP: {
      rxPdo.targetPosition = primaryEncoderConverter_.toInc(sendJointTrajectory_.position);
      MELO_INFO_STREAM("rx.Target Position: " << rxPdo.targetPosition);
      break;
    }
    case OperatingMode::HMM: {
      applyNextHomingStateTransition(controlWord, sendHomingState_);
      break;
    }
  }

  rxPdo.controlWord = controlWord;

  bus_->writeRxPdo(address_, rxPdo);
}

void EposEthercatSlave::shutdown() {
  RxPdo rxPdo;
  rxPdo.controlWord = 0;
  bus_->writeRxPdo(address_, rxPdo);
}

bool EposEthercatSlave::setup(const EposConfig &eposConfig) {
  assert(isStartedUp_);

  primaryEncoderConverter_ = eposConfig.primaryEncoderConverter;
  secondaryEncoderConverter_ = eposConfig.secondaryEncoderConverter;

  if(!writeSDO(EposCommandLibrary::SDOs::SOFTWARE_MIN_POSITION_LIMIT, eposConfig.minPositionLimitInc, true)) {
    return false;
  }

  if(!writeSDO(EposCommandLibrary::SDOs::SOFTWARE_MAX_POSITION_LIMIT, eposConfig.maxPositionLimitInc, true)) {
    return false;
  }

  if(!writeSDO(EposCommandLibrary::SDOs::MAX_MOTOR_SPEED, eposConfig.maxMotorSpeedRpm, true)) {
    return false;
  }

  return true;
}

bool EposEthercatSlave::writeOperatingMode(const OperatingMode &operatingMode) {
  uint8_t operatingModeCommand = EposCommandLibrary::EposOperatingMode::toOperatingModeCommand(operatingMode);

  // set mode to CSP
  if (!writeSDO(EposCommandLibrary::SDOs::MODES_OF_OPERATION, operatingModeCommand, true)) {
    MELO_ERROR_STREAM(name_ << ": Could not set CSP mode.")
    return false;
  }

  if (operatingMode != readOperatingMode()) {
    MELO_ERROR_STREAM("Operating mode could not be set");
    return false;
  }

  return true;
}

bool EposEthercatSlave::writeHomingMethod(const HomingMethod &homingMethod) {
  return writeSDO(EposCommandLibrary::SDOs::HOMING_METHOD, EposCommandLibrary::EposHomingMethod::toHomingMethodCommand(homingMethod), true);
}

OperatingMode EposEthercatSlave::readOperatingMode() {
  int8_t mode = 0;

  readSDO(EposCommandLibrary::SDOs::MODES_OF_OPERATION_DISPLAY, mode, false);

  currentOperatingMode_ = EposCommandLibrary::EposOperatingMode::toOperatingMode(mode);

  return currentOperatingMode_;
}

uint8_t EposEthercatSlave::readNodeId() {
  uint8_t nodeId = 0;
  if (!readSDO(EposCommandLibrary::SDOs::NODE_ID, nodeId, false)) {
    MELO_ERROR_STREAM("Could not read NodeID of " << name_);
  }
  return nodeId;
}

bool EposEthercatSlave::writeInterpolationTimePeriod(uint8_t timePeriod) {
  return writeSDO(EposCommandLibrary::SDOs::INTERPOLATION_TIME_PERIOD_VALUE, timePeriod, true);
}

template<typename Value>
bool EposEthercatSlave::writeSDO(const SDO &sdo, const Value value, const bool completeAccess) {
  if (!sendSdoWrite(sdo.index, sdo.subindex, completeAccess, value)) {
    MELO_ERROR_STREAM("Write SDO: " << sdo.name << " with value: " << value << " didn't work.")
    return false;
  }
  return true;
}

template<typename Value>
bool EposEthercatSlave::readSDO(const SDO &sdo, Value &value, const bool completeAccess) {
  if (!sendSdoRead(sdo.index, sdo.subindex, completeAccess, value)) {
    MELO_ERROR_STREAM("Read SDO: " << sdo.name << " didn't work.")
    return false;
  }
  return true;
}

bool EposEthercatSlave::applyNextDeviceStateTransition(uint16_t &controlword,
                                                       const DeviceState &currentState,
                                                       const DeviceState &targetState) {
  switch (currentState) {
    case DeviceState::STATE_UNKNOWN:
      return false;
    case DeviceState::STATE_NOT_READY_TO_SWITCH_ON: MELO_INFO_STREAM(
        "Doing nothing, will be autotransfered to state SWITCH ON DISABLED.")
      return true;
    case DeviceState::STATE_SWITCH_ON_DISABLED:
      switch (targetState) {
        case DeviceState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
        case DeviceState::STATE_READY_TO_SWITCH_ON:
        case DeviceState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::SHUTDOWN.apply(controlword);
          return true;
      }
      break;
    case DeviceState::STATE_READY_TO_SWITCH_ON:
      switch (targetState) {
        case DeviceState::STATE_READY_TO_SWITCH_ON:
          EposCommandLibrary::Controlwords::SHUTDOWN.apply(controlword);
          return true;
        case DeviceState::STATE_SWITCHED_ON:
          EposCommandLibrary::Controlwords::SWITCH_ON.apply(controlword);
          return true;
        case DeviceState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::SWITCH_ON_ENABLE_OP.apply(controlword);
          return true;
        case DeviceState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
      }
      break;
    case DeviceState::STATE_SWITCHED_ON:
      switch (targetState) {
        case DeviceState::STATE_SWITCHED_ON:
          EposCommandLibrary::Controlwords::SWITCH_ON.apply(controlword);
          return true;
        case DeviceState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::ENABLE_OP.apply(controlword);
          return true;
        case DeviceState::STATE_READY_TO_SWITCH_ON:
          EposCommandLibrary::Controlwords::SHUTDOWN.apply(controlword);
          return true;
        case DeviceState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
      }
      break;
    case DeviceState::STATE_OP_ENABLED:
      switch (targetState) {
        case DeviceState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::ENABLE_OP.apply(controlword);
          return true;
        case DeviceState::STATE_QUICK_STOP_ACTIVE:
          EposCommandLibrary::Controlwords::QUICK_STOP.apply(controlword);
          return true;
        case DeviceState::STATE_SWITCHED_ON:
          EposCommandLibrary::Controlwords::DISABLE_OP.apply(controlword);
          return true;
        case DeviceState::STATE_READY_TO_SWITCH_ON:
          EposCommandLibrary::Controlwords::SHUTDOWN.apply(controlword);
          return true;
        case DeviceState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
      }
      break;
    case DeviceState::STATE_QUICK_STOP_ACTIVE:
      switch (targetState) {
        case DeviceState::STATE_QUICK_STOP_ACTIVE:
          EposCommandLibrary::Controlwords::QUICK_STOP.apply(controlword);
          return true;
        case DeviceState::STATE_OP_ENABLED:
          EposCommandLibrary::Controlwords::ENABLE_OP.apply(controlword);
          return true;
        case DeviceState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::DISABLE_VOLTAGE.apply(controlword);
          return true;
      }
      break;
    case DeviceState::STATE_FAULT_REACTION_ACTIVE: MELO_INFO_STREAM("Doing nothing, will be autotransfered to state FAULT.")
      return true;
    case DeviceState::STATE_FAULT:
      switch (targetState) {
        case DeviceState::STATE_FAULT:
          return true;
        case DeviceState::STATE_SWITCH_ON_DISABLED:
          EposCommandLibrary::Controlwords::FAULT_RESET.apply(controlword);
          return true;
      }
      break;
  }

  MELO_ERROR_STREAM(
      "Target State: " << Enum::toString(targetState) << " can't be reached from current State: " << Enum::toString(currentState));
  return false;
}

bool EposEthercatSlave::applyNextHomingStateTransition(uint16_t &controlword,
                                                       const HomingState &targetState) {
  switch(targetState) {
    case HomingState::UNKNOWN:
      return false;
    case HomingState::HOMING_IN_PROGRESS:
      EposCommandLibrary::Controlwords::HOMING_OP_START.apply(controlword);
      EposCommandLibrary::Controlwords::HOMING_HALT_DISABLE.apply(controlword);
      return true;
    case HomingState::HOMING_INTERRUPTED:
      EposCommandLibrary::Controlwords::HOMING_HALT_ENABLE.apply(controlword);
      return true;
    case HomingState::HOMING_SUCCESSFUL:
    case HomingState::HOMING_ERROR:
      MELO_ERROR_STREAM("Target Homing State " << Enum::toString(targetState) << " cannot be reached by master!");
      return false;
  }
}

const HomingState EposEthercatSlave::getReceiveHomingState() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return receiveHomingState_;
}

const DeviceState EposEthercatSlave::getReceiveDeviceState() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return receiveDeviceState_;
}

void EposEthercatSlave::setSendHomingState(HomingState sendHomingState) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  sendHomingState_ = sendHomingState;
}

void EposEthercatSlave::setSendDeviceState(DeviceState sendMotorControllerState) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  sendDeviceState_ = sendMotorControllerState;
}

void EposEthercatSlave::setSendJointTrajectory(const JointTrajectory &sendJointTrajectory) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  sendJointTrajectory_ = sendJointTrajectory;
}

const JointState EposEthercatSlave::getReceiveJointState() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return JointState();
}
OperatingMode EposEthercatSlave::getCurrentOperatingMode() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return currentOperatingMode_;
}

}