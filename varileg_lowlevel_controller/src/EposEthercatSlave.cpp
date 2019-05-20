//
// Created by jolau on 05.03.19.
//

#include <varileg_lowlevel_controller/EposEthercatSlave.hpp>

#include "varileg_lowlevel_controller/EposEthercatSlave.hpp"

namespace varileg_lowlevel_controller {

EposEthercatSlave::EposEthercatSlave(const std::string &name,
                                     const soem_interface::EthercatBusBasePtr &bus,
                                     const uint32_t address,
                                     EposStartupConfig eposStartupConfig) : soem_interface::EthercatSlaveBase(bus, address),
                                                                            name_(name), eposStartupConfig_(eposStartupConfig) {
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

  writeInterpolationTimePeriod(eposStartupConfig_.interpolationTimePeriod);
  writeMotorCurrentLimit(eposStartupConfig_.motorCurrentLimit);

  isStartedUp_ = true;

  return true;
}

void EposEthercatSlave::readInbox() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  TxPdo txPdo;
  bus_->readTxPdo(address_, txPdo);
  receiveDeviceState_ = EposCommandLibrary::EposDeviceState::toDeviceState(txPdo.statusWord);
  receiveOperatingMode_ = EposCommandLibrary::EposOperatingMode::toOperatingMode(txPdo.operatingMode);

  switch (receiveOperatingMode_) {
    case OperatingMode::CSP: {
      receiveJointState_.position = jointSpecifications_.primaryEncoderConverter.toRad(txPdo.positionActualValue) + jointSpecifications_.homeOffset;
      receiveJointState_.primaryPosition = jointSpecifications_.primaryEncoderConverter.toRad(txPdo.positionPrimaryEncoder);
      receiveJointState_.secondaryPosition = jointSpecifications_.secondaryEncoderConverter.toRad(txPdo.positionSecondaryEncoder);
      receiveJointState_.velocity = txPdo.velocityActualValue;
      receiveJointState_.torque = txPdo.torqueActualValue;
      break;
    }
    case OperatingMode::HMM: {
      receiveHomingState_ = EposCommandLibrary::EposHomingState::toHomingState(txPdo.statusWord);
      break;
    }
  }
}

void EposEthercatSlave::writeOutbox() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  RxPdo rxPdo;

  rxPdo.operatingMode = EposCommandLibrary::EposOperatingMode::toOperatingModeCommand(sendOperatingMode_);

  uint16_t controlWord = 0;
  if(!jointSpecifications_.encoderCrosschecker.check(receiveJointState_.primaryPosition, receiveJointState_.secondaryPosition)) {
    MELO_ERROR_THROTTLE_STREAM(1.0, name_ << ": Encoder Crosscheck Failed with: Prim: " << receiveJointState_.primaryPosition << " Sec: " << receiveJointState_.secondaryPosition);
    sendDeviceState_ = DeviceState::STATE_SWITCHED_ON;
    applyNextDeviceStateTransition(controlWord, receiveDeviceState_, DeviceState::STATE_SWITCHED_ON);
    // TODO write error
  } else {
    // latch to current state if state is not reachable
    if (!applyNextDeviceStateTransition(controlWord, receiveDeviceState_, sendDeviceState_)) {
      applyNextDeviceStateTransition(controlWord, receiveDeviceState_, receiveDeviceState_);
    }
  }

  switch (receiveOperatingMode_) {
    case OperatingMode::CSP: {
      MELO_DEBUG_THROTTLE_STREAM(1.0, "Current Mode is CSP.");
      // apply position limits
      double targetPositionRad = boost::algorithm::clamp(sendJointTrajectory_.position, jointSpecifications_.minPositionLimit, jointSpecifications_.maxPositionLimit);
      MELO_INFO_THROTTLE_STREAM(1, name_ << ": targetPos: " << targetPositionRad);

      // apply home offset
      targetPositionRad -= jointSpecifications_.homeOffset;


      rxPdo.targetPosition = jointSpecifications_.primaryEncoderConverter.toInc(targetPositionRad);
      break;
    }
    case OperatingMode::HMM: {
      MELO_DEBUG_THROTTLE_STREAM(1.0, "Current Mode is HMM.");
      applyNextHomingStateTransition(controlWord, sendHomingState_);
      break;
    }
  }

  MELO_DEBUG_THROTTLE_STREAM(1.0, "Current state: " << Enum::toString(receiveDeviceState_) << " target state: "
                                     << Enum::toString(sendDeviceState_) << " next controlword "
                                     << std::bitset<16>(controlWord));
  rxPdo.controlWord = controlWord;

  bus_->writeRxPdo(address_, rxPdo);
}

void EposEthercatSlave::shutdown() {
  RxPdo rxPdo;
  rxPdo.controlWord = 0;
  bus_->writeRxPdo(address_, rxPdo);
}

bool EposEthercatSlave::writeHomingMethod(const HomingMethod &homingMethod) {
  return writeSDO(EposCommandLibrary::SDOs::HOMING_METHOD, EposCommandLibrary::EposHomingMethod::toHomingMethodCommand(homingMethod), true);
}

uint8_t EposEthercatSlave::readNodeId() {
  uint8_t nodeId = 0;
  if (!readSDO(EposCommandLibrary::SDOs::NODE_ID, nodeId, false)) {
    MELO_ERROR_STREAM("Could not read NodeID of " << name_);
  }
  return nodeId;
}

bool EposEthercatSlave::writeInterpolationTimePeriod(uint8_t timePeriod) {
  MELO_INFO_STREAM("Write Interpolation time period: " << static_cast<int>(timePeriod));
  return writeSDO(EposCommandLibrary::SDOs::INTERPOLATION_TIME_PERIOD_VALUE, timePeriod, false);
}

bool EposEthercatSlave::writeMotorCurrentLimit(uint32_t motorCurrent) {
  MELO_INFO_STREAM("Write Motor Current Limit:" << static_cast<int>(motorCurrent));
  return writeSDO(EposCommandLibrary::SDOs::OUTPUT_CURRENT_LIMIT, motorCurrent, false);
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

  MELO_ERROR_THROTTLE_STREAM(1.0,
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
    //  EposCommandLibrary::Controlwords::HOMING_HALT_DISABLE.apply(controlword);
      return true;
    case HomingState::HOMING_INTERRUPTED:
      EposCommandLibrary::Controlwords::HOMING_HALT_ENABLE.apply(controlword);
      return true;
    case HomingState::HOMING_SUCCESSFUL:
    case HomingState::HOMING_ERROR:
      MELO_ERROR_THROTTLE_STREAM(1.0, "Target Homing State " << Enum::toString(targetState) << " cannot be reached by master!");
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

bool EposEthercatSlave::setSendDeviceState(DeviceState sendDeviceState) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  sendDeviceState_ = sendDeviceState;

  uint16_t dummyControlword = 0;
  return applyNextDeviceStateTransition(dummyControlword, receiveDeviceState_, sendDeviceState_);
}

void EposEthercatSlave::setSendJointTrajectory(const JointTrajectory &sendJointTrajectory) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  sendJointTrajectory_ = sendJointTrajectory;
}

const JointState EposEthercatSlave::getReceiveJointState() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return receiveJointState_;
}

const OperatingMode EposEthercatSlave::getReceiveOperatingMode() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  return receiveOperatingMode_;
}

void EposEthercatSlave::setSendOperatingMode(OperatingMode sendOperatingMode) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  EposEthercatSlave::sendOperatingMode_ = sendOperatingMode;
}

void EposEthercatSlave::setJointSpecifications(JointSpecifications &jointSpecifications) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // set initial target position to home position, if not already set somehow else
  if(sendJointTrajectory_.position == jointSpecifications_.homeOffset) {
    sendJointTrajectory_.position = jointSpecifications.homeOffset;
  }

  jointSpecifications_ = jointSpecifications;
}

}