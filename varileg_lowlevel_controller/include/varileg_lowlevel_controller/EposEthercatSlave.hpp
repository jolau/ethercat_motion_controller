//
// Created by jolau on 05.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP

// soem_interface
#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>
#include <varileg_lowlevel_controller/entities/JointState.hpp>
#include <varileg_lowlevel_controller/entities/JointTrajectory.hpp>
#include "varileg_lowlevel_controller/entities/RxPdo.hpp"
#include "varileg_lowlevel_controller/entities/TxPdo.hpp"
#include "EposCommandLibrary.hpp"
#include "varileg_lowlevel_controller/entities/DeviceState.hpp"
#include "varileg_lowlevel_controller/entities/HomingMethod.hpp"
#include "varileg_lowlevel_controller/entities/EposConfig.hpp"

namespace varileg_lowlevel_controller {
class EposEthercatSlave : public soem_interface::EthercatSlaveBase {
 public:
  EposEthercatSlave() = delete;
  EposEthercatSlave(const std::string &name, const soem_interface::EthercatBusBasePtr &bus, const uint32_t address);
  ~EposEthercatSlave() override = default;

  std::string getName() const override;

  PdoInfo getCurrentPdoInfo() const override;

  /**
  * Was startup successfully called?
  * @return
  */
  const bool isStartedUp() const { return isStartedUp_ && bus_->isStartedUp(); }

  void setSendJointTrajectory(const JointTrajectory &sendJointTrajectory);
  void setSendHomingState(HomingState sendHomingState);
  void setSendDeviceState(DeviceState sendMotorControllerState);

  const JointState getReceiveJointState() const;
  const HomingState getReceiveHomingState() const;
  const DeviceState getReceiveDeviceState() const;
  OperatingMode getCurrentOperatingMode() const;

  bool startup() override;
  void readInbox();
  void writeOutbox();
  void shutdown() override;

  bool setup(const EposConfig &eposConfig);

  uint8_t readNodeId();
  OperatingMode readOperatingMode();

  bool writeInterpolationTimePeriod(uint8_t timePeriod);
  bool writeOperatingMode(const OperatingMode &operatingMode);
  bool writeHomingMethod(const HomingMethod &homingMethod);
 private:
  template <typename Value>
  bool writeSDO(const SDO &sdo, const Value value, const bool completeAccess);

  template <typename Value>
  bool readSDO(const SDO &sdo, Value &value, const bool completeAccess);

  static bool applyNextDeviceStateTransition(uint16_t &controlword,
                                             const DeviceState &currentState,
                                             const DeviceState &targetState);

  static bool applyNextHomingStateTransition(uint16_t &controlword,
                                             const HomingState &targetState);

  const std::string name_;
  PdoInfo pdoInfo_;
  PositionUnitConverter primaryEncoderConverter_ = {1};
  PositionUnitConverter secondaryEncoderConverter_ = {1};

  OperatingMode currentOperatingMode_ = OperatingMode::UNKNOWN;

  JointState receiveJointState_;
  JointTrajectory sendJointTrajectory_;

  HomingState sendHomingState_;
  HomingState receiveHomingState_;

  DeviceState sendDeviceState_;
  DeviceState receiveDeviceState_;

  // Bool indicating if slave and bus startup was called
  bool isStartedUp_{false};
};

using EposEthercatSlavePtr = std::shared_ptr<EposEthercatSlave>;

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
