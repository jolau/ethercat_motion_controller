//
// Created by jolau on 05.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP

// soem_interface
#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>
#include "RxPdo.hpp"
#include "TxPdo.hpp"
#include "ExtendedJointState.hpp"
#include "EposCommandLibrary.hpp"
#include "DeviceState.hpp"
#include "HomingMethod.hpp"

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

  bool startup() override;
  void readInbox();
  void writeOutbox();
  void shutdown() override;

  void setSendJointState(const ExtendedJointState &sendJointState);
  void setSendHomingState(HomingState sendHomingState);
  void setSendDeviceState(DeviceState sendMotorControllerState);

  const ExtendedJointState &getReceiveJointState() const;
  const HomingState getReceiveHomingState() const;
  const DeviceState getReceiveDeviceState() const;

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

  OperatingMode currentOperatingMode_ = OperatingMode::UNKNOWN;

  ExtendedJointState sendJointState_;
  ExtendedJointState receiveJointState_;

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
