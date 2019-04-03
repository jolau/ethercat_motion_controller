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
#include "MotorControllerState.hpp"

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
  const ExtendedJointState &getReceiveJointState() const;

  uint8_t readNodeId();
  bool writeInterpolationTimePeriod(uint8_t timePeriod);
  bool writeOperatingMode(const OperatingMode &operatingMode);
  OperatingMode readOperatingMode();
 private:
  template <typename Value>
  bool writeSDO(const SDO &sdo, const Value value, const bool completeAccess);

  template <typename Value>
  bool readSDO(const SDO &sdo, Value &value, const bool completeAccess);

  static bool applyNextStateTransition(uint16_t &controlword,
                                const MotorControllerState &currentState,
                                const MotorControllerState &targetState);
  MotorControllerState getMotorControllerState(const uint16_t &statusword);

  const std::map<MotorControllerState, Statusword> STATE_STATUSWORD_MAP
      {
          {MotorControllerState::STATE_NOT_READY_TO_SWITCH_ON,
           EposCommandLibrary::Statuswords::NOT_READY_TO_SWITCH_ON},
          {MotorControllerState::STATE_SWITCH_ON_DISABLED,
           EposCommandLibrary::Statuswords::SWITCH_ON_DISABLED},
          {MotorControllerState::STATE_READY_TO_SWITCH_ON,
           EposCommandLibrary::Statuswords::READY_TO_SWITCH_ON},
          {MotorControllerState::STATE_SWITCHED_ON,
           EposCommandLibrary::Statuswords::SWITCHED_ON},
          {MotorControllerState::STATE_OP_ENABLED,
           EposCommandLibrary::Statuswords::OP_ENABLED},
          {MotorControllerState::STATE_QUICK_STOP_ACTIVE,
           EposCommandLibrary::Statuswords::QUICK_STOP_ACTIVE},
          {MotorControllerState::STATE_FAULT_REACTION_ACTIVE,
           EposCommandLibrary::Statuswords::FAULT_REACTION_ACTIVE},
          {MotorControllerState::STATE_FAULT, EposCommandLibrary::Statuswords::FAULT}
      };

  const std::string name_;
  PdoInfo pdoInfo_;

  ExtendedJointState sendJointState_;
  ExtendedJointState receiveJointState_;

  //! Bool indicating if slave and bus startup was called
  bool isStartedUp_{false};
};

using EposEthercatSlavePtr = std::shared_ptr<EposEthercatSlave>;

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
