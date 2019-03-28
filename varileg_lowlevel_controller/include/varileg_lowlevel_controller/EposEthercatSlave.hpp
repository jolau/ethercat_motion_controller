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
#include "varileg_lowlevel_controller_msgs/MotorControllerState.h"
#include <varileg_lowlevel_controller/EposCommandLibrary.hpp>

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
  void updateRead() override;
  void updateWrite() override;
  void shutdown() override;

  uint8_t readNodeId();
 private:
  bool applyNextStateTransition(uint16_t &controlword,
                                const uint16_t currentStatusword,
                                const varileg_lowlevel_controller_msgs::MotorControllerState targetState);

  const std::map<uint8_t, Statusword> STATE_STATUSWORD_MAP
      {
          {varileg_lowlevel_controller_msgs::MotorControllerState::STATE_NOT_READY_TO_SWITCH_ON,
           EposCommandLibrary::Statuswords::NOT_READY_TO_SWITCH_ON},
          {varileg_lowlevel_controller_msgs::MotorControllerState::STATE_SWITCH_ON_DISABLED,
           EposCommandLibrary::Statuswords::SWITCH_ON_DISABLED},
          {varileg_lowlevel_controller_msgs::MotorControllerState::STATE_READY_TO_SWITCH_ON,
           EposCommandLibrary::Statuswords::READY_TO_SWITCH_ON},
          {varileg_lowlevel_controller_msgs::MotorControllerState::STATE_SWITCHED_ON,
           EposCommandLibrary::Statuswords::SWITCHED_ON},
          {varileg_lowlevel_controller_msgs::MotorControllerState::STATE_OP_ENABLED,
           EposCommandLibrary::Statuswords::OP_ENABLED},
          {varileg_lowlevel_controller_msgs::MotorControllerState::STATE_QUICK_STOP_ACTIVE,
           EposCommandLibrary::Statuswords::QUICK_STOP_ACTIVE},
          {varileg_lowlevel_controller_msgs::MotorControllerState::STATE_FAULT_REACTION_ACTIVE,
           EposCommandLibrary::Statuswords::FAULT_REACTION_ACTIVE},
          {varileg_lowlevel_controller_msgs::MotorControllerState::STATE_FAULT, EposCommandLibrary::Statuswords::FAULT}
      };

  const std::string name_;
  PdoInfo pdoInfo_;

  //! Bool indicating if slave and bus startup was called
  bool isStartedUp_{false};

  TxPdo tx_pdo_;
  bool ready_;
};

using EposEthercatSlavePtr = std::shared_ptr<EposEthercatSlave>;

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
