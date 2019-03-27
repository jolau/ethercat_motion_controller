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

namespace varileg_lowlevel_controller {
const uint8_t STATE_NOT_READY_TO_SWITCH_ON   = 0;
const uint8_t STATE_SWITCH_ON_DISABLED       = 1;
const uint8_t STATE_READY_TO_SWITCH_ON       = 2;
const uint8_t STATE_SWITCHED_ON              = 3;
const uint8_t STATE_OP_ENABLED               = 4;
const uint8_t STATE_QUICK_STOP_ACTIVE        = 5;
const uint8_t STATE_FAULT_REACTION_ACTIVE    = 6;
const uint8_t STATE_FAULT                    = 7;

class EposEthercatSlave : public soem_interface::EthercatSlaveBase {
  public:
   EposEthercatSlave() = delete;
    EposEthercatSlave(const std::string& name, const soem_interface::EthercatBusBasePtr& bus, const uint32_t address);
  ~EposEthercatSlave() override = default;

   std::string getName() const override;

   PdoInfo getCurrentPdoInfo() const override;

   bool startup() override;
   void updateRead() override;
   void updateWrite() override;
   void shutdown() override;

   uint8_t readNodeId();
  private:
   bool applyNextStateTransition(uint16_t &controlword, const uint16_t currentStatusword, const uint8_t targetState);

   const std::string name_;
   PdoInfo pdoInfo_;

   TxPdo tx_pdo_;
   bool ready_;
};

 using EposEthercatSlavePtr = std::shared_ptr<EposEthercatSlave>;

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
