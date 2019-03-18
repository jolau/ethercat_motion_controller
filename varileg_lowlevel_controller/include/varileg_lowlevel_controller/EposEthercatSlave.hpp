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
 class EposEthercatSlave : public soem_interface::EthercatSlaveBase {
  public:
   EposEthercatSlave() = delete;
    EposEthercatSlave(const std::string& name, const soem_interface::EthercatBusBasePtr& bus, const uint32_t address);
  ~EposEthercatSlave() override = default;

   bool startup() override;
   void updateRead() override;
   void updateWrite() override;
   void shutdown() override;

   std::string getName() const override;
   PdoInfo getCurrentPdoInfo() const override;

  private:
   const std::string name_;
   TxPdo tx_pdo_;
   bool ready_;
   PdoInfo pdoInfo_;
};

 using EposEthercatSlavePtr = std::shared_ptr<EposEthercatSlave>;

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
