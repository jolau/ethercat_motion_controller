//
// Created by jolau on 21.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_VARILEGETHERCATBUSMANAGER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_VARILEGETHERCATBUSMANAGER_HPP

#include <soem_interface/EthercatBusManagerBase.hpp>

namespace varileg_lowlevel_controller {
/**
 * Manages multiple buses
 */
class VarilegEthercatBusManager : public soem_interface::EthercatBusManagerBase {
 public:
  VarilegEthercatBusManager() = default;
  ~VarilegEthercatBusManager() = default;

  /**
   * Add bus to manager, stored by bus name
   * @param bus to store
   */
  void addEthercatBus(soem_interface::EthercatBusBasePtr bus);

 private:
};

using VarilegEthercatBusManagerPtr = std::shared_ptr<VarilegEthercatBusManager>;
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_VARILEGETHERCATBUSMANAGER_HPP
