//
// Created by jolau on 21.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_VARILEGETHERCATBUSMANAGER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_VARILEGETHERCATBUSMANAGER_HPP

#include <soem_interface/EthercatBusManagerBase.hpp>
class VarilegEthercatBusManager : public soem_interface::EthercatBusManagerBase {
 public:
  VarilegEthercatBusManager() = default;
  ~VarilegEthercatBusManager() = default;

  void addEthercatBus(const soem_interface::EthercatBusBasePtr& bus);
};

using VarilegEthercatBusManagerPtr = std::shared_ptr<VarilegEthercatBusManager>;

#endif //VARILEG_LOWLEVEL_CONTROLLER_VARILEGETHERCATBUSMANAGER_HPP
