//
// Created by jolau on 21.03.19.
//

#include "varileg_lowlevel_controller/VarilegEthercatBusManager.hpp"

void VarilegEthercatBusManager::addEthercatBus(soem_interface::EthercatBusBasePtr bus) {
  const auto &it = buses_.find(bus->getName());
  if (it == buses_.end()) {
    buses_.insert(std::make_pair(bus->getName(), bus));
  }
}