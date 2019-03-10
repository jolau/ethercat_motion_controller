//
// Created by jolau on 05.03.19.
//

#include <varileg_lowlevel_controller/EposEthercatSlave.hpp>

#include "varileg_lowlevel_controller/EposEthercatSlave.hpp"

namespace varileg_lowlevel_controller {

EposEthercatSlave::EposEthercatSlave(const std::string &name,
                                     const soem_interface::EthercatBusBasePtr &bus,
                                     const uint32_t address) : soem_interface::EthercatSlaveBase(bus, address), name_(name) {

}

bool EposEthercatSlave::startup() {
  return false;
}

void EposEthercatSlave::updateRead() {

}

void EposEthercatSlave::updateWrite() {

}

void EposEthercatSlave::shutdown() {

}

}