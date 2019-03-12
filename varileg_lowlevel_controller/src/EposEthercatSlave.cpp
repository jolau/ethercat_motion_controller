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
  // PDO mapping
  bus_->sendSdoWrite(address_, 0x1c12, 01, true, 0x1600);

  bus_->sendSdoWrite(address_, 0x1c13,01, true, 0x1a00);

  // set mode to CSP
  bus_->sendSdoWrite(address_, 0x6060, 0x0, false, 0X08);

  return false;
}

void EposEthercatSlave::updateRead() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  bus_->readTxPdo(address_, txPdo);
}

void EposEthercatSlave::updateWrite() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);


}

void EposEthercatSlave::shutdown() {

}

}