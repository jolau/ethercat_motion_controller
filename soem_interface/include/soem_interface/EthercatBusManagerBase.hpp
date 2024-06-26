#pragma once

// std
#include <map>
#include <memory>
#include <mutex>

#include <soem_interface/EthercatBusBase.hpp>

namespace soem_interface {

class EthercatBusManagerBase {
 public:
  EthercatBusManagerBase() = default;
  virtual ~EthercatBusManagerBase() = default;

  bool startupAllBuses(const std::map<std::string, std::vector<EthercatSlaveBasePtr>> &slavesOfBusesMap, bool waitForOperational = false);
  void receiveAllBusBuffers();
  void sendAllBusBuffers();
  void shutdownAllBuses(const std::map<std::string, std::vector<EthercatSlaveBasePtr>> &slavesOfBusesMap);

  EthercatBusBasePtr getBusByName(const std::string& name) {
    return buses_.at(name);
  }

  std::recursive_mutex* getMutex() { return &busMutex_; }
 protected:
  // Mutex prohibiting simultaneous access to EtherCAT bus manager.
  std::recursive_mutex busMutex_;
  std::map<std::string, soem_interface::EthercatBusBasePtr> buses_;
};

using EthercatBusManagerBasePtr = std::shared_ptr<EthercatBusManagerBase>;

}  // namespace soem_interface