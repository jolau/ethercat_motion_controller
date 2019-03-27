#include "soem_interface/EthercatBusManagerBase.hpp"

namespace soem_interface {

bool EthercatBusManagerBase::startupAllBuses(const std::map<std::string, std::vector<EthercatSlaveBasePtr>> &slavesOfBusesMap, bool waitForOperational) {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    if (!bus.second->startup(slavesOfBusesMap.at(bus.first))) {
      MELO_ERROR_STREAM("Failed to startup bus '" << bus.first << "'.");
      return false;
    }
  }

  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  for (auto& bus : buses_) {
    bus.second->setState(EC_STATE_OPERATIONAL);
    if(waitForOperational) {
      bus.second->waitForState(EC_STATE_OPERATIONAL);
    }
  }

  return true;
}

void EthercatBusManagerBase::receiveAllBusBuffers() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->receiveBuffer();
  }
}

void EthercatBusManagerBase::sendAllBusBuffers() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->sendBuffer();
  }
}

void EthercatBusManagerBase::shutdownAllBuses(const std::map<std::string, std::vector<EthercatSlaveBasePtr>> &slavesOfBusesMap) {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->shutdown(slavesOfBusesMap.at(bus.first));
  }
}

}  // namespace soem_interface
