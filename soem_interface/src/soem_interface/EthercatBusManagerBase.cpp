// anydrive
#include "soem_interface/EthercatBusManagerBase.hpp"

namespace soem_interface {

bool EthercatBusManagerBase::startupAllBuses() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    if (!bus.second->startup()) {
      MELO_ERROR_STREAM("Failed to startup bus '" << bus.first << "'.");
      return false;
    }
  }

  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  for (auto& bus : buses_) {
    bus.second->setState(EC_STATE_OPERATIONAL);
  }

  return true;
}

void EthercatBusManagerBase::readAllBuses() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->updateRead();
  }
}

void EthercatBusManagerBase::writeToAllBuses() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->updateWrite();
  }
}

void EthercatBusManagerBase::shutdownAllBuses() {
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  for (auto& bus : buses_) {
    bus.second->shutdown();
  }
}

}  // namespace soem_interface
