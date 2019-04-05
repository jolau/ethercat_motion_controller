//
// Created by jolau on 27.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVEMANAGER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVEMANAGER_HPP

#include "varileg_lowlevel_controller/EposEthercatSlave.hpp"

namespace varileg_lowlevel_controller {
class EposEthercatSlaveManager {
 public:
  EposEthercatSlaveManager() {};
  ~EposEthercatSlaveManager();

  bool addEposEthercatSlave(EposEthercatSlavePtr eposEthercatSlave);
  void writeAllOutboxes();
  void readAllInboxes();

  void setJointName2NodeIdMap(const std::map<std::string, int> &jointName2NodeIdMap);
 private:
  std::map<std::string, EposEthercatSlavePtr> eposEthercatSlaves_;
  std::map<std::string, int> jointName2NodeIdMap_;
};

using EposEthercatSlaveManagerPtr = std::shared_ptr<EposEthercatSlaveManager>;

}
#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVEMANAGER_HPP
