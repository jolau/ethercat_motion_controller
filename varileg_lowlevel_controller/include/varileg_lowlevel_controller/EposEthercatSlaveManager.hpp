//
// Created by jolau on 27.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVEMANAGER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVEMANAGER_HPP

#include "varileg_lowlevel_controller/EposEthercatSlave.hpp"
#include "mutex"
#include "varileg_msgs/ExtendedDeviceStates.h"
#include "varileg_msgs/ExtendedJointStates.h"
#include "varileg_msgs/ExtendedJointTrajectories.h"
#include "varileg_msgs/HomingGoal.h"

namespace varileg_lowlevel_controller {
class EposEthercatSlaveManager {
 public:
  EposEthercatSlaveManager() = default;
  ~EposEthercatSlaveManager();

  bool addEposEthercatSlave(EposEthercatSlavePtr eposEthercatSlave);
  void writeAllOutboxes();
  void readAllInboxes();

  varileg_msgs::ExtendedDeviceStates getExtendedDeviceStates();
  varileg_msgs::ExtendedJointStates getExtendedJointStates();
  varileg_msgs::DeviceState getDeviceState(const std::string &name);
  HomingState getHomingState(const std::string &name);
  OperatingMode getOperatingMode(const std::string &name);

  const bool isDeviceStateReachable(const std::string &name) const;

  void setExtendedJointTrajectories(const varileg_msgs::ExtendedJointTrajectories &extendedJointTrajectories);
  void setDeviceState(const std::string& name, const varileg_msgs::DeviceState &deviceStateRos);
  void setEncoderConfig(const std::string &name,
                        PositionUnitConverter primaryEncoderConverter,
                        PositionUnitConverter secondaryEncoderConverter,
                        EncoderCrosschecker encoderCrosschecker);
  void setHomingState(const std::string &name, const HomingState &homingState);
  void setOperatingMode(const std::string &name, const varileg_msgs::OperatingMode &operatingModeRos);

  bool writeSetup(const std::string& name, const EposConfig eposConfig);
  bool writeHomingMethod(const std::string& name, const varileg_msgs::HomingGoal::_mode_type &homingMode);
  bool writeAllInterpolationTimePeriod(uint8_t timePeriod);

  void setJointName2NodeIdMap(const std::map<std::string, int> &jointName2NodeIdMap);
  void setJointOffsetMap(const std::map<std::string, double> &jointOffsetMap);
 private:
  // Mutex prohibiting simultaneous access to EtherCAT slave manager.
  mutable std::mutex mutex_;

  std::map<std::string, EposEthercatSlavePtr> eposEthercatSlaves_;
  std::map<std::string, int> jointName2NodeIdMap_;
  std::map<std::string, double> jointOffsetMap_;

  EposEthercatSlavePtr getEposEthercatSlave(const std::string &name) const;
  double getJointOffset(const std::string &name) const;
  static void resizeExtendedJointStates(varileg_msgs::ExtendedJointStates &extendedJointStates, const unsigned long &size);
  static void resizeExtendedDeviceStates(varileg_msgs::ExtendedDeviceStates &extendedDeviceStates, const unsigned long &size);
};

using EposEthercatSlaveManagerPtr = std::shared_ptr<EposEthercatSlaveManager>;

}
#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVEMANAGER_HPP
