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
/**
 * Manages multiple EposEthercatSlave independent of their bus. It maps them by their joint name.
 * Represents the link between the ROS message type and the local ones.
 */
class EposEthercatSlaveManager {
 public:
  EposEthercatSlaveManager() = default;
  ~EposEthercatSlaveManager();

  /**
   * Requests the NodeID of the eposEthercatSlave and looks up its joint name as specified in jointName2NodeIdMap_.
   * Stores it by its joint name for further use.
   * @param eposEthercatSlave started up EPOS to be added.
   * @return true, if eposEthercatSlave could be added (NodeID could be retrieved and mapping was present)
   */
  bool addEposEthercatSlave(EposEthercatSlavePtr eposEthercatSlave);

  /**
   * Calls WriteOutbox on all epos.
   */
  void writeAllOutboxes();

  /**
 * Calls ReadInbox on all epos.
 */
  void readAllInboxes();

  /**
   * Convert and combine all #DeviceState to an ExtendedDeviceStates
   * @return ExtendedDeviceStates of all epos slaves
   */
  varileg_msgs::ExtendedDeviceStates getExtendedDeviceStates();

  /**
   * Convert and combine all #JointState of last update cycle to an ExtendedJointStates
   * @return ExtendedJointStates of all epos slaves
   */
  varileg_msgs::ExtendedJointStates getExtendedJointStates();

  /**
   * Get DeviceState of last update cycle by the joint name
   * @param name joint name
   * @return DeviceState of slave. If no slave was found, #varileg_msgs::DeviceState::STATE_UNKNOWN is returned.
   */
  varileg_msgs::DeviceState getDeviceState(const std::string &name);

  /**
 * Get HomingState of last update cycle by the joint name
 * @param name joint name
 * @return HomingState of slave. If no slave was found, #HomingState::UNKNOWN is returned.
 */
  HomingState getHomingState(const std::string &name);

  /**
 * Get OperatingMode of last update cycle by the joint name
 * @param name joint name
 * @return OperatingMode of slave. If no slave was found, #OperatingMode::UNKNOWN is returned.
 */
  OperatingMode getOperatingMode(const std::string &name);

  /**
   * Set #JointTrajectory of all in #ExtendedJointTrajectories specified slaves. To be sent with next update cycle.
   * @param extendedJointTrajectories
   */
  void setExtendedJointTrajectories(const varileg_msgs::ExtendedJointTrajectories &extendedJointTrajectories);

  /**
   * Set targeted DeviceState by joint name. To be sent with next update cycle.
   * @param name joint name
   * @param deviceStateRos targeted device state
   * @return is target DeviceState is reachable
   */
  bool setDeviceState(const std::string &name, const varileg_msgs::DeviceState &deviceStateRos);

  /**
   * Set JointSpecifications of EPOS slave by joint name.
   * @param name joint name
   * @param jointSpecifications
   */
  void setJointSpecifications(const std::string &name, JointSpecifications jointSpecifications);

  /**
   * Set targeted HomingState by joint name. To be sent with next update cycle.
   * @param name joint name
   * @param homingState targeted HomingState
   */
  void setHomingState(const std::string &name, const HomingState &homingState);

  /**
   * Set OperatingMode by joint name. To be sent with next update cycle.
   * @param name joint name
   * @param operatingModeRos
   */
  void setOperatingMode(const std::string &name, const varileg_msgs::OperatingMode &operatingModeRos);

  /**
   * Write HomingMethod by joint name.
   * @param name joint name
   * @param homingMode to write
   * @return true, if successful
   */
  bool writeHomingMethod(const std::string &name, const varileg_msgs::HomingGoal::_mode_type &homingMode);

  /**
   * Write interpolation time period to all slaves.
   * @param timePeriod in milliseconds
   * @return true, if successful
   */
  bool writeAllInterpolationTimePeriod(uint8_t timePeriod);

  /**
   * Write motor current limit to all slaves.
   * @param timePeriod in mA
   * @return true, if successful
   */
  bool writeAllMotorCurrentLimit(uint32_t motorCurrent);

  /**
   * Set mapping between joint name and unique NodeID.
   * @param jointName2NodeIdMap
   */
  void setJointName2NodeIdMap(const std::map<std::string, int> &jointName2NodeIdMap);
 private:
  // Mutex prohibiting simultaneous access to EtherCAT slave manager.
  mutable std::mutex mutex_;

  std::map<std::string, EposEthercatSlavePtr> eposEthercatSlaves_;
  std::map<std::string, int> jointName2NodeIdMap_;

  EposEthercatSlavePtr getEposEthercatSlave(const std::string &name) const;
  static void resizeExtendedJointStates(varileg_msgs::ExtendedJointStates &extendedJointStates, const unsigned long &size);
  static void resizeExtendedDeviceStates(varileg_msgs::ExtendedDeviceStates &extendedDeviceStates, const unsigned long &size);
};

using EposEthercatSlaveManagerPtr = std::shared_ptr<EposEthercatSlaveManager>;

}
#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVEMANAGER_HPP
