//
// Created by jolau on 05.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP

// soem_interface
#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>
#include <varileg_lowlevel_controller/entities/JointState.hpp>
#include <varileg_lowlevel_controller/entities/JointTrajectory.hpp>
#include <varileg_lowlevel_controller/entities/EncoderCrosschecker.hpp>
#include <varileg_lowlevel_controller/entities/JointSpecifications.hpp>
#include "varileg_lowlevel_controller/entities/RxPdo.hpp"
#include "varileg_lowlevel_controller/entities/TxPdo.hpp"
#include "EposCommandLibrary.hpp"
#include "varileg_lowlevel_controller/entities/DeviceState.hpp"
#include "varileg_lowlevel_controller/entities/HomingMethod.hpp"
#include "varileg_lowlevel_controller/entities/EposStartupConfig.hpp"
#include <boost/algorithm/clamp.hpp>

namespace varileg_lowlevel_controller {
/**
 * Represantion and interface to a EPOS motor controller (slave)
 */
class EposEthercatSlave : public soem_interface::EthercatSlaveBase {
 public:
  EposEthercatSlave() = delete;
  EposEthercatSlave(const std::string &name,
                    const soem_interface::EthercatBusBasePtr &bus,
                    const uint32_t address,
                    EposStartupConfig eposStartupConfig);
  ~EposEthercatSlave() override = default;

  std::string getName() const override;

  PdoInfo getCurrentPdoInfo() const override;

  /**
  * Was startup successfully called?
  * @return true if slave and bus are started up.
  */
  const bool isStartedUp() const { return isStartedUp_ && bus_->isStartedUp(); }

  /**
   * Set jointSpecifactions_ and set sendJointrajectory_.position to homePosition to prevent a jump to previous home position.
   * @param jointSpecifications
   */
  void setJointSpecifications(JointSpecifications &jointSpecifications);

  /**
   * Set JointTrajectory which will be sent by next cycle if in CSP mode.
   * @param sendJointTrajectory to be sent by next cycle.
   */
  void setSendJointTrajectory(const JointTrajectory &sendJointTrajectory);

  /**
   * Set target HomingState which will be sent by next cycle if in HMM mode.
   * @param sendHomingState to be sent by next cycle.
   */
  void setSendHomingState(HomingState sendHomingState);

  /**
   * Set target DeviceState which will be sent by next cycle.
   * @param sendDeviceState to be sent by next cycle.
   * @return true if target sendDeviceState is reachable.
   */
  bool setSendDeviceState(DeviceState sendDeviceState);

  /**
   * Set OperatingMode which will be sent by next cycle.
   * @param sendOperatingMode to be sent by next cycle.
   */
  void setSendOperatingMode(OperatingMode sendOperatingMode);

  /**
   * Get current JointState as received at last cycle. Only updated if in CSP mode.
   * @return current JointState
   */
  const JointState getReceiveJointState() const;

  /**
   * Get current HomingState as received at last cycle. Only updated if in HMM mode.
   * @return current HomingState
   */
  const HomingState getReceiveHomingState() const;

  /**
  * Get current DeviceState as received at last cycle.
  * @return current DeviceState
  */
  const DeviceState getReceiveDeviceState() const;

  /**
  * Get current OperatingMode as received at last cycle.
  * @return current OperatingMode
  */
  const OperatingMode getReceiveOperatingMode() const;

  bool startup() override;
  void readInbox();
  void writeOutbox();
  void shutdown() override;

  /**
   * Read current NodeID EPOS with SDO.
   * @return current NodeID of EPOS
   */
  uint8_t readNodeId();

  /**
   * Writes the interpolation time period for interpolating between position points in CSP mode. Has to be matched with the update loop.
   * @param timePeriod in milliseconds
   * @return true, if successful
   */
  bool writeInterpolationTimePeriod(uint8_t timePeriod);

  /**
   * Write the motor current limit.
   * @param motorCurrent in mA
   * @return true, if successful
   */
  bool writeMotorCurrentLimit(uint32_t motorCurrent);

  /**
   * Write the homing method to the EPOS.
   * @param homingMethod
   * @return true, if successful
   */
  bool writeHomingMethod(const HomingMethod &homingMethod);
 private:
  template<typename Value>
  bool writeSDO(const SDO &sdo, const Value value, const bool completeAccess);

  template<typename Value>
  bool readSDO(const SDO &sdo, Value &value, const bool completeAccess);

  /**
   * Generates the appropriate controlword to change a device state. All paths as specified in the EPOS firmware specifications are supported.
   * @param controlword the controlword to be changed. Only the necessary bits are changed, all others are untouched.
   * @param currentState current DeviceState of the EPOS
   * @param targetState targeted DeviceState to be reached
   * @return true, if a targeted DeviceState is reachable from the current DeviceState (namely a path is existent)
   */
  static bool applyNextDeviceStateTransition(uint16_t &controlword,
                                             const DeviceState &currentState,
                                             const DeviceState &targetState);

  /**
   * Generates the appropriate controlword to change the homing state.
   * @param controlword the controlword to be changed. Only the necessary bits are changed, all others are untouched.
   * @param targetState targeted HomingState to reached
   * @return true, if a targeted HomingState is reachable
   */
  static bool applyNextHomingStateTransition(uint16_t &controlword,
                                             const HomingState &targetState);

  const std::string name_;
  PdoInfo pdoInfo_;
  EposStartupConfig eposStartupConfig_;

  JointSpecifications jointSpecifications_;
  bool isJointSpecificationsSet_ = false;

  OperatingMode sendOperatingMode_ = OperatingMode::CSP;
  OperatingMode receiveOperatingMode_ = OperatingMode::UNKNOWN;

  JointState receiveJointState_{0, 0, 0, 0, 0};
  JointTrajectory sendJointTrajectory_{0, 0, 0};

  HomingState sendHomingState_ = HomingState::UNKNOWN;
  HomingState receiveHomingState_ = HomingState::UNKNOWN;

  DeviceState sendDeviceState_ = DeviceState::STATE_SWITCH_ON_DISABLED;
  DeviceState receiveDeviceState_ = DeviceState::STATE_UNKNOWN;

  bool isStartedUp_{false}; ///< Bool indicating if slave and bus startup was called
};

using EposEthercatSlavePtr = std::shared_ptr<EposEthercatSlave>;

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_EPOSETHERCATSLAVE_HPP
