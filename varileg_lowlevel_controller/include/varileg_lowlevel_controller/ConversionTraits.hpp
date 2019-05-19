//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_CONVERSIONTRAITS_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_CONVERSIONTRAITS_HPP

#include <string>
#include <varileg_lowlevel_controller/entities/DeviceState.hpp>
#include <varileg_lowlevel_controller/entities/OperatingMode.hpp>
#include "varileg_msgs/DeviceState.h"
#include "varileg_msgs/OperatingMode.h"

#define DEFINE_CONVERT_FUNCTIONS_WITH_RETURN        \
inline static MsgRos convert(const Msg& msg) {      \
    MsgRos msgRos;                                  \
    convert(msg, msgRos);                           \
    return msgRos;                                  \
}                                                   \
inline static Msg convert(const MsgRos& msgRos) {   \
    Msg msg;                                        \
    convert(msgRos, msg);                           \
    return msg;                                     \
}

namespace varileg_lowlevel_controller {

/**
 * Converts between representations of ROS messages and local data objects.
 * @tparam Msg_
 * @tparam MsgRos_
 */
template<typename Msg_, typename MsgRos_>
class ConversionTraits;

template<>
class ConversionTraits<DeviceState, varileg_msgs::DeviceState> {
 public:
  using Msg = DeviceState;
  using MsgRos = varileg_msgs::DeviceState;

  inline static void convert(const Msg &msg, MsgRos &rosMsg) {
    switch (msg) {
      case Msg::STATE_UNKNOWN:
        rosMsg.state = MsgRos::STATE_UNKNOWN;
        break;
      case Msg::STATE_NOT_READY_TO_SWITCH_ON:
        rosMsg.state = MsgRos::STATE_NOT_READY_TO_SWITCH_ON;
        break;
      case Msg::STATE_SWITCH_ON_DISABLED:
        rosMsg.state = MsgRos::STATE_SWITCH_ON_DISABLED;
        break;
      case Msg::STATE_READY_TO_SWITCH_ON:
        rosMsg.state = MsgRos::STATE_READY_TO_SWITCH_ON;
        break;
      case Msg::STATE_SWITCHED_ON:
        rosMsg.state = MsgRos::STATE_SWITCHED_ON;
        break;
      case Msg::STATE_OP_ENABLED:
        rosMsg.state = MsgRos::STATE_OP_ENABLED;
        break;
      case Msg::STATE_QUICK_STOP_ACTIVE:
        rosMsg.state = MsgRos::STATE_QUICK_STOP_ACTIVE;
        break;
      case Msg::STATE_FAULT_REACTION_ACTIVE:
        rosMsg.state = MsgRos::STATE_FAULT_REACTION_ACTIVE;
        break;
      case Msg::STATE_FAULT:
        rosMsg.state = MsgRos::STATE_FAULT;
        break;
    }
  }

  inline static void convert(const MsgRos &rosMsg, Msg &msg) {
    switch (rosMsg.state) {
      case MsgRos::STATE_UNKNOWN:
        msg = Msg::STATE_UNKNOWN;
        break;
      case MsgRos::STATE_NOT_READY_TO_SWITCH_ON:
        msg = Msg::STATE_NOT_READY_TO_SWITCH_ON;
        break;
      case MsgRos::STATE_SWITCH_ON_DISABLED:
        msg = Msg::STATE_SWITCH_ON_DISABLED;
        break;
      case MsgRos::STATE_READY_TO_SWITCH_ON:
        msg = Msg::STATE_READY_TO_SWITCH_ON;
        break;
      case MsgRos::STATE_SWITCHED_ON:
        msg = Msg::STATE_SWITCHED_ON;
        break;
      case MsgRos::STATE_OP_ENABLED:
        msg = Msg::STATE_OP_ENABLED;
        break;
      case MsgRos::STATE_QUICK_STOP_ACTIVE:
        msg = Msg::STATE_QUICK_STOP_ACTIVE;
        break;
      case MsgRos::STATE_FAULT_REACTION_ACTIVE:
        msg = Msg::STATE_FAULT_REACTION_ACTIVE;
        break;
      case MsgRos::STATE_FAULT:
        msg = Msg::STATE_FAULT;
        break;
    }
  }

  DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

template<>
class ConversionTraits<OperatingMode, varileg_msgs::OperatingMode> {
 public:
  using Msg = OperatingMode;
  using MsgRos = varileg_msgs::OperatingMode;

  inline static void convert(const Msg &msg, MsgRos &rosMsg) {
    switch (msg) {
      case Msg::UNKNOWN:
        rosMsg.mode = MsgRos::MODE_UNKNOWN;
        break;
      case Msg::PPM:
        rosMsg.mode = MsgRos::MODE_PPM;
        break;
      case Msg::PVM:
        rosMsg.mode = MsgRos::MODE_PVM;
        break;
      case Msg::HMM:
        rosMsg.mode = MsgRos::MODE_HMM;
        break;
      case Msg::CSP:
        rosMsg.mode = MsgRos::MODE_CSP;
        break;
      case Msg::CSV:
        rosMsg.mode = MsgRos::MODE_CSV;
        break;
      case Msg::CST:
        rosMsg.mode = MsgRos::MODE_CST;
        break;
    }
  }

  inline static void convert(const MsgRos &rosMsg, Msg &msg) {
    switch (rosMsg.mode) {
      case MsgRos::MODE_UNKNOWN:
        msg = Msg::UNKNOWN;
        break;
      case MsgRos::MODE_PPM:
        msg = Msg::PPM;
        break;
      case MsgRos::MODE_PVM:
        msg = Msg::PVM;
        break;
      case MsgRos::MODE_HMM:
        msg = Msg::HMM;
        break;
      case MsgRos::MODE_CSP:
        msg = Msg::CSP;
        break;
      case MsgRos::MODE_CSV:
        msg = Msg::CSV;
        break;
      case MsgRos::MODE_CST:
        msg = Msg::CST;
        break;
    }
  }

  DEFINE_CONVERT_FUNCTIONS_WITH_RETURN
};

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_CONVERSIONTRAITS_HPP
