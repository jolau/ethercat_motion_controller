//
// Created by jolau on 29.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_DEVICESTATE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_DEVICESTATE_HPP

namespace varileg_lowlevel_controller {

enum class DeviceState {
  STATE_UNKNOWN = 0u,
  STATE_NOT_READY_TO_SWITCH_ON = 1u,
  STATE_SWITCH_ON_DISABLED = 2u,
  STATE_READY_TO_SWITCH_ON = 3u,
  STATE_SWITCHED_ON = 4u,
  STATE_OP_ENABLED = 5u,
  STATE_QUICK_STOP_ACTIVE = 6u,
  STATE_FAULT_REACTION_ACTIVE = 7u,
  STATE_FAULT = 8u,
};

namespace Enum {
inline const std::string toString(const DeviceState &motorControllerState) {
  switch (motorControllerState) {
    case DeviceState::STATE_UNKNOWN:
      return "STATE_UNKNOWN";
    case DeviceState::STATE_NOT_READY_TO_SWITCH_ON:
      return "STATE_NOT_READY_TO_SWITCH_ON";
    case DeviceState::STATE_SWITCH_ON_DISABLED:
      return "STATE_SWITCH_ON_DISABLED";
    case DeviceState::STATE_READY_TO_SWITCH_ON:
      return "STATE_READY_TO_SWITCH_ON";
    case DeviceState::STATE_SWITCHED_ON:
      return "STATE_SWITCHED_ON";
    case DeviceState::STATE_OP_ENABLED:
      return "STATE_OP_ENABLED";
    case DeviceState::STATE_QUICK_STOP_ACTIVE:
      return "STATE_QUICK_STOP_ACTIVE";
    case DeviceState::STATE_FAULT_REACTION_ACTIVE:
      return "STATE_FAULT_REACTION_ACTIVE";
    case DeviceState::STATE_FAULT:
      return "STATE_FAULT";
  }
}
}

}

#endif //VARILEG_LOWLEVEL_CONTROLLER_DEVICESTATE_HPP
