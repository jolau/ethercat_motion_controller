//
// Created by jolau on 04.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_HOMINGSTATE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_HOMINGSTATE_HPP
#include <string>

namespace varileg_lowlevel_controller {

/**
 * State of EPOS while homing.
 */
enum class HomingState {
  UNKNOWN = 0,
  HOMING_IN_PROGRESS,
  HOMING_INTERRUPTED,
  HOMING_SUCCESSFUL,
  HOMING_ERROR
};

namespace Enum {
/**
 * Convert name of HomingState to String
 * @param homingState to be converted
 */
inline const std::string toString(const HomingState &homingState) {
  switch (homingState) {
    case HomingState::UNKNOWN:
      return "UNKNOWN";
    case HomingState::HOMING_IN_PROGRESS:
      return "HOMING_IN_PROGRESS";
    case HomingState::HOMING_INTERRUPTED:
      return "HOMING_INTERRUPTED";
    case HomingState::HOMING_SUCCESSFUL:
      return "HOMING_SUCCESSFUL";
    case HomingState::HOMING_ERROR:
      return "HOMING_ERROR";
    default:
      return "NO STRING CONVERSION FOR THIS HOMING STATE";
  }
}
}
}
#endif //VARILEG_LOWLEVEL_CONTROLLER_HOMINGSTATE_HPP
