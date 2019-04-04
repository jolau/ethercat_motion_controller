//
// Created by jolau on 04.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_HOMINGMETHOD_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_HOMINGMETHOD_HPP

#include <string>

namespace varileg_lowlevel_controller {
// TODO: add all homing methods
enum class HomingMethod {
  INDEX_NEGATIVE_SPEED,
  INDEX_POSITIVE_SPEED
};

namespace Enum {
inline const std::string toString(const HomingMethod &homingMethod) {
  switch (homingMethod) {
    case HomingMethod::INDEX_NEGATIVE_SPEED:
      return "INDEX_NEGATIVE_SPEED";
    case HomingMethod::INDEX_POSITIVE_SPEED:
      return "INDEX_POSITIVE_SPEED";
  }
}
}
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_HOMINGMETHOD_HPP
