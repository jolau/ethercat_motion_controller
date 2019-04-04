//
// Created by jolau on 03.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_OPERATINGMODE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_OPERATINGMODE_HPP
namespace varileg_lowlevel_controller {
enum class OperatingMode {
  UNKNOWN = 0,
  PPM,
  PVM,
  HMM,
  CSP,
  CSV,
  CST
};

namespace Enum {
inline const std::string toString(const OperatingMode &operatingMode) {
  switch (operatingMode) {
    case OperatingMode::UNKNOWN:      return "UNKNOWN";
    case OperatingMode::PPM:          return "PPM";
    case OperatingMode::PVM:          return "PVM";
    case OperatingMode::HMM:          return "HMM";
    case OperatingMode::CSP:          return "CSP";
    case OperatingMode::CSV:          return "CSV";
    case OperatingMode::CST:          return "CST";
  }
}
}
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_OPERATINGMODE_HPP
