//
// Created by jolau on 03.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_OPERATINGMODE_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_OPERATINGMODE_HPP

#include <string>

namespace varileg_lowlevel_controller {
/**
 * Operating Modes of EPOS
 */
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
/**
 * Convert name of OperatingMode to String
 * @param operatingMode to be converted
 */
inline const std::string toString(const OperatingMode &operatingMode) {
  switch (operatingMode) {
    case OperatingMode::UNKNOWN:
      return "UNKNOWN";
    case OperatingMode::PPM:
      return "PPM (Profile Position Mode)";
    case OperatingMode::PVM:
      return "PVM (Profile Velocity Mode)";
    case OperatingMode::HMM:
      return "HMM (Homing Mode)";
    case OperatingMode::CSP:
      return "CSP (Cyclic Synchronous Position Mode)";
    case OperatingMode::CSV:
      return "CSV (Cyclic Synchronous Velocity Mode)";
    case OperatingMode::CST:
      return "CST (Cyclic Synchronous Torque Mode)";
    default:
      return "NO STRING CONVERSION FOR THIS OPERATING MODE";
  }
}
}
}

#endif //VARILEG_LOWLEVEL_CONTROLLER_OPERATINGMODE_HPP
