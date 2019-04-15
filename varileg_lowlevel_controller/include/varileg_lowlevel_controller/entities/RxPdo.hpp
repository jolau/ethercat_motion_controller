//
// Created by jolau on 12.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_RXPDO_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_RXPDO_HPP

// std
#include <cstdint>

namespace varileg_lowlevel_controller {

struct RxPdo {
  uint16_t	controlWord;		// 0x6040
  int8_t    operatingMode;      // 0x6060
  int32_t	targetPosition;		// 0x607A
} __attribute__((packed));

}


#endif //VARILEG_LOWLEVEL_CONTROLLER_RXPDO_HPP