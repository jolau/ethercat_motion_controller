//
// Created by jolau on 12.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_RXPDO_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_RXPDO_HPP

// std
#include <cstdint>

namespace varileg_lowlevel_controller {

struct RxPdo {
  uint16_t	ControlWord;		//0x6040
  int32_t	TargetPosition;		//0x607A
  int32_t	PositionOffset;		//0x60B0
  int32_t	VelocityOffset;		//0x60B1
  int16_t	TorqueOffset;		//0x60B2
  uint8_t	ModeOfOperation;	//0x6060
  uint32_t	PhysicalOutput;		//0x60FE
  uint16_t	TouchProbeFunction;	//0x60B8
} __attribute__((packed));

}


#endif //VARILEG_LOWLEVEL_CONTROLLER_RXPDO_HPP