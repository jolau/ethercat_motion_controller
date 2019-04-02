//
// Created by jolau on 12.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_TXPDO_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_TXPDO_HPP

// std
#include <cstdint>

namespace varileg_lowlevel_controller {

struct TxPdo {
  uint16_t	StatusWord;					//0x6041
  int32_t	PositionActualValue;		//0x6064
  int32_t   PositionSecondEncoder;
  /*int32_t	VelocityActualValue;		//0x606C
  int16_t	TorqueActualValue;			//0x6077
  uint8_t	ModeOfOperationDisplay;		//0x6061
  uint32_t	DigitalInput;				//0x60FD
  uint16_t	TouchProbeStatus;			//0x60B9
  int32_t	TouchProbePosition1;		//0x60BA
  int32_t	TouchProbePosition2;		//0x60BB*/
} __attribute__((packed)) ;

}
#endif //VARILEG_LOWLEVEL_CONTROLLER_TXPDO_HPP
