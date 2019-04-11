//
// Created by jolau on 12.03.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_TXPDO_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_TXPDO_HPP

// std
#include <cstdint>

namespace varileg_lowlevel_controller {

struct TxPdo {
  uint16_t	statusWord;					//0x6041
  int32_t	positionActualValue;		//0x6064
  int32_t   positionPrimaryEncoder;     //0x60E4 0x02
  int32_t   positionSecondaryEncoder;   //0x6040 0x01
  int32_t	velocityActualValue;		//0x606C
  int16_t	torqueActualValue;			//0x6077
} __attribute__((packed)) ;

}
#endif //VARILEG_LOWLEVEL_CONTROLLER_TXPDO_HPP
