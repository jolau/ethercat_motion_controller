//
// Created by jolau on 05.03.19.
//

#include <varileg_lowlevel_controller/EposEthercatSlave.hpp>
#include <varileg_lowlevel_controller/EposCommandLibrary.hpp>

#include "varileg_lowlevel_controller/EposEthercatSlave.hpp"

#define _BV(bit) 				(1 << (bit))
#define bit_is_set(val, bit) 	(val & _BV(bit))
#define bit_is_clear(val, bit) 	(!(val & _BV(bit)))
#define sbi(val,bit) 			((val) |= _BV(bit))
#define cbi(val,bit) 			((val) &= ~_BV(bit))

#define STATUSWORD_READY_TO_SWITCH_ON_BIT 		0
#define STATUSWORD_SWITCHED_ON_BIT 				1
#define STATUSWORD_OPERATION_ENABLE_BIT 		2
#define STATUSWORD_FAULT_BIT 					3
#define STATUSWORD_VOLTAGE_ENABLE_BIT 			4
#define STATUSWORD_QUICK_STOP_BIT 				5
#define STATUSWORD_SWITCH_ON_DISABLE_BIT 		6
#define STATUSWORD_NO_USED_WARNING_BIT 			7
#define STATUSWORD_ELMO_NOT_USED_BIT 			8
#define STATUSWORD_REMOTE_BIT 					9
#define STATUSWORD_TARGET_REACHED_BIT 			10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT	11


namespace varileg_lowlevel_controller {

EposEthercatSlave::EposEthercatSlave(const std::string &name,
                                     const soem_interface::EthercatBusBasePtr &bus,
                                     const uint32_t address) : soem_interface::EthercatSlaveBase(bus, address), name_(name) {
  pdoInfo_.rxPdoId_ = 0x1600;
  pdoInfo_.txPdoId_ = 0x1200;
  pdoInfo_.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo_.txPdoSize_ = sizeof(TxPdo);
}

std::string EposEthercatSlave::getName() const {
  return name_;
}

soem_interface::EthercatSlaveBase::PdoInfo EposEthercatSlave::getCurrentPdoInfo() const {
  return pdoInfo_;
}

bool EposEthercatSlave::startup() {
  bus_->setState(EC_STATE_PRE_OP);
  if(!bus_->waitForState(EC_STATE_PRE_OP)) {
    MELO_ERROR_STREAM(name_ << ": not entered PRE OP");
    return false;
  }

  // set mode to CSP
  if(!bus_->sendSdoWrite(address_, EposCommandLibrary::SDOs::MODES_OF_OPERATION.index, EposCommandLibrary::SDOs::MODES_OF_OPERATION.subindex, true, 0X08)) {
    MELO_ERROR_STREAM(name_ << ": Could not set CSP mode.")
    return false;
  }

  // get mode
  int8_t mode = 0;
  bus_->sendSdoRead(address_, EposCommandLibrary::SDOs::MODES_OF_OPERATION_DISPLAY.index, EposCommandLibrary::SDOs::MODES_OF_OPERATION_DISPLAY.subindex, false, mode);
  if(mode != 0x08) {
    MELO_ERROR_STREAM(name_ << ": CSP mode was not set. Has mode: " << mode)
    return false;
  }

  return true;
}

void EposEthercatSlave::updateRead() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  bus_->readTxPdo(address_, tx_pdo_);
}

int ServoOn_GetCtrlWrd(uint16_t StatusWord, uint16_t *ControlWord)
{
  int  _enable=0;
  if (bit_is_clear(StatusWord, STATUSWORD_OPERATION_ENABLE_BIT)) //Not ENABLED yet
  {
    if (bit_is_clear(StatusWord, STATUSWORD_SWITCHED_ON_BIT)) //Not SWITCHED ON yet
    {
      if (bit_is_clear(StatusWord, STATUSWORD_READY_TO_SWITCH_ON_BIT)) //Not READY to SWITCH ON yet
      {
        if (bit_is_set(StatusWord, STATUSWORD_FAULT_BIT)) //FAULT exist
        {
          MELO_INFO_STREAM("COMMAND: fault reset")
          (*ControlWord)=0x80;	//FAULT RESET command
        }
        else //NO FAULT
        {
          MELO_INFO_STREAM("COMMAND: shutdown")
          (*ControlWord)=0x06;	//SHUTDOWN command (transition#2)
        }
      }
      else //READY to SWITCH ON
      {
        MELO_INFO_STREAM("COMMAND: switch on")
        (*ControlWord)=0x07;	//SWITCH ON command (transition#3)
      }
    }
    else //has been SWITCHED ON
    {
      MELO_INFO_STREAM("COMMAND: enable operation")
      (*ControlWord)=0x0F;	//ENABLE OPETATION command (transition#4)
      _enable=1;
    }
  }
  else //has been ENABLED
  {
    MELO_INFO_STREAM("COMMAND: maintain operation state")
    (*ControlWord)=0x0F;	//maintain OPETATION state
    _enable=1;
  }
  return _enable;
}

void EposEthercatSlave::updateWrite() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  uint16_t controlword = 0;

  RxPdo rxPdo;

  std::string binary = std::bitset<16>(tx_pdo_.StatusWord).to_string();
  MELO_INFO_STREAM("Status Word: " << binary)

  if(!ready_) {
    MELO_INFO("Enabling Drive");
    ready_ = ServoOn_GetCtrlWrd(tx_pdo_.StatusWord, &controlword);
    rxPdo.ControlWord = controlword;
  } else {
    MELO_INFO_STREAM("Actual Position: " << tx_pdo_.PositionActualValue);
    if (tx_pdo_.PositionActualValue < 5000) {
      rxPdo.TargetPosition = 5100;
    } else {
      rxPdo.TargetPosition = 0;
    }
    MELO_INFO_STREAM("Send Target Position: " << rxPdo.TargetPosition);
  }

  binary = std::bitset<16>(rxPdo.ControlWord).to_string();
  MELO_INFO_STREAM("Control Word: " << binary)

  bus_->writeRxPdo(address_, rxPdo);
}

uint8_t EposEthercatSlave::readNodeId() {
  uint8_t nodeId = 0;
  if(!bus_->sendSdoRead(address_, EposCommandLibrary::SDOs::NODE_ID.index, EposCommandLibrary::SDOs::NODE_ID.subindex, false, nodeId)) {
    MELO_ERROR_STREAM("Could not read NodeID of " << name_);
  }
  return nodeId;
}

void EposEthercatSlave::shutdown() {
  RxPdo rxPdo;
  rxPdo.ControlWord = 0;
  bus_->writeRxPdo(address_,rxPdo);
}

}