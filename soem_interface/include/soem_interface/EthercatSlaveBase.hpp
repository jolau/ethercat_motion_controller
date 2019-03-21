#pragma once

// std
#include <cstdint>
#include <memory>
#include <mutex>

namespace soem_interface {

class EthercatBusBase;

class EthercatSlaveBase {
 public:
  using EthercatBusBasePtr = std::shared_ptr<EthercatBusBase>;

  struct PdoInfo {
    uint16_t rxPdoId_ = 0;
    uint16_t txPdoId_ = 0;
    uint16_t rxPdoSize_ = 0;
    uint16_t txPdoSize_ = 0;
    uint32_t moduleId_ = 0;
  };

  EthercatSlaveBase(const EthercatBusBasePtr& bus, const uint32_t address);
  virtual ~EthercatSlaveBase() = default;

  virtual std::string getName() const = 0;

  virtual bool startup() = 0;
  virtual void updateRead() = 0;
  virtual void updateWrite() = 0;
  virtual void shutdown() = 0;

  virtual PdoInfo getCurrentPdoInfo() const = 0;

  uint32_t getAddress() const { return address_; }

  /*!
   * Send a writing SDO.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-inidices at once.
   * @param value          Value to write.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value);

  /*!
   * Send a reading SDO.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-inidices at once.
   * @param value          Return argument, will contain the value which was read.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

  // Send SDOs.
  virtual bool sendSdoReadInt8(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadInt16(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadInt32(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadInt64(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadUInt8(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadUInt16(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadUInt32(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadUInt64(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadFloat(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadDouble(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value) {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoWriteInt8(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteInt16(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteInt32(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteInt64(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteUInt8(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteUInt16(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteUInt32(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteUInt64(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteFloat(const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteDouble(const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value) {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString, const std::string& valueTypeString,
                                  std::string& valueString);
  virtual bool sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString, const std::string& valueTypeString,
                                   const std::string& valueString);

 protected:
  // Mutex prohibiting simultaneous access to EtherCAT slave.
  mutable std::recursive_mutex mutex_;  
  EthercatBusBasePtr bus_;
  const uint32_t address_{0};

  void printWarnNotImplemented();
};

using EthercatSlaveBasePtr = std::shared_ptr<EthercatSlaveBase>;

}  // namespace soem_interface
