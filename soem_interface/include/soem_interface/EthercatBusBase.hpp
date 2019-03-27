#pragma once

#include <cassert>
// std
#include <atomic>
#include <string>
#include <vector>

// soem
#include <soem/soem/ethercat.h>

// any measurements
#include <any_measurements/Time.hpp>

#include <message_logger/message_logger.hpp>

// soem_interface
#include <soem_interface/EthercatSlaveBase.hpp>
#include <soem_interface/common/ThreadSleep.hpp>

namespace soem_interface {

class EthercatBusBase {
 public:
  EthercatBusBase() = delete;
  /*!
   * Constructor.
   * @param name Name of the bus, e.g. "eth0".
   */
  explicit EthercatBusBase(const std::string& name);

  /*!
   * Destructor.
   */
  ~EthercatBusBase() = default;

  /*!
   * Get the name of the bus.
   * @return Name of the bus.
   */
  const std::string& getName() const { return name_; }

  /*!
   * Get the time of the last successful PDO reading.
   * @return Stamp.
   */
  const any_measurements::Time& getUpdateReadStamp() const { return updateReadStamp_; }

  /*!
   * Get the time of the last successful PDO writing.
   * @return Stamp.
   */
  const any_measurements::Time& getUpdateWriteStamp() const { return updateWriteStamp_; }

  /**
   * Was startup successfully called?
   * @return
   */
  const bool isStartedUp() const { return isStartedUp_; }

  /*!
   * Check if a bus is available.
   * @param name Name of the bus.
   * @return True if available.
   */
  static bool busIsAvailable(const std::string& name);

  /*!
   * Print all available busses.
   */
  static void printAvailableBusses();

  /*!
   * Check if this bus is available.
   * @return True if available.
   */
  bool busIsAvailable() const;

  /*!
   * Get the number of slaves which were detected on this bus.
   * @return Number of slaves.
   */
  int getNumberOfSlaves() const;

  /*!
   * Startup the bus communication.
   * @return True if successful.
   */
  bool startup(const std::vector<EthercatSlaveBasePtr> &slaves);

  /*!
   * Update step 1: Read all PDOs.
   */
  void receiveBuffer();

  /*!
   * Update step 2: Write all PDOs.
   */
  void sendBuffer();

  /*!
   * Shutdown the bus communication.
   */
  void shutdown(const std::vector<EthercatSlaveBasePtr> &slaves);

  /*!
   * Set the desired EtherCAT state machine state.
   * @param state Desired state.
   * @param slave Address of the slave, 0 for all slaves.
   */
  void setState(const uint16_t state, const uint16_t slave = 0);

  /*!
   * Wait for an EtherCAT state machine state to be reached.
   * @param state      Desired state.
   * @param slave      Address of the slave, 0 for all slaves.
   * @param maxRetries Maximum number of retries.
   * @param retrySleep Duration to sleep between the retries.
   * @return True if the state has been reached within the timeout.
   */
  bool waitForState(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 40, const double retrySleep = 0.001);

  /*!
   * Send a writing SDO.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value          Value to write.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    const int size = sizeof(Value);
    Value valueCopy = value;  // copy value to make it modifiable
    int wkc = 0;
    {
      std::lock_guard<std::recursive_mutex> guard(contextMutex_);
      wkc = ecx_SDOwrite(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), size, &valueCopy, EC_TIMEOUTRXM);
    }
    if (wkc <= 0) {
      MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for writing SDO (ID: 0x" << std::setfill('0')
                                 << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                                 << static_cast<uint16_t>(subindex) << ").");
      return false;
    }
    return true;
  }

  /*!
   * Send a reading SDO.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value          Return argument, will contain the value which was read.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    int size = sizeof(Value);
    int wkc = 0;
    {
      std::lock_guard<std::recursive_mutex> guard(contextMutex_);
      wkc = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), &size, &value, EC_TIMEOUTRXM);
    }
    if (wkc <= 0) {
      MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for reading SDO (ID: 0x" << std::setfill('0')
                                 << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                                 << static_cast<uint16_t>(subindex) << ").");
      return false;
    }
    if (size != sizeof(Value)) {
      MELO_ERROR_STREAM("Slave " << slave << ": Size mismatch (expected " << sizeof(Value) << " bytes, read " << size
                                 << " bytes) for reading SDO (ID: 0x" << std::setfill('0') << std::setw(4) << std::hex << index
                                 << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(subindex) << ").");
      return false;
    }
    return true;
  }

  /*!
   * Get the PDO expected working counter.
   * @param slave Address of the slave, 0 for all slaves.
   * @return Expected working counter.
   */
  int getExpectedWorkingCounter(const uint16_t slave = 0) const;

  /*!
   * Check if the current working counter for all slaves is high enough.
   * @return True if the working counter is equal or higher than expected.
   */
  bool workingCounterIsOk() const;

  /*!
   * Read a TxPDO from the buffer.
   * @param slave Address of the slave.
   * @param txPdo Return argument, TxPDO container.
   */
  template <typename TxPdo>
  void readTxPdo(const uint16_t slave, TxPdo& txPdo) const {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    assert(sizeof(TxPdo) == ecatContext_.slavelist[slave].Ibytes);
    memcpy(&txPdo, ecatContext_.slavelist[slave].inputs, sizeof(TxPdo));
  }

  /*!
   * Write an RxPDO to the buffer.
   * @param slave Address of the slave.
   * @param txPdo RxPDO container.
   */
  template <typename RxPdo>
  void writeRxPdo(const uint16_t slave, const RxPdo& rxPdo) {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    assert(sizeof(RxPdo) == ecatContext_.slavelist[slave].Obytes);
    memcpy(ecatContext_.slavelist[slave].outputs, &rxPdo, sizeof(RxPdo));
  }

 protected:
  //! Name of the bus.
  std::string name_;

  //! Bool indicating if bus startup was called
  bool isStartedUp_{false};

  //! Bool indicating whether PDO data has been sent and not read yet.
  bool sentProcessData_{false};

  //! Working counter of the most recent PDO.
  std::atomic<int> wkc_;

  //! Time of the last successful PDO reading.
  any_measurements::Time updateReadStamp_;
  //! Time of the last successful PDO writing.
  any_measurements::Time updateWriteStamp_;

  //! Maximal number of retries to configure the EtherCAT bus.
  const unsigned int ecatConfigMaxRetries_{5};
  //! Time to sleep between the retries.
  const double ecatConfigRetrySleep_{1.0};

  // EtherCAT input/output mapping of the slaves within the datagrams.
  char ioMap_[4096];

  // EtherCAT context data elements:

  // Port reference.
  ecx_portt ecatPort_;
  // List of slave data. Index 0 is reserved for the master, higher indices for the slaves.
  ec_slavet ecatSlavelist_[EC_MAXSLAVE];
  // Number of slaves found in the network.
  int ecatSlavecount_{0};
  // Slave group structure.
  ec_groupt ecatGrouplist_[EC_MAXGROUP];
  // Internal, reference to EEPROM cache buffer.
  uint8 ecatEsiBuf_[EC_MAXEEPBUF];
  // Internal, reference to EEPROM cache map.
  uint32 ecatEsiMap_[EC_MAXEEPBITMAP];
  // Internal, reference to error list.
  ec_eringt ecatEList_;
  // Internal, reference to processdata stack buffer info.
  ec_idxstackT ecatIdxStack_;
  // Boolean indicating if an error is available in error stack.
  boolean ecatError_{FALSE};
  // Reference to last DC time from slaves.
  int64 ecatDcTime_{0};
  // Internal, SM buffer.
  ec_SMcommtypet ecatSmCommtype_[EC_MAX_MAPT];
  // Internal, PDO assign list.
  ec_PDOassignt ecatPdoAssign_[EC_MAX_MAPT];
  // Internal, PDO description list.
  ec_PDOdesct ecatPdoDesc_[EC_MAX_MAPT];
  // Internal, SM list from EEPROM.
  ec_eepromSMt ecatSm_;
  // Internal, FMMU list from EEPROM.
  ec_eepromFMMUt ecatFmmu_;

  mutable std::recursive_mutex contextMutex_;
  // EtherCAT context data.
  // Note: SOEM does not use dynamic memory allocation (new/delete). Therefore
  // all context pointers must be null or point to an existing member.
  ecx_contextt ecatContext_ = {&ecatPort_,
                               &ecatSlavelist_[0],
                               &ecatSlavecount_,
                               EC_MAXSLAVE,
                               &ecatGrouplist_[0],
                               EC_MAXGROUP,
                               &ecatEsiBuf_[0],
                               &ecatEsiMap_[0],
                               0,
                               &ecatEList_,
                               &ecatIdxStack_,
                               &ecatError_,
                               0,
                               0,
                               &ecatDcTime_,
                               &ecatSmCommtype_[0],
                               &ecatPdoAssign_[0],
                               &ecatPdoDesc_[0],
                               &ecatSm_,
                               &ecatFmmu_,
                               nullptr};
};

using EthercatBusBasePtr = std::shared_ptr<EthercatBusBase>;

}  // namespace soem_interface
