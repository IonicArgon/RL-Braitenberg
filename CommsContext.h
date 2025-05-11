#pragma once
#include "Globals.h"
#include "nRF24L01P.h"

using nrf_address = unsigned long long;

/**
 * @brief Main context class for communication using
 * nRF24L01P RF transceiver. Sets up mailboxes for incoming
 * and outgoing messages.
 * @note `addr_tx` and `addr_rx` must be different from each other.
 * @note `addr_tx` and `addr_rx` must be inverse pairs between the two vehicles.
 */
class CommsContext {
 public:
  /**
   * @brief Constructor for communication context class.
   * @param nrf_mosi SPI MOSI pin for the transceiver.
   * @param nrf_miso SPI MISO pin for the transceiver.
   * @param nrf_sck SPI SCK (clock) pin for the transceiver.
   * @param nrf_ncs SPI NCS (chip select) pin for the transceiver.
   * @param nrf_ce SPI CE (chip enable) pin for the transceiver.
   * @param addr_tx Hexidecimal representation of transmission address from
   * `0x0000000000` - `0xffffffffff`.
   * @param addr_rx Hexidecimal representation of receiving address from
   * `0x0000000000` - `0xffffffffff`.
   */
  CommsContext(PinName nrf_mosi, PinName nrf_miso, PinName nrf_sck,
               PinName nrf_ncs, PinName nrf_ce, nrf_address addr_tx,
               nrf_address addr_rx);

  /**
   * @brief Is called every communication "tick".
   */
  void run_comms_cycle(void);

  /**
   * @brief Attempts to queue a `CommsMsg` in outbound mail for transmission.
   * @param msg A `CommsMsg` to queue.
   * @returns `true` if the message was queued, otherwise `false`.
   */
  bool try_queue_send(const CommsMsg msg);

  /**
   * @brief Attempts to read from incoming mail and write to a `CommsMsg`
   * pointer.
   * @param out A pointer to a `CommsMsg`.
   * @returns `true` if a message was read, otherwise `false`.
   */
  bool try_read(CommsMsg *out);

 private:
  Mail<CommsMsg, MAIL_SIZE> mail_incoming;
  Mail<CommsMsg, MAIL_SIZE> mail_outgoing;
  nRF24L01P nrf;
};
