#include "CommsContext.h"

CommsContext::CommsContext(PinName nrf_mosi, PinName nrf_miso, PinName nrf_sck,
                           PinName nrf_ncs, PinName nrf_ce, nrf_address addr_tx,
                           nrf_address addr_rx)
    : nrf(nrf_mosi, nrf_miso, nrf_sck, nrf_ncs, nrf_ce) {
  // When the context is created, set up and enable the transceiver.
  nrf.powerUp();
  nrf.setTxAddress(addr_tx);
  nrf.setRxAddress(addr_rx);
  nrf.setTransferSize(MSG_SIZE);
  nrf.setReceiveMode();
  nrf.disableAutoAcknowledge();
  nrf.enable();
}

void CommsContext::run_comms_cycle(void) {
  // Read from the transceiver if available.
  if (nrf.readable()) {
    char buffer[MSG_SIZE];
    nrf.read(NRF24L01P_PIPE_P0, buffer, MSG_SIZE);
    CommsMsg *msg = mail_incoming.try_alloc();

    // If the buffer is full (a nullptr was returned), just discard the message.
    if (msg == nullptr) {
      return;
    }

    // Otherwise, copy it over to the memory block and push to incoming mail
    // queue for consumption later.
    memcpy(msg, buffer, MSG_SIZE);
    mail_incoming.put(msg);
  } else {
    // Otherwise, attempt to transmit any message requests.
    CommsMsg *msg = mail_outgoing.try_get();
    if (msg == nullptr) {
      return;
    }

    int bytes_written = nrf.write(NRF24L01P_PIPE_P0, (char *)msg, MSG_SIZE);
#ifdef PRINT_DEBUG
    printf("Bytes written: %d\r\n", bytes_written);
#endif

    // If we fail to send the message, queue it for a re-attempt, otherwise
    // remove the transmission request from queue.
    if (bytes_written < MSG_SIZE) {
      mail_outgoing.put(msg);
    } else {
      mail_outgoing.free(msg);
    }
  }
}

bool CommsContext::try_queue_send(const CommsMsg msg_vals) {
  // Only attempt to add a transmission request to the queue if the
  // queue is not full.
  CommsMsg *msg = mail_outgoing.try_alloc();
  if (msg == nullptr) {
    return false;
  }

  memcpy(msg, &msg_vals, MSG_SIZE);
  mail_outgoing.put(msg);

  return true;
}

bool CommsContext::try_read(CommsMsg *out) {
  // Only return a message if there is one to get from the queue.
  CommsMsg *read_msg = mail_incoming.try_get();
  if (read_msg == nullptr) {
    out = nullptr;
    return false;
  }

  memcpy(out, read_msg, MSG_SIZE);
  mail_incoming.free(read_msg);

  return true;
}
