#include "radio_transport.h"

#include <cstring>

#include "protocol.h"

namespace esphome::wisafe2 {

void RadioTransport::note_frame_complete(uint32_t now_ticks) {
  this->last_frame_tick_ = now_ticks;
  this->has_completed_frame_ = true;
}

uint32_t RadioTransport::command_delay_ticks(uint32_t now_ticks) const {
  if (!this->has_completed_frame_)
    return 0;
  const uint32_t gap = this->io_.transport_ms_to_ticks(this->command_gap_ms_);
  const uint32_t elapsed = now_ticks - this->last_frame_tick_;
  return elapsed < gap ? gap - elapsed : 0;
}

TransportError RadioTransport::abort_(TransportError cause, TransportExchangeResult *result) {
  const TransportError reset_error = this->io_.transport_reset();
  result->spi_reset = reset_error == TransportError::OK;
  return result->spi_reset ? cause : reset_error;
}

TransportError RadioTransport::exchange(const uint8_t *data, size_t length,
                                        TransportExchangeResult *result, unsigned response_packets,
                                        uint32_t response_timeout_ms) {
  if (data == nullptr || result == nullptr || length == 0 || length > RADIO_PACKET_MAX ||
      response_packets == 0)
    return TransportError::INVALID_ARGUMENT;

  memset(result, 0, sizeof(*result));
  uint8_t wire_data[RADIO_WIRE_PACKET_MAX]{};
  size_t wire_length = 0;
  if (!escape_frame(data, length, wire_data, sizeof(wire_data), &wire_length))
    return TransportError::INVALID_ARGUMENT;
  result->tx_expected = wire_length;

  const uint32_t command_delay = this->command_delay_ticks(this->io_.transport_now_ticks());
  if (command_delay > 0)
    this->io_.transport_delay_ticks(command_delay);

  const uint32_t byte_timeout = this->io_.transport_ms_to_ticks(this->byte_timeout_ms_);
  bool first_rx_queued = false;
  for (size_t i = 0; i < wire_length; ++i) {
    TransportError err = this->io_.transport_queue(TransportSlot::TX, wire_data[i], byte_timeout);
    if (err != TransportError::OK)
      return this->abort_(err, result);

    // Preserve the zero-gap TX-to-RX turnaround: the first receive slot must
    // be waiting before the radio clocks the final transmitted byte.
    if (i + 1 == wire_length) {
      err = this->io_.transport_queue(TransportSlot::RX_0, 0x00, byte_timeout);
      if (err != TransportError::OK)
        return this->abort_(err, result);
      first_rx_queued = true;
    }

    this->io_.transport_set_irq(true);
    TransportByte byte{};
    err = this->io_.transport_wait(TransportSlot::TX, byte_timeout, &byte);
    this->io_.transport_set_irq(false);
    if (err != TransportError::OK)
      return this->abort_(err, result);

    result->tx_bits[i] = byte.bits;
    result->simultaneous_rx[i] = byte.value;
    result->tx_count = i + 1;
  }
  this->note_frame_complete(this->io_.transport_now_ticks());

  if (!first_rx_queued)
    return TransportError::IO_ERROR;

  const uint32_t start = this->io_.transport_now_ticks();
  const uint32_t overall = this->io_.transport_ms_to_ticks(response_timeout_ms);
  TransportSlot current = TransportSlot::RX_0;
  unsigned packets_received = 0;
  bool escape_pending = false;
  size_t escape_bits = 0;
  while (result->response_len < RADIO_PACKET_MAX) {
    const uint32_t elapsed = this->io_.transport_now_ticks() - start;
    if (elapsed >= overall)
      return this->abort_(TransportError::TIMEOUT, result);

    TransportByte byte{};
    TransportError err = this->io_.transport_wait(current, overall - elapsed, &byte);
    if (err != TransportError::OK)
      return this->abort_(err, result);

    uint8_t value = byte.value;
    const bool terminator = !escape_pending && value == 0x7E;
    bool append = true;
    if (escape_pending) {
      if (value == 0x01)
        value = 0x7E;
      else if (value == 0x02)
        value = 0x7D;
      else
        return this->abort_(TransportError::INVALID_RESPONSE, result);
      escape_pending = false;
    } else if (value == 0x7D) {
      escape_pending = true;
      escape_bits = byte.bits;
      append = false;
    }
    if (append) {
      const size_t index = result->response_len++;
      result->response[index] = value;
      result->response_bits[index] = escape_bits > byte.bits ? escape_bits : byte.bits;
      escape_bits = 0;
    }
    if (terminator && ++packets_received == response_packets) {
      this->note_frame_complete(this->io_.transport_now_ticks());
      this->io_.transport_acknowledge_received_byte();
      return TransportError::OK;
    }

    const TransportSlot next = current == TransportSlot::RX_0 ? TransportSlot::RX_1
                                                               : TransportSlot::RX_0;
    err = this->io_.transport_queue(next, 0x00, byte_timeout);
    if (err != TransportError::OK)
      return this->abort_(err, result);
    // The radio sees its acknowledgement only after the next transaction is
    // queued, preserving the existing byte-to-byte receive timing.
    this->io_.transport_acknowledge_received_byte();
    current = next;
  }
  return this->abort_(TransportError::NO_MEMORY, result);
}

void AsyncRequestState::start(RemoteRequestType type, uint8_t sid, uint32_t now_ticks) {
  this->type_ = type;
  this->sid_ = sid;
  this->started_tick_ = now_ticks;
}

void AsyncRequestState::clear() {
  this->type_ = RemoteRequestType::NONE;
  this->sid_ = 0xFF;
}

bool AsyncRequestState::complete(RemoteRequestType type, uint8_t sid) {
  if (this->type_ != type || this->sid_ != sid)
    return false;
  this->clear();
  return true;
}

bool AsyncRequestState::timed_out(uint32_t now_ticks, uint32_t timeout_ticks) const {
  return this->active() && now_ticks - this->started_tick_ >= timeout_ticks;
}

bool AsyncRequestState::ready(uint32_t now_ticks, uint32_t timeout_ticks,
                              bool has_pending_work) const {
  return this->active() ? this->timed_out(now_ticks, timeout_ticks) : has_pending_work;
}

}  // namespace esphome::wisafe2
