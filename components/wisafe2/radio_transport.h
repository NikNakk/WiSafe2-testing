#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::wisafe2 {

static constexpr size_t RADIO_PACKET_MAX = 64;
static constexpr size_t RADIO_WIRE_PACKET_MAX = RADIO_PACKET_MAX * 2;

enum class TransportError : uint8_t {
  OK,
  INVALID_ARGUMENT,
  TIMEOUT,
  INVALID_RESPONSE,
  NO_MEMORY,
  INVALID_STATE,
  IO_ERROR,
};

enum class TransportSlot : uint8_t { TX, RX_0, RX_1 };

struct TransportByte {
  uint8_t value;
  size_t bits;
};

struct TransportExchangeResult {
  bool spi_reset;
  size_t tx_count;
  size_t tx_expected;
  size_t tx_bits[RADIO_WIRE_PACKET_MAX];
  uint8_t simultaneous_rx[RADIO_WIRE_PACKET_MAX];
  size_t response_len;
  uint8_t response[RADIO_PACKET_MAX];
  size_t response_bits[RADIO_PACKET_MAX];
};

class RadioTransportIO {
 public:
  virtual ~RadioTransportIO() = default;
  virtual uint32_t transport_now_ticks() const = 0;
  virtual uint32_t transport_ms_to_ticks(uint32_t milliseconds) const = 0;
  virtual void transport_delay_ticks(uint32_t ticks) = 0;
  virtual TransportError transport_queue(TransportSlot slot, uint8_t tx_value,
                                         uint32_t timeout_ticks) = 0;
  virtual TransportError transport_wait(TransportSlot slot, uint32_t timeout_ticks,
                                        TransportByte *result) = 0;
  virtual void transport_set_irq(bool asserted) = 0;
  virtual void transport_acknowledge_received_byte() = 0;
  virtual TransportError transport_reset() = 0;
};

class RadioTransport {
 public:
  RadioTransport(RadioTransportIO &io, uint32_t byte_timeout_ms, uint32_t command_gap_ms)
      : io_(io), byte_timeout_ms_(byte_timeout_ms), command_gap_ms_(command_gap_ms) {}

  TransportError exchange(const uint8_t *data, size_t length, TransportExchangeResult *result,
                          unsigned response_packets, uint32_t response_timeout_ms);
  void note_frame_complete(uint32_t now_ticks);
  uint32_t command_delay_ticks(uint32_t now_ticks) const;

 protected:
  TransportError abort_(TransportError cause, TransportExchangeResult *result);

  RadioTransportIO &io_;
  uint32_t byte_timeout_ms_;
  uint32_t command_gap_ms_;
  uint32_t last_frame_tick_{0};
  bool has_completed_frame_{false};
};

enum class RemoteRequestType : uint8_t { NONE, IDENTITY, STATUS };

class AsyncRequestState {
 public:
  bool active() const { return this->type_ != RemoteRequestType::NONE; }
  RemoteRequestType type() const { return this->type_; }
  uint8_t sid() const { return this->sid_; }

  void start(RemoteRequestType type, uint8_t sid, uint32_t now_ticks);
  void clear();
  bool complete(RemoteRequestType type, uint8_t sid);
  bool timed_out(uint32_t now_ticks, uint32_t timeout_ticks) const;
  bool ready(uint32_t now_ticks, uint32_t timeout_ticks, bool has_pending_work) const;

 protected:
  RemoteRequestType type_{RemoteRequestType::NONE};
  uint8_t sid_{0xFF};
  uint32_t started_tick_{0};
};

}  // namespace esphome::wisafe2
