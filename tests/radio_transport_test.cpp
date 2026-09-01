#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "radio_transport.h"

using esphome::wisafe2::AsyncRequestState;
using esphome::wisafe2::RadioTransport;
using esphome::wisafe2::RadioTransportIO;
using esphome::wisafe2::RemoteRequestType;
using esphome::wisafe2::TransportByte;
using esphome::wisafe2::TransportError;
using esphome::wisafe2::TransportExchangeResult;
using esphome::wisafe2::TransportSlot;

namespace {

int failures = 0;
int checks = 0;

#define CHECK(condition)                                                                                              \
  do {                                                                                                                \
    ++checks;                                                                                                         \
    if (!(condition)) {                                                                                               \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                                      \
      ++failures;                                                                                                     \
    }                                                                                                                 \
  } while (false)

const char *slot_name(TransportSlot slot) {
  switch (slot) {
    case TransportSlot::TX: return "TX";
    case TransportSlot::RX_0: return "RX0";
    case TransportSlot::RX_1: return "RX1";
  }
  return "?";
}

struct WaitStep {
  TransportSlot slot;
  TransportError error;
  uint8_t value;
  size_t bits;
  uint32_t advance_ticks;
};

class FakeTransportIO : public RadioTransportIO {
 public:
  uint32_t transport_now_ticks() const override { return this->now; }
  uint32_t transport_ms_to_ticks(uint32_t milliseconds) const override { return milliseconds; }

  void transport_delay_ticks(uint32_t ticks) override {
    this->events.push_back("DELAY:" + std::to_string(ticks));
    this->now += ticks;
  }

  TransportError transport_queue(TransportSlot slot, uint8_t tx_value,
                                 uint32_t timeout_ticks) override {
    char event[40]{};
    std::snprintf(event, sizeof(event), "QUEUE:%s:%02X:%u", slot_name(slot), tx_value, timeout_ticks);
    this->events.emplace_back(event);
    if (this->queue_errors.empty())
      return TransportError::OK;
    const TransportError error = this->queue_errors.front();
    this->queue_errors.pop_front();
    return error;
  }

  TransportError transport_wait(TransportSlot slot, uint32_t timeout_ticks,
                                TransportByte *result) override {
    char event[40]{};
    std::snprintf(event, sizeof(event), "WAIT:%s:%u", slot_name(slot), timeout_ticks);
    this->events.emplace_back(event);
    this->wait_timeouts.push_back(timeout_ticks);
    if (this->wait_steps.empty())
      return TransportError::IO_ERROR;
    const WaitStep step = this->wait_steps.front();
    this->wait_steps.pop_front();
    if (step.slot != slot)
      this->slot_mismatch = true;
    this->now += step.advance_ticks;
    if (step.error == TransportError::OK && result != nullptr) {
      result->value = step.value;
      result->bits = step.bits;
    }
    return step.error;
  }

  void transport_set_irq(bool asserted) override {
    this->events.emplace_back(asserted ? "IRQ:ON" : "IRQ:OFF");
  }

  void transport_acknowledge_received_byte() override { this->events.emplace_back("ACK"); }

  TransportError transport_reset() override {
    this->events.emplace_back("RESET");
    ++this->reset_count;
    return this->reset_error;
  }

  void add_successful_exchange(uint8_t first_tx_bits = 8, uint8_t second_tx_bits = 8,
                               uint8_t first_rx_bits = 8, uint8_t second_rx_bits = 8) {
    this->wait_steps.push_back({TransportSlot::TX, TransportError::OK, 0x00, first_tx_bits, 0});
    this->wait_steps.push_back({TransportSlot::TX, TransportError::OK, 0x00, second_tx_bits, 0});
    this->wait_steps.push_back({TransportSlot::RX_0, TransportError::OK, 0x46, first_rx_bits, 0});
    this->wait_steps.push_back({TransportSlot::RX_1, TransportError::OK, 0x7E, second_rx_bits, 0});
  }

  uint32_t now{0};
  std::deque<WaitStep> wait_steps;
  std::deque<TransportError> queue_errors;
  std::vector<std::string> events;
  std::vector<uint32_t> wait_timeouts;
  TransportError reset_error{TransportError::OK};
  unsigned reset_count{0};
  bool slot_mismatch{false};
};

void check_events(const std::vector<std::string> &actual,
                  const std::vector<std::string> &expected) {
  CHECK(actual.size() == expected.size());
  const size_t count = actual.size() < expected.size() ? actual.size() : expected.size();
  for (size_t i = 0; i < count; ++i)
    CHECK(actual[i] == expected[i]);
}

void test_exchange_timing_and_order() {
  FakeTransportIO io;
  RadioTransport transport(io, 1000, 500);
  transport.note_frame_complete(100);
  io.now = 250;
  io.add_successful_exchange(9, 10, 9, 10);

  const uint8_t request[] = {0xD3, 0x7E};
  TransportExchangeResult result{};
  CHECK(transport.exchange(request, sizeof(request), &result, 1, 1000) == TransportError::OK);
  CHECK(!io.slot_mismatch);
  CHECK(io.now == 600);
  CHECK(result.tx_count == 2);
  CHECK(result.tx_expected == 2);
  CHECK(result.tx_bits[0] == 9);
  CHECK(result.tx_bits[1] == 10);
  CHECK(result.response_len == 2);
  CHECK(result.response[0] == 0x46);
  CHECK(result.response[1] == 0x7E);
  CHECK(result.response_bits[0] == 9);
  CHECK(result.response_bits[1] == 10);
  CHECK(!result.spi_reset);

  check_events(io.events,
               {"DELAY:350", "QUEUE:TX:D3:1000", "IRQ:ON", "WAIT:TX:1000", "IRQ:OFF",
                "QUEUE:TX:7E:1000", "QUEUE:RX0:00:1000", "IRQ:ON", "WAIT:TX:1000",
                "IRQ:OFF", "WAIT:RX0:1000", "QUEUE:RX1:00:1000", "ACK", "WAIT:RX1:1000",
                "ACK"});
}

void test_response_timeout_and_recovery() {
  FakeTransportIO io;
  RadioTransport transport(io, 1000, 500);
  io.wait_steps.push_back({TransportSlot::TX, TransportError::OK, 0x00, 8, 0});
  io.wait_steps.push_back({TransportSlot::TX, TransportError::OK, 0x00, 8, 0});
  io.wait_steps.push_back({TransportSlot::RX_0, TransportError::OK, 0x46, 8, 600});
  io.wait_steps.push_back({TransportSlot::RX_1, TransportError::TIMEOUT, 0x00, 0, 400});

  const uint8_t request[] = {0xD3, 0x7E};
  TransportExchangeResult result{};
  CHECK(transport.exchange(request, sizeof(request), &result, 1, 1000) == TransportError::TIMEOUT);
  CHECK(result.spi_reset);
  CHECK(result.response_len == 1);
  CHECK(io.reset_count == 1);
  CHECK(io.wait_timeouts.size() == 4);
  CHECK(io.wait_timeouts[2] == 1000);
  CHECK(io.wait_timeouts[3] == 400);

  io.events.clear();
  io.wait_timeouts.clear();
  io.add_successful_exchange();
  CHECK(transport.exchange(request, sizeof(request), &result, 1, 1000) == TransportError::OK);
  CHECK(result.response_len == 2);
  CHECK(io.reset_count == 1);
}

void test_failed_reset_is_reported() {
  FakeTransportIO io;
  RadioTransport transport(io, 1000, 500);
  io.queue_errors.push_back(TransportError::IO_ERROR);
  io.reset_error = TransportError::INVALID_STATE;

  const uint8_t request[] = {0xD3, 0x7E};
  TransportExchangeResult result{};
  CHECK(transport.exchange(request, sizeof(request), &result, 1, 1000) ==
        TransportError::INVALID_STATE);
  CHECK(!result.spi_reset);
  CHECK(result.tx_count == 0);
  CHECK(io.reset_count == 1);
}

void test_tick_wraparound() {
  FakeTransportIO io;
  RadioTransport transport(io, 1000, 500);
  transport.note_frame_complete(0xFFFFFFF0U);
  CHECK(transport.command_delay_ticks(0x00000020U) == 452);
}

void test_asynchronous_response_state() {
  AsyncRequestState state;
  CHECK(!state.active());
  CHECK(!state.ready(1000, 20000, false));
  CHECK(state.ready(1000, 20000, true));

  state.start(RemoteRequestType::IDENTITY, 7, 1000);
  CHECK(state.active());
  CHECK(!state.ready(20999, 20000, true));
  CHECK(!state.complete(RemoteRequestType::STATUS, 7));
  CHECK(!state.complete(RemoteRequestType::IDENTITY, 8));
  CHECK(state.active());
  CHECK(state.complete(RemoteRequestType::IDENTITY, 7));
  CHECK(!state.active());
  CHECK(state.ready(21000, 20000, true));

  state.start(RemoteRequestType::STATUS, 9, 0xFFFFFFF0U);
  CHECK(!state.timed_out(0x00000020U, 49));
  CHECK(state.timed_out(0x00000021U, 49));
  state.clear();
  CHECK(state.sid() == 0xFF);
  CHECK(state.type() == RemoteRequestType::NONE);
}

}  // namespace

int main() {
  test_exchange_timing_and_order();
  test_response_timeout_and_recovery();
  test_failed_reset_is_reported();
  test_tick_wraparound();
  test_asynchronous_response_state();

  if (failures != 0) {
    std::fprintf(stderr, "%d of %d checks failed\n", failures, checks);
    return 1;
  }
  std::printf("All %d WiSafe2 radio transport checks passed\n", checks);
  return 0;
}
