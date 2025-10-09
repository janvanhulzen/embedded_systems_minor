#pragma once
//
// Header-only StepperClient
// Merges declarations + definitions; all functions are inline.
//

#include <cstdint>
#include <vector>
#include <string>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iomanip>
#include <iostream>

#include "SerialStreamHelper.hpp"

class StepperClient {
public:
  // Protocol variants (your Arduino uses REGISTER)
  enum class Protocol : uint8_t { REGISTER = 0, COMPACT = 1 };

  struct Options {
    Protocol proto            = Protocol::REGISTER;
    bool     hex_dump         = false;
    int      ready_timeout_ms = 3000; // not used here (READY seen in SerialStreamHelper)
    int      rx_deadline_ms   = 600;  // per call overall deadline
    int      max_retries      = 2;    // retries on timeout
    int      backoff_ms       = 25;   // incremental backoff between retries
    bool     ping_on_timeout  = true; // send a 0x00 ping before retry
  };

  // --- constructors (split to avoid default-arg-inside-class rule) ---
  explicit StepperClient(SerialStreamHelper& ser)
  : ser_(ser), opt_() {}

  explicit StepperClient(SerialStreamHelper& ser, const Options& opt)
  : ser_(ser), opt_(opt) {}

  // -------- High-level framed ops ----------
  inline void write_u8(uint8_t id, uint8_t addr, uint8_t value) {
    (void)send_frame_expect(0x01, id, addr, static_cast<int32_t>(value), opt_.rx_deadline_ms);
  }

  inline void write_i32(uint8_t id, uint8_t addr, int32_t value) {
    (void)send_frame_expect(0x02, id, addr, value, opt_.rx_deadline_ms);
  }

  inline uint8_t read_u8(uint8_t id, uint8_t addr) {
    auto rx = send_frame_expect(0x03, id, addr, 0, opt_.rx_deadline_ms);
    return rx[5];
  }

  inline int32_t read_i32(uint8_t id, uint8_t addr) {
    auto rx = send_frame_expect(0x04, id, addr, 0, opt_.rx_deadline_ms);
    int32_t v =  (int32_t)(
        (uint32_t)rx[5]
      | ((uint32_t)rx[6] << 8)
      | ((uint32_t)rx[7] << 16)
      | ((uint32_t)rx[8] << 24));
    return v;
  }

  // Optional: best-effort sync poke
  inline void send_ping() {
    auto f = make_frame(0x00, 0, 0, 0);
    dump_hex(f, "TX");
    ser_.WriteTenBytes(f);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    try {
      auto rx = ser_.ReadTenBytes();
      dump_hex(rx, "RX");
    } catch (...) {
      // ignore best-effort timeout
    }
  }

private:
  SerialStreamHelper& ser_;
  Options             opt_;

  // --- helpers ---
  inline void dump_hex(const std::vector<uint8_t>& f, const char* tag) const {
    if (!opt_.hex_dump) return;
    std::cerr << (tag ? tag : "FR") << " [";
    for (size_t i = 0; i < f.size(); ++i) {
      if (i) std::cerr << ' ';
      std::cerr << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << (int)f[i];
    }
    std::cerr << std::dec << "]\n";
  }

  inline uint8_t checksum10(const std::vector<uint8_t>& f) const {
    unsigned s = 0;
    for (int i = 0; i < 9; ++i) s += f[i];
    return static_cast<uint8_t>(s & 0xFF);
  }

  inline std::vector<uint8_t> make_frame(uint8_t op, uint8_t id, uint8_t addr, int32_t val) const {
    std::vector<uint8_t> f(10, 0);
    f[0]=0xAA; f[1]=0x55;
    f[2]=op; f[3]=id; f[4]=addr;
    f[5] = (uint8_t)( val        & 0xFF);
    f[6] = (uint8_t)((val >> 8 ) & 0xFF);
    f[7] = (uint8_t)((val >> 16) & 0xFF);
    f[8] = (uint8_t)((val >> 24) & 0xFF);
    f[9] = checksum10(f);
    return f;
  }

  inline std::vector<uint8_t> send_frame_expect(uint8_t tx_op,
                                                uint8_t id,
                                                uint8_t addr,
                                                int32_t val,
                                                int rx_deadline_ms)
  {
    auto tx = make_frame(tx_op, id, addr, val);
    const uint8_t expected_op = static_cast<uint8_t>(tx_op | 0x80);

    for (int attempt = 0; attempt <= opt_.max_retries; ++attempt) {
      if (attempt > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(opt_.backoff_ms * attempt));
        if (opt_.ping_on_timeout) send_ping();
      }

      dump_hex(tx, "TX");
      ser_.WriteTenBytes(tx);
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // tiny pacing

      // collect frames in small slices up to deadline
      auto t0 = std::chrono::steady_clock::now();
      while (true) {
        int slice_ms = 250;
        int elapsed  = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0).count();
        int remain   = rx_deadline_ms - elapsed;
        if (remain <= 0) break;
        if (slice_ms > remain) slice_ms = remain;

        std::vector<uint8_t> rx;
        try {
          rx = ser_.ReadNBytes(10, slice_ms);
        } catch (...) {
          // slice timeout; loop to check overall deadline
          continue;
        }
        dump_hex(rx, "RX");

        // validate header + checksum
        if (!(rx.size() == 10 && rx[0] == 0xAA && rx[1] == 0x55)) {
          continue; // garbage
        }
        if (checksum10(rx) != rx[9]) {
          continue; // bad checksum
        }

        const uint8_t op = rx[2];

        // ignore READY (0x90) beacons
        if (op == 0x90) continue;

        // ignore generic error/ack (0x80) unless we sent a ping
        if (op == 0x80 && tx_op != 0x00) continue;

        // wrong op? might be a late frame, skip
        if (op != expected_op) continue;

        // check id/addr echo (your FW echoes these)
        if (rx[3] != id || rx[4] != addr) continue;

        return rx; // success
      }
      // try again (retry loop)
    }

    throw std::runtime_error("Timeout waiting for expected reply");
  }
};
