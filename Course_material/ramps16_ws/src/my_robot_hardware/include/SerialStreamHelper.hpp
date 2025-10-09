#pragma once
//
// Header-only SerialStreamHelper (LibSerial-based)
// All methods are defined inline so no .cpp is required.
//
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class SerialStreamHelper {
public:
  // Map integer baud to LibSerial enum
  static inline LibSerial::BaudRate to_baud(unsigned int baud) {
    using BR = LibSerial::BaudRate;
    switch (baud) {
      case 9600:    return BR::BAUD_9600;
      case 19200:   return BR::BAUD_19200;
      case 38400:   return BR::BAUD_38400;
      case 57600:   return BR::BAUD_57600;
      case 115200:  return BR::BAUD_115200;
      case 230400:  return BR::BAUD_230400;
      case 460800:  return BR::BAUD_460800;
      case 921600:  return BR::BAUD_921600;
      default: throw std::invalid_argument("Unsupported baud rate");
    }
  }

  // Open + basic port config
  inline SerialStreamHelper(const std::string& device, unsigned int baud_rate) {
    serial_port_.Open(device);
    if (!serial_port_.IsOpen()) {
      throw std::runtime_error("Failed to open serial port: " + device);
    }

    serial_port_.SetBaudRate(to_baud(baud_rate));
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    // Give Arduino CDC a moment (many boards reset on open)
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    // Drain boot noise quickly
    DrainInput(10);
  }

  inline ~SerialStreamHelper() {
    if (serial_port_.IsOpen()) {
      serial_port_.Close();
    }
  }

  // ---- Framed I/O helpers (10-byte frames) ----
  inline void WriteTenBytes(const std::vector<uint8_t>& data) {
    if (data.size() != 10) {
      throw std::invalid_argument("Data must be exactly 10 bytes long");
    }
    LibSerial::DataBuffer buf(data.begin(), data.end());
    serial_port_.Write(buf);
    serial_port_.DrainWriteBuffer();
  }

  inline std::vector<uint8_t> ReadTenBytes() {
    return ReadNBytes(10, /*total_timeout_ms*/ 500);
  }

  // Robust, byte-by-byte reader using IsDataAvailable + ReadByte
  inline std::vector<uint8_t> ReadNBytes(std::size_t n, int total_timeout_ms) {
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(total_timeout_ms);

    std::vector<uint8_t> out;
    out.reserve(n);

    while (out.size() < n) {
      if (clock::now() >= deadline) {
        throw std::runtime_error("Timeout waiting for " + std::to_string(n) + " bytes from serial port");
      }

      if (!serial_port_.IsDataAvailable()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }

      std::size_t want  = n - out.size();
      std::size_t avail = serial_port_.GetNumberOfBytesAvailable();
      if (avail == 0) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
      if (avail > want) avail = want;

      for (std::size_t i = 0; i < avail; ++i) {
        char ch{};
        serial_port_.ReadByte(ch); // safe: we checked availability
        out.push_back(static_cast<uint8_t>(ch));
      }
    }

    return out;
  }

  // Non-blocking drain for up to max_ms total time
  inline void DrainInput(int max_ms) {
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();

    while (clock::now() - t0 < std::chrono::milliseconds(max_ms)) {
      if (!serial_port_.IsDataAvailable()) break;

      std::size_t avail = serial_port_.GetNumberOfBytesAvailable();
      if (avail == 0) break;
      const std::size_t chunk_sz = std::min<std::size_t>(avail, 64);
      for (std::size_t i = 0; i < chunk_sz; ++i) {
        char ch{};
        serial_port_.ReadByte(ch);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // ---- Diagnostics / banner ----
  static inline std::string ToHex(const std::vector<uint8_t>& bytes) {
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      if (i) oss << ' ';
      oss << std::setw(2) << static_cast<int>(bytes[i]);
    }
    return oss.str();
  }

  inline void DumpHex(const std::vector<uint8_t>& frame, const char* tag) const {
    std::cerr << (tag ? tag : "RX") << " [" << ToHex(frame) << "]\n";
  }

  static inline bool IsReadyFrame(const std::vector<uint8_t>& f) {
    if (f.size() != 10) return false;
    if (!(f[0] == 0xAA && f[1] == 0x55 && f[2] == 0x90)) return false;
    unsigned sum = 0; for (int i = 0; i < 9; ++i) sum += f[i];
    return static_cast<uint8_t>(sum & 0xFF) == f[9];
  }

  inline void WaitForReadyBanner(bool hex_dump, int overall_ms = 2000) {
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(overall_ms);

    while (clock::now() < deadline) {
      try {
        auto f = ReadNBytes(10, /*total_timeout_ms*/ 200);
        if (hex_dump) DumpHex(f, "RX");
        if (IsReadyFrame(f)) {
          std::cerr << "[READY] banner seen.\n";
          return;
        }
      } catch (...) {
        // per-slice timeout; keep scanning until deadline
      }
    }
    std::cerr << "[READY] not seen (continuing anyway)\n";
  }

private:
  LibSerial::SerialPort serial_port_;
};
