#pragma once
//
// High-level stepper API built on top of StepperClient.
// Header-only; safe two-phase init (Prepare -> Init).
//

#include <cstdint>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>

#include "SerialStreamHelper.hpp"
#include "StepperClient.hpp"

namespace stepper {

// --- Register map (must match Arduino firmware) ---
static constexpr uint8_t ADDR_OPERATING_MODE    = 1;  // 0=velocity, 1=position
static constexpr uint8_t ADDR_TORQUE_ENABLE     = 2;  // 0=disable, 1=enable
static constexpr uint8_t ADDR_GOAL_VELOCITY     = 3;  // int32 steps/s (signed)
static constexpr uint8_t ADDR_GOAL_POSITION     = 4;  // int32 steps (absolute)
static constexpr uint8_t ADDR_PRESENT_VELOCITY  = 5;  // int32 steps/s
static constexpr uint8_t ADDR_PRESENT_POSITION  = 6;  // int32 steps

// Simple calibration container.
struct Cal {
  double steps_per_rev{200.0};
};

// A single-bundle “System” that carries config + owned objects.
// Build order:
//   1) Prepare(...) fills config only.
//   2) Init(System&) opens serial, creates client, then creates driver.
struct System {
  // ---- configuration (filled by Prepare) ----
  std::string device;
  unsigned    baud{57600};
  uint8_t     motor_id_1{1};
  uint8_t     motor_id_2{2};
  uint8_t     motor_id_3{3};
  uint8_t     motor_id_4{4};
  uint8_t     motor_id_5{5};
  double      steps_per_rev{200.0};
  bool        hex_dump{false};
  bool        probe{false};
  std::string protocol{"register"};

  // ---- owned objects (populated by Init) ----
  std::unique_ptr<SerialStreamHelper> ser;
  std::unique_ptr<StepperClient>      client;
  std::unique_ptr<class StepperDriver> driver;   // created last

  Cal cal{steps_per_rev};

  // convenience: true once driver is constructed
  bool ready() const noexcept { return static_cast<bool>(driver); }
};

class StepperDriver {
public:
  // Construct ONLY after StepperClient exists; we keep a reference.
  explicit StepperDriver(StepperClient& client) : client_(client) {}

  // -------- Two-phase init helpers --------

  // 1) Prepare: fill a System config (no HW side-effects).
  static inline System Prepare(const std::string& device,
                               unsigned baud,
                               uint8_t motor_id_1,
                               uint8_t motor_id_2,
                               uint8_t motor_id_3,
                               uint8_t motor_id_4,
                               uint8_t motor_id_5,
                               double steps_per_rev,
                               bool hex_dump,
                               bool probe,
                               const std::string& protocol)
  {
    System sys;
    sys.device        = device;
    sys.baud          = baud;
    sys.motor_id_1    = motor_id_1;
    sys.motor_id_2    = motor_id_2;
    sys.motor_id_3    = motor_id_3;
    sys.motor_id_4    = motor_id_4;
    sys.motor_id_5    = motor_id_5;
    sys.steps_per_rev = steps_per_rev;
    sys.hex_dump      = hex_dump;
    sys.probe         = probe;
    sys.protocol      = protocol;
    sys.cal.steps_per_rev = steps_per_rev;
    return sys;
  }

  // 2) Init: open serial, construct client, then driver.
  // Returns 0 on success; throws on fatal errors.
  static inline int Init(System& sys)
  {
    if (sys.device.empty()) {
      throw std::invalid_argument("Serial device path is empty");
    }

    // Open serial
    std::cout << "Opening " << sys.device << " @ " << sys.baud << "...\n";
    sys.ser = std::make_unique<SerialStreamHelper>(sys.device, sys.baud);

    // Give Arduino Mega CDC time after DTR open
    std::this_thread::sleep_for(std::chrono::milliseconds(1800));
    std::cout << "Port opened.\n";

    // Optional: scan READY banner (useful during bring-up)
    if (sys.probe || sys.hex_dump) {
      sys.ser->WaitForReadyBanner(/*hex_dump*/ true, /*overall_ms*/ 2000);
    }

    // Client options
    StepperClient::Options opt;
    opt.hex_dump         = sys.hex_dump || sys.probe;
    opt.ready_timeout_ms = 3000;
    opt.rx_deadline_ms   = 3000;
    opt.max_retries      = 2;
    opt.backoff_ms       = 25;
    opt.ping_on_timeout  = true;
    opt.proto = (sys.protocol == "compact")
                  ? StepperClient::Protocol::COMPACT
                  : StepperClient::Protocol::REGISTER;

    // Create client
    sys.client = std::make_unique<StepperClient>(*sys.ser, opt);

    // Finally create driver that *references* the client we just created
    sys.driver = std::make_unique<StepperDriver>(*sys.client);

    std::cout << "Connected. proto="
              << (opt.proto == StepperClient::Protocol::REGISTER ? "register" : "compact")
              << " id1=" << static_cast<int>(sys.motor_id_1)
              << " id2=" << static_cast<int>(sys.motor_id_2)
              << " id3=" << static_cast<int>(sys.motor_id_3)
              << " id4=" << static_cast<int>(sys.motor_id_4)
              << " id5=" << static_cast<int>(sys.motor_id_5)
              << " steps/rev=" << sys.steps_per_rev
              << " hex=" << (opt.hex_dump ? "on" : "off") << "\n";

    return 0;
  }

  // -------- Mode / power --------
  inline int activateWithVelocityMode(uint8_t id) {
    client_.write_u8(id, ADDR_OPERATING_MODE, 0); // velocity
    client_.write_u8(id, ADDR_TORQUE_ENABLE,  1); // enable
    return 0;
  }

  inline int activateWithPositionMode(uint8_t id) {
    client_.write_u8(id, ADDR_OPERATING_MODE, 1); // position
    client_.write_u8(id, ADDR_TORQUE_ENABLE,  1); // enable
    return 0;
  }

  inline int deactivate(uint8_t id) {
    client_.write_u8(id, ADDR_TORQUE_ENABLE, 0);
    return 0;
  }

  // -------- Commands (velocity / position) --------
  inline int setTargetVelocityRadianPerSec(uint8_t id, double rad_s, const Cal& cal) {
    const double sps_f = rad_s * (cal.steps_per_rev / (2.0 * M_PI));
    const int32_t sps  = static_cast<int32_t>(std::llround(sps_f));
    client_.write_i32(id, ADDR_GOAL_VELOCITY, sps);
    return 0;
  }

  inline int setTargetPositionRadian(uint8_t id, double rad, const Cal& cal) {
    const double steps_f = rad * (cal.steps_per_rev / (2.0 * M_PI));
    const int32_t steps  = static_cast<int32_t>(std::llround(steps_f));
    client_.write_i32(id, ADDR_GOAL_POSITION, steps);
    return 0;
  }

  inline int setTargetPositionSteps(uint8_t id, int32_t steps) {
    client_.write_i32(id, ADDR_GOAL_POSITION, steps);
    return 0;
  }

  // -------- Telemetry --------
  inline double getVelocityRadianPerSec(uint8_t id, const Cal& cal) {
    const int32_t sps = client_.read_i32(id, ADDR_PRESENT_VELOCITY);
    return static_cast<double>(sps) * (2.0 * M_PI / cal.steps_per_rev);
  }

  inline double getPositionRadian(uint8_t id, const Cal& cal) {
    const int32_t steps = client_.read_i32(id, ADDR_PRESENT_POSITION);
    return static_cast<double>(steps) * (2.0 * M_PI / cal.steps_per_rev);
  }

  inline int32_t getPositionSteps(uint8_t id) {
    return client_.read_i32(id, ADDR_PRESENT_POSITION);
  }

private:
  StepperClient& client_;
};

} // namespace stepper
