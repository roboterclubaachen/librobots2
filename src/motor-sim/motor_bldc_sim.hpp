#ifndef MOTOR_BLDC_SIM_HPP
#define MOTOR_BLDC_SIM_HPP

#include <array>
#include <cstdint>
#include <cstdlib>
#include <librobots2/motor/motor_bridge.hpp>
#include <modm/debug/logger.hpp>
#include <modm/processing/timer.hpp>

#include "motor_simulation.hpp"

struct SimPinU {};
struct SimPinV {};
struct SimPinW {};

template <size_t id> struct SimMotorInterface {

  using Phase = librobots2::motor::Phase;
  using PhaseConfig = librobots2::motor::PhaseConfig;
  using BridgeConfig = librobots2::motor::BridgeConfig;

  static inline SimMotor motorSim{};
  static inline modm::PreciseClock::time_point lastUpdate{};

  static uint_fast8_t readHall(); // TODO implement

  static constexpr uint16_t MaxPwm{std::numeric_limits<int16_t>::max()};

  static inline void setCompareValue(uint16_t compareValue) {
    motorSim.setPhasePWM(Phase::PhaseU, compareValue);
    motorSim.setPhasePWM(Phase::PhaseV, compareValue);
    motorSim.setPhasePWM(Phase::PhaseW, compareValue);
  }

  static inline void setCompareValue(Phase phase, uint16_t compareValue) {
    motorSim.setPhasePWM(phase, compareValue);
  }

  static inline void configure(Phase phase, PhaseConfig phaseOutputConfig) {
    motorSim.setPhaseConfig(phase, phaseOutputConfig);
  }

  static inline void configure(const BridgeConfig &config) {
    configure(Phase::PhaseU, config.config[0]);
    configure(Phase::PhaseV, config.config[1]);
    configure(Phase::PhaseW, config.config[2]);
  }

  static inline void configure(PhaseConfig config) {
    configure(Phase::PhaseU, config);
    configure(Phase::PhaseV, config);
    configure(Phase::PhaseW, config);
  }

  static inline void initializeHall() {}

  static inline void initialize() { lastUpdate = modm::PreciseClock::now(); }

  template <typename Pin1, typename Pin2, typename Pin3> struct SimPort {
    static inline uint_fast8_t read() {
      return SimMotorInterface<id>::readHall();
    }
  };
  using HallPort = SimPort<SimPinU, SimPinV, SimPinW>;
  static void update(); // TODO implement
};

#include "motor_bldc_sim_impl.hpp"
#endif