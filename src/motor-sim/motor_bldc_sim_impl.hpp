
#ifndef MOTOR_BLDC_SIM_HPP
#error "Do not use this file directly, include 'motor_bldc_sim.hpp' instead!"
#endif

#include <cmath>
#include <numbers>

#include <modm/debug/logger.hpp>

constexpr inline std::array<uint_fast8_t, 6> HallLut = {
    0b001, 0b010, 0b011, 0b100, 0b101, 0b110,
};

template <size_t id> uint_fast8_t SimMotorInterface<id>::readHall() {
  constexpr auto twoPi = 2 * std::numbers::pi;
  auto hall = std::fmod(motorSim.getRotorPos(), twoPi);
  if (hall < 0.0f)
    hall += twoPi;
  hall = 6 * hall / twoPi;
  const auto hall_integer = ((uint8_t)std::round(hall) + 1) % 6;
  const auto out = HallLut[hall_integer];
  // MODM_LOG_INFO << out << modm::endl;
  return out;
}
template <size_t id> void SimMotorInterface<id>::update() {
  const auto now = modm::PreciseClock::now();
  const auto diff = now - lastUpdate;
  motorSim.update((float)diff.count() / 1000000.0f);
  lastUpdate = now;
}