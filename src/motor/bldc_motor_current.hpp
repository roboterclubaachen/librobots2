#ifndef LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#define LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#include <cmath>
#include <modm/math/filter/moving_average.hpp>
namespace librobots2::motor {

template <size_t n> class BldcMotorCurrent {
public:
  BldcMotorCurrent() = default;

  void updateCurrentAverage(float alpha, float beta);
  void goToNextAverage();

  float getOrientedCurrent() const;
  float getAngleDifference() const;
  float getMagnitude() const;
  float getAngle() const;

private:
  modm::filter::MovingAverage<float, n> alpha_{}, beta_{};
  bool hasLast_{false};
  float lastMagnitude_{0.0f}, lastAngle_{0.0f};
};
} // namespace librobots2::motor
#include "bldc_motor_current_impl.hpp"
#endif
