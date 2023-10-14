#ifndef LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#define LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#include <cmath>
#include <modm/math/filter/moving_average.hpp>
namespace librobots2::motor {

template <size_t n> class BldcMotorCurrent {
public:
  BldcMotorCurrent() = default;

  void updateCurrentAverage(float alpha, float beta);

  float getOrientedCurrent() const;
  float getAngleDifference() const;
  float getMagnitude() const;

private:
  modm::filter::MovingAverage<float, n> magnitude_{}, angleDiff_{};
  float lastAngle_{0.0f};
  bool hasLast{false};
};
} // namespace librobots2::motor
#include "bldc_motor_current_impl.hpp"
#endif
