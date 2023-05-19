#ifndef LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#define LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#include <cmath>
#include <modm/math/filter/moving_average.hpp>
namespace librobots2::motor {

template <size_t n> class BldcMotorCurrent {
public:
  BldcMotorCurrent() = default;

  void update(float alpha, float beta);

  float getAngleDifference() const;
  float getOrientedCurrent() const;
  float getMagnitude() const;

private:
  static float angle(float alpha, float beta);
  static float magnitude(float alpha, float beta);
  static float angleDiff(float angle, float lastAngle);

  float alpha_{0.0f}, beta_{0.0f};
  bool hasLast_{false}, hasFirst_{false};
  float lastAlpha_{0.0f}, lastBeta_{0.0f};
};
} // namespace librobots2::motor
#include "bldc_motor_current_impl.hpp"
#endif
