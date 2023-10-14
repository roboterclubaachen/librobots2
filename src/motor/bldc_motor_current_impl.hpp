#ifndef LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#error                                                                         \
    "Do not use this file directly, include 'bldc_motor_current.hpp' instead!"
#endif
#include <numbers>

namespace librobots2::motor {

template <size_t n>
void BldcMotorCurrent<n>::updateCurrentAverage(float alpha, float beta) {
  auto magn = std::sqrt(alpha * alpha + beta * beta);
  auto angl = std::atan2(alpha, beta);
  magnitude_.update(magn);
  if (hasLast) {
    auto angleDiff = angl - lastAngle_;
    angleDiff = std::fmod(angleDiff + std::numbers::pi_v<float>,
                          2.0f * std::numbers::pi_v<float>);
    if (angleDiff <= 0.0f)
      angleDiff = angleDiff + std::numbers::pi_v<float>;
    else
      angleDiff = angleDiff - std::numbers::pi_v<float>;
    angleDiff_.update(angleDiff);
  }
  hasLast = true;
  lastAngle_ = angl;
}

template <size_t n> float BldcMotorCurrent<n>::getMagnitude() const {
  return magnitude_.getValue();
}

template <size_t n> float BldcMotorCurrent<n>::getAngleDifference() const {
  return angleDiff_.getValue();
}

template <size_t n> float BldcMotorCurrent<n>::getOrientedCurrent() const {
  if (getAngleDifference() < 0.0f) {
    return -getMagnitude();
  }
  return getMagnitude();
}

} // namespace librobots2::motor