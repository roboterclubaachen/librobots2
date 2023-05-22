#ifndef LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#error                                                                         \
    "Do not use this file directly, include 'bldc_motor_current.hpp' instead!"
#endif

namespace librobots2::motor {

static constexpr auto M_PI = 3.14159265358979323846;

template <size_t n>
void BldcMotorCurrent<n>::updateCurrentAverage(float alpha, float beta) {
  alpha_.update(alpha);
  beta_.update(beta);
}

template <size_t n> float BldcMotorCurrent<n>::getMagnitude() const {
  auto val_a = alpha_.getValue();
  val_a = val_a * val_a;
  auto val_b = beta_.getValue();
  val_b = val_b * val_b;
  return std::sqrt(val_a + val_b);
}

template <size_t n> float BldcMotorCurrent<n>::getAngle() const {
  auto val_a = alpha_.getValue();
  auto val_b = beta_.getValue();
  return std::atan2(val_a, val_b);
}

template <size_t n> void BldcMotorCurrent<n>::goToNextAverage() {
  lastMagnitude_ = getMagnitude();
  lastAngle_ = getAngle();
  hasLast_ = true;
}

template <size_t n> float BldcMotorCurrent<n>::getAngleDifference() const {
  if (!hasLast_)
    return 0.0f;
  auto angleDiff = getAngle() - lastAngle_;
  angleDiff = std::fmod(angleDiff + M_PI, 2.0 * M_PI);
  if (angleDiff <= 0.0)
    return angleDiff + M_PI;
  return angleDiff - M_PI;
}

template <size_t n> float BldcMotorCurrent<n>::getOrientedCurrent() const {
  if (hasLast_)
    return 0.0f;
  if (getAngleDifference() < 0.0f) {
    return -getMagnitude();
  }
  return getMagnitude();
}

} // namespace librobots2::motor