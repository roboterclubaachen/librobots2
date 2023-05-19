#ifndef LIBMOTOR_BLDC_MOTOR_CURRENT_HPP
#error                                                                         \
    "Do not use this file directly, include 'bldc_motor_current.hpp' instead!"
#endif

namespace librobots2::motor {

static constexpr auto M_PI = 3.14159265358979323846;

template <size_t n> float BldcMotorCurrent<n>::angle(float alpha, float beta) {
  return std::atan2(beta, alpha);
}

template <size_t n>
float BldcMotorCurrent<n>::magnitude(float alpha, float beta) {
  return std::sqrt((alpha * alpha) + (beta * beta));
}

template <size_t n>
float BldcMotorCurrent<n>::angleDiff(float angle, float lastAngle) {
  auto angleDiff = angle - lastAngle;
  return angleDiff;
  angleDiff = std::fmod(angleDiff + M_PI, 2.0 * M_PI);
  if (angleDiff <= 0.0)
    return angleDiff + M_PI;
  return angleDiff - M_PI;
}

template <size_t n> void BldcMotorCurrent<n>::update(float alpha, float beta) {
  if (hasFirst_) {
    hasLast_ = true;
    lastAlpha_ = alpha_;
    lastBeta_ = beta_;
  }
  hasFirst_ = true;
  alpha_ = alpha;
  beta_ = beta;
}

template <size_t n> float BldcMotorCurrent<n>::getMagnitude() const {
  if (hasFirst_)
    return magnitude(alpha_, beta_);
  else
    return 0;
}

template <size_t n> float BldcMotorCurrent<n>::getAngleDifference() const {
  if (hasFirst_ && hasLast_)
    return angleDiff(angle(alpha_, beta_), angle(lastAlpha_, lastBeta_));
  else
    return 0;
}

template <size_t n> float BldcMotorCurrent<n>::getOrientedCurrent() const {
  if (!hasFirst_ || !hasLast_)
    return 0.0f;
  if (getAngleDifference() < 0.0f) {
    return -getMagnitude();
  }
  return getMagnitude();
}

} // namespace librobots2::motor