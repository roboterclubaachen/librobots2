#ifndef CURRENT_CONTROL_HPP
#error "Do not include this file directly, use current_control.hpp instead"
#endif
#include <algorithm>
#include <cmath>

template <size_t id>
template <typename Device>
int16_t CurrentControl<id>::update(float inCurrent, const MotorState &state) {
  isLimiting_ = false;
  const auto now = modm::Clock::now();
  const auto timeSinceLastExecute = now - lastExecute_;
  const float secondsSinceLastExecute = timeSinceLastExecute.count() / 1000.0f;
  lastExecute_ = now;
  if (rampMultiplier_ < 1.0f) {
    rampMultiplier_ += rampIncrement_;
  }
  if (rampMultiplier_ > 1.0f) {
    rampMultiplier_ = 1.0f;
  }

  if (std::signbit(inCurrent) != std::signbit(filteredActualCurrent_)) {
    currentPid_.reset();
  }

  // Copy sign of in current to actual Current
  filteredActualCurrent_ =
      std::copysign(state.actualCurrent_ - zeroAverage_.getValue(), inCurrent);
  Device::setValueChanged(CurrentObjects::FilteredActualCurrent);

  // Calculate remaining charge
  currentValues_.appendOverwrite(
      {std::abs(filteredActualCurrent_), secondsSinceLastExecute});
  currentCharge_ = getCharge();
  Device::setValueChanged(CurrentObjects::CurrentCharge);

  commandedCurrent_ = inCurrent;
  if (std::abs(currentCharge_) > state.maxCharge_) {
    const auto projectedCharge =
        currentCharge_ + secondsSinceLastExecute * inCurrent;
    const auto toDoubleCharge =
        2.0f * state.maxCharge_ - std::abs(projectedCharge);
    const auto percentOfChargeRemaining = toDoubleCharge / state.maxCharge_;
    commandedCurrent_ *= percentOfChargeRemaining * percentOfChargeRemaining;
  }

  if (std::abs(currentCharge_) > state.maxCharge_ * 2.0f) {
    isLimiting_ = true;
    commandedCurrent_ = 0;
  }

  // Limit to max Current
  if (std::abs(commandedCurrent_) > std::abs(state.maxCurrent_)) {
    commandedCurrent_ =
        std::clamp(commandedCurrent_, -state.maxCurrent_, state.maxCurrent_);
    isLimiting_ = true;
  }
  Device::setValueChanged(CurrentObjects::CommandedCurrent);

  currentError_ = commandedCurrent_ - filteredActualCurrent_;
  Device::setValueChanged(CurrentObjects::CurrentError);

  if (std::abs(commandedCurrent_) < 0.05f)
    return 0;

  currentPid_.update(currentError_, false);
  const auto output = std::clamp(currentPid_.getValue(), -1.0f, 1.0f) *
                      std::numeric_limits<int16_t>::max();

  return (int16_t)output * rampMultiplier_;
}

template <size_t id>
void CurrentControl<id>::resetIfApplicable(const MotorState &state) {
  if (state.outputPWM_ == 0 &&
      std::abs(state.actualVelocity_.getValue()) <= 0.001f) {
    rampMultiplier_ = rampMultiplierReset_;
    if (zeroAverageCountdown_ == 0) {
      zeroAverage_.update(state.actualCurrent_);
    } else {
      zeroAverageCountdown_--;
    }
  } else {
    zeroAverageCountdown_ = zeroAverageCountdownReset_;
  }
  if (!state.enableMotor_ ||
      state.status_.state() != modm_canopen::cia402::State::OperationEnabled ||
      state.mode_ == OperatingMode::Disabled) {
    reset();
  }
}

template <size_t id> float CurrentControl<id>::getCharge() {
  float acc = 0.0f;
  for (auto &pair : currentValues_) {
    acc += pair.first * pair.second;
  }
  return acc;
}

template <size_t id> void CurrentControl<id>::reset() {
  currentPid_.reset();
  currentValues_.clear();
  lastExecute_ = modm::Clock::now();
}