#ifndef CURRENT_CONTROL_HPP
#error "Do not include this file directly, use current_control.hpp instead"
#endif
#include <algorithm>
#include <cmath>

template <size_t id>
template <typename Device>
int16_t CurrentControl<id>::update(float inCurrent, const MotorState &state) {

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

  // Copy sign of in current to actual Current
  filteredActualCurrent_ =
      std::copysign(state.actualCurrent_ - zeroAverage_.getValue(), inCurrent);
  Device::setValueChanged(CurrentObjects::FilteredActualCurrent);

  // Calculate remaining charge
  currentValues_.appendOverwrite(
      {std::abs(filteredActualCurrent_), secondsSinceLastExecute});
  currentCharge_ = getCharge();
  Device::setValueChanged(CurrentObjects::CurrentCharge);
  const auto remainingCharge =
      std::abs(std::abs(state.maxCharge_) - std::abs(currentCharge_));
  const auto remainingCurrent = remainingCharge / secondsSinceLastExecute;

  // Limit current to charge budget
  if (remainingCurrent <= 0.0f)
    commandedCurrent_ = 0;
  else if (std::abs(inCurrent) > remainingCurrent) {
    commandedCurrent_ =
        std::signbit(inCurrent) == std::signbit(remainingCurrent)
            ? remainingCurrent
            : -remainingCurrent;
  } else {
    commandedCurrent_ = inCurrent;
  }

  // Limit to max Current
  commandedCurrent_ =
      std::clamp(commandedCurrent_, -state.maxCurrent_, state.maxCurrent_);
  Device::setValueChanged(CurrentObjects::CommandedCurrent);

  currentError_ = commandedCurrent_ - filteredActualCurrent_;
  Device::setValueChanged(CurrentObjects::CurrentError);
  currentPid_.update(currentError_);

  return (int16_t)(currentPid_.getValue()) * rampMultiplier_;
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
  float accCurr = 0.0f;
  float accTime = 0;
  for (auto &pair : currentValues_) {
    accCurr += pair.first;
    accTime += pair.second;
  }
  if (accTime < 50)
    return 0.0f;
  accCurr /= currentValues_.getSize();
  return accCurr * accTime;
}

template <size_t id> void CurrentControl<id>::reset() {
  currentPid_.reset();
  currentValues_.clear();
  lastExecute_ = modm::Clock::now();
}