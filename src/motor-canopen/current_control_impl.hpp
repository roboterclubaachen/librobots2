#ifndef CURRENT_CONTROL_HPP
#error "Do not include this file directly, use current_control.hpp instead"
#endif
#include <algorithm>
#include <modm/debug/logger.hpp>

template <size_t id>
template <typename Device>
int16_t CurrentControl<id>::update(float commandedCurrent,
                                   const MotorState &state) {

  const auto now = modm::Clock::now();
  const auto timeSinceLastExecute = now - lastExecute_;
  lastExecute_ = now;

  currentValues_.appendOverwrite(
      {state.actualCurrent_, timeSinceLastExecute.count()});

  const auto remainingCharge = state.maxCharge_ - getCharge();
  commandedCurrent_ =
      commandedCurrent_ + (k_pt1 * commandedCurrent - commandedCurrent_) *
                              timeSinceLastExecute.count() / t_pt1;
  commandedCurrent_ =
      remainingCharge / (timeSinceLastExecute.count() / 1000.0f);
  if (commandedCurrent_ > commandedCurrent) {
    commandedCurrent_ = commandedCurrent;
  }
  commandedCurrent_ =
      std::clamp(commandedCurrent_, -state.maxCurrent_, state.maxCurrent_);

  currentError_ = commandedCurrent_ - state.actualCurrent_;
  currentPid_.update(currentError_);

  Device::setValueChanged(CurrentObjects::CurrentError);
  Device::setValueChanged(CurrentObjects::CommandedCurrent);

  auto newPWM = (int16_t)(currentPid_.getValue());
  constexpr auto maxPWMChange = 1000;
  if (std::abs(newPWM - state.outputPWM_) > maxPWMChange) {
    if (newPWM - state.outputPWM_ > 0) {
      newPWM = state.outputPWM_ + maxPWMChange;
    } else {
      newPWM = state.outputPWM_ - maxPWMChange;
    }
  }
  return newPWM;
}

template <size_t id>
void CurrentControl<id>::resetIfApplicable(const MotorState &state) {
  if (!state.enableMotor_ ||
      state.status_.state() != modm_canopen::cia402::State::OperationEnabled ||
      state.mode_ == OperatingMode::Disabled) {
    currentPid_.reset();
  }
}

template <size_t id> float CurrentControl<id>::getCharge() {
  float accCurr = 0.0f;
  int32_t accTime = 0;
  for (auto &pair : currentValues_) {
    accCurr += pair.first;
    accTime += pair.second;
  }
  accCurr /= currentValues_.getSize();
  return accCurr * accTime / 1000.0f; // Convert to As from A * ms
}