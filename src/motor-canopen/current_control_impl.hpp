#ifndef CURRENT_CONTROL_HPP
#error "Do not include this file directly, use current_control.hpp instead"
#endif
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
      remainingCharge / (timeSinceLastExecute.count() / 1000.0f);
  if (commandedCurrent_ > commandedCurrent) {
    commandedCurrent_ = commandedCurrent;
  }

  currentError_ = commandedCurrent_ - state.actualCurrent_;
  currentPid_.update(currentError_);

  Device::setValueChanged(CurrentObjects::CurrentError);
  Device::setValueChanged(CurrentObjects::CommandedCurrent);
  return (int16_t)(currentPid_.getValue() * 1.0f);
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