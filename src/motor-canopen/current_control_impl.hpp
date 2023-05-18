#ifndef CURRENT_CONTROL_HPP
#error "Do not include this file directly, use current_control.hpp instead"
#endif

template <size_t id>
int16_t CurrentControl<id>::update(float commandedCurrent,
                                   const MotorState &state) {

  currentError_ = commandedCurrent - state.actualCurrent_;
  currentPid_.update(currentError_);

  return (int16_t)currentPid_.getValue();
}

template <size_t id>
void CurrentControl<id>::resetIfApplicable(const MotorState &state) {
  if (!state.enableMotor_ ||
      state.status_.state() != modm_canopen::cia402::State::OperationEnabled ||
      state.mode_ == OperatingMode::Disabled) {
    currentPid_.reset();
  }
}