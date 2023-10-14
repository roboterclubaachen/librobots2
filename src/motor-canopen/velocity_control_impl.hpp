#ifndef VELOCITY_CONTROL_HPP
#error "Do not include this file directly, use velocity_control.hpp instead"
#endif

template <size_t id>
template <typename Device>
int16_t VelocityControl<id>::doVelocityUpdate(int32_t inVelocity,
                                              const MotorState &state) {

  commandedVel_ = inVelocity;
  if (std::signbit(commandedVel_) !=
          std::signbit(state.actualVelocity_.getValue()) &&
      (state.actualVelocity_.getValue() != 0 && commandedVel_ != 0)) {
    reset();
  }
  if (std::abs(commandedVel_) < 64) // deadband because we oscillate otherwise
    commandedVel_ = 0;
  if ((std::signbit(commandedVel_) ==
       std::signbit(state.actualVelocity_.getValue())) ||
      (state.actualVelocity_.getValue() == 0 && commandedVel_ != 0)) {
    isLimiting_ = state.outputPWM_ > profileAcceleration_ ||
                  CurrentControl<id>::isLimiting_;
    velocityError_ = commandedVel_ - state.actualVelocity_.getValue();
    velocityPid_.update(velocityError_, isLimiting_);
    Device::setValueChanged(VelocityObjects::VelocityError);
    return CurrentControl<id>::template update<Device>(velocityPid_.getValue(),
                                                       state);
  }
  return 0;
}

template <size_t id>
template <typename Device>
int16_t VelocityControl<id>::doDecelerationUpdate(int32_t commandedDeceleration,
                                                  const MotorState &state) {
  commandedVel_ = 0;
  velocityError_ = -state.actualVelocity_.getValue();
  isLimiting_ = state.outputPWM_ > commandedDeceleration ||
                CurrentControl<id>::isLimiting_;
  velocityPid_.update(velocityError_, isLimiting_);
  Device::setValueChanged(VelocityObjects::VelocityError);
  return (int16_t)std::clamp(
      (int32_t)CurrentControl<id>::template update<Device>(
          velocityPid_.getValue(), state),
      -commandedDeceleration, commandedDeceleration);
}

template <size_t id>
void VelocityControl<id>::resetIfApplicable(const MotorState &state) {
  if (!state.enableMotor_ ||
      state.status_.state() != modm_canopen::cia402::State::OperationEnabled ||
      state.mode_ == OperatingMode::Disabled ||
      state.mode_ == OperatingMode::Voltage) {
    reset();
  }
}

template <size_t id> void VelocityControl<id>::reset() {
  velocityPid_.reset();
  CurrentControl<id>::reset();
}