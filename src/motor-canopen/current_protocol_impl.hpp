#ifndef CURRENT_PROTOCOL_HPP
#error "Do not include this file directly, use current_protocol86.hpp instead"
#endif

using CommandBits = modm_canopen::cia402::CommandBits;
using StatusBits = modm_canopen::cia402::StatusBits;
using OperatingMode = modm_canopen::cia402::OperatingMode;

template <size_t id>
template <typename Device, typename MessageCallback>
bool CurrentProtocol<id>::update(MotorState &state, MessageCallback &&) {

  state.outputPWM_ =
      CurrentControl<id>::template update<Device>(targetCurrent_, state);
  if (targetCurrent_ == 0.0f) {
    state.outputPWM_ = 0;
  }
  return true;
}

template <size_t id>
template <typename ObjectDictionary, const MotorState &state>
constexpr void CurrentProtocol<id>::registerHandlers(
    modm_canopen::HandlerMap<ObjectDictionary> &map) {

  using modm_canopen::SdoErrorCode;
  map.template setReadHandler<CurrentObjects::CommandedCurrent>(+[]() {
    if (state.mode_ != OperatingMode::Current &&
        state.mode_ != OperatingMode::Velocity &&
        state.mode_ != OperatingMode::Position)
      return 0.0f;
    return CurrentControl<id>::commandedCurrent_;
  });

  map.template setReadHandler<CurrentObjects::FilteredActualCurrent>(+[]() {
    if (state.mode_ != OperatingMode::Current &&
        state.mode_ != OperatingMode::Velocity &&
        state.mode_ != OperatingMode::Position)
      return 0.0f;
    return CurrentControl<id>::filteredActualCurrent_;
  });

  map.template setReadHandler<CurrentObjects::TargetCurrent>(
      +[]() { return targetCurrent_; });

  map.template setWriteHandler<CurrentObjects::TargetCurrent>(+[](float value) {
    targetCurrent_ = value;
    return SdoErrorCode::NoError;
  });

  map.template setReadHandler<CurrentObjects::CurrentError>(+[]() {
    if (state.mode_ != OperatingMode::Current &&
        state.mode_ != OperatingMode::Velocity &&
        state.mode_ != OperatingMode::Position)
      return 0.0f;
    return CurrentControl<id>::currentError_;
  });

  map.template setWriteHandler<CurrentObjects::CurrentPID_kP>(+[](float value) {
    CurrentControl<id>::currentPidParameters_.setKp(value);
    CurrentControl<id>::currentPid_.setParameter(
        CurrentControl<id>::currentPidParameters_);
    return SdoErrorCode::NoError;
  });

  map.template setWriteHandler<CurrentObjects::CurrentPID_kI>(+[](float value) {
    CurrentControl<id>::currentPidParameters_.setKi(value);
    CurrentControl<id>::currentPid_.setParameter(
        CurrentControl<id>::currentPidParameters_);
    return SdoErrorCode::NoError;
  });

  map.template setWriteHandler<CurrentObjects::CurrentPID_kD>(+[](float value) {
    CurrentControl<id>::currentPidParameters_.setKd(value);
    CurrentControl<id>::currentPid_.setParameter(
        CurrentControl<id>::currentPidParameters_);
    return SdoErrorCode::NoError;
  });

  map.template setWriteHandler<CurrentObjects::CurrentPID_MaxErrorSum>(
      +[](float value) {
        CurrentControl<id>::currentPidParameters_.setMaxErrorSum(value);
        CurrentControl<id>::currentPid_.setParameter(
            CurrentControl<id>::currentPidParameters_);
        return SdoErrorCode::NoError;
      });
}
