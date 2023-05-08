#ifndef VELOCITY_PROTOCOL_HPP
#error "Do not include this file directly, use velocity_protocol.hpp instead"
#endif

#include <algorithm>
#include <modm/debug/logger.hpp>

using CommandBits = modm_canopen::cia402::CommandBits;
using StatusBits = modm_canopen::cia402::StatusBits;
using OperatingMode = modm_canopen::cia402::OperatingMode;

template <size_t id>
template <typename Device, typename MessageCallback>
bool VelocityProtocol<id>::update(MotorState &state, MessageCallback &&) {

  if (state.mode_ == OperatingMode::Velocity ||
      state.control_.isSet<CommandBits::ChangeImmediately>() ||
      VelocityControl<id>::velocityError_ == 0) {
    commandedVelocity_ = receivedVelocity_;
  }

  if (state.control_.isSet<CommandBits::Halt>()) {
    commandedVelocity_ = 0;
  }

  if (commandedVelocity_ == 0 && state.actualVelocity_.getValue() == 0) {
    state.outputPWM_ = 0;
  } else {
    state.outputPWM_ =
        VelocityControl<id>::doVelocityUpdate(commandedVelocity_, state);
  }

  state.status_.setBit<StatusBits::TargetReached>(
      VelocityControl<id>::velocityError_ == 0);
  state.status_.setBit<StatusBits::SpeedZero>(
      state.actualVelocity_.getValue() ==
      0); // TODO implement velocity Threshold (for zero and no speed
          // indication)
  // TODO implement max slippage
  Device::setValueChanged(VelocityObjects::VelocityDemandValue);
  Device::setValueChanged(VelocityObjects::VelocityError);
  return true;
}

template <size_t id>
template <typename ObjectDictionary, const MotorState &state>
constexpr void VelocityProtocol<id>::registerHandlers(
    modm_canopen::HandlerMap<ObjectDictionary> &map) {
  using modm_canopen::SdoErrorCode;
  map.template setReadHandler<VelocityObjects::VelocityDemandValue>(+[]() {
    return state.scalingFactors_.velocity.toUser(commandedVelocity_);
  });

  map.template setReadHandler<VelocityObjects::VelocityError>(+[]() {
    return state.scalingFactors_.velocity.toUser(
        VelocityControl<id>::velocityError_);
  });

  map.template setReadHandler<VelocityObjects::TargetVelocity>(+[]() {
    return state.scalingFactors_.velocity.toUser(receivedVelocity_);
  });

  map.template setWriteHandler<VelocityObjects::TargetVelocity>(
      +[](int32_t value) {
        receivedVelocity_ = state.scalingFactors_.velocity.toInternal(value);
        MODM_LOG_INFO << "Set Target Velocity to " << receivedVelocity_
                      << modm::endl;
        return SdoErrorCode::NoError;
      });

  map.template setWriteHandler<VelocityObjects::VelocityPID_kP>(
      +[](float value) {
        VelocityControl<id>::velocityPidParameters_.setKp(value);
        VelocityControl<id>::velocityPid_.setParameter(
            VelocityControl<id>::velocityPidParameters_);
        return SdoErrorCode::NoError;
      });

  map.template setWriteHandler<VelocityObjects::VelocityPID_kI>(
      +[](float value) {
        VelocityControl<id>::velocityPidParameters_.setKi(value);
        VelocityControl<id>::velocityPid_.setParameter(
            VelocityControl<id>::velocityPidParameters_);
        return SdoErrorCode::NoError;
      });

  map.template setWriteHandler<VelocityObjects::VelocityPID_kD>(
      +[](float value) {
        VelocityControl<id>::velocityPidParameters_.setKd(value);
        VelocityControl<id>::velocityPid_.setParameter(
            VelocityControl<id>::velocityPidParameters_);
        return SdoErrorCode::NoError;
      });

  map.template setWriteHandler<VelocityObjects::VelocityPID_MaxErrorSum>(
      +[](float value) {
        VelocityControl<id>::velocityPidParameters_.setMaxErrorSum(value);
        VelocityControl<id>::velocityPid_.setParameter(
            VelocityControl<id>::velocityPidParameters_);
        return SdoErrorCode::NoError;
      });

  map.template setReadHandler<VelocityObjects::ProfileAcceleration>(+[]() {
    return state.scalingFactors_.acceleration.toUser(
        VelocityControl<id>::profileAcceleration_);
  });

  map.template setWriteHandler<VelocityObjects::ProfileAcceleration>(
      +[](int32_t value) {
        VelocityControl<id>::profileAcceleration_ =
            state.scalingFactors_.acceleration.toInternal(value);
        return SdoErrorCode::NoError;
      });
}
