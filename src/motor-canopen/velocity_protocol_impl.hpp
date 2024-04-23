#ifndef VELOCITY_PROTOCOL_HPP
#error "Do not include this file directly, use velocity_protocol.hpp instead"
#endif

#include <algorithm>
#include <cmath>
#include <modm/debug/logger.hpp>

using CommandBits = modm_canopen::cia402::CommandBits;
using StatusBits = modm_canopen::cia402::StatusBits;
using OperatingMode = modm_canopen::cia402::OperatingMode;

template<size_t id>
template<typename Device, typename State, typename MessageCallback>
bool
VelocityProtocol<id>::update(MessageCallback &&)
{

	if (State::mode_ == OperatingMode::Velocity ||
		State::control_.template isSet<CommandBits::ChangeImmediately>() ||
		VelocityControl<id>::velocityError_ == 0)
	{
		commandedVelocity_ = receivedVelocity_;
	}

	if (State::control_.template isSet<CommandBits::Halt>()) { commandedVelocity_ = 0; }

	const auto [pwm, currentLimit] =
		VelocityControl<id>::template doVelocityUpdate<Device, State>(commandedVelocity_);
	State::outputPWM_ = pwm;
	State::outputCurrentLimit_ = currentLimit;

	State::status_.template setBit<StatusBits::TargetReached>(VelocityControl<id>::velocityError_ ==
															  0);
	State::status_.template setBit<StatusBits::SpeedZero>(State::actualVelocity_.getValue() ==
														  0);  // TODO implement velocity Threshold
															   // (for zero and no speed indication)
	// TODO implement max slippage
	Device::setValueChanged(VelocityObjects::VelocityDemandValue);
	Device::setValueChanged(VelocityObjects::VelocityError);
	return true;
}

template<size_t id>
template<typename ObjectDictionary, typename State>
constexpr void
VelocityProtocol<id>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{
	using modm_canopen::SdoErrorCode;
	map.template setReadHandler<VelocityObjects::VelocityDemandValue>(+[]() {
		if (State::mode_ != OperatingMode::Velocity && State::mode_ != OperatingMode::Position)
			return (int32_t)0;
		return State::scalingFactors_.velocity.toUser(VelocityControl<id>::commandedVel_);
	});

	map.template setReadHandler<VelocityObjects::VelocityError>(+[]() {
		if (State::mode_ != OperatingMode::Velocity && State::mode_ != OperatingMode::Position)
			return (int32_t)0;
		return State::scalingFactors_.velocity.toUser(VelocityControl<id>::velocityError_);
	});

	map.template setReadHandler<VelocityObjects::TargetVelocity>(
		+[]() { return State::scalingFactors_.velocity.toUser(receivedVelocity_); });

	map.template setWriteHandler<VelocityObjects::TargetVelocity>(+[](int32_t value) {
		receivedVelocity_ = State::scalingFactors_.velocity.toInternal(value);
		// MODM_LOG_INFO << "Set Target Velocity to " << receivedVelocity_
		//               << modm::endl;
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<VelocityObjects::VelocityPID_kP>(+[](float value) {
		VelocityControl<id>::velocityPidParameters_.setKp(value);
		VelocityControl<id>::velocityPid_.setParameter(VelocityControl<id>::velocityPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<VelocityObjects::VelocityPID_kI>(+[](float value) {
		VelocityControl<id>::velocityPidParameters_.setKi(value);
		VelocityControl<id>::velocityPid_.setParameter(VelocityControl<id>::velocityPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<VelocityObjects::VelocityPID_kD>(+[](float value) {
		VelocityControl<id>::velocityPidParameters_.setKd(value);
		VelocityControl<id>::velocityPid_.setParameter(VelocityControl<id>::velocityPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<VelocityObjects::VelocityPID_MaxErrorSum>(+[](float value) {
		VelocityControl<id>::velocityPidParameters_.setMaxErrorSum(value);
		VelocityControl<id>::velocityPid_.setParameter(VelocityControl<id>::velocityPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<VelocityObjects::ProfileAcceleration>(+[]() {
		return State::scalingFactors_.acceleration.toUser(
			VelocityControl<id>::profileAcceleration_);
	});

	map.template setWriteHandler<VelocityObjects::ProfileAcceleration>(+[](int32_t value) {
		VelocityControl<id>::profileAcceleration_ =
			State::scalingFactors_.acceleration.toInternal(value);
		return SdoErrorCode::NoError;
	});
}
