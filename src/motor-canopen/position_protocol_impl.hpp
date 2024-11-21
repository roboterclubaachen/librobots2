#ifndef POSITION_PROTOCOL_HPP
#error "Do not include this file directly, use position_protocol.hpp instead"
#endif
#include "velocity_protocol.hpp"
#include "state_objects.hpp"
#include <modm/debug/logger.hpp>

template<size_t id>
template<typename State>
bool
PositionProtocol<id>::applicable()
{
	const auto value = State::mode_ == OperatingMode::Position &&
					   State::status_.state() == modm_canopen::cia402::State::OperationEnabled;
	if (!value)
	{
		positionPid_.reset();
		if (State::mode_ == OperatingMode::Position) { VelocityControl<id>::reset(); }
	}
	return value;
}

template<size_t id>
template<typename Device, typename State, typename MessageCallback>
bool
PositionProtocol<id>::update(MessageCallback &&)
{
	if (State::control_.template isSet<CommandBits::NewSetPoint>())
	{
		MODM_LOG_DEBUG << "Set next Target position to " << receivedPosition_ << modm::endl;
		nextPosition_ = receivedPosition_;
		nextPositionIsNew_ = true;
		State::control_.template setBit<CommandBits::NewSetPoint>(false);
		Device::setValueChanged(StateObjects::ControlWord);
	}

	if ((positionError_ == 0 && nextPositionIsNew_) ||
		State::control_.template isSet<CommandBits::ChangeImmediately>())
	{
		nextPositionIsNew_ = false;
		if (State::control_.template isSet<CommandBits::IsRelative>())
		{
			commandedPosition_ += nextPosition_;
			MODM_LOG_DEBUG << "Set current Target position to " << commandedPosition_ << " (rel)"
						   << modm::endl;
		} else if (commandedPosition_ != nextPosition_)
		{
			commandedPosition_ = nextPosition_;
			MODM_LOG_DEBUG << "Set current Target position to " << commandedPosition_ << modm::endl;
		}
		Device::setValueChanged(PositionObjects::PositionDemandValue);
		VelocityControl<id>::reset();
	}

	positionError_ = commandedPosition_ - State::actualPosition_;
	Device::setValueChanged(PositionObjects::FollowingErrorActualValue);

	positionPid_.update(positionError_, std::abs(positionPid_.getValue()) > 6000 ||
											VelocityControl<id>::isLimiting_);
	const auto [pwm, currentLimit] =
		VelocityControl<id>::template doVelocityUpdate<Device, State>(positionPid_.getValue());
	State::outputPWM_ = pwm;
	State::outputCurrentLimit_ = currentLimit;

	Device::setValueChanged(VelocityObjects::VelocityError);
	Device::setValueChanged(VelocityObjects::VelocityDemandValue);

	if ((uint32_t)std::abs(positionError_) <= positionWindow_)
	{
		if (inPositionWindow_ < positionWindowTime_) inPositionWindow_++;
	} else
	{
		inPositionWindow_ = 0;
	}

	State::status_.template setBit<StatusBits::TargetReached>(inPositionWindow_ >=
															  positionWindowTime_);
	return true;
}

template<size_t id>
template<typename ObjectDictionary, typename State>
constexpr void
PositionProtocol<id>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{
	using modm_canopen::SdoErrorCode;

	map.template setReadHandler<PositionObjects::FollowingErrorActualValue>(+[]() {
		if (State::mode_ != OperatingMode::Position) return (int32_t)0;
		return State::scalingFactors_.position.template toUser<int32_t>(positionError_);
	});

	map.template setReadHandler<PositionObjects::PositionDemandValue>(+[]() {
		if (State::mode_ != OperatingMode::Position) return (int32_t)0;
		return State::scalingFactors_.position.template toUser<int32_t>(commandedPosition_);
	});

	map.template setReadHandler<PositionObjects::TargetPosition>(
		+[]() { return State::scalingFactors_.position.template toUser<int32_t>(receivedPosition_); });

	map.template setWriteHandler<PositionObjects::TargetPosition>(+[](int32_t value) {
		receivedPosition_ = State::scalingFactors_.position.template toInternal<int32_t>(value);
		MODM_LOG_DEBUG << "Set received Target Position to " << receivedPosition_ << modm::endl;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<PositionObjects::PositionWindow>(
		+[]() { return State::scalingFactors_.position.template toUser<uint32_t>(positionWindow_); });

	map.template setWriteHandler<PositionObjects::PositionWindow>(+[](uint32_t value) {
		positionWindow_ = State::scalingFactors_.position.template toInternal<uint32_t>(value);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<PositionObjects::PositionPID_kP>(+[](float value) {
		positionPidParameters_.setKp(value);
		positionPid_.setParameter(positionPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<PositionObjects::PositionPID_kI>(+[](float value) {
		positionPidParameters_.setKi(value);
		positionPid_.setParameter(positionPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<PositionObjects::PositionPID_kD>(+[](float value) {
		positionPidParameters_.setKd(value);
		positionPid_.setParameter(positionPidParameters_);
		return SdoErrorCode::NoError;
	});

	map.template setWriteHandler<PositionObjects::PositionPID_MaxErrorSum>(+[](float value) {
		positionPidParameters_.setMaxErrorSum(value);
		positionPid_.setParameter(positionPidParameters_);
		return SdoErrorCode::NoError;
	});
}
