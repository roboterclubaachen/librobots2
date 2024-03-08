#ifndef POSITION_PROTOCOL_HPP
#error "Do not include this file directly, use position_protocol.hpp instead"
#endif
#include "motor_state.hpp"
#include "velocity_protocol.hpp"
#include <modm/debug/logger.hpp>

template<size_t id>
bool
PositionProtocol<id>::applicable(const MotorState &state)
{
	const auto value = state.mode_ == OperatingMode::Position &&
					   state.status_.state() == modm_canopen::cia402::State::OperationEnabled;
	if (!value)
	{
		positionPid_.reset();
		if (state.mode_ == OperatingMode::Position) { VelocityControl<id>::reset(); }
	}
	return value;
}

template<size_t id>
template<typename Device, typename MessageCallback>
bool
PositionProtocol<id>::update(MotorState &state, MessageCallback &&)
{
	if (state.control_.isSet<CommandBits::NewSetPoint>())
	{
		nextPosition_ = receivedPosition_;
		nextPositionIsNew_ = true;
		state.control_.setBit<CommandBits::NewSetPoint>(false);
		Device::setValueChanged(StateObjects::ControlWord);
	}

	if ((positionError_ == 0 && nextPositionIsNew_) ||
		state.control_.isSet<CommandBits::ChangeImmediately>())
	{
		nextPositionIsNew_ = false;
		if (state.control_.isSet<CommandBits::IsRelative>())
		{
			commandedPosition_ += nextPosition_;
			// MODM_LOG_INFO << "Updated Target Position relative!" << modm::endl;
		} else if (commandedPosition_ != nextPosition_)
		{
			commandedPosition_ = nextPosition_;
			// MODM_LOG_INFO << "Updated Target Position absolute!" << modm::endl;
		}
		Device::setValueChanged(PositionObjects::PositionDemandValue);
		VelocityControl<id>::reset();
	}

	positionError_ = commandedPosition_ - state.actualPosition_;
	Device::setValueChanged(PositionObjects::FollowingErrorActualValue);

	positionPid_.update(positionError_, std::abs(positionPid_.getValue()) > 6000 ||
											VelocityControl<id>::isLimiting_);
	const auto [pwm, currentLimit] =
		VelocityControl<id>::template doVelocityUpdate<Device>(positionPid_.getValue(), state);
	state.outputPWM_ = pwm;
	state.outputCurrentLimit_ = currentLimit;

	Device::setValueChanged(VelocityObjects::VelocityError);
	Device::setValueChanged(VelocityObjects::VelocityDemandValue);

	if ((uint32_t)std::abs(positionError_) <= positionWindow_)
	{
		if (inPositionWindow_ < positionWindowTime_) inPositionWindow_++;
	} else
	{
		inPositionWindow_ = 0;
	}

	state.status_.setBit<StatusBits::TargetReached>(inPositionWindow_ >= positionWindowTime_);
	return true;
}

template<size_t id>
template<typename ObjectDictionary, const MotorState &state>
constexpr void
PositionProtocol<id>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{
	using modm_canopen::SdoErrorCode;

	map.template setReadHandler<PositionObjects::FollowingErrorActualValue>(+[]() {
		if (state.mode_ != OperatingMode::Position) return (int32_t)0;
		return state.scalingFactors_.position.toUser(positionError_);
	});

	map.template setReadHandler<PositionObjects::PositionDemandValue>(+[]() {
		if (state.mode_ != OperatingMode::Position) return (int32_t)0;
		return state.scalingFactors_.position.toUser(commandedPosition_);
	});

	map.template setReadHandler<PositionObjects::TargetPosition>(
		+[]() { return state.scalingFactors_.position.toUser(receivedPosition_); });

	map.template setWriteHandler<PositionObjects::TargetPosition>(+[](int32_t value) {
		receivedPosition_ = state.scalingFactors_.position.toInternal(value);
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<PositionObjects::PositionWindow>(
		+[]() { return state.scalingFactors_.position.toUser(positionWindow_); });

	map.template setWriteHandler<PositionObjects::PositionWindow>(+[](uint32_t value) {
		positionWindow_ = state.scalingFactors_.position.toInternal(value);
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
