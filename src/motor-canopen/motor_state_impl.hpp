#ifndef MOTOR_STATE_HPP
#error "Do not include this file directly, use motor_state.hpp instead"
#endif

template<size_t id>
inline float
MotorState<id>::getCharge()
{
	float acc = 0.0f;
	for (auto &pair : currentValues_) { acc += pair.first * pair.second; }
	return acc;
}

template<size_t id>
inline void
MotorState<id>::setActualPosition(int32_t position)
{
	actualPosition_ = position;
}

template<size_t id>
inline void
MotorState<id>::setOrientedCurrent(float current)
{
	orientedCurrent_ = current;
}

template<size_t id>
inline void
MotorState<id>::setOrientedCurrentAngleDiff(float angle)
{
	orientedCurrentAngleDiff_ = angle;
}

template<size_t id>
inline void
MotorState<id>::setUnorientedCurrent(float current)
{

	unorientedCurrent_ = current;
}

template<size_t id>
inline int16_t
MotorState<id>::outputPWM()
{
	return outputPWM_;
}

template<size_t id>
inline float
MotorState<id>::currentLimit()
{
	return outputCurrentLimit_;
}

template<size_t id>
inline float
MotorState<id>::maxCurrent()
{
	return maxCurrent_;
}

template<size_t id>
template<typename Device, typename MessageCallback>
bool
MotorState<id>::update(MessageCallback &&)
{
	const auto now = modm::chrono::micro_clock::now();
	lastUpdateTime_ = now - lastUpdate_;
	const auto lastUpdateTime_us = lastUpdateTime_.count();
	const auto lastUpdateTime_s = (float)lastUpdateTime_us / 1000000.0f;
	updateTime_us_.update(lastUpdateTime_us);
	lastUpdate_ = now;
	Device::setValueChanged(StateObjects::UpdateTime);

	if (outputPWM_ == 0 && std::abs(actualVelocity_.getValue()) <= 0.001f)
	{
		if (zeroAverageCountdown_ == 0)
		{
			zeroAverage_.update(unorientedCurrent_);
		} else
		{
			zeroAverageCountdown_--;
		}
	} else
	{
		zeroAverageCountdown_ = zeroAverageCountdownReset_;
	}

	// Calculate remaining charge
	currentValues_.appendOverwrite({std::abs(unorientedCurrent_), lastUpdateTime_s});
	currentCharge_ = getCharge();
	Device::setValueChanged(StateObjects::CurrentCharge);

	const auto newVelocity_ = (float)(actualPosition_ - lastPosition_) * (1 << 12) * 1000.0f *
							  1000.0f / (float)lastUpdateTime_us;
	lastPosition_ = actualPosition_;
	actualVelocity_.update(
		(int32_t)newVelocity_);  // Increase velocity resolution for better regulation

	Device::setValueChanged(StateObjects::VelocityActualValue);
	Device::setValueChanged(StateObjects::PositionInternalValue);
	Device::setValueChanged(StateObjects::PositionActualValue);
	Device::setValueChanged(StateObjects::OrientedCurrentAngleDiff);
	Device::setValueChanged(StateObjects::UnorientedCurrent);
	Device::setValueChanged(StateObjects::OrientedCurrent);

	bool value = false;
	if (status_.state() != modm_canopen::cia402::State::OperationEnabled ||
		mode_ == OperatingMode::Disabled)
	{
		enableMotor_ = false;
		outputPWM_ = 0;
		value = true;
	} else
	{
		enableMotor_ = true;
	}
	status_.setBit<StatusBits::VoltagePresent>(outputPWM_ != 0);
	Device::setValueChanged(StateObjects::OutputPWM);
	Device::setValueChanged(StateObjects::StatusWord);
	Device::setValueChanged(StateObjects::ModeOfOperationDisplay);  // TODO move to where this
																	// actually happens
	return value;
}

template<size_t id>
template<typename ObjectDictionary>
constexpr void
MotorState<id>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{
	using modm_canopen::SdoErrorCode;

	map.template setWriteHandler<StateObjects::Reset>(+[](int8_t value) {
		resetMotor_ = (value != 0);
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::UpdateTime>(
		+[]() { return uint32_t(updateTime_us_.getValue()); });

	map.template setReadHandler<StateObjects::ModeOfOperation>(+[]() { return int8_t(mode_); });

	map.template setReadHandler<StateObjects::ModeOfOperationDisplay>(
		+[]() { return int8_t(mode_); });

	map.template setWriteHandler<StateObjects::ModeOfOperation>(+[](int8_t value) {
		const bool valid = (value == int8_t(OperatingMode::Disabled)) ||
						   (value == int8_t(OperatingMode::Voltage)) ||
						   (value == int8_t(OperatingMode::Velocity)) ||
						   (value == int8_t(OperatingMode::Position)) ||
						   (value == int8_t(OperatingMode::Current));

		if (valid)
		{
			auto newMode = (static_cast<OperatingMode>(value));
			if (mode_ != newMode)
			{
				mode_ = newMode;
				MODM_LOG_INFO << "Set operating mode to "
							  << modm_canopen::cia402::operatingModeToString(mode_) << modm::endl;
			}
			return SdoErrorCode::NoError;
		} else
		{
			return SdoErrorCode::InvalidValue;
		}
	});

	map.template setReadHandler<StateObjects::ControlWord>(+[]() { return control_.value(); });

	map.template setWriteHandler<StateObjects::ControlWord>(+[](uint16_t value) {
		control_.update(value);
		status_.update(control_);
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::StatusWord>(+[]() { return status_.status(); });

	map.template setReadHandler<StateObjects::PositionActualValue>(
		+[]() { return scalingFactors_.position.toUser(actualPosition_); });

	map.template setReadHandler<StateObjects::PositionInternalValue>(
		+[]() { return actualPosition_; });

	map.template setReadHandler<StateObjects::VelocityActualValue>(
		+[]() { return scalingFactors_.velocity.toUser(actualVelocity_.getValue()); });

	map.template setReadHandler<StateObjects::OutputPWM>(+[]() { return outputPWM_; });

	map.template setReadHandler<StateObjects::UnorientedCurrent>(
		+[]() { return unorientedCurrent_; });

	map.template setReadHandler<StateObjects::PositionFactorNumerator>(
		+[]() { return scalingFactors_.position.numerator; });

	map.template setWriteHandler<StateObjects::PositionFactorNumerator>(+[](uint32_t value) {
		scalingFactors_.position.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::PositionFactorDivisor>(
		+[]() { return scalingFactors_.position.divisor; });

	map.template setWriteHandler<StateObjects::PositionFactorDivisor>(+[](uint32_t value) {
		scalingFactors_.position.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::VelocityFactorNumerator>(
		+[]() { return scalingFactors_.velocity.numerator; });

	map.template setWriteHandler<StateObjects::VelocityFactorNumerator>(+[](uint32_t value) {
		scalingFactors_.velocity.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::VelocityFactorDivisor>(
		+[]() { return scalingFactors_.velocity.divisor; });

	map.template setWriteHandler<StateObjects::VelocityFactorDivisor>(+[](uint32_t value) {
		scalingFactors_.velocity.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::AccelerationFactorNumerator>(
		+[]() { return scalingFactors_.acceleration.numerator; });

	map.template setWriteHandler<StateObjects::AccelerationFactorNumerator>(+[](uint32_t value) {
		scalingFactors_.acceleration.numerator = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::AccelerationFactorDivisor>(
		+[]() { return scalingFactors_.acceleration.divisor; });

	map.template setWriteHandler<StateObjects::AccelerationFactorDivisor>(+[](uint32_t value) {
		scalingFactors_.acceleration.divisor = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::Polarity>(
		+[]() { return scalingFactors_.getPolarity(); });

	map.template setWriteHandler<StateObjects::Polarity>(+[](uint8_t value) {
		scalingFactors_.setPolarity(value);
		return SdoErrorCode::NoError;
	});
	map.template setReadHandler<StateObjects::MaxCurrent>(+[]() { return maxCurrent_; });

	map.template setWriteHandler<StateObjects::MaxCurrent>(+[](float value) {
		maxCurrent_ = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::MaxCharge>(+[]() { return maxCharge_; });

	map.template setWriteHandler<StateObjects::MaxCharge>(+[](float value) {
		maxCharge_ = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::OrientedCurrent>(+[]() { return orientedCurrent_; });

	map.template setReadHandler<StateObjects::OrientedCurrentAngleDiff>(
		+[]() { return orientedCurrentAngleDiff_; });

	map.template setReadHandler<StateObjects::CurrentCharge>(+[]() { return currentCharge_; });
}