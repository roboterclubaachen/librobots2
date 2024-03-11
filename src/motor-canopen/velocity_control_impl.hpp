#ifndef VELOCITY_CONTROL_HPP
#error "Do not include this file directly, use velocity_control.hpp instead"
#endif

template<size_t id>
template<typename Device, typename State>
std::tuple<int16_t, float>
VelocityControl<id>::doVelocityUpdate(int32_t inVelocity)
{

	commandedVel_ = inVelocity;
	if (std::signbit(commandedVel_) != std::signbit(State::actualVelocity_.getValue()) &&
		(State::actualVelocity_.getValue() != 0 && commandedVel_ != 0))
	{
		reset();
	}
	if ((std::signbit(commandedVel_) == std::signbit(State::actualVelocity_.getValue())) ||
		(State::actualVelocity_.getValue() == 0 && commandedVel_ != 0))
	{
		isLimiting_ = State::outputPWM_ > profileAcceleration_ || CurrentControl<id>::isLimiting_;
		velocityError_ = commandedVel_ - State::actualVelocity_.getValue();
		velocityPid_.update(velocityError_, isLimiting_);
		Device::setValueChanged(VelocityObjects::VelocityError);
		return CurrentControl<id>::template update<Device,State>(velocityPid_.getValue());
	}
	return {0, 0.0};
}

template<size_t id>
template<typename Device, typename State>
std::tuple<int16_t, float>
VelocityControl<id>::doDecelerationUpdate(int32_t commandedDeceleration)
{
	commandedVel_ = 0;
	velocityError_ = -State::actualVelocity_.getValue();
	isLimiting_ = State::outputPWM_ > commandedDeceleration || CurrentControl<id>::isLimiting_;
	velocityPid_.update(velocityError_, isLimiting_);
	Device::setValueChanged(VelocityObjects::VelocityError);
	auto [pwm, currentLimit] =
		CurrentControl<id>::template update<Device,State>(velocityPid_.getValue());
	return {(int16_t)std::clamp((int32_t)pwm, -commandedDeceleration, commandedDeceleration),
			currentLimit};
}

template<size_t id>
template<typename State>
void
VelocityControl<id>::resetIfApplicable()
{
	if (!State::enableMotor_ ||
		State::status_.state() != modm_canopen::cia402::State::OperationEnabled ||
		State::mode_ == OperatingMode::Disabled || State::mode_ == OperatingMode::Voltage)
	{
		reset();
	}
}

template<size_t id>
void
VelocityControl<id>::reset()
{
	velocityPid_.reset();
	CurrentControl<id>::reset();
}