#ifndef QUICKSTOP_PROTOCOL_HPP
#error "Do not include this file directly, use quickstop_protocol.hpp instead"
#endif

template<size_t id>
template<typename Device, typename State, typename MessageCallback>
bool
QuickstopProtocol<id>::update(MessageCallback &&)
{
	if (State::status_.state() == modm_canopen::cia402::State::QuickStopActive)
	{
		const auto [pwm, currentLimit] = VelocityControl<id>::template doDecelerationUpdate<Device,State>(
			quickStopDeceleration_);
		State::outputPWM_ = pwm;
		State::outputCurrentLimit_ = currentLimit;
		Device::setValueChanged(VelocityObjects::VelocityDemandValue);
		Device::setValueChanged(VelocityObjects::VelocityError);
		State::status_. template setBit<modm_canopen::cia402::StatusBits::NotCurrentlyQuickStopping>(false);
	} else
	{
		State::status_. template setBit<modm_canopen::cia402::StatusBits::NotCurrentlyQuickStopping>(true);
	}
	return true;
}

template<size_t id>
template<typename ObjectDictionary, typename State>
constexpr void
QuickstopProtocol<id>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{
	using modm_canopen::SdoErrorCode;
	map.template setReadHandler<QuickStopObjects::QuickStopDeceleration>(
		+[]() { return State::scalingFactors_.acceleration.template toUser<int32_t>(quickStopDeceleration_); });

	map.template setWriteHandler<QuickStopObjects::QuickStopDeceleration>(+[](int32_t value) {
		quickStopDeceleration_ = State::scalingFactors_.acceleration.template toInternal<int32_t>(value);
		return SdoErrorCode::NoError;
	});
}