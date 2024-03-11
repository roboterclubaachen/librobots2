#ifndef STATE_BASE_HPP
#error "Do not include this file directly, use state_base.hpp instead"
#endif

template<typename Device, typename MessageCallback>
bool
StateBase::update(MessageCallback &&)
{
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
StateBase::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{
	using modm_canopen::SdoErrorCode;

	map.template setReadHandler<StateObjects::ModeOfOperation>(
		+[]() { return int8_t(mode_); });

	map.template setReadHandler<StateObjects::ModeOfOperationDisplay>(
		+[]() { return int8_t(mode_); });

	map.template setWriteHandler<StateObjects::ModeOfOperation>(+[](int8_t value) {
        static_assert(false, "Check which modes you actually support!");
		//TODO Generate this from which modes are enabled
		const bool valid = (value == int8_t(OperatingMode::Disabled)) ||
						   (value == int8_t(OperatingMode::Voltage)) ||
						   (value == int8_t(OperatingMode::Velocity)) ||
						   (value == int8_t(OperatingMode::Position)) ||
						   (value == int8_t(OperatingMode::Current));

		if (valid)
		{
			mode_ = (static_cast<OperatingMode>(value));
			return SdoErrorCode::NoError;
		} else
		{
			return SdoErrorCode::InvalidValue;
		}
	});

	map.template setReadHandler<StateObjects::ControlWord>(
		+[]() { return control_.value(); });

	map.template setWriteHandler<StateObjects::ControlWord>(+[](uint16_t value) {
		control_.update(value);
		status_.update(control_);
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<StateObjects::StatusWord>(
		+[]() { return status_.status(); });
}