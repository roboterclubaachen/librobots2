#ifndef CURRENT_PROTOCOL_HPP
#error "Do not include this file directly, use current_protocol86.hpp instead"
#endif
#include <modm-canopen/cia402/command_bits.hpp>
#include <modm-canopen/cia402/status_bits.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm/debug/logger.hpp>

using CommandBits = modm_canopen::cia402::CommandBits;
using StatusBits = modm_canopen::cia402::StatusBits;
using OperatingMode = modm_canopen::cia402::OperatingMode;

template<size_t id>
template<typename Device, typename State, typename MessageCallback>
bool
CurrentProtocol<id>::update(MessageCallback &&)
{

	const auto [pwm, currentLimit] =
		CurrentControl<id>::template update<Device,State>(targetCurrent_);
	State::outputPWM_ = pwm;
	State::outputCurrentLimit_ = currentLimit;
	return true;
}

template<size_t id>
template<typename ObjectDictionary, typename State>
constexpr void
CurrentProtocol<id>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{

	using modm_canopen::SdoErrorCode;
	map.template setReadHandler<CurrentObjects::CommandedCurrent>(+[]() {
		if (State::mode_ != OperatingMode::Current && State::mode_ != OperatingMode::Velocity &&
			State::mode_ != OperatingMode::Position)
			return 0.0f;
		return CurrentControl<id>::commandedCurrent_;
	});

	map.template setReadHandler<CurrentObjects::FilteredActualCurrent>(+[]() {
		if (State::mode_ != OperatingMode::Current && State::mode_ != OperatingMode::Velocity &&
			State::mode_ != OperatingMode::Position)
			return 0.0f;
		return CurrentControl<id>::filteredActualCurrent_;
	});

	map.template setReadHandler<CurrentObjects::TargetCurrent>(+[]() { return targetCurrent_; });

	map.template setWriteHandler<CurrentObjects::TargetCurrent>(+[](float value) {
		MODM_LOG_DEBUG << "Set Target Current to " << value << modm::endl;
		targetCurrent_ = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<CurrentObjects::CurrentError>(+[]() {
		if (State::mode_ != OperatingMode::Current && State::mode_ != OperatingMode::Velocity &&
			State::mode_ != OperatingMode::Position)
			return 0.0f;
		return CurrentControl<id>::currentError_;
	});

	map.template setWriteHandler<CurrentObjects::DefaultPWM>(+[](int16_t value) {
		CurrentControl<id>::maxPWM_ = value;
		return SdoErrorCode::NoError;
	});

	map.template setReadHandler<CurrentObjects::DefaultPWM>(
		+[]() { return CurrentControl<id>::maxPWM_; });

	map.template setWriteHandler<CurrentObjects::ShouldInvert>(+[](uint8_t value) {
		CurrentControl<id>::inverting_ = (value != 0);
		return SdoErrorCode::NoError;
	});
	map.template setReadHandler<CurrentObjects::ShouldInvert>(
		+[]() { return (uint8_t)CurrentControl<id>::inverting_; });
}
