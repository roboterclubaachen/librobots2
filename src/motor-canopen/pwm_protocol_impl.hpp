#ifndef PWM_PROTOCOL_HPP
#error "Do not include this file directly, use pwm_protocol.hpp instead"
#endif
#include <modm/debug/logger.hpp>

template <size_t id>
template <typename Device, typename State, typename MessageCallback>
bool PWMProtocol<id>::update(MessageCallback &&) {
  State::outputPWM_ = commandedPWM_;
  State::status_. template setBit<modm_canopen::cia402::StatusBits::TargetReached>(true);
  return true;
}

template <size_t id>
template <typename ObjectDictionary, typename State>
constexpr void PWMProtocol<id>::registerHandlers(
    modm_canopen::HandlerMap<ObjectDictionary> &map) {
  using modm_canopen::SdoErrorCode;

  map.template setReadHandler<PWMObjects::PWMCommand>(
      +[]() { return commandedPWM_; });

  map.template setWriteHandler<PWMObjects::PWMCommand>(+[](int16_t value) {
    commandedPWM_ = value;
	MODM_LOG_INFO << "Set Target PWM to " << commandedPWM_ << modm::endl;
	return SdoErrorCode::NoError;
  });
}
