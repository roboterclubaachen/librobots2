#ifndef STATE_BASE_HPP
#define STATE_BASE_HPP
#error "Do not use this file directly, without modifying it first!"

#include <modm-canopen/handler_map.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>

#include "identity.hpp"

struct StateBase
{
	static inline Identity identity_{};

	static inline modm_canopen::cia402::OperatingMode mode_{modm_canopen::cia402::OperatingMode::Disabled};
	static inline modm_canopen::cia402::StateMachine status_{modm_canopen::cia402::State::SwitchOnDisabled};
	static inline modm_canopen::cia402::CommandWord control_{0};

	static inline uint16_t outputPWM_{};

	template<typename Device, typename MessageCallback>
	static bool
	update(MessageCallback &&cb);

	template<typename ObjectDictionary>
	static constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);
};

#include "state_base_impl.hpp"
#endif