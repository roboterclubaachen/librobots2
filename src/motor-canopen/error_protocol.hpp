#ifndef ERROR_PROTOCOL_HPP
#define ERROR_PROTOCOL_HPP

#include "motor_state.hpp"

#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/math/filter/moving_average.hpp>

using namespace std::literals;

template<size_t id>
class ErrorProtocol
{
private:
	static inline int16_t lastPWM_{};
	static inline modm::filter::MovingAverage<int16_t, 16> pwmChange_;
	static inline uint32_t errorCounter_ = 0;
	static constexpr uint32_t maxErrorTime_ = 512;
	static constexpr int32_t stallVelocity_ = 10;
	static constexpr int16_t pwmWindow_ = 5;

public:
	template<typename State>
	static bool
	applicable()
	{
		return true;
	}

	template<typename Device, typename State, typename MessageCallback>
	static bool
	update(MessageCallback &&cb);

	template<typename ObjectDictionary, typename State>
	static constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &)
	{}

	template<typename Device, typename State, typename MessageCallback>
	static void
	processMessage(const modm::can::Message &, MessageCallback &&)
	{}
};

#include "error_protocol_impl.hpp"
#endif