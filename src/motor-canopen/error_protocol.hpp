#ifndef ERROR_PROTOCOL_HPP
#define ERROR_PROTOCOL_HPP

#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/math/filter/moving_average.hpp>

using namespace std::literals;

template<size_t id>
class ErrorProtocol
{
private:
	static inline modm::chrono::micro_clock::duration errorCounter_ = 0ms;
	static constexpr modm::chrono::micro_clock::duration maxErrorTime_ = 512ms;
	static constexpr int32_t stallVelocity_ = 10;

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