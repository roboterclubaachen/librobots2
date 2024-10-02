#ifndef ENCODER_PROTOCOL_HPP
#define ENCODER_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "encoder_objects.hpp"
#include <chrono>
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/processing/timer.hpp>

template<size_t id, typename EncoderTimer>
class EncoderProtocol
{
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
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

	template<typename Device, typename State, typename MessageCallback>
	static void
	processMessage(const modm::can::Message &, MessageCallback &&)
	{}

private:
	static inline bool firstUpdate = true;
	static inline modm::PreciseDuration lastTimestep{};
	static inline modm::PreciseClock::time_point lastUpdate{};
	static inline uint16_t counter{};
	static inline int16_t delta{};
};

#include "encoder_protocol_impl.hpp"
#endif