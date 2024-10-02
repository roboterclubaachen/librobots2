#ifndef ENCODER_PROTOCOL_HPP
#error "Do not include this file directly, use encoder_protocol.hpp instead"
#endif

#include <modm/debug/logger.hpp>

template<size_t id, typename EncoderTimer>
template<typename Device, typename State, typename MessageCallback>
bool
EncoderProtocol<id, EncoderTimer>::update(MessageCallback &&)
{

	auto now = modm::PreciseClock::now();
	if (firstUpdate)
	{
		firstUpdate = false;
	} else
	{
		lastTimestep = now - lastUpdate;
	}
	lastUpdate = now;

	auto newCounter = EncoderTimer::getEncoderRaw();
	delta = newCounter - counter;
	counter = newCounter;
	/*if (delta != 0) {
	  MODM_LOG_DEBUG << "Encoder " << id << ": " << counter
					 << "\nDelta: " << delta
					 << "\nTimestep: " << lastTimestep.count() << "us"
					 << modm::endl;
	}*/
	return true;
}
template<size_t id, typename EncoderTimer>
template<typename ObjectDictionary, typename State>
constexpr void
EncoderProtocol<id, EncoderTimer>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{
	using modm_canopen::SdoErrorCode;
	map.template setReadHandler<EncoderObjects::EncoderValue>(+[]() { return counter; });
	map.template setReadHandler<EncoderObjects::EncoderTimestep>(
		+[]() { return lastTimestep.count(); });
	map.template setReadHandler<EncoderObjects::EncoderDelta>(+[]() { return delta; });
	map.template setWriteHandler<EncoderObjects::EncoderOverrun>(+[](uint16_t value) {
		EncoderTimer::setOverflow(value);
		return SdoErrorCode::NoError;
	});
	map.template setWriteHandler<EncoderObjects::IndexGating>(+[](uint8_t value) {
		if (EncoderTimer::setGating(value))
			return SdoErrorCode::NoError;
		else
			return SdoErrorCode::InvalidValue;
	});
}
