#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

struct EncoderObjects
{
	static constexpr modm_canopen::Address EncoderValue{0x2008, 1};
	static constexpr modm_canopen::Address EncoderTimestep{0x2008, 2};
	static constexpr modm_canopen::Address EncoderDelta{0x2008, 3};
	static constexpr modm_canopen::Address EncoderOverrun{0x2008, 4};
	static constexpr modm_canopen::Address IndexGating{0x2008, 5};
	static constexpr modm_canopen::Address EncoderFactorNumerator{0x2022, 1};
	static constexpr modm_canopen::Address EncoderFactorDivisor{0x2022, 2};
};