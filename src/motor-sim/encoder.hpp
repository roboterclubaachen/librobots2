#pragma once
#include <cstdint>

namespace librobots2::motor_sim
{
class Encoder
{
public:
	static uint16_t
	getEncoderRaw();

	static void
	initialize();

	static bool
	setGating(uint8_t value);

	static bool
	hasSeenIndex();
};

}  // namespace librobots2::motor_sim
