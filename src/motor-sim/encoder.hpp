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

	static void
	setOverflow(uint16_t val);

private:
	static inline uint16_t overflow_{2048};
};

}  // namespace librobots2::motor_sim
