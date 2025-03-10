#pragma once
#include "pin.hpp"
namespace librobots2::motor_sim
{

template<typename... Pins>
class GPIOPort
{
private:
	template<typename Pin1, typename Pin2, typename Pin3>
	static inline uint8_t
	subRead()
	{
		return Pin1::read() + (Pin2::read() << 1) + (Pin3::read() << 2);
	}

public:
	static inline uint8_t
	read()
	{
		return subRead<Pins...>();
	};
};

}  // namespace librobots2::motor_sim