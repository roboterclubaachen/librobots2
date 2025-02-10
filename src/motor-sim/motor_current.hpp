#pragma once
#include <cstdlib>
#include <algorithm>
#include <modm/architecture/interface/register.hpp>
namespace librobots2::motor_sim
{

template<size_t ID>
class SimADC
{
protected:
	constexpr static float ShuntResistance = 5e-3;
	constexpr static float CurrentGain = 50;
	constexpr static float ReferenceVoltage = 2.9;
	constexpr static uint16_t AdcCounts = (1 << 12) - 1;

	constexpr static uint16_t
	convert(float current)
	{
		const float adcVoltage =
			(current * CurrentGain * ShuntResistance) + (ReferenceVoltage / 2.0f);
		const float adcValue = adcVoltage / (ReferenceVoltage / AdcCounts);
		return (uint16_t)std::clamp(adcValue, 0.0f, 4095.0f);
	}

	friend class MotorSimulation;
	static inline uint16_t value_{0x7ff};
	static inline void
	setValue(float current)
	{
		value_ = convert(current);
	}

public:
	enum class InterruptFlag : uint32_t
	{
		EndOfRegularConversion,
		EndOfSampling,
		Overrun,
	};

	MODM_FLAGS32(InterruptFlag);

	static void acknowledgeInterruptFlags(InterruptFlag_t){

	};
	static uint16_t
	getValue()
	{
		return value_;
	}
};

class MotorCurrent
{
public:
	using AdcU = SimADC<0>;
	using AdcV = SimADC<1>;
	using AdcW = SimADC<2>;
};
}  // namespace librobots2::motor_sim