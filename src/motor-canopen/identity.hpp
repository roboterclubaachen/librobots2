#pragma once
#include <cstdint>

enum class DeviceType : uint32_t
{
	BLDC = 412,
	Servo = 413,
};

enum class ProductCode : uint32_t
{
	MicroMotor = 0,
	Can2x = 1,
	AlphaMotor = 2,
};