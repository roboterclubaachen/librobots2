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

struct Identity
{
	DeviceType deviceType_;
	uint8_t errorRegister_ = 0;
	uint8_t identityObject_ = 4;
	uint32_t vendorId_ = 0xdeadbeef;
	ProductCode productCode_;
	uint32_t revisionId_ = 1;
	uint32_t serialNumber_ = 1;
};