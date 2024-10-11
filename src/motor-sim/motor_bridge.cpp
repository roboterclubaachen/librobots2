#include "motor_bridge.hpp"

#include <modm/debug/logger.hpp>
#include <algorithm>
namespace librobots2::motor_sim
{

uint8_t
MotorBridge::toIndex(Phase p)
{
	switch (p)
	{
		case Phase::PhaseU:
			return 0;
		case Phase::PhaseV:
			return 1;
		case Phase::PhaseW:
			return 2;
		default:
			return 0;
	}
}

void
MotorBridge::initialize()
{
	phaseConfig = {PhaseConfig::HiZ, PhaseConfig::HiZ, PhaseConfig::HiZ};
	pwms = {};
}

void
MotorBridge::configure(const BridgeConfig& config)
{
	phaseConfig = config.config;
}

void
MotorBridge::configure(PhaseConfig config)
{
	for (auto& p : phaseConfig) { p = config; }
}

void
MotorBridge::configure(Phase phase, PhaseConfig config)
{
	auto index = toIndex(phase);
	phaseConfig[index] = config;
}

void
MotorBridge::setCompareValue(uint16_t compareValue)
{
	for (auto& pwm : pwms) { pwm = compareValue; }
}

void
MotorBridge::setCompareValue(Phase phase, uint16_t compareValue)
{
	auto index = toIndex(phase);
	pwms[index] = compareValue;
}

void
MotorBridge::applyCompareValues()
{}

const std::array<PhaseConfig, 3>
MotorBridge::getConfig()
{
	return phaseConfig;
}

const std::array<float, 3>
MotorBridge::getPWMs()
{
	return {std::clamp((float)pwms[0] / MaxPwm, 0.0f, 1.0f),
			std::clamp((float)pwms[1] / MaxPwm, 0.0f, 1.0f),
			std::clamp((float)pwms[2] / MaxPwm, 0.0f, 1.0f)};
}

}  // namespace librobots2::motor_sim