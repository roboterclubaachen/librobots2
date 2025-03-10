#pragma once
#include <array>
#include <cstdint>

#include "motor_state.hpp"
#include "gpio_port.hpp"

#include <librobots2/motor/motor_bridge.hpp>

namespace librobots2::motor_sim
{
using librobots2::motor::BridgeConfig;
using librobots2::motor::Phase;
using librobots2::motor::PhaseConfig;

class MotorBridge
{
private:
	static uint8_t
	toIndex(Phase p);

public:
	using HallPort = GPIOPort<Pin<0>, Pin<1>, Pin<2>>;

	static constexpr uint16_t MaxPwm = 2047;

	static void
	initialize();

	static void
	configure(const BridgeConfig& config);

	static void
	configure(PhaseConfig config);

	static void
	configure(Phase phase, PhaseConfig config);

	static void
	setCompareValue(uint16_t compareValue);

	static void
	setCompareValue(Phase phase, uint16_t compareValue);

	static void
	applyCompareValues();

	static const std::array<PhaseConfig, 3>
	getConfig();

	static const std::array<float, 3>
	getPWMs();

private:
	static inline std::array<PhaseConfig, 3> phaseConfig{PhaseConfig::HiZ, PhaseConfig::HiZ,
														 PhaseConfig::HiZ};
	static inline std::array<uint16_t, 3> pwms{};
};

}  // namespace librobots2::motor_sim