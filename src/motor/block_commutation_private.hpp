/* block_commutation.hpp
 *
 * Copyright (C) 2018 Raphael Lehmann
 * Copyright (C) 2019 Christopher Durand
 *
 * TODO: license
 */

#ifndef LIBMOTOR_BLOCK_COMMUTATION_PRIVATE_HPP
#define LIBMOTOR_BLOCK_COMMUTATION_PRIVATE_HPP

#include <array>
#include <cstdint>
#include "motor_bridge.hpp"

// Internal definitions
namespace libmotor::block_commutation
{

// Lookup table for commutation sequence:
// ..., 1, 3, 2, 6, 4, 5, ...
constexpr inline std::array<uint_fast8_t, 8> SequenceLut = {
	6, // invalid
	0, // 0b001
	2, // 0b010
	1, // 0b011
	4, // 0b100
	5, // 0b101
	3, // 0b110
	6, // invalid
};

constexpr inline auto Pwm = PhaseConfig::Pwm;
constexpr inline auto Low = PhaseConfig::Low;
constexpr inline auto HiZ = PhaseConfig::HiZ;

constexpr inline std::array<BridgeConfig, 8> CommutationLut {{
	// 6 bridge states for block commutation
	{ Pwm, Low, HiZ }, // hall 0b001 -> 0
	{ Pwm, HiZ, Low }, // hall 0b011 -> 1
	{ HiZ, Pwm, Low }, // hall 0b010 -> 2
	{ Low, Pwm, HiZ }, // hall 0b110 -> 3
	{ Low, HiZ, Pwm }, // hall 0b100 -> 4
	{ HiZ, Low, Pwm }, // hall 0b101 -> 5

	// Motor disabled state (HiZ, HiZ, HiZ)
	{ HiZ, HiZ, HiZ }, // 6

	// Motor brake state (Low, Low, Low)
	{ Low, Low, Low }  // 7
}};

}

#endif // LIBMOTOR_BLOCK_COMMUTATION_PRIVATE
