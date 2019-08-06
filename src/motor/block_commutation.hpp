/* block_commutation.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * TODO: license
 */

#ifndef LIBMOTOR_BLOCK_COMMUTATION_HPP
#define LIBMOTOR_BLOCK_COMMUTATION_HPP

#include <cstdint>

#include "motor_bridge.hpp"

namespace libmotor
{

/// Implements block commutations with hall sensors
/// \tparam HallPort A modm::SoftwareGpioPort or compatible class with
///		3 hall pins as template parameters
template<typename HallPort>
class BlockCommutation
{
public:
	static BridgeConfig
	doCommutation(uint_fast8_t commutationOffset, bool reverse);

	static BridgeConfig
	disable();

	static BridgeConfig
	motorBrake();

private:
	//BlockCommutation() = delete;
};

}

#include "block_commutation_impl.hpp"

#endif // LIBMOTOR_BLOCK_COMMUTATION_HPP
