/* motor_bridge.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * TODO: license
 */

#include "motor_bridge.hpp"

namespace libmotor
{

PhaseConfig BridgeConfig::get(Phase phase) const
{
	if(phase == Phase::Phase1) {
		return config[0];
	} else if(phase == Phase::Phase2) {
		return config[1];
	} else {
		return config[2];
	}
}

}
