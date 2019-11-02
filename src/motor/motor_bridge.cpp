/* motor_bridge.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
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
