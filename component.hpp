/*
 * Copyright (c) 2015, Kevin Läufer
 * Copyright (c) 2016, Hauke Biss
 * Copyright (c) 2018, Niklas Hauser
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef ROBOT_COMPONENT_HPP
#define	ROBOT_COMPONENT_HPP

#include <modm/communication/xpcc/abstract_component.hpp>

#include "communication/packets.hpp"

namespace robot
{
class Component : public xpcc::AbstractComponent
{
public:
	Component(uint8_t ownIdentifier, xpcc::Dispatcher *communication);

	void
	actionPing(const xpcc::ResponseHandle& handler);

	void
	actionAwake(
			const xpcc::ResponseHandle& handler);
};
}

#endif	// ROBOT_COMPONENT_HPP
