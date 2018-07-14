/*
 * Copyright (c) 2015, Kevin Läufer
 * Copyright (c) 2016, Hauke Biss
 * Copyright (c) 2016, Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "component.hpp"

// ----------------------------------------------------------------------------
robot::Component::Component(uint8_t ownIdentifier, xpcc::Dispatcher *communication) : 
	xpcc::AbstractComponent(ownIdentifier, *communication)
{
}

// ----------------------------------------------------------------------------
void
robot::Component::actionPing(const xpcc::ResponseHandle& handler)
{
	this->sendNegativeResponse(handler);
}

// ----------------------------------------------------------------------------
void
robot::Component::actionAwake(const xpcc::ResponseHandle& handler)
{
	this->sendNegativeResponse(handler);
}
