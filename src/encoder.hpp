/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_ENCODER_HPP
#define MOTOR_CAN_ENCODER_HPP

namespace motorCan
{

template< class CommMaster, uint8_t MotorId>
class Encoder
{
public:
	static void
	initialize() {};

	static uint16_t
	getCounterRaw();
};

} // namespace motorCan

#include "encoder_impl.hpp"

#endif // MOTOR_CAN_ENCODER_HPP
