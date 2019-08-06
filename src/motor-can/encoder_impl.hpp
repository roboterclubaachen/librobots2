/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_ENCODER_HPP
#error "Do not include this file directly. Include encoder.hpp instead."
#endif

namespace motorCan
{

template< class CommMaster, uint8_t MotorId>
uint16_t
Encoder<CommMaster, MotorId>::getCounterRaw()
{
	// Access to _encoder member must be locked
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		return CommMaster::dataRx[MotorId / 2].encoderCounterRawM1;
	}
	else {
		return CommMaster::dataRx[MotorId / 2].encoderCounterRawM2;
	}
}

} // namespace motorCan
