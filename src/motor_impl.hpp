/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_MOTOR_HPP
#error "Do not include this file directly. Include motor.hpp instead."
#endif

#include <limits>

namespace motorCan
{
template< class CommMaster, uint8_t MotorId>
void
Motor<CommMaster, MotorId>::disable()
{
	// Access to _pwm member must be locked
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		CommMaster::dataTx[MotorId / 2].pwmM1 = std::numeric_limits<int16_t>::min();
	}
	else {
		CommMaster::dataTx[MotorId / 2].pwmM2 = std::numeric_limits<int16_t>::min();
	}
}

template< class CommMaster, uint8_t MotorId>
void
Motor<CommMaster, MotorId>::setPwm(int16_t pwm)
{
	// Access to _pwm member must be locked
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		CommMaster::dataTx[MotorId / 2].pwmM1 = pwm;
	}
	else {
		CommMaster::dataTx[MotorId / 2].pwmM2 = pwm;
	}
}

template< class CommMaster, uint8_t MotorId>
void
Motor<CommMaster, MotorId>::setCurrent(uint16_t current)
{
	// Access to _currentLimit member must be locked
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		CommMaster::dataTx[MotorId / 2].currentLimitM1 = current;
	}
	else {
		CommMaster::dataTx[MotorId / 2].currentLimitM2 = current;
	}
}

template< class CommMaster, uint8_t MotorId>
bool
Motor<CommMaster, MotorId>::isCurrentOverLimit()
{
	return false; // alpha motor never reaches current limit because its over 9000
}

template< class CommMaster, uint8_t MotorId>
uint16_t
Motor<CommMaster, MotorId>::getCurrent()
{
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		return CommMaster::dataRx[MotorId / 2].currentM1;
	}
	else {
		return CommMaster::dataRx[MotorId / 2].currentM2;
	}
}

template< class CommMaster, uint8_t MotorId>
uint16_t
Motor<CommMaster, MotorId>::getEncoderSteps()
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
