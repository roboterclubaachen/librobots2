/* alpha_motor.hpp
 *
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

namespace motorCan
{

/**
 * Encapsulate the motor access into this template class.
 * Instantiate this template class for each motor with the
 * matching MotorId.
 *
 * Example:
 *   using MyMotor = Motor<MyIMotorCommunicationMaster, 0>;
 *
 * Then it is quite easy to access the motors:
 *   MyMotor::setPwm(+100);
 */
template< class CommMaster, uint8_t MotorId>
class Motor
{
public:
	static void
	initialize();

	static void
	run();

	enum struct MotorMode {
		MotorBreak,
		Pwm,
		Current,
		PwmWithCurrentLimit,
	}

	static void
	setMode(MotorMode m);

	static void
	setDirection(bool direction);

	static void
	setPwm(uint16_t pwm);

	static void
	setCurrent(uint16_t current);

	static bool
	isCurrentOverLimit();

	static uint16_t
	getCurrent();

};

} // namespace motorCan

#endif // MOTOR_HPP
