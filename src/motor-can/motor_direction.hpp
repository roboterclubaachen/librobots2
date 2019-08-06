/*
 * Copyright (c) 2015, Georgi Grinshpun
 * Copyright (c) 2017, Oliver Lück
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef ROBOTS_MOTOR_DIRECTION_HPP
#define ROBOTS_MOTOR_DIRECTION_HPP

/**
 * Wraps around a Motor.
 * Controls the direction of the motor pwm and the encoder.
 */
template<typename Motor, bool reverseEncoderDirection, bool reversePwmDirection>
class MotorDirection
{
public:
	static inline void
	setPwm(int16_t pwm)
	{
		Motor::setPwm(reversePwmDirection?-pwm:pwm);
	}

	static inline void
	disable()
	{
		Motor::disable();
	}

	static inline void
	setCurrentLimit(uint16_t current)
	{
		Motor::setCurrentLimit(current);
	}

	static inline int16_t
	getEncoderSteps()
	{
		return reverseEncoderDirection?-Motor::getEncoderSteps():Motor::getEncoderSteps();
	}

	static inline int16_t
	getEncoderRaw()
	{
		return reverseEncoderDirection?-Motor::getEncoderRaw():Motor::getEncoderRaw();
	}

	static inline bool
	isCurrentOverLimit()
	{
		return Motor::isCurrentOverLimit();
	}

	static inline uint16_t
	getCurrent()
	{
		return Motor::getCurrent();
	}
};

#endif // ROBOTS_MOTOR_DIRECTION_HPP
