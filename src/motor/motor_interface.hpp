/* motor_interface.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * TODO: license
 */

#ifndef LIBMOTOR_MOTOR_INTERFACE_HPP
#define LIBMOTOR_MOTOR_INTERFACE_HPP

namespace libmotor
{

class MotorInterface
{
public:
	virtual ~MotorInterface() = default;

	/// Set pwm value in range of -32767...32767
	virtual void setPwm(int16_t pwm) = 0;

	virtual void disable() = 0;

	virtual void update() = 0;
};

}

#endif // LIBMOTOR_MOTOR_INTERFACE_HPP
