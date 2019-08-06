/* motor_disabled.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * TODO: license
 */

#ifndef LIBMOTOR_MOTOR_DISABLED_HPP
#define LIBMOTOR_MOTOR_DISABLED_HPP

#include <cstdint>

#include "motor_base.hpp"
#include "motor_bridge.hpp"

namespace libmotor
{

/// Dummy motor which disables all outputs
template<typename MotorBridge>
class MotorDisabled final : public MotorInterface
{
public:
	MotorDisabled() { disableMotors(); }

	~MotorDisabled() { disableMotors(); }

	inline void setPwm([[maybe_unused]] int16_t pwm) override {};

	inline void disable() override { disableMotors(); };

	inline void update() override {}

private:
	void disableMotors()
	{
		MotorBridge::configure(PhaseConfig::HiZ);
		MotorBridge::setCompareValue(0);
	}
};

}

#endif // LIBMOTOR_MOTOR_DISABLED_HPP
