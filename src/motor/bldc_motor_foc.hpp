/* bldc_motor_foc.hpp
*
* Copyright (C) 2021 Christopher Durand
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef LIBMOTOR_BLDC_MOTOR_FOC_HPP
#define LIBMOTOR_BLDC_MOTOR_FOC_HPP

#include <cstdint>
#include <modm/math/filter/pid.hpp>
#ifndef MODM_OS_HOSTED
#include <cmsis/dsp/arm_math.h>
#else
#include <cmath>
#endif

#include "motor_base.hpp"
#include "motor_bridge.hpp"

namespace librobots2::motor
{

/// Control of a BLDC motor with FOC
template<typename MotorBridge>
class BldcMotorFoc final : public MotorBase<MotorBridge>
{
public:
    using Pid = modm::Pid<float>;
    using PidParameters = modm::Pid<float>::Parameter;

    BldcMotorFoc();

    /// Disables the motor on destruction
    ~BldcMotorFoc();

    /// Set torque current command (q-Axis current) in mA
    void setSetpoint(int16_t current) override;

    /// Run control algorithm and set output duty-cycle
    void update() override;

    /// Disable motor operation, set phases to HiZ
    void disable() override;

    void enable();

    /// Set flux current command (d-Axis current) in mA
    void setFluxCurrentSetpoint(int16_t current);

    /// Set d/q current controller pid parameters
    void setControllerParameters(const PidParameters& parameters);

    /**
    * Set motor current in alpha/beta coordinates
    * Has to be set before running the control algorithm
    */
    void setCurrentMeasurement(float alpha, float beta);

    /**
    * Set electrical motor angle in degrees
    * Has to be set before running the control algorithm
    */
    void setMotorAngle(float angleDegrees);

private:
    void disableMotor();
    void enableMotor();

    Pid controllerD_;
    Pid controllerQ_;
    int16_t setpointD_ = 0;
    int16_t setpointQ_ = 0;
    float motorAngle_ = 0;
    float currentAlpha_ = 0;
    float currentBeta_ = 0;
};

}

#include "bldc_motor_foc_impl.hpp"

#endif // LIBMOTOR_BLDC_MOTOR_FOC_HPP
