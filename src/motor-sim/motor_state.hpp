#pragma once
#include "motor_data.hpp"
#include <modm/math/geometry/vector3.hpp>

namespace librobots2::motor_sim
{

struct MotorState
{
	modm::Vector3f i{0, 0, 0}, v{0, 0, 0}, e{0, 0, 0};  // Current Voltage and BackEMF
	double omega_m{};                                   // Mechanical angular velocity
	double t_e{};                                       // Electromagnetic Torque
	double t_f{};                                       // Friction Torque
	double t_l{0.0};                                    // Load Torque
	double theta_m{};                                   // Mechanical angle
	double theta_e{};                                   // Electrical angle
	double theta_e_integrated{};                        // Electrical angle not wrapped
};

}  // namespace librobots2::motor_sim