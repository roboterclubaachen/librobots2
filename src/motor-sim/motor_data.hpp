#pragma once
#include <cstdint>
#include <cmath>

namespace librobots2::motor_sim
{

struct MotorData
{
	double l{0.0215};           // Phase inductance
	double m{0.002};            // Mutual inductance
	double r_s{11.05};          // Stator resistance
	double k_e{0.0143};         // BackEMF constant
	uint8_t p{6};               // Number of pole pairs
	double vdc{2 * 10 * M_PI};  // Supply voltage

	double j{0.0001};   // Inertia
	double f_l{0.001};  // Linear Friction
	double f_s{0.1};    // Static Friction
};

}  // namespace librobots2::motor_sim