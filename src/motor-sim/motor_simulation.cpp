#include "motor_simulation.hpp"
#include <limits>
#include <modm/debug/logger.hpp>
#include <cmath>

#include "current_limit.hpp"

namespace librobots2::motor_sim
{

void
MotorSimulation::initialize(const MotorData& motor)
{
	data_ = motor;
	state_ = {};
}

std::array<PhaseConfig, 3>
MotorSimulation::applyCurrentlimit(double currentLimit, const std::array<PhaseConfig, 3> inConfig,
								   const modm::Vector3f& current)
{
	std::array<PhaseConfig, 3> out = inConfig;
	if (currentLimit >= 0.0)
	{
		for (size_t i = 0; i < inConfig.size(); i++)
		{
			if ((inConfig[i] == PhaseConfig::High || inConfig[i] == PhaseConfig::Pwm))
			{
				if (current[i] > currentLimit) { out[i] = PhaseConfig::Low; }
			}
		}
	}
	return out;
}
double
MotorSimulation::angleMod(double angle)
{
	angle = std::fmod(angle, M_PI * 2);
	if (angle < 0.0)
	{
		angle += M_PI * 2;
		angle = std::fmod(angle, M_PI * 2);
	}
	return angle;
}

modm::Vector3f
MotorSimulation::emfFunction(double rotor_p)
{
	// Switching Pattern-Independent Simulation Model for Brushless DC Motors (Kang et al. 2011)
	auto out = modm::Vector3f();
	for (size_t i = 0; i < 3; i++)
	{
		rotor_p = angleMod(rotor_p);
		if (0 <= rotor_p && rotor_p < M_PI / 6)
		{
			out[i] = rotor_p * 6 / M_PI;
		} else if (M_PI / 6 <= rotor_p && rotor_p < 5 * M_PI / 6)
		{
			out[i] = 1;
		} else if (5 * M_PI / 6 <= rotor_p && rotor_p < 7 * M_PI / 6)
		{
			out[i] = (M_PI - rotor_p) * 6 / M_PI;
		} else if (7 * M_PI / 6 <= rotor_p && rotor_p < 11 * M_PI / 6)
		{
			out[i] = -1;
		} else if (11 * M_PI / 6 <= rotor_p && rotor_p < 2 * M_PI)
		{
			out[i] = (rotor_p - 2 * M_PI) * 6 / M_PI;
		}
		rotor_p += M_PI * 4.0 / 3.0;
	}
	return -out;
}

modm::Vector3f
MotorSimulation::computeVoltages(double v, const std::array<float, 3>& pwms,
								 const std::array<PhaseConfig, 3>& config,
								 const modm::Vector3f& bemf)
{
	size_t acc{0};
	double center{0.0};
	modm::Vector3f voltages{0, 0, 0};

	// At this point we technically compute potentials
	for (size_t i = 0; i < config.size(); i++)
	{
		float mult = 0.0f;
		switch (config[i])
		{
			case PhaseConfig::HiZ:
			case PhaseConfig::Low:
				break;
			case PhaseConfig::High:
				mult = 1.0f;
				break;
			case PhaseConfig::Pwm:
				mult = pwms[i];
				break;
		}
		voltages[i] =
			v * mult;  // Set outside points to +-half VDC depending if Gate is high or low
		if (config[i] != PhaseConfig::HiZ)
		{
			acc++;
			center += voltages[i] - bemf[i];  // Add bemf subtracted voltage to midpoint
		}
	}
	center /= acc;  // Compute center voltage

	// Make voltages relative to center
	for (size_t i = 0; i < config.size(); i++)
	{
		if (config[i] == PhaseConfig::HiZ)
		{
			voltages[i] = bemf[i];
		} else
		{
			voltages[i] -= center;
		}
	}
	return voltages;
}

MotorState
MotorSimulation::nextState(const std::array<float, 3>& pwms,
						   const std::array<PhaseConfig, 3>& rawConfig, double timestep,
						   double currentLimit)
{
	// Trapezoidal function
	const auto emf_factor = emfFunction(state_.theta_e);

	// Actual back emf voltages
	const auto e = emf_factor * data_.k_e * state_.omega_m;

	const auto realConfig = applyCurrentlimit(currentLimit, rawConfig, state_.i);

	// Phase voltages
	const auto v = computeVoltages(data_.vdc, pwms, realConfig, e);

	// Phase Currents
	const auto d_i = (v - data_.r_s * state_.i - e) / (data_.l - data_.m);
	const auto i = state_.i + d_i * timestep;

	// Torque
	const auto t_e = (emf_factor * state_.i) * data_.k_e;

	// Friction torque
	const auto t_f = ((std::abs(state_.omega_m) < 0.001f) ? data_.f_s : data_.f_l) * state_.omega_m;

	// Mechanical torque
	const auto t_m = t_e - state_.t_l - t_f;

	const bool isFrictionOnly = (std::abs(t_e - state_.t_l) < 0.00001f);

	// Mechanics
	auto d_omega_m = t_m / data_.j;
	auto omega_m = state_.omega_m + d_omega_m * timestep;

	// Come to a complete stop if we are very slow
	if (isFrictionOnly &&
		(std::signbit(omega_m) != std::signbit(state_.omega_m) || std::abs(omega_m) < 0.00001f))
	{
		d_omega_m = -state_.omega_m / timestep;
		omega_m = 0.0f;
	}

	const auto theta_m = angleMod(state_.theta_m + state_.omega_m * timestep);
	const auto theta_e = angleMod(theta_m * data_.p / 2);

	// Create new state object
	MotorState out{};
	out.e = e;
	out.i = i;
	out.v = v;
	out.omega_m = omega_m;
	out.theta_m = theta_m;
	out.theta_e = theta_e;
	out.t_e = t_e;
	out.t_f = t_f;
	out.t_l = state_.t_l;
	return out;
}

void
MotorSimulation::update(double timestep)
{
	state_ =
		nextState(MotorBridge::getPWMs(), MotorBridge::getConfig(), timestep, CurrentLimit::get());
	updateHallPort();
}

double
MotorSimulation::maxCurrent()
{

	return std::max(std::max(std::abs(state_.i[0]), std::abs(state_.i[1])), std::abs(state_.i[2]));
}

void
MotorSimulation::updateHallPort()
{
	const auto index = ((unsigned int)std::round(angleMod(state_.theta_e) * 6 / (2 * M_PI))) % 6;

	Pin<0>::set(index == 5 || index == 0 || index == 1);
	Pin<1>::set(index == 1 || index == 2 || index == 3);
	Pin<2>::set(index == 3 || index == 4 || index == 5);
}

MotorState&
MotorSimulation::state()
{
	return state_;
}

}  // namespace librobots2::motor_sim