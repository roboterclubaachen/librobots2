#pragma once
#include <limits>

#include "motor_bridge.hpp"

namespace librobots2::motor_sim
{

class MotorSimulation
{
private:
	static inline MotorData data_{};
	static inline MotorState state_{};

	static std::array<PhaseConfig, 3>
	applyCurrentlimit(double currentLimit, const std::array<PhaseConfig, 3> inConfig,
					  const modm::Vector3f& current);

	static modm::Vector3f
	computeVoltages(double v, const std::array<float, 3>& pwms,
					const std::array<PhaseConfig, 3>& config, const modm::Vector3f& bemf);

	static MotorState
	nextState(const std::array<float, 3>& pwms, const std::array<PhaseConfig, 3>& config,
			  double timestep, double currentLimit);

	static double
	angleMod(double angle);

	static modm::Vector3f
	emfFunction(double rotor_p);

	static void
	updateHallPort();

	static void
	updateADCs();

public:
	static void
	initialize(const MotorData& motor);

	static void
	update(double timestep);

	static MotorState&
	state();

	static std::array<double, 3>
	getInputVoltages();

	static double
	maxCurrent();
};

}  // namespace librobots2::motor_sim