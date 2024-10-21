#include "encoder.hpp"
#include "motor_simulation.hpp"

namespace librobots2::motor_sim
{
void
Encoder::initialize()
{}

uint16_t
Encoder::getEncoderRaw()
{
	return (uint16_t)std::round(MotorSimulation::state().theta_e_integrated * 2000 / (2 * M_PI));
}

bool
Encoder::setGating(uint8_t)
{
	return true;
}

}  // namespace librobots2::motor_sim