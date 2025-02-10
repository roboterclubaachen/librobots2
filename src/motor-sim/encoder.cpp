#include "encoder.hpp"
#include "motor_simulation.hpp"

namespace librobots2::motor_sim
{
void
Encoder::initialize()
{}

void
Encoder::setOverflow(uint16_t val)
{
	overflow_ = val;
}

uint16_t
Encoder::getEncoderRaw()
{
	return ((uint16_t)std::round(MotorSimulation::state().theta_e_integrated * overflow_ /
								(2 * M_PI))) % overflow_;
}

bool
Encoder::setGating(uint8_t)
{
	return true;
}

}  // namespace librobots2::motor_sim