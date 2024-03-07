#include "current_limit.hpp"
#include <modm/debug/logger.hpp>

namespace librobots2::motor_sim
{

double CurrentLimit::currentLimit_{-1.0};

double
CurrentLimit::get()
{
	return currentLimit_;
}

void
CurrentLimit::unset()
{
	currentLimit_ = -1.0;
}

void
CurrentLimit::set(double ampere)
{
	if (ampere < 0.0) { MODM_LOG_ERROR << "Tried to set negative current limit!" << modm::endl; }
	currentLimit_ = ampere;
}

}  // namespace librobots2::motor_sim