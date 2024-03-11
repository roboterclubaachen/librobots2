#ifndef CURRENT_CONTROL_HPP
#error "Do not include this file directly, use current_control.hpp instead"
#endif
#include <algorithm>
#include <cmath>
#include <tuple>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/states.hpp>

using modm_canopen::cia402::OperatingMode;

template<size_t id>
template<typename Device, typename State>
std::tuple<int16_t, float>
CurrentControl<id>::update(float inCurrent)
{
	isLimiting_ = false;
	const float avg_updateTime_s = State::updateTime_us_.getValue() / 1000000.0f;

	// Copy sign of in current to actual Current
	filteredActualCurrent_ = std::abs(State::unorientedCurrent_ - State::zeroAverage_.getValue());
	Device::setValueChanged(CurrentObjects::FilteredActualCurrent);

	commandedCurrent_ = std::abs(inCurrent);
	const float sign = inCurrent * (inverting_ ? -1.0f : 1.0f);

	// Do charge limiting
  if (std::abs(State::currentCharge_) > State::maxCharge_)
  {
	  const auto projectedCharge = State::currentCharge_ + avg_updateTime_s * commandedCurrent_;
	  const auto toDoubleCharge = 2.0f * State::maxCharge_ - std::abs(projectedCharge);
	  const auto percentOfChargeRemaining = toDoubleCharge / State::maxCharge_;
	  commandedCurrent_ *= percentOfChargeRemaining * percentOfChargeRemaining;

	  if (std::abs(State::currentCharge_) > State::maxCharge_ * 2.0f)
	  {
		  isLimiting_ = true;
		  commandedCurrent_ = 0;
	  }
  }

  // Limit to max Current
  if (commandedCurrent_ > std::abs(State::maxCurrent_))
  {
	  commandedCurrent_ = std::clamp(commandedCurrent_, 0.0f, State::maxCurrent_);
	  isLimiting_ = true;
  }
  Device::setValueChanged(CurrentObjects::CommandedCurrent);

  currentError_ = commandedCurrent_ - filteredActualCurrent_;
  Device::setValueChanged(CurrentObjects::CurrentError);

  if (commandedCurrent_ == 0.0f) return {0, 0.0f};

  return {std::copysign(maxPWM_, sign), commandedCurrent_};
}

template<size_t id>
template<typename State>
void
CurrentControl<id>::resetIfApplicable()
{
	if (!State::enableMotor_ ||
		State::status_.state() != modm_canopen::cia402::State::OperationEnabled ||
		State::mode_ == OperatingMode::Disabled)
	{
		reset();
	}
}

template<size_t id>
void
CurrentControl<id>::reset()
{}