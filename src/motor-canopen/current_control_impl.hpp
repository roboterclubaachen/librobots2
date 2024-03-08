#ifndef CURRENT_CONTROL_HPP
#error "Do not include this file directly, use current_control.hpp instead"
#endif
#include <algorithm>
#include <cmath>

template<size_t id>
template<typename Device>
std::tuple<int16_t, float>
CurrentControl<id>::update(float inCurrent, const MotorState &state)
{
	isLimiting_ = false;
	const float avg_updateTime_s = state.updateTime_us_.getValue() / 1000000.0f;

	// Copy sign of in current to actual Current
	filteredActualCurrent_ = std::abs(state.unorientedCurrent_ - state.zeroAverage_.getValue());
	Device::setValueChanged(CurrentObjects::FilteredActualCurrent);

	commandedCurrent_ = std::abs(inCurrent);
	const float sign = inCurrent * (inverting_ ? -1.0f : 1.0f);

	// Do charge limiting
  if (std::abs(state.currentCharge_) > state.maxCharge_)
  {
	  const auto projectedCharge = state.currentCharge_ + avg_updateTime_s * commandedCurrent_;
	  const auto toDoubleCharge = 2.0f * state.maxCharge_ - std::abs(projectedCharge);
	  const auto percentOfChargeRemaining = toDoubleCharge / state.maxCharge_;
	  commandedCurrent_ *= percentOfChargeRemaining * percentOfChargeRemaining;

	  if (std::abs(state.currentCharge_) > state.maxCharge_ * 2.0f)
	  {
		  isLimiting_ = true;
		  commandedCurrent_ = 0;
	  }
  }

  // Limit to max Current
  if (commandedCurrent_ > std::abs(state.maxCurrent_))
  {
	  commandedCurrent_ = std::clamp(commandedCurrent_, 0.0f, state.maxCurrent_);
	  isLimiting_ = true;
  }
  Device::setValueChanged(CurrentObjects::CommandedCurrent);

  currentError_ = commandedCurrent_ - filteredActualCurrent_;
  Device::setValueChanged(CurrentObjects::CurrentError);

  if (commandedCurrent_ == 0.0f) return {0, 0.0f};

  return {std::copysign(maxPWM_, sign), commandedCurrent_};
}

template<size_t id>
void
CurrentControl<id>::resetIfApplicable(const MotorState &state)
{
	if (!state.enableMotor_ ||
		state.status_.state() != modm_canopen::cia402::State::OperationEnabled ||
		state.mode_ == OperatingMode::Disabled)
	{
		reset();
	}
}

template<size_t id>
void
CurrentControl<id>::reset()
{}