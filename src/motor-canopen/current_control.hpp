#ifndef CURRENT_CONTROL_HPP
#define CURRENT_CONTROL_HPP
#include "current_objects.hpp"
#include "motor_state.hpp"
#include <cstdlib>
#include <modm/container/deque.hpp>
#include <modm/math/filter/moving_average.hpp>
#include <modm/math/filter/pid.hpp>
#include <modm/processing/timer.hpp>

using Pid = modm::Pid<float>;

template<size_t id>
class CurrentControl
{
public:
	static inline float currentError_{};
	static inline float commandedCurrent_{};
	static inline bool isLimiting_{false};
	static inline bool inverting_{false};
	static inline float filteredActualCurrent_{0.0f};
	static inline int16_t maxPWM_{1 << 14};

	template<typename Device>
	static std::tuple<int16_t, float>
	update(float commandedCurrent, const MotorState &state);

	static void
	resetIfApplicable(const MotorState &state);
	static void
	reset();
};

#include "current_control_impl.hpp"
#endif