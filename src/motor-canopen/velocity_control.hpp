#ifndef VELOCITY_CONTROL_HPP
#define VELOCITY_CONTROL_HPP
#include "current_control.hpp"
#include <cstdlib>
#include <modm/math/filter/pid.hpp>
#include "velocity_objects.hpp"

using Pid = modm::Pid<float>;

template<size_t id>
class VelocityControl
{
public:
	static inline Pid::Parameter velocityPidParameters_{1.0f, 0.0f, 0.0f, 10000000.0f,
														std::numeric_limits<int16_t>::max()};
	static inline Pid velocityPid_;
	static inline int32_t profileAcceleration_{5000};
	static inline bool isLimiting_{false};

	static inline int32_t commandedVel_{0};
	static inline int32_t velocityError_{};

	template<typename Device, typename State>
	static inline std::tuple<int16_t, float>
	doVelocityUpdate(int32_t commandedVelocity);

	template<typename Device, typename State>
	static inline std::tuple<int16_t, float>
	doDecelerationUpdate(int32_t commandedDeceleration);

	template<typename State>
	static inline void
	resetIfApplicable();

	static inline void
	reset();

private:
};

#include "velocity_control_impl.hpp"
#endif