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

template <size_t id> class CurrentControl {
public:
  static inline Pid::Parameter currentPidParameters_{
      1.0f, 0.0f, 0.0f, 10000000.0f, std::numeric_limits<int16_t>::max()};
  static inline Pid currentPid_;
  static inline float currentError_{};
  static inline float commandedCurrent_{};
  static inline float currentCharge_{};
  static inline bool isLimiting_{false};
  static inline float filteredActualCurrent_{0.0f};
  static inline modm::BoundedDeque<std::pair<float, float>, 256>
      currentValues_{};
  static constexpr uint16_t zeroAverageCountdownReset_{256};
  static inline uint16_t zeroAverageCountdown_{zeroAverageCountdownReset_};
  static inline modm::filter::MovingAverage<float, 16> zeroAverage_{};
  static constexpr float rampMultiplierReset_{0.01f}, rampIncrement_ = 0.01f;
  static inline float rampMultiplier_{rampMultiplierReset_};

  static float getCharge();

  template <typename Device>
  static int16_t update(float commandedCurrent, const MotorState &state);

  static void resetIfApplicable(const MotorState &state);
  static void reset();
};

#include "current_control_impl.hpp"
#endif