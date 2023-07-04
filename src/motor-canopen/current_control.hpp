#ifndef CURRENT_CONTROL_HPP
#define CURRENT_CONTROL_HPP
#include "motor_state.hpp"
#include <cstdlib>
#include <modm/container/deque.hpp>
#include <modm/math/filter/moving_average.hpp>
#include <modm/math/filter/pid.hpp>
#include <modm/processing/timer.hpp>

using Pid = modm::Pid<float>;

struct CurrentObjects {
  static constexpr modm_canopen::Address TargetCurrent{0x2014, 0};    // Custom
  static constexpr modm_canopen::Address CurrentError{0x2012, 0};     // Custom
  static constexpr modm_canopen::Address CommandedCurrent{0x2016, 0}; // Custom
  static constexpr modm_canopen::Address CurrentCharge{0x2017, 0};    // Custom
  static constexpr modm_canopen::Address FilteredActualCurrent{0x2018,
                                                               0}; // Custom

  static constexpr modm_canopen::Address CurrentPID_kP{0x2010, 1}; // Custom
  static constexpr modm_canopen::Address CurrentPID_kI{0x2010, 2}; // Custom
  static constexpr modm_canopen::Address CurrentPID_kD{0x2010, 3}; // Custom
  static constexpr modm_canopen::Address CurrentPID_MaxErrorSum{0x2010,
                                                                4}; // Custom
};

template <size_t id> class CurrentControl {
public:
  static inline Pid::Parameter currentPidParameters_{
      1.0f, 0.0f, 0.0f, 10000000.0f, std::numeric_limits<int16_t>::max()};
  static inline Pid currentPid_;
  static inline float currentError_{};
  static inline float commandedCurrent_{};
  static inline float currentCharge_{};
  static inline float filteredActualCurrent_{0.0f};
  static inline modm::Clock::time_point lastExecute_{modm::Clock::now()};
  static inline modm::BoundedDeque<std::pair<float, float>, 256>
      currentValues_{};
  static constexpr uint16_t zeroAverageCountdownReset_{256};
  static inline uint16_t zeroAverageCountdown_{zeroAverageCountdownReset_};
  static inline modm::filter::MovingAverage<float, 16> zeroAverage_{};
  static inline float t_pt1 = 2.0f, k_pt1 = 1.0f;
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