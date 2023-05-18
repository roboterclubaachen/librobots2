#ifndef CURRENT_CONTROL_HPP
#define CURRENT_CONTROL_HPP
#include "motor_state.hpp"
#include <cstdlib>
#include <modm/math/filter/pid.hpp>

using Pid = modm::Pid<float>;

struct CurrentObjects {
  static constexpr modm_canopen::Address CommandedCurrent{0x2014, 0}; // Custom
  static constexpr modm_canopen::Address CurrentError{0x2012, 0};     // Custom

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

  static int16_t update(float commandedCurrent, const MotorState &state);

  static void resetIfApplicable(const MotorState &state);
};

#include "current_control_impl.hpp"
#endif