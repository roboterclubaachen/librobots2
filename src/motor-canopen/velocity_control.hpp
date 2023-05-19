#ifndef VELOCITY_CONTROL_HPP
#define VELOCITY_CONTROL_HPP
#include "current_control.hpp"
#include <cstdlib>
#include <modm/math/filter/pid.hpp>

using Pid = modm::Pid<float>;

struct VelocityObjects {
  static constexpr modm_canopen::Address VelocityDemandValue{0x606B,
                                                             0}; // User units
  static constexpr modm_canopen::Address TargetVelocity{0x60FF,
                                                        0}; // User units
  static constexpr modm_canopen::Address ProfileAcceleration{0x6083,
                                                             0}; // User units

  static constexpr modm_canopen::Address VelocityError{0x2004, 1}; // Custom

  static constexpr modm_canopen::Address VelocityPID_kP{0x2005, 1}; // Custom
  static constexpr modm_canopen::Address VelocityPID_kI{0x2005, 2}; // Custom
  static constexpr modm_canopen::Address VelocityPID_kD{0x2005, 3}; // Custom
  static constexpr modm_canopen::Address VelocityPID_MaxErrorSum{0x2005,
                                                                 4}; // Custom
};

template <size_t id> class VelocityControl {
public:
  static inline Pid::Parameter velocityPidParameters_{
      1.0f, 0.0f, 0.0f, 10000000.0f, std::numeric_limits<int16_t>::max()};
  static inline Pid velocityPid_;
  static inline int32_t profileAcceleration_{5000};

  static inline int32_t velocityError_{};

  template <typename Device>
  static inline int16_t doVelocityUpdate(int32_t commandedVelocity,
                                         const MotorState &state);

  template <typename Device>
  static inline int16_t doDecelerationUpdate(int32_t commandedDeceleration,
                                             const MotorState &state);

  static inline void resetIfApplicable(const MotorState &state);

private:
  static inline int32_t lastCommandedVel_{0};
};

template <size_t id>
template <typename Device>
int16_t VelocityControl<id>::doVelocityUpdate(int32_t commandedVelocity,
                                              const MotorState &state) {

  if (std::signbit(lastCommandedVel_) != std::signbit(commandedVelocity) &&
      (commandedVelocity != 0 && lastCommandedVel_ != 0)) {
    velocityPid_.reset();
  }
  lastCommandedVel_ = commandedVelocity;
  if (std::signbit(commandedVelocity) ==
          std::signbit(state.actualVelocity_.getValue()) ||
      state.actualVelocity_.getValue() == 0) {

    velocityError_ = commandedVelocity - state.actualVelocity_.getValue();
    velocityPid_.update(velocityError_,
                        state.outputPWM_ > profileAcceleration_);
    Device::setValueChanged(VelocityObjects::VelocityError);
    return CurrentControl<id>::template update<Device>(velocityPid_.getValue(),
                                                       state);
  }
  return 0;
}

template <size_t id>
template <typename Device>
int16_t VelocityControl<id>::doDecelerationUpdate(int32_t commandedDeceleration,
                                                  const MotorState &state) {
  lastCommandedVel_ = 0;
  velocityError_ = -state.actualVelocity_.getValue();
  velocityPid_.update(velocityError_, state.outputPWM_ > commandedDeceleration);
  Device::setValueChanged(VelocityObjects::VelocityError);
  return (int16_t)std::clamp(
      (int32_t)CurrentControl<id>::template update<Device>(
          velocityPid_.getValue(), state),
      -commandedDeceleration, commandedDeceleration);
}

template <size_t id>
void VelocityControl<id>::resetIfApplicable(const MotorState &state) {
  if (!state.enableMotor_ ||
      state.status_.state() != modm_canopen::cia402::State::OperationEnabled ||
      state.mode_ == OperatingMode::Disabled ||
      state.mode_ == OperatingMode::Voltage) {
    velocityPid_.reset();
  }
}

#endif