#ifndef VELOCITY_PROTOCOL_HPP
#define VELOCITY_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include <modm-canopen/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include "motor_state.hpp"
#include "velocity_control.hpp"

template <size_t id> class VelocityProtocol {
public:
  static inline int32_t receivedVelocity_{};
  static inline int32_t commandedVelocity_{};

  static bool applicable(const MotorState &state) {
    VelocityControl<0>::resetIfApplicable(state);
    return state.mode_ == OperatingMode::Velocity &&
           state.status_.state() ==
               modm_canopen::cia402::State::OperationEnabled;
  }

  template <typename Device, typename MessageCallback>
  static bool update(MotorState &state, MessageCallback &&cb);

  template <typename ObjectDictionary, const MotorState &state>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename MessageCallback>
  static void processMessage(MotorState &, const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "velocity_protocol_impl.hpp"
#endif