#ifndef CURRENT_PROTOCOL_HPP
#define CURRENT_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include <modm-canopen/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include "current_control.hpp"
#include "motor_state.hpp"

template <size_t id> class CurrentProtocol {
public:
  static inline float targetCurrent_{};

  static bool applicable(const MotorState &state) {
    CurrentControl<0>::resetIfApplicable(state);
    return state.mode_ == OperatingMode::Current &&
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

#include "current_protocol_impl.hpp"
#endif