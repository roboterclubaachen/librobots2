#ifndef IDENTITY_PROTOCOL_HPP
#define IDENTITY_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "motor_state.hpp"
#include "identity_objects.hpp"
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include <chrono>
#include <modm/processing/timer.hpp>

template <size_t id> class IdentityProtocol {
public:
  static bool applicable(const MotorState &) { return false; }

  template <typename Device, typename MessageCallback>
  static bool update(MotorState &, MessageCallback &&) {
    return true;
  }

  template <typename ObjectDictionary, const MotorState &state>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename MessageCallback>
  static void processMessage(MotorState &, const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "identity_protocol_impl.hpp"
#endif