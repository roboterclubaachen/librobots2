#ifndef PROTOCOL_BASE_HPP
#define PROTOCOL_BASE_HPP
#include <cstdint>
#include <limits>

#include "motor_state.hpp"
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
using Pid = modm::Pid<float>;

template <size_t id> class ProtocolBase {

public:
  static bool applicable(const MotorState &state);

  template <typename Device, typename MessageCallback>
  static bool update(MotorState &state, MessageCallback &&cb);

  template <typename ObjectDictionary, const MotorState &state>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename MessageCallback>
  static void processMessage(MotorState &, const modm::can::Message &,
                             MessageCallback &&) {}
};
#endif
