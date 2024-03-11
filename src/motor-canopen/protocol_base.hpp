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
  template <typename State>
  static bool applicable();

  template <typename Device, typename State, typename MessageCallback>
  static bool update(MessageCallback &&cb);

  template <typename ObjectDictionary, typename State>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename State, typename MessageCallback>
  static void processMessage(const modm::can::Message &,
                             MessageCallback &&);
};
#endif
