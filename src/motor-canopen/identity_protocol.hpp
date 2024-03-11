#ifndef IDENTITY_PROTOCOL_HPP
#define IDENTITY_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "identity_objects.hpp"
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include <chrono>
#include <modm/processing/timer.hpp>

template <size_t id> class IdentityProtocol {
public:
  template<typename State>
  static bool applicable() { return false; }

  template <typename Device, typename State, typename MessageCallback>
  static bool update(MessageCallback &&) {
    return true;
  }

  template <typename ObjectDictionary, typename State>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename State, typename MessageCallback>
  static void processMessage(const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "identity_protocol_impl.hpp"
#endif