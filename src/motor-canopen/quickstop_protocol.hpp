#ifndef QUICKSTOP_PROTOCOL_HPP
#define QUICKSTOP_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "velocity_control.hpp"
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include "quickstop_objects.hpp"


template <size_t id> class QuickstopProtocol {
public:
  static inline int32_t quickStopDeceleration_{10000};

public:
  template <typename State>
  static bool applicable() { return State::enableMotor_; }

  template <typename Device, typename State, typename MessageCallback>
  static bool update(MessageCallback &&cb);

  template <typename ObjectDictionary, typename State>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename State, typename MessageCallback>
  static void processMessage(const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "quickstop_protocol_impl.hpp"
#endif