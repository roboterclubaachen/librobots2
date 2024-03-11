#ifndef CURRENT_PROTOCOL_HPP
#define CURRENT_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

#include "current_control.hpp"

template <size_t id> class CurrentProtocol {
public:
  static inline float targetCurrent_{};

  template <typename State>
  static bool applicable() {
    CurrentControl<0>::resetIfApplicable<State>();
    return State::mode_ == OperatingMode::Current &&
           State::status_.state() ==
               modm_canopen::cia402::State::OperationEnabled;
  }

  template <typename Device, typename State, typename MessageCallback>
  static bool update(MessageCallback &&cb);

  template <typename ObjectDictionary, typename State>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename State,  typename MessageCallback>
  static void processMessage(const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "current_protocol_impl.hpp"
#endif