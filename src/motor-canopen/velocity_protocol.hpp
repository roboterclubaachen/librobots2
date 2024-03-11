#ifndef VELOCITY_PROTOCOL_HPP
#define VELOCITY_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "velocity_control.hpp"
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

template <size_t id> class VelocityProtocol {
public:
  static inline int32_t receivedVelocity_{};
  static inline int32_t commandedVelocity_{};

  template<typename State>
  static bool applicable() {
    VelocityControl<id>:: template resetIfApplicable<State>();
    return State::mode_ == OperatingMode::Velocity &&
           State::status_.state() ==
               modm_canopen::cia402::State::OperationEnabled;
  }

  template <typename Device, typename State,  typename MessageCallback>
  static bool update(MessageCallback &&cb);

  template <typename ObjectDictionary, typename State>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename State, typename MessageCallback>
  static void processMessage(const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "velocity_protocol_impl.hpp"
#endif