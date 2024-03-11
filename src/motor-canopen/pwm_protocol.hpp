#ifndef PWM_PROTOCOL_HPP
#define PWM_PROTOCOL_HPP
#include <cstdint>

#include "motor_state.hpp"
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/states.hpp>
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include "pwm_objects.hpp"


template <size_t id> class PWMProtocol {
public:
  static inline int16_t commandedPWM_{0};

public:
template <typename State>
  static bool applicable() {
    return State::mode_ == OperatingMode::Voltage &&
           State::status_.state() ==
               modm_canopen::cia402::State::OperationEnabled;
  }

  template <typename Device, typename State, typename MessageCallback>
  static bool update(MessageCallback &&cb);

  template <typename ObjectDictionary, typename State>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename State, typename MessageCallback>
  static void processMessage(const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "pwm_protocol_impl.hpp"
#endif