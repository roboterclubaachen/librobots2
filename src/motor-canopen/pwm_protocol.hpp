#ifndef PWM_PROTOCOL_HPP
#define PWM_PROTOCOL_HPP
#include <cstdint>

#include "motor_state.hpp"
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/states.hpp>
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>

struct PWMObjects {
  static constexpr modm_canopen::Address PWMCommand{0x2002, 0}; // Custom
};

template <size_t id> class PWMProtocol {
public:
  static inline int16_t commandedPWM_{0};

public:
  static bool applicable(const MotorState &state) {
    return state.mode_ == OperatingMode::Voltage &&
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

#include "pwm_protocol_impl.hpp"
#endif