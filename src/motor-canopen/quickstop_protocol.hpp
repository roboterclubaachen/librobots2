#ifndef QUICKSTOP_PROTOCOL_HPP
#define QUICKSTOP_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "motor_state.hpp"
#include "velocity_control.hpp"
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include "quickstop_objects.hpp"


template <size_t id> class QuickstopProtocol {
public:
  static inline int32_t quickStopDeceleration_{10000};

public:
  static bool applicable(const MotorState &state) { return state.enableMotor_; }

  template <typename Device, typename MessageCallback>
  static bool update(MotorState &state, MessageCallback &&cb);

  template <typename ObjectDictionary, const MotorState &state>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename MessageCallback>
  static void processMessage(MotorState &, const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "quickstop_protocol_impl.hpp"
#endif