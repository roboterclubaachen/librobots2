#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP
#include <modm-canopen/device/canopen_device.hpp>

template <size_t id, typename State, typename... Modes> class MotorControl {
private:
  template <typename Device, typename MessageCallback, typename First,
            typename Second, typename... Rest>
  static bool updateMode(MessageCallback &&cb);

  template <typename Device, typename MessageCallback, typename First>
  static bool updateMode(MessageCallback &&cb);

public:
  template <typename Device, typename MessageCallback>
  static bool update(MessageCallback &&cb);

  template <typename Device, typename MessageCallback>
  static void processMessage(const modm::can::Message &message,
                             MessageCallback &&cb);

  template <typename ObjectDictionary>
  constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);
};

#include "motor_control_impl.hpp"
#endif