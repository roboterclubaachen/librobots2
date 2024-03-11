#ifndef MOTOR_CONTROL_HPP
#error "Do not include this file directly, use motor_control.hpp instead"
#endif

#include <modm/debug/logger.hpp>

template <size_t id, typename State, typename... Modes>
template <typename Device, typename MessageCallback, typename First>
bool MotorControl<id, State, Modes...>::updateMode(MessageCallback &&cb) {
  if (First:: template applicable<State>())
    return First::template update<Device, State, MessageCallback>(
        std::forward<MessageCallback>(cb));
  return true;
}

template <size_t id, typename State, typename... Modes>
template <typename Device, typename MessageCallback, typename First,
          typename Second, typename... Rest>
bool MotorControl<id, State, Modes...>::updateMode(MessageCallback &&cb) {
  auto out = false;
  if (First::template applicable<State>())
    out = First::template update<Device, State, MessageCallback>(
        std::forward<MessageCallback>(cb));
  return updateMode<Device, MessageCallback, Second, Rest...>(
             std::forward<MessageCallback>(cb)) &&
         out;
}

template <size_t id, typename State, typename... Modes>
template <typename Device, typename MessageCallback>
void MotorControl<id, State, Modes...>::processMessage(
    const modm::can::Message &message, MessageCallback &&cb) {
  (Modes{}.template processMessage<Device,State, MessageCallback>(
       message, std::forward<MessageCallback>(cb)),
   ...);
}

template <size_t id, typename State, typename... Modes>
template <typename Device, typename MessageCallback>
bool MotorControl<id, State, Modes...>::update(MessageCallback &&cb) {
    auto value = State:: template update<Device, MessageCallback>(std::forward<MessageCallback>(cb));
		auto temp =
			updateMode<Device, MessageCallback, Modes...>(std::forward<MessageCallback>(cb));
		value = value || temp;

  return value;
}

template <size_t id, typename State, typename... Modes>
template <typename ObjectDictionary>
constexpr void MotorControl<id, State, Modes...>::registerHandlers(
    modm_canopen::HandlerMap<ObjectDictionary> &map) {
  
  State::template registerHandlers<ObjectDictionary>(map);
  (Modes::template registerHandlers<ObjectDictionary, State>(map), ...);
}
