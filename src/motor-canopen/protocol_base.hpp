#ifndef PROTOCOL_BASE_HPP
#define PROTOCOL_BASE_HPP
#include <modm/architecture/interface/can_message.hpp>
#include <modm-canopen/handler_map.hpp>

#error "Do not use this file directly, without modifying it first!"
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
