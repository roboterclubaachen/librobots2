#ifndef HEARTBEAT_PROTOCOL_HPP
#define HEARTBEAT_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "heartbeat_objects.hpp"
#include <chrono>
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/processing/timer.hpp>


using namespace std::literals;

template <size_t id> class HeartbeatProtocol {
public:
  static constexpr auto masterID = 0;
  static inline auto timeBetweenHeatbeats{400ms};
  static inline modm::PeriodicTimer heartBeatTimer_{timeBetweenHeatbeats / 4};
  static inline auto masterHeartbeatTimeout{400ms};
  static inline bool receivedMasterHeartbeat{false};
  static inline modm::Clock::time_point lastMasterHeartbeat{};

public:
  template <typename State>
  static bool applicable() { return true; }

  template <typename Device, typename State, typename MessageCallback>
  static bool update(MessageCallback &&cb);

  template <typename ObjectDictionary, typename State>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename State, typename MessageCallback>
  static void processMessage(const modm::can::Message &,
                             MessageCallback &&);

private:
  static inline void makeHeartbeatMSG(uint8_t canId, modm::can::Message &msg);
};

#include "heartbeat_protocol_impl.hpp"
#endif