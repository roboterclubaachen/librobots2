#ifndef HEARTBEAT_PROTOCOL_HPP
#define HEARTBEAT_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "heartbeat_objects.hpp"
#include "motor_state.hpp"
#include <chrono>
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/processing/timer.hpp>


using namespace std::literals;

template <size_t id> class HeartbeatProtocol {
public:
  static constexpr auto masterID = 0;
  static inline auto timeBetweenHeatbeats{200ms};
  static inline modm::PeriodicTimer heartBeatTimer_{timeBetweenHeatbeats / 2};
  static inline auto masterHeartbeatTimeout{200ms};
  static inline bool receivedMasterHeartbeat{false};
  static inline modm::Clock::time_point lastMasterHeartbeat{};

public:
  static bool applicable(const MotorState &) { return true; }

  template <typename Device, typename MessageCallback>
  static bool update(MotorState &state, MessageCallback &&cb);

  template <typename ObjectDictionary, const MotorState &state>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename MessageCallback>
  static void processMessage(MotorState &, const modm::can::Message &,
                             MessageCallback &&);

private:
  static inline void makeHeartbeatMSG(uint8_t canId, modm::can::Message &msg);
};

#include "heartbeat_protocol_impl.hpp"
#endif