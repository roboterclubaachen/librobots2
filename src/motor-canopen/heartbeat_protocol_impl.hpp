#ifndef HEARTBEAT_PROTOCOL_HPP
#error "Do not include this file directly, use heartbeat_protocol.hpp instead"
#endif

#include <modm/debug/logger.hpp>
using modm_canopen::cia402::State;

template <size_t id>
void HeartbeatProtocol<id>::makeHeartbeatMSG(uint8_t canId,
                                             modm::can::Message &message) {
  message = modm::can::Message{0x700 + (uint32_t)canId, 1};
  message.setExtended(false);
  message.data[0] = 0x05; // Always report operational, as we otherwise would
                          // not be able to send
}

template <size_t id>
template <typename Device, typename MessageCallback>
bool HeartbeatProtocol<id>::update(MotorState &state, MessageCallback &&cb) {
  if (heartBeatTimer_.execute()) {
    modm::can::Message message;
    makeHeartbeatMSG(Device::nodeId(), message);
    cb(message);
  }
  auto now = modm::Clock::now();
  if (receivedMasterHeartbeat &&
      now - lastMasterHeartbeat > masterHeartbeatTimeout) {
    MODM_LOG_WARNING << "Master heartbeat timed out!" << modm::endl;

    state.mode_ = OperatingMode::Disabled;
    // TODO implement this in statemachine
    auto stateWord_ = state.status_.status();
    constexpr uint16_t disableVoltageMask_ = 0b0100'1111;
    constexpr uint16_t disableVoltageValue_ = 0b0100'0000;
    stateWord_ = (stateWord_ & ~disableVoltageMask_) |
                 (disableVoltageValue_ & disableVoltageMask_);
    state.status_.set(stateWord_);
    Device::setValueChanged(StateObjects::ModeOfOperation);
    Device::setValueChanged(StateObjects::StatusWord);

    receivedMasterHeartbeat = false;
  }
  return true;
}

template <size_t id>
template <typename Device, typename MessageCallback>
void HeartbeatProtocol<id>::processMessage(MotorState &,
                                           const modm::can::Message &msg,
                                           MessageCallback &&) {
  if (msg.getIdentifier() != 0x700 + masterID)
    return;
  if (msg.getLength() != 1)
    return;
  if (msg.data[0] != 0x5)
    return;
  if (!receivedMasterHeartbeat) {
    MODM_LOG_INFO << "Received Master heartbeat!" << modm::endl;
  }
  receivedMasterHeartbeat = true;
  lastMasterHeartbeat = modm::Clock::now();
}

template <size_t id>
template <typename ObjectDictionary, const MotorState &state>
constexpr void HeartbeatProtocol<id>::registerHandlers(
    modm_canopen::HandlerMap<ObjectDictionary> &map) {
  using modm_canopen::SdoErrorCode;
  map.template setReadHandler<HeartbeatObjects::TimeBetweenHeartbeats>(
      +[]() { return (uint16_t)timeBetweenHeatbeats.count(); });

  map.template setWriteHandler<HeartbeatObjects::TimeBetweenHeartbeats>(
      +[](uint16_t value) {
        timeBetweenHeatbeats = std::chrono::milliseconds((int64_t)value);
        heartBeatTimer_ = modm::PeriodicTimer{timeBetweenHeatbeats / 2};
        return SdoErrorCode::NoError;
      });

  map.template setReadHandler<HeartbeatObjects::MasterHeartbeatTimeout>(
      +[]() { return (uint16_t)masterHeartbeatTimeout.count(); });

  map.template setWriteHandler<HeartbeatObjects::MasterHeartbeatTimeout>(
      +[](uint16_t value) {
        masterHeartbeatTimeout = std::chrono::milliseconds((int64_t)value);
        return SdoErrorCode::NoError;
      });
}