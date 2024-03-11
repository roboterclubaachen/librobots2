#ifndef POSITION_PROTOCOL_HPP
#define POSITION_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "motor_state.hpp"
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/math/filter/pid.hpp>
#include "position_objects.hpp"
using Pid = modm::Pid<float>;



template <size_t id> class PositionProtocol {
public:
  static inline Pid::Parameter positionPidParameters_{1.0f, 0.0f, 0.0f,
                                                      1000000.0f, 7000};
  static inline Pid positionPid_;
  static inline bool receivedPositionRelative_{true};
  static inline int32_t receivedPosition_{};
  static inline bool nextPositionIsNew_{false};
  static inline int32_t nextPosition_{};
  static inline int32_t commandedPosition_{};
  static inline uint32_t positionWindow_{5};
  static inline int32_t positionError_{};
  static inline uint32_t positionWindowTime_{10};
  static inline uint32_t inPositionWindow_{0};

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
                             MessageCallback &&){}
};

#include "position_protocol_impl.hpp"
#endif
