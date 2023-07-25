#ifndef POSITION_PROTOCOL_HPP
#define POSITION_PROTOCOL_HPP
#include <cstdint>
#include <limits>

#include "motor_state.hpp"
#include <modm-canopen/device/canopen_device.hpp>
#include <modm-canopen/object_dictionary.hpp>
#include <modm/math/filter/pid.hpp>
using Pid = modm::Pid<float>;

struct PositionObjects {
  static constexpr modm_canopen::Address PositionDemandValue{0x6062,
                                                             0}; // User units
  static constexpr modm_canopen::Address TargetPosition{0x607A,
                                                        0}; // User units
  static constexpr modm_canopen::Address PositionWindow{0x6067,
                                                        0}; // User units
  static constexpr modm_canopen::Address FollowingErrorActualValue{
      0x60F4, 0}; // User units

  static constexpr modm_canopen::Address PositionPID_kP{0x2006, 1}; // Custom
  static constexpr modm_canopen::Address PositionPID_kI{0x2006, 2}; // Custom
  static constexpr modm_canopen::Address PositionPID_kD{0x2006, 3}; // Custom
  static constexpr modm_canopen::Address PositionPID_MaxErrorSum{0x2006,
                                                                 4}; // Custom
};

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
  static bool applicable(const MotorState &state);

  template <typename Device, typename MessageCallback>
  static bool update(MotorState &state, MessageCallback &&cb);

  template <typename ObjectDictionary, const MotorState &state>
  static constexpr void
  registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);

  template <typename Device, typename MessageCallback>
  static void processMessage(MotorState &, const modm::can::Message &,
                             MessageCallback &&) {}
};

#include "position_protocol_impl.hpp"
#endif
