#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

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