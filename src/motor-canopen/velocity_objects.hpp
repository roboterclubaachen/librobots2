#pragma once 
#include <modm-canopen/object_dictionary_common.hpp>


struct VelocityObjects {
  static constexpr modm_canopen::Address VelocityDemandValue{0x606B,
                                                             0}; // User units
  static constexpr modm_canopen::Address TargetVelocity{0x60FF,
                                                        0}; // User units
  static constexpr modm_canopen::Address ProfileAcceleration{0x6083,
                                                             0}; // User units

  static constexpr modm_canopen::Address VelocityError{0x2004, 1}; // Custom

  static constexpr modm_canopen::Address VelocityPID_kP{0x2005, 1}; // Custom
  static constexpr modm_canopen::Address VelocityPID_kI{0x2005, 2}; // Custom
  static constexpr modm_canopen::Address VelocityPID_kD{0x2005, 3}; // Custom
  static constexpr modm_canopen::Address VelocityPID_MaxErrorSum{0x2005,
                                                                 4}; // Custom
};