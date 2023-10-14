#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

struct StateObjects {
  static constexpr modm_canopen::Address ControlWord{0x6040, 0};
  static constexpr modm_canopen::Address StatusWord{0x6041, 0};
  static constexpr modm_canopen::Address ModeOfOperation{0x6060, 0};
  static constexpr modm_canopen::Address ModeOfOperationDisplay{0x6061, 0};

  static constexpr modm_canopen::Address UpdateTime{0x2001, 0}; // Micro seconds

  static constexpr modm_canopen::Address PositionInternalValue{
      0x6063, 0}; // internal units
  static constexpr modm_canopen::Address PositionActualValue{0x6064,
                                                             0}; // User units

  static constexpr modm_canopen::Address VelocityActualValue{0x606C,
                                                             0}; // User units

  static constexpr modm_canopen::Address UnorientedCurrent{0x2013, 0}; // Custom
  static constexpr modm_canopen::Address OrientedCurrent{0x2019, 0};   // Custom
  static constexpr modm_canopen::Address OrientedCurrentAngle{0x2020,
                                                              0};  // Custom
  static constexpr modm_canopen::Address MaxCurrent{0x2011, 0};    // Custom
  static constexpr modm_canopen::Address MaxCharge{0x2015, 0};     // Custom
  static constexpr modm_canopen::Address CurrentCharge{0x2017, 0}; // Custom

  static constexpr modm_canopen::Address OutputPWM{0x2003, 0}; // Custom
  static constexpr modm_canopen::Address Reset{0x2007, 0};     // Set 1/0

  static constexpr modm_canopen::Address PositionFactorNumerator{0x6093, 1};
  static constexpr modm_canopen::Address PositionFactorDivisor{0x6093, 2};
  static constexpr modm_canopen::Address VelocityFactorNumerator{0x6094, 1};
  static constexpr modm_canopen::Address VelocityFactorDivisor{0x6094, 2};
  static constexpr modm_canopen::Address AccelerationFactorNumerator{0x6097, 1};
  static constexpr modm_canopen::Address AccelerationFactorDivisor{0x6097, 2};
  static constexpr modm_canopen::Address Polarity{0x607E, 0};
};