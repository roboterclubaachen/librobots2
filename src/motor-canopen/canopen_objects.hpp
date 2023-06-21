#pragma once
#include <modm-canopen/object_dictionary.hpp>

struct Objects {
  // Canopen stuff
  static constexpr modm_canopen::Address ControlWord{0x6040, 0};
  static constexpr modm_canopen::Address StatusWord{0x6041, 0};
  static constexpr modm_canopen::Address ModeOfOperation{0x6060, 0};
  static constexpr modm_canopen::Address ModeOfOperationDisplay{0x6061, 0};

  // PWM Mode
  static constexpr modm_canopen::Address UpdateTime{0x2001, 0}; // Custom
  static constexpr modm_canopen::Address PWMCommand{0x2002, 0}; // Custom
  static constexpr modm_canopen::Address OutputPWM{0x2003, 0};  // Custom
  static constexpr modm_canopen::Address Reset{0x2007, 0};      // Set 1/0

  // Position Mode
  static constexpr modm_canopen::Address PositionDemandValue{0x6062,
                                                             0}; // User units
  static constexpr modm_canopen::Address PositionInternalValue{
      0x6063, 0}; // internal units
  static constexpr modm_canopen::Address PositionActualValue{0x6064,
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

  // Velocity Mode
  static constexpr modm_canopen::Address VelocityActualValue{0x606C,
                                                             0}; // User units
  static constexpr modm_canopen::Address TargetVelocity{0x60FF,
                                                        0}; // User units

  static constexpr modm_canopen::Address VelocityError{0x2004, 1}; // Custom

  static constexpr modm_canopen::Address VelocityPID_kP{0x2005, 1}; // Custom
  static constexpr modm_canopen::Address VelocityPID_kI{0x2005, 2}; // Custom
  static constexpr modm_canopen::Address VelocityPID_kD{0x2005, 3}; // Custom
  static constexpr modm_canopen::Address VelocityPID_MaxErrorSum{0x2005,
                                                                 4}; // Custom

  // Current Mode

  static constexpr modm_canopen::Address TargetCurrent{0x2014, 0};    // Custom
  static constexpr modm_canopen::Address CurrentError{0x2012, 0};     // Custom
  static constexpr modm_canopen::Address CommandedCurrent{0x2016, 0}; // Custom

  static constexpr modm_canopen::Address CurrentPID_kP{0x2010, 1}; // Custom
  static constexpr modm_canopen::Address CurrentPID_kI{0x2010, 2}; // Custom
  static constexpr modm_canopen::Address CurrentPID_kD{0x2010, 3}; // Custom
  static constexpr modm_canopen::Address CurrentPID_MaxErrorSum{0x2010,
                                                                4}; // Custom

  static constexpr modm_canopen::Address ActualCurrent{0x2013, 0}; // Custom
  static constexpr modm_canopen::Address MaxCurrent{0x2011, 0};    // Custom
  static constexpr modm_canopen::Address MaxCharge{0x2015, 0};     // Custom
  static constexpr modm_canopen::Address FilteredActualCurrent{0x2018,
                                                               0}; // Custom

  // Factors
  static constexpr modm_canopen::Address PositionFactorNumerator{0x6093, 1};
  static constexpr modm_canopen::Address PositionFactorDivisor{0x6093, 2};
  static constexpr modm_canopen::Address VelocityFactorNumerator{0x6094, 1};
  static constexpr modm_canopen::Address VelocityFactorDivisor{0x6094, 2};
  static constexpr modm_canopen::Address AccelerationFactorNumerator{0x6097, 1};
  static constexpr modm_canopen::Address AccelerationFactorDivisor{0x6097, 2};
  static constexpr modm_canopen::Address Polarity{0x607E, 0}; // TODO handle
};