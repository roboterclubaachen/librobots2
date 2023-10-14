#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

struct CurrentObjects {
  static constexpr modm_canopen::Address TargetCurrent{0x2014, 0};    // Custom
  static constexpr modm_canopen::Address CurrentError{0x2012, 0};     // Custom
  static constexpr modm_canopen::Address CommandedCurrent{0x2016, 0}; // Custom
  static constexpr modm_canopen::Address FilteredActualCurrent{0x2018,
                                                               0}; // Custom

  static constexpr modm_canopen::Address CurrentPID_kP{0x2010, 1}; // Custom
  static constexpr modm_canopen::Address CurrentPID_kI{0x2010, 2}; // Custom
  static constexpr modm_canopen::Address CurrentPID_kD{0x2010, 3}; // Custom
  static constexpr modm_canopen::Address CurrentPID_MaxErrorSum{0x2010,
                                                                4}; // Custom
};
