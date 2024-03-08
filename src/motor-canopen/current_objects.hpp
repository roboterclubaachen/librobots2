#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

struct CurrentObjects {
  static constexpr modm_canopen::Address TargetCurrent{0x2014, 0};    // Custom
  static constexpr modm_canopen::Address CurrentError{0x2012, 0};     // Custom
  static constexpr modm_canopen::Address CommandedCurrent{0x2016, 0}; // Custom
  static constexpr modm_canopen::Address FilteredActualCurrent{0x2018,
                                                               0}; // Custom

  static constexpr modm_canopen::Address DefaultPWM{0x2010, 0};  // Custom
};
