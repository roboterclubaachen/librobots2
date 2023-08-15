#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

struct PWMObjects {
  static constexpr modm_canopen::Address PWMCommand{0x2002, 0}; // Custom
};