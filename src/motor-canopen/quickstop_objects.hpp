#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

struct QuickStopObjects {
  static constexpr modm_canopen::Address QuickStopDeceleration{0x6085,
                                                               0}; // User units
};