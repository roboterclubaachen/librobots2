#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

struct HeartbeatObjects {
  static constexpr modm_canopen::Address TimeBetweenHeartbeats{0x1017, 0};
  static constexpr modm_canopen::Address MasterHeartbeatTimeout{0x2009, 0};
};