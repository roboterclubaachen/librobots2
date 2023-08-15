#pragma once
#include <modm-canopen/object_dictionary_common.hpp>

struct IdentityObjects {
  static constexpr modm_canopen::Address DeviceType{0x1000, 0};
  static constexpr modm_canopen::Address ErrorRegister{0x1001, 0};
  static constexpr modm_canopen::Address IdentityObject{0x1018, 0};
  static constexpr modm_canopen::Address VendorId{0x1018, 1};
  static constexpr modm_canopen::Address ProductCode{0x1018, 2};
  static constexpr modm_canopen::Address RevisionId{0x1018, 3};
  static constexpr modm_canopen::Address SerialNumber{0x1018, 4};
};
