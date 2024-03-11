#ifndef IDENTITY_PROTOCOL_HPP
#error "Do not include this file directly, use identity_protocol.hpp instead"
#endif

#include <modm/debug/logger.hpp>

template<size_t id>
template<typename ObjectDictionary, typename State>
constexpr void
IdentityProtocol<id>::registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map)
{
	map.template setReadHandler<IdentityObjects::DeviceType>(
		+[]() { return (uint32_t)State::identity_.deviceType_; });
	map.template setReadHandler<IdentityObjects::ErrorRegister>(
		+[]() { return (uint8_t)State::identity_.errorRegister_; });
	map.template setReadHandler<IdentityObjects::IdentityObject>(
		+[]() { return (uint8_t)State::identity_.identityObject_; });
	map.template setReadHandler<IdentityObjects::VendorId>(
		+[]() { return (uint32_t)State::identity_.vendorId_; });
	map.template setReadHandler<IdentityObjects::ProductCode>(
		+[]() { return (uint32_t)State::identity_.productCode_; });
	map.template setReadHandler<IdentityObjects::RevisionId>(
		+[]() { return (uint32_t)State::identity_.revisionId_; });
	map.template setReadHandler<IdentityObjects::SerialNumber>(
		+[]() { return (uint32_t)State::identity_.serialNumber_; });
}