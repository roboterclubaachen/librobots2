
#ifndef ROBOT__COMPONENT_HPP
#define	ROBOT__COMPONENT_HPP

#include <xpcc/communication/xpcc/abstract_component.hpp>

#include "communication/packets.hpp"

namespace robot
{
class Component : public xpcc::AbstractComponent
{
public:
	Component(uint8_t ownIdentifier, xpcc::Dispatcher *communication);

	void
	actionPing(const xpcc::ResponseHandle& handler);

	void
	actionAwake(
			const xpcc::ResponseHandle& handler,
			const robot::packet::TeamColour *payload);
};
}

#endif	// ROBOT__COMPONENT_HPP
