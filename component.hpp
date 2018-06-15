
#ifndef ROBOT_COMPONENT_HPP
#define	ROBOT_COMPONENT_HPP

#include <modm/communication/xpcc/abstract_component.hpp>

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
			const xpcc::ResponseHandle& handler);
};
}

#endif	// ROBOT_COMPONENT_HPP
