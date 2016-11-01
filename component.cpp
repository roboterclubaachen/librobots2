
#include "component.hpp"

// ----------------------------------------------------------------------------
robot::Component::Component(uint8_t ownIdentifier, xpcc::Dispatcher *communication) : 
	xpcc::AbstractComponent(ownIdentifier, *communication)
{
}

// ----------------------------------------------------------------------------
void
robot::Component::actionPing(const xpcc::ResponseHandle& handler)
{
	this->sendNegativeResponse(handler);
}

// ----------------------------------------------------------------------------
void
robot::Component::actionAwake(const xpcc::ResponseHandle& handler)
{
	this->sendNegativeResponse(handler);
}
