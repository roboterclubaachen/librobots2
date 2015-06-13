#ifndef	ROBOT_RESULT_HPP
#define	ROBOT_RESULT_HPP

#include "communication/packets.hpp"

namespace robot
{

class BooleanResult
{
public:
	BooleanResult( xpcc::ActionResult<robot::packet::Bool*> result) :
		success(result.response == xpcc::Response::Positive and *result.data)
	{
	}

	operator bool () const
	{
		return success;
	}

private:
	bool success;
};

}	// namespace robot

#endif // ROBOT_RESULT_HPP
