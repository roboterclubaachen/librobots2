/*
 * Copyright (c) 2015, 2018, Niklas Hauser
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef	ROBOT_RESULT_HPP
#define	ROBOT_RESULT_HPP

#include "communication/packets.hpp"
#include <modm/communication/xpcc/postman/response.hpp>

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
