/*
 * Copyright (c) 2015, Kevin Läufer
 * Copyright (c) 2018, Niklas Hauser
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef	ROBOT_CONVERSION_HPP
#define	ROBOT_CONVERSION_HPP

#include <modm/math/geometry/vector.hpp>
#include <modm/math/geometry/location_2d.hpp>
#include <modm/math/geometry/line_2d.hpp>
#include <modm/math/geometry/line_segment_2d.hpp>

#include "communication/packets.hpp"

namespace robot
{
	/// conversion interface
//	template <typename T, typename U>
//	inline T
//	convert(const U&);
	
	// robot::packet::Position <-> modm::Vector2D<T>
	template <typename T>
	inline modm::Vector<T, 2>
	convert(const robot::packet::Position& in)
	{
		return modm::Vector<T, 2>(in.x, in.y);
	}
	
	template <typename U>
	inline robot::packet::Position
	convert(const modm::Vector<U, 2>& in)
	{
		return robot::packet::Position(in.getX(), in.getY());
	}
	
	// robot::packet::Location <-> modm::Location2D<T>
	template <typename T>
	inline modm::Location2D<T>
	convert(const robot::packet::Location& in)
	{
		return modm::Location2D<T>(in.x, in.y, in.phi);
	}
	
	template <typename U>
	inline robot::packet::Location
	convert(const modm::Location2D<U>& in)
	{
		return robot::packet::Location(
				in.getPosition().getX(),
				in.getPosition().getY(),
				in.getOrientation());
	}
	
	// robot::packet::Line <-> modm::Line2D<T>
	template <typename T>
	inline modm::LineSegment2D<T>
	convert(const robot::packet::Line& in)
	{
		return modm::LineSegment2D<T>(
				modm::Vector<T, 2>( in.start.x, in.start.y ),
				modm::Vector<T, 2>( in.end.x, in.end.y ) );
	}

	template <typename U>
	inline robot::packet::Line
	convert(const modm::Line2D<U>& in)
	{
		return robot::packet::Line(
				in.getPoint().getX(),
				in.getPoint().getY(),
				in.getPoint().getX() + in.getDirectionVector().getX(),
				in.getPoint().getY() + in.getDirectionVector().getY());
	}

	template <typename U>
	inline robot::packet::Line
	convert(const modm::LineSegment2D<U>& in)
	{
		return robot::packet::Line(
				in.getStartPoint().getX(),
				in.getStartPoint().getY(),
				in.getEndPoint().getX(),
				in.getEndPoint().getY());
	}
} // robot namespace

inline bool
operator == (const robot::packet::ViewingDirection& lhs, const robot::packet::DriveDirection& rhs)
{
	if (((lhs == robot::packet::ViewingDirection::Forward ) && (rhs == robot::packet::DriveDirection::Forward)) ||
		((lhs == robot::packet::ViewingDirection::Backward) && (rhs == robot::packet::DriveDirection::Backward)))
	{
		return true;
	} else {
		return false;
	}
}

inline bool
operator != (const robot::packet::ViewingDirection& lhs, const robot::packet::DriveDirection& rhs)
{
	return not (lhs == rhs);
}

inline bool
operator ^ (const robot::packet::ViewingDirection& lhs, const robot::packet::DriveDirection& rhs)
{
	if (((lhs == robot::packet::ViewingDirection::Forward ) && (rhs == robot::packet::DriveDirection::Forward)) ||
		((lhs == robot::packet::ViewingDirection::Backward) && (rhs == robot::packet::DriveDirection::Backward)) )
	{
		return false;
	} else {
		return true;
	}
}

inline bool operator^(const robot::packet::DriveDirection& lhs, const robot::packet::ViewingDirection& rhs)
{
	return rhs ^ lhs;
}

// Swap Forward with Backward, keep Automatically
inline robot::packet::DriveDirection operator!(const robot::packet::DriveDirection& lhs)
{
	switch (lhs)
	{
		case robot::packet::DriveDirection::Forward:
			return robot::packet::DriveDirection::Backward;
			break;
		case robot::packet::DriveDirection::Backward:
			return robot::packet::DriveDirection::Forward;
			break;
		default:
			__builtin_unreachable();
	}
}

// Swap Forward with Backward
inline robot::packet::ViewingDirection operator !(const robot::packet::ViewingDirection & lhs){
	switch (lhs)
	{
		case robot::packet::ViewingDirection::Forward:
			return robot::packet::ViewingDirection::Backward;
			break;
		case robot::packet::ViewingDirection::Backward:
			return robot::packet::ViewingDirection::Forward;
			break;
		default:
			__builtin_unreachable();
	}
}

#endif // ROBOT_CONVERSION_HPP
