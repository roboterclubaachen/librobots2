/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_MASTER_HPP
#define MOTOR_CAN_MASTER_HPP

#include "motor_configuration.hpp"

#include <array>
#include <modm/processing/timer/timeout.hpp>
#include <modm/architecture/interface/can.hpp>
#include <modm/architecture/interface/atomic_lock.hpp>

namespace motorCan
{

/**
 * Communicate at high speed with all motor connected by High Speed CAN.
 *
 * sendSync() must be called at a frequency of 2 milliseconds, preferrably from an
 * interrupt.
 * update() must be called as often as possible to process incoming messages.
 *
 * Template parameter CAN must be a CAN peripheral. It must be already initialized
 * and the filter must be set to receive all messages from the bus.
 *
 *
 * E.g.:
 * 	// Receive every message
 * 	CanFilter::setFilter(0, CanFilter::FIFO0,
 * 			CanFilter::StandardIdentifier(0),
 * 			CanFilter::StandardFilterMask(0));
 */
template < typename CAN >
class MotorCanMaster
{
public:
	static void
	initialize();

	/**
	 * Process all received messages and update internal variables.
	 */
	static void
	update();

	// Send sync packet (CAN Id 0) to sample all remote encoders at the same time.
	// After sampling all remote motors begin transmitting their actual values to
	// the master.
	//
	// Check @areResponsesOutstanding() to see if all responses arrived.
	static void
	sendSync();

	// Start transmitting all new values to motors over CAN bus
	static inline void
	transmit();

public:
	// Shadow registers of PWM, encoder, current (target, actual)
	class DataTx
	{
	public:
		void toMessageData(uint8_t* a) const
		{
			a[0] = pwmM1 >> 8;
			a[1] = pwmM1 & 0xff;
			a[2] = pwmM2 >> 8;
			a[3] = pwmM2 & 0xff;
			a[4] = currentLimitM1 >> 8;
			a[5] = currentLimitM1 & 0xff;
			a[6] = currentLimitM2 >> 8;
			a[7] = currentLimitM2 & 0xff;
		}

	public:
		// Pulse-width ratio of 10 bit PWM register
		// -1023 .. -1: negative PWM
		//           0: no PWM
		// +1 .. +1023: positive PWM
		int16_t
		pwmM1;
		int16_t
		pwmM2;

		// Current limit that is limited by the motor hardware
		uint16_t
		currentLimitM1;
		uint16_t
		currentLimitM2;
	} __attribute__((packed));

	class DataRx
	{
	public:
		// assignment operator
		// T& T::operator =(const T2& b);
		//DataRx(std::array<uint8_t, 8> data)
		void operator=(const uint8_t* data)
		{
			encoderCounterRawM1 = (data[0] << 8) | data[1];
			encoderCounterRawM2 = (data[2] << 8) | data[3];
			currentM1 = (data[4] << 8) | data[5];
			currentM2 = (data[6] << 8) | data[7];
		}

	public:
		uint16_t
		encoderCounterRawM1;
		uint16_t
		encoderCounterRawM2;

		// Actual combined motor current in milliamperes
		// Negative current means regenerative breaking
		int16_t
		currentM1;
		int16_t
		currentM2;
	} __attribute__((packed));

	static_assert(sizeof(DataTx) <= 8, "DataTx struct is too large for a CAN message.");
	static_assert(sizeof(DataRx) <= 8, "DataRx struct is too large for a CAN message.");

	static inline std::array<DataTx, Configuration::BoardCount>
	dataTx;

	static inline std::array<DataRx, Configuration::BoardCount>
	dataRx;

private:
	static inline std::array<modm::ShortTimeout, Configuration::BoardCount>
	aliveTimer;
};

} // namespace motorCan

#include "master_impl.hpp"

#endif // MOTOR_CAN_MASTER_HPP
