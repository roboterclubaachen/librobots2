/* alpha_motor.hpp
 *
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_SLAVE_HPP
#error "Do not include this file directly. Include motor-can_slave.hpp instead."
#endif

#include "motor-can.hpp"
#include "motor-can_slave.hpp"

#undef  MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG

namespace motorCan {

template < typename CAN_BUS >
uint8_t
MotorCanSlave< CAN_BUS >::boardId;

template < typename CAN_BUS >
typename MotorCanSlave< CAN_BUS >::DataToMotor
MotorCanSlave< CAN_BUS >::dataToMotor;

template < typename CAN_BUS >
typename MotorCanSlave< CAN_BUS >::DataFromMotor
MotorCanSlave< CAN_BUS >::dataFromMotor;

template < typename CAN_BUS >
modm::ShortTimeout
MotorCanSlave< CAN_BUS >::aliveTimer;


template < typename CAN_BUS >
void
MotorCanSlave< CAN_BUS >::initialize(uint8_t _boardId)
{
	boardId = boardId;

	using CanFilter = modm::platform::CanFilter;

	// Only receive sync message
	CanFilter::setFilter(0, CanFilter::FIFO0,
			CanFilter::StandardIdentifier(0),
			CanFilter::StandardFilterMask(0x7ff));

	MODM_LOG_DEBUG << "[motorCan] boardId = " << boardId << modm::endl;
	MODM_LOG_DEBUG << "[motorCan] Listening to messageId = " << (Configuration::base_id + boardId) << modm::endl;

	CanFilter::setFilter(0, CanFilter::FIFO1,
			CanFilter::StandardIdentifier(Configuration::sync_id),
			CanFilter::StandardFilterMask(0x7ff));
	CanFilter::setFilter(1, CanFilter::FIFO0,
			CanFilter::StandardIdentifier(Configuration::base_id + boardId),
			CanFilter::StandardFilterMask(0x7ff));

	aliveTimer.restart(0);
}

// ----------------------------------------------------------------------------

template < typename CAN_BUS >
void
MotorCanSlave< CAN_BUS >::update()
{
	receive();
}

// ----------------------------------------------------------------------------

template < typename CAN_BUS >
void
MotorCanSlave< CAN_BUS >::sampleMotors() {
	//_dataFromMotor[bridge].encoderCounterRaw = IMotorBoard::getRawEncoderValue(bridge);
	//_dataFromMotor[bridge].current		   = IMotorBoard::getCurrent(bridge);
	//_dataFromMotor[bridge].statusFlags	   = IMotorBoard::isCurrentOverLimit(bridge);
	// TODO (alpha-motor)
};

// ----------------------------------------------------------------------------

template < typename CAN_BUS >
void
MotorCanSlave< CAN_BUS >::updateMotors() {
	//IMotorBoard::setPwm(		 bridge, _dataToMotor[bridge].pwm);
	//IMotorBoard::setCurrentLimit(bridge, _dataToMotor[bridge].currentLimit);
	// TODO (alpha-motor)
};

// ----------------------------------------------------------------------------

template < typename CAN_BUS >
void
MotorCanSlave< CAN_BUS >::receive()
{
	while (CAN_BUS::isMessageAvailable())
	{
		modm::can::Message message;
		CAN_BUS::getMessage(message);

		// Check if sync packet received
		if ((message.identifier == Configuration::sync_id) && (message.length == Configuration::sync_length)) {

			// Sample synchronously
			sampleMotors(); // Hardware dependent

			// Then reply instantly
			// No collisions occur due to the arbitration mechanism of CAN_BUS bus
			modm::can::Message msg(Configuration::base_id_reply + boardId, sizeof(DataFromMotor));
			msg.setExtended(false);
			msg.data = dataFromMotor.toMessageData();
			bool ret = CAN_BUS::sendMessage(msg);
			if (ret == false) {
				// ToDo Some error handling / showing
			}

		}
		else
		{
			// Check if new motor data was received
			// CAN filtering guarantees that only messages with motors on the board are received.
			if ((message.identifier >=  Configuration::base_id + boardId) && (message.length == sizeof(DataToMotor)))
			{

				dataToMotor.updateFromMessageData(message.data);

				updateMotors();

				// Valid frame received, motorCan master seems to be up
				aliveTimer.restart(100);
			}
		}
	}
}


} // namespace motorCan

// ----------------------------------------------------------------------------
// Must be visible to IRQ Handler
//typedef MotorCanSlave< Can1 > MyMotorHighSpeedCommunicationSlaveNode;
