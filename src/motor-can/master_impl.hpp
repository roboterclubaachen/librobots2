/*
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_MASTER_HPP
#error "Do not include this file directly. Include master.hpp instead."
#endif

using namespace std::chrono_literals;

namespace motorCan
{

template < typename CAN >
void
MotorCanMaster< CAN >::initialize()
{
}

template < typename CAN >
void
MotorCanMaster< CAN >::sendSync()
{
	modm::can::Message msg(Configuration::sync_id, Configuration::sync_length);
	msg.setExtended(false);

	CAN::sendMessage(msg);
}

template < typename CAN >
void
MotorCanMaster< CAN >::update()
{
	modm::atomic::Lock lock;

    // Process all waiting CAN messages
	while (CAN::isMessageAvailable())
	{
		modm::can::Message message;
		CAN::getMessage(message);

		// Check length
		if (message.length == sizeof(DataRx)) {

			// Check ID
			// 0x20 to 0x2f are valid responses from MotorId 0 to 15.
			if (
					(message.identifier >= (Configuration::base_id_reply) &&
					(message.identifier <  (Configuration::base_id_reply + (Configuration::MotorCount))))
				)
			{
				uint8_t idx = message.identifier - Configuration::base_id_reply;

				dataRx[idx] = message.data;

				// Valid frame received, iMotor seems to be up
				aliveTimer[idx].restart(100ms);
			}
		}
	}
}

template < typename CAN >
void
MotorCanMaster< CAN >::transmit()
{
	modm::atomic::Lock lock;
	for (std::size_t idx = 0; idx < Configuration::BoardCount; ++idx)
	{
		// Id, length
		modm::can::Message msg(idx + Configuration::base_id, sizeof(DataTx));
		msg.setExtended(false);
		dataTx[idx].toMessageData(msg.data);

		bool ret = CAN::sendMessage(msg);
		if (ret == false) {
	//			LedRed::set();
		}
		else {
	//			LedGreen::set();
		}
	}
}

} // namespace motorCan
