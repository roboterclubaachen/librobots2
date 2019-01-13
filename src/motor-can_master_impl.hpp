/* alpha_motor.hpp
 *
 * Copyright (C) 2019 Raphael Lehmann <raphael@rleh.de>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MOTOR_CAN_MASTER_HPP
#error "Do not include this file directly. Include motor-can_master.hpp instead."
#endif

namespace motorCan
{

template < typename CAN >
std::array<typename MotorCanMaster< CAN >::DataTx, Configuration::BoardCount>
MotorCanMaster< CAN >::dataTx;

template < typename CAN >
std::array<typename MotorCanMaster< CAN >::DataRx, Configuration::BoardCount>
MotorCanMaster< CAN >::dataRx;

template < typename CAN >
std::array<modm::ShortTimeout, Configuration::BoardCount>
MotorCanMaster< CAN >::aliveTimer;


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
				aliveTimer[idx].restart(100);
			}
		}
	}
}

template < typename CAN >
void
MotorCanMaster< CAN >::transmit()
{
	modm::atomic::Lock lock;
	for (std::size_t idx = 0; idx < Configuration::MotorCount; ++idx)
	{
		// Id, length
		modm::can::Message msg(idx + Configuration::base_id, sizeof(DataTx));
		msg.setExtended(false);
		msg.data = dataTx[idx].toMessageData();

		bool ret = CAN::sendMessage(msg);
		if (ret == false) {
	//			LedRed::set();
		}
		else {
	//			LedGreen::set();
		}
	}
}

template < typename CAN >
template < uint8_t MotorId >
void
MotorCanMaster< CAN >::disable()
{
	// Access to _pwm member must be locked
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		dataTx[MotorId / 2].pwmM1 = 0;
	}
	else {
		dataTx[MotorId / 2].pwmM2 = 0;
	}

}

template < typename CAN >
template < uint8_t MotorId >
void
MotorCanMaster< CAN >::setPwm(int16_t pwm)
{
	// Access to _pwm member must be locked
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		dataTx[MotorId / 2].pwmM1 = pwm;
	}
	else {
		dataTx[MotorId / 2].pwmM2 = pwm;
	}
}

template < typename CAN >
template < uint8_t MotorId >
void
MotorCanMaster< CAN >::setCurrentLimit(uint16_t currentLimit)
{
	// Access to _currentLimit member must be locked
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		dataTx[MotorId / 2].currentLimitM1 = currentLimit;
	}
	else {
		dataTx[MotorId / 2].currentLimitM2 = currentLimit;
	}
}


template < typename CAN >
template < uint8_t MotorId >
uint16_t
MotorCanMaster< CAN >::getCurrent()
{
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		return dataRx[MotorId / 2].currentM1;
	}
	else {
		return dataRx[MotorId / 2].currentM2;
	}
}

template < typename CAN >
template < uint8_t MotorId >
uint16_t
MotorCanMaster< CAN >::getEncoderCounterRaw()
{
	// Access to _encoder member must be locked
	modm::atomic::Lock lock;

	if constexpr(MotorId % 2 == 0){
		return dataRx[MotorId / 2].encoderCounterRawM1;
	}
	else {
		return dataRx[MotorId / 2].encoderCounterRawM2;
	}
}

} // namespace motorCan
