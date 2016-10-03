// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------


#ifndef XPCC_MOTION_STM32_ENCODER_HPP
#define XPCC_MOTION_STM32_ENCODER_HPP


/* Example Hardware Configuration
 * ==============================
 *  struct EncoderAConfiguration {
 *  	typedef GpioChannelB4 ChannelA;
 *  	typedef GpioChannelB5 ChannelB;
 *  	static constexpr xpcc::stm32::Gpio::InputType ChannelAInputType
 *  		= xpcc::stm32::Gpio::InputType::Floating;
 *  	static constexpr xpcc::stm32::Gpio::InputType ChannelBInputType
 *  		= xpcc::stm32::Gpio::InputType::Floating;
 *  	typedef Timer xpcc::stm32::Timer3;
 *  }
*/
namespace xpcc
{
namespace motion
{
namespace stm32
{

/**
 * Use a Timer of STM32 in slave mode to read a quadrature encoder.
 *
 * Configuration parameter Timer must be one of the Advanced Timers.
 * Template parameter ChannelA and ChannelB must be channel 1 and 2 pins
 * of the corresponding timer connected to signal A and B, respectively, 
 * of the quadrature encoder.
 *
 * run() must be called on a regular basis and the getEncoderRaw() returns
 * the current counter value.
 *
 * This encoder can then be used in encoder differentiation.
 *
 * Example:
 *	struct EncoderConfiguration {
 *		typedef Timer1          Timer;
 *		typedef GpioInputA8     ChannelA;
 *		static constexpr Gpio::InputType ChannelAInputType =
 *			Gpio::InputType::Floating;	// there are external pullups to 5V
 *		typedef GpioInputA9     ChannelB;
 *		static constexpr Gpio::InputType ChannelBInputType =
 *			Gpio::InputType::Floating;	// there are external pullups to 5V
 *	};
 *
 * typedef Encoder< EncoderConfiguration > Stm32EncoderLeft;
 *
 * Run once:
 *   initialize();
 *
 * Run on regular basis (e.g. 1 ms in interrupt handler)
 * 	run()
 *
 * Then use 
 *  getCounterRaw()
 * to get the latest counter value
 */

template<typename Configuration>
class Encoder
{
private:
	/// gpio pin connected to channel A
	typedef typename Configuration::ChannelA ChannelA;
	/// gpio pin connected to channel B
	typedef typename Configuration::ChannelB ChannelB;
	/// input configuration of channel A
	static constexpr xpcc::stm32::Gpio::InputType ChannelAInputType
		= Configuration::ChannelAInputType;
	/// input configuration of channel B
	static constexpr xpcc::stm32::Gpio::InputType ChannelBInputType
		= Configuration::ChannelAInputType;
	/// advanced or general purpose timer that will be conncted to inputs A and B
	typedef typename Configuration::Timer Timer;

public:
	typedef uint16_t CounterT;

	/// Intializes the timer and pins in encoder mode.
	static inline void
	initialize()
	{
		Timer::enable();
		Timer::setMode(Timer::Mode::UpCounter, Timer::SlaveMode::Encoder3);
		// Overflow must be 16bit because else a lot of our motor control code will break!
		Timer::setOverflow(0xffff);
		ChannelA::connect(Timer::Channel1, ChannelAInputType);
		ChannelB::connect(Timer::Channel2, ChannelBInputType);
		Timer::start();
	};

	// Sample the counter value to member variable
	static inline void
	run()
	{
		counterRaw = Timer::getValue();
	}

	// Get the raw counter value from last sample point
	static inline CounterT
	getCounterRaw()
	{
		return counterRaw;
	}

private:
	static
	CounterT counterRaw;
};

template<typename Configuration>
typename Encoder<Configuration>::CounterT
Encoder<Configuration>::counterRaw;

}	// namespace stm32
}	// namespace motion
}	// namespace xpcc

#endif // XPCC_MOTION_STM32_ENCODER_HPP
