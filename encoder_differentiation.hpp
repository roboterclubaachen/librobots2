#ifndef ROBOTS_MOTION_ENCODER_DIFFERENTIATION_HPP
#define ROBOTS_MOTION_ENCODER_DIFFERENTIATION_HPP

#include <stdint.h>

namespace robots {
namespace motion {

// Signature for all valid template params
typedef uint16_t(*getEncoderRaw)(void);

/**
 * Interface to an encoder.
 *
 * This template differentiate the steps of an encoder. 
 * It can be used for motors and odometry.
 *
 * Template parameter GER (getEncoderRaw) should be a method that returns the raw value of an encoder counter, 
 * e.g. from a timer in encoder mode or from an unsigned counter in the FPGA.
 *
 * If template parameter REVERSED is set to true the counting direction is reversed.
 *
 * Complete Example:
 *   typedef EncoderDifferentiation< Fpga::getEncoderSteps<Fpga::Odometry0 >, true  > MyOdometryLeft;
 * 
 * Methods MyOdometryLeft::initialize() must be called during startup and 
 * MyOdometryLeft::run() must be called on a regular basis. 
 * 
 * MyOdometryLeft can then be used as a template parameter for class \c Odometry.
 *
 */
template < getEncoderRaw GER, bool REVERSED = false >
class EncoderDifferentiation
{
public:
	static void
	initialize()
	{

	}

	/**
	 * Get the steps of an encoder since last run().
	 *
	 * Returns the number of steps the encoder counted since last run().
	 * Encoder Reversion is considered.
	 * Call run() on a regular basis.
	 */
	static inline int16_t
	getEncoderSteps()
	{
		return encoderSteps;
	}

	/**
	 * Get the counter value of an encoder since last run().
	 *
	 * Returns the counter value of an encoder at the time of the last run().
	 * Encoder Reversion is considered.
	 * Call run() on a regular basis.
	 */
	static inline int16_t
	getEncoderRaw()
	{
		return encoderLast;
	}

	/**
	 * must be called from on a regular basis (e.g. 1ms) at the same frequency as the underlying encoder.
	 * Read raw encoder counter value and calculate number of steps moved since last call
	 */
	static inline void
	run()
	{
		uint16_t encoderCurrent = GER();
		if (REVERSED) {
			encoderCurrent = -encoderCurrent;
		}

		encoderSteps = encoderCurrent - encoderLast;
		encoderLast = encoderCurrent;
	}

private:
	/**
	 * Number of ticks the encoder counted.
	 */
	static int16_t
	encoderSteps;

	/**
	 * Last value of encoder to calculate difference.
	 */
	static uint16_t
	encoderLast;
};

template < getEncoderRaw GER, bool REVERSED >
int16_t EncoderDifferentiation< GER, REVERSED >::encoderSteps = 0;

template < getEncoderRaw GER, bool REVERSED >
uint16_t EncoderDifferentiation< GER, REVERSED >::encoderLast = 0;

}
}

#endif // ROBOTS_MOTION_ENCODER_DIFFERENTIATION_HPP
