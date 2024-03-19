#ifndef MOTOR_STATE_HPP
#define MOTOR_STATE_HPP
#include <modm-canopen/cia402/factors.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>
#include <modm-canopen/object_dictionary_common.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/container/deque.hpp>
#include <modm/math/filter/moving_average.hpp>

#include "state_objects.hpp"
#include "identity.hpp"

using OperatingMode = modm_canopen::cia402::OperatingMode;
using StateMachine = modm_canopen::cia402::StateMachine;
using StatusBits = modm_canopen::cia402::StatusBits;
using ControlWord = modm_canopen::cia402::CommandWord;
using Factors = modm_canopen::cia402::Factors;

using namespace std::literals;

constexpr modm::Clock::duration controlTiming_{1ms};

template<size_t id>
struct MotorState
{
	static inline Identity identity_{.deviceType_ = DeviceType::BLDC,
									 .productCode_ = ProductCode::MicroMotor};

	static inline OperatingMode mode_{OperatingMode::Disabled};
	static inline StateMachine status_{modm_canopen::cia402::State::SwitchOnDisabled};
	static inline ControlWord control_{0};
	static inline Factors scalingFactors_{};

	static inline modm::filter::MovingAverage<uint32_t, 32> updateTime_us_{};
	static inline modm::chrono::micro_clock::duration lastUpdateTime_{};
	static inline modm::chrono::micro_clock::time_point lastUpdate_{};

	static inline int32_t actualPosition_{};
	static inline int32_t lastPosition_{};

	static inline float orientedCurrent_{};
	static inline float unorientedCurrent_{};
	static inline float orientedCurrentAngleDiff_{};
	static inline float maxCurrent_{6.0f};

	static constexpr uint16_t zeroAverageCountdownReset_{256};
	static inline uint16_t zeroAverageCountdown_{zeroAverageCountdownReset_};
	static inline modm::filter::MovingAverage<float, 16> zeroAverage_{};

	static inline float maxCharge_{400.0f};
	static inline modm::BoundedDeque<std::pair<float, float>, 256> currentValues_{};
	static inline float currentCharge_{0.0f};

	static inline modm::filter::MovingAverage<int32_t, 512> actualVelocity_{};

	static inline bool enableMotor_{true};
	static inline bool resetMotor_{false};

	static inline int16_t outputPWM_{};
	static inline float outputCurrentLimit_{};

	static inline float
	getCharge();

	static inline void
	setActualPosition(int32_t position);

	static inline void
	setOrientedCurrent(float current);

	static inline void
	setOrientedCurrentAngleDiff(float angle);

	static inline void
	setUnorientedCurrent(float current);

	static inline int16_t
	outputPWM();

	static inline float
	currentLimit();

	static inline float
	maxCurrent();

	template<typename Device, typename MessageCallback>
	static bool
	update(MessageCallback &&cb);

	template<typename ObjectDictionary>
	static constexpr void
	registerHandlers(modm_canopen::HandlerMap<ObjectDictionary> &map);
};

#include "motor_state_impl.hpp"
#endif