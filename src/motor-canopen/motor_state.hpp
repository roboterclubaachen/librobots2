#pragma once
#include <modm-canopen/cia402/factors.hpp>
#include <modm-canopen/cia402/operating_mode.hpp>
#include <modm-canopen/cia402/state_machine.hpp>
#include <modm-canopen/object_dictionary_common.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/math/filter/moving_average.hpp>

#include "state_objects.hpp"

using OperatingMode = modm_canopen::cia402::OperatingMode;
using StateMachine = modm_canopen::cia402::StateMachine;
using ControlWord = modm_canopen::cia402::CommandWord;
using Factors = modm_canopen::cia402::Factors;

struct MotorState {
  OperatingMode mode_{OperatingMode::Disabled};
  StateMachine status_{modm_canopen::cia402::State::SwitchOnDisabled};
  ControlWord control_{0};
  Factors scalingFactors_{};

  modm::filter::MovingAverage<uint32_t, 32> updateTime_{};
  modm::chrono::micro_clock::time_point lastUpdate_{};

  int32_t actualPosition_{};
  int32_t lastPosition_{};

  float orientedCurrent_{};
  float unorientedCurrent_{};
  float orientedCurrentAngle_{};
  float maxCurrent_{3.0f};
  float maxCharge_{400.0f};

  modm::filter::MovingAverage<int32_t, 16> actualVelocity_{};

  bool enableMotor_{true};
  bool resetMotor_{false};

  modm::Clock::time_point lastExecute_;
  modm::Clock::duration lastExecutionTime_;

  int16_t outputPWM_{};
};