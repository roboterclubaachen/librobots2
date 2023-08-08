#include "motor_simulation.hpp"
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <thread>

using namespace std::literals;

#include <modm/debug/logger.hpp>

void SimMotor::setSupplyVoltage(float v) { supplyVoltage_V = v; }

float SimMotor::getRotorAcc() const { return state.shaft_acceleration_rad_s2; }

float SimMotor::getRotorVel() const { return state.shaft_speed_rad_s; }

float SimMotor::getRotorPos() const { return state.shaft_angle_rad; }

float SimMotor::getPhaseCurrent(Phase phase) const {
  return state.phaseCurrent_a[getPhaseIndex(phase)];
}

void SimMotor::setPhasePWM(Phase phase, uint16_t pwm) {
  const auto phaseIndex = getPhaseIndex(phase);
  state.phasePWM[phaseIndex] = pwm;
}

uint8_t SimMotor::getPhaseIndex(Phase phase) {
  uint8_t phaseIndex = 0;
  switch (phase) {
  case Phase::PhaseU:
    break;
  case Phase::PhaseV:
    phaseIndex = 1;
    break;
  case Phase::PhaseW:
    phaseIndex = 2;
    break;
  default: // TODO support more than 3 phases?
    break;
  }
  return phaseIndex;
}

float SimMotor::getDampingTorque_kg_m(float linear_damping_kg_m_s_rad,
                                      float shaft_speed_rad_s) {
  return linear_damping_kg_m_s_rad * shaft_speed_rad_s;
}

float SimMotor::getDetentTorque_kg_m(float detent_torque_kg_m,
                                     uint8_t num_stator_phases,
                                     uint8_t num_rotor_poles,
                                     float shaft_position_rad) {

  return detent_torque_kg_m *
         std::sin(num_stator_phases * num_rotor_poles * shaft_position_rad);
}

float SimMotor::applyDrag_kg_m(float phaseTorque_kg_m, float drag_kg_m) {
  if (phaseTorque_kg_m >= 0.0f) {
    if (phaseTorque_kg_m < drag_kg_m) {
      return 0.0f;
    }
    return phaseTorque_kg_m - drag_kg_m;
  } else {
    if (phaseTorque_kg_m > -drag_kg_m) {
      return 0.0f;
    }
    return phaseTorque_kg_m + drag_kg_m;
  }
}

void SimMotor::update(float timestep_s) {
  for (size_t i = 0; i < phaseCount; i++) {
    state.forwardVoltage_V[i] =
        computePhaseVoltage_V(i, supplyVoltage_V, state, properties);
    state.backEMF_V[i] = computeBackEMF_V(i, state, properties);
    state.phaseCurrent_a[i] =
        computePhaseCurrent_a(i, timestep_s, state, properties);
  }

  state.electricTorque_kg_m = computeElectricTorque_kg_m(state);
  state.shaft_acceleration_rad_s2 =
      computeAcceleration_rad_s2(state, properties);
  state.shaft_speed_rad_s += getRotorAcc() * timestep_s;
  state.shaft_angle_rad += getRotorVel() * timestep_s;

  MODM_LOG_INFO << state.electricTorque_kg_m << ":"
                << state.shaft_acceleration_rad_s2 << ":" << modm::endl;
  std::this_thread::sleep_for(100ms);
}
void SimMotor::setPhaseConfig(Phase phase, PhaseConfig config) {
  const auto phaseIndex = getPhaseIndex(phase);
  state.phaseConfigs[phaseIndex] = config;
}

float SimMotor::computePhaseVoltage_V(uint8_t phaseIndex, float supplyVoltage_V,
                                      const MotorState_t<phaseCount> &state,
                                      const MotorProperties &properties) {
  float phasePotentialIn_V = 0.0f;
  switch (state.phaseConfigs[phaseIndex]) {
  case PhaseConfig::HiZ:
    // TODO implement?
    break;
  case PhaseConfig::Low:
    break;
  case PhaseConfig::High:
    phasePotentialIn_V = supplyVoltage_V;
    break;
  case PhaseConfig::Pwm:
    phasePotentialIn_V = supplyVoltage_V * (float)state.phasePWM[phaseIndex] /
                         (float)std::numeric_limits<uint16_t>::max();
    break;
  }
  return phasePotentialIn_V;
}

float SimMotor::computeBackEMF_V(uint8_t phaseIndex,
                                 const MotorState_t<phaseCount> &state,
                                 const MotorProperties &properties) {
  const auto phaseOffset = 2 * std::numbers::pi_v<float> / phaseCount;
  const auto factor =
      std::sin(state.shaft_angle_rad - phaseIndex * phaseOffset);
  const auto backEMF_V =
      -state.shaft_speed_rad_s * properties.back_emf_constant_v_s_rad * factor;
  return backEMF_V;
}

float SimMotor::computePhaseCurrent_a(uint8_t phaseIndex, float timestep_s,
                                      const MotorState_t<phaseCount> &state,
                                      const MotorProperties &properties) {
  const float inductance_factor =
      1 / (properties.phaseInductance_h - properties.phaseInductance_h *
                                              properties.phaseInductance_h *
                                              properties.phaseCouplingFactor);
  const float resistanceDrop_V =
      properties.phaseResistance_Ohm * state.phaseCurrent_a[phaseIndex];

  const float currentChange_a_s =
      inductance_factor * (state.forwardVoltage_V[phaseIndex] -
                           resistanceDrop_V + state.backEMF_V[phaseIndex]);
  return state.phaseCurrent_a[phaseIndex] + currentChange_a_s * timestep_s;
}

float SimMotor::computeElectricTorque_kg_m(
    const MotorState_t<phaseCount> &state) {
  float value = 0.0f;
  for (size_t i = 0; i < phaseCount; i++) {
    value += state.phaseCurrent_a[i] * state.backEMF_V[i];
  }
  if (state.shaft_speed_rad_s != 0.0f) {
    value /= state.shaft_speed_rad_s;
  } else {
    return 0.0f;
  }
  return value;
}

float SimMotor::computeAcceleration_rad_s2(
    const MotorState_t<phaseCount> &state, const MotorProperties &properties) {
  const float torque_kg_m =
      state.electricTorque_kg_m -
      getDampingTorque_kg_m(properties.lin_damping_kg_m_s_rad,
                            state.shaft_speed_rad_s) -
      getDetentTorque_kg_m(properties.detent_torque_kg_m,
                           properties.num_stator_phases,
                           properties.num_rotor_poles, state.shaft_angle_rad);
  const float dragged_torque_kg_m =
      applyDrag_kg_m(torque_kg_m, properties.const_damping_kg_m);
  return dragged_torque_kg_m / properties.inertia_kg_m_s2_rad;
}
