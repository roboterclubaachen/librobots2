#pragma once
#include <array>
#include <cstdint>
#include <cstdlib>

#include <librobots2/motor/motor_bridge.hpp>

struct MotorProperties {
  float inertia_kg_m_s2_rad = 0.00005f;   // kg*m*s^2/rad
  float lin_damping_kg_m_s_rad = 0.0f;    // kg*m*s/rad
  float const_damping_kg_m = 0.0f;        // kg*m
  float detent_torque_kg_m = 0.0f;        // kg*m
  uint8_t num_rotor_poles = 3;            // 1
  uint8_t num_stator_phases = 3;          // 1
  float torque_contant_kg_m_a = 1.0f;     // kg*m/A
  float back_emf_constant_v_s_rad = 1.0f; // V*s/rad
  float phaseInductance_h = 0.003f;       // H
  float phaseResistance_Ohm = 0.190f;     // Ohm
  float phaseCouplingFactor = 0.5f;       // 1
};

template <size_t N> struct MotorState_t {
  using PhaseConfig = librobots2::motor::PhaseConfig;
  std::array<PhaseConfig, N> phaseConfigs{};

  float shaft_angle_rad = 0.0f;                               // rad
  float shaft_speed_rad_s = 0.0f;                             // rad/s
  float shaft_acceleration_rad_s2 = 0.0f;                     // rad/s^2
  float electricTorque_kg_m = 0.0f;                           // kg*m
  std::array<uint16_t, N> phasePWM = {0, 0, 0};               // 1
  std::array<float, N> phaseCurrent_a = {0.0f, 0.0f, 0.0f};   // A
  std::array<float, N> backEMF_V = {0.0f, 0.0f, 0.0f};        // V
  std::array<float, N> forwardVoltage_V = {0.0f, 0.0f, 0.0f}; // V
};

class SimMotor {
private:
  static constexpr auto phaseCount = 3;

public:
  using Phase = librobots2::motor::Phase;
  using PhaseConfig = librobots2::motor::PhaseConfig;

  SimMotor() = default;
  ~SimMotor() = default;

  void setSupplyVoltage(float v);                       // Volts
  void setMotorProperties(MotorProperties props);       // Motor Properties
  void setPhasePWM(Phase phase, uint16_t pwm);          // 16-bit PWM
  void setPhaseConfig(Phase phase, PhaseConfig config); // Config
  void update(float timestep_s);                        // Timestep in seconds

  float getRotorPos() const;                // Radian
  float getRotorVel() const;                // Radian per second
  float getRotorAcc() const;                // Radian per second^2
  float getPhaseCurrent(Phase phase) const; // Ampere

private:
  // Motor Properties
  MotorProperties properties;

  // State
  float supplyVoltage_V = 24.0f; // V
  MotorState_t<phaseCount> state;

  // Computation helpers
  static uint8_t getPhaseIndex(Phase phase);

  static float getDampingTorque_kg_m(float linear_damping_kg_m_s_rad,
                                     float shaft_speed_rad_s);

  static float getDetentTorque_kg_m(float detent_torque_kg_m,
                                    uint8_t num_stator_phases,
                                    uint8_t num_rotor_poles,
                                    float shaft_position_rad);

  static float applyDrag_kg_m(float phaseTorque_kg_m, float drag_kg_m);

  static float computePhaseVoltage_V(uint8_t phaseIndex, float supplyVoltage_V,
                                     const MotorState_t<phaseCount> &state,
                                     const MotorProperties &properties);

  static float computeBackEMF_V(uint8_t phaseIndex,
                                const MotorState_t<phaseCount> &state,
                                const MotorProperties &properties);

  static float computePhaseCurrent_a(uint8_t phaseIndex, float timestep_s,
                                     const MotorState_t<phaseCount> &state,
                                     const MotorProperties &properties);

  static float
  computeElectricTorque_kg_m(const MotorState_t<phaseCount> &state);

  static float computeAcceleration_rad_s2(const MotorState_t<phaseCount> &state,
                                          const MotorProperties &properties);
};