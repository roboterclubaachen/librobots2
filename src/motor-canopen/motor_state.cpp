#include "motor_state.hpp"

float MotorState::getCharge() const {
  float acc = 0.0f;
  for (auto &pair : currentValues_) {
    acc += pair.first * pair.second;
  }
  return acc;
}