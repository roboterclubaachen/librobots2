#ifndef ERROR_PROTOCOL_HPP
#error "Do not include this file directly, use error_protocol.hpp instead"
#endif
#include <cmath>

template <size_t id>
template <typename Device,typename State,  typename MessageCallback>
bool ErrorProtocol<id>::update(MessageCallback &&) {
  pwmChange_.update(State::outputPWM_ - lastPWM_);
  if (std::abs(pwmChange_.getValue()) > pwmWindow_ &&
      std::abs(State::actualVelocity_.getValue()) < stallVelocity_) {
    errorCounter_++;
    if (errorCounter_ > maxErrorTime_ &&
        State::status_.state() != modm_canopen::cia402::State::Fault) {
      // TODO implement this in statemachine
      auto stateWord_ = State::status_.status();
      constexpr uint16_t faultMask_ = 0b0100'1111;
      constexpr uint16_t faultValue_ = 0b0000'1000;
      stateWord_ = (stateWord_ & ~faultMask_) | (faultValue_ & faultMask_);
      State::status_.set(stateWord_);
      MODM_LOG_ERROR << "Detected Fault!" << modm::endl;
    }
  } else {
    errorCounter_ = 0;
  }
  return true;
}