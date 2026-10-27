#ifndef CABOT_DNN_CONTROLLER__COMMAND_ACCELERATION_LIMITER_HPP_
#define CABOT_DNN_CONTROLLER__COMMAND_ACCELERATION_LIMITER_HPP_

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace cabot_dnn_controller
{

// Limit base_footprint commands [m/s, rad/s] after maximum-velocity scaling and
// before odom_adapter's wheel-center compensation. Bounds are [m/s^2, rad/s^2].
// Use the previous limited command and actual ROS timestamp difference [ns].
// Share one normalized acceleration budget between translation and rotation.
// Require |dv| / max_linear_acceleration + |dw| / max_angular_acceleration <= dt.
// Scaling both changes together reduces simultaneous wheel acceleration demands
// after offset compensation; the downstream wheel dynamics remain unchanged.
// Initial/reversed-time output is zero; equal timestamps hold output.
// Normal zero requests decelerate; callers reset on immediate stops/pauses.
class CommandAccelerationLimiter
{
public:
  struct Result
  {
    double linear;
    double angular;
    double dt;
    bool limited;
    bool reset;
  };

  void reset()
  {
    initialized_ = false;
    previous_linear_ = 0.0;
    previous_angular_ = 0.0;
  }

  Result limit(
    double linear, double angular, std::int64_t stamp_ns,
    double max_linear_acceleration = 1.0, double max_angular_acceleration = 2.0)
  {
    if (!std::isfinite(max_linear_acceleration) || max_linear_acceleration <= 0.0 ||
      !std::isfinite(max_angular_acceleration) || max_angular_acceleration <= 0.0)
    {
      throw std::invalid_argument("Acceleration limits must be finite and positive");
    }
    if (!std::isfinite(linear) || !std::isfinite(angular)) {
      reset();
      return {0.0, 0.0, 0.0, true, true};
    }

    const bool restart = !initialized_ || stamp_ns < previous_stamp_ns_;
    if (restart) {
      reset();
      initialized_ = true;
      previous_stamp_ns_ = stamp_ns;
      return {0.0, 0.0, 0.0, linear != 0.0 || angular != 0.0, true};
    }

    const double dt = static_cast<double>(stamp_ns - previous_stamp_ns_) * 1e-9;
    const double linear_delta = linear - previous_linear_;
    const double angular_delta = angular - previous_angular_;
    const double required_dt =
      std::abs(linear_delta) / max_linear_acceleration +
      std::abs(angular_delta) / max_angular_acceleration;
    const bool limited = required_dt > dt;
    const double scale = limited ? dt / required_dt : 1.0;
    const double limited_linear = limited ? previous_linear_ + scale * linear_delta : linear;
    const double limited_angular = limited ? previous_angular_ + scale * angular_delta : angular;
    previous_linear_ = limited_linear;
    previous_angular_ = limited_angular;
    previous_stamp_ns_ = stamp_ns;
    return {limited_linear, limited_angular, dt, limited, false};
  }

private:
  bool initialized_{false};
  std::int64_t previous_stamp_ns_{0};
  double previous_linear_{0.0};
  double previous_angular_{0.0};
};

}  // namespace cabot_dnn_controller

#endif  // CABOT_DNN_CONTROLLER__COMMAND_ACCELERATION_LIMITER_HPP_
