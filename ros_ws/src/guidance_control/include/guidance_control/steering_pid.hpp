#ifndef GUIDANCE_CONTROL__STEERING_PID_HPP_
#define GUIDANCE_CONTROL__STEERING_PID_HPP_

#include <algorithm>
#include <cmath>

namespace dagozilla
{

// A sign-preserving PID core for image horizontal error. With ki=kd=0 this
// returns exactly -kp * error, matching the historical steering behavior.
class SteeringPid
{
public:
  void configure(double kp, double ki, double kd, double output_limit,
    double integral_limit, double dt_min, double dt_max)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    output_limit_ = std::abs(output_limit);
    integral_limit_ = std::abs(integral_limit);
    dt_min_ = std::max(1e-6, dt_min);
    dt_max_ = std::max(dt_min_, dt_max);
  }

  void reset()
  {
    integral_ = 0.0;
    previous_error_ = 0.0;
    has_previous_error_ = false;
  }

  double update(double error, double dt, double & p, double & i, double & d)
  {
    p = kp_ * error;
    i = 0.0;
    d = 0.0;

    if (std::isfinite(dt) && dt >= dt_min_ && dt <= dt_max_) {
      if (ki_ != 0.0) {
        const double candidate = integral_ + error * dt;
        integral_ = std::clamp(candidate, -integral_limit_, integral_limit_);
        i = ki_ * integral_;
      }
      if (kd_ != 0.0 && has_previous_error_) {
        d = kd_ * (error - previous_error_) / dt;
      }
    }

    previous_error_ = error;
    has_previous_error_ = true;
    const double unclamped = -(p + i + d);
    return std::clamp(unclamped, -output_limit_, output_limit_);
  }

private:
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};
  double output_limit_{1.0};
  double integral_limit_{1.0};
  double integral_{0.0};
  double previous_error_{0.0};
  double dt_min_{0.001};
  double dt_max_{1.0};
  bool has_previous_error_{false};
};

}  // namespace dagozilla

#endif  // GUIDANCE_CONTROL__STEERING_PID_HPP_
