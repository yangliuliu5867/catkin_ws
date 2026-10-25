#include <fsm_ctrl/nonlinear_disturbance_observer.hpp>

#include <algorithm>
#include <cmath>

void NonlinearDisturbanceObserver::configure(
    bool enabled,
    const Eigen::Vector3d &bandwidth,
    const Eigen::Vector3d &estimate_limit,
    double gravity)
{
    enabled_ = enabled;
    bandwidth_ = bandwidth.cwiseMax(Eigen::Vector3d::Zero());
    estimate_limit_ = estimate_limit.cwiseAbs();
    gravity_ = std::max(0.0, gravity);
    initialized_ = false;
    auxiliary_state_.setZero();
    estimate_.setZero();
}

void NonlinearDisturbanceObserver::reset(const Eigen::Vector3d &velocity)
{
    estimate_.setZero();
    auxiliary_state_ = -bandwidth_.cwiseProduct(velocity);
    initialized_ = velocity.allFinite();
}

void NonlinearDisturbanceObserver::update(
    const Eigen::Vector3d &velocity,
    const Eigen::Quaterniond &attitude,
    double collective_acceleration,
    double dt)
{
    if (!enabled_)
    {
        estimate_.setZero();
        return;
    }
    if (!velocity.allFinite() || !attitude.coeffs().allFinite() ||
        !std::isfinite(collective_acceleration) || !std::isfinite(dt) ||
        dt <= 0.0 || dt > 0.1)
    {
        return;
    }
    if (!initialized_)
    {
        reset(velocity);
        return;
    }

    Eigen::Quaterniond normalized_attitude = attitude;
    normalized_attitude.normalize();
    const Eigen::Vector3d nominal_acceleration =
        normalized_attitude * Eigen::Vector3d(0.0, 0.0, collective_acceleration) -
        Eigen::Vector3d(0.0, 0.0, gravity_);

    const Eigen::Vector3d raw_estimate =
        auxiliary_state_ + bandwidth_.cwiseProduct(velocity);
    estimate_ = raw_estimate;

    for (int axis = 0; axis < 3; ++axis)
    {
        estimate_(axis) = std::max(
            -estimate_limit_(axis),
            std::min(estimate_limit_(axis), estimate_(axis)));
    }
    if (!estimate_.isApprox(raw_estimate))
    {
        // Project the internal state to the saturation boundary to avoid
        // wind-up after a large velocity or command transient.
        auxiliary_state_ = estimate_ - bandwidth_.cwiseProduct(velocity);
    }
    auxiliary_state_ += -bandwidth_.cwiseProduct(
        estimate_ + nominal_acceleration) * dt;
}
