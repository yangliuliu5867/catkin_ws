#ifndef FSM_CTRL_NONLINEAR_DISTURBANCE_OBSERVER_HPP_
#define FSM_CTRL_NONLINEAR_DISTURBANCE_OBSERVER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

// First-order nonlinear disturbance observer for
//   v_dot = R(q) * e3 * a_z - g * e3 + d.
// The auxiliary-state realization avoids differentiating measured velocity:
//   d_hat = z + L v
//   z_dot  = -L (z + L v + f(q, a_z)).
// For a constant disturbance, d_hat_dot = -L (d_hat - d).
class NonlinearDisturbanceObserver
{
public:
    void configure(bool enabled,
                   const Eigen::Vector3d &bandwidth,
                   const Eigen::Vector3d &estimate_limit,
                   double gravity);

    void reset(const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero());

    void update(const Eigen::Vector3d &velocity,
                const Eigen::Quaterniond &attitude,
                double collective_acceleration,
                double dt);

    bool enabled() const { return enabled_; }
    bool initialized() const { return initialized_; }
    const Eigen::Vector3d &estimate() const { return estimate_; }

private:
    bool enabled_ = false;
    bool initialized_ = false;
    double gravity_ = 9.8015;
    Eigen::Vector3d bandwidth_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d estimate_limit_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d auxiliary_state_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d estimate_ = Eigen::Vector3d::Zero();
};

#endif  // FSM_CTRL_NONLINEAR_DISTURBANCE_OBSERVER_HPP_
