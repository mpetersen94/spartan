#pragma once
#include <drake_robot_control/plan_base.h>

namespace drake {
namespace robot_plan_runner {

class HandDrivenPlan : public PlanBase {
 public:
  HandDrivenPlan(std::shared_ptr<const RigidBodyTreed> tree,
                 Eigen::VectorXd kp_tracking, double tau_threshold)
      : PlanBase(std::move(tree)),
        kp_tracking_(kp_tracking),
        tau_threshold_(tau_threshold), t_prev_(0) {
    DRAKE_ASSERT(kp_tracking_.rows() == get_num_positions());
  }

  // Current robot state x = [q,v]
  // Current time t
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) override;

 private:
  Eigen::VectorXd kp_tracking_;
  double tau_threshold_;
  double t_prev_;
};

}  // namespace robot_plan_runner
}  // namespace drake
