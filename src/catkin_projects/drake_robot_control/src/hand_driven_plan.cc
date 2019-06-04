#include <drake_robot_control/hand_driven_plan.h>

namespace drake {
namespace robot_plan_runner {

// Current robot state x = [q,v]
// Current time t
void HandDrivenPlan::Step(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &tau_external,
                          double t, Eigen::VectorXd *const q_commanded,
                          Eigen::VectorXd *const v_commanded,
                          Eigen::VectorXd *const tau_commanded) {
  PlanStatus not_started_status = PlanStatus::NOT_STARTED;
  PlanStatus running_status = PlanStatus::RUNNING;

  plan_status_.compare_exchange_strong(not_started_status, PlanStatus::RUNNING);

  // check if the plan has been stopped
  // if so just echo the last command
  if (this->is_stopped()) {
    *q_commanded = q_commanded_prev_;
    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
    return;
  }

  // do kinematics so we can check the force guards
  Eigen::VectorXd q = x.head(this->get_num_positions());
  Eigen::VectorXd v = x.tail(this->get_num_positions());
  // cache_measured_state_.initialize(q, v);
  // tree_->doKinematics(cache_measured_state_);

  // // Check the external force guards
  // if (guard_container_) {
  //   std::pair<bool, std::pair<double, std::shared_ptr<ForceGuard>>> result =
  //       guard_container_->EvaluateGuards(cache_measured_state_, q,
  //                                        tau_external);

  //   bool guard_triggered = result.first;

  //   if (guard_triggered) {
  //     std::cout << "Force Guard Triggered, stopping plan" << std::endl;
  //     std::cout << "ForceGuardType: " << result.second.second->get_type()
  //               << std::endl;
  //     plan_status_ = PlanStatus::STOPPED_BY_FORCE_GUARD;
  //     this->SetPlanFinished();

  //     *q_commanded = q_commanded_prev_;
  //     *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
  //     return;
  //   }
  // }

  if (tau_external.norm() < tau_threshold_) {
    *q_commanded = q_commanded_prev_;
    *v_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
  } else {
    DRAKE_ASSERT(t >= 0);
    double dt = t - t_prev_;
    Eigen::VectorXd q_dot_commanded =
        kp_tracking_.cwiseProduct(q - q_commanded_prev_);
    *q_commanded = q_commanded_prev_ + q_dot_commanded * dt;
    *v_commanded = q_dot_commanded;
    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
  }
  t_prev_ = t;
}

}  // namespace robot_plan_runner
}  // namespace drake
