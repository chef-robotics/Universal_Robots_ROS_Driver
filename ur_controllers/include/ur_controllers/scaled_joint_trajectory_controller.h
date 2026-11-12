// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-18
 *
 */
//----------------------------------------------------------------------
#ifndef UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED
#define UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED

#include <algorithm>
#include <string>
#include <vector>

#include "ur_controllers/compliance_state_interface.h"
#include "ur_controllers/hardware_interface_adapter.h"
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace ur_controllers
{
template <class SegmentImpl, class HardwareInterface>
class ScaledJointTrajectoryController
  : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
{
public:
  ScaledJointTrajectoryController() = default;
  virtual ~ScaledJointTrajectoryController() = default;

  /*!
   * \brief Initialize as the base controller, then read the optional compliance path tolerances.
   *
   * While the robot script is compliant, and for `compliance_constraints/grace` seconds after, each joint's
   * path tolerance widens to `compliance_constraints/<joint>/trajectory`. Without a `compliance_state`
   * handle or `compliance_constraints`, path tolerances are as configured.
   */
  bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh,
                   controller_interface::ControllerBase::ClaimedResources& claimed_resources) override
  {
    if (!Base::initRequest(robot_hw, root_nh, controller_nh, claimed_resources))
    {
      return false;
    }
    ComplianceStateInterface* compliance_state_interface = robot_hw->get<ComplianceStateInterface>();
    if (compliance_state_interface == nullptr || !controller_nh.hasParam("compliance_constraints"))
    {
      return true;
    }
    try
    {
      compliance_state_ = compliance_state_interface->getHandle("compliance_state");
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_WARN_STREAM_NAMED(this->name_, "No compliance state, path tolerances as configured: " << e.what());
      return true;
    }
    ros::NodeHandle compliance_nh(controller_nh, "compliance_constraints");
    compliance_nh.param("grace", dt_compliance_grace_, 0.0);
    compliance_path_tolerances_.assign(this->joint_names_.size(), 0.0);
    for (size_t i = 0; i < this->joint_names_.size(); ++i)
    {
      compliance_nh.param(this->joint_names_[i] + "/trajectory", compliance_path_tolerances_[i], 0.0);
    }
    has_compliance_state_ = true;
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    this->scaling_factor_ = this->joints_[0].getScalingFactor();
    const bool is_in_compliance = updateIsInCompliance(time);
    // Get currently followed trajectory
    typename Base::TrajectoryPtr curr_traj_ptr;
    this->curr_trajectory_box_.get(curr_traj_ptr);
    typename Base::Trajectory& curr_traj = *curr_traj_ptr;

    // Update time data
    typename Base::TimeData time_data;
    time_data.time = time;                                                        // Cache current time
    time_data.period = ros::Duration(this->scaling_factor_ * period.toSec());     // Cache current control period
    time_data.uptime = this->time_data_.readFromRT()->uptime + time_data.period;  // Update controller uptime
    ros::Time traj_time = this->time_data_.readFromRT()->uptime + period;
    this->time_data_.writeFromNonRT(time_data);  // TODO: Grrr, we need a lock-free data structure here!

    // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
    // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
    // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
    // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
    // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time
    // we fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts
    // in the next control cycle, leaving the current cycle without a valid trajectory.

    // Update current state and state error
    for (unsigned int i = 0; i < this->joints_.size(); ++i)
    {
      this->current_state_.position[i] = this->joints_[i].getPosition();
      this->current_state_.velocity[i] = this->joints_[i].getVelocity();
      // There's no acceleration data available in a joint handle

      typename Base::TrajectoryPerJoint::const_iterator segment_it =
          sample(curr_traj[i], traj_time.toSec(), this->desired_joint_state_);
      if (curr_traj[i].end() == segment_it)
      {
        // Non-realtime safe, but should never happen under normal operation
        ROS_ERROR_NAMED(this->name_, "Unexpected error: No trajectory defined at current time. Please contact the "
                                     "package "
                                     "maintainer.");
        return;
      }
      this->desired_state_.position[i] = this->desired_joint_state_.position[0];
      this->desired_state_.velocity[i] = this->desired_joint_state_.velocity[0];
      this->desired_state_.acceleration[i] = this->desired_joint_state_.acceleration[0];
      ;

      this->state_joint_error_.position[0] =
          angles::shortest_angular_distance(this->current_state_.position[i], this->desired_joint_state_.position[0]);
      this->state_joint_error_.velocity[0] = this->desired_joint_state_.velocity[0] - this->current_state_.velocity[i];
      this->state_joint_error_.acceleration[0] = 0.0;

      this->state_error_.position[i] =
          angles::shortest_angular_distance(this->current_state_.position[i], this->desired_joint_state_.position[0]);
      this->state_error_.velocity[i] = this->desired_joint_state_.velocity[0] - this->current_state_.velocity[i];
      this->state_error_.acceleration[i] = 0.0;

      // Check tolerances
      const typename Base::RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
      if (rt_segment_goal && rt_segment_goal == this->rt_active_goal_)
      {
        // Check tolerances
        if (time_data.uptime.toSec() < segment_it->endTime())
        {
          // Currently executing a segment: check path tolerances
          const joint_trajectory_controller::SegmentTolerancesPerJoint<typename Base::Scalar>& joint_tolerances =
              segment_it->getTolerances();
          joint_trajectory_controller::StateTolerances<typename Base::Scalar> path_tolerance =
              joint_tolerances.state_tolerance;
          // A zero tolerance is unchecked; compliance only widens a checked one.
          if (is_in_compliance && path_tolerance.position > 0.0)
          {
            path_tolerance.position = std::max(path_tolerance.position, compliance_path_tolerances_[i]);
          }
          if (!checkStateTolerancePerJoint(this->state_joint_error_, path_tolerance))
          {
            if (this->verbose_)
            {
              ROS_ERROR_STREAM_NAMED(this->name_, "Path tolerances failed for joint: " << this->joint_names_[i]);
              checkStateTolerancePerJoint(this->state_joint_error_, path_tolerance, true);
            }
            rt_segment_goal->preallocated_result_->error_code =
                control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
            this->rt_active_goal_.reset();
            this->successful_joint_traj_.reset();
          }
        }
        else if (segment_it == --curr_traj[i].end())
        {
          if (this->verbose_)
            ROS_DEBUG_STREAM_THROTTLE_NAMED(1, this->name_,
                                            "Finished executing last segment, checking goal "
                                            "tolerances");

          // Controller uptime
          const ros::Time uptime = this->time_data_.readFromRT()->uptime;

          // Checks that we have ended inside the goal tolerances
          const joint_trajectory_controller::SegmentTolerancesPerJoint<typename Base::Scalar>& tolerances =
              segment_it->getTolerances();
          const bool inside_goal_tolerances =
              checkStateTolerancePerJoint(this->state_joint_error_, tolerances.goal_state_tolerance);

          if (inside_goal_tolerances)
          {
            this->successful_joint_traj_[i] = 1;
          }
          else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
          {
            // Still have some time left to meet the goal state tolerances
          }
          else
          {
            if (this->verbose_)
            {
              ROS_ERROR_STREAM_NAMED(this->name_, "Goal tolerances failed for joint: " << this->joint_names_[i]);
              // Check the tolerances one more time to output the errors that occurs
              checkStateTolerancePerJoint(this->state_joint_error_, tolerances.goal_state_tolerance, true);
            }

            rt_segment_goal->preallocated_result_->error_code =
                control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
            rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
            this->rt_active_goal_.reset();
            this->successful_joint_traj_.reset();
          }
        }
      }
    }

    // If there is an active goal and all segments finished successfully then set goal as succeeded
    typename Base::RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
    if (current_active_goal && this->successful_joint_traj_.count() == this->joints_.size())
    {
      current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
      current_active_goal.reset();  // do not publish feedback
      this->rt_active_goal_.reset();
      this->successful_joint_traj_.reset();
    }

    // Hardware interface adapter: Generate and send commands
    this->hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period, this->desired_state_, this->state_error_);

    // Set action feedback
    if (current_active_goal)
    {
      current_active_goal->preallocated_feedback_->header.stamp = this->time_data_.readFromRT()->time;
      current_active_goal->preallocated_feedback_->desired.positions = this->desired_state_.position;
      current_active_goal->preallocated_feedback_->desired.velocities = this->desired_state_.velocity;
      current_active_goal->preallocated_feedback_->desired.accelerations = this->desired_state_.acceleration;
      current_active_goal->preallocated_feedback_->actual.positions = this->current_state_.position;
      current_active_goal->preallocated_feedback_->actual.velocities = this->current_state_.velocity;
      current_active_goal->preallocated_feedback_->error.positions = this->state_error_.position;
      current_active_goal->preallocated_feedback_->error.velocities = this->state_error_.velocity;
      current_active_goal->setFeedback(current_active_goal->preallocated_feedback_);
    }

    // Publish state
    this->publishState(time_data.uptime);
  }

protected:
  using Base = joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>;
  double scaling_factor_;

private:
  /*!
   * \brief Whether the script is compliant now or was within the grace (its exit blend back onto the plan).
   *
   * Records `time` as the last compliant time when it is compliant.
   */
  bool updateIsInCompliance(const ros::Time& time)
  {
    if (!has_compliance_state_)
    {
      return false;
    }
    if (compliance_state_.isCompliant())
    {
      t_last_compliant_ = time;
      return true;
    }
    return !t_last_compliant_.isZero() && (time - t_last_compliant_).toSec() < dt_compliance_grace_;
  }

  ComplianceStateHandle compliance_state_;
  bool has_compliance_state_ = false;
  double dt_compliance_grace_ = 0.0;  // [s]
  std::vector<double> compliance_path_tolerances_;  // [rad], per joint in `joint_names_` order
  ros::Time t_last_compliant_;
};
}  // namespace ur_controllers

#endif  // ifndef UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED
