/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void stop();
  void resume();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using ComputePathGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>;
  using SpinGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>;

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  // ---------------------------------------------------------------------------
  // NodePhase — authoritative macro-state of the node.
  // Always kept 1:1 with ExploreStatus.msg: every transition calls setPhase()
  // which atomically updates node_phase_ and publishes the corresponding status.
  // ---------------------------------------------------------------------------
  enum class NodePhase {
    EXPLORATION_STARTED,
    EXPLORATION_IN_PROGRESS,
    EXPLORATION_PAUSED,
    EXPLORATION_PAUSED_DURING_RETURN,
    EXPLORATION_COMPLETE,
    RETURNING_TO_ORIGIN,
    RETURN_TO_ORIGIN_FAILED,
  };

  // Set the node macro-state and publish the corresponding ExploreStatus message.
  // This is the single point where internal state and external topic are updated.
  void setPhase(NodePhase phase);

  // Phase name for logging.
  static const char* phaseToString(NodePhase phase);

  // Reset all exploration state to a clean starting point. Cancels all in-flight
  // Nav2 actions (NavigateToPose, Spin, ComputePath) and the return-to-init
  // watchdog timer, invalidates all pending callbacks via generation counters,
  // clears the frontier blacklist, resets progress tracking and the pre-rotation
  // state machine. Cancels exploring_timer_.
  // Always safe to call: all cancel operations are idempotent.
  void resetExplorationState();

  // ---------------------------------------------------------------------------------
  // Exploration pre-rotation state machine
  // ---------------------------------------------------------------------------------
  // States:
  //   IDLE                        - no sequence in progress
  //   NAV_ACTIVE                  - NavigateToPose is running toward a frontier
  //   CANCEL_REQUESTED            - async_cancel_all_goals() sent, awaiting ack
  //   WAITING_NAV_TERMINATION     - cancel ack received, awaiting nav result callback
  //   PRE_ROTATION_PATH_REQUESTED - ComputePathToPose sent, target LOCKED
  //   SPIN_ACTIVE                 - Nav2 Spin behavior running, target LOCKED
  //
  // Target is LOCKED from PRE_ROTATION_PATH_REQUESTED onward: the heading
  // computed by ComputePathToPose must remain coherent with the NavigateToPose
  // goal that follows the Spin.
  // ---------------------------------------------------------------------------------
  enum class CustomSequenceState
  {
    IDLE,
    NAV_ACTIVE,
    CANCEL_REQUESTED,
    WAITING_NAV_TERMINATION,
    PRE_ROTATION_PATH_REQUESTED,
    SPIN_ACTIVE,
  };

  // Exploration pre-rotation sequence
  void sendNavigateToPoseGoal(const geometry_msgs::msg::Point& target_position);
  void beginCustomPreRotationSequence(const geometry_msgs::msg::Point& target_position);
  void requestPathAndMaybePreRotate(const geometry_msgs::msg::Point& target_position);
  void handleCancelAllGoalsResponse(
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr response,
      uint64_t sequence_id);
  void tryAdvancePendingSequenceAfterNavTermination();
  void fallbackToCanonicalNavigate(const char* error_message);
  void checkCustomSequenceWatchdogs();
  void handleComputePathResult(
      const ComputePathGoalHandle::WrappedResult& result,
      const geometry_msgs::msg::Point& target_position);
  void handleSpinResult(const SpinGoalHandle::WrappedResult& result,
                        const geometry_msgs::msg::Point& target_position);
  bool tryExtractInitialPathHeading(const nav_msgs::msg::Path& path,
                                    double& heading) const;

  // Return-to-init pre-rotation sequence (separate from exploration state machine)
  void beginReturnToInitSequence();
  void handleReturnComputePathResult(const ComputePathGoalHandle::WrappedResult& result);
  void handleReturnSpinResult(const SpinGoalHandle::WrappedResult& result);

  // ---------------------------------------------------------------------------
  // State machine members
  // ---------------------------------------------------------------------------
  bool nav_active_ = false;
  bool cancel_ack_received_ = false;
  uint64_t current_nav_generation_ = 0;
  uint64_t active_custom_sequence_id_ = 0;
  uint64_t current_spin_generation_ = 0;
  uint64_t current_compute_path_generation_ = 0;
  CustomSequenceState custom_sequence_state_ = CustomSequenceState::IDLE;
  geometry_msgs::msg::Point pending_target_position_{};
  bool pending_target_valid_ = false;
  rclcpp::Time cancel_request_start_time_;
  rclcpp::Time nav_termination_wait_start_time_;
  rclcpp::Time spin_start_time_;
  rclcpp::Time compute_path_request_start_time_;
  // All timeout values below are loaded from YAML parameters in the constructor.
  // See params.yaml for documentation and inter-parameter constraints.
  double cancel_request_timeout_sec_;
  double nav_termination_timeout_sec_;
  double spin_watchdog_timeout_sec_;
  double compute_path_timeout_sec_;
  // Maximum time (seconds) granted to Nav2's Spin behavior to complete a
  // pre-rotation. Passed as time_allowance to both exploration spins and the
  // return-to-init spin. INVARIANT: spin_watchdog_timeout_sec_ >= spin_time_allowance_sec_
  // (enforced and auto-corrected in the constructor with a WARN log).
  double spin_time_allowance_sec_;
  // Spin retry state: reset at the start of each new pre-rotation sequence.
  // current_spin_retry_count_ counts how many spins have been completed in
  // the current sequence. last_computed_heading_ stores the path heading from
  // the most recent ComputePath, used to detect post-spin costmap drift.
  int    current_spin_retry_count_ = 0;
  double last_computed_heading_ = 0.0;

  // ---------------------------------------------------------------------------
  // Visualization
  // ---------------------------------------------------------------------------
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);

  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;

  /**
   * @brief Publisher for exploration status updates (see ExploreStatus.msg for status values)
   */
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_pub_;

  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr
      compute_path_client_;
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;
  frontier_exploration::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  // One-shot watchdog timer for the entire return-to-init sequence.
  // Created in beginReturnToInitSequence(), cancelled on any return outcome.
  rclcpp::TimerBase::SharedPtr return_watchdog_timer_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_ = std::numeric_limits<double>::infinity();
  rclcpp::Time last_progress_;
  size_t last_markers_count_ = 0;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose();

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  double min_prerotation_angle_;
  double post_spin_heading_tolerance_;
  int    max_spin_retries_;
  bool visualize_;
  bool return_to_init_;
  // Maximum total duration (seconds) allowed for the entire return-to-init
  // sequence (ComputePath + optional spin + NavigateToPose combined).
  // Must be >> spin_time_allowance + expected NavigateToPose duration.
  double return_to_init_timeout_sec_;
  std::string robot_base_frame_;
  bool resuming_ = false;
  // True after the very first exploration start (either via resume() or via
  // start_exploration_immediately=true in the constructor). Used by setPhase()
  // to publish EXPLORATION_STARTED exactly once instead of EXPLORATION_IN_PROGRESS.
  bool has_ever_started_ = false;
  // Authoritative macro-state of the node. Always 1:1 with ExploreStatus.msg.
  NodePhase node_phase_ = NodePhase::EXPLORATION_PAUSED;
  // Generation counter for the return-to-init NavigateToPose goal.
  uint64_t return_nav_generation_ = 0;
};
}  // namespace explore

#endif
