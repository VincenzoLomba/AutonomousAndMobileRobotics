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

#include <explore/explore.h>

#include <thread>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <angles/angles.h>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<double>("planner_frequency", 1.0);
  this->declare_parameter<double>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<double>("potential_scale", 1e-3);
  this->declare_parameter<double>("orientation_scale", 0.0);
  this->declare_parameter<double>("gain_scale", 1.0);
  this->declare_parameter<double>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<double>("min_prerotation_angle", 0.1745);
  this->declare_parameter<double>("post_spin_heading_tolerance", 0.3490);
  this->declare_parameter<int>("max_spin_retries", 1);
  // Watchdog timeouts for each phase of the pre-rotation custom sequence.
  // All values are in seconds. See params.yaml for full documentation and
  // inter-parameter constraints.
  this->declare_parameter<double>("cancel_request_timeout", 5.0);
  this->declare_parameter<double>("nav_termination_timeout", 5.0);
  this->declare_parameter<double>("compute_path_timeout", 5.0);
  this->declare_parameter<double>("spin_watchdog_timeout", 25.0);
  // Time budget (seconds) granted to Nav2's Spin behavior for each pre-rotation.
  // INVARIANT (enforced below): spin_watchdog_timeout >= spin_time_allowance.
  this->declare_parameter<double>("spin_time_allowance", 20.0);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("min_prerotation_angle", min_prerotation_angle_);
  this->get_parameter("post_spin_heading_tolerance", post_spin_heading_tolerance_);
  this->get_parameter("max_spin_retries", max_spin_retries_);
  this->get_parameter("cancel_request_timeout", cancel_request_timeout_sec_);
  this->get_parameter("nav_termination_timeout", nav_termination_timeout_sec_);
  this->get_parameter("compute_path_timeout", compute_path_timeout_sec_);
  this->get_parameter("spin_watchdog_timeout", spin_watchdog_timeout_sec_);
  this->get_parameter("spin_time_allowance", spin_time_allowance_sec_);

  progress_timeout_ = timeout;

  // MOD 2 — Enforce the invariant: spin_watchdog_timeout >= spin_time_allowance.
  //
  // If violated, the ExploreLite watchdog would fire while Nav2's Spin behavior
  // is still legitimately executing and publishing cmd_vel. The overlap window
  // between the watchdog firing and Nav2 stopping is bounded by Nav2's
  // behavior_server cycle period (1 / cycle_frequency). Because cycle_frequency
  // is an externally configurable parameter (default 10 Hz → 100 ms, but can be
  // as low as 1 Hz → 1 s or lower), this window is not bounded to a fixed small
  // value. Relying on numeric defaults is therefore insufficient: the invariant
  // must be guaranteed structurally via this runtime check.
  if (spin_watchdog_timeout_sec_ < spin_time_allowance_sec_) {
    RCLCPP_WARN(
        logger_,
        "spin_watchdog_timeout (%.2fs) is less than spin_time_allowance (%.2fs). "
        "This would cause the ExploreLite watchdog to fire while Nav2 Spin is still "
        "legitimately running, risking concurrent cmd_vel commands from both the Spin "
        "behavior and the NavigateToPose controller. "
        "Adjusting spin_watchdog_timeout to spin_time_allowance = %.2fs.",
        spin_watchdog_timeout_sec_,
        spin_time_allowance_sec_,
        spin_time_allowance_sec_);
    spin_watchdog_timeout_sec_ = spin_time_allowance_sec_;
  }
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);
  compute_path_client_ =
      rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
          this, "compute_path_to_pose");
  spin_client_ =
      rclcpp_action::create_client<nav2_msgs::action::Spin>(
          this, "spin");

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  RCLCPP_INFO(logger_, "Waiting to connect to Nav2 compute_path_to_pose server");
  compute_path_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to Nav2 compute_path_to_pose server");

  RCLCPP_INFO(logger_, "Waiting to connect to Nav2 spin server");
  spin_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to Nav2 spin server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  // Create the timer but cancel it immediately: the node starts in a paused
  // state and waits for a True message on /explore/resume before exploring.
  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint32_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  exploring_timer_->cancel();

  RCLCPP_INFO(logger_,
              "ExploreLite ready. Publish true on /explore/resume to start exploration.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
  status_pub_->publish(status_msg);
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  checkCustomSequenceWatchdogs();

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }

  // MOD 4 — Freeze the progress timer during intentional non-navigation states.
  //
  // The progress_timeout mechanism was designed for the original ExploreLite
  // behaviour where the robot always navigates directly toward the frontier:
  // if min_distance does not decrease for progress_timeout seconds, the robot
  // is genuinely stuck and the frontier should be blacklisted.
  //
  // The V5 pre-rotation sequence introduces four states in which the robot is
  // stationary or spinning in place by design, NOT because it is stuck:
  //   CANCEL_REQUESTED          — waiting for Nav2 to acknowledge the cancel
  //   WAITING_NAV_TERMINATION   — waiting for the previous nav result callback
  //   PRE_ROTATION_PATH_REQUESTED — waiting for ComputePathToPose response
  //   SPIN_ACTIVE               — Nav2 Spin behavior running
  //
  // All four watchdog timeouts that bound these states are user-configurable
  // parameters and may be set to values that, individually or collectively,
  // exceed progress_timeout. Assuming "with current defaults this cannot
  // happen" is therefore insufficient: the invariant must hold for any valid
  // parameter combination. Freezing last_progress_ during these states ensures
  // the blacklist timer never fires while the robot is intentionally paused.
  //
  // IDLE and NAV_ACTIVE are deliberately excluded:
  //   IDLE      — no sequence of ours is active; the robot may be stopped for
  //               unknown external reasons, and the original timer must apply.
  //   NAV_ACTIVE — the robot is navigating toward the frontier; this is
  //               exactly the phase progress_timeout is designed to monitor.
  if (custom_sequence_state_ != CustomSequenceState::IDLE &&
      custom_sequence_state_ != CustomSequenceState::NAV_ACTIVE) {
    last_progress_ = this->now();
  }

  // black list if we've made no progress for a long time
  if ((this->now() - last_progress_ >
      tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "Adding current goal to black list");
    makePlan();
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  // State machine guards:
  // In CANCEL_REQUESTED / WAITING_NAV_TERMINATION: save the latest target so
  // it will be used once the sequence becomes dispatchable.
  if (custom_sequence_state_ == CustomSequenceState::CANCEL_REQUESTED ||
      custom_sequence_state_ == CustomSequenceState::WAITING_NAV_TERMINATION) {
    pending_target_position_ = target_position;
    pending_target_valid_ = true;
    RCLCPP_DEBUG(logger_,
                 "A custom pre-rotation sequence is already canceling or waiting for the previous "
                 "navigation to terminate. The latest target has been recorded and will be used "
                 "when the sequence becomes dispatchable.");
    return;
  }

  // In PRE_ROTATION_PATH_REQUESTED / SPIN_ACTIVE: target is LOCKED.
  // The heading computed by ComputePathToPose must remain coherent with the
  // NavigateToPose goal that will follow the Spin. Do not dispatch a new
  // sequence; the current one will complete and the next makePlan() cycle
  // will pick up the updated frontier.
  if (custom_sequence_state_ == CustomSequenceState::PRE_ROTATION_PATH_REQUESTED ||
      custom_sequence_state_ == CustomSequenceState::SPIN_ACTIVE) {
    RCLCPP_DEBUG(logger_,
                 "A custom pre-rotation sequence is already computing a helper path or executing "
                 "a spin (target is locked for heading coherence). This makePlan() cycle will "
                 "preserve normal frontier/progress/blacklist updates but will not dispatch any "
                 "new custom sequence yet.");
    return;
  }

  beginCustomPreRotationSequence(target_position);
}

// -----------------------------------------------------------------------------
// Exploration pre-rotation sequence
// -----------------------------------------------------------------------------

void Explore::sendNavigateToPoseGoal(
    const geometry_msgs::msg::Point& target_position)
{
  RCLCPP_DEBUG(logger_, "Sending canonical NavigateToPose goal to Nav2");

  custom_sequence_state_ = CustomSequenceState::NAV_ACTIVE;
  pending_target_valid_ = false;
  nav_active_ = true;

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  const uint64_t goal_generation = ++current_nav_generation_;

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this, goal_generation](NavigationGoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_,
                       "Canonical NavigateToPose goal was rejected by Nav2. ExploreLite will "
                       "keep its internal logic alive and wait for the next makePlan() opportunity.");
          if (goal_generation == current_nav_generation_) {
            nav_active_ = false;
            custom_sequence_state_ = CustomSequenceState::IDLE;
          }
          return;
        }
        if (goal_generation == current_nav_generation_) {
          nav_active_ = true;
          custom_sequence_state_ = CustomSequenceState::NAV_ACTIVE;
        }
      };
  send_goal_options.result_callback =
      [this,
       target_position,
       goal_generation](const NavigationGoalHandle::WrappedResult& result) {
        if (goal_generation == current_nav_generation_) {
          nav_active_ = false;
        }
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::beginCustomPreRotationSequence(
    const geometry_msgs::msg::Point& target_position)
{
  pending_target_position_ = target_position;
  pending_target_valid_ = true;
  cancel_ack_received_ = false;
  cancel_request_start_time_ = this->now();
  const uint64_t sequence_id = ++active_custom_sequence_id_;
  custom_sequence_state_ = CustomSequenceState::CANCEL_REQUESTED;
  // Reset spin retry state for this new sequence.
  current_spin_retry_count_ = 0;
  last_computed_heading_ = 0.0;

  RCLCPP_DEBUG(logger_,
               "Starting a new custom pre-rotation sequence. All active ExploreLite "
               "NavigateToPose goals will be canceled before requesting a fresh helper path.");

  move_base_client_->async_cancel_all_goals(
      [this, sequence_id](
          rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr response) {
        handleCancelAllGoalsResponse(response, sequence_id);
      });
}

void Explore::requestPathAndMaybePreRotate(
    const geometry_msgs::msg::Point& target_position)
{
  custom_sequence_state_ = CustomSequenceState::PRE_ROTATION_PATH_REQUESTED;
  compute_path_request_start_time_ = this->now();
  const uint64_t compute_gen = ++current_compute_path_generation_;

  auto goal = nav2_msgs::action::ComputePathToPose::Goal();
  goal.goal.pose.position = target_position;
  goal.goal.pose.orientation.w = 1.;
  goal.goal.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.goal.header.stamp = this->now();
  goal.planner_id = "";
  goal.use_start = false;

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();

  // goal_response_callback: handle immediate rejection by the planner server.
  // If the goal is rejected, result_callback will NOT be called, so we must
  // fall back here to avoid permanently stalling in PRE_ROTATION_PATH_REQUESTED.
  send_goal_options.goal_response_callback =
      [this, compute_gen, target_position](
          ComputePathGoalHandle::SharedPtr goal_handle) {
        if (compute_gen != current_compute_path_generation_) {
          // Stale callback from a previous sequence; ignore.
          return;
        }
        if (!goal_handle) {
          // Planner server rejected the goal immediately.
          RCLCPP_ERROR(logger_,
                       "ComputePathToPose goal was rejected by the planner server. "
                       "ExploreLite will now deliberately fall back to its original canonical "
                       "behavior: it will send the normal NavigateToPose goal without any custom "
                       "pre-rotation.");
          custom_sequence_state_ = CustomSequenceState::IDLE;
          pending_target_valid_ = false;
          sendNavigateToPoseGoal(target_position);
        }
        // If accepted: do nothing, wait for result_callback.
      };

  send_goal_options.result_callback =
      [this, compute_gen, target_position](
          const ComputePathGoalHandle::WrappedResult& result) {
        if (compute_gen != current_compute_path_generation_) {
          RCLCPP_WARN(logger_,
                      "Ignoring stale ComputePathToPose result callback (generation mismatch). "
                      "This is expected after stop() or a sequence restart.");
          return;
        }
        handleComputePathResult(result, target_position);
      };

  compute_path_client_->async_send_goal(goal, send_goal_options);
}

bool Explore::tryExtractInitialPathHeading(const nav_msgs::msg::Path& path,
                                           double& heading) const
{
  if (path.poses.size() < 2) {
    return false;
  }

  constexpr double min_segment_length = 0.05;
  const auto& first_pose = path.poses.front().pose.position;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto& next_pose = path.poses[i].pose.position;
    const double dx = next_pose.x - first_pose.x;
    const double dy = next_pose.y - first_pose.y;
    const double dist = std::hypot(dx, dy);
    if (dist > min_segment_length) {
      heading = std::atan2(dy, dx);
      return true;
    }
  }

  return false;
}

void Explore::handleComputePathResult(
    const ComputePathGoalHandle::WrappedResult& result,
    const geometry_msgs::msg::Point& target_position)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
        logger_,
        "Pre-rotation helper path request failed before NavigateToPose dispatch. "
        "ExploreLite will now deliberately fall back to its original canonical behavior: "
        "it will send the normal NavigateToPose goal without any custom pre-rotation, and "
        "Nav2 plus the existing ExploreLite logic will handle planning, replanning, recovery, "
        "timeouts, and any eventual frontier blacklisting exactly as in the unmodified package.");
    custom_sequence_state_ = CustomSequenceState::IDLE;
    pending_target_valid_ = false;
    sendNavigateToPoseGoal(target_position);
    return;
  }

  double new_heading = 0.0;
  if (result.result->path.poses.empty() ||
      !tryExtractInitialPathHeading(result.result->path, new_heading)) {
    RCLCPP_ERROR(
        logger_,
        "Pre-rotation helper path request succeeded but did not provide a usable global path "
        "heading for the custom pre-rotation step. ExploreLite will now deliberately fall back "
        "to its original canonical behavior: it will send the normal NavigateToPose goal without "
        "any custom pre-rotation, and Nav2 plus the existing ExploreLite logic will handle "
        "planning, replanning, recovery, timeouts, and any eventual frontier blacklisting exactly "
        "as in the unmodified package.");
    custom_sequence_state_ = CustomSequenceState::IDLE;
    pending_target_valid_ = false;
    sendNavigateToPoseGoal(target_position);
    return;
  }

  // ---------------------------------------------------------------------------
  // Post-spin recheck: compare the newly computed heading with the heading that
  // was used to launch the previous Spin. If the costmap changed significantly
  // during the spin (e.g. the laser revealed new obstacles while the robot
  // rotated in place), the global planner may produce a path that diverges from
  // the direction the robot just aligned to — which is exactly the condition
  // that causes the robot to deviate on departure and risk collisions.
  //
  // This block runs only when current_spin_retry_count_ > 0, i.e. at least one
  // spin has already been executed in this sequence (recheck path).
  // ---------------------------------------------------------------------------
  if (current_spin_retry_count_ > 0) {
    const double heading_diff = std::abs(
        angles::shortest_angular_distance(last_computed_heading_, new_heading));

    if (heading_diff <= post_spin_heading_tolerance_) {
      // Heading is stable: the costmap did not drift enough to require re-alignment.
      // Proceed directly with NavigateToPose.
      RCLCPP_INFO(logger_,
                  "Post-spin heading recheck (spin %d/%d): heading change = %.4f rad (%.2f deg) "
                  "is within tolerance = %.4f rad (%.2f deg). "
                  "Alignment is stable. Proceeding with NavigateToPose.",
                  current_spin_retry_count_, max_spin_retries_,
                  heading_diff, heading_diff * 180.0 / M_PI,
                  post_spin_heading_tolerance_, post_spin_heading_tolerance_ * 180.0 / M_PI);
      custom_sequence_state_ = CustomSequenceState::IDLE;
      pending_target_valid_ = false;
      sendNavigateToPoseGoal(target_position);
      return;
    }

    // Heading drifted beyond tolerance: re-spin is warranted.
    RCLCPP_INFO(logger_,
                "Post-spin heading recheck (spin %d/%d): heading changed by %.4f rad (%.2f deg), "
                "exceeding tolerance = %.4f rad (%.2f deg). "
                "Will perform an additional alignment spin.",
                current_spin_retry_count_, max_spin_retries_,
                heading_diff, heading_diff * 180.0 / M_PI,
                post_spin_heading_tolerance_, post_spin_heading_tolerance_ * 180.0 / M_PI);
  }

  // Store the current heading as the reference for the next potential recheck.
  last_computed_heading_ = new_heading;

  const auto robot_pose = costmap_client_.getRobotPose();
  const double current_yaw = tf2::getYaw(robot_pose.orientation);
  const double relative_spin = angles::shortest_angular_distance(current_yaw, new_heading);

  // Skip spin if the rotation magnitude (in either direction) is below the
  // configured threshold. The check is on the absolute value so the threshold
  // applies symmetrically to both left and right rotations.
  if (std::abs(relative_spin) < min_prerotation_angle_) {
    RCLCPP_INFO(logger_,
                "Custom pre-rotation skipped: |angle| = %.4f rad (%.2f deg) is below "
                "threshold min_prerotation_angle = %.4f rad (%.2f deg). "
                "Proceeding directly with NavigateToPose.",
                std::abs(relative_spin), std::abs(relative_spin) * 180.0 / M_PI,
                min_prerotation_angle_, min_prerotation_angle_ * 180.0 / M_PI);
    custom_sequence_state_ = CustomSequenceState::IDLE;
    pending_target_valid_ = false;
    sendNavigateToPoseGoal(target_position);
    return;
  }

  auto spin_goal = nav2_msgs::action::Spin::Goal();
  spin_goal.target_yaw = static_cast<float>(relative_spin);
  spin_goal.time_allowance.sec =
      static_cast<int32_t>(std::floor(spin_time_allowance_sec_));
  spin_goal.time_allowance.nanosec =
      static_cast<uint32_t>((spin_time_allowance_sec_ -
                             std::floor(spin_time_allowance_sec_)) * 1e9);

  const uint64_t spin_generation = ++current_spin_generation_;
  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
  send_goal_options.result_callback =
      [this,
       target_position,
       spin_generation](const SpinGoalHandle::WrappedResult& result) {
        if (spin_generation != current_spin_generation_) {
          RCLCPP_WARN(logger_,
                      "Ignoring a stale Nav2 Spin result callback belonging to an older "
                      "custom pre-rotation sequence.");
          return;
        }
        handleSpinResult(result, target_position);
      };

  RCLCPP_INFO(logger_,
              "Launching Nav2 Spin behavior for custom pre-rotation before NavigateToPose. "
              "Requested relative spin: %.4f rad (%.2f deg)%s",
              relative_spin, relative_spin * 180.0 / M_PI,
              current_spin_retry_count_ > 0 ? " [retry alignment]" : "");
  spin_start_time_ = this->now();
  custom_sequence_state_ = CustomSequenceState::SPIN_ACTIVE;
  spin_client_->async_send_goal(spin_goal, send_goal_options);
}

void Explore::handleSpinResult(const SpinGoalHandle::WrappedResult& result,
                               const geometry_msgs::msg::Point& target_position)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    // Spin failed: fall back to canonical NavigateToPose, same as before.
    RCLCPP_ERROR(
        logger_,
        "Custom pre-rotation via Nav2 Spin did not complete successfully. ExploreLite will now "
        "deliberately fall back to its original canonical behavior: it will send the normal "
        "NavigateToPose goal without any further custom interception, and Nav2 plus the existing "
        "ExploreLite logic will handle planning, replanning, recovery, timeouts, and any eventual "
        "frontier blacklisting exactly as in the unmodified package.");
    custom_sequence_state_ = CustomSequenceState::IDLE;
    pending_target_valid_ = false;
    sendNavigateToPoseGoal(target_position);
    return;
  }

  ++current_spin_retry_count_;

  if (current_spin_retry_count_ < max_spin_retries_) {
    // Retries are still available. Re-request the global path to check whether
    // the costmap changed significantly during the spin (e.g. new obstacles
    // revealed by the laser while the robot was rotating). If the new path
    // heading differs from the one used to launch this spin by more than
    // post_spin_heading_tolerance_, handleComputePathResult will launch an
    // additional corrective spin. Otherwise it will proceed with NavigateToPose.
    RCLCPP_INFO(logger_,
                "Custom pre-rotation via Nav2 Spin completed (spin %d of max %d). "
                "Re-requesting global path to verify heading stability after spin.",
                current_spin_retry_count_, max_spin_retries_);
    // State returns to PRE_ROTATION_PATH_REQUESTED; target remains locked.
    requestPathAndMaybePreRotate(target_position);
  } else {
    // All allowed spin attempts have been used. Proceed with NavigateToPose
    // using the alignment achieved so far (canonical fallback from this point).
    RCLCPP_INFO(
        logger_,
        "Custom pre-rotation via Nav2 Spin completed successfully "
        "(spin %d of max %d, retries exhausted). Proceeding with NavigateToPose.",
        current_spin_retry_count_, max_spin_retries_);
    custom_sequence_state_ = CustomSequenceState::IDLE;
    pending_target_valid_ = false;
    sendNavigateToPoseGoal(target_position);
  }
}

// -----------------------------------------------------------------------------
// Watchdogs
// -----------------------------------------------------------------------------

void Explore::checkCustomSequenceWatchdogs()
{
  const auto now = this->now();

  if (custom_sequence_state_ == CustomSequenceState::CANCEL_REQUESTED &&
      (now - cancel_request_start_time_ > tf2::durationFromSec(cancel_request_timeout_sec_))) {
    fallbackToCanonicalNavigate(
        "The custom pre-rotation sequence has been waiting too long for the NavigateToPose "
        "cancel-all acknowledgement. ExploreLite will now deliberately fall back to its original "
        "canonical behavior by dispatching the pending NavigateToPose goal without any custom "
        "helper path or spin.");
    return;
  }

  if (custom_sequence_state_ == CustomSequenceState::WAITING_NAV_TERMINATION &&
      (now - nav_termination_wait_start_time_ > tf2::durationFromSec(nav_termination_timeout_sec_))) {
    fallbackToCanonicalNavigate(
        "The custom pre-rotation sequence has been waiting too long for the previously active "
        "NavigateToPose goal to reach a terminal result after cancel-all was acknowledged. "
        "ExploreLite will now deliberately fall back to its original canonical behavior by "
        "dispatching the pending NavigateToPose goal without any custom helper path or spin.");
    return;
  }

  if (custom_sequence_state_ == CustomSequenceState::PRE_ROTATION_PATH_REQUESTED &&
      (now - compute_path_request_start_time_ > tf2::durationFromSec(compute_path_timeout_sec_))) {
    fallbackToCanonicalNavigate(
        "The ComputePathToPose request for the custom pre-rotation has not responded within the "
        "timeout. This may indicate that the goal was accepted but the planner is unresponsive. "
        "ExploreLite will now deliberately fall back to its original canonical behavior by "
        "dispatching the pending NavigateToPose goal without any custom helper path or spin.");
    return;
  }

  if (custom_sequence_state_ == CustomSequenceState::SPIN_ACTIVE &&
      (now - spin_start_time_ > tf2::durationFromSec(spin_watchdog_timeout_sec_))) {
    // MOD 3 — Cancel the in-flight Spin on Nav2's behavior_server before falling back.
    //
    // fallbackToCanonicalNavigate() will invalidate our result callback via
    // ++current_spin_generation_, but that only prevents ExploreLite from acting
    // on the callback — it does NOT stop the physical spin on Nav2. If
    // behavior_server's cycle_frequency is low (e.g. 1 Hz), Nav2 may continue
    // publishing cmd_vel for up to one full cycle period after NavigateToPose has
    // already been sent. This fire-and-forget cancel ensures Nav2 stops publishing
    // spin cmd_vel as soon as it receives the request, regardless of cycle_frequency.
    //
    // The cancel response is intentionally not registered: Nav2 will report
    // CANCELED or ERROR_GOAL_TERMINATED (if already done), either of which is
    // harmless. The result callback, if it arrives, will be dropped due to the
    // generation mismatch set by fallbackToCanonicalNavigate().
    //
    // This call is placed ONLY in this watchdog branch — it is the sole caller of
    // fallbackToCanonicalNavigate() where a Spin is physically active on Nav2.
    spin_client_->async_cancel_all_goals();
    fallbackToCanonicalNavigate(
        "The custom pre-rotation sequence watchdog detected that Nav2 Spin has exceeded the "
        "local waiting budget before its result callback returned. ExploreLite will now "
        "deliberately fall back to its original canonical behavior by dispatching the pending "
        "NavigateToPose goal without waiting any longer for the custom spin result.");
  }
}

// -----------------------------------------------------------------------------
// Cancel-all response and nav termination
// -----------------------------------------------------------------------------

void Explore::handleCancelAllGoalsResponse(
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::CancelResponse::SharedPtr response,
    uint64_t sequence_id)
{
  if (sequence_id != active_custom_sequence_id_) {
    RCLCPP_WARN(logger_,
                "Ignoring a stale NavigateToPose cancel-all acknowledgement belonging to an "
                "older custom pre-rotation sequence.");
    return;
  }

  cancel_ack_received_ = true;

  if (!response) {
    fallbackToCanonicalNavigate(
        "The NavigateToPose cancel-all callback returned a null response pointer. ExploreLite "
        "will now deliberately fall back to its original canonical behavior by dispatching the "
        "pending NavigateToPose goal without any custom helper path or spin.");
    return;
  }

  if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
    RCLCPP_DEBUG(logger_,
                 "NavigateToPose cancel-all request was acknowledged by Nav2 and at least one "
                 "goal entered the CANCELING state.");
  } else {
    RCLCPP_WARN(logger_,
                "NavigateToPose cancel-all request was acknowledged by Nav2 but "
                "return_code=%d and goals_canceling.size()=%zu. ExploreLite will keep following "
                "its local state machine and decide whether it still needs to wait for the "
                "current navigation result before proceeding.",
                static_cast<int>(response->return_code),
                response->goals_canceling.size());
  }

  if (!pending_target_valid_) {
    custom_sequence_state_ = nav_active_ ? CustomSequenceState::NAV_ACTIVE : CustomSequenceState::IDLE;
    return;
  }

  if (!nav_active_) {
    RCLCPP_DEBUG(logger_,
                 "NavigateToPose cancel-all request acknowledged and no frontier navigation is "
                 "currently active. Proceeding directly with the helper path request for the "
                 "pending target.");
    requestPathAndMaybePreRotate(pending_target_position_);
    return;
  }

  nav_termination_wait_start_time_ = this->now();
  custom_sequence_state_ = CustomSequenceState::WAITING_NAV_TERMINATION;
  RCLCPP_DEBUG(logger_,
               "NavigateToPose cancel-all request acknowledged. Waiting for the currently active "
               "frontier navigation to reach a terminal result before running the custom helper "
               "path request and spin.");
}

void Explore::tryAdvancePendingSequenceAfterNavTermination()
{
  if (!pending_target_valid_ || !cancel_ack_received_ || nav_active_) {
    return;
  }

  RCLCPP_DEBUG(logger_,
               "The previously active frontier navigation has now terminated. Proceeding with "
               "the helper path request for the pending custom pre-rotation sequence.");
  requestPathAndMaybePreRotate(pending_target_position_);
}

void Explore::fallbackToCanonicalNavigate(const char* error_message)
{
  if (error_message != nullptr) {
    RCLCPP_ERROR(logger_, "%s", error_message);
  }

  ++current_spin_generation_;
  cancel_ack_received_ = false;
  custom_sequence_state_ = CustomSequenceState::IDLE;

  if (pending_target_valid_) {
    const auto target_position = pending_target_position_;
    pending_target_valid_ = false;
    sendNavigateToPoseGoal(target_position);
  }
}

// -----------------------------------------------------------------------------
// reachedGoal
// -----------------------------------------------------------------------------

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  // If we are waiting for the previously active nav to terminate (because we
  // already have a new pending target), advance the sequence rather than
  // applying the normal goal-result logic. Note: we deliberately do not
  // blacklist the frontier here regardless of result code, because any
  // termination in this state is a consequence of our own cancel request, not
  // an autonomous Nav2 failure signal.
  if (custom_sequence_state_ == CustomSequenceState::WAITING_NAV_TERMINATION) {
    tryAdvancePendingSequenceAfterNavTermination();
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      custom_sequence_state_ = CustomSequenceState::IDLE;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "Goal was aborted");
      frontier_blacklist_.push_back(frontier_goal);
      RCLCPP_DEBUG(logger_, "Adding current goal to black list");
      custom_sequence_state_ = CustomSequenceState::IDLE;
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      custom_sequence_state_ = CustomSequenceState::IDLE;
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      custom_sequence_state_ = CustomSequenceState::IDLE;
      break;
  }

  makePlan();
}

// -----------------------------------------------------------------------------
// Blacklist helper
// -----------------------------------------------------------------------------

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
// Return-to-init pre-rotation sequence
// -----------------------------------------------------------------------------

void Explore::beginReturnToInitSequence()
{
  RCLCPP_INFO(logger_,
              "Starting return-to-init sequence with pre-rotation. "
              "Computing path to initial pose to determine heading.");

  // Publish RETURNING_TO_ORIGIN early so external observers know we are
  // attempting to return, even before the actual NavigateToPose is sent.
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  // Increment the compute path generation counter before sending: any callback
  // arriving with the old generation value will be ignored as stale.
  const uint64_t compute_gen = ++current_compute_path_generation_;

  auto goal = nav2_msgs::action::ComputePathToPose::Goal();
  goal.goal.pose.position = initial_pose_.position;
  goal.goal.pose.orientation = initial_pose_.orientation;
  goal.goal.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.goal.header.stamp = this->now();
  goal.planner_id = "";
  goal.use_start = false;

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();

  // goal_response_callback: handle immediate rejection by the planner server.
  send_goal_options.goal_response_callback =
      [this, compute_gen](ComputePathGoalHandle::SharedPtr goal_handle) {
        if (compute_gen != current_compute_path_generation_) {
          return;  // stale
        }
        if (!goal_handle) {
          RCLCPP_WARN(logger_,
                      "ComputePathToPose for return-to-init was rejected by the planner server. "
                      "Falling back to direct NavigateToPose for return.");
          returnToInitialPose();
        }
        // If accepted: wait for result_callback.
      };

  send_goal_options.result_callback =
      [this, compute_gen](const ComputePathGoalHandle::WrappedResult& result) {
        if (compute_gen != current_compute_path_generation_) {
          RCLCPP_WARN(logger_,
                      "Ignoring stale ComputePathToPose result for return-to-init sequence "
                      "(generation mismatch). This is expected if resume() was called while "
                      "the return sequence was in progress.");
          return;
        }
        handleReturnComputePathResult(result);
      };

  compute_path_client_->async_send_goal(goal, send_goal_options);
}

void Explore::handleReturnComputePathResult(
    const ComputePathGoalHandle::WrappedResult& result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(logger_,
                "ComputePathToPose for return-to-init failed (result code: %d). "
                "Falling back to direct NavigateToPose for return.",
                static_cast<int>(result.code));
    returnToInitialPose();
    return;
  }

  double desired_heading = 0.0;
  if (result.result->path.poses.empty() ||
      !tryExtractInitialPathHeading(result.result->path, desired_heading)) {
    RCLCPP_WARN(logger_,
                "ComputePathToPose for return-to-init succeeded but path heading could not be "
                "extracted. Falling back to direct NavigateToPose for return.");
    returnToInitialPose();
    return;
  }

  const auto robot_pose = costmap_client_.getRobotPose();
  const double current_yaw = tf2::getYaw(robot_pose.orientation);
  const double relative_spin = angles::shortest_angular_distance(current_yaw, desired_heading);

  // Skip spin if the rotation is below the configured threshold (symmetric).
  if (std::abs(relative_spin) < min_prerotation_angle_) {
    RCLCPP_INFO(logger_,
                "Pre-rotation for return-to-init skipped: |angle| = %.4f rad (%.2f deg) is "
                "below threshold min_prerotation_angle = %.4f rad (%.2f deg). "
                "Proceeding directly with return NavigateToPose.",
                std::abs(relative_spin), std::abs(relative_spin) * 180.0 / M_PI,
                min_prerotation_angle_, min_prerotation_angle_ * 180.0 / M_PI);
    returnToInitialPose();
    return;
  }

  auto spin_goal = nav2_msgs::action::Spin::Goal();
  spin_goal.target_yaw = static_cast<float>(relative_spin);
  spin_goal.time_allowance.sec =
      static_cast<int32_t>(std::floor(spin_time_allowance_sec_));
  spin_goal.time_allowance.nanosec =
      static_cast<uint32_t>((spin_time_allowance_sec_ -
                             std::floor(spin_time_allowance_sec_)) * 1e9);

  // Increment spin generation counter before sending.
  const uint64_t spin_gen = ++current_spin_generation_;
  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
  send_goal_options.result_callback =
      [this, spin_gen](const SpinGoalHandle::WrappedResult& result) {
        if (spin_gen != current_spin_generation_) {
          RCLCPP_WARN(logger_,
                      "Ignoring stale Spin result for return-to-init sequence "
                      "(generation mismatch).");
          return;
        }
        handleReturnSpinResult(result);
      };

  RCLCPP_INFO(logger_,
              "Launching Nav2 Spin for pre-rotation before return-to-init NavigateToPose. "
              "Requested relative spin: %.4f rad (%.2f deg)",
              relative_spin, relative_spin * 180.0 / M_PI);
  spin_client_->async_send_goal(spin_goal, send_goal_options);
}

void Explore::handleReturnSpinResult(const SpinGoalHandle::WrappedResult& result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(logger_,
                "Pre-rotation Spin for return-to-init did not complete successfully "
                "(result code: %d). Proceeding with return NavigateToPose anyway.",
                static_cast<int>(result.code));
  } else {
    RCLCPP_INFO(logger_,
                "Pre-rotation Spin for return-to-init completed successfully. "
                "Proceeding with return NavigateToPose.");
  }
  returnToInitialPose();
}

// -----------------------------------------------------------------------------
// Return to initial pose (NavigateToPose)
// -----------------------------------------------------------------------------

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Sending NavigateToPose goal to return to initial pose.");

  // Increment the generation counter before sending, so that if resume() is
  // called while this goal is in flight, the counter will be incremented again
  // and the result callback will detect the mismatch and ignore the stale result.
  const uint64_t return_gen = ++return_nav_generation_;

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this, return_gen](const NavigationGoalHandle::WrappedResult& result) {
        if (return_gen != return_nav_generation_) {
          // This callback belongs to a return-to-init NavigateToPose that was
          // intentionally cancelled by a subsequent resume() call. Ignore it
          // to avoid publishing a spurious RETURN_TO_ORIGIN_FAILED status.
          // Note: there is a theoretical narrow race window where a genuinely
          // failed (ABORTED) return arrives after resume() has already incremented
          // return_nav_generation_, causing the failure notification to be
          // silently swallowed. This is accepted: the user has already expressed
          // intent to resume exploration, making the return-to-init outcome
          // irrelevant to the current operational context.
          RCLCPP_DEBUG(logger_,
                       "Ignoring stale return-to-init NavigateToPose result "
                       "(generation mismatch - cancelled by resume()).");
          return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        } else {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURN_TO_ORIGIN_FAILED;
          status_pub_->publish(status_msg);
          RCLCPP_WARN(logger_,
                      "Return to initial pose failed (result code: %d). "
                      "The robot did not reach its starting position.",
                      static_cast<int>(result.code));
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

// -----------------------------------------------------------------------------
// start / stop / resume
// -----------------------------------------------------------------------------

void Explore::start()
{
  // NOTE: this function is part of the public API (declared in explore.h) but is
  // never called internally in this version. In the original package, the constructor
  // started exploration immediately. In v5 the node starts paused and
  // EXPLORATION_STARTED is published by the first resume() call via the
  // has_ever_started_ flag. This function is kept for API compatibility only.
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  finished_exploring_ = finished_exploring;

  // Only publish paused status if manually stopped (not finished exploring).
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  // Invalidate all in-flight action callbacks by bumping generation counters.
  ++active_custom_sequence_id_;         // stale cancel-all response callbacks
  ++current_spin_generation_;           // stale Spin result callbacks
  ++current_compute_path_generation_;   // stale ComputePathToPose result callbacks
  ++return_nav_generation_;             // stale return-to-init NavigateToPose result callbacks

  cancel_ack_received_ = false;
  pending_target_valid_ = false;
  custom_sequence_state_ = CustomSequenceState::IDLE;
  nav_active_ = false;

  if (return_to_init_ && finished_exploring) {
    beginReturnToInitSequence();
  }
}

void Explore::resume()
{
  if (finished_exploring_) {
    // Invalidate any in-flight return-to-init ComputePath, Spin, and
    // NavigateToPose callbacks that may still be pending from the completed
    // exploration session.
    ++current_compute_path_generation_;
    ++current_spin_generation_;
    // Incrementing return_nav_generation_ here ensures that any result callback
    // from a return-to-init NavigateToPose that was cancelled by this resume()
    // call will be silently ignored instead of publishing RETURN_TO_ORIGIN_FAILED.
    // Note: there is a theoretical narrow race window where a genuinely failed
    // (ABORTED) return NavigateToPose result arrives after this increment, causing
    // that failure notification to be silently swallowed. This is accepted because
    // the user has already expressed the intent to resume exploration, making the
    // return-to-init outcome irrelevant to the current operational context.
    ++return_nav_generation_;

    frontier_blacklist_.clear();
    prev_goal_ = geometry_msgs::msg::Point();
    prev_distance_ = 0.0;
    last_progress_ = this->now();
    finished_exploring_ = false;
    move_base_client_->async_cancel_all_goals();
  }

  resuming_ = true;

  // Force the first planning cycle after resume to be treated as fresh.
  // This avoids an immediate early-return when the selected frontier matches
  // the previous goal and makes resumption noticeably more responsive.
  prev_goal_.x = std::numeric_limits<double>::quiet_NaN();
  prev_goal_.y = std::numeric_limits<double>::quiet_NaN();
  prev_distance_ = std::numeric_limits<double>::infinity();
  last_progress_ = this->now();

  // Publish EXPLORATION_STARTED on the very first resume() call ever,
  // EXPLORATION_IN_PROGRESS on all subsequent ones (pause/resume cycles).
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  if (!has_ever_started_) {
    has_ever_started_ = true;
    RCLCPP_INFO(logger_, "Exploration starting for the first time.");
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  } else {
    RCLCPP_INFO(logger_, "Exploration resuming.");
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  }
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<explore::Explore>());
  rclcpp::shutdown();
  return 0;
}
