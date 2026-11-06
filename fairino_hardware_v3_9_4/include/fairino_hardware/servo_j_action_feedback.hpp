#ifndef FAIRINO_HARDWARE__SERVO_J_ACTION_FEEDBACK_HPP_
#define FAIRINO_HARDWARE__SERVO_J_ACTION_FEEDBACK_HPP_

#include "fairino_hardware/servo_j_feedback.hpp"
#include "fairino_msgs/action/stream_servo_j.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace fairino_hardware
{
using ServoJActionFeedbackMessage = fairino_msgs::action::StreamServoJ::Impl::FeedbackMessage;
using ServoJActionFeedbackPublisher = rclcpp::Publisher<ServoJActionFeedbackMessage>;

inline ServoJActionFeedbackPublisher::SharedPtr create_servo_j_action_feedback_publisher(
  rclcpp::Node & node, const std::string & action_name)
{
  // ServerGoalHandle publication can hold the action server lock across a stalled RMW call.
  const auto qos = rcl_action_server_get_default_options().feedback_topic_qos;
  const auto resolved = node.get_node_topics_interface()->resolve_topic_name(action_name);
  return node.create_publisher<ServoJActionFeedbackMessage>(resolved + "/_action/feedback",
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
}

inline ServoJFeedbackPublisher::Callback servo_j_action_feedback_callback(
  ServoJActionFeedbackPublisher::SharedPtr publisher, rclcpp_action::GoalUUID goal_id)
{
  return [publisher = std::move(publisher), goal_id](uint64_t sent, uint64_t id, int64_t start) {
      auto message = std::make_unique<ServoJActionFeedbackMessage>();
      message->goal_id.uuid = goal_id;
      message->feedback.samples_sent = sent;
      message->feedback.last_command_id = id;
      message->feedback.stream_start = rclcpp::Time(start);
      publisher->publish(std::move(message));
    };
}
}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__SERVO_J_ACTION_FEEDBACK_HPP_
