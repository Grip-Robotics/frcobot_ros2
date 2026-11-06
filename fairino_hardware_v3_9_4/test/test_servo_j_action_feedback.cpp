#include "fairino_hardware/servo_j_streamer.hpp"
#include "fairino_hardware/servo_j_action_feedback.hpp"
#include "fairino_msgs/action/stream_servo_j.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmw/rmw.h>
#include <dlfcn.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Action = fairino_msgs::action::StreamServoJ;
using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;
using ClientGoal = rclcpp_action::ClientGoalHandle<Action>;

namespace
{
struct PublicationGate
{
  std::mutex mutex;
  std::condition_variable changed;
  bool entered{false}, released{false};
  std::chrono::steady_clock::time_point entered_at;
};
std::mutex gate_mutex;
std::shared_ptr<PublicationGate> active_gate;

class RmwStall
{
public:
  RmwStall()
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    active_gate = gate;
  }
  ~RmwStall()
  {
    release();
    std::lock_guard<std::mutex> lock(gate_mutex);
    active_gate.reset();
  }
  bool wait_entered()
  {
    std::unique_lock<std::mutex> lock(gate->mutex);
    return gate->changed.wait_for(lock, 2s, [&]() {return gate->entered;});
  }
  void release()
  {
    std::lock_guard<std::mutex> lock(gate->mutex);
    gate->released = true;
    gate->changed.notify_all();
  }
  void hold_for_at_least(std::chrono::milliseconds duration)
  {
    std::unique_lock<std::mutex> lock(gate->mutex);
    gate->changed.wait_until(lock, gate->entered_at + duration,
      [&]() {return gate->released;});
  }
private:
  std::shared_ptr<PublicationGate> gate = std::make_shared<PublicationGate>();
};
}  // namespace

// Stall inside an actual ROS publication, then forward to the real middleware.
// This executable has no vendor SDK and runs only in a network-isolated container.
extern "C" rmw_ret_t rmw_publish(
  const rmw_publisher_t * publisher, const void * message, rmw_publisher_allocation_t * allocation)
{
  using Publish = rmw_ret_t (*)(const rmw_publisher_t *, const void *, rmw_publisher_allocation_t *);
  static auto real_publish = reinterpret_cast<Publish>(dlsym(RTLD_NEXT, "rmw_publish"));
  if (!real_publish) {std::abort();}
  std::shared_ptr<PublicationGate> gate;
  if (publisher && publisher->topic_name &&
    std::string(publisher->topic_name) == "/offline_servo_rebound/stream/_action/feedback")
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    gate = active_gate;
  }
  if (gate) {
    std::unique_lock<std::mutex> lock(gate->mutex);
    if (!gate->entered) {
      gate->entered = true;
      gate->entered_at = std::chrono::steady_clock::now();
      gate->changed.notify_all();
      gate->changed.wait(lock, [&]() {return gate->released;});
    }
  }
  return real_publish(publisher, message, allocation);
}

namespace
{
class VirtualClock : public fairino_hardware::MonotonicClock
{
public:
  int64_t now_nanoseconds() override {return now;}
  int sleep_until(int64_t deadline) override
  {
    if (++sleeps == 2 && second_sleep && !second_sleep()) {return ETIMEDOUT;}
    now = std::max(now, deadline);
    return 0;
  }
  int64_t now{1000000000};
  int sleeps{0};
  std::function<bool()> second_sleep;
};

class FakeRobot : public fairino_hardware::ServoJRobot
{
public:
  explicit FakeRobot(VirtualClock & clock) : clock(clock) {}
  int servo_move_start(int, const fairino_hardware::ServoStartSettings &) override {return 0;}
  int servo_j(const std::array<double, 6> &, float, int, int) override
  {
    ++sends;
    if (sends == delayed_sample) {clock.now += delay_ns;}
    return sdk_error;
  }
  int stop_motion() override {calls.push_back("stop"); return 0;}
  int servo_move_end(int) override {calls.push_back("end"); return 0;}
  VirtualClock & clock;
  int sends{0}, sdk_error{0}, delayed_sample{1};
  int64_t delay_ns{0};
  std::vector<std::string> calls;
};

class ServoJActionFeedback : public ::testing::Test
{
protected:
  struct Goal
  {
    ClientGoal::SharedPtr client;
    std::shared_ptr<ServerGoal> server;
  };
  static void SetUpTestSuite() {int argc = 0; rclcpp::init(argc, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  static fairino_hardware::ServoJStreamerConfig config()
  {
    fairino_hardware::ServoJStreamerConfig value;
    value.feedback_period_sec = 0;
    value.trace_capacity = 4;
    return value;
  }
  void SetUp() override
  {
    const auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r", "offline_servo_test:=/offline_servo_rebound/stream"});
    auto server_options = options;
    server_options.use_intra_process_comms(true);
    server_node = std::make_shared<rclcpp::Node>("offline_servo_server", "/offline_ns", server_options);
    client_node = std::make_shared<rclcpp::Node>("offline_servo_client", "/offline_ns", options);
    server = rclcpp_action::create_server<Action>(server_node, "offline_servo_test",
      [this](const auto &, auto goal) {
        std::string reason;
        return streamer.reserve(request(*goal), reason) ?
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
      },
      [this](auto) {streamer.cancel_and_stop(); return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto goal) {
        std::lock_guard<std::mutex> lock(accepted_mutex);
        accepted.push_back(goal);
        accepted_changed.notify_all();
      });
    client = rclcpp_action::create_client<Action>(client_node, "offline_servo_test");
    feedback_messages = fairino_hardware::create_servo_j_action_feedback_publisher(
      *server_node, "offline_servo_test");
    EXPECT_STREQ(feedback_messages->get_topic_name(),
      "/offline_servo_rebound/stream/_action/feedback");
    executor.add_node(server_node);
    executor.add_node(client_node);
    executor_thread = std::thread([this]() {executor.spin();});
    ASSERT_TRUE(client->wait_for_action_server(2s));
  }
  void TearDown() override
  {
    executor.cancel();
    if (executor_thread.joinable()) {executor_thread.join();}
  }
  static fairino_hardware::ServoJStreamRequest request(const Action::Goal & goal)
  {
    fairino_hardware::ServoJStreamRequest value;
    value.joints_deg = goal.joints_deg;
    value.period_sec = goal.period_sec;
    value.communication_type = goal.communication_type;
    return value;
  }
  Goal submit(size_t samples, const rclcpp_action::Client<Action>::SendGoalOptions & options = {})
  {
    Action::Goal goal;
    goal.joints_deg.resize(samples * 6, 0);
    goal.period_sec = 0.006;
    goal.communication_type = 1;
    auto future = client->async_send_goal(goal, options);
    if (future.wait_for(2s) != std::future_status::ready || !future.get()) {
      throw std::runtime_error("Mock action goal was not accepted");
    }
    std::unique_lock<std::mutex> lock(accepted_mutex);
    if (!accepted_changed.wait_for(lock, 2s, [&]() {return !accepted.empty();})) {
      throw std::runtime_error("Mock action server accepted callback did not run");
    }
    auto server_goal = accepted.front();
    accepted.erase(accepted.begin());
    return {future.get(), server_goal};
  }
  std::shared_ptr<fairino_hardware::ServoJFeedbackPublisher::Session> session(
    fairino_hardware::ServoJFeedbackPublisher & publisher, const Goal & goal)
  {
    return publisher.start(fairino_hardware::servo_j_action_feedback_callback(
        feedback_messages, goal.server->get_goal_id()));
  }
  fairino_hardware::ServoJStreamMetrics run(
    const Goal & goal, fairino_hardware::ServoJFeedbackPublisher::Session & session, int64_t origin)
  {
    clock.sleeps = 0;
    return streamer.run_reserved(request(*goal.server->get_goal()),
      [&](uint64_t n, uint64_t id, int64_t start) {session.offer(n, id, start);},
      [origin]() {return origin;});
  }
  static void finish(const Goal & goal, const fairino_hardware::ServoJStreamMetrics & metrics)
  {
    auto result = std::make_shared<Action::Result>();
    result->success = metrics.success;
    result->cancelled = metrics.cancelled;
    result->samples_requested = metrics.samples_requested;
    result->samples_sent = metrics.samples_sent;
    result->last_command_id = metrics.last_command_id;
    result->controller_error = metrics.controller_error;
    result->message = metrics.message;
    result->missed_deadlines = metrics.missed_deadlines;
    if (goal.server->is_canceling()) {goal.server->canceled(result);}
    else if (metrics.success) {goal.server->succeed(result);}
    else {goal.server->abort(result);}
  }

  VirtualClock clock;
  FakeRobot robot{clock};
  fairino_hardware::ServoJStreamer streamer{robot, clock, config()};
  rclcpp::Node::SharedPtr server_node, client_node;
  rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp_action::Client<Action>::SharedPtr client;
  fairino_hardware::ServoJActionFeedbackPublisher::SharedPtr feedback_messages;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 4};
  std::thread executor_thread;
  std::mutex accepted_mutex;
  std::condition_variable accepted_changed;
  std::vector<std::shared_ptr<ServerGoal>> accepted;
};

TEST_F(ServoJActionFeedback, ResultsAndNextSessionSurviveAnActualRmwPublicationStall)
{
  fairino_hardware::ServoJFeedbackPublisher publisher;
  RmwStall stall;  // Releases the blocked call before publisher destruction, including on failure.
  const auto first = submit(1000);
  auto first_session = session(publisher, first);
  clock.second_sleep = [&]() {return stall.wait_entered();};
  auto metrics = run(first, *first_session, 111);
  ASSERT_TRUE(metrics.success) << metrics.message;
  EXPECT_EQ(metrics.samples_sent, 1000U);
  EXPECT_EQ(metrics.missed_deadlines, 0U);
  EXPECT_TRUE(first_session->close().in_flight_at_close);
  auto received = client->async_get_result(first.client);
  auto completion = std::async(std::launch::async, [&]() {finish(first, metrics);});
  const auto status = received.wait_for(2s);
  if (status != std::future_status::ready) {stall.release();}
  ASSERT_EQ(status, std::future_status::ready);
  EXPECT_EQ(received.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(received.get().result->samples_sent, 1000U);
  completion.get();

  const auto second = submit(2);
  auto second_session = session(publisher, second);
  clock.second_sleep = {};
  metrics = run(second, *second_session, 222);
  second_session->close();
  auto second_result = client->async_get_result(second.client);
  finish(second, metrics);
  ASSERT_EQ(second_result.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(second_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(second_result.get().result->last_command_id, 1002U);
  EXPECT_NE(first.client->get_goal_id(), second.client->get_goal_id());
  const auto canceled = submit(2);
  auto canceled_session = session(publisher, canceled);
  auto cancellation = client->async_cancel_goal(canceled.client);
  ASSERT_EQ(cancellation.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(cancellation.get()->goals_canceling.empty());
  metrics = run(canceled, *canceled_session, 0);
  EXPECT_TRUE(metrics.cancelled);
  canceled_session->close();
  auto canceled_result = client->async_get_result(canceled.client);
  finish(canceled, metrics);
  ASSERT_EQ(canceled_result.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(canceled_result.get().code, rclcpp_action::ResultCode::CANCELED);
  stall.hold_for_at_least(500ms);
  stall.release();

  auto progress = std::make_shared<std::promise<rclcpp_action::GoalUUID>>();
  auto progress_received = progress->get_future();
  auto first_feedback = std::make_shared<std::atomic_bool>(true);
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [progress, first_feedback](auto handle, auto feedback) {
      EXPECT_EQ(rclcpp::Time(feedback->stream_start).nanoseconds(), 333);
      EXPECT_NE(handle, nullptr);
      if (first_feedback->exchange(false)) {
        EXPECT_EQ(feedback->last_command_id, 1003U);
        progress->set_value(handle->get_goal_id());
      }
    };
  const auto third = submit(2, options);
  auto third_session = session(publisher, third);
  clock.second_sleep = [&]() {return progress_received.wait_for(2s) == std::future_status::ready;};
  metrics = run(third, *third_session, 333);
  ASSERT_TRUE(metrics.success) << metrics.message;
  EXPECT_EQ(progress_received.get(), third.server->get_goal_id());
  third_session->close();
  auto third_result = client->async_get_result(third.client);
  finish(third, metrics);
  ASSERT_EQ(third_result.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(third_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(ServoJActionFeedback, ActualActionCancelAndSdkFailuresDeliverTerminalResults)
{
  fairino_hardware::ServoJFeedbackPublisher publisher;
  const auto canceled = submit(10);
  auto canceled_session = session(publisher, canceled);
  auto cancellation = client->async_cancel_goal(canceled.client);
  ASSERT_EQ(cancellation.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(cancellation.get()->goals_canceling.empty());
  auto metrics = run(canceled, *canceled_session, 0);
  canceled_session->close();
  EXPECT_TRUE(metrics.cancelled);
  EXPECT_EQ(robot.sends, 0);
  auto canceled_result = client->async_get_result(canceled.client);
  finish(canceled, metrics);
  ASSERT_EQ(canceled_result.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(canceled_result.get().code, rclcpp_action::ResultCode::CANCELED);

  for (const bool sdk_stall : {false, true}) {
    robot.calls.clear();
    robot.sends = 0;
    robot.sdk_error = sdk_stall ? 0 : 42;
    robot.delay_ns = sdk_stall ? 325090000 : 0;
    robot.delayed_sample = 2;  // A late final send must still fail the action.
    const auto goal = submit(2);
    auto feedback = session(publisher, goal);
    metrics = run(goal, *feedback, 444);
    feedback->close();
    EXPECT_FALSE(metrics.success);
    EXPECT_EQ(metrics.controller_error, sdk_stall ? 0 : 42);
    EXPECT_EQ(metrics.samples_sent, sdk_stall ? 2U : 0U);
    EXPECT_EQ(robot.calls, (std::vector<std::string>{"stop", "end"}));
    auto result = client->async_get_result(goal.client);
    finish(goal, metrics);
    ASSERT_EQ(result.wait_for(2s), std::future_status::ready);
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_FALSE(result.get().result->success);
  }
}
}  // namespace
