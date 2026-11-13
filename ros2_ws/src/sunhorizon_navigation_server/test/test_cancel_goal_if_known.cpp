#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "sunhorizon_navigation_server/cancel_goal_if_known.hpp"

using namespace std::chrono_literals;
using Action = nav2_msgs::action::NavigateToPose;
using ServerHandle = rclcpp_action::ServerGoalHandle<Action>;
using ClientHandle = rclcpp_action::ClientGoalHandle<Action>;
using sunhorizon_navigation_server::cancel_goal_if_known;

class CancelGoalRace : public ::testing::Test {
 protected:
  void SetUp() override {
    // Only a fake action server on loopback; no Nav2 or command publisher.
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    unsetenv("FASTRTPS_DEFAULT_PROFILES_FILE");
    unsetenv("FASTDDS_DEFAULT_PROFILES_FILE");
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions options;
    options.set_domain_id(91);
    context->init(0, nullptr, options);
    node = std::make_shared<rclcpp::Node>(
        "cancel_goal_race_test", rclcpp::NodeOptions().context(context));
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor->add_node(node);
    server = rclcpp_action::create_server<Action>(
        node, "/cancel_goal_race_test/navigate",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<ServerHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<ServerHandle> handle) { server_handle = handle; });
    client = rclcpp_action::create_client<Action>(node, "/cancel_goal_race_test/navigate");
    ASSERT_TRUE(client->wait_for_action_server(3s));
    auto goal = client->async_send_goal(Action::Goal{});
    ASSERT_EQ(executor->spin_until_future_complete(goal, 3s), rclcpp::FutureReturnCode::SUCCESS);
    client_handle = goal.get();
    ASSERT_NE(client_handle, nullptr);
    ASSERT_NE(server_handle, nullptr);
  }

  void TearDown() override {
    executor->remove_node(node);
    client.reset();
    server.reset();
    node.reset();
    executor.reset();
    context->shutdown("test complete");
  }

  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp_action::Client<Action>::SharedPtr client;
  std::shared_ptr<ServerHandle> server_handle;
  ClientHandle::SharedPtr client_handle;
};

TEST_F(CancelGoalRace, RetiredSuccessHandleReproducesExceptionAndBecomesNoop) {
  auto result = client->async_get_result(client_handle);
  server_handle->succeed(std::make_shared<Action::Result>());
  ASSERT_EQ(executor->spin_until_future_complete(result, 3s), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  // This is the exact Humble failure that escaped the navigation worker.
  EXPECT_THROW(client->async_cancel_goal(client_handle),
               rclcpp_action::exceptions::UnknownGoalHandleError);
  EXPECT_FALSE(cancel_goal_if_known(*client, client_handle));
  EXPECT_TRUE(context->is_valid());
}

TEST_F(CancelGoalRace, ActiveCancelIsSentAndAlreadyCanceledHandleBecomesNoop) {
  auto result = client->async_get_result(client_handle);
  EXPECT_TRUE(cancel_goal_if_known(*client, client_handle));
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!server_handle->is_canceling() && std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(server_handle->is_canceling());
  server_handle->canceled(std::make_shared<Action::Result>());
  ASSERT_EQ(executor->spin_until_future_complete(result, 3s), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_THROW(client->async_cancel_goal(client_handle),
               rclcpp_action::exceptions::UnknownGoalHandleError);
  EXPECT_FALSE(cancel_goal_if_known(*client, client_handle));
}

TEST(CancelGoalErrorHandling, OtherFailuresAreNotMistakenForTerminalGoals) {
  struct FailedTransport {
    void async_cancel_goal(int) { throw std::runtime_error("transport failed"); }
  } client;
  EXPECT_THROW(cancel_goal_if_known(client, 0), std::runtime_error);
}
