#include <chrono>
#include <cstdlib>
#include <memory>

#include "canopen_interfaces/srv/co_target_double.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("position_tick_motor_node");

  RCLCPP_INFO(node->get_logger(), "Position Tick Motor Node Started");
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr init_client =
    node->create_client<std_srvs::srv::Trigger>("/trinamic_pd42/init");
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mode_client =
    node->create_client<std_srvs::srv::Trigger>("/trinamic_pd42/cyclic_position_mode");
  rclcpp::Client<canopen_interfaces::srv::COTargetDouble>::SharedPtr target_client =
    node->create_client<canopen_interfaces::srv::COTargetDouble>("/trinamic_pd42/target");

  while (!init_client->wait_for_service(std::chrono::seconds(1)) &&
         !mode_client->wait_for_service(std::chrono::seconds(1)) &&
         !target_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto trigger_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = init_client->async_send_request(trigger_req);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Init service called successfully");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call init service");
  }

  result = mode_client->async_send_request(trigger_req);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Config position mode service called successfully");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call config service");
  }

  RCLCPP_INFO(node->get_logger(), "Starting to send target values");

  auto targer_req = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
  double target = 0;
  while (rclcpp::ok())
  {
    targer_req->target = target;
    auto res = target_client->async_send_request(targer_req);
    if (rclcpp::spin_until_future_complete(node, res) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "Set Target: %.2f", target);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to call target service");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    target += 1.0;
    if (target >= 105.0) target = 0;
  }

  return 0;
}
