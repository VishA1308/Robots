#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/full_name_summ.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class FullNameClient : public rclcpp::Node
{
public:
  FullNameClient() : Node("client_name")
  {
    client_ = this->create_client<service_full_name::srv::FullNameSumm>("SummFullName");
  }

  bool send_request(const std::string& last_name, const std::string& name, const std::string& first_name)
  {
    // Ждем пока сервис станет доступен
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<service_full_name::srv::FullNameSumm::Request>();
    request->last_name = last_name;
    request->name = name;
    request->first_name = first_name;

    auto future = client_->async_send_request(request);
    
    // Ждем ответ
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s", response->full_name.c_str());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      return false;
    }
  }

private:
  rclcpp::Client<service_full_name::srv::FullNameSumm>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Проверяем аргументы командной строки
  if (argc != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: client_name <last_name> <name> <first_name>");
    return 1;
  }

  auto client_node = std::make_shared<FullNameClient>();
  bool success = client_node->send_request(argv[1], argv[2], argv[3]);

  rclcpp::shutdown();
  return success ? 0 : 1;
}
