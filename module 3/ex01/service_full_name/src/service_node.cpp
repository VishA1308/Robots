#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/full_name_summ.hpp"

#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

class FullNameService : public rclcpp::Node
{
public:
  FullNameService()
  : Node("service_name")
  {
    service_ = this->create_service<service_full_name::srv::FullNameSumm>(
      "SummFullName",
      std::bind(&FullNameService::handle_service, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "Service is ready");
  }

private:
  void handle_service(
    const std::shared_ptr<service_full_name::srv::FullNameSumm::Request> request,
    const std::shared_ptr<service_full_name::srv::FullNameSumm::Response> response)
  {
    // Объединяем фамилию, имя и отчество
    response->full_name = request->last_name + " " + request->name + " " + request->first_name;
    
    RCLCPP_INFO(this->get_logger(), "Received: %s %s %s -> Sending: %s",
                request->last_name.c_str(), request->name.c_str(), 
                request->first_name.c_str(), response->full_name.c_str());
  }

  rclcpp::Service<service_full_name::srv::FullNameSumm>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FullNameService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
