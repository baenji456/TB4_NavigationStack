#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("apf_node"), "Artificial Potential Field node started...");
  rclcpp::spin(std::make_shared<rclcpp::Node>("apf_dummy_node"));
  rclcpp::shutdown();
  return 0;
}
