#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "ros2-power-ina3221/node.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<INA3221Publisher> node =
      std::make_shared<INA3221Publisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
