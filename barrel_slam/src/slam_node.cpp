#include <rclcpp/rclcpp.hpp>
#include "slam.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Slam>();

  rclcpp::spin(node);
  return 0;

}

