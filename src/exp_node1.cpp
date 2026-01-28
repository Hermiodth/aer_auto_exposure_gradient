#include "aer_auto_exposure_gradient/auto_exp.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<exp_node::ExpNode>());
  rclcpp::shutdown();
  return 0;
}
