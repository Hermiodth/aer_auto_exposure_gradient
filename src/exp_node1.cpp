#include "aer_auto_exposure_gradient/auto_exp.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exp_node = std::make_shared<exp_node::ExpNode>();
  exp_node->init(exp_node);
  rclcpp::spin(exp_node);
  rclcpp::shutdown();
  return 0;
}
