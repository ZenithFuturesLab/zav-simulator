#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("laserMapping");
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh);
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}