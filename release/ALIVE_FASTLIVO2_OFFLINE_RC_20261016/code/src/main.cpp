#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  try
  {
    LIVMapper mapper(nh);
    mapper.initializeSubscribersAndPublishers(nh, it);
    mapper.run();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("FAST-LIVO2 startup failed: %s", error.what());
    return 2;
  }
  return 0;
}
