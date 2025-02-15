#include "center_point.h"

int main(int argc,char ** argv)
{
  
  std::cout<<" hello world "<<std::endl;
  ros::init(argc,argv,"center_point_node");
  ROS_INFO("center_point_node");
  CenterPointNode app;
  app.init();
  app.createRosPubSub();
  ros::spin();
  return 0;
}
