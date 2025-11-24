#include "lidar_object_fusion/lidar_object_fusion.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);

  ROSRangeVisionFusionApp app;

  app.Run();

  return 0;
}
