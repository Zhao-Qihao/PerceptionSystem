
#include "visualize_rects.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_rects");
  VisualizeRects app;
  ros::spin();

  return 0;
}
