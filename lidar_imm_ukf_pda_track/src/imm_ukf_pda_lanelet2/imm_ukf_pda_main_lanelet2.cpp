

#include <imm_ukf_pda_lanelet2/imm_ukf_pda_lanelet2.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imm_ukf_pda_tracker_lanelet2");
  ImmUkfPdaLanelet2 app;
  app.run();
  ros::spin();
  return 0;
}
