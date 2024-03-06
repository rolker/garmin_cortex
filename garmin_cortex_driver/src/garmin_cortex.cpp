#include "garmin_cortex/cortex.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "garmin_cortex");

  garmin_cortex::Cortex cortex;

  ros::spin();


  return 0;
}

