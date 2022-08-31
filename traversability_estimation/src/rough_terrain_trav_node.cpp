#include <ros/ros.h>
#include "traversability_estimation/RoughTerrainTrav.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rough_trav_node");
  ros::NodeHandle nh;
  RoughTerrainTrav rtt(nh);

  ros::spin();

  return 0;
}
