#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <filters/filter_chain.hpp>

#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <Eigen/Core>

#include <param_io/get_param.hpp>

#include <vector>
#include <string>
#include <algorithm>

class RoughTerrainTrav {
public:
  RoughTerrainTrav(ros::NodeHandle& nodehandle);
  void ElevationMapInCallback(const grid_map_msgs::GridMap& msg);
  bool LoadConfigParameters();

private:
  void PublishTravMap();

private:
  ros::NodeHandle nh_;
  ros::Subscriber elev_map_in_sub_;
  ros::Publisher trav_map_pub_;

  filters::FilterChain<grid_map::GridMap> filter_chain_;
  grid_map::GridMap trav_map_;

  std::string elev_map_in_topic_;
  const std::string trav_type_;
  const std::string slope_type_;
  const std::string step_type_;
  const std::string roughness_type_;
  std::string filter_chain_param_name_;
};
