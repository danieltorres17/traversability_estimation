#include "traversability_estimation/RoughTerrainTrav.h"

RoughTerrainTrav::RoughTerrainTrav(ros::NodeHandle& nodehandle)
  : nh_(nodehandle),
    trav_type_("traversability"),
    slope_type_("traversability_slope"),
    step_type_("traversability_step"),
    roughness_type_("traversability_roughness"),
    filter_chain_("grid_map::GridMap") {
  // read config parameters
  LoadConfigParameters();

  // set up subscriber
  elev_map_in_sub_ = nh_.subscribe(elev_map_in_topic_, 1, &RoughTerrainTrav::ElevationMapInCallback, this);

  // set up publisher
  trav_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("traversability_map", 1);
}

bool RoughTerrainTrav::LoadConfigParameters() {
  // set elevation map topic
  nh_.param("elevation_map_topic", elev_map_in_topic_, elev_map_in_topic_);
  // set filter chain parameter name
  nh_.param("filter_chain_parameter_name", filter_chain_param_name_,
            std::string("traversability_map_filters"));
  // configure filter chain
  if (!filter_chain_.configure("traversability_map_filters", nh_)) {
    ROS_ERROR("Could not configure filter chain!");
  }
  return true;
}

void RoughTerrainTrav::ElevationMapInCallback(const grid_map_msgs::GridMap& msg) {
  // convert message to grid map
  grid_map::GridMap map_in;
  grid_map::GridMapRosConverter::fromMessage(msg, map_in);

  // apply filter chain
  grid_map::GridMap map_out;
  if (!filter_chain_.update(map_in, map_out)) {
    ROS_ERROR("Could not update grid map filter chain!");
    return;
  }

  // set processed grid map to be traversability map
  trav_map_ = map_out;

  // publish traversability map
  PublishTravMap();
}

void RoughTerrainTrav::PublishTravMap() {
  // convert traversability map to grid map message and publish
  grid_map_msgs::GridMap map_out;
  grid_map::GridMapRosConverter::toMessage(trav_map_, map_out);
  trav_map_pub_.publish(map_out);
}
