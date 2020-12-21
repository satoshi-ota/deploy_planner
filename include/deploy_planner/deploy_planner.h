#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include "base_landing_planner/GetPos.h"

class DeployPlanner
{
public:
    DeployPlanner(tf2_ros::Buffer& tf);
    ~DeployPlanner();

    void octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg);
    void convert_and_publish();
    bool get_pos_callback(base_landing_planner::GetPos::Request &req, base_landing_planner::GetPos::Response &res);
    bool getGlobalPose(geometry_msgs::PoseStamped& global_pose);

private:

    tf2_ros::Buffer& tf_;

    grid_map::GridMap map_;
    std::string octomap_service_;
    std::string probe_tf_;
    std::string filter_chain_parameter_name_;
    filters::FilterChain<grid_map::GridMap> filter_chain_;

    ros::ServiceServer probe_service_;
    ros::Publisher grid_map_publisher_;
    ros::Publisher elev_map_publisher_;
    ros::Publisher octomap_publisher_;
    ros::Publisher landing_marker_publisher_;
    ros::Subscriber octomap_subscriber_;

    tf::TransformListener listener_;
    octomap_msgs::Octomap octomap_;

    float probe_range_limit_x_;
    float probe_range_limit_y_;
    float probe_range_limit_z_down_;
    float probe_range_limit_z_up_;
    float probe_traversability_threshold_;
    float landing_traversability_threshold_;
    bool octomap_received_;
    bool visualize_position_;
    bool visualize_grid_map_;
    bool visualize_elevation_map_;
};
