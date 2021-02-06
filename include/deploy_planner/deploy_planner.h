#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <filters/filter_chain.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "base_landing_planner/GetPos.h"

class DeployPlanner
{
public:
    DeployPlanner(tf2_ros::Buffer& tf);
    ~DeployPlanner();

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

    ros::ServiceServer deploy_planning_service_;
    ros::ServiceClient gridmap_crient_;
    ros::Publisher grid_map_publisher_;
    ros::Publisher landing_marker_publisher_;

    tf::TransformListener listener_;

    double deploy_point_x_;
    double deploy_point_y_;

    double grid_map_center_x_;
    double grid_map_center_y_;
    double grid_map_length_x_;
    double grid_map_length_y_;

    float probe_range_limit_x_;
    float probe_range_limit_y_;
    float probe_range_limit_z_down_;
    float probe_range_limit_z_up_;

    double uav_traversability_threshold_;
    double ugv_traversability_threshold_;

    bool visualize_position_;
    bool visualize_grid_map_;
    bool visualize_elevation_map_;
};
