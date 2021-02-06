#include "deploy_planner/deploy_planner.h"
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <visualization_msgs/Marker.h>

DeployPlanner::DeployPlanner(tf2_ros::Buffer& tf)
 :tf_(tf), map_({"elevation"}),
  filter_chain_("grid_map::GridMap")
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("filter_chain_parameter_name", filter_chain_parameter_name_, std::string("grid_map_filters"));
    private_nh.param("probe_tf", probe_tf_, std::string("base_link"));

    private_nh.param("probe_range_limit_x", probe_range_limit_x_, NAN);
    private_nh.param("probe_range_limit_y", probe_range_limit_y_, NAN);
    private_nh.param("probe_range_limit_z_down", probe_range_limit_z_down_, NAN);
    private_nh.param("probe_range_limit_z_up", probe_range_limit_z_up_, NAN);

    private_nh.param("uav_traversability_threshold", uav_traversability_threshold_, 0.05);
    private_nh.param("ugv_traversability_threshold", ugv_traversability_threshold_, 0.52);

    private_nh.param("visualize_position", visualize_position_, true);
    private_nh.param("visualize_grid_map", visualize_grid_map_, true);
    private_nh.param("visualize_elevation_map", visualize_elevation_map_, true);

    private_nh.param("grid_map_center_x", grid_map_center_x_, 0.0);
    private_nh.param("grid_map_center_y", grid_map_center_y_, 0.0);
    private_nh.param("grid_map_length_x", grid_map_length_x_, 10.0);
    private_nh.param("grid_map_length_y", grid_map_length_y_, 10.0);

    deploy_planning_service_ = nh.advertiseService("deploy/probe", &DeployPlanner::get_pos_callback, this);
    gridmap_crient_ = nh.serviceClient<grid_map_msgs::GetGridMap>("/elevation_mapping/get_raw_submap");

    map_.setBasicLayers({"elevation"});

    if (!filter_chain_.configure(filter_chain_parameter_name_, nh)) {
        ROS_ERROR("Could not configure the filter chain!");
    }
    if (visualize_grid_map_) {
        grid_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    }
    if (visualize_position_) {
        landing_marker_publisher_ = nh.advertise<visualization_msgs::Marker>("deploy_marker", 1, true);
    }
}

DeployPlanner::~DeployPlanner(){}

bool DeployPlanner::get_pos_callback(
    base_landing_planner::GetPos::Request &req,
    base_landing_planner::GetPos::Response &res)
{
    ROS_INFO("Service call invoked.");

    grid_map_msgs::GetGridMap gridmap_srv;
    gridmap_srv.request.frame_id = "map";
    gridmap_srv.request.position_x = grid_map_center_x_;
    gridmap_srv.request.position_y = grid_map_center_y_;
    gridmap_srv.request.length_x = grid_map_length_x_;
    gridmap_srv.request.length_y = grid_map_length_y_;

    if(!gridmap_crient_.call(gridmap_srv)){
        ROS_ERROR("could not get the grid map!");
        return false;
    }
    ROS_INFO("Service call invoked.");

    grid_map::GridMapRosConverter::fromMessage(gridmap_srv.response.map, map_);
    ROS_INFO("Service call invoked.");

    grid_map::GridMap outputmap;
    if (!filter_chain_.update(map_, outputmap)) {
        ROS_ERROR("could not update the grid map filter chain!");
        return false;
    }
    ROS_INFO("Service call invoked.");

    if (visualize_grid_map_) {
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(outputmap, gridMapMessage);
        grid_map_publisher_.publish(gridMapMessage);
    }
    ROS_INFO("Service call invoked.");

    grid_map::Position landing_position(grid_map_center_x_, grid_map_center_y_);
    grid_map::Index landing_index;

    bool find_deploy_point = false;
    double r, s, e;
    double d, d_max = 0.0;
    for (grid_map::SpiralIterator outside_iterator(outputmap, landing_position, probe_range_limit_x_ / 2.0); !outside_iterator.isPastEnd(); ++outside_iterator) {

        grid_map::Position p1; outputmap.getPosition(*outside_iterator, p1);
        r = outputmap.at("roughness_deviation", *outside_iterator);
        s = outputmap.at("slope_inflated", *outside_iterator);
        e = outputmap.at("elevation", *outside_iterator);

        if(s < ugv_traversability_threshold_){

            find_deploy_point = true;

            for (grid_map::SpiralIterator inside_iterator(outputmap, p1, 3); !inside_iterator.isPastEnd(); ++inside_iterator) {

                grid_map::Position p2; outputmap.getPosition(*inside_iterator, p2);
                s = outputmap.at("slope_inflated", *inside_iterator);

                if(ugv_traversability_threshold_ < s){
                    if(d_max < (p1-p2).norm()){
                        res.position.x = p1[0];
                        res.position.y = p1[1];
                        res.position.z = e;

                        if(r < uav_traversability_threshold_){
                            res.result = base_landing_planner::GetPos::Response::DEPLOY_LANDING;
                        } else {
                            res.result = base_landing_planner::GetPos::Response::DEPLOY_HOVERING;
                        }

                        d_max = (p1-p2).norm();
                    }
                    break;
                }
            }
        }
    }

    if(!find_deploy_point){
        res.position.x = NAN;
        res.position.y = NAN;
        res.position.z = NAN;
    }

    if (visualize_grid_map_) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        geometry_msgs::Point start_point, end_point;
        start_point.x = end_point.x = res.position.x;
        start_point.y = end_point.y = res.position.y;
        start_point.z = end_point.z = res.position.z;
        start_point.z += 0.7;

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.1;
        marker.scale.y = 0.3;
        marker.scale.z = 0.5;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        landing_marker_publisher_.publish(marker);
    }

    return true;
}
