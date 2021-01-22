#include "deploy_planner/deploy_planner.h"
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <visualization_msgs/Marker.h>

DeployPlanner::DeployPlanner(tf2_ros::Buffer& tf)
 :tf_(tf), map_({"elevation"}),
  filter_chain_("grid_map::GridMap"),
  octomap_received_(false)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // std::string err;
    // while (ros::ok() && !tf_.canTransform("map", "base_link", ros::Time(0), ros::Duration(1.0), &err)){
    //     ROS_INFO_STREAM_NAMED("commander","Waiting for transform to be available. (tf says: " << err << ")");
    // }

    private_nh.param("octomap_service_topic", octomap_service_, std::string("/octomap_binary"));
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

    private_nh.param("deploy_point_x", deploy_point_x_, 0.0);
    private_nh.param("deploy_point_y", deploy_point_y_, 0.0);

    private_nh.param("grid_map_center_x", grid_map_center_x_, 0.0);
    private_nh.param("grid_map_center_y", grid_map_center_y_, 0.0);

    probe_service_ = nh.advertiseService("/deploy_probe", &DeployPlanner::get_pos_callback, this);
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
    octomap_subscriber_ = nh.subscribe("/octomap_binary", 10, &DeployPlanner::octomap_callback, this);

}

DeployPlanner::~DeployPlanner(){}

void DeployPlanner::octomap_callback(const octomap_msgs::Octomap::ConstPtr& map)
{
    octomap_ = *map;
    octomap_received_ = true;
}

bool DeployPlanner::get_pos_callback(
    base_landing_planner::GetPos::Request &req,
    base_landing_planner::GetPos::Response &res)
{
    ROS_INFO("Service call invoked.");

    grid_map_msgs::GetGridMap gridmap_srv;
    gridmap_srv.request.frame_id = "map";
    gridmap_srv.request.position_x = grid_map_center_x_;
    gridmap_srv.request.position_y = grid_map_center_y_;
    gridmap_srv.request.length_x = 10.0;
    gridmap_srv.request.length_y = 10.0;

    if(!gridmap_crient_.call(gridmap_srv)){
        ROS_ERROR("could not get the grid map!");
        return false;
    }

    grid_map::GridMapRosConverter::fromMessage(gridmap_srv.response.map, map_);

    grid_map::GridMap outputmap;
    if (!filter_chain_.update(map_, outputmap)) {
        ROS_ERROR("could not update the grid map filter chain!");
        return false;
    }

    if (visualize_grid_map_) {
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(outputmap, gridMapMessage);
        grid_map_publisher_.publish(gridMapMessage);
    }

    grid_map::Position landing_position(req.position.x, req.position.y);
    grid_map::Index landing_index;

    res.position.x = NAN;
    res.position.y = NAN;
    res.position.z = NAN;
    res.traversability = INFINITY;
    res.result = base_landing_planner::GetPos::Response::DEPLOY_POINT_NOT_FOUND;

    double r, s, e;
    double r_min, s_min;
    for (grid_map::SpiralIterator iterator(outputmap, landing_position, probe_range_limit_x_ / 2.0); !iterator.isPastEnd(); ++iterator) {

        grid_map::Position p; outputmap.getPosition(*iterator, p);
        r = outputmap.at("roughness_deviation", *iterator);
        s = outputmap.at("slope_inflated", *iterator);
        e = outputmap.at("elevation", *iterator);

        if(s < res.traversability && s < ugv_traversability_threshold_){
            res.position.x = p[0];
            res.position.y = p[1];
            res.position.z = e;
            res.traversability = s;

            r_min = r;
            s_min = s;
        }
    }

    ROS_INFO("%f %f", r_min, s_min);

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

        if(r_min < uav_traversability_threshold_){
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        } else {
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        }

        marker.lifetime = ros::Duration();

        landing_marker_publisher_.publish(marker);

        marker.id = 1;
        start_point.x = end_point.x = req.position.x;
        start_point.y = end_point.y = req.position.y;
        start_point.z = end_point.z = outputmap.atPosition("elevation", landing_position);;
        start_point.z += 1.0;

        marker.points.clear();
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.2;
        marker.scale.y = 0.4;
        marker.scale.z = 0.5;

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        landing_marker_publisher_.publish(marker);
    }

    return true;
}

bool DeployPlanner::getGlobalPose(geometry_msgs::PoseStamped& global_pose)
{
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = "mav_base_link";
    robot_pose.header.stamp = ros::Time();

    try
    {
        tf_.transform(robot_pose, global_pose, "map");
    }
    catch (tf2::LookupException& ex)
    {
        ROS_ERROR_THROTTLE_NAMED(1.0,"commander","No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
        ROS_ERROR_THROTTLE_NAMED(1.0,"commander","Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        ROS_ERROR_THROTTLE_NAMED(1.0,"commander","Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }

    return true;
}
