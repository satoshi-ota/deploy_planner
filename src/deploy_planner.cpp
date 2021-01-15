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

    std::string err;
    while (ros::ok() && !tf_.canTransform("map", "base_link", ros::Time(0), ros::Duration(1.0), &err)){
        ROS_INFO_STREAM_NAMED("commander","Waiting for transform to be available. (tf says: " << err << ")");
    }

    private_nh.param("octomap_service_topic", octomap_service_, std::string("/octomap_binary"));
    private_nh.param("filter_chain_parameter_name", filter_chain_parameter_name_, std::string("grid_map_filters"));
    private_nh.param("probe_tf", probe_tf_, std::string("base_link"));

    private_nh.param("probe_range_limit_x", probe_range_limit_x_, NAN);
    private_nh.param("probe_range_limit_y", probe_range_limit_y_, NAN);
    private_nh.param("probe_range_limit_z_down", probe_range_limit_z_down_, NAN);
    private_nh.param("probe_range_limit_z_up", probe_range_limit_z_up_, NAN);
    private_nh.param("probe_traversability_threshold", probe_traversability_threshold_, 0.8f);
    private_nh.param("landing_traversability_threshold", landing_traversability_threshold_, 0.8f);
    private_nh.param("visualize_position", visualize_position_, true);
    private_nh.param("visualize_grid_map", visualize_grid_map_, true);
    private_nh.param("visualize_elevation_map", visualize_elevation_map_, true);

    probe_service_ = nh.advertiseService("/deploy_probe", &DeployPlanner::get_pos_callback, this);
    gridmap_crient_ = nh.serviceClient<grid_map_msgs::GetGridMap>("/elevation_mapping/get_raw_submap");

    map_.setBasicLayers({"elevation"});

    if (!filter_chain_.configure(filter_chain_parameter_name_, nh)) {
        ROS_ERROR("Could not configure the filter chain!");
    }
    if (visualize_grid_map_) {
        grid_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
        elev_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("elev_map", 1, true);
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

    octomap::OcTree* octomap = nullptr;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap_);
    if (tree) {
        octomap = dynamic_cast<octomap::OcTree*>(tree);
    } else {
        ROS_ERROR("Failed to call convert Octomap.");
        return;
    }

    grid_map::Position3 min_bound;
    grid_map::Position3 max_bound;
    octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
    octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

    double probe_min[3] = {probe_range_limit_x_, probe_range_limit_y_, probe_range_limit_z_down_};
    double probe_max[3] = {probe_range_limit_x_, probe_range_limit_y_, probe_range_limit_z_up_};

    geometry_msgs::PoseStamped global_pose;
    getGlobalPose(global_pose);

    double current_pose[3] = {global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z};

    for (int i : {0, 1, 2}) {
        if(!std::isnan(probe_min[i])) {
            min_bound(i) = std::max(min_bound(i), current_pose[i] - probe_min[i]);
        }
        if(!std::isnan(probe_max[i])) {
            max_bound(i) = std::min(max_bound(i), current_pose[i] + probe_max[i]);
        }
    }

    if (!grid_map::GridMapOctomapConverter::fromOctomap(*octomap, "elevation", map_, &min_bound, &max_bound)) {
        ROS_ERROR("Failed to call convert Octomap.");
        return;
    }
    map_.setFrameId(octomap_.header.frame_id);

    if (visualize_elevation_map_) {
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
        grid_map_publisher_.publish(gridMapMessage);
    }
}

bool DeployPlanner::get_pos_callback(
    base_landing_planner::GetPos::Request &req,
    base_landing_planner::GetPos::Response &res)
{
    ROS_INFO("Service call invoked.");

    if (!octomap_received_) {
        ROS_ERROR("No octomap received prior to this service call. Aborting.");
        return false;
    }
    octomap::OcTree* octomap = nullptr;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap_);
    if (tree) {
        octomap = dynamic_cast<octomap::OcTree*>(tree);
    } else {
        ROS_ERROR("Failed to call convert Octomap.");
        return false;
    }

    grid_map::Position3 min_bound;
    grid_map::Position3 max_bound;
    octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
    octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

    double probe_min[3] = {probe_range_limit_x_, probe_range_limit_y_, probe_range_limit_z_down_};
    double probe_max[3] = {probe_range_limit_x_, probe_range_limit_y_, probe_range_limit_z_up_};

    double current_pose[3] = {req.position.x, req.position.y, req.position.z};

    for (int i : {0, 1, 2}) {
        if(!std::isnan(probe_min[i])) {
            min_bound(i) = std::max(min_bound(i), current_pose[i] - probe_min[i]);
        }
        if(!std::isnan(probe_max[i])) {
            max_bound(i) = std::min(max_bound(i), current_pose[i] + probe_max[i]);
        }
    }

    if (!grid_map::GridMapOctomapConverter::fromOctomap(*octomap, "elevation", map_, &min_bound, &max_bound)) {
        ROS_ERROR("Failed to call convert Octomap.");
        return false;
    }
    map_.setFrameId(octomap_.header.frame_id);

    grid_map::GridMap outputmap;
    if (!filter_chain_.update(map_, outputmap)) {
        ROS_ERROR("could not update the grid map filter chain!");
        return false;
    }

    if (visualize_grid_map_) {
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
        elev_map_publisher_.publish(gridMapMessage);
    }

    grid_map::Position landing_position(current_pose[0], current_pose[1]);
    grid_map::Index landing_index;

    res.position.x = NAN;
    res.position.y = NAN;
    res.position.z = NAN;
    res.traversability = 0.0;
    res.result = base_landing_planner::GetPos::Response::DEPLOY_POINT_NOT_FOUND;

    for (grid_map::SpiralIterator iterator(outputmap, landing_position, probe_range_limit_x_ / 2.0); !iterator.isPastEnd(); ++iterator) {

        grid_map::Position p; outputmap.getPosition(*iterator, p);
        double t = outputmap.at("traversability_inflated", *iterator);
        double e = outputmap.at("elevation", *iterator);

        if(res.traversability < t && probe_traversability_threshold_ < t){
            res.position.x = p[0];
            res.position.y = p[1];
            res.position.z = e;
            res.traversability = t;
            if(landing_traversability_threshold_ < t){
                res.result = base_landing_planner::GetPos::Response::DEPLOY_LANDING;
            } else {
                res.result = base_landing_planner::GetPos::Response::DEPLOY_HOVERING;
            }
        }
    }

    ROS_INFO_NAMED("deploy_planner","Maximum traversabiilty: %f", res.traversability);

    if (visualize_grid_map_ && res.position.x != NAN && res.position.y != NAN) {
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
        start_point.z += 1.0;

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
