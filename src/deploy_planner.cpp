#include "deploy_planner/deploy_planner.h"
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <visualization_msgs/Marker.h>

DeployPlanner::DeployPlanner(ros::NodeHandle& node_handle)
 :node_handle_(node_handle),
  map_({"elevation"}),
  filter_chain_("grid_map::GridMap"),
  octomap_received_(false)
{
    node_handle_.param("octomap_service_topic", octomap_service_, std::string("/octomap_binary"));
    node_handle_.param("filter_chain_parameter_name", filter_chain_parameter_name_, std::string("grid_map_filters"));
    node_handle_.param("probe_tf", probe_tf_, std::string("base_link"));

    node_handle_.param("probe_range_limit_x", probe_range_limit_x_, NAN);
    node_handle_.param("probe_range_limit_y", probe_range_limit_y_, NAN);
    node_handle_.param("probe_range_limit_z_down", probe_range_limit_z_down_, NAN);
    node_handle_.param("probe_range_limit_z_up", probe_range_limit_z_up_, NAN);
    node_handle_.param("probe_traversability_threshold", probe_traversability_threshold_, 0.8f);
    node_handle_.param("visualize_position", visualize_position_, true);
    node_handle_.param("visualize_grid_map", visualize_grid_map_, true);

    probe_service_ = node_handle_.advertiseService("/deploy_probe", &DeployPlanner::get_pos_callback, this);

    //map_.setBasicLayers({"elevation"});

    if (!filter_chain_.configure(filter_chain_parameter_name_, node_handle_)) {
        ROS_ERROR("Could not configure the filter chain!");
    }
    if (visualize_grid_map_) {
        grid_map_publisher_ = node_handle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    }
    if (visualize_position_) {
        landing_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("deploy_marker", 1, true);
    }
    octomap_subscriber_ = node_handle_.subscribe("/octomap_binary", 10, &DeployPlanner::octomap_callback, this);

}

DeployPlanner::~DeployPlanner() {
}


void DeployPlanner::octomap_callback(const octomap_msgs::Octomap::ConstPtr& map) {
    octomap_ = *map;
    octomap_received_ = true;
}

bool DeployPlanner::get_pos_callback(
    base_landing_planner::GetPos::Request &req,
    base_landing_planner::GetPos::Response &res
) {
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

    float probe_min[3] = {probe_range_limit_x_, probe_range_limit_y_, probe_range_limit_z_down_};
    float probe_max[3] = {probe_range_limit_x_, probe_range_limit_y_, probe_range_limit_z_up_};

    double current_pose[3] = {req.position.x, req.position.y, req.position.z};

    ROS_DEBUG("%f %f %f", min_bound(0), min_bound(1), min_bound(2));
    ROS_DEBUG("%f %f %f", max_bound(0), max_bound(1), max_bound(2));

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
        grid_map::GridMapRosConverter::toMessage(outputmap, gridMapMessage);
        grid_map_publisher_.publish(gridMapMessage);
    }

    grid_map::Position landing_position(current_pose[0], current_pose[1]);
    grid_map::Index landing_index;

    for (grid_map::SpiralIterator iterator(outputmap, landing_position, 3.0); !iterator.isPastEnd(); ++iterator) {

        grid_map::Position p; outputmap.getPosition(*iterator, p);
        double t = outputmap.at("traversability_inflated", *iterator);
        double e = outputmap.at("elevation", *iterator);

        ROS_DEBUG("traversability_inflated %f", t);

        if (t > probe_traversability_threshold_) {

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
                start_point.x = end_point.x = p[0];
                start_point.y = end_point.y = p[1];
                start_point.z = end_point.z = e;
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

            res.position.x = p[0];
            res.position.y = p[1];
            res.position.z = e;
            ROS_DEBUG("%f %f %f", p[0], p[1], e);
            return true;
        }

    }

    ROS_INFO("Could not find safety point.");
    return false;
}
