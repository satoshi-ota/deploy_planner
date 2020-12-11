/*
* octomap_to_gridmap_demo_node.cpp
*
*  Created on: May 03, 2017
*      Author: Jeff Delmerico
*   Institute: University of Zürich, Robotics and Perception Group
*/

#include <tf2_ros/transform_listener.h>
#include <deploy_planner/deploy_planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deploy_planner_node");

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    DeployPlanner deploy_planner_node(buffer);

    ros::Duration(2.0).sleep();
    ros::spin();
    return 0;
}
