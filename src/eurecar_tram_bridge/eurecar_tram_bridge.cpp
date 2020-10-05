/*
 * @file eurecar_tram_bridge.cpp
 * @date 2020-09-24
 * @author sw
 * @brief simple bridge between eurecar_ros and tram for message type conversion
 *        to not make any additional branch
*/

#include "eurecar_tram_bridge/eurecar_tram_bridge.h"


eurecar_tram_bridge::eurecar_tram_bridge() : nh_(""), private_nh_("~"), steering_ratio_(M_PI / 180 * 17.154744)
{
    // parameters
    private_nh_.param<std::string>("lane_topic_name", lane_topic_name_, "final_waypoints/local_coordinate");
    private_nh_.param<std::string>("frenet_path_topic_name", frenet_path_topic_name_, "frenet_path");
    private_nh_.param<bool>("use_frenet_candidates", use_frenet_candidates_, true);

    // subscribers
    ackermann_command_subscriber_ = nh_.subscribe("Ackermann/command", 1, &eurecar_tram_bridge::AckermannCommandCallback, this);
    eurecar_can_t_subscriber_     = nh_.subscribe(common_ros_param::rosTopicName::VEH_CAN, 10, &eurecar_tram_bridge::EurecarCanCallback, this);
    path_subscriber_              = nh_.subscribe("/vehicle/local_coor_wpt", 1, &eurecar_tram_bridge::PathCallback, this);
    // path_subscriber_              = nh_.subscribe(common_ros_param::rosTopicName::VEH_LOCAL_WPT, 1, &eurecar_tram_bridge::PathCallback, this);
    lane_subscriber_              = nh_.subscribe(lane_topic_name_, 1, &eurecar_tram_bridge::LaneCallback, this);
    frenet_local_subscriber_      = nh_.subscribe(frenet_path_topic_name_, 1, &eurecar_tram_bridge::FrenetLocalCallback, this);

    // publishers
    ackermann_state_publisher_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("Ackermann/veh_state", 1);
    lane_to_path_publisher_ = nh_.advertise<nav_msgs::Path>("/final_waypoints/pathmsg", 1);
    frenet_to_path_publisher_ = nh_.advertise<nav_msgs::Path>("frenet_path/pathmsg", 1);
    if(use_frenet_candidates_ == true){
        /* TODO: add more path candidates to local_weighted_trajectories
         * @brief: get rollouts with cost from frenet planner
         *         Probably not needed. Send local_weighted_trajectories directly from the frenet planner.
        */
        lane_with_cost_publisher_  = nh_.advertise<autoware_msgs::Lane>("local_trajectory_cost", 1);
        lane_array_candidate_publisher_  = nh_.advertise<autoware_msgs::LaneArray>("local_weighted_trajectories", 1);
    }
    else{
        // @brief use standard costmap_based_... planning
        lane_array_publisher_  = nh_.advertise<autoware_msgs::LaneArray>("local_trajectories", 1); // path candidates without costs
    }
    
}

eurecar_tram_bridge::~eurecar_tram_bridge()
{

}

void eurecar_tram_bridge::EurecarCanCallback(const eurecar_lcm_to_ros_publisher::eurecar_can_t &message_can_t)
{
    /* @brief convert CAN messages into Ackermann states */
    ackermann_msgs::AckermannDriveStamped msg;
    // TODO: test if stamp.fromSec(message_can_t.time) works with UNIX ROS time
    msg.header.stamp.fromSec(message_can_t.time); 
    // msg.header.stamp = ros::Time::now();
    msg.drive.steering_angle = message_can_t.mdps_str_ang * steering_ratio_; // wheel angle 
    msg.drive.steering_angle_velocity = 0.0;
    msg.drive.speed = message_can_t.VS_CAN;
    msg.drive.acceleration = 0.0; // TODO: decide whether to put IMU
    msg.drive.jerk = 0.0;
    ackermann_state_publisher_.publish(msg);
}

void eurecar_tram_bridge::AckermannCommandCallback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg)
{
    /* @brief convert Ackermann command to controlCMD message */
    /* ??? */
    // common_ros_param::rosTopicName::VEH_CTR_INPUT
}

void eurecar_tram_bridge::PathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    /* @brief convert path messages (possibly from frenet planner) to lane array(local_trajectories topic from trajectory_generate_node) 
     *        msg       : local coordinate
     *        lane_array: ??(TODO)
     * */
    PublishLane(*msg);
}

void eurecar_tram_bridge::LaneCallback(const autoware_msgs::Lane::ConstPtr &msg)
{
    /* @brief (temporal)visualising autoware lane */
    nav_msgs::Path path;
    path.header = msg->header;
    for(int i = 0; i < msg->waypoints.size(); i++){
        path.poses.push_back(msg->waypoints[i].pose);
    }
    lane_to_path_publisher_.publish(path);
}

void eurecar_tram_bridge::FrenetLocalCallback(const eurecar_msgs::frenetPath &msg)
{
    /* @brief convert path messages (possibly from frenet planner) to lane array(local_trajectories topic from trajectory_generate_node) 
     *        msg       : local coordinate
     *        lane_array: ??(TODO)
     * */
    PublishLane(msg.mincost_frenet_path);
    /* @brief (temporal)visualising frenet msg */
    frenet_to_path_publisher_.publish(msg.mincost_frenet_path);
}

// @brief Publish lane messages to behavior node(use_frenet_candidates_: true) or costmap node(use_frenet_candidates_: false)
void eurecar_tram_bridge::PublishLane(const nav_msgs::Path &msg)
{
    int path_length = msg.poses.size();

    if(use_frenet_candidates_ == true){
        // single local trajectory with cost
        autoware_msgs::Lane lane;
        lane.header     = msg.header;
        lane.cost       = 0.0;
        for(int i = 0; i < path_length; i++){
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = msg.poses[i].pose.position.x;
            waypoint.pose.pose.position.y = msg.poses[i].pose.position.y;
            lane.waypoints.push_back(waypoint);
        }
        lane_with_cost_publisher_.publish(lane);

        // candidates with cost, only one candidate for now
        autoware_msgs::LaneArray lane_array;
        lane_array.lanes.push_back(lane);
        lane_array_candidate_publisher_.publish(lane_array);
    }
    else{
        autoware_msgs::Lane lane;
        autoware_msgs::LaneArray lane_array;
        lane.header = msg.header;
        for(int i = 0; i < path_length; i++){
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = msg.poses[i].pose.position.x;
            waypoint.pose.pose.position.y = msg.poses[i].pose.position.y;
            lane.waypoints.push_back(waypoint);
        }
        lane_array.lanes.push_back(lane);
        lane_array_publisher_.publish(lane_array);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eurecar_tram_bridge");
    // ros::NodeHandle nh("");
    // eurecar_tram_bridge bridge(nh);
    eurecar_tram_bridge bridge;
    ros::spin();
    return 0;
}