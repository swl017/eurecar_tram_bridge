/*
 * @file eurecar_tram_bridge.cpp
 * @date 2020-09-24
 * @author sw
 * @brief simple bridge between eurecar_ros and tram for message type conversion
 *        to not make any additional branch
*/

#include "eurecar_tram_bridge/eurecar_tram_bridge.h"


eurecar_tram_bridge::eurecar_tram_bridge(ros::NodeHandle nh)
{
    nh_ = nh;

    // parameters
    nh.param<std::string>("/lane_topic_name", lane_topic_name_, "final_waypoints/local_coordinate");
    nh.param<std::string>("/frenet_path_topic_name", frenet_path_topic_name_, "frenet_path");
    nh.param<bool>("/use_frenet_candidates", use_frenet_candidates_, true);

    // subscribers
    ackermann_command_subscriber_ = nh.subscribe("Ackermann/command", 1, &eurecar_tram_bridge::AckermannCommandCallback, this);
    eurecar_can_t_subscriber_     = nh.subscribe(common_ros_param::rosTopicName::VEH_CAN, 10, &eurecar_tram_bridge::EurecarCanCallback, this);
    path_subscriber_              = nh.subscribe("/vehicle/local_coor_wpt", 1, &eurecar_tram_bridge::PathCallback, this);
    // path_subscriber_              = nh.subscribe(common_ros_param::rosTopicName::VEH_LOCAL_WPT, 1, &eurecar_tram_bridge::PathCallback, this);
    lane_subscriber_              = nh.subscribe(lane_topic_name_, 1, &eurecar_tram_bridge::LaneCallback, this);

    // publishers
    ackermann_state_publisher_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("Ackermann/veh_state", 1);
    path_publisher_ = nh.advertise<nav_msgs::Path>("/final_waypoints/pathmsg", 1);
    if(use_frenet_candidates_ == true){
        /* TODO: incomplete
         * @brief: get rollouts with cost from frenet planner
         *         Probably not needed. Send local_weighted_trajectories directly from the frenet planner.
        */
        lane_with_cost_publisher_  = nh.advertise<autoware_msgs::Lane>("local_trajectory_cost", 1);
        lane_array_candidate_publisher_  = nh.advertise<autoware_msgs::LaneArray>("local_weighted_trajectories", 1);
    }
    else{
        // @brief use standard costmap_based_... planning
        lane_array_publisher_  = nh.advertise<autoware_msgs::LaneArray>("local_trajectories", 1); // path candidates without costs
    }
    
}

eurecar_tram_bridge::~eurecar_tram_bridge()
{
}

void eurecar_tram_bridge::EurecarCanCallback(const eurecar_lcm_to_ros_publisher::eurecar_can_t &message_can_t)
{
    /* @brief convert CAN messages into Ackermann states */
    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp.fromSec(message_can_t.time);
    // msg.header.stamp = ros::Time::now();// TODO: convert wall time (double) to ros Time (Time) ros::Time::Time(message_can_t.time);
    msg.drive.steering_angle = message_can_t.mdps_str_ang * steering_ratio_; // wheel angle 
    msg.drive.steering_angle_velocity = 0.0;
    msg.drive.speed = message_can_t.VS_CAN;
    msg.drive.acceleration = 0.0; // TODO: decide whether to put IMU
    msg.drive.jerk = 0.0;
    ackermann_state_publisher_.publish(msg);
}

void eurecar_tram_bridge::AckermannCommandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
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
    int path_length = msg->poses.size();
    // ROS_WARN("%s", use_frenet_candidates_ ? "true" : "false");
    if(use_frenet_candidates_ == true){
        // single local trajectory with cost
        autoware_msgs::Lane lane;
        lane.header     = msg->header;
        lane.cost       = 0.0;
        for(int i = 0; i < path_length; i++){
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = msg->poses[i].pose.position.x;
            waypoint.pose.pose.position.y = msg->poses[i].pose.position.y;
            lane.waypoints.push_back(waypoint);
        }
        lane_with_cost_publisher_.publish(lane);

        // candidates with cost
        autoware_msgs::LaneArray lane_array;
        // lane_array.lanes[0].header = msg->header;
        // lane_array.lanes[0].cost   = 0.0;
        // for(int i = 0; i < path_length; i++){
        //     lane_array.lanes[0].waypoints[i].pose.pose.position.x = msg->poses[i].pose.position.x;
        //     lane_array.lanes[0].waypoints[i].pose.pose.position.y = msg->poses[i].pose.position.y;
        // }
        lane_array.lanes.push_back(lane);
        lane_array_candidate_publisher_.publish(lane_array);

    }
    else{
        autoware_msgs::Lane lane;
        autoware_msgs::LaneArray lane_array;
        lane.header = msg->header;
        for(int i = 0; i < path_length; i++){
            autoware_msgs::Waypoint waypoint;
            waypoint.pose.pose.position.x = msg->poses[i].pose.position.x;
            waypoint.pose.pose.position.y = msg->poses[i].pose.position.y;
            lane.waypoints.push_back(waypoint);
        }
        lane_array.lanes.push_back(lane);
        lane_array_publisher_.publish(lane_array);
    }

}

void eurecar_tram_bridge::LaneCallback(const autoware_msgs::Lane::ConstPtr &msg)
{
    /* @brief (temporal)visualising autoware lane */
    nav_msgs::Path path;
    path.header = msg->header;
    for(int i = 0; i < msg->waypoints.size(); i++){
        path.poses.push_back(msg->waypoints[i].pose);
    }
    path_publisher_.publish(path);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "eurecar_tram_bridge");
    ros::NodeHandle nh("");
    eurecar_tram_bridge bridge(nh);
    ros::spin();
    return 0;
}