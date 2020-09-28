#include <iostream>
#include <ros/ros.h>
#include <autoware_msgs/LaneArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "common/eurecar_param_utils.h"
#include "eurecar_lcm_to_ros_publisher/eurecar_can_t.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

class eurecar_tram_bridge
{
public:
    eurecar_tram_bridge(ros::NodeHandle nh);
    ~eurecar_tram_bridge();

private:
    ros::NodeHandle nh_;
    ros::Subscriber eurecar_can_t_subscriber_;
    ros::Subscriber eurecar_current_idx_subscriber;
    ros::Subscriber ackermann_command_subscriber_;
    ros::Subscriber path_subscriber_;
    ros::Subscriber lane_subscriber_;
    ros::Publisher ackermann_state_publisher_;
    ros::Publisher lane_array_publisher_;
    ros::Publisher lane_with_cost_publisher_;
    ros::Publisher lane_array_candidate_publisher_;
    ros::Publisher path_publisher_;

    void EurecarCanCallback(const eurecar_lcm_to_ros_publisher::eurecar_can_t &topic_can_t);
    void AckermannCommandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);
    void PathCallback(const nav_msgs::Path::ConstPtr &msg);
    void PathCandidateCallback(const nav_msgs::Path::ConstPtr &msg); // probably not needed. Send local_weighted_trajectories directly from frenet planner
    void LaneCallback(const autoware_msgs::Lane::ConstPtr &msg);

    const double PI = 3.1415926535897932384626433832795;
    /* @brief steering_ratio_
     * For calculating front wheel angle(rad) = steering_ratio_ * steering wheel angle(deg),
     * steering_ratio_                 = (deg to rad) * (steering wheel to front wheel)
     *      (deg to rad)                    = PI / 180
     *      (steering wheel to front wheel) = 17.154744 (Veloster)
     *                                      = 21.511585 (Carnival)
     * sign convention:
     *      front wheel   : left is +
     *      steering wheel: left is + (but not always, case by case)
     */
    double steering_ratio_ = PI / 180 * 17.154744;
    // variables for parameters
    std::string frenet_path_topic_name_;
    std::string lane_topic_name_;
    bool use_frenet_candidates_;

};
