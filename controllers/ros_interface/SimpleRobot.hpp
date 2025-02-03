#ifndef SIMPLEROBOT_HPP
#define SIMPLEROBOT_HPP

#include <webots/Supervisor.hpp>
#include <webots/Lidar.hpp>
#include "Ros.hpp"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace webots;
/*  @param bool dataset_creator: define if the robot waits for position command before sending pointcloud (useful for dataset creation)
    @param bool verbose */
class SimpleRobot {
public:
    SimpleRobot(bool dataset_creator = false, bool verbose = false);
    ~SimpleRobot();

    bool verbose;
    void advertiseTransform();
    void publishLidar2D();
    void publishLidar3D();
    void teleop_cmd_vel();
    void teleop_cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg);
    void set_position();
    void set_position_Callback(const geometry_msgs::Pose::ConstPtr& msg);


    ros::NodeHandle n;
    Supervisor* supervisor;
    Node* simpleRobot_node;
    int timeStep;


    // TF
    tf2_ros::TransformBroadcaster tfBroadcaster;
    geometry_msgs::TransformStamped transformMsg;
    const double *position;
    const double *rotation; 
    tf2::Quaternion quaternion;
    tf2::Matrix3x3 mat;

    // 2D Lidar
    Lidar *lidar2D;
    int numRays2D;
    double minRange2D, maxRange2D, fov2D;
    const float *ranges2D;

    sensor_msgs::LaserScan scanMsg2D;
    ros::Publisher lidar2DPub;

    // 3D Lidar
    Lidar *lidar3D;
    int numRays3D, numLayers3D;
    double minRange3D, maxRange3D, fov3D;
    const float *ranges3D;

    sensor_msgs::PointCloud2 scanMsg3D;
    ros::Publisher lidar3DPub;
    sensor_msgs::PointCloud2Modifier* modifier;

    // Teleoperation 
    bool cmd_vel_received = false;
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber teleop_sub;
    float v_robot = 0, w_robot = 0;
    double velocity_world[6] = {0, 0, 0, 0, 0, 0};

    // Set position
    bool set_position_received = false;
    Field *translationField, *rotationField;
    double set_position_translation[3] = {0, 0, 0};
    tf2::Quaternion set_position_quaternion;
    tf2::Matrix3x3 set_position_mat;
    double set_position_RPY[3] = {0, 0, 0};
    double set_position_rotation[4] = {0, 0, 0, 0};
    ros::Subscriber set_position_sub;

    // Modes of operation
    bool dataset_creator;
    bool send_PCL;

};

#endif // ROBOT_HPP