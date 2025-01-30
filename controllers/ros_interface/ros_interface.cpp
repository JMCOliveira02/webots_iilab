#include <webots/Supervisor.hpp>
#include <webots/Lidar.hpp>

#include "Ros.hpp"
#include "Robot.hpp"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace webots;


int main(int argc, char **argv) {
    ros::init(argc, argv, "robot");

    SimpleRobot robot;

    // Main loop
    while (robot.supervisor->step(robot.timeStep) != -1 && ros::ok()) {
        ros::spinOnce();
        
        robot.advertiseTransform();
    }

    return 0;
}
