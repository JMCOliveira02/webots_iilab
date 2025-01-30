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

class SimpleRobot {
public:
    // Initialize ROS node
    ros::NodeHandle n;

    Supervisor* supervisor;
    Node* simpleRobot_node;
    int timeStep;

    // TF
    tf2_ros::TransformBroadcaster tfBroadcaster;
    const double *position;
    const double *rotation; 

    // 2D Lidar
    Lidar *lidar2D;
    int numRays2D;
    double minRange2D, maxRange2D, fov2D;

    ros::Publisher lidar2DPub = n.advertise<sensor_msgs::LaserScan>("scan2D", 10);
    sensor_msgs::LaserScan scanMsg2D;
    const float *ranges2D;
    
    SimpleRobot() {

        // Initialize Webots supervisor
        supervisor = new webots::Supervisor();
        simpleRobot_node = supervisor->getSelf();
        timeStep = static_cast<int>(supervisor->getBasicTimeStep());

        // Initialize and enable 2D Lidar
        lidar2D->enable(timeStep);
        int numRays2D = lidar2D->getHorizontalResolution();
        double minRange2D = lidar2D->getMinRange();
        double maxRange2D = lidar2D->getMaxRange();
        double fov2D = lidar2D->getFov();
        scanMsg2D.header.frame_id = "robot";
        scanMsg2D.angle_min = -fov2D / 2.0;
        scanMsg2D.angle_max = fov2D / 2.0;
        scanMsg2D.angle_increment = fov2D / numRays2D;
        scanMsg2D.time_increment = 0.0; 
        scanMsg2D.scan_time = timeStep / 1000.0;
        scanMsg2D.range_min = minRange2D;
        scanMsg2D.range_max = maxRange2D;
    }

    void advertiseTransform() {
        position = simpleRobot_node->getPosition();
        rotation = simpleRobot_node->getOrientation();
        
        tf2::Matrix3x3 mat(rotation[0], rotation[1], rotation[2], 
                           rotation[3], rotation[4], rotation[5], 
                           rotation[6], rotation[7], rotation[8]);

        tf2::Quaternion quaternion;
        mat.getRotation(quaternion);
        
        geometry_msgs::TransformStamped transformMsg;
        transformMsg.header.stamp = ros::Time::now();
        transformMsg.header.frame_id = "world";
        transformMsg.child_frame_id = "robot";
        
        transformMsg.transform.translation.x = position[0];
        transformMsg.transform.translation.y = position[1];
        transformMsg.transform.translation.z = position[2];
        
        transformMsg.transform.rotation.x = quaternion.x();
        transformMsg.transform.rotation.y = quaternion.y();
        transformMsg.transform.rotation.z = quaternion.z();
        transformMsg.transform.rotation.w = quaternion.w();
        
        tfBroadcaster.sendTransform(transformMsg);
    }

    void publishLidar2D() {
        ranges2D = lidar2D->getRangeImage();
        scanMsg2D.header.stamp = ros::Time::now();
        scanMsg2D.ranges.resize(numRays2D);
        for (int i = 0; i < numRays2D; ++i) {
            float distance = ranges2D[i];
            scanMsg2D.ranges[i] = (distance >= minRange2D && distance <= maxRange2D) ? distance : std::numeric_limits<float>::infinity();
        }
        lidar2DPub.publish(scanMsg2D);
    }
    
};
