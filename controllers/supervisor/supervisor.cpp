#include <webots/Supervisor.hpp>
#include <webots/Lidar.hpp>

#include "Ros.hpp"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace webots;

float vx = 0.0, vy = 0.0, vz = 0.0, wx = 0.0, wy = 0.0, wz = 0.0; 

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg){
  ROS_INFO("Received velocity command!");
  vx = msg->linear.x;
  vy = msg->linear.y;
  vz = msg->linear.z;
  
  wx = msg->angular.x;
  wy = msg->angular.y;
  wz = msg->angular.z;
}

int main(int argc, char **argv) {
  // Webots stuff
  Supervisor *supervisor = new Supervisor();
  Node *simpleRobot_node = supervisor->getSelf();
  // WS // Lidar 
  Lidar *lidar = supervisor->getLidar("lidar2D");
  int timeStep = static_cast<int>(supervisor->getBasicTimeStep());
  lidar->enable(timeStep);
  int numRays = lidar->getHorizontalResolution();
  double minRange = lidar->getMinRange();
  double maxRange = lidar->getMaxRange();
  double fov = lidar->getFov();
   
  // ROS stuff
  ros::init(argc, argv, "robot");
  ros::NodeHandle n;
  
  // ROS // Velocity subscriber
  ros::Subscriber cmd_velSub;
  cmd_velSub = n.subscribe("/cmd_vel", 1, &cmd_velCallback);
  
  // ROS // TF broadcaster
  tf2_ros::TransformBroadcaster tfBroadcaster;
  geometry_msgs::TransformStamped transformMsg;
  
  // ROS // Lidar publisher
  ros::Publisher lidarPub = n.advertise<sensor_msgs::LaserScan>("scan", 10);
  
  sensor_msgs::LaserScan scanMsg;
  scanMsg.header.frame_id = "robot";
  scanMsg.angle_min = -fov / 2.0;
  scanMsg.angle_max = fov / 2.0;
  scanMsg.angle_increment = fov / numRays;
  scanMsg.time_increment = 0.0; 
  scanMsg.scan_time = timeStep / 1000.0;
  scanMsg.range_min = minRange;
  scanMsg.range_max = maxRange;

  
  
  // Other stuff
  double velocity[6] = {0, 0, 0, 0, 0, 0};
  
  while(supervisor->step(timeStep) != -1 && ros::ok())
  {
    ros::spinOnce();
    // Apply velocity from subscribed cmd_vel topic
    const double *rotation = simpleRobot_node->getOrientation();
    // Transform linear velocity from robot frame to world frame
    double vx_world = rotation[0] * vx + rotation[1] * vy + rotation[2] * vz;
    double vy_world = rotation[3] * vx + rotation[4] * vy + rotation[5] * vz;
    double vz_world = rotation[6] * vx + rotation[7] * vy + rotation[8] * vz;

    // Set the velocity array with transformed values
    velocity[0] = vx_world;
    velocity[1] = vy_world;
    velocity[2] = vz_world;
    velocity[3] = wx;
    velocity[4] = wy;
    velocity[5] = wz;
    simpleRobot_node->setVelocity(velocity);
    
    // Advertise transform
    const double *position = simpleRobot_node->getPosition();
    

    tf2::Matrix3x3 mat(rotation[0], rotation[1], rotation[2], 
                   rotation[3], rotation[4], rotation[5], 
                   rotation[6], rotation[7], rotation[8]);

    tf2::Quaternion quaternion;
    mat.getRotation(quaternion);
    
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
    
    // Publish Lidar laser_scan
    const float *ranges = lidar->getRangeImage();
    scanMsg.header.stamp = ros::Time::now();
    scanMsg.ranges.resize(numRays);
    for (int i = 0; i < numRays; ++i) {
        float distance = ranges[i];
        scanMsg.ranges[i] = (distance >= minRange && distance <= maxRange) ? distance : std::numeric_limits<float>::infinity();
    }
    lidarPub.publish(scanMsg);
    
  }
}
