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

float vx = 0.0, vy = 0.0, vz = 0.0, wx = 0.0, wy = 0.0, wz = 0.0; 
double translation_x = 0.0, translation_y = 0.0, translation_z = 0.0;
double rotation_axis0 = 0.0, rotation_axis1 = 0.0, rotation_axis2 = 0.0, rotation_angle = 0.0;

bool send_PCL = 0;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg){
  ROS_INFO("Received velocity command!");
  vx = msg->linear.x;
  vy = msg->linear.y;
  vz = msg->linear.z;
  
  wx = msg->angular.x;
  wy = msg->angular.y;
  wz = msg->angular.z;
}

// ROS subscriber callback to set the robot's position
void set_positionCallback(const geometry_msgs::Pose::ConstPtr &msg, Node *simpleRobot_node) {
  ROS_INFO("Setting robot position!");

  // Extract position from the message
  const double x = msg->position.x;
  const double y = msg->position.y;
  const double z = msg->position.z;

  // Extract orientation (quaternion) from the message
  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;
  const double qw = msg->orientation.w;

  // Convert quaternion to axis-angle representation
  tf2::Quaternion quaternion(qx, qy, qz, qw);
  tf2::Matrix3x3 mat(quaternion);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  // Set position
  Field *translationField = simpleRobot_node->getField("translation");
  double translation[3] = {x, y, z};
  translationField->setSFVec3f(translation);

  // Set orientation
  Field *rotationField = simpleRobot_node->getField("rotation");
  double axis[3] = {0.0, 1.0, 0.0}; // Default axis for rotation around Y-axis
  double angle = yaw;  // Assume yaw as the rotation angle
  double rotation[4] = {axis[0], axis[1], axis[2], angle};
  rotationField->setSFRotation(rotation);
  send_PCL = 1;
  
}


int main(int argc, char **argv) {

  // Webots stuff
  Supervisor *supervisor = new Supervisor();
  Node *simpleRobot_node = supervisor->getSelf();
  int timeStep = static_cast<int>(supervisor->getBasicTimeStep());
  
    // ROS stuff
  ros::init(argc, argv, "robot");
  ros::NodeHandle n;
  
 // WS // Lidar2D 
  Lidar *lidar2D = supervisor->getLidar("lidar2D");
  lidar2D->enable(timeStep);
  int numRays2D = lidar2D->getHorizontalResolution();
  double minRange2D = lidar2D->getMinRange();
  double maxRange2D = lidar2D->getMaxRange();
  double fov2D = lidar2D->getFov();
  

  // WS // Lidar3D 
  Lidar *lidar3D = supervisor->getLidar("lidar3D");
  lidar3D->enable(timeStep);
  int numRays3D = lidar3D->getHorizontalResolution();
  double minRange3D = lidar3D->getMinRange();
  double maxRange3D = lidar3D->getMaxRange();
  double fov3D = lidar3D->getFov();
  // ROS publisher for 3D Lidar point cloud
  ros::Publisher lidar3DPub = n.advertise<sensor_msgs::PointCloud2>("pointcloud3D", 10);
  
  // Create PointCloud2 message
  sensor_msgs::PointCloud2 pointCloud3D;
  pointCloud3D.header.frame_id = "robot";
  pointCloud3D.height = 1;  // Single scan, row-major (height is 1 for unordered clouds)
  pointCloud3D.is_dense = false;  // Allow for invalid points
  
  // PointCloud2 fields
  sensor_msgs::PointCloud2Modifier modifier(pointCloud3D);
  modifier.setPointCloud2FieldsByString(1, "xyz");  // Add x, y, z fields

  // ROS // Velocity subscriber
  ros::Subscriber cmd_velSub;
  cmd_velSub = n.subscribe("/cmd_vel", 1, &cmd_velCallback);
  
  // Subscriber for setting robot position
  ros::Subscriber set_positionSub = n.subscribe<geometry_msgs::Pose>("/set_position", 1,
  boost::bind(&set_positionCallback, _1, simpleRobot_node));
  
  // ROS // TF broadcaster
  tf2_ros::TransformBroadcaster tfBroadcaster;
  geometry_msgs::TransformStamped transformMsg;
  
  // ROS // Lidar publisher
  ros::Publisher lidar2DPub = n.advertise<sensor_msgs::LaserScan>("scan2D", 10);
  
  sensor_msgs::LaserScan scanMsg2D;
  scanMsg2D.header.frame_id = "robot";
  scanMsg2D.angle_min = -fov2D / 2.0;
  scanMsg2D.angle_max = fov2D / 2.0;
  scanMsg2D.angle_increment = fov2D / numRays2D;
  scanMsg2D.time_increment = 0.0; 
  scanMsg2D.scan_time = timeStep / 1000.0;
  scanMsg2D.range_min = minRange2D;
  scanMsg2D.range_max = maxRange2D;
  
  
  
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
    
    // Publish Lidar2D laser_scan
    const float *ranges2D = lidar2D->getRangeImage();
    scanMsg2D.header.stamp = ros::Time::now();
    scanMsg2D.ranges.resize(numRays2D);
    for (int i = 0; i < numRays2D; ++i) {
        float distance = ranges2D[i];
        scanMsg2D.ranges[i] = (distance >= minRange2D && distance <= maxRange2D) ? distance : std::numeric_limits<float>::infinity();
    }
    lidar2DPub.publish(scanMsg2D);
    
        // 3D Lidar point cloud publishing
    const float *ranges3D = lidar3D->getRangeImage();
    pointCloud3D.header.stamp = ros::Time::now();
    
    if(send_PCL)
    {
      send_PCL = 0;
      // Resize PointCloud2 to the number of points
      int totalPoints = lidar3D->getHorizontalResolution() * lidar3D->getNumberOfLayers();
      modifier.resize(totalPoints);
  
      sensor_msgs::PointCloud2Iterator<float> iter_x(pointCloud3D, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pointCloud3D, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pointCloud3D, "z");
  
      int idx = 0;
      for (int layer = 0; layer < lidar3D->getNumberOfLayers(); ++layer) {
          for (int ray = 0; ray < lidar3D->getHorizontalResolution(); ++ray) {
              float distance = ranges3D[idx++];
              if (distance >= minRange3D && distance <= maxRange3D) {
                  // Convert polar coordinates to Cartesian (x, y, z)
                  double horizontal_angle = (ray / (double)lidar3D->getHorizontalResolution() - 0.5) * fov3D;
                  double vertical_angle = (layer / (double)lidar3D->getNumberOfLayers() - 0.5) * lidar3D->getVerticalFov();
                  
                  *iter_x = distance * cos(vertical_angle) * cos(horizontal_angle);
                  *iter_y = distance * cos(vertical_angle) * sin(horizontal_angle);
                  *iter_z = distance * sin(vertical_angle);
              } else {
                  // Invalid point
                  *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
              }
              ++iter_x; ++iter_y; ++iter_z;
          }
      }
      // Publish the 3D point cloud
      lidar3DPub.publish(pointCloud3D);
    
    }


    
  }
}
