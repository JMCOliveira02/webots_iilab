#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Global flag to indicate point cloud is saved
bool pointCloudSaved = false;
int index_file = 0;
// Define a pose to publish
geometry_msgs::Pose targetPose;

void savePoseToFile(const geometry_msgs::Pose &pose, const std::string &filename)
{
    std::ofstream file(filename);
    if (file.is_open())
    {
        // Write position and orientation in a single line, space-separated
        file << pose.position.x << " "
             << pose.position.y << " "
             << pose.position.z << " "
             << pose.orientation.x << " "
             << pose.orientation.y << " "
             << pose.orientation.z << " "
             << pose.orientation.w << "\n";

        file.close();
        std::cout << "Pose saved to " << filename << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}

// Callback to process the incoming PointCloud2 message
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Save to a PCD file
    std::string pcl_filename = "dataset/pcl/pointcloud" + std::to_string(index_file) + ".pcd";
    if (pcl::io::savePCDFileASCII(pcl_filename, cloud) == 0)
    {
        ROS_INFO("Saved point cloud to %s", pcl_filename.c_str());
        pointCloudSaved = true; // Set flag to indicate success
    }
    else
    {
        ROS_ERROR("Failed to save point cloud to %s", pcl_filename.c_str());
    }

    // Save the pose to a text file
    std::string pose_filename = "dataset/pose/pose" + std::to_string(index_file) + ".txt";
    savePoseToFile(targetPose, pose_filename);
    index_file++;
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "pose_and_pointcloud_saver");
    ros::NodeHandle nh;

    // Publisher for setting the robot's pose
    ros::Publisher posePub = nh.advertise<geometry_msgs::Pose>("/set_position", 1);

    // Subscriber for the PointCloud2 topic
    ros::Subscriber sub = nh.subscribe("/pointcloud3D", 1, pointCloudCallback);

    targetPose.position.x = 0.0;
    targetPose.position.y = -5.0;
    targetPose.position.z = 0.33;
    targetPose.orientation.x = 0.0;
    targetPose.orientation.y = 0.0;
    targetPose.orientation.z = 0.0;
    targetPose.orientation.w = 1.0;

    // Publish the pose
    ros::Duration(0.5).sleep(); // Sleep for 500 ms
    ROS_INFO("Publishing target pose...");
    posePub.publish(targetPose);

    // Wait for the point cloud to be saved
    ros::Rate rate(10); // 10 Hz loop
    int timeout = 100;  // Maximum wait time (in loops)
    int count = 0;

    while (ros::ok() && !pointCloudSaved && count < timeout)
    {
        ros::spinOnce();
        rate.sleep();
        count++;
    }

    if (pointCloudSaved)
    {
        ROS_INFO("Point cloud successfully saved.");
    }
    else
    {
        ROS_WARN("Timed out waiting for point cloud.");
    }

    return 0;
}