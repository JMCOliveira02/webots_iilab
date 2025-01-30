#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>

const double target_positions[][2] =
    {
        {-10.0, -6.2}, {-9.5, -6.2}, {-9.0, -6.2}, {-8.5, -6.2}, {-8.0, -6.2}, {-7.5, -6.2}, {-7.0, -6.2}, {-6.5, -6.2}, {-6.0, -6.2}, {-5.5, -6.2},         
{-10.0, -6.7}, {-9.5, -6.7}, {-9.0, -6.7}, {-8.5, -6.7}, {-8.0, -6.7}, {-7.5, -6.7}, {-7.0, -6.7}, {-6.5, -6.7}, {-6.0, -6.7}, {-5.5, -6.7},         
{-10.0, -7.2}, {-9.5, -7.2}, {-9.0, -7.2}, {-8.5, -7.2}, {-8.0, -7.2}, {-7.5, -7.2}, {-7.0, -7.2}, {-6.5, -7.2}, {-6.0, -7.2}, {-5.5, -7.2},         
{-10.0, -7.7}, {-9.5, -7.7}, {-9.0, -7.7}, {-8.5, -7.7}, {-8.0, -7.7}, {-7.5, -7.7}, {-7.0, -7.7}, {-6.5, -7.7}, {-6.0, -7.7}, {-5.5, -7.7},         
{-10.0, -8.2}, {-9.5, -8.2}, {-9.0, -8.2}, {-8.5, -8.2}, {-8.0, -8.2}, {-7.5, -8.2}, {-7.0, -8.2}, {-6.5, -8.2}, {-6.0, -8.2}, {-5.5, -8.2},         
{-10.0, -8.7}, {-9.5, -8.7}, {-9.0, -8.7}, {-8.5, -8.7}, {-8.0, -8.7}, {-7.5, -8.7}, {-7.0, -8.7}, {-6.5, -8.7}, {-6.0, -8.7}, {-5.5, -8.7},         
{-10.0, -9.2}, {-9.5, -9.2}, {-9.0, -9.2}, {-8.5, -9.2}, {-8.0, -9.2}, {-7.5, -9.2}, {-7.0, -9.2}, {-6.5, -9.2}, {-6.0, -9.2}, {-5.5, -9.2}   
};



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
    std::string pcl_filename = "dataset/B2/pcl/pointcloud" + std::to_string(index_file) + ".pcd";
    if (pcl::io::savePCDFileASCII(pcl_filename, cloud) == 0)
    {
        ROS_INFO("Saved point cloud to %s", pcl_filename.c_str());
        // Save the pose to a text file
        std::string pose_filename = "dataset/B2/pose/pose" + std::to_string(index_file) + ".txt";
        savePoseToFile(targetPose, pose_filename);
        pointCloudSaved = true; // Set flag to indicate success
        index_file++;
    }
    else
    {
        ROS_ERROR("Failed to save point cloud to %s", pcl_filename.c_str());
    }
}

int main(int argc, char **argv)
{
    const int num_positions = sizeof(target_positions) / sizeof(target_positions[0]);
    // Initialize ROS node
    ros::init(argc, argv, "pose_and_pointcloud_saver");
    ros::NodeHandle nh;

    // Publisher for setting the robot's pose
    ros::Publisher posePub = nh.advertise<geometry_msgs::Pose>("/set_position", 1);

    // Subscriber for the PointCloud2 topic
    ros::Subscriber sub = nh.subscribe("/pointcloud3D", 1, pointCloudCallback);

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

    float x_min = 3.9;
    float x_max = 3.9;
    float x_spacing = 0.5;
    float y_min = 3;
    float y_max = 3;
    float y_spacing = 0.5;

    // Calculate the number of grid points
    int num_x = static_cast<int>((x_max - x_min) / x_spacing) + 1;
    int num_y = static_cast<int>((y_max - y_min) / y_spacing) + 1;

    for (int i = 0; i < num_x; ++i)
    {
        for (int j = 0; j < num_y; ++j)
        {
            targetPose.position.x = x_min + i * x_spacing;
            targetPose.position.y = y_min + j * y_spacing;

            ROS_INFO("Publishing pose: x=%.2f, y=%.2f, z=%.2f",
                     targetPose.position.x, targetPose.position.y, targetPose.position.z);

            // Publish the pose
            posePub.publish(targetPose);
            pointCloudSaved = false;

            while (ros::ok() && !pointCloudSaved) // Wait until the point cloud is saved
            {
                ros::spinOnce();
                rate.sleep();
            }
        }
    }

    return 0;
}