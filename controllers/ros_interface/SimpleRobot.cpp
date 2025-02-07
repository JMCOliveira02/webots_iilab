#include "SimpleRobot.hpp"

SimpleRobot::SimpleRobot(bool dataset_creator, bool verbose) {
        
        SimpleRobot::dataset_creator = dataset_creator;
        SimpleRobot::verbose = verbose;


        // Initialize Webots supervisor
        supervisor = new webots::Supervisor();
        simpleRobot_node = supervisor->getSelf();
        timeStep = static_cast<int>(supervisor->getBasicTimeStep());
        verbose ? printf("Supervisor initialized\n") : 0;

        // Initialize and enable 2D Lidar
        lidar2DPub = n.advertise<sensor_msgs::LaserScan>("scan2D", 10);
        lidar2D = supervisor->getLidar("lidar2D");
        lidar2D->enable(timeStep);
        numRays2D = lidar2D->getHorizontalResolution();
        minRange2D = lidar2D->getMinRange();
        maxRange2D = lidar2D->getMaxRange();
        fov2D = lidar2D->getFov();
        scanMsg2D.header.frame_id = "base_link";
        scanMsg2D.angle_min = -fov2D / 2.0;
        scanMsg2D.angle_max = fov2D / 2.0;
        scanMsg2D.angle_increment = fov2D / numRays2D;
        scanMsg2D.time_increment = 0.0; 
        scanMsg2D.scan_time = timeStep / 1000.0;
        scanMsg2D.range_min = minRange2D;
        scanMsg2D.range_max = maxRange2D;
        verbose ? printf("2D Lidar initialized\n") : 0;

        // Initialize and enable 3D Lidar
        lidar3DPub = n.advertise<sensor_msgs::PointCloud2>("scan3D", 10);
        lidar3D = supervisor->getLidar("lidar3D");
        lidar3D->enable(timeStep);
        numRays3D = lidar3D->getHorizontalResolution();
        minRange3D = lidar3D->getMinRange();
        maxRange3D = lidar3D->getMaxRange();
        numLayers3D = lidar3D->getNumberOfLayers();
        fov3D = lidar3D->getFov();
        scanMsg3D.header.frame_id = "base_link";
        scanMsg3D.height = 1;
        scanMsg3D.is_dense = false;
        sensor_msgs::PointCloud2Modifier modifier(scanMsg3D);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(numRays3D * numLayers3D);
        verbose ? printf("3D Lidar initialized\n") : 0;

        // Initialize teleoperation
        teleop_sub = n.subscribe("cmd_vel", 1, &SimpleRobot::teleop_cmd_vel_Callback, this);
        verbose ? printf("Teleoperation initialized\n") : 0;

        // Initialize set_position
        set_position_sub = n.subscribe("set_position", 1, &SimpleRobot::set_position_Callback, this);
        translationField = simpleRobot_node->getField("translation");
        rotationField = simpleRobot_node->getField("rotation");
        verbose ? printf("Set position initialized\n") : 0;

}

SimpleRobot::~SimpleRobot() {
    delete supervisor;
    delete modifier;
}

void SimpleRobot::advertiseTransform() {
    verbose ? printf("Advertising transform...\n") : 0;
    position = simpleRobot_node->getPosition();
    rotation = simpleRobot_node->getOrientation();
    
    mat.setValue(rotation[0], rotation[1], rotation[2], 
                       rotation[3], rotation[4], rotation[5], 
                       rotation[6], rotation[7], rotation[8]);

    quaternion.setRPY(0, 0, 0);
    mat.getRotation(quaternion);
    
    transformMsg.header.stamp = ros::Time::now();
    transformMsg.header.frame_id = "odom";
    transformMsg.child_frame_id = "base_link";
    
    transformMsg.transform.translation.x = position[0];
    transformMsg.transform.translation.y = position[1];
    transformMsg.transform.translation.z = position[2];
    
    transformMsg.transform.rotation.x = quaternion.x();
    transformMsg.transform.rotation.y = quaternion.y();
    transformMsg.transform.rotation.z = quaternion.z();
    transformMsg.transform.rotation.w = quaternion.w();
    
    tfBroadcaster.sendTransform(transformMsg);
    verbose ? printf("Transform advertised\n") : 0;
}

void SimpleRobot::publishLidar2D() {
    verbose ? printf("Publishing 2D Lidar...\n") : 0;
    ranges2D = lidar2D->getRangeImage();
    scanMsg2D.header.stamp = ros::Time::now();
    scanMsg2D.ranges.resize(numRays2D);
    for (int i = 0; i < numRays2D; ++i) {
        float distance = ranges2D[i];
        scanMsg2D.ranges[i] = (distance >= minRange2D && distance <= maxRange2D) ? distance : std::numeric_limits<float>::infinity();
    }
    lidar2DPub.publish(scanMsg2D);
    verbose ? printf("2D Lidar published\n") : 0;
}

void SimpleRobot::publishLidar3D() {
    if(dataset_creator){
        if(!send_PCL) return;
        send_PCL = false;
    }
    verbose ? printf("Publishing 3D Lidar...\n") : 0;
    ranges3D = lidar3D->getRangeImage();
    scanMsg3D.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2Iterator<float> iter_x(scanMsg3D, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(scanMsg3D, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(scanMsg3D, "z");

    std::vector<double> horizontal_angles(numRays3D);
    std::vector<double> vertical_angles(numLayers3D);

    for (int ray = 0; ray < numRays3D; ++ray) {
        horizontal_angles[ray] = (ray / static_cast<double>(numRays3D) - 0.5) * fov3D;
    }

    for (int layer = 0; layer < numLayers3D; ++layer) {
        vertical_angles[layer] = (layer / static_cast<double>(numLayers3D) - 0.5) * lidar3D->getVerticalFov();
    }

    int idx = 0;
    for (int layer = 0; layer < numLayers3D; ++layer) {
        double vertical_angle = vertical_angles[layer];
        for (int ray = 0; ray < numRays3D; ++ray) {
            float distance = ranges3D[idx++];
            double horizontal_angle = horizontal_angles[ray];
            if (distance >= minRange3D && distance <= maxRange3D) {
                // Convert polar coordinates to Cartesian (x, y, z)
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
    lidar3DPub.publish(scanMsg3D);
    verbose ? printf("3D Lidar published\n") : 0;
}

void SimpleRobot::teleop_cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg) {
    verbose ? printf("Teleoperation command received\n") : 0;
    v_robot = msg->linear.x;
    w_robot = msg->angular.z;
    cmd_vel_received = true;
}

void SimpleRobot::teleop_cmd_vel() {
    if (!cmd_vel_received) {
        return;
    }
    cmd_vel_received = false;
    velocity_world[0] = rotation[0] * v_robot;
    velocity_world[1] = rotation[3] * v_robot;
    velocity_world[2] = rotation[6] * v_robot; 
    velocity_world[5] = w_robot;  
    simpleRobot_node->setVelocity(velocity_world);
    verbose ? printf("Teleoperation command executed\n") : 0;

}

void SimpleRobot::set_position_Callback(const geometry_msgs::Pose::ConstPtr& msg) {
    verbose ? printf("Set position received\n") : 0;
    set_position_translation[0] = msg->position.x;
    set_position_translation[1] = msg->position.y;
    set_position_translation[2] = msg->position.z;
    set_position_quaternion.setX(msg->orientation.x);
    set_position_quaternion.setY(msg->orientation.y);
    set_position_quaternion.setZ(msg->orientation.z);
    set_position_quaternion.setW(msg->orientation.w);
    set_position_received = true;
    
}

void SimpleRobot::set_position() {
    if (!set_position_received) {
        return;
    }
    set_position_received = false;
    mat.setRotation(set_position_quaternion);
    mat.getRPY(set_position_RPY[0], set_position_RPY[1], set_position_RPY[2]);
    set_position_rotation[0] = 0;
    set_position_rotation[1] = 0;
    set_position_rotation[2] = 1;
    set_position_rotation[3] = set_position_RPY[2];
    translationField->setSFVec3f(set_position_translation);
    rotationField->setSFRotation(set_position_rotation);
    if(dataset_creator){
        send_PCL = true;
    }
    verbose ? printf("Set position executed\n") : 0;
}
