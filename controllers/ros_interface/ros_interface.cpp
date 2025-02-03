#include "SimpleRobot.hpp"

using namespace webots;


int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_1");
    printf("ROS node initialized\n");
    SimpleRobot robot_1(false, false);
    if(robot_1.verbose) printf("robot_1 initialized with dataset_creator as %s\n", robot_1.dataset_creator ? "true" : "false");
    // Main loop
    while (robot_1.supervisor->step(robot_1.timeStep) != -1 && ros::ok()) {
        ros::spinOnce();
        robot_1.advertiseTransform();
        robot_1.publishLidar2D();
        robot_1.publishLidar3D();
        robot_1.teleop_cmd_vel();
        robot_1.set_position();
    }

    return 0;
}
