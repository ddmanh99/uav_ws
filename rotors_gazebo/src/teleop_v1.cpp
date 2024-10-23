#include <thread>
#include <chrono>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose robotPose;

// Hàm để đọc ký tự từ bàn phím không chờ đợi
char getch()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void robotPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    robotPose = *msg;
    // std::cout << "--" << std::endl;
    // std::cout << robotPose.position.x << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;

    ros::Publisher trajectory_pub =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    ROS_INFO("Started teleop control.");

    // std_srvs::Empty srv;
    // bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;
    double offset_z = 0.5;

    // while (i <= 10 && !unpaused)
    // {
    //     ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    //     unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    //     ++i;
    // }

    // if (!unpaused)
    // {
    //     ROS_FATAL("Could not wake up Gazebo.");
    //     return -1;
    // }
    // else
    // {
    //     ROS_INFO("Unpaused the Gazebo simulation.");
    // }

    ros::Duration(5.0).sleep();

    // Eigen::Vector3d position(0.0, 0.0, 3.0 + offset_z);
    double desired_yaw_init = M_PI / 4;
    double yaw = desired_yaw_init;

    double delta_omega = M_PI / 8;
    double delta_v = 0.5; // Tốc độ di chuyển

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::Pose>("/hummingbird/odometry_sensor1/pose", 10, robotPoseCallback);
    ros::AsyncSpinner spinner(1); // Sử dụng 1 thread
    spinner.start();

    ROS_INFO("Controls: \n'w': Move forward\n'x': Move backward\n'a': Move left\n'd': Move right\n'c': Move up\n'z': Move down\n'e': Rotate right\n'q': Rotate left\n'i, j': Velocity\n'o,k': Rotation");
    while (ros::ok())
    {

        char key = getch();
        Eigen::Vector3d position(robotPose.position.x, robotPose.position.y, robotPose.position.z + offset_z);
        // std::cout << robotPose.position.x << " " << robotPose.position.y << " " << robotPose.position.z << std::endl;
        switch (key)
        {
        case 'i': // Tang v
            delta_v += 0.1;
            break;
        case 'j': // Giam v
            delta_v -= 0.1;
            break;
        case 'o': // Tang omega
            delta_omega += M_PI / 36;
            break;
        case 'k': // Giam omega
            delta_omega -= M_PI / 36;
            break;
        case 'w': // Di chuyển lên theo trục x
            position.x() += delta_v;
            break;
        case 'x': // Di chuyển lui theo trục x
            position.x() -= delta_v;
            break;
        case 'a':                    // Di chuyển sang trái theo trục y
            position.y() += delta_v; // Sửa để di chuyển trái
            break;
        case 'd': // Di chuyển sang phải theo trục y
            position.y() -= delta_v;
            break;
        case 'c': // Di chuyển lên theo trục z
            position.z() += delta_v;
            break;
        case 'z': // Di chuyển xuống theo trục z
            position.z() -= delta_v;
            break;
        case 'e': // Xoay sang phải
            yaw -= delta_omega;
            break;
        case 'q': // Xoay sang trái
            yaw += delta_omega;
            break;
        default:
            ROS_WARN("Invalid key pressed.");
            break;
        }
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
            position, yaw, &trajectory_msg);

        ROS_INFO("Publishing waypoint: [%f, %f, %f, %f]. Velocity, Omega: [%f, %f].",
                 position.x(), position.y(), position.z() - offset_z, (yaw - desired_yaw_init) * 180 / M_PI,
                 delta_v, delta_omega * 180 / M_PI);

        trajectory_pub.publish(trajectory_msg);
        // ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
