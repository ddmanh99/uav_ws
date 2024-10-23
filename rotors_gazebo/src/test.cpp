/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>
#include <iostream> // Add for input handling

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_point");
    ros::NodeHandle nh;
    ros::Publisher trajectory_pub =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    ROS_INFO("Started publish point example.");

    // std_srvs::Empty srv;
    // bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    // unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
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

    // Wait for 3 seconds to let the Gazebo GUI show up.
    ros::Duration(3.0).sleep();

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    Eigen::Vector3d desired_position(0.0, 0.0, 1.5);
    double desired_yaw_init = M_PI / 4;
    double desired_yaw_deg_init = 45;
    double desired_yaw;
    double desired_yaw_deg;
    while (ros::ok())
    {
        // std::cout << "--> Enter desired x, y, z, yaw: ";
        // std::cin >> desired_position.x() >> desired_position.y() >> desired_position.z() >> desired_yaw_deg;
        // desired_yaw = desired_yaw_deg * M_PI / 180.0 + desired_yaw_init;

        // mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        //     desired_position, desired_yaw_init, &trajectory_msg);

        // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f, %f (deg)].",
        //          nh.getNamespace().c_str(), desired_position.x(),
        //          desired_position.y(), desired_position.z(), desired_yaw);

        // std::this_thread::sleep_for(std::chrono::seconds(1));
        // trajectory_pub.publish(trajectory_msg);

        // ros::spinOnce();
        std::cout << "--> Enter desired x, y, z: ";
        std::cin >> desired_position.x() >> desired_position.y() >> desired_position.z();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
            desired_position, desired_yaw_init, &trajectory_msg);
        ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
                 nh.getNamespace().c_str(), desired_position.x(),
                 desired_position.y(), desired_position.z());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        trajectory_pub.publish(trajectory_msg);

        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
