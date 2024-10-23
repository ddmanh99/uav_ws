#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <Eigen/Core>
#include <cmath>
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>

const double GOAL_TOLERANCE = 0.3; // Tolerance for reaching the goal
const double MOVE_STEP = 0.5;      // Step size for movement
const double offset_z = 0.2;
const double time_wait_step = 0.75;

class GoToGoal
{
public:
    GoToGoal()
    {

        // Initialize publishers and subscribers
        // 2020
        m_trajectoryPub = m_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        m_pointReachedPub = m_nh.advertise<std_msgs::Bool>("/red/exploration/point_reached", 1);
        m_goalSub = m_nh.subscribe<geometry_msgs::PoseStamped>("/red/exploration/goal", 1, &GoToGoal::goalCallback, this);
        m_robotPoseSub = m_nh.subscribe<geometry_msgs::Pose>("/hummingbird/odometry_sensor1/pose", 100, &GoToGoal::robotPoseCallback, this);
        m_pathSub = m_nh.subscribe<nav_msgs::Path>("/planning/uav_path", 1, &GoToGoal::pathCallback, this);
        m_stop = m_nh.subscribe<std_msgs::Bool>("/stop", 10, &GoToGoal::stopCB, this);

        m_serviceStop = m_nh.advertiseService("stop/toggle",
                                              &GoToGoal::stopToggledServiceCb, this);

        // 2022
        // m_trajectoryPub = m_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        //     mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        // m_pointReachedPub = m_nh.advertise<std_msgs::Bool>("/octomanager/exploration/point_reached", 1);
        // m_goalSub = m_nh.subscribe<geometry_msgs::PoseStamped>("/uav1/exploration/goal", 1, &GoToGoal::goalCallback, this);
        // m_robotPoseSub = m_nh.subscribe<geometry_msgs::Pose>("/hummingbird/odometry_sensor1/pose", 100, &GoToGoal::robotPoseCallback, this);
        // m_pathSub = m_nh.subscribe<nav_msgs::Path>("/planning/uav_path", 1, &GoToGoal::pathCallback, this);
        // m_stop = m_nh.subscribe<std_msgs::Bool>("/stop", 10, &GoToGoal::stopCB, this);

        // m_serviceStop = m_nh.advertiseService("stop/toggle",
        //                                       &GoToGoal::stopToggledServiceCb, this);

        // Initialize goal and robot pose
        m_goalPose.header.stamp = ros::Time(0);
    }

    void spin()
    {
        ros::Rate rate(10); // 10 Hz update rate

        while (ros::ok())
        {
            if (!m_goalPose.header.stamp.isZero())
            {
                checkGoalReached();
                // moveToGoal();
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void robotPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        m_robotPose = *msg;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        m_goalPose = *msg;
    }
    void stopCB(const std_msgs::Bool::ConstPtr &msg)
    {
        stopMsg = *msg;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (msg->poses.size() >= 2)
        {
            for (size_t i = 1; i < msg->poses.size(); ++i)
            {
                // Nếu stopToggled được bật, dừng ngay lập tức
                if (stopToggled)
                {
                    ROS_INFO("Robot dừng ngay lập tức do lệnh dừng.");
                    return; // Thoát khỏi vòng lặp và dừng việc gửi waypoint
                }

                reachedMsg.data = false;
                m_pointReachedPub.publish(reachedMsg);

                double x = msg->poses[i].pose.position.x;
                double y = msg->poses[i].pose.position.y;
                double z = msg->poses[i].pose.position.z + offset_z;

                Eigen::Vector3d goal_position(x, y, z);
                trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
                trajectory_msg.header.stamp = ros::Time::now();
                mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                    goal_position, M_PI / 4, &trajectory_msg);

                m_trajectoryPub.publish(trajectory_msg);

                // Kiểm tra xem robot đã đến điểm hiện tại chưa trước khi đi tiếp
                ros::Rate rate(10); // Tốc độ kiểm tra 10Hz
                while (ros::ok())
                {
                    // Cập nhật thông tin từ các topic (bao gồm vị trí robot)
                    ros::spinOnce();

                    // Tính khoảng cách từ robot đến điểm hiện tại
                    double distance = std::sqrt(std::pow(m_robotPose.position.x - x, 2) +
                                                std::pow(m_robotPose.position.y - y, 2) +
                                                std::pow(m_robotPose.position.z - z, 2));

                    // ROS_INFO("Vị trí hiện tại của robot: [%f, %f, %f]",
                    //          m_robotPose.position.x, m_robotPose.position.y, m_robotPose.position.z);
                    // ROS_INFO("Khoảng cách tới điểm [%f, %f, %f]: %f", x, y, z, distance);

                    if (distance < GOAL_TOLERANCE)
                    {
                        // ROS_INFO("Robot đã đến điểm [%f, %f, %f]", x, y, z);
                        break; // Đến điểm rồi, ra khỏi vòng lặp
                    }

                    // Nếu stopToggled được bật trong quá trình di chuyển, dừng ngay lập tức
                    if (stopToggled)
                    {
                        ROS_INFO("Robot dừng ngay lập tức do lệnh dừng trong quá trình di chuyển.");
                        return;
                    }

                    rate.sleep(); // Tiếp tục kiểm tra khoảng cách sau mỗi 0.1 giây
                }

                if (i < msg->poses.size() - 1)
                {
                    ros::Duration(time_wait_step).sleep(); // Chờ trước khi gửi điểm tiếp theo
                }
            }

            reachedMsg.data = true;
            m_pointReachedPub.publish(reachedMsg);
        }
    }

    void checkGoalReached()
    {
    }

    void moveToGoal()
    {
        if (m_goalPose.header.stamp.isZero())
            return;

        Eigen::Vector3d current_position(m_robotPose.position.x, m_robotPose.position.y, m_robotPose.position.z);
        Eigen::Vector3d goal_position(m_goalPose.pose.position.x, m_goalPose.pose.position.y, m_goalPose.pose.position.z);

        Eigen::Vector3d direction = goal_position - current_position;
        double distance = direction.norm();

        if (distance < GOAL_TOLERANCE)
        {
            ROS_INFO("Goal is within tolerance.");
            return;
        }
        else
        {
            trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
            trajectory_msg.header.stamp = ros::Time::now();
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                goal_position, M_PI / 4, &trajectory_msg);

            // ROS_INFO("Publishing waypoint: [%f, %f, %f].",
            //          goal_position.x(), goal_position.y(), goal_position.z());

            m_trajectoryPub.publish(trajectory_msg);
        }
    }
    bool stopToggledServiceCb(std_srvs::SetBool::Request &request,
                              std_srvs::SetBool::Response &response)
    {
        stopToggled = request.data;
        if (stopToggled)
            std::cout << "Stop ON." << std::endl
                      << std::endl;
        else
            std::cout << "Stop OFF." << std::endl
                      << std::endl;
        response.success = true;
        response.message = "stopToggledService called!";
        return true;
    }

    ros::NodeHandle m_nh;
    ros::Subscriber m_goalSub;
    ros::Subscriber m_robotPoseSub;
    ros::Subscriber m_pathSub;
    ros::Subscriber m_stop;
    ros::Publisher m_trajectoryPub;
    ros::Publisher m_pointReachedPub;
    ros::ServiceServer m_serviceStop;
    std_msgs::Bool reachedMsg;
    std_msgs::Bool stopMsg;
    geometry_msgs::PoseStamped m_goalPose;
    geometry_msgs::Pose m_robotPose;
    bool stopToggled{false};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_to_goal_node");
    GoToGoal goToGoal;
    goToGoal.spin();
    return 0;
}
