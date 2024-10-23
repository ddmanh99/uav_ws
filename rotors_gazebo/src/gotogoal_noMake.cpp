#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <Eigen/Core>
#include <cmath>

const double GOAL_TOLERANCE = 0.4; // Tolerance for reaching the goal
const double MOVE_STEP = 0.5;      // Step size for movement

class GoToGoal
{
public:
    GoToGoal()
    {

        // Initialize publishers and subscribers
        m_trajectoryPub = m_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        m_pointReachedPub = m_nh.advertise<std_msgs::Bool>("/red/exploration/point_reached", 1);
        m_goalSub = m_nh.subscribe<geometry_msgs::PoseStamped>("/red/exploration/goal", 1, &GoToGoal::goalCallback, this);
        m_robotPoseSub = m_nh.subscribe<geometry_msgs::Pose>("/hummingbird/odometry_sensor1/pose", 1, &GoToGoal::robotPoseCallback, this);

        // m_pointReachedPub = m_nh.advertise<std_msgs::Bool>("/octomanager/exploration/point_reached", 1);
        // m_goalSub = m_nh.subscribe<geometry_msgs::PoseStamped>("/uav1/exploration/goal", 1, &GoToGoal::goalCallback, this);
        // m_robotPoseSub = m_nh.subscribe<geometry_msgs::Pose>("/hummingbird/odometry_sensor1/pose", 1, &GoToGoal::robotPoseCallback, this);

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

    void checkGoalReached()
    {
        if (!m_goalPose.header.stamp.isZero())
        {
            double dx = m_robotPose.position.x - m_goalPose.pose.position.x;
            double dy = m_robotPose.position.y - m_goalPose.pose.position.y;
            double dz = m_robotPose.position.z - m_goalPose.pose.position.z;

            double distance = sqrt(dx * dx + dy * dy + dz * dz);
            std_msgs::Bool reachedMsg;
            reachedMsg.data = (distance < GOAL_TOLERANCE);
            m_pointReachedPub.publish(reachedMsg);
            if (distance < GOAL_TOLERANCE)
            {
                ROS_INFO("Robot has reached the goal.");
                // Optionally, stop the robot or take some action
                return;
            }
            else
            {
                moveToGoal();
                ROS_INFO("Robot is %f meters away from the goal.", distance);
            }
        }
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

            ROS_INFO("Publishing waypoint: [%f, %f, %f].",
                     goal_position.x(), goal_position.y(), goal_position.z());

            m_trajectoryPub.publish(trajectory_msg);
        }
    }

    ros::NodeHandle m_nh;
    ros::Subscriber m_goalSub;
    ros::Subscriber m_robotPoseSub;
    ros::Publisher m_trajectoryPub;
    ros::Publisher m_pointReachedPub;
    geometry_msgs::PoseStamped m_goalPose;
    geometry_msgs::Pose m_robotPose;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_to_goal_node");
    GoToGoal goToGoal;
    goToGoal.spin();
    return 0;
}
