# Nav_stack
Autonomous motions using nav stack ros1

steps to teleop robot and save unknown map 
1. roslaunch urdfe2bot12 gazebo.launch 
2. rviz
3. rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
4. rosrun gmapping slam_gmapping scan:=/scan _base_frame:=body
5. rosrun map_server map_saver 

roslaunch my_robot_name_2dnav move_base.launch - launch the move base for autonomous drive




#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

class RobotController {
public:
    RobotController() : nh_("~") {
        // Parameters
        nh_.param("linear_speed", linear_speed_, 0.2);    // Adjust linear speed as needed
        nh_.param("angular_speed", angular_speed_, 0.5);  // Adjust angular speed as needed

        // Publishers and Subscribers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        odom_sub_ = nh_.subscribe("odom", 10, &RobotController::odomCallback, this);

        // Initialize twist message
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = 0.0;
    }

    void moveForward(double distance) {
        resetOdom();
        ros::Rate rate(10);  // 10 Hz
        double traveled_distance = 0.0;

        while (ros::ok() && traveled_distance < distance) {
            cmd_vel_msg_.linear.x = linear_speed_;
            cmd_vel_pub_.publish(cmd_vel_msg_);
            ros::spinOnce();
            rate.sleep();
            traveled_distance = getTraveledDistance();
        }

        stopRobot();
    }

    void turnRight(double angle) {
        resetOdom();
        ros::Rate rate(10);  // 10 Hz
        double rotated_angle = 0.0;

        while (ros::ok() && rotated_angle < angle) {
            cmd_vel_msg_.angular.z = -angular_speed_;  // Turn right
            cmd_vel_pub_.publish(cmd_vel_msg_);
            ros::spinOnce();
            rate.sleep();
            rotated_angle = getRotatedAngle();
        }

        stopRobot();
    }

    void stopRobot() {
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel_msg_);
    }

private:
    void resetOdom() {
        initial_odom_ = current_odom_;
    }

    double getTraveledDistance() {
        return sqrt(pow(current_odom_.pose.pose.position.x - initial_odom_.pose.pose.position.x, 2) +
                    pow(current_odom_.pose.pose.position.y - initial_odom_.pose.pose.position.y, 2));
    }

    double getRotatedAngle() {
        return atan2(2.0 * (current_odom_.pose.pose.orientation.w * current_odom_.pose.pose.orientation.z +
                           current_odom_.pose.pose.orientation.x * current_odom_.pose.pose.orientation.y),
                     1.0 - 2.0 * (current_odom_.pose.pose.orientation.y * current_odom_.pose.pose.orientation.y +
                                 current_odom_.pose.pose.orientation.z * current_odom_.pose.pose.orientation.z));
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
    }

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    geometry_msgs::Twist cmd_vel_msg_;
    nav_msgs::Odometry initial_odom_;
    nav_msgs::Odometry current_odom_;
    double linear_speed_;
    double angular_speed_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    RobotController robot_controller;

    // Move 2 meters forward
    robot_controller.moveForward(2.0);

    // Turn right by 90 degrees
    robot_controller.turnRight(M_PI / 2.0);

    // Move 1 meter forward
    robot_controller.moveForward(1.0);

    ros::shutdown();
    return 0;
}
