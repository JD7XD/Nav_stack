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
#include "std_msgs/Int32.h"

class RobotController {
public:
    RobotController() : nh_("~") {
        // Parameters
        nh_.param("linear_speed", linear_speed_, 0.1);     // Adjust linear speed as needed
        nh_.param("angular_speed", angular_speed_, 0.5);   // Adjust angular speed as needed
        nh_.param("encoder_ticks_per_meter", ticks_per_meter_, 1000.0);  // Adjust as needed

        // Publishers and Subscribers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        left_encoder_sub_ = nh_.subscribe("left_encoder", 10, &RobotController::leftEncoderCallback, this);
        right_encoder_sub_ = nh_.subscribe("right_encoder", 10, &RobotController::rightEncoderCallback, this);

        // Initialize twist message
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = 0.0;
    }

    void moveForward(int encoder_ticks) {
        resetEncoders();
        ros::Rate rate(10);  // 10 Hz

        while (ros::ok() && left_encoder_ticks_ < encoder_ticks) {
            cmd_vel_msg_.linear.x = linear_speed_;
            cmd_vel_pub_.publish(cmd_vel_msg_);
            ros::spinOnce();
            rate.sleep();
        }

        stopRobot();
    }

    void turnRight(int encoder_ticks) {
        resetEncoders();
        ros::Rate rate(10);  // 10 Hz

        while (ros::ok() && right_encoder_ticks_ < encoder_ticks) {
            cmd_vel_msg_.angular.z = -angular_speed_;  // Turn right
            cmd_vel_pub_.publish(cmd_vel_msg_);
            ros::spinOnce();
            rate.sleep();
        }

        stopRobot();
    }

    void stopRobot() {
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel_msg_);
    }

private:
    void resetEncoders() {
        left_encoder_ticks_ = 0;
        right_encoder_ticks_ = 0;
    }

    void leftEncoderCallback(const std_msgs::Int32::ConstPtr& msg) {
        left_encoder_ticks_ = msg->data;
    }

    void rightEncoderCallback(const std_msgs::Int32::ConstPtr& msg) {
        right_encoder_ticks_ = msg->data;
    }

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber left_encoder_sub_;
    ros::Subscriber right_encoder_sub_;
    geometry_msgs::Twist cmd_vel_msg_;
    int left_encoder_ticks_;
    int right_encoder_ticks_;
    double linear_speed_;
    double angular_speed_;
    double ticks_per_meter_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    RobotController robot_controller;

    // Move forward for 1000 encoder ticks
    robot_controller.moveForward(1000);

    // Turn right by 90 degrees (approximately 500 encoder ticks for example purposes)
    robot_controller.turnRight(500);

    // Move forward for 500 encoder ticks
    robot_controller.moveForward(500);

    ros::shutdown();
    return 0;
}


    ros::shutdown();
    return 0;
}
