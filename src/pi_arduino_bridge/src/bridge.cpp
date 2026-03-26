#include <chrono>
#include <memory>
#include <string>
#include <cmath>

//ros2 core headers
#include "rclcpp/rclcpp.hpp"
//ros2 message types
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// TF2 headers for SLAM
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

//parser file
#include "dataparser.hpp"

//serial library (requires sudo apt install librclcpp-serial-dev) sudo apt install libserial-dev 
//update package file to include the serial library <depend>serial</depend>
#include "serial/serial.h"

using namespace std::chrono_literals;

//write the ros node here that publishes the data handled from the arduino 
class Bridge : public rclcpp::Node {
public: 
    Bridge() : Node("bridge_node") {
        // Initialize ROS publishers
        //Creating topics for IMU, and left/right encoder data
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
        left_encoder_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_encoder_data", 10);
        right_encoder_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_encoder_data", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                //NEW: subscriber for Nav2 -> Pi -> Arduino 
        //This listens for velocity commands and calls 'cmdVelCallback' to send them to the arduino serial monitor
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Bridge::cmdVelCallback, this, std::placeholders::_1));
        
        //setup the serial connection to the arduino, will print if error
        try{
            ser_.setPort("/dev/arduino"); //try this usb port, may be different. 
            //can check in pi terminal for correct one
            ser_.setBaudrate(1000000);
            auto timeout = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(timeout);
            ser_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port");
        }
        last_time_ = this->get_clock()->now(); 
        //Create a timer sunning at 50Hz / 20 ms 
        timer_ = this->create_wall_timer(20ms, std::bind(&Bridge::processSerial, this));

        RCLCPP_INFO(this->get_logger(), "Bridge node has been started.");
    }


private: 
    double calc_linear_velocity(double omega_L, double omega_R, double wheel_radius) {
    return (omega_L + omega_R) * wheel_radius / 2.0;
    }

    double x_pos_integration(double velocity, double x_old, double angle, double dt) {
        return x_old + velocity * cos(angle) * dt;
    }

    double y_pos_integration(double velocity, double y_old, double angle, double dt) {
        return y_old + velocity * sin(angle) * dt;
    }

    double angle_integration(double old_angle, double dt, double omega_z) {
        return old_angle + omega_z * dt;
    }

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Extract linear and angular velocities from the message
        // linear.x and angular.z are the only relevant fields for our purposes
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        // Convert to wheel speeds (assuming a differential drive robot). 
        //CHECK IF THIS IS THE CORRECT MATH, we might not need it if the PID control only wants linear and angular velocity commands 
        //double wheel_base = 0.2223; // distance between wheels in meters, may need to change?
        //double left_wheel_speed = linear_x - (angular_z * wheel_base / 2.0);
        //double right_wheel_speed = linear_x + (angular_z * wheel_base / 2.0);

        // Send the wheel speeds to the Arduino via serial
        if (ser_.isOpen()) {
            // Format string sent to arduino as starting with "V" so we know to correctly parse for this in PID control code
            std::string command = "V," + std::to_string(linear_x) + "," + std::to_string(angular_z) + "\n";
            ser_.write(command);
            //log this so we make sure Nav2 is actually logging this to the bridge
            RCLCPP_INFO(this->get_logger(), "Sent cmd_vel to Arduino: %s", command.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open. Cannot send cmd_vel data.");
        }
    }

void processSerial() {
        if (ser_.isOpen() && ser_.available()) {
            
            std::string raw_line;
            try {
                // Read until the newline character, ensuring a complete packet
                raw_line = ser_.readline(65536, "\n"); 
            } catch (const serial::SerialException& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s", e.what());
                return;
            }

            // Clean up any trailing \r characters
            raw_line.erase(std::remove(raw_line.begin(), raw_line.end(), '\r'), raw_line.end());
            raw_line.erase(std::remove(raw_line.begin(), raw_line.end(), '\n'), raw_line.end());

            // --- STRICT VALIDATION ---
            int comma_count = std::count(raw_line.begin(), raw_line.end(), ',');
            if (comma_count != 5) {
                // Log the exact string that failed so you can debug the Arduino output
                RCLCPP_WARN(this->get_logger(), "Dropped malformed packet: '%s'", raw_line.c_str());
                return;
            }

            // 1. PARSE FIRST
            data_parser_.parse(raw_line);

            auto current_time = rclcpp::Clock(RCL_ROS_TIME).now();            double dt = (current_time - last_time_).seconds();
            last_time_ = current_time; 

            // 2. NOW DO THE MATH
            // Integrate velocity to get absolute wheel position
            left_wheel_angle += data_parser_.omega_L * dt;
            right_wheel_angle += data_parser_.omega_R * dt;

            // --- JOINT STATES ---
            auto joint_msg = sensor_msgs::msg::JointState();
            joint_msg.header.stamp = current_time;
            joint_msg.name = {"Front_left_wheel_joint", "Back_left_wheel_joint", "Front_right_wheel_joint", "Back_right_wheel_joint"};
            joint_msg.position = {left_wheel_angle, left_wheel_angle, right_wheel_angle, right_wheel_angle};
            joint_msg.velocity = {data_parser_.omega_L, data_parser_.omega_L, data_parser_.omega_R, data_parser_.omega_R};
            joint_state_pub_->publish(joint_msg);

            // --- ENCODERS ---
            auto l_speed = std_msgs::msg::Float64();
            l_speed.data = data_parser_.omega_L;
            left_encoder_pub_->publish(l_speed);

            auto r_speed = std_msgs::msg::Float64();
            r_speed.data = data_parser_.omega_R;
            right_encoder_pub_->publish(r_speed);
            
            // --- ODOMETRY MATH ---
            linear_velocity = calc_linear_velocity(data_parser_.omega_L, data_parser_.omega_R, wheel_radius);
            double angle = angle_integration(angle_old, dt, data_parser_.gyroZ);
            double x_pos = x_pos_integration(linear_velocity, x_old, angle, dt);
            double y_pos = y_pos_integration(linear_velocity, y_old, angle, dt);

            x_old = x_pos;
            y_old = y_pos;
            angle_old = angle;

            // 1. Create the Quaternion from your calculated angle
            tf2::Quaternion q;
            q.setRPY(0, 0, angle);

            // --- IMU (Moved down here to grab the new 'q' variable) ---
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = current_time;
            imu_msg.header.frame_id = "imu_link"; 
            
            // Add the Orientation!
            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            imu_msg.angular_velocity.z = data_parser_.gyroZ;
            imu_msg.linear_acceleration.x = data_parser_.accelX;
            imu_msg.linear_acceleration.y = data_parser_.accelY;
            imu_msg.linear_acceleration.z = data_parser_.accelZ;
            imu_pub_->publish(imu_msg);
            
            // Broadcast Transform
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = current_time;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = x_pos;
            t.transform.translation.y = y_pos;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(t);

            // Publish Odometry
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            odom_msg.pose.pose.position.x = x_pos;
            odom_msg.pose.pose.position.y = y_pos;
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();
            odom_msg.twist.twist.linear.x = linear_velocity;
            odom_msg.twist.twist.angular.z = data_parser_.gyroZ;

            // Optional but highly recommended: Add a tiny bit of covariance so EKFs don't crash
            odom_msg.pose.covariance[0] = 0.01;
            odom_msg.pose.covariance[7] = 0.01;
            odom_msg.pose.covariance[35] = 0.01;
            odom_msg.twist.covariance[0] = 0.01;
            odom_msg.twist.covariance[7] = 0.01;
            odom_msg.twist.covariance[35] = 0.01;

            odom_pub_->publish(odom_msg);

        }
    }

    //member variables 
    serial::Serial ser_;
    DataParser data_parser_;

    double x_old = 0.0;
    double y_old = 0.0;
    double angle_old = 0.0;
    double wheel_radius = 0.0603;
    double linear_velocity = 0.0;
    double left_wheel_angle = 0.0;
    double right_wheel_angle = 0.0;
    rclcpp::Time last_time_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_encoder_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bridge>());
    rclcpp::shutdown();
    return 0;
}




