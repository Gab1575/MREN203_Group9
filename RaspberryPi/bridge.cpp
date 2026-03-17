#include <chrono>
#include <memory>
#include <string>

//ros2 core headers
#include "rclcpp/rclcpp.hpp"
//ros2 message types
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

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

        //setup the serial connection to the arduino, will print if error
        try{
            ser_.setPort("/dev/ttyACM0"); //try this usb port, may be different. 
            //can check in pi terminal for correct one
            ser_.setBaudrate(115200);
            auto timeout = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(timeout);
            ser_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port");
        }
        //Create a timer sunning at 50Hz / 20 ms , may need to chage this
        timer_ = this->create_wall_timer(20ms, std::bind(&Bridge::processSerial, this));

        RCLCPP_INFO(this->get_logger(), "Bridge node has been started.");
    }


private: 
    void processSerial() {
        if (ser_.isOpen() && ser_.available()) {
            //read a line of data from the serial port
            std::string raw_line = ser_.readline();

        //Use dataparser class to process the string
        data_parser_.parse(raw_line);

        //Create and publish IMU message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link"; // Set appropriate frame ID

        //Assign parsed data to ros message fields
        imu_msg.angular_velocity.z = data_parser_.gyroZ;
        imu_msg.linear_acceleration.x = data_parser_.accelX;
        imu_msg.linear_acceleration.y = data_parser_.accelY;
        imu_msg.linear_acceleration.z = data_parser_.accelZ;\
        
        imu_pub_->publish(imu_msg);

        //Create and publish left encoder message
        auto l_speed = std_msgs::msg::Float64();
        l_speed.data = data_parser_.omega_L;
        left_encoder_pub_->publish(l_speed);

        //Create and publish right encoder message
        auto r_speed = std_msgs::msg::Float64();
        r_speed.data = data_parser_.omega_R;
        right_encoder_pub_->publish(r_speed);
        }
    }
    //member variables 
    serial::Serial ser_;
    DataParser data_parser_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_encoder_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bridge>());
    rclcpp::shutdown();
    return 0;
}