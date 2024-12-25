#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/bool.hpp>  

class WheelieControlNode : public rclcpp::Node
{
public:
    WheelieControlNode()
    : Node("wheelie_control_node"), pitch_threshold_(-0.2), is_wheelie_(false)
    {
        declare_parameter("pitch_threshold", pitch_threshold_);
        pitch_threshold_ = get_parameter("pitch_threshold").as_double();

        // Subscribe IMU data
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "cabot/imu/data", 10,
            std::bind(&WheelieControlNode::imuCallback, this, std::placeholders::_1));

        wheelie_state_pub_ = create_publisher<std_msgs::msg::Bool>("wheelie_state", 10);

        RCLCPP_INFO(this->get_logger(), "WheelieControlNode has been started.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Calculate pitch angle from IMU `orientation`
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        pitch_ = pitch;  
        RCLCPP_DEBUG(this->get_logger(), "Current pitch: %f", pitch_);

        // If the pitch angle is less than the threshold, it is determined that the vehicle is in a wheelie state.
        bool new_wheelie_state = pitch_ < pitch_threshold_;
        if (new_wheelie_state != is_wheelie_)
        {
            is_wheelie_ = new_wheelie_state;

            std_msgs::msg::Bool wheelie_msg;
            wheelie_msg.data = is_wheelie_;
            wheelie_state_pub_->publish(wheelie_msg);  
            RCLCPP_INFO(this->get_logger(), "Wheelie state: %s", is_wheelie_ ? "True" : "False");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wheelie_state_pub_;  

    double pitch_threshold_;  
    double pitch_;            
    bool is_wheelie_;         
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelieControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}