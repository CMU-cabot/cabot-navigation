#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h> 

class WheelieControlNode : public rclcpp::Node
{
public:
    WheelieControlNode()
    : Node("wheelie_control_node"),
      pitch_threshold_(this->declare_parameter("pitch_threshold", -0.15)),  // Pitch angle for Wheelie judgement
      latest_pitch_(0.0)
    {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "cabot/imu/data", 10,
            std::bind(&WheelieControlNode::imuCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cabot/cmd_vel", 10);
        wheelie_state_pub_ = create_publisher<std_msgs::msg::Bool>("wheelie_state", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),  // 100ms check
            std::bind(&WheelieControlNode::checkWheelieState, this));

        RCLCPP_INFO(this->get_logger(), "WheelieControlNode has been started.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        latest_pitch_ = pitch;
    }

    void checkWheelieState()
    {  
        bool new_wheelie_state = latest_pitch_ < pitch_threshold_;

        std_msgs::msg::Bool wheelie_msg;
        wheelie_msg.data = new_wheelie_state;
        wheelie_state_pub_->publish(wheelie_msg);

        RCLCPP_INFO(this->get_logger(), "Wheelie state: %s", new_wheelie_state ? "True" : "False");

        if (new_wheelie_state)
        {
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.linear.y = 0.0;
            stop_msg.linear.z = 0.0;

            cmd_vel_pub_->publish(stop_msg);
            RCLCPP_INFO(this->get_logger(), "Velocity set to 0 due to wheelie state.");
        }
        else
        {
            geometry_msgs::msg::Twist normal_speed_msg;
            normal_speed_msg.linear.x = 1.0;  // *Normal forward speed
            cmd_vel_pub_->publish(normal_speed_msg);
            RCLCPP_INFO(this->get_logger(), "Restoring normal speed with velocity: %f", normal_speed_msg.linear.x);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wheelie_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double pitch_threshold_;
    double latest_pitch_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelieControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
