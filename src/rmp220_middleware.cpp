#include <memory>
#include <chrono>
#include <cmath>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <segway_msgs/srv/ros_set_chassis_enable_cmd.hpp>

using namespace std::chrono_literals;

enum class State {
    DISABLED,
    ENABLED
};

class rmp220_middleware : public rclcpp::Node {
public:
    rmp220_middleware() : Node("state_machine_node") {
        state = State::DISABLED;
        timeout = 20.0;

        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_mux", 10,
            std::bind(&rmp220_middleware::cmd_vel_callback, this, std::placeholders::_1)
        );
        joy_sub = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&rmp220_middleware::joy_callback, this, std::placeholders::_1)
        );

        timer = create_wall_timer(10ms, std::bind(&rmp220_middleware::timer_callback, this));

        chassis_enable_client = create_client<segway_msgs::srv::RosSetChassisEnableCmd>("set_chassis_enable");
        while (!chassis_enable_client->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "Service not available, waiting for chassis enable service...");
        }
        RCLCPP_INFO(get_logger(), "Chassis enable service available.");
    }

private:
    State state;
    double timeout;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Twist latest_cmd_vel;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Client<segway_msgs::srv::RosSetChassisEnableCmd>::SharedPtr chassis_enable_client;

    void enable_chassis() {
        auto request = std::make_shared<segway_msgs::srv::RosSetChassisEnableCmd::Request>();
        request->ros_set_chassis_enable_cmd = true;
        auto future = chassis_enable_client->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Enabling chassis...");
    }

    void disable_chassis() {
        auto request = std::make_shared<segway_msgs::srv::RosSetChassisEnableCmd::Request>();
        request->ros_set_chassis_enable_cmd = false;
        auto future = chassis_enable_client->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Disabling chassis...");
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->buttons[7] == 1) {  // Joystick button 'start'
            state = State::ENABLED;
            RCLCPP_INFO(get_logger(), "State: ENABLED (Button 'start')");
            enable_chassis();
        }
        if (msg->buttons[6] == 1) {  // Joystick button 'select'
            state = State::DISABLED;
            RCLCPP_INFO(get_logger(), "State: DISABLED (Button 'select')");
            disable_chassis();
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_vel = *msg;
        timeout = 20.0;
    }

    void timer_callback() {
        if (state == State::ENABLED) {
            if (timeout <= 0) {
                state = State::DISABLED;
                RCLCPP_INFO(get_logger(), "State: DISABLED (Timeout)");
                disable_chassis();
            } else {
                timeout -= 0.01;
                cmd_vel_pub->publish(latest_cmd_vel);
            }
        }
        if (state == State::DISABLED &&
            (std::abs(latest_cmd_vel.linear.x) > 0.1 || std::abs(latest_cmd_vel.angular.z) > 0.1)) {
            state = State::ENABLED;
            RCLCPP_INFO(get_logger(), "State: ENABLED (cmd_vel)");
            enable_chassis();
        } else {
            cmd_vel_pub->publish(twist);
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rmp220_middleware>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
