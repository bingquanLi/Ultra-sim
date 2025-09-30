#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "vehicle_msgs/msg/vehicle_cmd.hpp"

class JoyToVehicleCmd : public rclcpp::Node
{
public:
    JoyToVehicleCmd()
    : Node("joy_to_vehicle_cmd"), gear_(0), last_button4_state_(0)
    {
        // 参数化比例
        this->declare_parameter("speed_scale", 1.0);
        this->declare_parameter("steering_scale", 1.0);
        this->get_parameter("speed_scale", speed_scale_);
        this->get_parameter("steering_scale", steering_scale_);

        // 档位对应倍率，可以通过参数调整
        gear_multipliers_ = {0.3, 1.0, 3.0, 5.0};

        // 发布 VehicleCmd
        pub_ = this->create_publisher<vehicle_msgs::msg::VehicleCmd>("/vehicle_cmd", 10);

        // 订阅 /joy
        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyToVehicleCmd::joy_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "JoyToVehicleCmd node started with 5 gears.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // 按钮切换档位逻辑（上升沿触发）
        int button4 = msg->buttons[4];
        if (button4 == 1 && last_button4_state_ == 0)  // 按下瞬间
        {
            gear_ = (gear_ + 1) % gear_multipliers_.size();  
            RCLCPP_INFO(this->get_logger(), "Gear switched to: %d (x%.1f)", gear_, gear_multipliers_[gear_]);
        }
        last_button4_state_ = button4;

        // 构造 VehicleCmd
        vehicle_msgs::msg::VehicleCmd cmd;
        cmd.speed = msg->axes[1] * speed_scale_ * gear_multipliers_[gear_];
        cmd.steering_angle = msg->axes[3] * steering_scale_;

        pub_->publish(cmd);
    }

    rclcpp::Publisher<vehicle_msgs::msg::VehicleCmd>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

    double speed_scale_;
    double steering_scale_;

    int gear_;               // 当前档位
    int last_button4_state_; // 上一帧按钮状态
    std::vector<double> gear_multipliers_; // 每个档位的速度倍率
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyToVehicleCmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
