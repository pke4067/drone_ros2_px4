#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SquareFlight : public rclcpp::Node {
public:
    SquareFlight() : Node("square_flight") {
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        auto timer_callback = [this]() {
            // 1. 처음 1초(10번) 동안은 '준비' 신호만 보냄
            if (counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // 오프보드 모드
                this->arm(); // 시동
            }

            publish_offboard_control_mode();
            
            // 2. 시간에 따라 목적지(Setpoint) 바꾸기 (5초마다 변경)
            // 10Hz이므로 counter 50마다 5초가 지남
            if (counter_ < 150) { // 0~15초: 제자리 이륙 (고도 10m)
                publish_trajectory_setpoint(0.0, 0.0, -10.0);
            } else if (counter_ < 400) { // 15~40초: 앞으로 100m 이동
                publish_trajectory_setpoint(100.0, 0.0, -10.0);
            } else if (counter_ < 600) { // 40~60초: 오른쪽으로 100m 이동
                publish_trajectory_setpoint(100.0, 100.0, -10.0);
            } else if (counter_ < 800) { // 60~80초: 뒤로 100m 이동
                publish_trajectory_setpoint(0.0, 100.0, -10.0);
            } else { // 80초 이후: 처음 위치로 복귀
                publish_trajectory_setpoint(0.0, 0.0, -10.0);
            }

            counter_++;
        };
        timer_ = this->create_wall_timer(100ms, timer_callback); // 0.1초마다 실행
    }

    void arm() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }

private:
    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z) {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {x, y, z}; // NED 좌표계 사용
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    uint64_t counter_ = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareFlight>());
    rclcpp::shutdown();
    return 0;
}