/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v1.2(date_20260302)
# 주요 수정 사항
1. 비행 시간 변경(이동 시간 5초 -> 20~25초)
2. 착륙 기능 추가

*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SquareFlightAndLand : public rclcpp::Node {
public:
    SquareFlightAndLand() : Node("square_flight_land") {
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        auto timer_callback = [this]() {
            // 1. 초기화 (1초 지점)
            if (counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            // 오프보드 유지를 위해 제어 모드는 계속 발행
            publish_offboard_control_mode();
            
            // 2. 비행 스케줄러
            if (counter_ < 150) {          // 0~15초: 제자리 이륙 (고도 10m)
                publish_trajectory_setpoint(0.0, 0.0, -10.0);
            } else if (counter_ < 400) {   // 15~40초: 앞으로 100m 이동
                publish_trajectory_setpoint(100.0, 0.0, -10.0);
            } else if (counter_ < 600) {   // 40~60초: 오른쪽으로 100m 이동
                publish_trajectory_setpoint(100.0, 100.0, -10.0);
            } else if (counter_ < 800) {   // 60~80초: 뒤로 100m 이동
                publish_trajectory_setpoint(0.0, 100.0, -10.0);
            } else if (counter_ < 950) {   // 80~95초: 출발지로 복귀 및 5초간 대기(안정화)
                publish_trajectory_setpoint(0.0, 0.0, -10.0);
            } else if (counter_ == 950) {  // 95초 정각: 착륙 명령 실행!
                this->land();
                RCLCPP_INFO(this->get_logger(), "착륙 명령을 전송했습니다.");
            }

            // 950(95초) 이후에는 좌표 명령을 중단하여 PX4가 착륙에 집중하게 함
            if (counter_ < 950) {
                counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }
    
    // [추가] 착륙 명령 함수
    void land() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

private:
    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z) {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {x, y, z};
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
    rclcpp::spin(std::make_shared<SquareFlightAndLand>());
    rclcpp::shutdown();
    return 0;
}