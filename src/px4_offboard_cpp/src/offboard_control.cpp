/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v1.3(date_20260302)
# 주요 수정 사항 로그
1. 비행 시간 변경(이동 시간 5초 -> 20~25초)
2. 착륙 기능 추가(VEHICLE_CMD_NAV_LAND 함수)
3. publisher-subscriber 기능 구현으로 현재 위치 수신 가능(유클리드 알고리즘 적용)
4. 유클리드 거리 공식 알고리즘 추가(시간 기반 -> 거리 기반)

*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp> // 현재 위치 수신을 위해 추가
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath> // 수학 계산(제곱근 등)을 위해 추가

using namespace std::chrono_literals;

class SmartSquareFlight : public rclcpp::Node {
public:
    SmartSquareFlight() : Node("smart_square_flight") {
        // [발행자] 명령 내리는 통로
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // [구독자] 드론의 현재 위치를 실시간으로 받아오는 통로 (추가됨)
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 10,
            std::bind(&SmartSquareFlight::position_callback, this, std::placeholders::_1));

        auto timer_callback = [this]() {
            if (init_counter_ < 10) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint(0.0, 0.0, -10.0);
                init_counter_++;
                return;
            }

            if (init_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
                init_counter_++;
            }

            publish_offboard_control_mode();

            // --- 거리 기반 상태 전환 로직 ---
            float target_x = 0.0, target_y = 0.0, target_z = -10.0;

            if (state_ == 0) { // 이륙 상태
                target_x = 0.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) state_ = 1;
            } else if (state_ == 1) { // 앞으로 100m
                target_x = 100.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) state_ = 2;
            } else if (state_ == 2) { // 오른쪽 100m
                target_x = 100.0; target_y = 100.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) state_ = 3;
            } else if (state_ == 3) { // 뒤로 100m
                target_x = 0.0; target_y = 100.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) state_ = 4;
            } else if (state_ == 4) { // 복귀
                target_x = 0.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) {
                    this->land();
                    RCLCPP_INFO(this->get_logger(), "목적지 도착! 착륙합니다.");
                    state_ = 5; // 착륙 중 상태 (더 이상 좌표 안 보냄)
                }
            }

            // 착륙 상태가 아닐 때만 좌표 발행
            if (state_ < 5) {
                publish_trajectory_setpoint(target_x, target_y, target_z);
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }
    void land() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

private:
    // 드론 위치 수신 시 실행되는 함수
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
    }

    // 도착 여부 확인 함수 (유클리드 거리 공식)
    bool is_arrived(float tx, float ty, float tz) {
        float distance = std::sqrt(std::pow(tx - current_x_, 2) + 
                                  std::pow(ty - current_y_, 2) + 
                                  std::pow(tz - current_z_, 2));
        return distance < 2.0; // 오차 범위 2m 이내면 도착으로 인정
    }

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
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

    uint64_t init_counter_ = 0;
    int state_ = 0; // 현재 비행 단계
    float current_x_{0}, current_y_{0}, current_z_{0};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartSquareFlight>());
    rclcpp::shutdown();
    return 0;
}