/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v1.4.2(date_20260312)
# 주요 수정 사항 로그
1. 반지름 50m에 최적화된 각속도 적용 (속도 약 5m/s 유지)
2. Yaw(머리 방향) 제어 로직 추가: 드론이 원의 접선 방향을 바라보며 비행
3. publish_trajectory_setpoint 함수에 Yaw 인자 통합
4. 도착 판단 범위(is_arrived) 최적화
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class SmartCircleFlight : public rclcpp::Node {
public:
    SmartCircleFlight() : Node("smart_circle_flight") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                this->current_x_ = msg->x;
                this->current_y_ = msg->y;
                this->current_z_ = msg->z;
                this->received_first_pos_ = true;
            });

        auto timer_callback = [this]() {
            if (!received_first_pos_) {
                publish_offboard_control_mode();
                return;
            }

            if (init_counter_ < 10) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint(0.0, 0.0, -10.0, 0.0);
                init_counter_++;
                return;
            }

            if (init_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
                init_counter_++;
            }

            publish_offboard_control_mode();

            float target_x = 0.0, target_y = 0.0, target_z = -10.0, target_yaw = 0.0;
            const float radius = 50.0; // 반지름 50m

            if (state_ == 0) { // 1단계: 이륙
                target_x = 0.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) state_ = 1;
            } 
            else if (state_ == 1) { // 2단계: 시작점 이동
                target_x = radius; target_y = 0.0; target_z = -10.0;
                target_yaw = M_PI / 2.0; // 동쪽(East)을 바라보며 대기
                if (is_arrived(target_x, target_y, target_z)) state_ = 2;
            } 
            else if (state_ == 2) { // 3단계: 원형 비행 (Yaw 제어 포함)
                // 속도 5m/s 유지를 위해 0.01 증분 사용
                theta_ += 0.01; 
                
                target_x = radius * std::cos(theta_);
                target_y = radius * std::sin(theta_);
                target_z = -10.0;
                
                // 드론의 머리를 현재 각도(theta_)에서 90도(PI/2) 더한 방향으로 고정
                target_yaw = theta_ + M_PI / 2.0; 

                if (theta_ >= 2 * M_PI) {
                    state_ = 3;
                    RCLCPP_INFO(this->get_logger(), ">> 원형 비행 완료!");
                }
            } 
            else if (state_ == 3) { // 4단계: 복귀 및 착륙
                target_x = 0.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) {
                    this->land();
                    state_ = 4;
                }
            }

            if (state_ < 4) publish_trajectory_setpoint(target_x, target_y, target_z, target_yaw);
            init_counter_++;
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }
    void land() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

private:
    bool is_arrived(float tx, float ty, float tz) {
        float distance = std::sqrt(std::pow(tx - current_x_, 2) + std::pow(ty - current_y_, 2) + std::pow(tz - current_z_, 2));
        return distance < 3.0; // 50m 원형 비행을 위해 도착 범위를 3m로 여유있게 설정
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    // [수정] Yaw 값이 포함된 궤적 명령 전송
    void publish_trajectory_setpoint(float x, float y, float z, float yaw) {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {x, y, z};
        msg.yaw = yaw; // 라디안 단위 방향 제어
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
    int state_ = 0;
    float current_x_{0}, current_y_{0}, current_z_{0};
    bool received_first_pos_ = false;
    double theta_ = 0.0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartCircleFlight>());
    rclcpp::shutdown();
    return 0;
}