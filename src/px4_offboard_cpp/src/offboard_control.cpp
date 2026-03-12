/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v1.5.0(date_20260312)
# 주요 수정 사항 로그
1. 하트 모양 비행(Heart Shape Flight) 알고리즘 구현
2. 하트 매개변수 방정식(Parametric Heart Equation) 적용
3. 곡선 경로에 따른 실시간 Yaw(머리 방향) 제어 통합
4. 비행 스케일(Scale) 조정을 통한 하트 크기 최적화
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class SmartHeartFlight : public rclcpp::Node {
public:
    SmartHeartFlight() : Node("smart_heart_flight") {
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
            const float scale = 2.5; // 하트 크기 조절 (값이 클수록 큰 하트)

            if (state_ == 0) { // 1단계: 이륙
                target_x = 0.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) state_ = 1;
            } 
            else if (state_ == 1) { // 2단계: 하트 시작점으로 이동 (t=0 지점)
                // t=0일 때 하트 공식의 값: X = scale * 5, Y = 0
                target_x = scale * 5.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) {
                    state_ = 2;
                    RCLCPP_INFO(this->get_logger(), ">> 하트 비행을 시작합니다!");
                }
            } 
            else if (state_ == 2) { // 3단계: 하트 그리기 실행
                theta_ += 0.02; // 부드러운 곡선을 위해 증분값 조정
                
                // 하트 공식 적용 (매개변수 t = theta_)
                float x_val = 13 * std::cos(theta_) - 5 * std::cos(2 * theta_) - 2 * std::cos(3 * theta_) - std::cos(4 * theta_);
                float y_val = 16 * std::pow(std::sin(theta_), 3);

                target_x = scale * x_val;
                target_y = scale * y_val;
                target_z = -10.0;

                // [Yaw 제어] 현재 위치와 다음 목표 사이의 각도를 계산하여 머리 방향을 맞춤
                // 간단하게는 theta_를 활용하여 접선 방향을 추정할 수 있습니다.
                target_yaw = std::atan2(target_y - current_y_, target_x - current_x_);

                if (theta_ >= 2 * M_PI) {
                    state_ = 3;
                    RCLCPP_INFO(this->get_logger(), ">> 하트 비행 완료! 복귀합니다.");
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
        return distance < 2.0;
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z, float yaw) {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {x, y, z};
        msg.yaw = yaw;
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
    rclcpp::spin(std::make_shared<SmartHeartFlight>());
    rclcpp::shutdown();
    return 0;
}