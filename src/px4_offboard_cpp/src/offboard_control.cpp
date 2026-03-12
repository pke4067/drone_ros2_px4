/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v1.4.1(date_20260312)
# 주요 수정 사항 로그
1. 원형 비행(Circle Flight) 알고리즘 추가
2. 삼각함수(cos, sin)를 이용한 실시간 좌표 계산 적용
3. NED 좌표계 기반 정밀 위치 제어
4. 원 비행 시작점(Radius, 0)으로의 사전 이동 단계 추가
5. 한 바퀴(2 * PI) 회전 완료 후 자동 복귀 및 착륙 로직 통합
6. [v1.4.1에서 추가됨]반지름 15m -> 50m로 수정
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
        // [핵심] PX4 v1.14 이상 권장 QoS 설정
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // [구독자] v1.3.5에서 성공했던 _v1 토픽 사용
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                this->current_x_ = msg->x;
                this->current_y_ = msg->y;
                this->current_z_ = msg->z;
                this->received_first_pos_ = true;
            });

        auto timer_callback = [this]() {
            // 위치 데이터 수신 전까지 대기
            if (!received_first_pos_) {
                publish_offboard_control_mode();
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, ">>> 드론 위치 신호 대기 중...");
                return;
            }

            // 초기 하트비트 및 오프보드 모드 진입
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

            float target_x = 0.0, target_y = 0.0, target_z = -10.0;
            const float radius = 50.0; // 원의 반지름 (50m)

            // [상태 머신] 원형 비행 로직
            if (state_ == 0) { // 1단계: 이륙 (고도 10m)
                target_x = 0.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) {
                    state_ = 1;
                    RCLCPP_INFO(this->get_logger(), ">> 1단계 완료: 원의 시작점으로 이동합니다.");
                }
            } 
            else if (state_ == 1) { // 2단계: 원의 시작점(North=Radius, East=0)으로 이동
                target_x = radius; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) {
                    state_ = 2;
                    RCLCPP_INFO(this->get_logger(), ">> 2단계 완료: 원형 비행을 시작합니다!");
                }
            } 
            else if (state_ == 2) { // 3단계: 원형 비행 수행 (삼각함수)
                // 0.1초마다 각도를 약 2.8도(0.05 라디안)씩 증가
                theta_ += 0.05; 
                
                // NED 좌표계: X=North, Y=East
                target_x = radius * std::cos(theta_);
                target_y = radius * std::sin(theta_);
                target_z = -10.0;

                // 한 바퀴(2 * PI) 돌았는지 확인
                if (theta_ >= 2 * M_PI) {
                    state_ = 3;
                    RCLCPP_INFO(this->get_logger(), ">> 3단계 완료: 복귀 후 착륙합니다.");
                }
            } 
            else if (state_ == 3) { // 4단계: 홈으로 복귀 및 착륙
                target_x = 0.0; target_y = 0.0; target_z = -10.0;
                if (is_arrived(target_x, target_y, target_z)) {
                    this->land();
                    state_ = 4;
                }
            }

            // 최종 목표 좌표 전송
            if (state_ < 4) {
                publish_trajectory_setpoint(target_x, target_y, target_z);
            }
            
            // 실시간 위치 로그 (1초마다 출력)
            if (init_counter_ % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "상태: %d | 현재: X=%.1f, Y=%.1f, Z=%.1f | 각도: %.2f", 
                            state_, current_x_, current_y_, current_z_, theta_);
            }
            init_counter_++;
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }
    void land() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

private:
    // 유클리드 거리 기반 도착 판단 함수
    bool is_arrived(float tx, float ty, float tz) {
        float distance = std::sqrt(std::pow(tx - current_x_, 2) + std::pow(ty - current_y_, 2) + std::pow(tz - current_z_, 2));
        return distance < 2.0; // 원형 비행 진입을 위해 오차 범위를 2m로 조정
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
    int state_ = 0;
    float current_x_{0}, current_y_{0}, current_z_{0};
    bool received_first_pos_ = false;
    double theta_ = 0.0; // 원형 비행을 위한 각도 변수
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartCircleFlight>());
    rclcpp::shutdown();
    return 0;
}