/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v2.0.0(date_20260314)
# 주요 수정 사항 로그
1. 장애물 감지(Obstacle Detection) 기능 추가
2. /fmu/out/obstacle_distance 토픽 구독 (라이다 센서 데이터 수신)
3. 긴급 정지(Emergency Braking) 상태 머신 로직 구현
4. 장애물 거리 임계값(3m) 설정 및 경고 시스템 가동
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp> // [추가] 장애물 데이터 메시지 헤더
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class SmartSensorFlight : public rclcpp::Node {
public:
    SmartSensorFlight() : Node("smart_sensor_flight") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        // 퍼블리셔 설정
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // [구독자 1] 위치 정보
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                this->current_x_ = msg->x; this->current_y_ = msg->y; this->current_z_ = msg->z;
                this->received_first_pos_ = true;
            });

        // [구독자 2] 장애물 정보 추가
        obstacle_distance_sub_ = this->create_subscription<px4_msgs::msg::ObstacleDistance>(
            "/fmu/out/obstacle_distance", qos,
            [this](const px4_msgs::msg::ObstacleDistance::SharedPtr msg) {
                // distances[0]은 전방 방향의 거리를 나타냅니다. (단위: cm)
                this->dist_front_ = (float)msg->distances[0] / 100.0f; // m 단위로 변환
            });

        auto timer_callback = [this]() {
            if (!received_first_pos_) return;

            if (init_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            publish_offboard_control_mode();

            float target_x = 0.0, target_y = 0.0, target_z = -10.0;

            // [핵심] 장애물 감지 로직 (5m 이내 감지 시 정지)
            if (dist_front_ > 0.1f && dist_front_ < 5.0f) { 
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "!!! 위험: 전방 장애물 감지 (%.2fm)", dist_front_);
                // 현재 위치에 멈추도록(Hovering) 목표 좌표를 현재 위치로 고정
                target_x = current_x_;
                target_y = current_y_;
                target_z = current_z_;
            } 
            else {
                // 장애물이 없을 때만 기존 비행 로직(예: 전진) 수행
                target_x = 50.0; target_y = 0.0; target_z = -10.0; 
            }

            publish_trajectory_setpoint(target_x, target_y, target_z, 0.0);
            init_counter_++;
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }

private:
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
    rclcpp::Subscription<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_distance_sub_; //

    uint64_t init_counter_ = 0;
    float current_x_{0}, current_y_{0}, current_z_{0};
    float dist_front_ = 100.0f; // 초기 장애물 거리를 충분히 멀게 설정
    bool received_first_pos_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartSensorFlight>());
    rclcpp::shutdown();
    return 0;
}