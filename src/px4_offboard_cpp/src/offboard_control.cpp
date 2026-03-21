/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v2.4.0 (2026-03-21)
# 주요 업데이트: 회피 후 목표 거리 도달 시 자동 착륙 기능 추가
# 목표: 
# 1. 이륙 후 X=60 지점의 기둥 조준 (사용자 v2.3.1 로직 유지)
# 2. 20m 전방 감지 시 10m 회피
# 3. Y축 기준 100m 도달 시 자동 착륙 명령 실행
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// 착륙(LAND) 상태를 추가했습니다!
enum FlightState { TAKEOFF, FORWARD, AVOID, GOAL, LAND };

class SmartSensorFlight : public rclcpp::Node {
public:
    SmartSensorFlight() : Node("smart_sensor_flight") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                this->current_x_ = msg->x; this->current_y_ = msg->y; this->current_z_ = msg->z;
                if (!yaw_locked_) {
                    this->initial_yaw_ = msg->heading;
                    this->yaw_locked_ = true;
                    RCLCPP_INFO(this->get_logger(), "▶ [방향 잠금] 초기 방향 유지 시작");
                }
                this->received_first_pos_ = true;
            });

        obstacle_distance_sub_ = this->create_subscription<px4_msgs::msg::ObstacleDistance>(
            "/fmu/out/obstacle_distance", qos,
            [this](const px4_msgs::msg::ObstacleDistance::SharedPtr msg) {
                float min_val = 655.35f; 
                bool found = false;
                for (auto d : msg->distances) {
                    float dist = (float)d / 100.0f;
                    if (dist > 0.1f && dist < min_val) {
                        min_val = dist;
                        found = true;
                    }
                }
                if (found) { 
                    this->dist_front_ = min_val; 
                    if (min_val < 29.0f && state_ == FORWARD) {
                        RCLCPP_ERROR(this->get_logger(), "!!! [기둥 포착] 거리: %.2f m", min_val);
                    }
                }
            });

        auto timer_callback = [this]() {
            if (!received_first_pos_ || !yaw_locked_) return;

            if (init_counter_ % 10 == 0 && init_counter_ < 100) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            // 착륙 상태가 아닐 때만 Offboard 모드를 유지합니다.
            if (state_ != LAND) {
                publish_offboard_control_mode();
            }

            float target_yaw = initial_yaw_; 
            float target_x = 0.0, target_y = 0.0, target_z = -10.0;

            switch (state_) {
                case TAKEOFF:
                    target_x = 0.0; target_y = 0.0; target_z = -10.0;
                    if (current_z_ <= -9.5f) { state_ = FORWARD; }
                    break;

                case FORWARD:
                    target_x = 0.0; 
                    target_y = current_y_ + 0.6; // 사용자님의 v2.3.1 속도
                    
                    if (dist_front_ < 20.0f) { 
                        state_ = AVOID; 
                        avoid_target_x_ = current_x_ - 10.0; 
                        stop_y_ = current_y_;
                        RCLCPP_WARN(this->get_logger(), ">> 장애물 발견! 20m 전방에서 10m 회피 실시");
                    }
                    break;

                case AVOID:
                    target_x = avoid_target_x_; 
                    target_y = stop_y_; 
                    if (std::abs(current_x_ - avoid_target_x_) < 0.5f) { state_ = GOAL; }
                    break;

                case GOAL:
                    target_x = avoid_target_x_; 
                    target_y = 100.0; // 100m 지점까지 갑니다.
                    
                    // 목표 거리인 100m에 거의 다 왔다면 착륙 상태로 변경!
                    if (current_y_ >= 95.0f) {
                        state_ = LAND;
                        RCLCPP_INFO(this->get_logger(), "▶ [목표 도달] 자동 착륙을 시작합니다.");
                    }
                    break;

                case LAND:
                    // PX4에게 착륙 명령을 한 번 던집니다.
                    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
                    break;
            }

            if (state_ != LAND) {
                publish_trajectory_setpoint(target_x, target_y, target_z, target_yaw);
            }
            init_counter_++;
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }

private:
    // ... 이하 publish 함수들은 v2.3.1과 동일합니다 ...
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
    rclcpp::Subscription<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_distance_sub_;

    uint64_t init_counter_ = 0;
    FlightState state_ = TAKEOFF;
    float current_x_{0}, current_y_{0}, current_z_{0};
    float dist_front_ = 100.0f;
    float initial_yaw_ = 0.0;
    bool yaw_locked_ = false;
    float avoid_target_x_ = 0.0;
    float stop_y_ = 0.0;
    bool received_first_pos_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartSensorFlight>());
    rclcpp::shutdown();
    return 0;
}