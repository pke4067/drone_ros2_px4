/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v2.3.0 (2026-03-21)
# 주요 해결 과제
1. 저속 비행: 초속 약 0.2~0.3m로 천천히 이동하여 드론이 고개를 숙이지 않게 함 (센서 시야 확보).
2. 감지 로직 강화: 모든 각도(0~72개 인덱스)를 다 검사해서 기둥을 놓치지 않게 함.
3. 조기 회피: 기둥이 20m 거리(기존 15m)에 들어오면 즉시 10m 옆으로 회피 시작.
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

enum FlightState { TAKEOFF, FORWARD, AVOID, GOAL };

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
                // 전 영역(0~72개 레이저)을 다 검사해서 기둥을 절대 놓치지 않게 합니다.
                for (auto d : msg->distances) {
                    float dist = (float)d / 100.0f;
                    if (dist > 0.1f && dist < min_val) {
                        min_val = dist;
                        found = true;
                    }
                }
                if (found) { 
                    this->dist_front_ = min_val; 
                    // 기둥이 센서 시야(30m)에 들어오는 순간부터 빨간 로그를 찍습니다.
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

            publish_offboard_control_mode();

            float target_yaw = initial_yaw_; 
            float target_x = 0.0, target_y = 0.0, target_z = -10.0;

            switch (state_) {
                case TAKEOFF:
                    target_x = 0.0; target_y = 0.0; target_z = -10.0;
                    if (current_z_ <= -9.5f) { state_ = FORWARD; }
                    break;

                case FORWARD:
                    // [중요] target_y를 아주 조금씩만 늘려서 드론이 수평을 유지하며 전진하게 합니다.
                    target_x = 0.0; 
                    target_y = current_y_ + 0.2; // 초속 약 2m의 아주 천천히 가는 명령
                    
                    // 기둥이 20m 거리로 들어오면 회피 시작
                    if (dist_front_ < 20.0f) { 
                        state_ = AVOID; 
                        avoid_target_x_ = current_x_ - 10.0; // 오른쪽으로 10m 회피
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
                    target_y = 100.0; 
                    break;
            }

            publish_trajectory_setpoint(target_x, target_y, target_z, target_yaw);
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