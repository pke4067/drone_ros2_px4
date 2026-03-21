/*
# 소스코드 이름 : offboard_control.cpp
# 버전 : v2.1.9 (2026-03-21)
# 초보자를 위한 설명:
# 1. 'Lidar'는 드론의 눈입니다. 이 눈이 장애물을 발견하면 숫자를 알려줍니다.
# 2. 'obstacle_confirmed_'는 "한 번이라도 장애물을 봤니?"라고 묻는 기억장치입니다.
# 3. 'target_y = current_y_ + 0.2'는 "현재 위치에서 딱 20cm 앞까지만 가!"라는 뜻입니다.
#    이렇게 하면 드론이 아주 천천히 움직여서 센서가 앞을 똑바로 쳐다봅니다.
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

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
                this->received_first_pos_ = true;
            });

        // 눈(센서)으로부터 데이터를 받는 곳
        obstacle_distance_sub_ = this->create_subscription<px4_msgs::msg::ObstacleDistance>(
            "/fmu/out/obstacle_distance", qos,
            [this](const px4_msgs::msg::ObstacleDistance::SharedPtr msg) {
                float min_valid_dist = 655.35f; 
                bool found = false;
                for (size_t i = 0; i < msg->distances.size(); i++) {
                    if (i <= 15 || i >= 57) { // 시야를 더 넓게(약 80도) 확보
                        float meter_dist = (float)msg->distances[i] / 100.0f;
                        if (meter_dist > 0.1f && meter_dist < 30.0f) {
                            if (meter_dist < min_valid_dist) {
                                min_valid_dist = meter_dist;
                                found = true;
                            }
                        }
                    }
                }
                
                if (found) {
                    this->dist_front_ = min_valid_dist;
                    // 디버깅의 핵심: 센서가 무언가 읽으면 무조건 터미널에 거리를 출력합니다.
                    RCLCPP_INFO(this->get_logger(), "!!! 센서 감지 중: %.2f m", min_valid_dist);
                    
                    if (min_valid_dist < 10.0f) {
                        this->obstacle_confirmed_ = true; // 10m 안쪽이면 기억장치에 저장
                    }
                }
            });

        auto timer_callback = [this]() {
            if (!received_first_pos_) return;

            if (init_counter_ % 10 == 0 && init_counter_ < 100) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            publish_offboard_control_mode();

            float target_yaw = 1.57; // 동쪽(오른쪽) 응시
            float target_x = 0.0, target_y = 0.0, target_z = -10.0;

            if (current_z_ > -5.0f) { 
                target_x = 0.0; target_y = 0.0; target_z = -10.0;
            }
            else {
                // 한 번이라도 장애물을 봤거나 지금 보고 있다면?
                if (obstacle_confirmed_ || dist_front_ < 10.0f) { 
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "!!! 정지 명령 수행 중 (거리: %.2fm)", dist_front_);
                    target_x = current_x_; 
                    target_y = current_y_; // 현재 위치에 말뚝 박기
                    target_z = -10.0;
                } 
                else {
                    // 장애물이 안 보일 때: 아주 천천히 전진하여 기체 수평 유지 (라이다 보호)
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "전진 중... (센서가 정면을 보게 천천히)");
                    target_x = 0.0;
                    target_y = current_y_ + 0.2; // 20cm씩만 앞으로 가라고 명령
                    target_z = -10.0;
                }
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
    float current_x_{0}, current_y_{0}, current_z_{0};
    float dist_front_ = 100.0f;
    bool obstacle_confirmed_ = false; 
    bool received_first_pos_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartSensorFlight>());
    rclcpp::shutdown();
    return 0;
}