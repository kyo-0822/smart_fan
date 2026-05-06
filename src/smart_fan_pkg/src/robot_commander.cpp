#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Robot_Commander : public rclcpp::Node {
    public:
        Robot_Commander() : Node("robot_commander_node") {
            current_mode = "waiting";

            // publisher
            mode_pub = this->create_publisher<std_msgs::msg::String>("current_mode", 10);
            gesture_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("gesture_cmd", 10);
            auto_drive_goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("auto_drive_goal", 10);
            follow_target_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("follow_target_pose", 10);

            // subscription
            scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", rclcpp::SensorDataQoS(),
                std::bind(&Robot_Commander::scan_callback, this, std::placeholders::_1)
            );
            gesture_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
                "gesture_data", 10,
                std::bind(&Robot_Commander::gesture_callback, this, std::placeholders::_1)
            );
            human_offset_sub = this->create_subscription<std_msgs::msg::Float64>(
                "human_offset", 10,
                std::bind(&Robot_Commander::human_offset_callback, this, std::placeholders::_1)
            );
            call_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "call_position", 10,
                std::bind(&Robot_Commander::call_request_callback, this, std::placeholders::_1)
            );
            nav2_status_sub = this->create_subscription<std_msgs::msg::String>(
                "nav2_status", 10,
                std::bind(&Robot_Commander::nav2_status_callback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "robot_commander activated ...");
        }
    
    private:
        void set_mode (const std::string& new_mode) {
            current_mode = new_mode;
            std_msgs::msg::String mode_msg;
            mode_msg.data = current_mode;
            mode_pub->publish(mode_msg);
            RCLCPP_INFO(this->get_logger(), "Current mode : %s", current_mode.c_str());
        }

        // ㅡㅡㅡㅡ 로봇 호출 처리 ㅡㅡㅡㅡ
        void call_request_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            set_mode("auto_drive");
            auto_drive_goal_pub->publish(*msg);
        }

        void nav2_status_callback(const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "arrived" && current_mode == "auto_drive") { set_mode("gesture"); }
        }

        // ㅡㅡㅡㅡ LiDAR ㅡㅡㅡㅡ
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            latest_scan = msg;
        }

        // ㅡㅡㅡㅡ 제스쳐 인식 / 처리 ㅡㅡㅡㅡ
        void gesture_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            if (msg->data.size() < 2) { return; }

            int class_id = msg->data[0];
            int current_x = msg->data[1];

            // class_id : 3 (주먹)으로 모드 전환
            if (class_id == 3) {
                if (current_mode == "gesture") {
                    set_mode("follow");
                } else if ( current_mode == "follow") {
                    set_mode("gesture");
                }
                last_x = -1;
                return;
            }

            // class_id :0 ~ 2로 동작 명령
            if (current_mode == "gesture") {
                geometry_msgs::msg::Twist cmd;

                // 클래스 동작 (0:손바닥, 1:손등, 2:손가락, 3:주먹)
                switch (class_id) {
                    case 0: // 손바닥 ( 정지 / 회전 )
                        if (last_x > 0) {
                            int diff = current_x - last_x;
                            if (diff < -20) {
                                cmd.angular.z = 0.5;
                            } else if (diff > 20) {
                                cmd.angular.z = -0.5;
                            } else { 
                                cmd.linear.x = 0.0;
                                cmd.angular.z = 0.0; 
                            }
                        }
                        last_x = current_x;
                        break;

                    case 1: // 손등 (전진)
                        cmd.linear.x = 0.2;  
                        last_x = -1;
                        break;

                    case 2: // 손가락 (후진)
                        cmd.linear.x = -0.2;
                        last_x = -1;
                        break;

                    case 3: // 주먹 (모드 전환)
                        // 위에서 미리 처리
                        break;

                    default:
                        break;
                }
                gesture_cmd_pub->publish(cmd);
            }
        }

        // ㅡㅡㅡㅡ 사람 추종 ㅡㅡㅡㅡ
        void human_offset_callback(const std_msgs::msg::Float64::SharedPtr msg) {
            if (current_mode != "follow") { return; }

            double offset = msg->data;
            double angle_radian = -offset * (camera_rad / 2.0);
            double target_distance = angle_to_distance(angle_radian);

            if (std::isfinite(target_distance) && target_distance < 5.0) {
                double target_x = target_distance * std::cos(angle_radian);
                double target_y = target_distance * std::sin(angle_radian);

                geometry_msgs::msg::PoseStamped target_pose;
                target_pose.header.stamp = this->now();
                target_pose.header.frame_id = "base_link";
                target_pose.pose.position.x = target_x;
                target_pose.pose.position.y = target_y;

                follow_target_pub->publish(target_pose);
            }
        }
        
        // ㅡㅡㅡㅡ 거리 계산 ㅡㅡㅡㅡ
        double angle_to_distance(double target_angle_radian) {
            if (!latest_scan || latest_scan->ranges.empty()) { return std::numeric_limits<double>::infinity(); }
            double angle_min = latest_scan->angle_min;
            double angle_increment = latest_scan->angle_increment;
            int total_ray = latest_scan->ranges.size();
            int target_idx = std::round((target_angle_radian - angle_min) / angle_increment);

            if (target_idx < 0 || target_idx >= total_ray) { return std::numeric_limits<double>::infinity(); }

            int count = 0;
            double sum = 0.0;
            for (int i=-5; i<=5; i++){
                int idx = target_idx + i;
                if (idx >= 0 && idx < total_ray) {
                    double range = latest_scan->ranges[idx];
                    if (std::isfinite(range) && range > 0.1) {
                        sum += range;
                        count++;
                    }
                }
            }
            return (count > 0) ? (sum / count) : std::numeric_limits<double>::infinity();
        }

        int last_x = -1;
        double camera_rad = 1.047; // 약 60도

        std::string current_mode;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gesture_cmd_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr auto_drive_goal_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follow_target_pub;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr gesture_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr human_offset_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav2_status_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr call_sub;

        sensor_msgs::msg::LaserScan::SharedPtr latest_scan;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_Commander>());
    rclcpp::shutdown();
    return 0;
}