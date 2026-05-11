//robot_commander.cpp
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
                "/scan", rclcpp::SensorDataQoS(),
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

            RCLCPP_INFO(this->get_logger(), "robot_commander 노드 활성화 ...");


            init_timer = this->create_wall_timer(
                500ms, [this]() -> void {
                    pub_mode();
                    init_timer->cancel();
                }
            );

            // // 제자리에서 바로 제스쳐로 전환
            // sim_timer = this->create_wall_timer(
            //     5s, [this]() -> void {
            //         RCLCPP_INFO(this->get_logger(), "테스트: auto_drive 전환");
            //         set_mode("auto_drive");

            //         // 3초 뒤 gesture 모드로 전환
            //         gesture_timer = this->create_wall_timer(
            //             3s, [this]() -> void {
            //                 RCLCPP_INFO(this->get_logger(), "테스트: gesture 전환");
            //                 set_mode("gesture");
            //                 if (this->gesture_timer) { this->gesture_timer->cancel(); }
            //             }
            //         );

            //         if (this->sim_timer) { this->sim_timer->cancel(); }
            //     }
            // );

            sim_timer = this->create_wall_timer(
                5s, [this]() -> void {
                    RCLCPP_INFO(this->get_logger(), "5초 경과 → auto_drive 모드 시작");

                    auto goal_pose = geometry_msgs::msg::PoseStamped();
                    goal_pose.header.stamp    = this->now();
                    goal_pose.header.frame_id = "map";
                    goal_pose.pose.position.x = 2.46;
                    goal_pose.pose.position.y = -2.62;
                    goal_pose.pose.orientation.w = 1.0;

                    this->call_request_callback(
                        std::make_shared<geometry_msgs::msg::PoseStamped>(goal_pose)
                    );

                    // 한 번만 실행하고 타이머 종료
                    if (this->sim_timer) { this->sim_timer->cancel(); }
                }
            );
        }
    
    private:
        void set_mode (const std::string& new_mode) {
            if (current_mode == new_mode) { return; }
            
            stop_robot();
            current_mode = new_mode;
            pub_mode();

            RCLCPP_INFO(this->get_logger(), "모드 전환 -> %s", current_mode.c_str());
        }

        void pub_mode () {
            std_msgs::msg::String mode_msg;
            mode_msg.data = current_mode;
            mode_pub->publish(mode_msg);
        }

        void stop_robot() {
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            gesture_cmd_pub->publish(stop_cmd);
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
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan = msg; }

        // ㅡㅡㅡㅡ 제스쳐 인식 / 처리 ㅡㅡㅡㅡ
        void gesture_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            if (msg->data.size() < 2) { return; }

            int class_id = msg->data[0];
            int current_x = msg->data[1];

            // class_id : 3 (주먹)으로 모드 전환
            if (class_id == 3) {
                if      (current_mode == "gesture") { set_mode("follow"); }
                else if ( current_mode == "follow") { set_mode("gesture"); }
                last_x = -1;
                return;
            }

            // class_id :0 ~ 2로 동작 명령
            if (current_mode != "gesture") { return; }

            geometry_msgs::msg::Twist cmd;

            // 클래스 동작 (0:손바닥, 1:손등, 2:손가락, 3:주먹)
            switch (class_id) {
                case 0: // 손바닥 ( 정지 / 회전 )
                    if (last_x > 0) {
                        int diff = current_x - last_x;
                        if      (diff < -20) { cmd.angular.z = 0.5; }
                        else if (diff > 20)  { cmd.angular.z = -0.5; }
                        else                 { cmd.linear.x = 0.0; cmd.angular.z = 0.0; }
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

                default:
                    break;
            }
            gesture_cmd_pub->publish(cmd);
        }

        // ㅡㅡㅡㅡ 사람 추종 ㅡㅡㅡㅡ
        void human_offset_callback(const std_msgs::msg::Float64::SharedPtr msg) {
            if (current_mode != "follow") { return; }

            double offset = msg->data;
            // zone0 : -1.0 ~ -0.6 (좌회전)
            if (offset < -0.6) {
                geometry_msgs::msg::Twist align_cmd;
                align_cmd.angular.z = 0.4;
                gesture_cmd_pub->publish(align_cmd);
                return;
            }

            // zone4 : 0.6 ~ 1.0 (우회전)
            if (offset > 0.6) {
                geometry_msgs::msg::Twist align_cmd;
                align_cmd.angular.z = -0.4;
                gesture_cmd_pub->publish(align_cmd);
                return;
            }

            // zone 1~3 : 중앙 (LiDAR)
            double angle_radian = -offset * (camera_rad / 2.0);
            double distance = angle_to_distance(angle_radian);

            if (std::isfinite(distance) && distance < 5.0) {
                geometry_msgs::msg::PoseStamped target;
                target.header.stamp = this->now();
                target.header.frame_id = "base_link";
                target.pose.position.x = distance * std::cos(angle_radian);
                target.pose.position.y = distance * std::sin(angle_radian);

                follow_target_pub->publish(target);
            }
        }
            
        // ㅡㅡㅡㅡ 거리 계산 ㅡㅡㅡㅡ
        double angle_to_distance(double target_angle_radian) {
            if (!latest_scan || latest_scan->ranges.empty()) { return std::numeric_limits<double>::infinity(); }

            int total_ray = static_cast<int>(latest_scan->ranges.size());
            int target_idx = static_cast<int>(std::round(target_angle_radian - latest_scan->angle_min / latest_scan->angle_increment));
            target_idx = std::max(0, std::min(target_idx, total_ray -1));           

            int count = 0;
            double sum = 0.0;
            for (int i= -5; i <= 5; i++){
                int idx = target_idx + i;
                if (idx < 0 || idx >= total_ray) { continue; }

                double range = latest_scan->ranges[idx];
                if (std::isfinite(range) && range > 0.1) {
                    sum += range;
                    count++;
                }
            }
            return (count > 0) ? (sum / count) : std::numeric_limits<double>::infinity();
        }

        std::string current_mode;

        int last_x;
        double camera_rad; // 약 60도

        sensor_msgs::msg::LaserScan::SharedPtr latest_scan;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gesture_cmd_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr auto_drive_goal_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follow_target_pub;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr gesture_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr human_offset_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr call_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav2_status_sub;

        rclcpp::TimerBase::SharedPtr init_timer;
        rclcpp::TimerBase::SharedPtr sim_timer;
        rclcpp::TimerBase::SharedPtr gesture_timer;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_Commander>());
    rclcpp::shutdown();
    return 0;
}