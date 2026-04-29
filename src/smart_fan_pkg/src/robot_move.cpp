#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class Robot_move : public rclcpp::Node {
    public :
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        Robot_move() : Node("robot_move_node") {
            current_mode = "waiting";

            // ㅡㅡㅡㅡ publisher ㅡㅡㅡㅡ
            cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            nav_status_pub = this->create_publisher<std_msgs::msg::String>("nav_status", 10);

            // ㅡㅡㅡㅡ subsciption ㅡㅡㅡㅡ
            mode_sub = this->create_subscription<std_msgs::msg::String>(
                "current_mode", 10, std::bind(&Robot_move::mode_callback, this, std::placeholders::_1)
            );
            gesture_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "gesture_cmd", 10, std::bind(&Robot_move::gesture_cmd_callback, this, std::placeholders::_1)
            );
            auto_drive_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "auto_drive_goal", rclcpp::SensorDataQoS(), std::bind(&Robot_move::auto_drive_callback, this, std::placeholders::_1)
            );
            follow_target_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "follow_target_pose", 10, std::bind(&Robot_move::follow_target_callback, this, std::placeholders::_1)
            );

            // ㅡㅡㅡㅡ 알고리즘 센서 ㅡㅡㅡㅡ
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, std::bind(&Robot_move::odom_callback, this, std::placeholders::_1)
            );
            costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "global_costmap/costmap", 10, std::bind(&Robot_move::costmap_callback, this, std::placeholders::_1)
            );
            scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&Robot_move::scan_callback, this, std::placeholders::_1)
            );

            // ㅡㅡㅡㅡ 타이머 ㅡㅡㅡㅡ
            timer = this->create_wall_timer(
                100ms, std::bind(&Robot_move::control_loop, this)
            );

            nav2_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
            RCLCPP_INFO(this->get_logger(), "robot activated");
        }

    private:
        // ㅡㅡㅡㅡㅡ 센서 ㅡㅡㅡㅡ
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { current_odom = msg; }
        void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { current_costmap = msg; }
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan = msg; }

        // ㅡㅡㅡㅡ 기본 모드 설정 ㅡㅡㅡㅡ
        void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
            current_mode = msg->data;
            if (current_mode == "waiting") {
                stop_robot();
            }
        }

        void gesture_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            if (current_mode == "gesture") {
                cmd_vel_pub->publish(*msg);
            }
        }
        
        void auto_drive_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { target_auto_goal = msg; }
        void follow_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { target_follow_pose = msg; }

        void control_loop() {
            if (!current_odom) { return; }

            if (current_mode == "auto_drive") {
                // 현재 위치, 지도 정보 확인
                if (!current_odom || !current_costmap || !target_auto_goal) { return; }

                // 20cm 이하로 가까워지면 도착 판정
                double dx = target_auto_goal->pose.position.x - current_odom->pose.position.x;
                double dy = target_auto_goal->pose.position.y - current_odom->pose.position.y;
                if (std::hypot(dx, dy) < 0.2) {
                    current_mode = "waiting";
                    stop_robot();

                    std_msgs::msg::String status_msg;
                    status_msg.data = "arrived";
                    nav_status_pub->publish(status_msg);

                    return;
                }

                std::vector<geometry_msgs::msg::Pose> global_path = theta_planner(current_odom->pose.pose, target_auto_goal->pose, current_costmap);
                if (!global_path.empty()) {
                    geometry_msgs::msg::Twist cmd_vel;
                    if (teb_planner(current_odom->pose.pose, global_path, current_costmap, cmd_vel)) {
                        cmd_vel_pub->publish(cmd_vel);
                    }
                }
            } else if (current_mode == "follow") {
                // 현재 위치, 지도 정보 확인
                if (!current_odom || !latest_scan || !target_follow_pose) { return; }

                // 추종 대상과 40cm 이하로 가까워지면 안전 거리 유지
                double target_x = target_follow_pose->pose.position.x;
                double target_y = target_follow_pose->pose.position.y;

                if (std::hypot(target_x, target_y) < 0.6) {
                    stop_robot();
                    return;
                }

                // 로봇 위치, yaw 각도 구하기 ( odom 기준 )
                double robot_x = current_odom->pose.pose.position.x;
                double robot_y = current_odom->pose.pose.position.y;

                // 로봇이 바라보는 각도에서 yaw 추출
                double siny_cosp = 2.0 * (current_odom->pose.pose.orientation.w * current_odom->pose.pose.orientation.z);
                double cosy_cosp = 1.0 - 2.0 * (current_odom->pose.pose.orientation.z * current_odom->pose.pose.orientation.z);
                double robot_yaw = std::atan2(siny_cosp, cosy_cosp);

                // 상대 좌표 > 절대 좌표로 전환
                double abs_target_x = robot_x + (target_x * std::cos(robot_yaw) - target_y * std::sin(robot_yaw));
                double abs_target_y = robot_y + (target_x * std::sin(robot_yaw) + target_y * std::cos(robot_yaw));

                geometry_msgs::msg::Pose absolute_target_pose;
                absolute_target_pose.position.x = abs_target_x;
                absolute_target_pose.position.y = abs_target_y;
                absolute_target_pose.orientation = current_odom->pose.pose.orientation;

                std::vector<geometry_msgs::msg::Pose> local_path = { target_follow_pose->pose };
                geometry_msgs::msg::Twist cmd_vel;

                if (teb_planner(current_odom->pose.pose, local_path, current_costmap, cmd_vel)) {
                    cmd_vel_pub->publish(cmd_vel);
                }
            }
        }

        void stop_robot() {
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub->publish(stop_cmd);
        }

        // ㅡㅡㅡㅡ 자율주행 알고리즘 ㅡㅡㅡㅡ
        // Theta*
        std::vector<geometry_msgs::msg::Pose> theta_planner(
            const geometry_msgs::msg::Pose& start,
            const geometry_msgs::msg::Pose& goal,
            const nav_msgs::msg::OccupancyGrid::SharedPtr costmap
        ) {
            std::vector<geometry_msgs::msg::Pose> path;
            
            // 전방 직선 확인
            bool has_LoS = true;

            // 장애물이 없으면 직진
            if (has_LoS) {
                path.push_back(start);

                // 경로를 5등분해서 웨이 포인트 지정
                for (int i=1; i<5; i++) {
                    geometry_msgs::msg::Pose way_point;
                    way_point.position.x = start.position.x + (goal.position.x - start.position.x) * (i / 5.0);
                    way_point.position.y = start.position.y + (goal.position.y - start.position.y) * (i / 5.0);
                    path.push_back(way_point);
                }
                path.push_back(goal);
            } else { // 장애물 있으면 현재 위치와 목적지만 TEB에 전달
                path.push_back(start);
                path.push_back(goal);
            }

            return path;
        }

        // TEB
        bool teb_planner (
            const geometry_msgs::msg::Pose& current_pose,
            const std::vector<geometry_msgs::msg::Pose>& plan,
            const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
            geometry_msgs::msg::Twist& cmd_vel_out
        ) {
            if (plan.empty()) { return false; }

            // 가까운 목표점 지정
            geometry_msgs::msg::Pose target_way_point = plan.front();
            if (plan.size() > 1) target_way_point = plan[1];

            // 목표 지점에 인력 작용
            double dx = target_way_point.position.x - current_pose.position.x;
            double dy = target_way_point.position.y - current_pose.position.y;
            double dist_to_target = std::hypot(dx, dy);

            // 로봇의 yaw 계산
            double siny_cosp = 2 * (current_pose.orientation.w * current_pose.orientation.z);
            double cosy_cosp = 1 - 2 * (current_pose.orientation.z * current_pose.orientation.z);
            double current_yaw = std::atan2(siny_cosp, cosy_cosp);

            // 목표 지점 yaw 계산
            double target_yaw = std::atan2(dy, dx);
            double heading_diff = target_yaw - current_yaw;

            // 각도 차이 정규화
            while(heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
            while(heading_diff < -M_PI) heading_diff += 2.0 * M_PI;

            // 거리, 각도 차이 비례 속도 제어
            double linear_v = 0.5 * dist_to_target;
            double angular_w = 1.0 * heading_diff;

            // 장애물에 척력 작용
            double avoid_angular_w = 0.0;
            bool obstacle = false;

            if (latest_scan) {
                int center_index = latest_scan->ranges.size() / 2; // 180도
                int scan_range = latest_scan->ranges.size() / 12;  // 30도

                double left_space = 0.0;
                double right_space = 0.0;

                // ??
                for (int i=center_index - scan_range; i<=center_index + scan_range; i++) {
                    if (i < 0 || i >= (int)latest_scan->ranges.size()) { continue; }

                    double range = latest_scan->ranges[i];
                    if (std::isinf(range)) range = latest_scan->range_max;

                    // 전방에 장애물 감지되면 회피 기동
                    if (range < 0.5) { obstacle = true; }

                    if (i < center_index) {
                        right_space += range;
                    } else {
                        left_space += range;
                    }
                }

                // 좌우 중 더 넓은 공간으로 회전
                if (obstacle) {
                    if (left_space > right_space) {
                        avoid_angular_w = 0.8;
                    } else {
                        avoid_angular_w = -0.8;
                    }
                }
            }

            if (obstacle) {
                linear_v *= 0.2;
                angular_w = avoid_angular_w;
            }

            // 터틀봇3 와플파이 하드웨어 스펙 제한
            if (linear_v > 0.26) { linear_v = 0.26; }
            if (angular_w > 1.8) { angular_w = 1.8; }
            if (angular_w < -1.8) { angular_w = -1.8; }

            cmd_vel_out.linear.x = linear_v;
            cmd_vel_out.angular.z = angular_w;

            return true;
        }

        // 호출 지점 출발
        void send_goal(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
            if (!nav2_client->wait_for_action_server(2s)) {
                return;
            }
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose = *pose;

            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&Robot_move::arrive_callback, this, std::placeholders::_1);

            nav2_client->async_send_goal(goal_msg, send_goal_options);
        }
        
        // 호출 지점 도착 여부 확인
        void arrive_callback(const GoalHandleNav::WrappedResult & result) {
            std_msgs::msg::String status_msg;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                status_msg.data = "arrived";
            } else {
                status_msg.data = "failed";
            }
            nav_status_pub->publish(status_msg);
        }


        std::string current_mode;
        sensor_msgs::msg::LaserScan::SharedPtr latest_scan;
        nav_msgs::msg::Odometry::SharedPtr current_odom;
        nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap;
        geometry_msgs::msg::PoseStamped::SharedPtr target_auto_goal;
        geometry_msgs::msg::PoseStamped::SharedPtr target_follow_pose;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_status_pub;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gesture_cmd_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr auto_drive_goal_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr follow_target_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_move>());
    rclcpp::shutdown();
    return 0;
}