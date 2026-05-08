//robot_move.cpp
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <queue>
#include <unordered_map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

struct GridNode {
    int x, y;
    int parent_x, parent_y;
    double g_cost;
    double f_cost;

    bool operator>(const GridNode& other) const { return f_cost > other.f_cost; }
};

class Robot_move : public rclcpp::Node {
    public :
        Robot_move() : Node("robot_move_node") {
            current_mode = "waiting";
            replanning = false;
            costmap_update = false;
            prev_linear_v = 0.0;
            prev_angular_w = 0.0;

            // ㅡㅡㅡㅡ publisher ㅡㅡㅡㅡ
            cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            nav2_status_pub = this->create_publisher<std_msgs::msg::String>("nav2_status", 10);

            // ㅡㅡㅡㅡ subscription ㅡㅡㅡㅡ
            mode_sub = this->create_subscription<std_msgs::msg::String>(
                "current_mode", 10, std::bind(&Robot_move::mode_callback, this, std::placeholders::_1)
            );
            gesture_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "gesture_cmd", 10, std::bind(&Robot_move::gesture_cmd_callback, this, std::placeholders::_1)
            );
            auto_drive_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "auto_drive_goal", 10, std::bind(&Robot_move::auto_drive_callback, this, std::placeholders::_1)
            );
            follow_target_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "follow_target_pose", 10, std::bind(&Robot_move::follow_target_callback, this, std::placeholders::_1)
            );

            // ㅡㅡㅡㅡ 알고리즘 센서 ㅡㅡㅡㅡ
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                std::bind(&Robot_move::odom_callback, this, std::placeholders::_1)
            );
            costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                std::bind(&Robot_move::costmap_callback, this, std::placeholders::_1)
            );
            scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", rclcpp::SensorDataQoS(),
                std::bind(&Robot_move::scan_callback, this, std::placeholders::_1)
            );

            timer = this->create_wall_timer(100ms, std::bind(&Robot_move::control_loop, this));

            RCLCPP_INFO(this->get_logger(), "robot_move 노드 활성화 ...");
        }

    private:
        // ㅡㅡㅡㅡㅡ 센서 ㅡㅡㅡㅡ
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { current_odom = msg; }
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan = msg; }
        void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            if (!current_costmap) {
                current_costmap = msg;
                costmap_update = true;
            }
        }
        
        // ㅡㅡㅡㅡ 기본 모드 설정 ㅡㅡㅡㅡ
        void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
            current_mode = msg->data;
            if (current_mode == "waiting") { stop_robot(); }
        }

        void gesture_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            if (current_mode == "gesture") {
                last_gesture_time = this->now();
                cmd_vel_pub->publish(*msg);

                prev_linear_v = msg->linear.x;
                prev_angular_w = msg->angular.z;
            }
        }
        
        void auto_drive_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            target_auto_goal = msg;
            replanning = true;
        }
        void follow_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { target_follow_pose = msg; }

        void control_loop() {
            if (!current_odom) { return; }
            if (current_mode == "gesture") { return; }
            if (current_mode == "waiting") { return; }

            if (current_mode == "auto_drive") {
                // 현재 위치, 지도 정보 확인
                if (!current_costmap || !target_auto_goal) { return; }

                // 20cm 이하로 가까워지면 도착 판정
                double dx = target_auto_goal->pose.position.x - current_odom->pose.pose.position.x;
                double dy = target_auto_goal->pose.position.y - current_odom->pose.pose.position.y;

                if (std::hypot(dx, dy) < 0.2) {
                    stop_robot();
                    std_msgs::msg::String status_msg;
                    status_msg.data = "arrived";
                    nav2_status_pub->publish(status_msg);
                    return;
                }
                
                // theta*로 전역 경로 생성
                if (replanning || costmap_update) {
                    current_global_path = theta_planner(
                        current_odom->pose.pose,
                        target_auto_goal->pose,
                        current_costmap
                    );
                    replanning = false;
                    costmap_update = false;
                }
                
                if (!current_global_path.empty()) {
                    geometry_msgs::msg::Twist cmd_vel;
                    if (teb_planner(
                        current_odom->pose.pose,
                        current_global_path,
                        cmd_vel)
                    ) { cmd_vel_pub->publish(cmd_vel); }
                } else {
                    // 경로 없으면 정지
                    stop_robot();
                }
            } else if (current_mode == "follow") {
                // 현재 위치, 지도 정보 확인
                if (!latest_scan || !target_follow_pose) { return; }

                // 추종 대상과 40cm 이하로 가까워지면 안전 거리 유지
                double target_x = target_follow_pose->pose.position.x;
                double target_y = target_follow_pose->pose.position.y;
                if (std::hypot(target_x, target_y) < 0.4) {
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

                std::vector<geometry_msgs::msg::Pose> local_path = { absolute_target_pose };
                geometry_msgs::msg::Twist cmd_vel;

                if (teb_planner(current_odom->pose.pose, local_path, cmd_vel)) {
                    cmd_vel_pub->publish(cmd_vel);
                }
            }
        }

        void stop_robot() {
            prev_linear_v = 0.0;
            prev_angular_w = 0.0;
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub->publish(stop_cmd);
        }

        // ㅡㅡㅡㅡ 좌표 변환 ㅡㅡㅡㅡ
        bool world_to_map(double wx, double wy, int& mx, int& my,
                          const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
            double origin_x = costmap->info.origin.position.x;
            double origin_y = costmap->info.origin.position.y;
            double res = costmap->info.resolution;

            mx = static_cast<int>((wx - origin_x) / res);
            my = static_cast<int>((wy - origin_y) / res);

            return (mx >= 0 && mx < (int)costmap->info.width &&
                    my >= 0 && my < (int)costmap->info.height);
        }

        void map_to_world(int mx, int my, double& wx, double& wy,
                          const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {

            wx = costmap->info.origin.position.x + (mx + 0.5) * costmap->info.resolution;
            wy = costmap->info.origin.position.y + (my + 0.5) * costmap->info.resolution;
        }

        // ㅡㅡㅡㅡ Bresenham LoS( 시야선 ) 체크 ㅡㅡㅡㅡ
        bool check_LoS (int x0, int y0, int x1, int y1,
                        const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
            int dx = std::abs(x1 - x0);
            int dy = std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx - dy;

            while (true) {
                if (x0 < 0 || x0 >= (int)costmap->info.width ||
                    y0 < 0 || y0 >= (int)costmap->info.height) { return false; }

                int cost = costmap->data[y0 * costmap->info.width + x0];
                if (cost >= 50 || cost == -1) { return false; }
                if (x0 == x1 && y0 == y1) { break; }

                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x0 += sx; }
                if (e2 < dx) { err += dx; y0 += sy; }
            }
            return true;
        }

        // ㅡㅡㅡㅡ 자율주행 알고리즘 Theta* ㅡㅡㅡㅡ
        std::vector<geometry_msgs::msg::Pose> theta_planner(
            const geometry_msgs::msg::Pose& start,
            const geometry_msgs::msg::Pose& goal,
            const nav_msgs::msg::OccupancyGrid::SharedPtr costmap
        ) {
            std::vector<geometry_msgs::msg::Pose> path;
            // 전방 직선 확인
            int start_x, start_y, goal_x, goal_y;
            // 맵 밖이면 빈 값 반환
            if(!world_to_map(start.position.x, start.position.y, start_x, start_y, costmap) ||
               !world_to_map(goal.position.x, goal.position.y, goal_x, goal_y, costmap)) { return path; }

            // 장애물이 없으면 직진
            if (check_LoS(start_x, start_y, goal_x, goal_y, costmap)) {
                path.push_back(start);
                path.push_back(goal);
                return path;
            }
            
            std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open_set;
            std::unordered_map<int, GridNode> closed_set;

            auto get_index = [&](int x, int y) { return y * costmap->info.width + x; };
            auto heuristic = [&](int x, int y) { return std::hypot(goal_x - x, goal_y - y); };

            GridNode start_node = {start_x, start_y, start_x, start_y, 0.0, heuristic(start_x, start_y) };
            open_set.push(start_node);

            int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
            int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

            bool found = false;
            int iterations = 0;

            while (!open_set.empty() && iterations < 5000) {
                GridNode current = open_set.top(); open_set.pop();
                iterations++;

                int current_idx = get_index(current.x, current.y);

                // 더 좋은 비용으로 왔으면 스킵
                if (closed_set.find(current_idx) != closed_set.end() &&
                    closed_set[current_idx].g_cost <= current.g_cost) { continue; }

                closed_set[current_idx] = current;

                if (current.x == goal_x && current.y == goal_y) { found = true;  break; }

                // 8방향 경로 탐색
                for (int i=0; i<8; i++) {
                    int nx = current.x + dx[i];
                    int ny = current.y + dy[i];

                    if (nx < 0 || nx >= (int)costmap->info.width ||
                        ny < 0 || ny >= (int)costmap->info.height) { continue; }

                    int n_idx = get_index(nx, ny);
                    int cost = costmap->data[n_idx];
                    if (cost >= 50 || cost == -1) { continue; } // 장애물 스킵

                    // 상위 노드와 이웃 노드간 LoS 체크
                    int px = current.parent_x, py = current.parent_y;
                    GridNode next_node;
                    next_node.x = nx;
                    next_node.y = ny;

                    if (check_LoS(px, py, nx, ny, costmap)) { // LoS가 통하면 부모 노드로 직접 연결
                        next_node.g_cost = closed_set[get_index(px, py)].g_cost + std::hypot(nx-px, ny-py);
                        next_node.parent_x = px;
                        next_node.parent_y = py;
                    } else { // LoS가 안통하면 현재 노드를 부모 노드로
                        next_node.g_cost = current.g_cost + std::hypot(dx[i], dy[i]);
                        next_node.parent_x = current.x;
                        next_node.parent_y = current.y;
                    }
                    next_node.f_cost = next_node.g_cost + heuristic(nx, ny);

                    if (!closed_set.count(n_idx) || next_node.g_cost < closed_set[n_idx].g_cost) {
                        open_set.push(next_node);
                    }
                }
            }

            // 경로 역추적
            if (found) {
                int cx = goal_x, cy = goal_y;
                std::vector<geometry_msgs::msg::Pose> temp_path;

                while (cx != start_x || cy != start_y) {
                    geometry_msgs::msg::Pose p;
                    double wx, wy;
                    map_to_world(cx, cy, wx, wy, costmap);
                    p.position.x = wx;
                    p.position.y = wy;
                    temp_path.push_back(p);

                    int p_idx = get_index(cx, cy);
                    cx = closed_set[p_idx].parent_x;
                    cy = closed_set[p_idx].parent_y;
                }
                temp_path.push_back(start);
                
                for (auto it = temp_path.rbegin(); it != temp_path.rend(); ++it) { path.push_back(*it); }            
            } else { // 장애물 있으면 현재 위치와 목적지만 TEB에 전달
                path.push_back(start);
                path.push_back(goal);
            }
            return path;
        }

        // ㅡㅡㅡㅡ 자율주행 알고리즘 EB ( 최단시간 x) ㅡㅡㅡㅡ
        bool teb_planner (
            const geometry_msgs::msg::Pose& current_pose,
            const std::vector<geometry_msgs::msg::Pose>& plan,
            geometry_msgs::msg::Twist& cmd_vel_out
        ) {
            if (plan.empty()) { return false; }

            // 로봇의 yaw 계산
            double siny_cosp = 2.0 * (current_pose.orientation.w * current_pose.orientation.z);
            double cosy_cosp = 1.0 - 2.0 * (current_pose.orientation.z * current_pose.orientation.z);
            double current_yaw = std::atan2(siny_cosp, cosy_cosp);

            // 고무줄 경로 추출
            std::vector<std::pair<double, double>> band;
            band.push_back({current_pose.position.x, current_pose.position.y});
            int extract_pts = std::min((int)plan.size(), 10);
            for(int i=0; i<extract_pts; i++) {
                band.push_back({plan[i].position.x, plan[i].position.y});
            }

            // 장애물 위치 > 로봇 기준 절대 좌표
            std::vector<std::pair<double, double>> obstacles;
            if (latest_scan && !latest_scan->ranges.empty()) {
                int total_ranges = latest_scan->ranges.size();
                double angle_increment = latest_scan->angle_increment;
                double angle_min = latest_scan->angle_min;

                for (int i = 0; i < total_ranges; i++) {
                    double range = latest_scan->ranges[i];
                    if (std::isinf(range)) { range = latest_scan->range_max; }

                    // 전방에 장애물 감지되면 회피 기동
                    if (range < 0.6 && range > 0.05) {
                        double angle = angle_min + i * angle_increment;
                        double obs_x = current_pose.position.x + range * std::cos(current_yaw + angle);
                        double obs_y = current_pose.position.y + range * std::sin(current_yaw + angle);
                        obstacles.push_back({obs_x, obs_y});
                    }
                }
            }

            // 밴드 최적화
            double internal_weight = 0.5; // 장력
            double external_weight = 0.02; // 척력

            for (int iter = 0; iter < 5; iter++) {
                std::vector<std::pair<double, double>> new_band = band;
                // 내부 점들만 최적화
                for (size_t i = 1; i < band.size() - 1; ++i) {
                    // 내부 장력
                    double spring_x = ((band[i-1].first + band[i+1].first) / 2.0) - band[i].first;
                    double spring_y = ((band[i-1].second + band[i+1].second) / 2.0) - band[i].second;

                    // 외부 척력
                    double rep_x = 0.0, rep_y = 0.0;
                    for (const auto& obs : obstacles) {
                        double dist = std::hypot(band[i].first - obs.first, band[i].second - obs.second);
                        if (dist < 0.4 && dist >0.01) {
                            double force = 1.0 / (dist * dist);
                            rep_x += force * (band[i].first - obs.first) / dist;
                            rep_y += force * (band[i].second - obs.second) / dist;
                        }
                    }
                    new_band[i].first += internal_weight * spring_x + external_weight * rep_x;
                    new_band[i].second += internal_weight * spring_y + external_weight * rep_y;
                }
                band = new_band;
            }

            // 목표 지점에 인력 작용
            int target_idx = std::min(2, (int)band.size() - 1);
            double dx = band[target_idx].first - current_pose.position.x;
            double dy = band[target_idx].second - current_pose.position.y;

            // 목표 지점 yaw 계산
            double target_yaw = std::atan2(dy, dx);
            double heading_diff = target_yaw - current_yaw;

            // 각도 차이 정규화
            while(heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
            while(heading_diff < -M_PI) heading_diff += 2.0 * M_PI;

            // 거리, 각도 차이 비례 속도 제어
            double target_linear_v = 0.2 * std::max(0.0, 1.0 - std::abs(heading_diff) / (M_PI / 2.0));
            double target_angular_w = heading_diff * 1.5;
            target_linear_v = std::clamp(target_linear_v, 0.0, 0.26);
            target_angular_w = std::clamp(target_angular_w, -1.8, 1.8);

            double max_accel_v = 0.02; // 선속도
            double max_accel_w = 0.2; // 각속도
            target_linear_v = std::clamp(target_linear_v, prev_linear_v - max_accel_v, prev_linear_v + max_accel_v);
            target_angular_w = std::clamp(target_angular_w, prev_angular_w - max_accel_w, prev_angular_w + max_accel_w);

            prev_linear_v = target_linear_v;
            prev_angular_w = target_angular_w;

            cmd_vel_out.linear.x = target_linear_v;
            cmd_vel_out.angular.z = target_angular_w;

            return true;
        }

        bool replanning;
        bool costmap_update;
        double prev_linear_v, prev_angular_w;

        std::string current_mode;
        std::vector<geometry_msgs::msg::Pose> current_global_path;

        sensor_msgs::msg::LaserScan::SharedPtr latest_scan;
        nav_msgs::msg::Odometry::SharedPtr current_odom;
        nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap;
        geometry_msgs::msg::PoseStamped::SharedPtr target_auto_goal;
        geometry_msgs::msg::PoseStamped::SharedPtr target_follow_pose;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav2_status_pub;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gesture_cmd_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr auto_drive_goal_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr follow_target_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

        rclcpp::Time last_gesture_time;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_move>());
    rclcpp::shutdown();
    return 0;
}