// robot_move.cpp
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
// [변경] nav_msgs/odometry.hpp 제거 → AMCL pose 헤더 추가
// [삭제] #include "nav_msgs/msg/odometry.hpp"
// [삭제] #include "tf2_ros/transform_listener.h"
// [삭제] #include "tf2_ros/buffer.h"
// [삭제] #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // [추가] /amcl_pose 타입
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

struct GridNode {
    int x, y;
    int parent_x, parent_y;
    double g_cost;
    double f_cost;
    bool operator>(const GridNode& other) const { return f_cost > other.f_cost; }
};

class Robot_move : public rclcpp::Node {
    public:
        Robot_move() : Node("robot_move_node") {
            current_mode   = "waiting";
            replanning     = false;
            costmap_update = false;
            prev_linear_v  = 0.0;
            prev_angular_w = 0.0;
            // [변경] tf_buffer_, tf_listener_ 초기화 제거

            // publisher
            cmd_vel_pub     = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            nav2_status_pub = this->create_publisher<std_msgs::msg::String>("nav2_status", 10);
            global_path_pub = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);

            // subscription
            mode_sub = this->create_subscription<std_msgs::msg::String>(
                "current_mode", 10,
                std::bind(&Robot_move::mode_callback, this, std::placeholders::_1));
            gesture_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "gesture_cmd", 10,
                std::bind(&Robot_move::gesture_cmd_callback, this, std::placeholders::_1));
            auto_drive_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "auto_drive_goal", 10,
                std::bind(&Robot_move::auto_drive_callback, this, std::placeholders::_1));
            follow_target_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "follow_target_pose", 10,
                std::bind(&Robot_move::follow_target_callback, this, std::placeholders::_1));
            // [변경] /odom 구독 제거 → /amcl_pose 구독으로 교체
            // [삭제] odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", ...)
            // [추가] AMCL은 map 프레임 기준 위치를 직접 퍼블리시하므로 TF 변환 불필요
            amcl_pose_sub = this->create_subscription<
                geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "/amcl_pose",
                    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
                    std::bind(&Robot_move::amcl_pose_callback, this, std::placeholders::_1));
            costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                std::bind(&Robot_move::costmap_callback, this, std::placeholders::_1));
            scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", rclcpp::SensorDataQoS(),
                std::bind(&Robot_move::scan_callback, this, std::placeholders::_1));

            timer = this->create_wall_timer(100ms, std::bind(&Robot_move::control_loop, this));
            RCLCPP_INFO(this->get_logger(), "robot_move 노드 활성화 ...");
        }

    private:
        // [변경] odom_callback 제거 → amcl_pose_callback 으로 교체
        // /amcl_pose : AMCL이 map 프레임 기준으로 퍼블리시하는 위치 추정값
        void amcl_pose_callback(
            const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            current_amcl_pose = msg;
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan = msg; }

        void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            if (!current_costmap) {
                current_costmap = msg;
                costmap_update  = true;
                RCLCPP_INFO(this->get_logger(), "Map origin x:%.2f, y:%.2f",
                            msg->info.origin.position.x, msg->info.origin.position.y);
            }
        }

        void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
            if (current_mode != msg->data) {
                current_mode = msg->data;
                if (current_mode == "waiting") { stop_robot(); }
            }
        }

        void gesture_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            if (current_mode != "gesture" && current_mode != "follow") { return; }
            cmd_vel_pub->publish(*msg);
            prev_linear_v  = msg->linear.x;
            prev_angular_w = msg->angular.z;
        }

        void auto_drive_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            target_auto_goal = msg;
            replanning = true;
            current_global_path.clear();
        }

        void follow_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            target_follow_pose = msg;
        }

        // [변경] TF 변환(odom → map) 제거
        // AMCL이 이미 map 프레임 기준 pose를 퍼블리시하므로 그대로 사용
        bool get_robot_pose_in_map(geometry_msgs::msg::Pose& map_pose) {
            if (!current_amcl_pose) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "AMCL pose 미수신 - 자율주행 대기 중");
                return false;
            }
            // PoseWithCovarianceStamped.pose.pose → geometry_msgs::Pose 추출
            map_pose = current_amcl_pose->pose.pose;
            return true;
        }

        void publish_global_path() {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp    = this->now();
            path_msg.header.frame_id = "map";
            for (const auto& pose : current_global_path) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.frame_id = "map";
                ps.pose = pose;
                path_msg.poses.push_back(ps);
            }
            global_path_pub->publish(path_msg);
        }

        void control_loop() {
            if (current_mode == "waiting" || current_mode == "gesture" || current_mode == "follow") { return; }
            if (current_mode != "auto_drive") { return; }
            if (!current_costmap || !target_auto_goal) { return; }

            // 긴급 정지 : LiDAR 전방 ±45도 이내 0.3m 미만 장애물 감지 시 즉시 정지
            if (latest_scan && !latest_scan->ranges.empty()) {
                int    total     = latest_scan->ranges.size();
                double angle_min = latest_scan->angle_min;
                double angle_inc = latest_scan->angle_increment;
                for (int i = 0; i < total; i++) {
                    double angle = angle_min + i * angle_inc;
                    if (std::abs(angle) > M_PI / 4.0) { continue; }
                    double r = latest_scan->ranges[i];
                    if (std::isfinite(r) && r < 0.3) {
                        stop_robot();
                        return;
                    }
                }
            }

            geometry_msgs::msg::Pose robot_map_pose;
            if (!get_robot_pose_in_map(robot_map_pose)) { return; }

            double dx = target_auto_goal->pose.position.x - robot_map_pose.position.x;
            double dy = target_auto_goal->pose.position.y - robot_map_pose.position.y;

            if (std::hypot(dx, dy) < 0.2) {
                stop_robot();
                target_auto_goal = nullptr;
                current_global_path.clear();
                publish_global_path();

                std_msgs::msg::String status_msg;
                status_msg.data = "arrived";
                nav2_status_pub->publish(status_msg);
                return;
            }

            if (replanning || costmap_update) {
                current_global_path = theta_planner(
                    robot_map_pose,
                    target_auto_goal->pose,
                    current_costmap
                );
                replanning     = false;
                costmap_update = false;
                publish_global_path();
            }

            if (!current_global_path.empty()) {
                while (current_global_path.size() > 1) {
                    double wx = current_global_path[0].position.x;
                    double wy = current_global_path[0].position.y;
                    if (std::hypot(wx - robot_map_pose.position.x,
                                   wy - robot_map_pose.position.y) < 0.15) {
                        current_global_path.erase(current_global_path.begin());
                    } else { break; }
                }

                geometry_msgs::msg::Twist cmd_vel;
                if (teb_planner(robot_map_pose, current_global_path, cmd_vel)) {
                    cmd_vel_pub->publish(cmd_vel);
                } else {
                    stop_robot();
                }
            } else {
                stop_robot();
            }
        }

        void stop_robot() {
            prev_linear_v  = 0.0;
            prev_angular_w = 0.0;
            cmd_vel_pub->publish(geometry_msgs::msg::Twist());
        }

        bool world_to_map(double wx, double wy, int& mx, int& my,
                          const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
            double origin_x = costmap->info.origin.position.x;
            double origin_y = costmap->info.origin.position.y;
            double res      = costmap->info.resolution;
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

        bool check_LoS(int x0, int y0, int x1, int y1,
                       const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
            int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
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
                if (e2 <  dx) { err += dx; y0 += sy; }
            }
            return true;
        }

        std::vector<geometry_msgs::msg::Pose> theta_planner(
            const geometry_msgs::msg::Pose& start,
            const geometry_msgs::msg::Pose& goal,
            const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
        {
            std::vector<geometry_msgs::msg::Pose> path;
            int start_x, start_y, goal_x, goal_y;
            if (!world_to_map(start.position.x, start.position.y, start_x, start_y, costmap) ||
                !world_to_map(goal.position.x,  goal.position.y,  goal_x,  goal_y,  costmap)) { return path; }

            if (check_LoS(start_x, start_y, goal_x, goal_y, costmap)) {
                path.push_back(start); path.push_back(goal);
                return path;
            }

            std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open_set;
            std::unordered_map<int, GridNode> closed_set;

            auto get_index = [&](int x, int y) { return y * (int)costmap->info.width + x; };
            auto heuristic = [&](int x, int y)  { return std::hypot(goal_x - x, goal_y - y); };

            open_set.push({start_x, start_y, start_x, start_y, 0.0, heuristic(start_x, start_y)});

            int ddx[8] = {1,-1, 0, 0, 1, 1,-1,-1};
            int ddy[8] = {0, 0, 1,-1, 1,-1, 1,-1};
            bool found      = false;
            int  iterations = 0;

            while (!open_set.empty() && iterations < 5000) {
                GridNode current = open_set.top(); open_set.pop();
                iterations++;
                int current_idx = get_index(current.x, current.y);
                if (closed_set.count(current_idx) &&
                    closed_set[current_idx].g_cost <= current.g_cost) { continue; }
                closed_set[current_idx] = current;
                if (current.x == goal_x && current.y == goal_y) { found = true; break; }

                for (int i = 0; i < 8; i++) {
                    int nx = current.x + ddx[i];
                    int ny = current.y + ddy[i];
                    if (nx < 0 || nx >= (int)costmap->info.width ||
                        ny < 0 || ny >= (int)costmap->info.height) { continue; }
                    int n_idx = get_index(nx, ny);
                    int cost  = costmap->data[n_idx];
                    if (cost >= 50 || cost == -1) { continue; }

                    GridNode next_node;
                    next_node.x = nx; next_node.y = ny;
                    int px = current.parent_x, py = current.parent_y;
                    if (check_LoS(px, py, nx, ny, costmap)) {
                        next_node.g_cost   = closed_set[get_index(px, py)].g_cost + std::hypot(nx-px, ny-py);
                        next_node.parent_x = px; next_node.parent_y = py;
                    } else {
                        next_node.g_cost   = current.g_cost + std::hypot(ddx[i], ddy[i]);
                        next_node.parent_x = current.x; next_node.parent_y = current.y;
                    }
                    next_node.f_cost = next_node.g_cost + heuristic(nx, ny);
                    if (!closed_set.count(n_idx) || next_node.g_cost < closed_set[n_idx].g_cost) {
                        open_set.push(next_node);
                    }
                }
            }

            if (found) {
                int cx = goal_x, cy = goal_y;
                std::vector<geometry_msgs::msg::Pose> temp_path;
                while (cx != start_x || cy != start_y) {
                    geometry_msgs::msg::Pose p;
                    double wx, wy;
                    map_to_world(cx, cy, wx, wy, costmap);
                    p.position.x = wx; p.position.y = wy;
                    temp_path.push_back(p);
                    int p_idx = get_index(cx, cy);
                    cx = closed_set[p_idx].parent_x;
                    cy = closed_set[p_idx].parent_y;
                }
                temp_path.push_back(start);
                for (auto it = temp_path.rbegin(); it != temp_path.rend(); ++it) { path.push_back(*it); }
            } else {
                path.push_back(start); path.push_back(goal);
            }
            return path;
        }

        // [변경] current_pose 는 AMCL 기반 map 프레임 pose 사용
        // LiDAR 장애물 좌표 변환 시 AMCL yaw(누적 오차 없음) 사용 → 정확도 향상
        bool teb_planner(
            const geometry_msgs::msg::Pose& current_pose,
            const std::vector<geometry_msgs::msg::Pose>& plan,
            geometry_msgs::msg::Twist& cmd_vel_out)
        {
            if (plan.empty()) { return false; }

            double siny        = 2.0 * (current_pose.orientation.w * current_pose.orientation.z);
            double cosy        = 1.0 - 2.0 * (current_pose.orientation.z * current_pose.orientation.z);
            double current_yaw = std::atan2(siny, cosy);

            std::vector<std::pair<double,double>> band;
            band.push_back({current_pose.position.x, current_pose.position.y});
            int extract_pts = std::min((int)plan.size(), 10);
            for (int i = 0; i < extract_pts; i++) {
                band.push_back({plan[i].position.x, plan[i].position.y});
            }

            // [변경] 장애물 좌표 변환에 AMCL yaw(map 프레임) 사용
            // 기존 odom yaw 대비 장거리 주행 시 누적 오차 제거
            std::vector<std::pair<double,double>> obstacles;
            if (latest_scan && !latest_scan->ranges.empty()) {
                int    total_ranges    = latest_scan->ranges.size();
                double angle_increment = latest_scan->angle_increment;
                double angle_min       = latest_scan->angle_min;
                for (int i = 0; i < total_ranges; i++) {
                    double range = latest_scan->ranges[i];
                    if (std::isinf(range)) { range = latest_scan->range_max; }
                    if (range < 0.6 && range > 0.05) {
                        double angle = angle_min + i * angle_increment;
                        obstacles.push_back({
                            current_pose.position.x + range * std::cos(current_yaw + angle),
                            current_pose.position.y + range * std::sin(current_yaw + angle)
                        });
                    }
                }
            }

            double internal_weight = 0.5;
            double external_weight = 0.02;
            for (int iter = 0; iter < 5; iter++) {
                auto new_band = band;
                for (size_t i = 1; i < band.size() - 1; ++i) {
                    double spring_x = ((band[i-1].first  + band[i+1].first)  / 2.0) - band[i].first;
                    double spring_y = ((band[i-1].second + band[i+1].second) / 2.0) - band[i].second;
                    double rep_x = 0.0, rep_y = 0.0;
                    for (const auto& obs : obstacles) {
                        double dist = std::hypot(band[i].first - obs.first, band[i].second - obs.second);
                        if (dist < 0.4 && dist > 0.01) {
                            double force = 1.0 / (dist * dist);
                            rep_x += force * (band[i].first  - obs.first)  / dist;
                            rep_y += force * (band[i].second - obs.second) / dist;
                        }
                    }
                    new_band[i].first  += internal_weight * spring_x + external_weight * rep_x;
                    new_band[i].second += internal_weight * spring_y + external_weight * rep_y;
                }
                band = new_band;
            }

            double lookahead  = 0.4;
            int    target_idx = 1;
            for (int i = 1; i < (int)band.size(); i++) {
                double d = std::hypot(band[i].first  - current_pose.position.x,
                                      band[i].second - current_pose.position.y);
                target_idx = i;
                if (d >= lookahead) { break; }
            }

            double dx = band[target_idx].first  - current_pose.position.x;
            double dy = band[target_idx].second - current_pose.position.y;

            double target_yaw   = std::atan2(dy, dx);
            double heading_diff = target_yaw - current_yaw;
            while (heading_diff >  M_PI) heading_diff -= 2.0 * M_PI;
            while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;

            double target_linear_v  = 0.0;
            double target_angular_w = heading_diff * 0.8;
            if (std::abs(heading_diff) < M_PI / 3.0) {
                target_linear_v = 0.2 * (1.0 - std::abs(heading_diff) / (M_PI / 2.0));
            }

            target_linear_v  = std::clamp(target_linear_v,  0.0,  0.26);
            target_angular_w = std::clamp(target_angular_w, -1.8,  1.8);

            double max_accel_v = 0.05;
            double max_accel_w = 0.2;
            target_linear_v  = std::clamp(target_linear_v,  prev_linear_v  - max_accel_v, prev_linear_v  + max_accel_v);
            target_angular_w = std::clamp(target_angular_w, prev_angular_w - max_accel_w, prev_angular_w + max_accel_w);

            prev_linear_v  = target_linear_v;
            prev_angular_w = target_angular_w;

            cmd_vel_out.linear.x  = target_linear_v;
            cmd_vel_out.angular.z = target_angular_w;
            return true;
        }

        std::string current_mode;
        bool   replanning, costmap_update;
        double prev_linear_v, prev_angular_w;

        std::vector<geometry_msgs::msg::Pose>     current_global_path;
        sensor_msgs::msg::LaserScan::SharedPtr     latest_scan;
        // [변경] current_odom 제거 → current_amcl_pose 로 교체
        // [삭제] nav_msgs::msg::Odometry::SharedPtr current_odom;
        geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_amcl_pose; // [추가]
        nav_msgs::msg::OccupancyGrid::SharedPtr    current_costmap;
        geometry_msgs::msg::PoseStamped::SharedPtr target_auto_goal;
        geometry_msgs::msg::PoseStamped::SharedPtr target_follow_pose;

        // [변경] TF buffer/listener 멤버 제거
        // [삭제] std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
        // [삭제] std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     nav2_status_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr       global_path_pub;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           mode_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       gesture_cmd_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr auto_drive_goal_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr follow_target_sub;
        // [변경] odom_sub 제거 → amcl_pose_sub 로 교체
        // [삭제] rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub; // [추가]
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  costmap_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub;

        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_move>());
    rclcpp::shutdown();
    return 0;
}