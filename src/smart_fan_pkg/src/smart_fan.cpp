//smart_fan.cpp
#include <memory>
#include <cmath>
#include <vector>
#include <numeric>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class Smart_Fan : public rclcpp::Node {
    public: 
        Smart_Fan() : Node("smart_fan_node") {
            this->declare_parameter("fan_offset_y", 0.07); // 카메라 기준 왼쪽 7cm 지점
            this->declare_parameter("lidar_to_camera_offset", 0.1); // LiDAR와 카메라 앞 뒤 간격
            fan_offset_y = this->get_parameter("fan_offset_y").as_oduble();
            lidar_to_camera_offset = this->get_parameter("lidar_to_camera_offset").as_oduble();

            fan_angle_pub = this->create_publisher<std_msgs::msg::Float64>("fan_angle", 10);
            gazebo_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 10);

            scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", rclcpp::SensorDataQoS(),
                std::bind(&Smart_Fan::scan_callback, this, std::placeholders::_1)
            );

            yolo_zone_sub = this->create_subscription<std_msgs::msg::Int32>(
                "yolo_zone", 10,
                std::bind(&Smart_Fan::zone_callback, this, std::placeholders::_1)
            );

            mode_sub = this->create_subscription<std_msgs::msg::String>(
                "current_mode", 10,
                std::bind(&Smart_Fan::mode_callback, this, std::placeholders::_1)
            );

            // 구역별 각도
            camera_angle_degree = {24.0, 12.0, 0.0, -12.0, -24.0};

            RCLCPP_INFO(this->get_logger(), "smart_fan 노드 활성화 ...");
        }

    private:
        void mode_callback(const std_msgs::msg::String::SharedPtr msg) { current_mode = msg->data; }
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan = msg; }

        void zone_callback(const std_msgs::msg::Int32::SharedPtr msg) {
            if (current_mode != "follow") { return ;}

            if (!latest_scan) {
                RCLCPP_WARN(this->get_logger(), "LiDAR 데이터 없음");
                return;
            }

            // 이미지 섹터 (이미지 섹터 분류 오류 방어)
            int zone = msg->data;
            if (zone < 0 || zone > 4) { return; }

            // LiDAR에서 해당 각도 거리 추출
            double target_cam_degree = camera_angle_degree[zone];
            double target_cam_radian = target_cam_degree * M_PI / 180.0;
            double lidar_distance = get_distance_from_image(target_cam_radian);

            if (!std::isfinite(lidar_distance) || lidar_distance < 0.2) {
                RCLCPP_WARN(this->get_logger(), "LiDAR 거리 기본값 : 1m");
                lidar_distance = 1.0;
            }

            // 카메라 기준 객체 거리 계산 ( LiDAR 활용 )
            double distance = lidar_distance - lidar_to_camera_offset;
            if (distance <= 0.0) { distance = 0.1; }

            // 삼각 함수를 활용, 선풍기 회전 각도 계산
            double c_x = distance * cos(target_cam_radian);
            double c_y = distance * sin(target_cam_radian);

            double vector_x = c_x; // x축은 고정
            double vector_y = c_y - fan_offset_y;
            
            double target_radian = std::atan2(vector_y, vector_x);
            double target_angle = target_radian * 180.0 / M_PI;

            // 아두이노 퍼블리시
            auto angle_msg = std_msgs::msg::Float64();
            angle_msg.data = target_angle;
            fan_angle_pub->publish(angle_msg);
            
            // 가제보 퍼블리시
            trajectory_msgs::msg::JointTrajectory trajec_msg;
            trajec_msg.header.stamp = this->now();
            trajec_msg.joint_names.push_back("fan_joint");

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions.push_back(target_radian);
            point.time_from_start.sec = 0;
            point.time_from_start.nanosec = 200000000;
            trajec_msg.points.push_back(point);
            gazebo_pub->publish(trajec_msg);
            
            RCLCPP_INFO(this-> get_logger(), "Zone: %d | LiDAR: %.2fm | Dist: %.2fm | target angle: %.1f deg",
                                              zone, lidar_distance, distance, target_angle);
        }
    
        double get_distance_from_image(double target_angle_radian) {
            // LiDAR(반시계) 각도 보정 (음수인 경우 360도에서 빼서 계산)
            double diff = target_angle_radian - latest_scan->angle_min;
            while (diff < 0) { diff += 2.0 * M_PI; }
            whiel (diff >= 2.0 * M_PI) { diff -= 2.0 * M_PI }

            int index = static_cast<int>(std::round(diff / latest_scan->angle_increment));
            int total_ray = static_cast<int>(latest_scan->ranges.size());

            index = std::max(0, std::min(index, total_ray - 1));

            double sum = 0.0;
            int count = 0;
            // 객체 감지 LiDAR 데이터 주위 5개 인덱스 평균값
            for (int i = -2; i <= 2; i++) {
                int idx = std::max(0, std::min(index + i, total_ray -1));
                double range = latest_scan->ranges[idx];
                if (std::isfinite(range) &&
                    range >= latest_scan->range_min &&
                    range <= latest_scan->range_max) {
                    sum += range;
                    count++;
                }
            }
            // (조건식) count가 하나라도 있으면 평균값, 없으면 무한값(측정 불가)
            return (count > 0) ? (sum / count) : std::numeric_limits<double>::infinity(); 
        }

        std::string current_mode = "waiting";
        
        double fan_offset_y;
        double lidar_to_camera_offset;

        std::vector<double> camera_angle_degree;
        sensor_msgs::msg::LaserScan::SharedPtr latest_scan;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fan_angle_pub;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gazebo_pub;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_zone_sub;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared <Smart_Fan>());
    rclcpp::shutdown();
    return 0;
}