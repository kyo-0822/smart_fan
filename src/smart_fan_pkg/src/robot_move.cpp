#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
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
            nav_staus_pub = this->create_puiblisher<std_msgs::msg::String>("nav_status", 10);

            // ㅡㅡㅡㅡ subsciption ㅡㅡㅡㅡ
            mode_sub = this->create_subscription<std_msgs::msg::String>(
                "current_mode", 10, std::bind(&Robot_Move::mode_callback, this, std::placeholders::_1)
            );
            gesture_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "gesture_cmd", 10, std::bind(&Robot_Move::gesture_cmd_callback, this, std::placeholders::_1)
            );
            auto_drive_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "auto_drive_goal", 10, std::bind(&Robot_Move::auto_drive_callback, this, std::placeholders::_1)
            );
            follow_target_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "follow_target", 10, std::bind(&Robot_Move::follow_target_callback, this, std::placeholders::_1)
            );

            nav2_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
            RCLCPP_INFO(this->get_logger(), "robot activated");
        }

    private:
        void mode_callback() {
            currnet_mode = msg->data;

            if (current_mode == "waiting") {
                geometry_msgs::msg::Twist stpo_cmd;
                cmd_vel_pub->pulbish(stop_cmd);
            }
        }

        void gesture_cmd_callback(const geometry_msgs::mgs::Twist::SharedPtr msg) {
            if (current_mode == "gesture") {
                cmd_vel_pub->publish(*msg);
            }
        }
        
        void auto_drive_callback(const geomety_smgs::msg::PoseStamped::SharedPtr msg) {
            if (current_mode == "auto_drive") {
                // 여기에 Thet* + TEB 알고리즘을 직접 구현하고 싶어
            }
        }

        void follow_target_callback() {
            if (current_mode == "follow") { 
                // 여기에 TEB 알고리즘을 직접 구현하고 싶어
            }
        }


        // 호출 지점 출발
        void send_nav2_goal(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
            if (!nav2_client->wait_for_action_server(2s)) {
                return;
            }
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose = *pose;

            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&Robot_Commander::nav2_result_callback, this, std::placeholders::_1);

            nav2_client->async_send_goal(goal_msg, send_goal_options);
        }
        
        // 호출 지점 도착 여부 확인
        void nav2_result_callback(const GoalHandleNav::WrappedResult & result) {
            std_msgs::msg::String status_msg;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "mode change");
                current_mode = "arrived";
            } else {
                current_mode = "failed";
            }
            nav_status_pub->publish(status_msg);
        }


        std::string current_mode;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_staus_pub;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gesture_cmd_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr auto_drive_goal_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr follow_target_sub;

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot_move());
    rclcpp::shutdown();
    return 0;
}






















        this->declare_parameter("target_distance", 0.8);



        