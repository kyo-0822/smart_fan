#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/strign.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class RobotVision : public rclcpp::Node {
    public:
        RobotVision() : Node("robot_vision_node"), current_mode("waiting"), frame_counter(0) {
            // ㅡㅡㅡㅡ Yolo 모델 ㅡㅡㅡㅡ
            this->declare_parameter("model_path", "'yolo 모델 이름'.onnx");
            std::string model_path = this->get_parameter("model_path").as_string();

            try {
                yolo_net = cv::dnn::readNetFromeONNX(model_path);
                // yolo_net.setPreferableBackend(cv::dnn:DNN_BACKEND_CUDA);
                // yolo_net.setPreferableTarget(cv::dnn:DNN_TARGET_CUDA);
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "모델 로드 실패");
            }

            // ㅡㅡㅡㅡ publisher ㅡㅡㅡㅡ
            // 사람 - 로봇 각도 오차
            human_offset_pub = this->create_publisher<std_msgs::msg::Float64>("human_offset", 10);
            // -> smart_fan
            yolo_zone_pub = this->create_publisher<std_msgs::msg::Int32>("yolo_zone)", 10);
            // 제스쳐 ID
            gesture_pub = this->create_publisher<std_msgs::msg::Int32>("gesture_id", 10);

            // ㅡㅡㅡㅡ subscription ㅡㅡㅡㅡ
            mode_sub = this->create_subscription<std_msgs::msg::String>(
                "current_mode", 10, std::bind(&RobotVision::mode_callback, this, std::placeholders::_1)
            );

            // 터틀봇3 카메라 ( human )
            cam_A.open(0);
            // ros2 PC 카메라 ( 0, 1, 2, 3 )
            cam_B.open(1);

            timer = this->create_wall_timer(33ms, std::bind(&RobotViison::vision_loop, this));
        }
    
    private:
        void mode_callback() {
            if (current_mode != msg->daat) {
                current_mode = msg->data;
                frame_counter = 0; // 모드 전환시 프레임 초기화
            }
        }

        void vision_loop() {
            cv::Mat frame;

            if(current_mode == "auto_drive" || current_mode == "align") {
                if (cam_A.read(frame)) { // 자율주행 모드이면서 정렬 모드일때 A 카메라 읽기
                    process_human_detection(frame);
                }
            } else if (current_mode == "gesture") { // 제스쳐 모드일 때 B 카메라 읽기
                if (cam_B.read(frame)) {
                    process_gesture_detection(frame);
                }
            } else if (current_mode == "follow") { // follow 모드일때 5프레임 단위로 카메라 스위칭
                if (frame_counter < 5) {
                    if (cam_A.read(frame)) {
                        process_human_detection(frame);
                    }
                } else {
                    if (cam_B.read(frame)) {
                        process_gesture_detection(frame);
                    }
                }

                frame_counter = (frame_counter + 1) % 10;
            }
            // waiting 모드일 때 카메라 연산 x
        }

        void process_human_detection() {
            // TODO: 여기에 사람 인식 YOLO 추론 코드 삽입









            bool human_detected = true;
            double bbox_center_x = frame.cols / 2.0;

            // 사람이 감지되면 사람이 화면 중앙에 오도록
            if (human_detected) {
                double offset = (bbox_center_x - (frame.cols / 2.0)) / (frame.cols / 2.0);

                auto offset_msg = std_msgs::msg::Float64();
                offset_msg.data = offset;
                human_offset_pub->publish(offset_msg);

                int zone = std::round((offset + 1.0) / 2.0 * 4.0);
                if (zone < 0) zone = 0;
                if (zone > 4) zone = 4;

                auto zone_msg = std_msgs::msg::Int32();
                zone_msg.data = zone;
                yolo_zone_pub->publish(zone_msg);
            }
        }

        void process_gesture_detection() {
            //TODO : 여기에 손 제스쳐 인식 yolo 추론 코드 삽입







            int detected_class_id = -1;

            if (detected_class_id >= 0 && detected_class_id <= 3) {
                auto gesture_msg = std_msgs::msg::Int32();
                gesture_msg.data = detected_class_id;
                gesture_pub->publish(gesture_msg);
            }
        }
        
        int frame_counter;
        std::string current_mode;
        cv::VideoCapture cam_A;
        cv::VideoCapture cam_B;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr human_offset_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr yolo_zone_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gesture_pub;

        rclcpp::Subsciption<std_msgs::msg::String>::SharedPtr mode_sub;

        rclcpp::TimerBase::SharedPtr timer;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotVision>());
    rclcpp::shutdown();
    return 0;
}