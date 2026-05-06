#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multu_array.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class RobotVision : public rclcpp::Node {
    public:
        RobotVision() : Node("robot_vision_node"), current_mode("waiting"), frame_counter(0) {
            // ㅡㅡㅡㅡ Yolo 모델 ㅡㅡㅡㅡ
            this->declare_parameter("model_path", "'yolo 모델 이름'.onnx");
            std::string model_path = this->get_parameter("model_path").as_string();

            try {
                yolo_net = cv::dnn::readNetFromONNX(model_path);
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
            gesture_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("gesture_id", 10);

            // ㅡㅡㅡㅡ subscription ㅡㅡㅡㅡ
            mode_sub = this->create_subscription<std_msgs::msg::String>(
                "current_mode", 10, std::bind(&RobotVision::mode_callback, this, std::placeholders::_1)
            );

            // 터틀봇3 카메라 ( human )
            cam_A.open(0);
            // ros2 PC 카메라 ( 0, 1, 2, 3 )
            cam_B.open(2);

            timer = this->create_wall_timer(33ms, std::bind(&RobotVision::vision_loop, this));
            
            RCLCPP_INFO(this->get_logger(), "robot_vision activated ...");
        }
    
    private:
        void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
            if (current_mode != msg->daat) {
                current_mode = msg->data;
                frame_counter = 0; // 모드 전환시 프레임 초기화
            }
        }

        void vision_loop() {
            cv::Mat frame;
            if (current_mode == "waiting") { return; }
            
            if(current_mode == "auto_drive" || current_mode == "align") {
                if (cam_A.read(frame)) { process_human_detection(frame); } // 자율주행 모드이면서 정렬 모드일때 A 카메라 읽기
            } else if (current_mode == "gesture") { // 제스쳐 모드일 때 B 카메라 읽기
                if (cam_B.read(frame)) { process_gesture_detection(frame); }
            } else if (current_mode == "follow") {
                if (frame_counter < 5) { // follow 모드일때 5프레임 단위로 카메라 스위칭
                    if (cam_A.read(frame)) { process_human_detection(frame); }
                } else {
                    if (cam_B.read(frame)) { process_gesture_detection(frame); }
                }
                frame_counter = (frame_counter + 1) % 10;
            }
        }

        void process_human_detection() {
            cv::Mat blob;
            cv::dnn::blobFromImage(frame, blob, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false); 

            yolo_net.setInput(blob);
            std::vector<cv::Mat> outputs;
            yolo_net.forward(outputs);

            float* data = (float*)outputs[0].data;
            const int rows = outputs[0].size[2];

            bool human_detected = true;
            double best_confidence = 0.0; // 신뢰도
            double bbox_center_x = -1.0;

            for (int i=0; i<rows; i++) {
                float confidence = data[(4+4) * rows + i];
                if (confidence > 0.6 && confidence > best_confidence) {
                    best_confidence = confidence;
                    bbox_center_x = data[0 * rows + i];
                    human_detected = true;
                }
            }

            // 사람이 감지되면 사람이 화면 중앙에 오도록
            if (human_detected) {
                double scale = static_cast<double>(frame.cols) / 640.0;
                double real_center_x = bbox_center_x * scale;
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
            cv::Mat blob;
            cv::dnn:blobFromImage(frame, blob, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false);

            yolo_net.setInput(blob);
            std::vector<cv::Mat> outputs;
            yolo_net.forward(outputs);

            float* data = (float*)outputs[0].data;
            const int rows = outputs[0].size[2];

            int detected_class_id = -1;
            double best_confidence = 0.0;
            double bbox_center_x = -1.0;

            // 가장 정확도 높은 제스쳐 찾기
            for (int i=0; i<rows; i++) {
                for (int j=0; j<4; ++j) { // (0, 1, 2, 3)
                    float confidence = data[(4 + j) * rows + i];
                    if (confidence > 0.6 && confidence > best_confidence){
                        best_confidence = confidence;
                        detected_class_id = j;
                        bbox_center_x = data[0 * rows + i];
                    }
                }
            }

            if (detected_class_id >= 0 && detected_class_id <= 3) {
                double scale = static_cast<double>(frame.cols) / 640.0;
                int real_center_x = std::round(bbox_center_x * scale); 

                auto gesture_msg = std_msgs::msg::Int32MultiArray();
                gesture_msg.data.push_back(detected_class_id);
                gesture_msg.data.push_back(real_center_x);
                gesture_pub->publish(gesture_msg);
            }
        }
        
        int frame_counter;

        std::string current_mode;

        cv::VideoCapture cam_A;
        cv::VideoCapture cam_B;
        cv::dnn::Net yolo_net;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr human_offset_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr yolo_zone_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gesture_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
        rclcpp::TimerBase::SharedPtr timer;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotVision>());
    rclcpp::shutdown();

    return 0;
}