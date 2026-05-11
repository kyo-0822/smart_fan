//robot_vision.cpp
#include <memory>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

using namespace std::chrono_literals;

class RobotVision : public rclcpp::Node {
    public:
        RobotVision() : Node("robot_vision_node"),
                        frame_counter(0),   
                        current_mode("waiting"),
                        camA_ready(false) {
            // ㅡㅡㅡㅡ Yolo 모델 ㅡㅡㅡㅡ
            this->declare_parameter("model_path", "/home/handsome/handsome_ws/src/handsome_pkg/tired_human.v3i.yolov8/runs/detect/yolo_model/weights/best.onnx");
            std::string model_path = this->get_parameter("model_path").as_string();

            try {
                yolo_net = cv::dnn::readNetFromONNX(model_path);
                RCLCPP_INFO(this->get_logger(), "모델 로드 성공");
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "모델 로드 실패: %s", e.what());
            }

            // ㅡㅡㅡㅡ publisher ㅡㅡㅡㅡ
            human_offset_pub = this->create_publisher<std_msgs::msg::Float64>("human_offset", 10);
            yolo_zone_pub = this->create_publisher<std_msgs::msg::Int32>("yolo_zone", 10);
            gesture_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("gesture_data", 10);

            // ㅡㅡㅡㅡ subscription ㅡㅡㅡㅡ
            mode_sub = this->create_subscription<std_msgs::msg::String>(
                "current_mode", 10,
                std::bind(&RobotVision::mode_callback, this, std::placeholders::_1)
            );
            
            // 터틀봇3 카메라
            cam_a_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                "/camera/image_raw/compressed", rclcpp::SensorDataQoS(),
                std::bind(&RobotVision::cam_a_callback, this, std::placeholders::_1)
            );

            // ros2 PC 카메라
            cam_B.open(0, cv::CAP_V4L2);
            if (!cam_B.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "cam_B 연결 실패");
            } else {
                cam_B.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
                cam_B.set(cv::CAP_PROP_FPS, 30);
                RCLCPP_INFO(this->get_logger(), "cam_B 연결 성공");
            }

            timer = this->create_wall_timer(100ms, std::bind(&RobotVision::vision_loop, this));
            
            RCLCPP_INFO(this->get_logger(), "robot_vision 노드 활성화 ...");
        }
    
    private:
        void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
            if (current_mode != msg->data) {
                current_mode = msg->data;
                frame_counter = 0; // 모드 전환시 프레임 초기화

                if (current_mode == "gesture" || current_mode == "auto_drive") {
                    if (cam_B.isOpened()) {
                        cv::Mat dummy;
                        for (int i = 0; i < 5; i++) { cam_B.read(dummy); }
                    }
                }

                RCLCPP_INFO(this->get_logger(), "모드 전환 %s", current_mode.c_str());
            }
        }

        void cam_a_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            try {
                cv::Mat decoded = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
                if (!decoded.empty()) {
                    camA_frame = decoded;
                    camA_ready = true;
                }
            } catch (const cv::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "camA 디코드 실패: %s", e.what());
            }
        }

        void vision_loop() {
            if (current_mode == "waiting" || current_mode == "auto_drive") { return; }

            cv::Mat frame_A, frame_B;
            bool has_A = false, has_B = false;

            if (camA_ready && !camA_frame.empty()) {
                frame_A = camA_frame.clone();
                has_A = true;
                camA_ready = false;
            }
            if (cam_B.isOpened()) { has_B = cam_B.read(frame_B); }

            if(current_mode == "gesture") {
                if (has_B) {
                    process_gesture_detection(frame_B, false);
                    cv::imshow("camB", frame_B);
                }
            } else if (current_mode == "follow") {
                bool use_cam_b = ((frame_counter / 5) % 2 == 1);
                if (!use_cam_b) {
                    if (has_A) {
                        process_human_detection(frame_A);
                        cv::imshow("camA", frame_A);
                    }
                } else {
                    if (has_B) {
                        process_gesture_detection(frame_B, true);
                        cv::imshow("camB", frame_B);
                    }
                }
                frame_counter = (frame_counter + 1) % 10;
            }
            cv::waitKey(1);
        }

        void process_human_detection(cv::Mat& frame) {
            cv::Mat blob;
            cv::dnn::blobFromImage(frame, blob, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false); 
            yolo_net.setInput(blob);

            std::vector<cv::Mat> outputs;
            yolo_net.forward(outputs);

            if (outputs.empty() || outputs[0].dims < 3) { return; }

            const int dimensions = outputs[0].size[1];
            const int rows = outputs[0].size[2];
            if (dimensions < 9) return;

            float* data = (float*)outputs[0].data;

            bool human_detected = false;
            double best_confidence = 0.0; // 신뢰도

            // 바운딩 박스
            double bbox_w = 0.0, bbox_cy = 0.0, bbox_h = 0.0;
            double bbox_center_x = -1.0;
            

            for (int i = 0; i < rows; i++) {
                float confidence = data[8 * rows + i];
                if (confidence > 0.6 && confidence > best_confidence) {
                    best_confidence = confidence;
                    bbox_center_x = data[0 * rows + i];
                    bbox_cy = data[1 * raws + i];
                    bbox_w = data[2 * raws + i];
                    bbox_h = data[3 * raws + i];
                    human_detected = true;
                }
            }

            // 사람이 감지되면 사람이 화면 중앙에 오도록
            if (human_detected) {
                double scale_x = static_cast<double>(frame.cols) / 640.0;
                double scale_y = static_cast<double>(frame.rows) / 640.0;

                double real_center_x = bbox_center_x * scale_x;
                double offset = (real_center_x - (frame.cols / 2.0)) / (frame.cols / 2.0);

                auto offset_msg = std_msgs::msg::Float64();
                offset_msg.data = offset;
                human_offset_pub->publish(offset_msg);

                int zone = static_cast<int>(std::round((offset + 1.0) / 2.0 * 4.0));
                zone = std::max(0, std::min(zone, 4));
                auto zone_msg = std_msgs::msg::Int32();
                zone_msg.data = zone;
                yolo_zone_pub->publish(zone_msg);

                // 바운딩 박스 계산
                int x1 = static_case<int>((bbox_center_x - bbox_w / 2.0) * scale_x);
                int y1 = static_case<int>((bbox_cy - bbox_h / 2.0) * scale_y);
                int x2 = static_case<int>((bbox_center_x - bbox_w / 2.0) * scale_x);
                int y2 = static_case<int>((bbox_cy - bbox_h / 2.0) * scale_y);

                x1 = std::max(0, x1); y1 = std::max(0, y1);
                x2 = std::min(frame.cols - 1, x2);
                y2 = std::min(frame.rows - 1, y2);

                cv::Scalar bbox_color;
                if (zone==0) { bbox_color = cv::Scalar(255, 0, 0); } // 파랑 : 좌회전
                else if (zone == 4) { bbox_color = cv::Scalar(0, 0, 255); } // 빨강 : 우회전
                else { bbox_color = cv::Scalar(0, 255, 0); } // 초록 : 정지

                cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), bbox_color, 2);

                cv::putText(frame,
                    "Human : " + std::to_string((int)(best_confidence * 100)) + "%",
                    cv::Point(x1, std::max(y1 - 10, 15)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, bbox_color, 2
                );
            }
        }

        void process_gesture_detection(cv::Mat& frame, bool fist_only) {
            cv::Mat blob;
            cv::dnn::blobFromImage(frame, blob, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false);
            yolo_net.setInput(blob);

            std::vector<cv::Mat> outputs;
            yolo_net.forward(outputs);

            if (outputs.empty() || outputs[0].dims < 3) { return; }

            const int dimensions = outputs[0].size[1];
            const int rows = outputs[0].size[2];
            if (dimensions < 8) { return; }

            float* data = (float*)outputs[0].data;

            int detected_class_id = -1;
            double best_confidence = 0.0;
            double bbox_center_x = -1.0;

            // 가장 정확도 높은 제스쳐 찾기
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < 4; j++) {
                    float confidence = data[(4 + j) * rows + i];
                    if (confidence > 0.6 && confidence > best_confidence){
                        best_confidence = confidence;
                        detected_class_id = j;
                        bbox_center_x = data[0 * rows + i];
                    }
                }
            }

            if (detected_class_id < 0) { return; }
            if (fist_only && detected_class_id != 3) { return; }

            double scale = static_cast<double>(frame.cols) / 640.0;
            int real_center_x = static_cast<int>(std::round(bbox_center_x * scale)); 

            auto gesture_msg = std_msgs::msg::Int32MultiArray();
            gesture_msg.data.push_back(detected_class_id);
            gesture_msg.data.push_back(real_center_x);
            gesture_pub->publish(gesture_msg);

            // ㅡㅡㅡㅡ 감지 결과 시각화 ㅡㅡㅡㅡ
            const std::vector<std::string> class_names =
                {"Stop", "Come", "Back", "Mode_Change"};
            cv::putText(frame,
                class_names[detected_class_id]
                    + " (" + std::to_string((int)(best_confidence * 100)) + "%)",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 0, 255), 2);
        }
        
        int frame_counter;
        std::string current_mode;
        bool camA_ready;
        
        cv::VideoCapture cam_B;
        cv::Mat camA_frame;
        cv::dnn::Net yolo_net;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr human_offset_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr yolo_zone_pub;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr gesture_pub;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr cam_a_sub;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotVision>());
    rclcpp::shutdown();

    return 0;
}