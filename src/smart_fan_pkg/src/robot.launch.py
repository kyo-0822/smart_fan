# handsome_robot.launch.py
#
# 실행 방법:
#   ros2 launch handsome_pkg handsome_robot.launch.py
#
# 카메라 토픽 직접 지정:
#   ros2 launch handsome_pkg handsome_robot.launch.py cam_a_topic:=/your/camera/topic
#
# YOLO 모델 경로 직접 지정:
#   ros2 launch handsome_pkg handsome_robot.launch.py model_path:=/your/model/path.onnx

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ================================================================
    # Launch Arguments (런치 시 외부에서 덮어쓸 수 있는 파라미터)
    # ================================================================

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(
            '/home/handsome/handsome_ws/src/handsome_pkg',
            'tired_human.v3i.yolov8/runs/detect/yolo_model/weights/best.onnx'
        ),
        description='YOLO ONNX 모델 파일 경로'
    )

    # ※ 실제 터틀봇에서 ros2 topic list 로 확인 후 맞게 수정
    # 예시: /camera/image_raw/compressed
    #       /raspicam_node/image/compressed
    cam_a_topic_arg = DeclareLaunchArgument(
        'cam_a_topic',
        default_value='/camera/image_raw/compressed',
        description='TurtleBot3 camA 압축 이미지 토픽'
    )

    fan_offset_y_arg = DeclareLaunchArgument(
        'fan_offset_y',
        default_value='0.07',
        description='카메라 기준 선풍기 Y축 오프셋 (m)'
    )

    lidar_cam_offset_arg = DeclareLaunchArgument(
        'lidar_to_camera_offset',
        default_value='0.1',
        description='LiDAR와 카메라 간 전후 거리 (m)'
    )

    # ================================================================
    # 노드 정의
    # ================================================================

    # ── 1. robot_commander ──────────────────────────────────────────
    # 모드 관리 / 제스쳐 명령 / 자율주행 목표 / 사람 추종 타겟 발행
    # pub: current_mode, gesture_cmd, auto_drive_goal, follow_target_pose
    # sub: /scan, gesture_data, human_offset, call_position, nav2_status
    robot_commander_node = Node(
        package='handsome_pkg',
        executable='robot_commander',
        name='robot_commander_node',
        output='screen',
        emulate_tty=True,
    )

    # ── 2. robot_move ───────────────────────────────────────────────
    # 자율주행(Theta* + TEB) / 제스쳐 속도 중계 / 사람 추종 이동
    # pub: /cmd_vel, nav2_status
    # sub: current_mode, gesture_cmd, auto_drive_goal, follow_target_pose,
    #      /odom, /map (transient_local QoS), /scan
    robot_move_node = Node(
        package='handsome_pkg',
        executable='robot_move',
        name='robot_move_node',
        output='screen',
        emulate_tty=True,
    )

    # ── 3. robot_vision ─────────────────────────────────────────────
    # YOLO 추론 / camA(TurtleBot3) + camB(PC 웹캠) 처리
    # pub: human_offset, yolo_zone, gesture_data
    # sub: current_mode, /camera/image_raw/compressed (camA)
    robot_vision_node = Node(
        package='handsome_pkg',
        executable='robot_vision',
        name='robot_vision_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
        }],
        # camA 토픽이 로봇마다 다를 수 있으므로 remapping으로 유연하게 처리
        remappings=[
            ('/camera/image_raw/compressed', LaunchConfiguration('cam_a_topic')),
        ]
    )

    # ── 4. smart_fan ────────────────────────────────────────────────
    # YOLO zone + LiDAR 거리로 선풍기 회전 각도 계산
    # pub: fan_angle, /set_joint_trajectory
    # sub: current_mode, yolo_zone, /scan
    smart_fan_node = Node(
        package='handsome_pkg',
        executable='smart_fan',
        name='smart_fan_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'fan_offset_y':           LaunchConfiguration('fan_offset_y'),
            'lidar_to_camera_offset': LaunchConfiguration('lidar_to_camera_offset'),
        }]
    )

    # ================================================================
    # LaunchDescription 조합
    # ================================================================
    return LaunchDescription([
        # Launch Arguments
        model_path_arg,
        cam_a_topic_arg,
        fan_offset_y_arg,
        lidar_cam_offset_arg,

        # 시작 로그
        LogInfo(msg='========================================'),
        LogInfo(msg='  handsome_robot launch 시작'),
        LogInfo(msg='========================================'),

        # 노드 실행
        robot_commander_node,
        robot_move_node,
        robot_vision_node,
        smart_fan_node,
    ])