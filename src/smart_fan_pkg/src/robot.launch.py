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
#
# AMCL 파라미터 파일 직접 지정:
#   ros2 launch handsome_pkg handsome_robot.launch.py amcl_params:=/your/amcl_params.yaml
#
# 런치 실행할 때 :
# ros2 launch handsome_pkg handsome_robot.launch.py amcl_params:=''
# ──────────────────────────────────────────────────────────────────
# [변경] AMCL 노드 추가
#   - /scan + /map 을 입력받아 /amcl_pose (map 프레임 기준 위치 추정) 퍼블리시
#   - robot_move 노드가 /odom 대신 /amcl_pose 를 구독하도록 변경됨에 따라 추가
#   - 초기 위치는 RViz "2D Pose Estimate" 버튼 → /initialpose 토픽으로 설정
# ──────────────────────────────────────────────────────────────────

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ================================================================
    # Launch Arguments
    # ================================================================

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(
            '/home/handsome/handsome_ws/src/handsome_pkg',
            'tired_human.v3i.yolov8/runs/detect/yolo_model/weights/best.onnx'
        ),
        description='YOLO ONNX 모델 파일 경로'
    )

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

    # [추가] AMCL 파라미터 yaml 파일 경로 인자
    # 별도 yaml 없이 기본값으로 실행하려면 빈 문자열 유지
    amcl_params_arg = DeclareLaunchArgument(
        'amcl_params',
        default_value=os.path.join(
            '/home/handsome/handsome_ws/src/handsome_pkg',
            'config/amcl_params.yaml'
        ),
        description='AMCL 파라미터 yaml 파일 경로'
    )

    # ================================================================
    # 노드 정의
    # ================================================================

    # ── 1. robot_commander ──────────────────────────────────────────
    # 모드 관리 / 제스쳐 명령 / 자율주행 목표 / 사람 추종 타겟 발행
    # pub: current_mode (500ms 주기 재퍼블리시), gesture_cmd,
    #      auto_drive_goal, follow_target_pose
    # sub: /scan, gesture_data, human_offset, call_position, nav2_status
    robot_commander_node = Node(
        package='handsome_pkg',
        executable='robot_commander',
        name='robot_commander_node',
        output='screen',
        emulate_tty=True,
    )

    # ── 2. robot_move ───────────────────────────────────────────────
    # 자율주행(Theta* + TEB) / 제스쳐 속도 중계
    # pub: /cmd_vel, nav2_status, global_path
    # sub: current_mode, gesture_cmd, auto_drive_goal, follow_target_pose,
    #      /amcl_pose (transient_local QoS),   ← [변경] /odom 에서 교체
    #      /map (transient_local QoS), /scan
    robot_move_node = Node(
        package='handsome_pkg',
        executable='robot_move',
        name='robot_move_node',
        output='screen',
        emulate_tty=True,
    )

    # ── 3. robot_vision ─────────────────────────────────────────────
    # YOLO 추론 / camA(TurtleBot3) + camB(PC 웹캠) 처리
    # pub: human_offset, yolo_zone, gesture_data,
    #      camA_display, camB_display
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

    # ── 5. AMCL ─────────────────────────────────────────────────────
    # [추가] LiDAR + 지도 기반 위치 추정 (Monte Carlo Localization)
    # pub : /amcl_pose (map 프레임 기준 로봇 위치, transient_local QoS)
    #       /particlecloud (RViz 파티클 시각화용)
    # sub : /scan, /map, /initialpose (RViz "2D Pose Estimate" 입력)
    # ※ 초기 위치는 반드시 RViz의 "2D Pose Estimate" 로 지정해야 함
    # ※ amcl_params.yaml 이 없으면 아래 inline parameters 기본값으로 동작
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('amcl_params'),  # yaml 파일 우선 적용
            {
                # yaml 파일이 없을 때 사용할 기본 파라미터
                'use_sim_time':            False,

                # LiDAR 토픽 (터틀봇3 기본값)
                'scan_topic':              '/scan',

                # 파티클 수 (실내 소규모 지도 기준 권장값)
                'min_particles':           500,
                'max_particles':           2000,

                # 로봇이 움직여야 파티클 업데이트 (너무 잦은 연산 방지)
                'update_min_d':            0.2,   # 이동 거리 임계값 (m)
                'update_min_a':            0.5,   # 회전 각도 임계값 (rad)

                # 레이저 모델 파라미터
                'laser_model_type':        'likelihood_field',
                'laser_max_range':         3.5,   # 터틀봇3 LiDAR 최대 범위
                'laser_min_range':         0.12,
                'max_beams':               60,

                # 오도메트리 모델 (터틀봇3 기본값)
                'odom_model_type':         'diff',
                'odom_alpha1':             0.2,
                'odom_alpha2':             0.2,
                'odom_alpha3':             0.2,
                'odom_alpha4':             0.2,

                # 프레임 설정
                'base_frame_id':           'base_footprint',
                'odom_frame_id':           'odom',
                'global_frame_id':         'map',

                # /amcl_pose 퍼블리시 주기 (초)
                'transform_tolerance':     0.5,

                # 초기 위치를 RViz 2D Pose Estimate 로 받을 때 필요
                'set_initial_pose':        False,
            }
        ],
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
        amcl_params_arg,           # [추가]

        # 시작 로그
        LogInfo(msg='========================================'),
        LogInfo(msg='  handsome_robot launch 시작'),
        LogInfo(msg='========================================'),

        # 노드 실행
        robot_commander_node,
        robot_move_node,
        robot_vision_node,
        smart_fan_node,
        amcl_node,                 # [추가]
    ])