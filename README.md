## 프로젝트 개요
> 손 제스쳐 인식 기반 원격 동작 및 대상 추종 로봇 prototype

## Teck Stack
- **Language** : C++
- **Framework** : ROS2 (humble)
- **Node** : smart_fan, robot_commander

## node별 주요 기능
- smart_fan : 카메라와 선풍기, 사용자 위치를 기준으로 삼각함수를 계산하여 선풍기 방향 조절
- robot_commander : 제스쳐 제어 모드, 자율 주행 모드, 추종 모드, 대기 모드
- auto_drive : 모드별 로봇 제어 담당, Theta* + TEB 알고리즘 기반 자율주행