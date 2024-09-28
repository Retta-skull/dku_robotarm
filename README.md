# RobotArm ROS2 패키지

## 목차

- [소개](#소개)
- [기능](#기능)
- [아키텍처](#아키텍처)
- [필수 조건](#필수-조건)
- [설치](#설치)
  - [패키지 클론하기](#1-패키지-클론하기)
  - [의존성 설치](#2-의존성-설치)
  - [Python 패키지 설치](#3-python-패키지-설치)
  - [작업공간 빌드하기](#4-작업공간-빌드하기)
  - [설정 파일 소싱하기](#5-설정-파일-소싱하기)
- [사용법](#사용법)
  - [패키지 빌드하기](#패키지-빌드하기)
  - [RobotArm 실행하기](#robotarm-실행하기)
- [노드 개요](#노드-개요)
- [구성](#구성)
  - [URDF 파일](#urdf-파일)
  - [RViz 설정](#rviz-설정)
- [라이선스](#라이선스)

## 소개

RobotArm은 다중 조인트를 가진 로봇 팔을 제어하기 위해 설계된 ROS2 패키지입니다. 이 패키지는 역기구학(Inverse Kinematics)을 활용하여 원하는 엔드 이펙터 위치에 도달하기 위한 조인트 각도를 계산하고, 조인트 상태를 퍼블리시하며, RViz에서 로봇 모델을 시각화합니다. 시뮬레이션 및 실제 환경 모두에서 사용할 수 있습니다.

## 기능

- **역기구학 계산**: 원하는 엔드 이펙터 위치를 달성하기 위한 조인트 각도 계산.
- **조인트 상태 퍼블리셔**: 현재 모든 조인트의 상태를 `/joint_states` 토픽으로 퍼블리시.
- **마커 퍼블리셔**: RViz에서 목표 위치를 마커로 시각화.
- **RViz 통합**: 로봇 및 움직임을 시각화하기 위한 포괄적인 RViz 설정 제공.
- **모듈화된 아키텍처**: 입력 처리, 역기구학 계산, 상태 퍼블리싱, 시각화를 위한 개별 노드 구성.

## 아키텍처

시스템은 다음과 같은 주요 구성 요소로 이루어져 있습니다:

- **XYZ Publisher**: 목표 위치에 대한 사용자 입력을 처리.
- **Angle Setter**: 역기구학을 계산하고 조인트 각도를 퍼블리시.
- **Marker Publisher**: RViz에서 목표 위치를 시각화.
- **Joint State Publisher**: 로봇의 현재 조인트 상태를 퍼블리시.
- **Robot State Publisher**: URDF 모델을 기반으로 로봇의 상태를 퍼블리시.
- **RViz**: 로봇의 상태와 움직임을 모니터링하기 위한 시각적 인터페이스 제공.

## 필수 조건

`robotarm` 패키지를 설치하기 전에 다음이 설치되어 있는지 확인하세요:

- **Unbuntu 24.04**
- **ROS2**: [ROS2 JAZZY](https://docs.ros.org/en/jazzy/) (사용 중인 버전에 따라 조정).
- **Python 3.8 이상**
- **필요한 Python 패키지**:
  - requirements.txt 참고
    
## 설치

`robotarm` 패키지를 설치하고 설정하려면 다음 단계를 따르세요.

### 1. 패키지 클론하기

ROS2 작업공간의 `src` 디렉토리로 이동하여 저장소를 클론합니다.

```bash
cd ~/ros2_ws/src
git clone https://github.com/Retta-skull/dku_robotarm.git
```

### 2. 의존성 설치
작업공간의 루트로 이동하여 필요한 의존성을 설치합니다.

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Python 패키지 설치
필요한 Python 패키지를 설치합니다. 

```bash
pip install -r src/robotarm/requirements.txt
```

### 4. 작업공간 빌드하기
ROS2 작업공간을 colcon을 사용하여 빌드합니다.

```bash
colcon build --packages-select robotarm
```

### 5. 설정 파일 소싱하기
성공적으로 빌드한 후, 설정 파일을 소싱하여 작업공간을 오버레이합니다.

```bash
source install/setup.bash
```

## 사용법

### 패키지 빌드하기
설치 단계를 완료한 후, 소스 코드에 변경 사항이 있는 경우 작업공간을 다시 빌드합니다.

```bash
cd ~/ros2_ws
colcon build --packages-select robotarm
source install/setup.bash
```

### RobotArm 실행하기
제공된 런치 파일을 사용하여 모든 필요한 노드를 시작하고 RViz에서 로봇을 시각화합니다.

```bash
ros2 launch robotarm display_launch.py
ros2 launch robotarm control_robotarm_.py
ros2 run robotarm Interface.py
```

## 노드 개요

## 1. XYZ Publisher (xyz_publisher.py)

기능: 사용자로부터 목표 XYZ 좌표를 입력받아 /target_position 토픽으로 퍼블리시.

메시지 타입: geometry_msgs/Point 또는 std_msgs/Float32MultiArray (구현에 따라 다름).

## 2. Angle Setter (angle_setter.py)

기능: /target_position을 구독하여 역기구학을 계산하고, /joint_angles로 조인트 각도를 퍼블리시.

메시지 타입: std_msgs/Float32MultiArray

## 3. Marker Publisher (marker_publisher.py)

기능: /target_position을 구독하여 RViz에서 목표 위치를 마커로 시각화.

메시지 타입: visualization_msgs/Marker

## 4. Joint State Publisher (JointStatePublisher.py)

기능: 모든 조인트의 현재 상태를 /joint_states 토픽으로 퍼블리시.

메시지 타입: sensor_msgs/JointState

## 5. Application RobotArm (ApplicationRobotarm.py)

기능: 조인트의 상태를 실제 로봇팔에 적용.


## 구성

### URDF 파일

위치: urdf/robotarm.urdf
설명: 로봇의 물리적 설명을 포함하며, 링크와 조인트에 대한 정보를 담고 있습니다.
사용법: robot_state_publisher 노드에서 로봇을 시각화하고, 역기구학 계산에 사용됩니다.

### RViz 설정

위치: rviz/robotarm.rviz
설명: 로봇 모델, 조인트 상태, 마커 등을 시각화하기 위한 사전 설정된 RViz 구성 파일.
사용법: 런치 파일을 통해 RViz에서 일관된 시각화 설정을 제공합니다.

## 라이선스
이 프로젝트는 Apache License 2.0에 따라 라이선스가 부여되었습니다.

