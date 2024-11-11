# RobotArm ROS2 패키지
## 목차

- [소개](#소개)
- [기능](#기능)
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
- [구성](#구성)
  - [URDF 파일](#urdf-파일)
  - [RViz 설정](#rviz-설정)
- [라이선스](#라이선스)
- [문의](#문의)

## 소개

이 프로젝트는 2024 단국대학교 종합설계 졸업작품입니다.

GPT API를 이용하여 음성 명령으로 로봇팔을 제어하고, 사용자와의 자연스러운 음성 대화를 통해 로봇의 조작을 간편하게 만드는 것이 최종적인 목적입니다.

## 기능

- **음성 대화 제어**: GPT API를 사용하여 음성 명령을 입력받아 로봇팔을 제어하는 기능.
- **역기구학 계산**: 원하는 엔드 이펙터 위치를 달성하기 위한 조인트 각도 계산.
- **조인트 상태 퍼블리셔**: 현재 모든 조인트의 상태를 /joint_states 토픽으로 퍼블리시하여 시뮬레이터 및 실제 서보모터의 각도에 적용.
- **마커 퍼블리셔**: RViz에서 목표 위치를 마커로 시각화.
- **RViz 통합**: 로봇 및 움직임을 시각화하기 위한 포괄적인 RViz 설정 제공.
- **모듈화된 아키텍처**: 입력 처리, 역기구학 계산, 상태 퍼블리싱, 시각화를 위한 개별 노드 구성.

## 필수 조건

robotarm 패키지를 설치하기 전에 다음이 설치되어 있는지 확인하세요:

- **Unbuntu 24.04**
- **ROS2**: [ROS2 JAZZY](https://docs.ros.org/en/jazzy/)
- **Python 3.8 이상**
- **필요한 Python 패키지**:
  - requirements.txt 참고
    
## 설치

robotarm 패키지를 설치하고 설정하려면 다음 단계를 따르세요.

### 1. 패키지 클론하기

ROS2 작업공간의 src 디렉토리로 이동하여 저장소를 클론합니다.

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
pip install -r src/dku_robotarm/requirements.txt
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
ros2 launch dku_robotarm display_launch.py
ros2 launch dku_robotarm control_launch.py
ros2 run dku_robotarm Interface.py
```

![image](https://github.com/user-attachments/assets/8818d1a9-366c-4d31-9cab-e2768513642d)


## 구성

### Mesh

- **위치**: mesh/
- **설명**: 로봇의 각 링크에 대한 3D 모델을 포함하고 있으며, RViz 시각화 및 물리적 시뮬레이션에 사용됩니다.
- **사용법**: 로봇의 시각적 표현을 개선하기 위해 URDF 파일에서 참조됩니다.

### Launch

- **위치**: launch/
- **설명**: ROS2 노드를 시작하기 위한 다양한 런치 파일을 포함하고 있습니다. 이 파일들은 로봇팔의 제어와 시각화를 쉽게 실행할 수 있도록 도와줍니다.
  - **display_launch.py**: RViz에서 로봇팔의 상태와 모델을 시각화하기 위한 런치 파일입니다.
  - **control_launch.py**: 로봇팔의 제어 노드를 실행하기 위한 런치 파일로, 로봇팔을 실제로 움직이기 위한 제어 명령을 보냅니다.
- **사용법**: ROS2 런치 명령을 통해 여러 노드를 동시에 실행하는 데 사용됩니다.

### Src

- **위치**: src/
- **설명**: 로봇팔 제어 및 음성 대화 기능 구현을 위한 Python 코드와 ROS2 노드를 포함하고 있습니다.
- **사용법**: ROS2 환경에서 빌드되어 각 노드가 실행됩니다.

### URDF 파일

- **위치**: urdf/dku_robotarm.urdf
- **설명**: 로봇의 물리적 설명을 포함하며, 링크와 조인트에 대한 정보를 담고 있습니다.
- **사용법**: robot_state_publisher 노드에서 로봇을 시각화하고, 역기구학 계산에 사용됩니다.

### RViz 설정

- **위치**: rviz/dku_robotarm.rviz
- **설명**: 로봇 모델, 조인트 상태, 마커 등을 시각화하기 위한 사전 설정된 RViz 구성 파일.
- **사용법**: 런치 파일을 통해 RViz에서 일관된 시각화 설정을 제공합니다.

## 문의

- **메일**: burutal@naver.com

## 라이선스

이 프로젝트는 Apache License 2.0에 따라 라이선스가 부여되었습니다.
