# 워크스페이스 아키텍처 가이드

## 목적

이 문서는 이 ROS 2 워크스페이스의 구조를 초보자 관점에서 설명합니다.
특히 아래 내용을 중심으로 정리합니다.

- 패키지 간 관계
- 런치 시점의 실행 흐름
- `ros2_control` 구조
- `arm_sdk`, `mobile_sdk`, `gripper_sdk` 내부 구조
- MoveIt, Nav2, YOLO, 웹 UI가 서로 어떻게 연결되는지

이 워크스페이스는 "서로 관련 없는 여러 앱이 한 저장소에 들어 있는 구조"에 가깝지 않습니다.
오히려 "하나의 ROS 2 로봇 시스템을 역할별 패키지로 나눠 놓은 구조"에 가깝습니다.

## 1. 최상위 구조

워크스페이스 수준에서 보면 패키지는 대략 아래처럼 나뉩니다.

```text
frjoco_ws
├─ FrJoCo_bringup
│  ├─ 최상위 오케스트레이션 패키지
│  ├─ 전체 로봇 시스템을 조립하는 launch 파일들
│  └─ pick-and-place 스크립트
├─ arm_sdk
│  ├─ 순수 C++ 기반 arm native SDK
│  └─ arm용 ROS 2 ros2_control 어댑터
├─ mobile_sdk
│  ├─ 순수 C++ 기반 mobile base native SDK
│  └─ mobile base용 ROS 2 ros2_control 어댑터
├─ gripper_sdk
│  ├─ gripper native 로직
│  └─ gripper용 ROS 2 Python 런타임 노드
├─ moveit/mobile_manipulator_moveit_config
│  └─ MoveIt 설정 패키지
├─ navigation/robot_nav2
│  └─ Nav2 설정 패키지
├─ navigation/robot_slam
│  └─ SLAM 패키지
├─ robot_web_interface
│  └─ 브라우저 기반 제어 UI
├─ yolo_realsense
│  └─ YOLO + RealSense 인식 노드
└─ FrJoCo_model
   └─ 로봇 모델 에셋 저장소, 이 워크스페이스의 런타임 중심은 아님
```

## 2. 패키지 관계도

이 워크스페이스를 이해할 때 가장 유용한 정신 모델은 아래와 같습니다.

```text
FrJoCo_bringup
├─ mobile_sdk 실행
├─ arm_sdk 실행
├─ gripper_sdk 실행
├─ MoveIt 실행
├─ EKF + Nav2 실행
├─ RealSense + YOLO 실행
├─ 웹 인터페이스 실행
└─ topic_pick_place.py 실행

MoveIt
├─ arm motion 계획
├─ arm_controller로 FollowJointTrajectory goal 전송
└─ gripper action server로 GripperCommand goal 전송

Nav2
├─ /cmd_vel 출력
└─ relay가 이를 mobile diff_drive_controller로 전달

YOLO + RealSense
├─ target point 발행
└─ topic_pick_place.py가 이를 구독

Web UI
├─ teleop 명령 발행
├─ execute_pick_place 서비스 호출
└─ joint state, 카메라 스트림, 상태 메시지 모니터링
```

실행 시점 관계를 조금 더 직접적으로 그리면 아래와 같습니다.

```text
Browser
  -> robot_web_interface
  -> rosbridge/web_video_server
  -> ROS topics/services

YOLO camera pipeline
  -> yolo_realsense
  -> /can_target_point, /box_target_point, /phone_target_point
  -> topic_pick_place.py

topic_pick_place.py
  -> MoveIt services/actions
  -> arm_controller follow_joint_trajectory
  -> gripper_controller gripper_cmd

Nav2
  -> /cmd_vel
  -> topic relay
  -> /mobile/diff_drive_controller/cmd_vel_unstamped

arm_controller
  -> ArmSystemInterface
  -> ArmDriver
  -> USBCAN + RMD motors

diff_drive_controller
  -> MobileSystemInterface
  -> MDDriver
  -> serial + MD200T motor drivers
```

## 3. 실제 진입점은 어떤 패키지인가

이 워크스페이스를 이해할 때 실제 진입점 역할을 하는 패키지는 `FrJoCo_bringup`입니다.

핵심 파일:

- `FrJoCo_bringup/package.xml`
- `FrJoCo_bringup/launch/full_system.launch.py`
- `FrJoCo_bringup/launch/moveit.launch.py`
- `FrJoCo_bringup/launch/navigation.launch.py`
- `FrJoCo_bringup/launch/sensors.launch.py`
- `FrJoCo_bringup/launch/full_system_pickplace.launch.py`

처음 하나의 패키지만 먼저 읽는다면 `FrJoCo_bringup`을 읽는 게 가장 좋습니다.
이 패키지가 전체 로봇 시스템을 어떻게 조립하는지 보여주기 때문입니다.

## 4. Launch 흐름

### 4.1 `full_system.launch.py`

역할:

- mobile base 시작
- arm 시작
- gripper 시작

즉 이 파일은 "하드웨어 시스템 조립층"입니다.

```text
full_system.launch.py
├─ include mobile_sdk/launch/mobile.launch.py
└─ include frjoco_bringup/launch/manipulator.launch.py
   ├─ include arm_sdk/launch/arm.launch.py
   └─ include gripper_sdk/launch/gripper.launch.py
```

### 4.2 `moveit.launch.py`

역할:

- `full_system.launch.py` 포함
- 그 다음 `move_group` 시작
- 그 다음 선택적으로 RViz 시작

즉 의미상 다음과 같습니다.

```text
먼저 하드웨어 시스템
-> 그 위에 planning 계층
-> 그 위에 시각화
```

### 4.3 `navigation.launch.py`

역할:

- `robot_localization` 기반 EKF 시작
- `/cmd_vel` relay 시작
- Nav2 노드들 시작

이 파일은 arm 쪽이 아니라 navigation stack입니다.

### 4.4 `sensors.launch.py`

역할:

- D405 gripper camera 시작
- D455 navigation camera 시작
- static TF bridge 발행
- 카메라 워밍업 이후 YOLO 노드 시작

즉 perception 쪽 묶음입니다.

### 4.5 `full_system_pickplace.launch.py`

이 파일은 가장 완전한 orchestration 파일입니다.

실질적으로 다음을 수행합니다.

```text
1. MoveIt stack
2. Navigation stack
3. Sensor stack
4. Web interface
5. Pick-and-place controller
```

초보자 기준으로는 이 파일이 워크스페이스 전체를 한눈에 보는 데 가장 좋은 파일입니다.

## 5. 초보자용 권장 읽기 순서

아래 순서로 읽는 것을 권장합니다.

1. `FrJoCo_bringup/package.xml`
2. `FrJoCo_bringup/launch/*.launch.py`
3. `arm_sdk/ros2/launch/*` 와 `mobile_sdk/ros2/launch/*`
4. `arm_sdk/ros2/urdf/*` 와 `mobile_sdk/ros2/urdf/*`
5. `arm_sdk/ros2/config/*` 와 `mobile_sdk/ros2/config/*`
6. `arm_sdk/ros2/src/arm_system_interface.cpp`
7. `mobile_sdk/ros2/src/mobile_system_interface.cpp`
8. `arm_sdk/C++`, `mobile_sdk/C++` 아래의 native C++ SDK 파일
9. `yolo_realsense`
10. `FrJoCo_bringup/scripts/topic_pick_place.py`

이 순서가 좋은 이유는 다음과 같습니다.

- `package.xml`은 패키지 책임을 보여줌
- `launch`는 런타임 조립 방식을 보여줌
- `urdf/xacro`는 로봇 구조와 하드웨어 선언을 보여줌
- `config`는 컨트롤러 동작을 보여줌
- `SystemInterface`는 ROS와 native SDK 사이 브리지를 보여줌
- native SDK는 실제 하드웨어 통신을 보여줌

## 6. 이 워크스페이스에서의 `ros2_control`

이 저장소를 이해하려면 아래 두 층의 분리를 이해해야 합니다.

- ROS 쪽 어댑터 계층
- native hardware SDK 계층

전체 패턴은 대략 아래와 같습니다.

```text
URDF/xacro
  -> <ros2_control> 하드웨어와 joint 선언
  -> controller_manager가 hardware plugin 로드
  -> controller가 command/state interface 사용
  -> SystemInterface가 native SDK 호출
  -> native SDK가 실제 하드웨어와 통신
```

좀 더 구체적으로 쓰면:

```text
xacro
  -> robot_description
  -> ros2_control_node
  -> controller_manager
  -> pluginlib가 SystemInterface 로드
  -> read()/write() 루프
  -> native driver API
  -> CAN 또는 serial 통신
```

이 패턴을 따르는 패키지:

- `arm_sdk`
- `mobile_sdk`

완전히 같은 방식은 아닌 패키지:

- `gripper_sdk`

이 비대칭 구조가 중요합니다.

## 7. `arm_sdk` 상세 구조

### 7.1 패키지 분리 방식

`arm_sdk`는 의도적으로 두 계층으로 나뉩니다.

```text
arm_sdk
├─ C++
│  ├─ 순수 C++ SDK
│  ├─ ROS 의존성 없음
│  ├─ USBCAN wrapper
│  ├─ RMD protocol 코드
│  └─ ArmDriver
└─ ros2
   ├─ ros2_control hardware plugin
   ├─ launch 파일
   ├─ URDF/xacro
   ├─ controller yaml
   └─ teleop node
```

이 분리는 `arm_sdk/ros2/CMakeLists.txt`에도 명시되어 있습니다.

### 7.2 arm 모델과 하드웨어 선언

주요 파일:

- `arm_sdk/ros2/urdf/arm_sdk.urdf.xacro`
- `arm_sdk/ros2/urdf/arm_only/manipulator_arm.xacro`
- `arm_sdk/ros2/urdf/ros2_control_arm_sdk.urdf.xacro`

핵심 개념:

- `manipulator_arm.xacro`는 링크와 조인트를 설명함
- `ros2_control_arm_sdk.urdf.xacro`는 그 조인트들에 대한 하드웨어 인터페이스를 설명함

arm의 주요 joint는 다음과 같습니다.

- `link2_to_link1`
- `link3_to_link2`
- `link4_to_link3`
- `gripper_to_link4`

arm 모델 안에는 gripper finger joint도 있습니다.

- `gripper_left_joint`
- `gripper_right_joint`

하지만 arm의 `ros2_control` plugin은 finger runtime이 아니라 4개의 arm joint를 제어합니다.

### 7.3 arm `ros2_control` 선언

`ros2_control_arm_sdk.urdf.xacro` 안의 hardware block은 다음을 의미합니다.

- plugin으로 `arm_sdk/ArmSystemInterface` 사용
- CAN 및 motor 설정을 hardware param으로 전달
- 각 joint에 대해 command/state interface 노출

실제 하드웨어일 때:

- command interfaces: `position`, `velocity`, `effort`
- state interfaces: `position`, `velocity`, `effort`

mock 하드웨어일 때:

- `mock_components/GenericSystem` 사용

즉 URDF는 단순히 형상만 설명하는 파일이 아닙니다.
ROS가 하드웨어를 어떻게 바라볼지도 함께 정의합니다.

### 7.4 arm launch 흐름

`arm.launch.py`는 다음 순서로 동작합니다.

```text
1. xacro로 robot_description 생성
2. robot_state_publisher 시작
3. ros2_control_node 시작
4. joint_state_broadcaster spawn
5. arm_controller spawn
6. 선택적으로 RViz 시작
```

이것이 표준적인 `ros2_control` bootstrap 순서입니다.

### 7.5 arm controller 계층

`arm_sdk/ros2/config/arm_controllers.yaml`에는 아래가 정의되어 있습니다.

- `joint_state_broadcaster`
- `arm_controller` as `JointTrajectoryController`

arm controller는 아래 인터페이스를 사용합니다.

- command interface: `position`
- state interfaces: `position`, `velocity`

즉 의미상:

- MoveIt이나 teleop가 원하는 joint trajectory를 보냄
- controller가 이를 시간에 따라 position 명령으로 변환함
- hardware plugin이 이를 native driver 쪽으로 기록함

### 7.6 arm 런타임 제어 경로

실제 런타임 경로는 다음과 같습니다.

```text
MoveIt 또는 arm_teleop
  -> /arm_controller/follow_joint_trajectory 또는 /arm_controller/joint_trajectory
  -> JointTrajectoryController
  -> ArmSystemInterface command buffers
  -> ArmDriver
  -> RMDProtocol
  -> USBCANDevice
  -> CAN bus
  -> RMD motors
```

### 7.7 `ArmSystemInterface`가 실제로 하는 일

파일:

- `arm_sdk/ros2/src/arm_system_interface.cpp`

개념적으로는 매우 얇은 브리지입니다.

```text
ROS 2 world <-> ArmSystemInterface <-> ArmDriver <-> CAN hardware
```

주요 책임:

1. `HardwareInfo` 파싱
2. hardware param을 `ArmDriverConfig`로 변환
3. `ArmDriver` 생성 및 configure
4. command/state interface export
5. `read()`에서 `driver_->readAll()` 호출
6. `write()`에서 `driver_->writeAll()` 호출
7. auto-home, ready pose 같은 lifecycle 동작 관리

초보자 관점에서 중요한 점:

`ArmSystemInterface` 안에는 실제 CAN 프로토콜 로직이 거의 없습니다.
주된 역할은 ROS 2 lifecycle과 interface를 native driver API에 연결하는 것입니다.

### 7.8 `ArmDriver`가 실제로 하는 일

파일:

- `arm_sdk/C++/include/arm_sdk/arm_driver.hpp`
- `arm_sdk/C++/src/arm_driver.cpp`

주요 책임:

- `libusbcan.so` 로드
- USBCAN device open
- CAN channel 초기화
- 각 motor와 통신 테스트
- motor state 읽기
- motor command 쓰기
- acceleration 설정
- joint별 command/state buffer 유지

### 7.9 arm 저수준 계층

저수준 파일:

- `usbcan_device.hpp`
- `rmd_protocol.hpp`
- `arm_driver.hpp`

각 역할:

```text
USBCANDevice
  -> raw USB-to-CAN device wrapper

RMDProtocol
  -> RMD motor CAN frame 생성/파싱

ArmDriver
  -> 여러 joint를 묶어 관리하는 상위 coordination 계층
```

즉:

- `USBCANDevice`는 CAN frame을 주고받는 법을 안다
- `RMDProtocol`은 그 frame 안에 어떤 바이트를 넣어야 하는지 안다
- `ArmDriver`는 어떤 motor/joint에 어떤 명령을 보낼지 안다

### 7.10 arm read/write 루프

controller가 동작하는 동안:

```text
controller_manager cycle
  -> ArmSystemInterface.read()
  -> ArmDriver.readAll()
  -> 각 motor 상태가 position/velocity/effort에 반영됨

controller가 새 명령 계산
  -> ArmSystemInterface.write()
  -> ArmDriver.writeAll()
  -> joint별 command가 CAN으로 전송됨
```

이 루프가 arm에서의 `ros2_control` 핵심입니다.

## 8. `mobile_sdk` 상세 구조

### 8.1 패키지 분리 방식

`mobile_sdk`도 같은 구조를 따릅니다.

```text
mobile_sdk
├─ C++
│  ├─ 순수 C++ SDK
│  ├─ SerialPort
│  ├─ MDProtocol
│  └─ MDDriver
└─ ros2
   ├─ MobileSystemInterface
   ├─ launch
   ├─ URDF/xacro
   └─ diff_drive controller config
```

### 8.2 mobile robot model

주요 파일:

- `mobile_sdk/ros2/urdf/mobile_sdk.urdf.xacro`
- `mobile_sdk/ros2/urdf/ros2_control_mobile_sdk.urdf.xacro`

mobile model은 아래를 정의합니다.

- `base_footprint`
- `base_link`
- 4개의 wheel link
- 4개의 wheel joint

wheel joint:

- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

### 8.3 mobile `ros2_control` 선언

xacro 안에서는:

- 실제 하드웨어 plugin: `mobile_sdk/MobileSystemInterface`
- mock 하드웨어 plugin: `mock_components/GenericSystem`

mobile base는 아래 인터페이스를 노출합니다.

- command interface: `velocity`
- state interfaces: `position`, `velocity`

이 부분이 arm과 가장 큰 차이입니다.

왜냐하면:

- mobile base는 wheel velocity를 기준으로 구동되고
- odometry는 wheel state를 바탕으로 다시 계산되기 때문입니다

### 8.4 mobile launch 흐름

`mobile.launch.py`는 다음 순서로 실행됩니다.

```text
1. xacro로 robot_description 생성
2. robot_state_publisher 시작
3. ros2_control_node 시작
4. joint_state_broadcaster spawn
5. diff_drive_controller spawn
```

따라서 mobile controller 경로는:

```text
cmd_vel
  -> diff_drive_controller
  -> wheel velocity commands
  -> MobileSystemInterface
  -> MDDriver
  -> serial packet으로 motor driver에 전달
```

### 8.5 mobile controller 계층

파일:

- `mobile_sdk/ros2/config/diff_drive_controller.yaml`

이 설정은 아래를 정의합니다.

- left/right wheel group
- wheel radius, wheel separation
- odometry frame id
- velocity, acceleration limit
- command timeout

즉 mobile base의 "기구학 설정"이 이 파일에 명시됩니다.

### 8.6 `MobileSystemInterface`가 실제로 하는 일

파일:

- `mobile_sdk/ros2/src/mobile_system_interface.cpp`

주요 책임:

1. `HardwareInfo`에서 hardware param 파싱
2. 정확히 4개의 wheel joint가 있는지 검증
3. native `MDDriver` 생성
4. wheel command/state interface export
5. `read()`에서 `driver_->readState()` 호출
6. `write()`에서 `driver_->writeVelocity()` 호출

여기서도 핵심은 같습니다.

`MobileSystemInterface`는 실제 driver라기보다 브리지입니다.

### 8.7 `MDDriver`가 실제로 하는 일

파일:

- `mobile_sdk/C++/include/mobile_sdk/md_driver.hpp`
- `mobile_sdk/C++/src/md_driver.cpp`

주요 책임:

- 두 개의 serial port open
- front/rear motor driver와 통신
- rad/s를 RPM으로 변환
- velocity packet 전송
- motor packet 수신
- wheel position, velocity 갱신
- encoder tick을 누적해 wheel angle 계산

즉 실제 mobile hardware runtime은 여기에 있습니다.

### 8.8 mobile 저수준 계층

파일:

- `serial_port.hpp`
- `md_protocol.hpp`
- `md_driver.hpp`

각 역할:

```text
SerialPort
  -> raw POSIX serial wrapper

MDProtocol
  -> packet format, parser, conversion helper

MDDriver
  -> 2개의 MD200T driver를 통해 4륜을 묶어 제어하는 계층
```

### 8.9 mobile read/write 루프

런타임에서는:

```text
controller_manager cycle
  -> MobileSystemInterface.read()
  -> MDDriver.readState()
  -> wheel position/velocity 갱신

diff_drive_controller가 새 wheel command 계산
  -> MobileSystemInterface.write()
  -> MDDriver.writeVelocity(fl, fr, rl, rr)
  -> serial packet 전송
```

이것이 mobile base에서의 전체 `ros2_control` 루프입니다.

## 9. 왜 `gripper_sdk`는 다르게 생겼는가

이 부분은 이 워크스페이스에서 가장 중요한 구조적 차이 중 하나입니다.

`gripper_sdk`는 `arm_sdk`, `mobile_sdk`와 똑같이 구성되어 있지 않습니다.

즉, 같은 스타일의 `ros2_control` `SystemInterface` plugin을 중심으로 동작하지 않습니다.
대신 런타임 중심은 아래입니다.

- `gripper_node.py`
- `gripper_action_server.py`

### 9.1 gripper 런타임 그림

```text
MoveIt 또는 web UI
  -> GripperCommand action
  -> gripper_action_server.py
  -> cmd_position topic
  -> gripper_node.py
  -> native gripper runtime
```

### 9.2 `gripper_node.py`

주요 책임:

- 실제 gripper 또는 mock gripper 객체 생성
- gripper state 발행
- position/current command 수신
- enable/disable, mode service 제공

즉 `ros2_control` hardware plugin이 아니라 직접적인 ROS node 인터페이스입니다.

### 9.3 `gripper_action_server.py`

주요 책임:

- MoveIt 호환용 `GripperCommand` action 제공
- 정규화된 open/close 명령을 motor별 encoder 목표값으로 변환
- feedback/result 발행

즉 gripper는 MoveIt이 이해할 수 있는 action server 형태로 감싸져 있습니다.

### 9.4 왜 이 차이가 중요한가

초보자들이 자주 하는 착각:

```text
arm, mobile, gripper가 모두 같은 방식으로 ros2_control에 붙어 있다
```

하지만 이 저장소에서는 그렇지 않습니다.

실제 구조는:

```text
arm     -> ros2_control SystemInterface
mobile  -> ros2_control SystemInterface
gripper -> standalone ROS node + action server
```

## 10. 이 워크스페이스에서의 MoveIt 구조

주요 패키지:

- `moveit/mobile_manipulator_moveit_config`

이 패키지에는 다음이 들어 있습니다.

- SRDF group 정의
- controller mapping
- kinematics 설정
- planning 설정
- RViz 설정

### 10.1 MoveIt group

SRDF 안에서:

- `arm` group은 4개의 arm joint 사용
- `gripper` group은 finger joint 사용
- end effector는 `end_effector_link` 사용

### 10.2 MoveIt controller mapping

`config/moveit_controllers.yaml`에는:

- `arm_controller` -> `FollowJointTrajectory`
- `gripper_controller` -> `GripperCommand`

즉 MoveIt은 CAN이나 serial을 직접 알지 않습니다.
MoveIt이 아는 것은 controller와 action뿐입니다.

### 10.3 MoveIt 런타임 경로

```text
MoveIt planning
  -> /move_action 및 execution service
  -> arm_controller follow_joint_trajectory
  -> ArmSystemInterface
  -> ArmDriver

MoveIt gripper execution
  -> /gripper_controller/gripper_cmd
  -> gripper_action_server
  -> gripper_node
```

## 11. Navigation 구조

관련 패키지:

- `navigation/robot_nav2`
- `navigation/robot_slam`

`FrJoCo_bringup/launch/navigation.launch.py`는 아래를 묶습니다.

- `robot_localization` 기반 EKF
- Nav2 노드들
- `/cmd_vel` relay

초보자 관점에서 중요한 포인트:

Nav2는 하드웨어를 직접 움직이지 않습니다.
Nav2는 navigation velocity command만 내보냅니다.
그리고 그 명령이 relay를 통해 mobile controller topic으로 전달됩니다.

즉:

```text
Nav2
  -> /cmd_vel
  -> topic relay
  -> mobile diff_drive_controller command topic
  -> MobileSystemInterface
  -> MDDriver
```

## 12. Perception 구조

perception 쪽 중심은 아래입니다.

- `FrJoCo_bringup/launch/sensors.launch.py`
- `yolo_realsense/yolo_realsense/yolo_publisher_node.py`

런타임 그림:

```text
D405 camera
  -> RGB + depth + camera_info
  -> yolo_realsense
  -> object target point

D455 camera
  -> navigation camera 및 IMU data
  -> navigation/sensing 쪽에서 사용
```

YOLO node는 아래 토픽을 발행합니다.

- `can_target_point`
- `box_target_point`
- `phone_target_point`

이 토픽들은 pick-and-place 노드가 구독합니다.

## 13. Pick-and-Place 구조

주요 파일:

- `FrJoCo_bringup/scripts/topic_pick_place.py`

이 노드는 "전체 시스템 관점"에서 가장 공부하기 좋은 파일 중 하나입니다.

왜냐하면 다음을 모두 건드리기 때문입니다.

- TF transform
- MoveIt planning
- trajectory execution
- gripper action
- YOLO target 입력
- 웹에서 트리거되는 실행 흐름

런타임 그림:

```text
YOLO target topic
  -> topic_pick_place.py
  -> camera frame에서 base frame으로 변환
  -> MoveIt으로 계획
  -> arm trajectory 실행
  -> gripper close/open 명령
  -> status 발행
```

즉 이 파일은 여러 패키지가 어떻게 하나의 동작으로 이어지는지 보여줍니다.

## 14. Web Interface 구조

패키지:

- `robot_web_interface`

이 패키지는 무거운 프론트엔드 빌드 시스템이 아니라 비교적 단순한 구조입니다.

- 정적 웹 파일
- `rosbridge_server`
- `web_video_server`
- 간단한 HTTP server

즉 브라우저는 websocket과 topic/service/action API를 통해 ROS와 통신합니다.

이 패키지는 바깥쪽 제어 관점에서 전체 시스템을 이해하는 데 유용합니다.

- base teleop
- arm teleop
- gripper 제어
- camera view
- pick-and-place 실행

## 15. `arm_teleop.py`와 MoveIt의 차이

이 부분도 초보자들이 자주 헷갈립니다.

arm은 최소 두 가지 경로로 움직일 수 있습니다.

### 15.1 teleop 경로

```text
/arm_teleop/joint_delta 또는 /arm_teleop/joint_cmd
  -> arm_teleop.py
  -> /arm_controller/joint_trajectory
  -> JointTrajectoryController
  -> ArmSystemInterface
```

### 15.2 MoveIt 경로

```text
MoveIt
  -> /arm_controller/follow_joint_trajectory
  -> JointTrajectoryController
  -> ArmSystemInterface
```

즉 상위 입력은 다르지만, 결국 같은 arm controller로 수렴합니다.

## 16. `FrJoCo_model`은 런타임 중심이 아니다

`FrJoCo_model`은 로봇 모델과 에셋을 저장합니다.

중요한 용도:

- URDF
- MJCF
- mesh resource

하지만 "현재 워크스페이스가 어떻게 돌아가는지"를 이해할 때는 우선순위가 약간 낮습니다.

런타임 이해를 위해 먼저 봐야 할 것은:

- `FrJoCo_bringup`
- `arm_sdk`
- `mobile_sdk`
- `gripper_sdk`
- `moveit/mobile_manipulator_moveit_config`

## 17. 가장 중요한 아키텍처 패턴

하나만 기억한다면 이걸 기억하면 됩니다.

```text
launch file이 패키지를 조립하고
URDF/xacro가 하드웨어와 로봇 구조를 선언하며
controller yaml이 제어 동작을 정의하고
SystemInterface가 ROS와 native SDK를 연결하며
native SDK가 실제 하드웨어와 통신하고
그 위에 MoveIt/Nav2/web/YOLO 같은 상위 패키지가 올라간다
```

다른 식으로 요약하면:

```text
고수준 동작
  -> MoveIt / Nav2 / web / pick-place / YOLO

중간 제어 계층
  -> joint_trajectory_controller / diff_drive_controller / gripper action server

하드웨어 추상화
  -> ArmSystemInterface / MobileSystemInterface

native SDK
  -> ArmDriver / MDDriver / gripper runtime

장치 프로토콜
  -> RMD CAN / MD serial / gripper device protocol
```

## 18. 초보자용 빠른 읽기 체크리스트

효율적으로 공부하려면 아래 체크리스트를 따라가면 됩니다.

### Step 1

전체 launch chain 이해:

- `FrJoCo_bringup/launch/full_system_pickplace.launch.py`
- `FrJoCo_bringup/launch/moveit.launch.py`
- `FrJoCo_bringup/launch/full_system.launch.py`

### Step 2

하드웨어 패키지 이해:

- `arm_sdk/ros2/launch/arm.launch.py`
- `mobile_sdk/ros2/launch/mobile.launch.py`
- `gripper_sdk/ros2/launch/gripper.launch.py`

### Step 3

`ros2_control` 선언 이해:

- `arm_sdk/ros2/urdf/ros2_control_arm_sdk.urdf.xacro`
- `mobile_sdk/ros2/urdf/ros2_control_mobile_sdk.urdf.xacro`

### Step 4

controller 설정 이해:

- `arm_sdk/ros2/config/arm_controllers.yaml`
- `mobile_sdk/ros2/config/diff_drive_controller.yaml`
- `moveit/mobile_manipulator_moveit_config/config/moveit_controllers.yaml`

### Step 5

bridge layer 이해:

- `arm_sdk/ros2/src/arm_system_interface.cpp`
- `mobile_sdk/ros2/src/mobile_system_interface.cpp`

### Step 6

native driver 이해:

- `arm_sdk/C++/src/arm_driver.cpp`
- `mobile_sdk/C++/src/md_driver.cpp`

### Step 7

전체 시스템 동작 이해:

- `FrJoCo_bringup/scripts/topic_pick_place.py`
- `yolo_realsense/yolo_realsense/yolo_publisher_node.py`

## 19. 공부할 때 유용한 명령

워크스페이스 루트에서:

```bash
colcon list
```

사용 가능한 패키지를 보여줍니다.

```bash
ros2 launch frjoco_bringup full_system_pickplace.launch.py use_mock_hardware:=true
```

전체 시스템이 어떻게 조립되는지 보기 위한 가장 좋은 단일 명령입니다.

```bash
ros2 node list
ros2 topic list
ros2 control list_controllers
```

런타임 상태를 확인할 때 유용합니다.

```bash
ros2 topic echo /joint_states
```

arm과 mobile state가 실제로 흐르는지 확인할 수 있습니다.

```bash
ros2 action list
```

MoveIt과 gripper action endpoint를 볼 때 유용합니다.

## 20. 최종 요약

이 워크스페이스는 다음과 같이 이해하는 것이 가장 좋습니다.

```text
하나의 로봇 시스템을
역할별 패키지로 분리해 두었고
bringup launch 파일이 이를 조립하며
arm과 mobile은 ros2_control 기반이고
gripper는 standalone node/action wrapping 기반이다
```

아키텍처 공부에 가장 가치가 큰 파일은 아래입니다.

- `FrJoCo_bringup/launch/full_system_pickplace.launch.py`
- `FrJoCo_bringup/launch/moveit.launch.py`
- `arm_sdk/ros2/urdf/ros2_control_arm_sdk.urdf.xacro`
- `mobile_sdk/ros2/urdf/ros2_control_mobile_sdk.urdf.xacro`
- `arm_sdk/ros2/src/arm_system_interface.cpp`
- `mobile_sdk/ros2/src/mobile_system_interface.cpp`
- `FrJoCo_bringup/scripts/topic_pick_place.py`

이 파일들을 이해하면, 나머지 워크스페이스도 훨씬 쉽게 머릿속에 정리됩니다.
