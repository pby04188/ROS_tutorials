# ROS_tutorials
## 실습1(Turtlesim을 활용한 ROS 기초 프로그래밍)
![rosgraph](https://user-images.githubusercontent.com/87575823/147875087-5e53f1a6-3571-4dea-a420-085a650b4218.png)

###  개요
원운동의 회전반경(radius), linear velocity(velocity), 회전방향(direction) 정보를 담은 "/rad_vel_dir" 토픽을 입력받으면 그에 맞게 turtlesim이 움직이도록 패키지를 설계하여야 한다. "/turtlesim_node" 는 "/turtle1/cmd_vel" 로부터 속도명령을 받아 움직이므로 "/rad_vel_dir" 를 subscribe 한 뒤, 이를 변환하여 "/turtle1/cmd_vel" 토픽 메세지의 형태로 publish 하는 node가 필요하다. 이에 해당하는 node가 그림의 "/rvd_to_cmd" 이다.

먼저, radius, velocity, direction(이하 rvd) 메세지 타입을 정의하기 위해 인터페이스로만 구성된 패키지인 "msg_srv_action_interface" 와 등속 원운동을 위한 패키지 "turtle_circle" 을 생성하였다.

### 1) msg_srv_action_interface 패키지 생성
입력받게 될 rvd 메세지 타입을 새롭게 정의한다.
```
cd ~/robot_ws/src
ros2 pkg create  msg_srv_action_interface --build-type ament_cmake
cd msg_srv_action_interface
mkdir msg srv action
```
이후 [package.xml](https://github.com/pby04188/ROS_tutorials/blob/207e3afbc35bbecd63247a714cbd63cc47343313/msg_srv_action_interface/package.xml), [CMakeLists.txt](https://github.com/pby04188/ROS_tutorials/blob/207e3afbc35bbecd63247a714cbd63cc47343313/msg_srv_action_interface/CMakeLists.txt) 파일을 변경한 뒤, msg 폴더 안에 [RVD.msg](https://github.com/pby04188/ROS_tutorials/blob/207e3afbc35bbecd63247a714cbd63cc47343313/msg_srv_action_interface/msg/RVD.msg) 를 생성한다.

### 2) turtle_circle 패키지 생성

```
cd ~/robot_ws/src
ros2 pkg create turtle_circle --build-type ament_python dependencies rclpy std_msgs
```
이후 [package.xml](https://github.com/pby04188/ROS_tutorials/blob/207e3afbc35bbecd63247a714cbd63cc47343313/turtle_circle/package.xml), [setup.py](https://github.com/pby04188/ROS_tutorials/blob/207e3afbc35bbecd63247a714cbd63cc47343313/turtle_circle/setup.py) 파일을 변경한 뒤, turtle_circle 폴더 안에 [rvd_to_cmd.py](https://github.com/pby04188/ROS_tutorials/blob/207e3afbc35bbecd63247a714cbd63cc47343313/turtle_circle/turtle_circle/rvd_to_cmd.py) 를 생성한다.

### 3) launch 파일 작성
해당 작업을 수행하기 위해선 "/turtlesim_node" 노드와 "/rvd_to_cmd" 노드를 모두 실행시켜야 하는데 이 때 "ros2 run" 명령어를 이용하면 명령어를 2번 입력해야 하기 때문에 비효율적이다. 따라서, 두 노드를 한 번에 실행시키기 위해 "ros2 launch" 를 이용한다.
```
cd ~/robot_ws/src/turtle_circle
mkdir launch
cd launch
```
이후 [turtlecircle.launch.py](https://github.com/pby04188/ROS_tutorials/blob/207e3afbc35bbecd63247a714cbd63cc47343313/turtle_circle/launch/turtlecircle.launch.py) 를 생성한다.


### 4) 빌드

```
cd ~/robot_ws
colcon build --symlink-install --packages-select msg_srv_action_interface
colcon build --symlink-install --packages-select turtle_circle
. ~/robot_ws/install/local_setup.bash
```

### 5) 실행
```
ros2 launch turtle_circle turtlecircle.launch.py
```
이후 "ros2 topic pub" 명령어를 통해 "/rad_vel_dir" 토픽을 발행할 수 있다.
```
ros2 topic pub --once /rad_vel_dir msg_srv_action_interface/msg/RVD "{radius: 1.0, velocity: 2.0, direction: True}"
```
(시계반대방향일 때 direction : True, 시계방향일 때 direction : False)
