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
cd turtle_circle
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


***


## 실습2(오픈소스예제 ROS 패키지화)

### 개요
<img src="https://user-images.githubusercontent.com/87575823/148650155-5136e71c-bf54-467d-a42f-dcae0c02712f.png" width="400" height="200"/>　　<img src="https://user-images.githubusercontent.com/87575823/148650680-43b41ad7-2557-4cbe-897f-ff26b024f48c.png" width="200" height="200"/>

turtlesim에서 turtle을 총 2마리(turtle1, turtle2) 생성한 후, turtle2가 turtle1과 부딪히지 않게 지정된 waypoints를 순환하는 dwa_local_planner 패키지를 설계하였다. 이 때, 기존의 turtle1에 추가된 turtle2는 "ros2 service call /spawn turtlesim/srv/Spawn" 명령어를 통하여 생성한다. rqt_graph와 같이 "/turtlesim_node" 에서 각 turtle의 위치 정보를 가지고 있는 "/turtle1/pose" 와 "/turtle2/pose" 를 발행하면 "/dwa_planner" 노드에서 구독한 뒤, turtle2의 경로, 속도를 계산하게 된다. 그리고 계산된 속도 정보를 가지고 있는 "/turtle2/cmd_vel" 를 "/turtlesim_node" 로 발행하여 turtle2를 움직이게 한다.

### 1) dwa_local_planner 패키지 생성

```
cd ~/robot_ws/src
ros2 pkg create dwa_local_planner --build-type ament_python dependencies rclpy std_msgs
cd dwa_local_planner
```

이후 [package.xml](https://github.com/pby04188/ROS_tutorials/blob/d4f3616d23c17cfebbd87c0b76fada576634d0b0/dwa_local_planner/package.xml), [setup.py](https://github.com/pby04188/ROS_tutorials/blob/d4f3616d23c17cfebbd87c0b76fada576634d0b0/dwa_local_planner/setup.py) 파일을 수정한 뒤, dwa_local_planner 폴더 안에 [dwa.py](https://github.com/pby04188/ROS_tutorials/blob/d4f3616d23c17cfebbd87c0b76fada576634d0b0/dwa_local_planner/dwa_local_planner/dwa.py) 를 생성한다.

### 2) parameter 파일 생성

[dwa.py](https://github.com/pby04188/ROS_tutorials/blob/d4f3616d23c17cfebbd87c0b76fada576634d0b0/dwa_local_planner/dwa_local_planner/dwa.py) 에서 turtle2의 waypoints 정보를 파라미터로 설정하였다. 따라서, 이 파라미터의 정보를 담고 있는 파일을 생성한다.

```
cd ~/robot_ws/src/dwa_local_planner
mkdir param
cd param
```

이후 [waypoints.yaml](https://github.com/pby04188/ROS_tutorials/blob/810375a6239446005de0106ca017b2ac2094cbae/dwa_local_planner/param/waypoints.yaml) 을 생성한다. num_waypoints 는 waypoints의 수, waypoint_n은 각 waypoint의 [x(m), y(m)] 정보이다.

### 3) launch 파일 생성

해당 작업을 launch 없이 실행하기 위해선 다음과 같은 작업이 필요하다.

```
ros2 run turtlesim turtlesim_node
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0, theta: 0, name: 'turtle2'}"
ros2 run dwa_local_planner dwa_planner
```

이 작업을 하나의 launch 명령어로 단순화할 수 있도록 launch 파일을 생성하였다.

```
cd ~/robot_ws/src/dwa_local_planner
mkdir launch
cd launch
```

이후 [dwa.launch.py](https://github.com/pby04188/ROS_tutorials/blob/810375a6239446005de0106ca017b2ac2094cbae/dwa_local_planner/launch/dwa.launch.py) 를 생성한다.

### 4) 빌드

```
cd ~/robot_ws
colcon build --symlink-install --packages-select dwa_local_planner
. ~/robot_ws/install/local_setup.bash
```

### 5) 실행

```
ros2 launch dwa_local_planner dwa.launch.py
```

waypoints를 변경하고 싶으면 파라미터 파일 [waypoints.yaml](https://github.com/pby04188/ROS_tutorials/blob/810375a6239446005de0106ca017b2ac2094cbae/dwa_local_planner/param/waypoints.yaml) 을 다시 빌드하여 사용하면 된다.
