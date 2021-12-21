# indy7_ign_moveit2
This is a basic project to operate Indy7 in Ignition Gazebo environment using MoveIt2 interface.

## Dependency
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)
- [Ignition Fortress](https://ignitionrobotics.org/docs/fortress)
- [MoveIt 2](https://moveit.ros.org/)
  - [Binary Install](https://moveit.ros.org/install-moveit2/binary)
  - [Source Build(Recommend)](https://moveit.ros.org/install-moveit2/source/)
- [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/ros2)
- Ignition package for indy7
  - [indy7_ign](https://github.com/HYU-PBLRC-PB1/indy7_ign.git)
  - [indy7_moveit2_config](https://github.com/HYU-PBLRC-PB1/indy7_moveit2_config.git)

## Tip
In order not to repeatedly perform the personal access token authentication process of the private repository, 
it is recommended to run the following command.

```bash
git config --global credential.helper cache
git config --global credential.helper "cache --timeout=3600" # Cache the authenticated token for 3600 seconds
```

## Install & Build
Run the following command to download the package from the remote repository and install it in your colcon workspace.

```bash
mkdir -p ~/robot_ws/src # Make colcon workespace directory. If colcon workspace does not exist, run this command.
cd ~/robot_ws/src
git clone https://github.com/HYU-PBLRC-PB1/indy7_ign_moveit2.git # Download the package from the remote repository.
cd ~/robot_ws && colcon build --symlink-install # Build colcon workspace.
source ~/robot_ws/install/setup.bash
```

## Package Structure
If you want to check the file structure of a package, run the following command.
```bash
cd ~/robot_ws/src/indy7_ign_moveit2
tree
```

```bash
indy7_ign_moveit2
├── CMakeLists.txt
├── examples
│   ├── cpp
│   │   └── example_ign_moveit2.cpp
│   └── python
│       ├── example_follow_object_direct.py
│       ├── example_follow_object.py
│       ├── example_follow_object_wait_until_executed.py
│       ├── example_gripper.py
│       ├── example_joint_goal.py
│       ├── example_pose_goal.py
│       └── example_throw.py
├── include
│   └── indy7_ign_moveit2
├── launch
│   ├── examples
│   │   ├── example_cpp.launch.py
│   │   ├── example_follow_object_direct.launch.py
│   │   ├── example_follow_object.launch.py
│   │   ├── example_follow_object_wait_until_executed.launch.py
│   │   ├── example_throw.launch.py
│   │   └── worlds
│   │       ├── world_indy7_follow.launch.py
│   │       └── world_indy7_throw.launch.py
│   ├── ign_moveit2.launch.py
│   └── rviz.rviz
├── LICENSE
├── moveit2_py
│   └── moveit2.py
├── package.xml
├── README.md
├── src
└── worlds
    ├── indy7_follow.sdf
    └── indy7_throw.sdf

```

## Example1
Run the following command to execute the basic example file.
```bash
cd ~/robot_ws && source install/setup.bash
ros2 launch indy7_ign_moveit2 example_follow_object.launch.py
```
![fig1](figure/fig1.png)

Change the position of the target (cube) using the MoveIt2 interface to check if the Indy7 follows the target well.

* 실행된 Ignition Gazebo에서 우측 상단의 ⋮ 모양을 클릭합니다.
* Component inspector를 입력한 뒤 클릭합니다.
* Ignition world에 있는 큐브를 클릭합니다.
* Component inspector상의 Pose 정보를 변경합니다.

![fig2](figure/fig2.png)

예상되는 실행 결과는 다음 그림과 같습니다.
![example](figure/example.gif)

## 예제2





### Terminal 1
```bash
cd ~/robot_ws && source install/setup.bash
ros2 launch indy7_ign_moveit2 example_topic.launch.py 
```
### Terminal 2
```bash
ros2 topic pub --once /xyz_pose std_msgs/msg/String 'data: "-0.1 0.0 0.0"'
# wait for 5 seconds!
ros2 topic pub --once /xyz_pose std_msgs/msg/String 'data: "-0.1 0.0 0.0"'
# wait for 5 seconds!
ros2 topic pub --once /xyz_pose std_msgs/msg/String 'data: "0.1 0.0 0.0"'
```

### How does it work? (examples/python/example_topic.py)
```python
# base pose
self.latest_position = [0.25, 0.25, 0.25]
self.latest_quat = [1.0, 0.0, 0.0, 0.0]
```

```python
# msg.data = "-0.1, 0.0, 0.0"
# xyz_msg = ["-0.1", "0.0", "0.0"]
# xyz = [-0.1, 0.0, 0.0] 
xyz_msg = msg.data.split(' ')
xyz = [float(elem) for elem in xyz_msg]

self.latest_position[0] += xyz[0]       
self.latest_position[1] += xyz[1]
self.latest_position[2] += xyz[2]

self.moveit2_.set_pose_goal(self.latest_position, self.latest_quat)
# Plan and execute
self.moveit2_.plan_kinematic_path()
self.moveit2_.execute()
```

## 기타
현재 Ignition Gazebo에서 spawn이 되는 indy7의 model은 'Ignition Robotics'의 robot asset 공유 저장소인 [Fuel](https://app.ignitionrobotics.org/pmh5050/fuel/models/indy7)에서 다운로드 후 spawn이 수행되게끔 패키지 내에 구현되어 있습니다.

따라서, 원격 저장소([Fuel](https://app.ignitionrobotics.org/pmh5050/fuel/models/indy7))에서 변경사항이 있을 경우 이를 반영하기 위해선 local 장치에 설치되어 있는 indy7 model을 직접 삭제해주어야 합니다.

다음의 명령어를 통해 local 장치에 설치되어 있는 indy7 model을 제거할 수 있습니다. 
```bash
cd ~/.ignition/
rm -rf fuel
```
또는 다음의 경로에 직접 접근하는 방식으로 indy7의 model 파일(sdf)을 수정할 수 있습니다.
```
~/.ignition/fuel/fuel.ignitionrobotics.org/pmh5050/models/indy7
```
