# 1st Miniproject - Classifying Stuff
- 로보틱스 10주 과정 첫번째 미니 프로젝트로, 사용자의 입력을 바탕으로 색이 있는 다양한 물건을 구분하고 이동시키는 것을 목표로 한다.
- This repository includes Doosan ROS2 packages as a dependency.

## Set Environment
### 프로젝트 환경
- OS: Ubuntu 22.04 (Jammy)
- ROS 2: Humble
- Camera: Intel RealSense
- Robot: Doosan DSR
- Build tool: colcon
### 기본 패키지 설치
```
sudo apt update
sudo apt install -y \
  git build-essential \
  python3-colcon-common-extensions \
  python3-rosdep

# rosdeb 초기화
sudo rosdep init
rosdep update
```
### ROS2 Humble 설치 및 기본 설정
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### RealSense (ROS2 패키지 설치)
```
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  librealsense2-utils

# 권한 설정 (이후 재시작 필요)
sudo usermod -aG video $USER
sudo udevadm control --reload-rules && sudo udevadm trigger

```
### Doosan DSR 설치
```
cd /src
git clone https://github.com/DoosanRobotics/doosan-robot2.git
```
### Build
```
colcon build --symlink-install
source install/setup.bash
```
