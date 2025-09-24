# tdk_ros2_ws
## docker
### 啟動dokcer
在 docker 資料夾底下
compose container
```javascript=
docker compose up -d //後面可以接container name指定開啟
```
右鍵想要的containerAttach Shell可以開啟container環境的terminal
![image](https://hackmd.io/_uploads/BJ1mvDb3lg.png)
### 關閉docker
```javascript=
docker compose down //後面可以接container name指定關閉
```
## launch
在 task container 內
先讓程式啟動
```javascript=
colcon build --package-select main-pkg
source install/setup.bash
ros2 launch main-pkg bringup.launch.py
```
自特定關卡開啟
```javascript=
ros2 topic pub --once mission_starter std_msgs/msg/Int32 "{data: 1}" 
//data: 後面放關卡代號1 2 3 4
```
## topic
### 看特定topic的狀態
上位機發下去的速度
```javascript=
ros2 topic echo robot/cmd_vel
```
下位機發上來的速度
```javascript=
ros2 topic echo robot/pose --field twist.twist
```
下位機發上來的位置
```javascript=
ros2 topic echo robot/pose --field pose.pose
```
其他topics
```javascript=
//下發的手臂任務代號
robot/cmd_arm
//上發的手臂狀態代號
robot/status
//目前關卡
mission_command
//關卡狀態
mission_status
```
## stm
看stm連接狀態
```javascript=
ls -la /dev/serial/by-id/ | grep -i stm
```
