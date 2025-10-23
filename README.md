# code-as-control

```bash
.\.venv\Scripts\activate
```

# Need:

Python 3.13 and Sawyer

# ROS Commands

## Terminal 1 - Launch MoveIt
```bash
source /opt/ros/noetic/setup.bash
roslaunch sawyer_moveit_config sim_sawyer_moveit.launch electric_gripper:=true
```
Leave this running.

## Terminal 2 - Execute Policy
```bash
source /opt/ros/noetic/setup.bash
cd /home/lebron/code-as-control
python3 execute_policy.py policies/20251016-163928_wave-to-me.py
```

## Prerequisites
Need sudo to install: `ros-noetic-sawyer-robot ros-noetic-sawyer-moveit`
