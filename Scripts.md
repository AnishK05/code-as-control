```bash
.\.venv\Scripts\activate
```

# ROS Commands

## Terminal 1 - Launch MoveIt
```bash
source /home/fri/robot_learning_ws/devel/setup.bash
roslaunch sawyer_moveit_config sim_sawyer_moveit.launch electric_gripper:=true
```
Leave this running.

## Terminal 2 - Execute Policy
```bash
source /home/fri/robot_learning_ws/devel/setup.bash
cd /home/lebron/code-as-control
python3 execute_policy.py policies/20251016-163928_wave-to-me.py
```
