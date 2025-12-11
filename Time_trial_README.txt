final race time trial
====================
ssh into car
* roscore
* roslaunch f1tenth_purepursuit final_race.launch max_vel:=65 min_vel:=35 file:=pure_pursuit.py

locally
* roslaunch move_base remote_teleop.launch
* rviz 