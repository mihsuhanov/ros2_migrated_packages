## Домашнее задание по курсу "Автономные мобильные роботы"
Выполнено с использованием ROS2 Humble. Вероятно не будет работать на других версиях.

Clone:
```bash
   mkdir ros2_ws || cd ros2_ws
   git clone https://github.com/mihsuhanov/ros2_migrated_packages.git --recursive
```
Build (in directory ros2_ws):
```bash
   colcon build --symlink-install
   source install/setup.bash
```
Launch (in directory ros2_ws):
```bash
   ros2 launch simple_planner planner.launch.py
```
```bash
   ros2 launch feature_matcher matcher.launch.py
```
```bash
   ros2 launch barrel_slam slam.launch.py
```
   