# Project 1 — Q1 / Q2 ROS 2 navigation

ROS 2 Humble 导航包：Q1 / 1.1 使用几何 RRT，Q2 / 1.2 使用你提供的 kinodynamic RRT。两套代码分别以 `q1_`、`q2_` 命名，各自实现 1 m 激光建图、机器人尺寸膨胀和 ROS 接口，并记录重规划次数与耗时。包名仍为 `q1_navigation`，两套模块互不导入。

1.1 配置在 [`q1_navigation/q1_config.py`](q1_navigation/q1_config.py)，1.2 配置在 [`q1_navigation/q2_config.py`](q1_navigation/q2_config.py)，修改一份不会影响另一份。两者默认地图原点均为 `(-5,-5)`，范围 `[-5,5] × [-5,5]`，起点 `(-4,-4,0)`，目标矩形 `(4,4,0.7,0.7)`。起点和目标可以在启动时通过 argparse 覆盖，不需要 YAML。地图原点表示地图左下角，不是机器人起点。

| 功能 | 1.1 | 1.2 |
| --- | --- | --- |
| 配置 | `q1_config.py` | `q2_config.py` |
| 建图 / 膨胀 | `q1_mapping.py` | `q2_mapping.py` |
| 运动 / 控制 | `q1_control.py` | `q2_control.py` |
| RRT | `q1_planner.py` | `q2_planner.py` |
| ROS 节点 | `q1_ros_node.py` | `q2_ros_node.py` |
| ROS 消息转换 | `q1_ros_utils.py` | `q2_ros_utils.py` |
| 启动文件 | `q1.launch.py` | `q2.launch.py` |

- [代码修改及使用说明](README_zh.md)
- [已完成的验证与验证范围](VALIDATION.txt)

## 编译和启动

将交付的 `q1_navigation` 文件夹放到 `~/turtlebot3_ws/src/`，然后编译。partner 若使用 `~/ros2_ws`，将下面工作区路径相应替换。

```bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws
rosdep install --from-paths src/q1_navigation --ignore-src -r -y
colcon build --symlink-install --packages-select q1_navigation
source install/setup.bash
```

修改配置中的地图范围、起始位姿和目标区域，使其与 partner 的场景一致。先启动 Gazebo 场景和 Waffle Pi，再运行：

```bash
ros2 launch q1_navigation q1.launch.py
```

使用 argparse 指定起点 `(-4,-4,0)` 和目标矩形 `(4,4,0.7,0.7)`：

```bash
ros2 run q1_navigation q1_navigation --start -4 -4 0 --goal 4 4 0.7 0.7 --ros-args -p use_sim_time:=true
```

`--start` 的顺序是 `x y yaw`（yaw 单位为弧度）；`--goal` 的顺序是 `x y width height`。不传时使用配置文件默认值。起点参数用于导航坐标初始化，Gazebo 中的实际出生位置仍由场景启动命令设置。

原来的 launch 方式也支持传值：

```bash
ros2 launch q1_navigation q1.launch.py start_x:=-4.0 start_y:=-4.0 start_yaw:=0.0 goal_x:=4.0 goal_y:=4.0 goal_width:=0.7 goal_height:=0.7
```

## 运行 1.2 / Q2

同一包中切换可执行入口即可，起终点参数与 1.1 相同：

```bash
ros2 run q1_navigation q2_navigation --start -4 -4 0 --goal 4 4 0.7 0.7 --ros-args -p use_sim_time:=true
```

也可以用 `ros2 launch q1_navigation q2.launch.py`，launch 参数与上面 Q1 相同。

Q2 保留原来的离散 `(v, omega)` 控制和 `tau=0.5 s`，按顺序执行控制，偏差过大或发现障碍时停车重规划。Q2 的控制、地图范围、半径、话题和默认起终点全部在 `q1_navigation/q2_config.py`。RViz 使用 `/q2/map`、`/q2/path`，Fixed Frame 为 `q2_map`；Q1 仍使用 `q1_map`。规划和控制执行日志在 `~/.ros/q2_navigation/`。

执行偏差在每段控制开始前或执行满 `tau` 后评估，并对齐里程计时间戳，避免加速过程触发短促的停车/重规划循环。碰撞和输入超时仍每周期检查。终端 `trigger=` 显示重规划原因；第一步控制不安全时返回 `local_motion_blocked`。详细逻辑与验证范围见中文说明和 `VALIDATION.txt`。

Q1、Q2 以及 teleop 不要同时运行：它们都会向 `/cmd_vel` 发指令。比较两种算法时，复位到同一场景、同一起点，并分别启动导航。

已完成包编译、核心测试与空场景接口检查；Q2 空场景近距离到达检查通过。四场景正式对比实验和 Q3 动态障碍演示仍需完成。
