# Q1 / Q2 ROS 2 Humble 导航包

这版包含你提供的 Q1 / 1.1 几何 RRT，以及基于原始 `1.2.py` 的 Q2 kinodynamic RRT。两套代码现在分别以 `q1_`、`q2_` 命名：配置、建图、运动公式、规划器、ROS 节点和消息转换各自独立，互不导入。仍使用一个 ROS 包 `q1_navigation` 安装，保留两个运行入口。没有 YAML、Nav2 插件或后台规划线程。四个 Gazebo 场景和正式实验由 partner 完成。

两份配置默认都使用地图原点 `(-5,-5)`、宽高 `10 m`，因此地图范围是 `[-5,5] × [-5,5]`。默认起点为 `(-4,-4,0)`，默认目标矩形为 `(4,4,0.7,0.7)`。地图原点是左下角，与机器人起点是不同参数；起终点仍支持 argparse。以后修改地图范围、半径或话题时，需要在想运行的那一套配置里修改。

## 新增的 1.2 / Q2

原来的 `1.2.py` 是离线实验：先生成全部障碍物，再规划和画图。现在保留它的核心流程：**采样 → 找最近节点 → 枚举控制 → 圆弧碰撞检查 → 扩展树 → 回溯状态和控制序列**，接到现有 ROS 建图接口上。原始文件不修改。

| 文件 | 用途 |
| --- | --- |
| `q2_planner.py` | 你的 kinodynamic RRT；返回 `(x,y,yaw)` 状态和每段 `(v,omega)`，不做几何直线捷径或平滑。 |
| `q2_config.py` | 1.2 的全部配置，包括地图、起终点和话题；保留 `tau=0.5 s`、`v=±0.25` 和原来的 11 个角速度、50000 次迭代上限，增加 3 秒规划时限。 |
| `q2_mapping.py` | 1.2 自己的激光建图、圆盘膨胀和碰撞检查，逻辑沿用已验证的 1.1 版本。 |
| `q2_control.py` | 1.2 自己的状态更新公式、坐标变换和执行圆弧检查。 |
| `q2_ros_node.py` | 独立处理扫描、TF、地图、起点初始化、CSV 和控制序列；不继承 Q1 节点。 |
| `q2_ros_utils.py` | 1.2 的地图和路径消息转换，保留规划状态中的朝向。 |
| `launch/q2.launch.py` | 1.2 的启动入口，起终点参数和 1.1 相同。 |

起点和目标不再写死。目标偏置在目标矩形内采样，默认概率从原来的 0.05 调整为 0.15，可在 `q2_config.py` 修改。最近节点仍按你原来的 XY 距离选取；控制候选先按终点距离排序，再依次检查碰撞，选择结果仍是有效候选中最接近采样点的控制。`(0,0)` 用于停车，不用它给树添加重复节点。

障碍物来自扫描，初始地图未知；不读取 Gazebo 障碍真值。机器人半径在各自配置中设为 `0.25 m`，加 `0.03 m` 余量并做保守栅格膨胀。规划可以经过未知区，但从树根出发的第一步必须通过与执行层相同的已知空闲区检查，包括给输入超时停车预留的预测范围。若没有安全的起始控制，返回 `local_motion_blocked`，等待新扫描后重试，避免返回一条无法开始执行的路线。

正常情况下每段控制保持 `TAU` 秒仿真时间，实际切换由 0.1 秒周期定时器触发，可能有调度误差。实际执行中每个周期都检查输入时效和碰撞；到达目标、输入超时或发现不安全运动时立即停车。模型执行偏差则在控制开始前或完整执行 `TAU` 后检查，并将预测状态对齐到里程计消息的时间戳，避免 Gazebo 加速过程中提前取消尚未完成的控制。偏差阈值仍为 0.10 m 和 0.25 rad，可在 `q2_config.py` 修改；超限后从当前实际位姿重规划。它不会调用 Q1 的几何路径跟踪器来改变规划出的速度。

规划成功后需要重规划时立即处理；只有规划失败才等待 `RETRY_PERIOD` 后重试。终端的 `Kino plan` 行现在包含 `trigger`，执行偏差时还会打印位置和角度误差。`execution_deviation` 表示执行偏差，`future_trajectory_blocked` 表示后续轨迹碰到已知障碍，`blocked_or_unobserved_local_motion` 表示当前运动的安全检查不通过。短暂 TF 等待与这些重规划原因分开记录。

编译步骤与下面相同；新增入口后需要重新编译。先启动 Gazebo 和 Waffle Pi，再运行：

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run q1_navigation q2_navigation --start -4 -4 0 --goal 4 4 0.7 0.7 --ros-args -p use_sim_time:=true
```

partner 若使用 `~/ros2_ws`，替换工作区路径即可。launch 方式为 `ros2 launch q1_navigation q2.launch.py`，也支持下面 Q1 示例中的七个 launch 参数。

Q2 的输出为 `/q2/map`、`/q2/path`，RViz Fixed Frame 用 `q2_map`；Q1 用 `q1_map`。输入和 `/cmd_vel` 仍对应同一台机器人。**Q1、Q2、teleop 不要同时运行。**对比实验必须分别复位到同一场景和起点后启动，否则 `--start` 会将当时实际位置重新当作初始位置。

Q2 日志在 `~/.ros/q2_navigation/`：普通 CSV 记录每次规划；同名前缀的 `_controls.csv` 记录控制值、目标时长、实际仿真开始/结束时间和完成或中断原因。不能把提前中断的一段当作完整执行了 `TAU` 秒。

## 1.1 改了什么

| 文件 | 修改 |
| --- | --- |
| `q1_config.py` | 半径 0.5 改为项目要求的 0.25；保存默认起点、目标矩形、地图范围和话题。起点/目标也可通过 argparse 传入。 |
| `q1_mapping.py` | 只使用 1 m 内激光，射线经过区域标空闲，命中端点标障碍；使用实际雷达 TF；不读取障碍真值。 |
| `q1_planner.py` | 保留自写 RRT；去掉写死的 (9,9)，改为到达目标区域；加入圆盘碰撞检查和规划时间上限。 |
| `q1_ros_node.py` | 原来只接收数据/建图，现在接上规划、速度发布、路径跟踪和重规划。 |
| `q1_ros_utils.py` | 转换并发布 ROS 地图和路径。 |
| 新增 `q1_control.py` | 按路径控制线/角速度，并检查预测圆弧是否安全。 |
| `q1_main.py` | 作为 ROS 入口；原来的直接修改机器人坐标不能用于 Gazebo。 |
| `q1_simulation.py` | 不放入运行包；它是 HW2 随机障碍生成器，Gazebo 场景由 partner 提供。 |

原节点缺 `TransformListener`，且用底盘位姿代替激光位姿；现在查询 `odom <- scan.header.frame_id`，包含雷达安装偏移。

## ROS 封装

包的外层新增 `package.xml`（依赖）、`setup.py`（安装和可执行入口）、`setup.cfg`（安装路径）、`resource/q1_navigation`（注册标记）、`launch/q1.launch.py`（启动文件）。Python 模块放在内层同名 `q1_navigation/` 文件夹并使用相对导入。

结构核对了 [ROS 2 Humble 官方包教程](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.rst) 与 [官方 launch 教程](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/Launch/Launch-system.rst)。

## partner 怎么用

把整个 `q1_navigation` 目录放在 `~/turtlebot3_ws/src/`，然后：

```bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws
rosdep install --from-paths src/q1_navigation --ignore-src -r -y
colcon build --symlink-install --packages-select q1_navigation
source install/setup.bash
```

1.1 在 `src/q1_navigation/q1_navigation/q1_config.py` 修改起点、目标和地图范围；1.2 在同目录的 `q2_config.py` 修改。两套默认地图均是 [-5,5]×[-5,5]，原点 (-5,-5)，起点和目标也用同一坐标系。`GOAL` 是 `(左下角x, 左下角y, 宽, 高)`；面积不超过约 5 m²。到达条件是机器人中心进入目标矩形。

起点和目标也可以直接用 argparse 覆盖，不用修改文件：

```bash
ros2 run q1_navigation q1_navigation --start -4 -4 0 --goal 4 4 0.7 0.7 --ros-args -p use_sim_time:=true
```

`--start X Y YAW` 表示地图中的初始位姿，朝向单位为弧度；`--goal X Y WIDTH HEIGHT` 表示目标矩形。省略参数时，Q1 使用 `q1_config.py`，Q2 使用 `q2_config.py` 的值。`--ros-args -p use_sim_time:=true` 启用仿真时钟。程序先通过 `rclpy.utilities.remove_ros_args` 分离 ROS 参数，再交给 argparse 处理起终点，最后将原始参数交给 ROS 初始化。[Humble 官方实现](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/utilities.py)

如果继续使用 launch，则这样传值（launch 会将这些值转成上面的 argparse 参数）：

```bash
ros2 launch q1_navigation q1.launch.py start_x:=-4.0 start_y:=-4.0 start_yaw:=0.0 goal_x:=4.0 goal_y:=4.0 goal_width:=0.7 goal_height:=0.7
```

以上两个导航启动命令选一种运行。这些参数不会移动或重新生成 Gazebo 机器人；实际出生位姿仍由场景启动命令设置。命令行更换起终点只需重新启动节点，不需要重新编译。

**`START_X/Y/YAW` 要对应启动节点时的实际出生位姿。**节点将首次 odom 平移/旋转到这个起点。改配置、复位 Gazebo 或换场景后重启节点。`--symlink-install` 后改 Python 文件通常只需重启；新增文件或修改包安装信息则重新编译。

partner 先启动自己的 Gazebo world 和 TurtleBot3 Waffle Pi（模型启动终端设置 `export TURTLEBOT3_MODEL=waffle_pi`），再新开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch q1_navigation q1.launch.py
```

launch 只启动导航节点，不创建 world。不同时运行 teleop、turtlebot3_drive 或其他向 `/cmd_vel` 发指令的节点。

## 1.1 对接接口

| 方向 | 话题 | 类型/用途 |
| --- | --- | --- |
| 输入 | `/scan` | `sensor_msgs/msg/LaserScan` |
| 输入 | `/odom` | `nav_msgs/msg/Odometry` |
| 输入 | `/tf`、`/tf_static`、`/clock` | 坐标变换和仿真时间 |
| 输出 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 输出 | `/q1/map` | `nav_msgs/msg/OccupancyGrid` |
| 输出 | `/q1/path` | `nav_msgs/msg/Path` |

Gazebo/robot_state_publisher 必须提供 `odom -> base_footprint/base_link -> base_scan`（实际 frame 名以 scan 为准）。本节点提供 `q1_map -> odom`，不要同时运行给同一个 odom 发布父变换的 SLAM 节点。本包自己建图。TF API 核对了 [Humble 官方 TransformListener](https://github.com/ros2/geometry2/blob/humble/tf2_ros_py/tf2_ros/transform_listener.py)。

RViz 的 Fixed Frame 设为 `q1_map`，添加 Map `/q1/map`、Path `/q1/path`、LaserScan `/scan`。

两套导航节点不再打印 TF 等待和输入超时的 warning。缺少可用数据时仍等待或停车。如果持续不动，可以用 `ros2 run tf2_ros tf2_echo odom base_scan --ros-args -p use_sim_time:=true` 检查当前 TF 链；实际 frame 名以消息 header 为准。最新 TF 能查到，并不代表每帧扫描对应时刻的 TF 已经到达。

节点会暂存一帧扫描等待对应时间的 TF，处理后再接收下一帧；超过输入超时的待处理扫描会被替换。这样可避免新扫描不断覆盖旧扫描，导致 TF 稍有延迟时始终无法建图。它仍使用扫描的时间戳，不以最新 TF 替代扫描时刻的变换。重启/复位 Gazebo 后应同时重启导航节点。

## 报告中说明的逻辑

规划允许尝试未知区域；实际执行前，预测经过的圆盘区域必须已知空闲。只用 1 m 内激光逐步发现障碍。机器人当前圆盘内部视为空闲以填补雷达盲区，不提前开放周围地图。缺扫描或里程计时停车。

重规划条件：剩余路径被新障碍挡住、局部运动不安全或未知、路径走完但未进入目标、上次规划失败。规划时先停车，RRT 有时间上限；返回后继续处理新扫描，再执行下一步。

每次启动在 `~/.ros/q1_navigation/` 新建 CSV，记录次数、原因、耗时、结果和路径长度。第一次规划不算重规划，此后每次尝试（含失败）都计入。实际行驶距离在终端结束信息中打印。规划耗时使用 wall time。成功率、碰撞与视频由实际实验确认。

## 范围与检查

本包包含 **Q1 和 Q2** 的代码。两套配置和运行模块独立；Q2 用 kinodynamic 控制扩展和控制序列执行。两人组的 Q3 动态障碍演示仍需单独完成。

1 m 视野、保守栅格膨胀、没有独立 frontier 探索、扫描未逐束去畸变、里程计误差可能导致窄通道拒绝或停滞。Q2 的离散控制及模型与 Gazebo 的执行偏差也可能增加重规划次数。新障碍可进入地图并触发重规划，但不保证任意动态场景安全。空场景检查不代表四场景正式实验通过；实际成功率、碰撞、耗时、重规划次数与视频仍需 partner 测试和分析。

核心测试：

```bash
cd ~/turtlebot3_ws/src/q1_navigation
python3 -m pytest -q
```

实际编译/检查记录见 `VALIDATION.txt`。
