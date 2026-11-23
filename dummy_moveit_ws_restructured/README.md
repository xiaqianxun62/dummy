# Dummy MoveIt WS (restructured)

这版工作空间按照“**统一模型 + MoveIt / ros2_control 主控 + STM32 作为执行器与状态反馈层**”重构。

## 包结构

- `dummy_ros2_description`
  - 唯一的机器人模型来源（URDF/xacro、mesh、RViz/Gazebo 展示）
- `dummy_moveit_config`
  - MoveIt 配置（SRDF、OMPL、joint limits、controllers、Servo demo）
- `dummy_hardware_interface`
  - `ros2_control` 真机插件 + ASCII 维护 CLI
- `dummy_bringup`
  - 统一 bringup 入口，区分 fake / real / simulation

## 推荐运行路线

### 1. 先做板端维护阶段（推荐）

这一步不启动 MoveIt，只对板子做启停、参考位、状态确认。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 start
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 getjpos
```

如果需要：

```bash
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 home
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 reset
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 set-home-current
```

### 2. 再启动 ROS 侧真机链

`hardware.launch.py` 默认走安全模式：
- `use_fake_hardware:=false`
- `start_on_activate:=false`
- `auto_home_on_activate:=false`

```bash
ros2 launch dummy_bringup hardware.launch.py serial_port:=/dev/ttyACM0

```

### 3.  Moveit2 仿真模式 

```bash
ros2 launch dummy_moveit_config demo.launch.py
```

### 5. 真实角度和RIVZ角度补偿说明

> RViz角度 = 真机角度 × sign - offset


```bash
真机：0  -73  180  0  0  0
RIVZ：0  -73  90  0  0  0
固定角度补偿
Joint1: 0°
Joint2: 0°
Joint3: +90°
Joint4: 0°
Joint5: 0°
Joint6: 0°
方向补偿
Joint1: 正向
Joint2: 正向
Joint3: 正向
Joint4: 正向
Joint5: 反向
Joint6: 反向
```

### 4. Fake hardware 开发模式

```bash
ros2 launch dummy_bringup bringup.launch.py
```

### 5. Gazebo 兼容展示模式

> 当前 simulation 仍然是 “Gazebo 展示 + fake hardware 控制链”，不是完整 Gazebo 闭环。

```bash
ros2 launch dummy_bringup simulation.launch.py
```

## 构建

```bash
mkdir -p ~/ros2_ws/src
# 把本目录内容放到 ~/ros2_ws/ 下
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 这次重构的重点

1. 去掉了 workspace 里的 `build/ install/ log/` 污染，只保留源码。
2. `dummy_bringup` 成为唯一主入口，launch 逻辑集中。
3. `dummy_moveit_config` 改成更贴近 Jazzy 的 OMPL 配置格式。
4. 补了 acceleration limits，避免 MoveIt 在时间参数化阶段因为加速度缺失而失败。
5. `dummy_hardware_interface` 加入了更稳的 ASCII 命令超时与队列确认逻辑。
6. 新增 `dummy_ascii_cli`，把“板端维护阶段”和“ROS 接管阶段”明确拆开。

## 关于 HOME / RESET

固件实现里 `!HOME` 和 `!RESET` 是**阻塞动作**：控制器会先执行整段动作，再返回 `Started ok`。因此这两个命令不适合继续按 300ms 的短应答思路处理；本工作区已经为它们单独拉长等待时间。

另外，固件 `#GETJPOS` 返回的是**已经叠加了 `initPose` 的语义关节角**，也就是“机械臂语义角度”，不是电机原始角度。因此 ROS 侧固定的 `position_offsets_rad` / `position_signs` 主要负责**ROS 模型与固件语义之间**的坐标映射，而不是把 EEPROM home offset 再次叠加一遍。
