# Dummy 6-DOF 机械臂工作空间

本仓库包含同一台 6 自由度机械臂的两套上位机路线：

- **ROS 2 / MoveIt 路线**：统一 URDF 模型 + MoveIt 2 / `ros2_control` 主控，STM32 作为执行器与状态反馈层。
- **纯 C# WPF 路线**：不依赖 ROS / rosbridge，自行实现运动学、仿真、规划、伺服和串口协议，用于上位机学习与工程验证。

> 建议顺序：先跑板端维护命令确认硬件，再启动 ROS 真机链；无硬件时先用 fake hardware、MoveIt demo 或 C# CLI 验证。

---

## 目录

- [这个项目包含什么](#这个项目包含什么)
- [环境要求](#环境要求)
- [构建](#构建)
- [ROS 2 运行路线](#ros-2-运行路线)
- [C# WPF 上位机](#c-wpf-上位机)
- [关键约定](#关键约定)
- [安全提示](#安全提示)
- [常见问题](#常见问题)
- [相关文档](#相关文档)

---

## 这个项目包含什么

### ROS 2 包（`src/`）

| 包 | 作用 |
| --- | --- |
| `dummy_ros2_description` | 唯一的模型来源：URDF/xacro、STL mesh、RViz/Gazebo 展示配置 |
| `dummy_moveit_config` | MoveIt 2 配置：SRDF、OMPL、关节限位、控制器、Servo 示例 |
| `dummy_hardware_interface` | `ros2_control` 真机硬件插件 + ASCII 维护 CLI |
| `dummy_bringup` | 统一启动入口，区分 fake / real / simulation 三种模式 |
| `dummy_kinematics_validation` | 独立的 FK / IK / roundtrip 验证包 |

### C# 上位机（`DummyWpfUpper/`）

- .NET 8 WPF 主程序：真机/仿真实时状态、关节与笛卡尔控制、示教点、日志、3D 视图、路径可视化。
- 自研核心算法：解析 IK/FK、数值雅可比、碰撞检测、RRT-Connect、笛卡尔速度伺服。
- `DummyCli`：复用核心算法的命令行验证工具，可脱离 UI 和硬件运行。

### 目录速览

```text
.
├── src/                              # ROS 2 工作空间源码
│   ├── dummy_ros2_description/
│   ├── dummy_moveit_config/
│   ├── dummy_hardware_interface/
│   ├── dummy_bringup/
│   └── dummy_kinematics_validation/
├── DummyWpfUpper/                    # .NET 8 WPF 上位机
│   ├── DummyWpfUpper.csproj
│   ├── DummyCli/                     # 命令行验证工具
│   ├── Services/                     # 运动学、规划、伺服、串口、持久化
│   ├── ViewModels/
│   ├── Models/
│   ├── Views/
│   └── DummyWpfUpper_学习计划与指导书.md
├── README.md
├── RESTRUCTURE_SUMMARY.md
├── 上下文交接.md
└── 1.md                              # UI 架构模式学习笔记，非运行依赖
```

---

## 环境要求

### ROS 2 侧

- Ubuntu + ROS 2 **Jazzy**（命令中默认 `source /opt/ros/jazzy/setup.bash`）。
- MoveIt 2、`ros2_control`、`ros2_controllers`、`robot_state_publisher`、`xacro`、RViz2。
- Gazebo 相关包仅用于展示模式；当前 simulation **不是完整 Gazebo 闭环仿真**。
- 真机串口：ASCII over serial，`115200 8N1`，命令以 CRLF 结束；Linux 默认 `/dev/ttyACM0`。
- 当前用户需要串口访问权限，例如加入 `dialout` 组后重新登录。

### C# 侧

- Windows 10/11 + .NET 8 SDK。
- 主程序目标框架 `net8.0-windows`，启用 WPF；CLI 目标框架 `net8.0`。
- NuGet 依赖：`System.IO.Ports 8.0.0`。
- Windows 默认串口示例为 `COM3`，波特率同样为 `115200`。

---

## 构建

### ROS 2

把本仓库的 `src/` 放到 ROS 2 工作空间根目录（例如 `~/dummy_moveit_ws/src/`），然后执行：

```bash
cd ~/dummy_moveit_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

本仓库刻意不保留 `build/`、`install/`、`log/`，这些目录由本机构建生成。

只构建某个包时：

```bash
colcon build --packages-select dummy_kinematics_validation --symlink-install
source install/setup.bash
```

### C# WPF

在 Windows PowerShell 中、仓库根目录执行：

```powershell
dotnet build DummyWpfUpper/DummyWpfUpper.csproj
dotnet build DummyWpfUpper/DummyCli/DummyCli.csproj
```

> `DummyCli` 位于主工程子目录内，主工程必须保留 `<Compile Remove="DummyCli/**" />`，否则会把两个工程的程序集属性一起编译导致失败。

---

## ROS 2 运行路线

以下命令均假设已执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 1. 板端维护阶段（推荐第一步）

此阶段不启动 MoveIt，只通过 ASCII CLI 对板子做启停、参考位设置和状态确认。

```bash
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 start
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 getjpos
```

按需执行：

```bash
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 home
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 reset
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 set-home-current
```

`home` / `reset` 是阻塞动作，等待时间已单独拉长；不要用普通查询的短超时判断它们。

### 2. 真机 ROS 链

```bash
ros2 launch dummy_bringup hardware.launch.py serial_port:=/dev/ttyACM0
```

默认安全配置：

- `use_fake_hardware:=false`
- `start_on_activate:=false`
- `auto_home_on_activate:=false`
- `disable_on_deactivate:=true`
- `position_offsets_rad:=0,0,1.57079,0,0,0`
- `position_signs:=1,1,1,1,-1,-1`

确认现场安全后，可按需显式传入 `start_on_activate:=true`、`auto_home_on_activate:=true` 等参数；不要依赖默认自动启动。

### 3. Fake hardware 开发模式

不接板子也能跑完整控制链，适合开发 UI、MoveIt 配置和规划逻辑：

```bash
ros2 launch dummy_bringup bringup.launch.py
```

### 4. MoveIt 2 Demo

```bash
ros2 launch dummy_moveit_config demo.launch.py
```

### 5. Gazebo 兼容展示模式

```bash
ros2 launch dummy_bringup simulation.launch.py
```

当前语义是“Gazebo 展示 + fake hardware 控制链”，不是完整的 Gazebo 物理闭环仿真。

### 6. FK / IK 验证

推荐用 launch 启动，它会自动加载 `robot_description`、`robot_description_semantic` 和 `kinematics.yaml`：

```bash
ros2 launch dummy_kinematics_validation ik_fk_validation.launch.py
```

也可直接运行节点并指定模式：

```bash
ros2 run dummy_kinematics_validation ik_fk_validator --ros-args \
  -p mode:=roundtrip \
  -p group_name:=dummy_arm \
  -p tip_link:=link6_1_1
```

`mode` 支持 `fk`、`ik`、`roundtrip`。更多参数示例见 `src/dummy_kinematics_validation/README.md`。

---

## C# WPF 上位机

### 启动 GUI

```powershell
dotnet run --project DummyWpfUpper/DummyWpfUpper.csproj
```

已集成的主要能力：

- 真机 / 仿真后端切换，20 Hz 仿真状态刷新。
- C# 解析 IK/FK、关节/笛卡尔控制、关节速度显示。
- RRT-Connect 关节空间规划、示教点管理、3D 路径预览。
- 笛卡尔速度伺服（30 Hz，DLS 伪逆）。
- 串口 ASCII 协议、电机 DCE PID / REBOOT、JSON 配置持久化、日志导出。
- WPF 3D 视图使用 `Meshes/` 中的 STL，与 ROS 描述包共用模型文件。

### 无硬件 CLI 验证

在仓库根目录执行：

```powershell
dotnet run --project DummyWpfUpper/DummyCli/DummyCli.csproj -- iktest 30 -50 90 20 -30 45
dotnet run --project DummyWpfUpper/DummyCli/DummyCli.csproj -- rrttest
dotnet run --project DummyWpfUpper/DummyCli/DummyCli.csproj -- servotest 30 -50 90 20 -30 45 10 0 0 0 0 0 base
```

- `iktest`：FK → IK → FK 回环，误差应接近 0。
- `rrttest`：RRT 规划并逐段做碰撞检查。
- `servotest`：用有限差分反推末端速度，与给定 twist 对比；最后一参数为 `base` 或 `tool`。

交接文档记录的本机验证结果：两个工程构建均为 0 警告 0 错误，`iktest`、`rrttest`、`servotest` 均通过。

---

## 关键约定

### 1. 真机角度 ↔ ROS/RViz 角度

硬件插件中的映射为：

```text
ROS(rad) = sign * deg2rad(fw_deg) - offset
fw(deg)  = rad2deg((ROS(rad) + offset) * sign)
```

默认参数：

```text
offset = [0, 0, 1.57079, 0, 0, 0] rad   # J3 为 +90°
sign   = [1, 1, 1, 1, -1, -1]           # J5 / J6 反向
```

示例：

```text
真机:  0  -73  180  0  0  0
RViz:  0  -73   90  0  0  0
```

### 2. `#GETJPOS` 的语义

固件 `#GETJPOS` 返回的是**已经叠加 `initPose` 的语义关节角**，不是电机原始角度。

因此 ROS 侧的 `position_offsets_rad` / `position_signs` 只负责“ROS 模型 ↔ 固件语义”的坐标映射，不能再把 EEPROM 里的 home offset 叠加一次。

### 3. `!HOME` / `!RESET` 的阻塞语义

固件执行 `!HOME` / `!RESET` 时会先跑完整段动作，再返回 `Started ok`。所以：

- 它们是长耗时命令，不是 300 ms 级查询。
- ROS 侧和 C# CLI 都为这些命令设置了单独的 long timeout。
- 调用前确认机械臂周围没有人员和障碍物。

### 4. 单位分层

| 层级 | 单位 |
| --- | --- |
| ROS / URDF / `ros2_control` | m / rad |
| 下位机固件原始数学接口 | m / rad 或 deg，按具体函数而定 |
| C# 高层 API（`SolveFkMm` / `SolveIkBest` / `ComputeJacobian`） | mm / deg |
| C# `CartesianServo` twist | 线速度 mm/s，角速度 deg/s |
| C# `GetLinkPoints()` | m，供碰撞检测使用 |

### 5. C# 侧关键数值

```text
DH 参数: (0.109, 0.035, 0.146, 0.115, 0.052, 0.072) m

关节下限: [-170, -73, 35, -180, -120, -720] deg
关节上限: [ 170,  90, 180, 180,  120,  720] deg

Rest 姿态:    [0, -73, 180, 0, 0, 0]  # 奇异位形，不用于 IK 回环测试
推荐测试姿态: [30, -50, 90, 20, -30, 45]
```

---

## 安全提示

- 真机运行前清空工作区，先低速、小范围验证，再逐步提高速度。
- `HOME` / `RESET` / `CALIBRATION` 前确认机械臂周边无人、无障碍。
- 硬件默认安全模式不会自动 `start` 或 `auto_home`；显式开启参数前要确认现场状态。
- 软件限位和碰撞检测不能替代硬件急停、机械限位和安全回路。
- Gazebo 模式仅用于展示与链路验证，不应用来验证真实碰撞/动力学行为。

---

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| `/dev/ttyACM0` 打不开 | `ls /dev/ttyACM*`；用户是否加入 `dialout` 组并重新登录 |
| `home` / `reset` 看起来卡住 | 固件是阻塞应答，使用长超时命令，不要按普通查询超时判断 |
| RViz 角度与真机差 90° 或方向相反 | 检查 `position_offsets_rad` 与 `position_signs` |
| MoveIt 时间参数化失败 | 检查 `joint_limits.yaml` 是否包含加速度限制 |
| C# 编译报重复 `AssemblyAttributes` | 主工程必须保留 `<Compile Remove="DummyCli/**" />` |
| C# 3D 视图连杆错位/空白 | 确认 STL 已复制到输出目录 `Meshes/`；必要时用 `--probe-3d` 检查硬件渲染 |
| `rosdep` 找不到包 | 确认已 source `/opt/ros/jazzy/setup.bash` 且 Jazzy apt 源可用 |

---

## 相关文档

- [RESTRUCTURE_SUMMARY.md](./RESTRUCTURE_SUMMARY.md) — 四包重构与 Jazzy 配置调整摘要
- [上下文交接.md](./上下文交接.md) — C# 上位机开发背景、实现细节、已修复问题与验证状态
- [DummyWpfUpper_学习计划与指导书.md](./DummyWpfUpper/DummyWpfUpper_学习计划与指导书.md) — WPF 上位机工程地图与 CLI 使用说明
- [dummy_kinematics_validation/README.md](./src/dummy_kinematics_validation/README.md) — FK/IK 验证节点参数与示例
- [1.md](./1.md) — Code-Behind / MVC / MVP / MVVM / MVVM-C 架构模式笔记，非运行依赖
