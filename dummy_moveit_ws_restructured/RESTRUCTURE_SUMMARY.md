# Restructure Summary

本次重构基于你当前的四包工作空间做“同包重构”，没有再引入第五个业务包。

## 方向

- 一套 `dummy_ros2_description` 统一模型
- `dummy_moveit_config` 只负责 MoveIt 配置和示例
- `dummy_hardware_interface` 负责 ros2_control 真机插件与 ASCII 维护工具
- `dummy_bringup` 负责 fake / real / simulation 的统一启动

## 关键变化

### 1. 工作空间清理
- 去掉了 `build/ install/ log/` 等构建污染
- 保留源码与必要文档

### 2. MoveIt Jazzy 化
- `ompl_planning.yaml` 改为 `planning_plugins` / `request_adapters` / `response_adapters` 的数组格式
- `joint_limits.yaml` 补充 acceleration limits，避免时间参数化失败
- `bringup.launch.py` 改为优先使用 `MoveItConfigsBuilder(...).to_moveit_configs()` 聚合参数

### 3. ros2_control / 真机链收敛
- 在 `dummy-ros2.ros2_control.xacro` 中加入 `slow_command_timeout_ms`
- 真机默认改成安全模式：
  - `start_on_activate: false`
  - `auto_home_on_activate: false`
- `dummy_hardware_interface` 在写命令时等待固件队列确认
- `!HOME` / `!RESET` 改为长超时等待，适配固件阻塞应答语义

### 4. 板端维护阶段工具
- 新增 `ros2 run dummy_hardware_interface dummy_ascii_cli ...`
- 用来做 `start / home / reset / getjpos / set-home-current` 等板端维护动作

## 推荐启动方式

```bash
# 维护阶段
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 start
ros2 run dummy_hardware_interface dummy_ascii_cli --port /dev/ttyACM0 getjpos

# ROS 接管阶段
ros2 launch dummy_bringup hardware.launch.py serial_port:=/dev/ttyACM0
```
