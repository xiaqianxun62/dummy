# dummy_kinematics_validation

独立的 IK / FK 验证包，基于现有的 `dummy_moveit_config` 参数加载机器人模型、SRDF 和 `kinematics.yaml`。

## 功能
- `fk`：输入关节角，输出末端位姿
- `ik`：输入末端位姿，求一组 IK 关节解，并回算 FK 误差
- `roundtrip`：先对输入关节做 FK，再对得到的位姿做 IK，最后比较往返误差

## 构建
```bash
colcon build --packages-select dummy_kinematics_validation --symlink-install
source install/setup.bash
```

## 启动示例
### 1) 直接验证 SRDF 里的 reset 姿态（默认）
```bash
ros2 launch dummy_kinematics_validation ik_fk_validation.launch.py
```

### 2) 只做 FK
```bash
ros2 run dummy_kinematics_validation ik_fk_validator \
  --ros-args \
  -p robot_description:="$(xacro $(ros2 pkg prefix dummy_moveit_config)/share/dummy_moveit_config/config/dummy-ros2.urdf.xacro)"
```

更推荐用 launch，因为它会自动加载：
- `robot_description`
- `robot_description_semantic`
- `robot_description_kinematics`

### 3) 用给定关节做 FK
```bash
ros2 run dummy_kinematics_validation ik_fk_validator --ros-args \
  --params-file $(ros2 pkg prefix dummy_moveit_config)/share/dummy_moveit_config/config/kinematics.yaml \
  -p mode:=fk \
  -p group_name:=dummy_arm \
  -p tip_link:=link6_1_1 \
  -p joint_values:="[0.0,-1.2741,1.5708,0.0,0.0,0.0]"
```

### 4) 用给定位姿做 IK
```bash
ros2 run dummy_kinematics_validation ik_fk_validator --ros-args \
  -p mode:=ik \
  -p target_pose_xyz:="[0.20,0.00,0.30]" \
  -p target_pose_quat_xyzw:="[0.0,0.0,0.0,1.0]"
```

## 推荐流程
先用 `roundtrip` 验证 `reset` 或一组已知关节角，再根据输出的：
- IK 求解是否成功
- FK 回算位姿误差
- roundtrip 的关节误差

判断当前 `kinematics.yaml`、URDF、SRDF 是否一致。
