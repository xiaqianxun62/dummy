# DummyWpfUpper 学习计划与指导书

> 面向：学习 WPF 上位机与 dummy 机械臂控制的学生  
> 目标项目：`C:\Users\Administrator\Desktop\Badminton\dummy\dummy_moveit_ws_restructured\DummyWpfUpper`  
> 技术栈：C# 12 / .NET 8 / WPF / MVVM / 串口 ASCII 协议 / 6DOF 运动学 / 轨迹规划 / RRT / Cartesian Servo / WPF 3D  
> 建议总学时：60～90 学时（可按 8～10 周安排）

---

## 0. 这份文档要达到的目标

学完后，学生应当能够：

1. 说清楚 `DummyWpfUpper` 的分层结构、启动流程和模块依赖关系。
2. 独立修改 `MainWindow.xaml` 的布局、样式和绑定，而不破坏已有功能。
3. 解释 `MainViewModel` 中属性通知、命令、状态节流、后台任务和 `Dispatcher` 的作用。
4. 说清楚真机后端、仿真后端和统一接口 `IRobotBackend` 的设计意义。
5. 使用 `DummyCli` 在无硬件条件下验证 IK、RRT、Servo 三个核心算法。
6. 理解固件角度（度）、ROS/URDF 弧度、毫米/米、twist 与关节速度之间的换算关系。
7. 能沿着一条完整调用链阅读代码：从界面按钮 → ViewModel 命令 → 服务层 → 串口/仿真 → 状态回写 UI/3D。
8. 在仿真模式下安全完成示教、RRT 规划、路径播放、笛卡尔 Servo 和 3D 可视化实验。

---

## 1. 安全与学习纪律

**先读这一节，再运行真机。**

1. 所有新功能必须先在内置仿真中验证，再切真机。
2. 软件限位、碰撞检查、红灯报警都只是辅助，**不能替代硬件急停和安全回路**。
3. 真机调试时保持低速、小步长，手不要放在机械臂运动范围内。
4. 执行 `Home`、`Reset`、`Calibration` 前确认工作区无障碍物。
5. 修改 `offset/sign`、DH 参数、限位、碰撞半径前，先用 `iktest` 做 FK→IK→FK 回环验证。
6. 切换到仿真时程序会自动连接仿真并启动轮询；切回真机后需手动选择串口并连接。
7. 若 3D 视图黑屏或渲染异常，程序可能已自动切换到软件渲染；不要把它误判为机械臂故障。

---

## 2. 环境准备与基本运行

### 2.1 必备环境

- Windows 10/11
- .NET 8 SDK
- Visual Studio 2022 或 VS Code + C# 扩展
- Git（可选）
- 串口设备驱动（仅真机需要）

检查环境：

```powershell
dotnet --info
```

### 2.2 编译

在项目根目录执行：

```powershell
dotnet build DummyWpfUpper/DummyWpfUpper.csproj
dotnet build DummyWpfUpper/DummyCli/DummyCli.csproj
```

两个工程说明：

| 工程 | 输出类型 | 作用 |
|---|---|---|
| `DummyWpfUpper.csproj` | `WinExe` | WPF 上位机主程序，`net8.0-windows`，启用 `UseWPF` |
| `DummyCli/DummyCli.csproj` | `Exe` | 跨平台命令行验证工具，复用核心算法源码 |

主工程必须保留：

```xml
<Compile Remove="DummyCli/**" />
```

原因是 `DummyCli` 位于主工程子目录下，若不排除，两个工程的顶层语句和程序集属性会混在一起编译。

CLI 工程通过 `Link` 复用 5 个共享服务文件：

```text
SerialAsciiClient.cs
Dof6Kinematics.cs
TrajectoryPlanner.cs
RrtPlanner.cs
CartesianServo.cs
```

这体现了“核心算法与协议层独立于 UI，可被不同宿主复用”的设计。

### 2.3 无硬件验证

推荐先运行：

```powershell
dotnet run --project DummyWpfUpper/DummyCli/DummyCli.csproj -- iktest 30 -50 90 20 -30 45
dotnet run --project DummyWpfUpper/DummyCli/DummyCli.csproj -- rrttest
dotnet run --project DummyWpfUpper/DummyCli/DummyCli.csproj -- servotest 30 -50 90 20 -30 45 10 0 0 0 0 0 base
```

预期：

- `iktest` 的 FK→IK→FK 回环误差应接近 0。
- `rrttest` 应输出是否成功、迭代次数、路径点数量，并逐段通过碰撞检查。
- `servotest` 应把关节速度积分后的末端速度与目标 twist 对比，误差在容差内。

> 注意：`{0,-73,180,0,0,0}` 是奇异姿态，不适合作为 IK 回环测试输入。

---

## 3. 项目地图

### 3.1 目录结构

```text
DummyWpfUpper/
├── App.xaml / App.xaml.cs                 应用入口、3D 渲染探针
├── MainWindow.xaml / .cs                  主界面、DataContext、连续点动事件
├── DummyWpfUpper.csproj                    WPF 主工程
├── Models/
│   └── RobotState.cs                       机器人状态数据载体
├── ViewModels/
│   ├── MainViewModel.cs                    MVVM 核心、全部功能集成点
│   └── RelayCommand.cs                     ICommand 轻量实现
├── Views/
│   ├── Robot3DWindow.xaml                  Viewport3D、相机、灯光
│   └── Robot3DWindow.xaml.cs               3D 场景构建、路径显示、相机控制
├── Services/
│   ├── IRobotBackend.cs                    真机/仿真统一接口
│   ├── IRobotPathProvider.cs               3D 路径与目标数据源接口
│   ├── SerialAsciiClient.cs                串口 ASCII 协议客户端
│   ├── RobotController.cs                  真机后端
│   ├── SimulationController.cs             软件仿真后端
│   ├── Dof6Kinematics.cs                   DH/FK/IK/雅可比/连杆点
│   ├── TrajectoryPlanner.cs                梯形速度规划 + 碰撞检测
│   ├── RrtPlanner.cs                       RRT-Connect 关节空间规划
│   ├── CartesianServo.cs                   阻尼最小二乘 Servo 数学
│   ├── ServoController.cs                  后台 Servo 控制循环
│   ├── PersistenceService.cs               JSON 配置持久化
│   └── StlMeshLoader.cs                    STL 网格加载
└── DummyCli/
    ├── DummyCli.csproj                      CLI 工程
    └── Program.cs                           命令行参数、离线测试、串口维护命令
```

### 3.2 分层架构

```text
┌──────────────────────────────────────────────┐
│ 表现层：MainWindow / Robot3DWindow / App       │
│ XAML、样式、控件、事件、Viewport3D              │
├──────────────────────────────────────────────┤
│ MVVM 层：MainViewModel / RelayCommand          │
│ 属性通知、命令、状态节流、后台任务、配置          │
├──────────────────────────────────────────────┤
│ 模型与接口：RobotState / IRobotBackend         │
│ IRobotPathProvider / IRobotTargetProvider      │
├──────────────────────────────────────────────┤
│ 服务层：RobotController / SimulationController │
│ SerialAsciiClient / 运动学 / 规划 / Servo / 配置 │
├──────────────────────────────────────────────┤
│ 设备与数据：STM32 下位机 / STL 模型 / config.json│
└──────────────────────────────────────────────┘
```

### 3.3 C# 项目与 ROS 概念对照

本项目是**纯 C# 上位机学习项目**，不依赖 ROS 运行时，但大量概念来自 ROS/MoveIt/URDF 生态。

| ROS/MoveIt 概念 | 本项目对应物 | 说明 |
|---|---|---|
| `/joint_states` | `RobotState.JointsDeg` / `JointVelocitiesDegPerSec` | 关节状态来源不同，语义类似 |
| `hardware_interface` | `IRobotBackend` + `RobotController` | 对接下位机的抽象层 |
| `fake hardware` | `SimulationController` | 无硬件学习模式 |
| URDF 关节原点/轴 | `Robot3DWindow` 中的 `JointOrigin` / `JointAxis` | 用于 3D 连杆链 |
| `joint_limits.yaml` | `KinematicMin` / `KinematicMax` | 关节限位 |
| MoveIt 规划器 | `RrtPlanner` | 简化版关节空间规划 |
| MoveIt 时间参数化 | `TrajectoryPlanner.PlanTimeParameterized()` | 梯形速度曲线 |
| MoveIt Servo | `CartesianServo` + `ServoController` | 笛卡尔速度伺服 |
| RViz | `Robot3DWindow` | WPF 内置 3D 可视化 |
| `position_offsets_rad` / `position_signs` | `OffsetInput` / `SignInput` | 固件语义与 ROS 语义的角度映射 |

这些是**概念对应关系**，不是功能完全等价。移动中的 ROS 链仍以工作区根目录 `README.md` 为准。

---

## 4. 知识点总览

### 4.1 WPF / XAML / MVVM

| 知识点 | 代码位置 | 必须理解的内容 |
|---|---|---|
| Application 生命周期 | `App.xaml.cs` | 无 `StartupUri`，手动 `new MainWindow()`；`OnStartup`；`--probe-3d` 子进程 |
| 渲染能力探测 | `App.xaml.cs` | `RenderOptions.ProcessRenderMode = SoftwareOnly`；探针窗口渲染 3 帧后退出 |
| Grid 布局 | `MainWindow.xaml` | 6 行主布局；`Auto`、`*`、固定高度混合使用 |
| 常用容器 | `MainWindow.xaml` | `StackPanel`、`WrapPanel`、`GroupBox`、`Expander` |
| 动态列表 | `MainWindow.xaml` | `ItemsControl + DataTemplate` 生成 J1~J6 和 X/Y/Z 点动按钮 |
| 样式与资源 | `MainWindow.xaml` | `x:Key`、`StaticResource`、隐式样式、`BasedOn` |
| 控件模板 | `MainWindow.xaml` | `ControlTemplate`、`TemplateBinding`、`ContentPresenter` |
| 触发器 | `MainWindow.xaml` | `Trigger`、`DataTrigger`，例如按 `IsConnected` 改变状态胶囊颜色 |
| 数据绑定 | `MainWindow.xaml` | `Text`、`SelectedItem`、`IsChecked`、`Command` 绑定 |
| 更新触发时机 | `MainWindow.xaml` | `UpdateSourceTrigger=PropertyChanged`，适用于实时解析目标输入 |
| `DataContext` | `MainWindow.xaml.cs` | `DataContext = new MainViewModel()` 是绑定生效的前提 |
| `INotifyPropertyChanged` | `MainViewModel.cs` | `[CallerMemberName]` + `PropertyChanged?.Invoke` |
| `ObservableCollection<T>` | `MainViewModel.cs` | 端口、日志、点动行、示教点、命名姿态自动刷新 UI |
| `ICommand` | `RelayCommand.cs` | `Execute`、`CanExecute`、`RaiseCanExecuteChanged` |
| 命令启用状态 | `MainViewModel.cs` | `CanConnect()`、`CanOperate()`、`CanManualMove()`、`RefreshCommands()` |
| 代码隐藏事件 | `MainWindow.xaml.cs` | 连续点动用 `PreviewMouseDown/Up`、`MouseLeave`，按钮 `Tag` 携带索引 |
| `Dispatcher` | `MainViewModel.cs` | `CheckAccess()` + `Invoke` 保证后台线程安全回写 UI |
| `DispatcherTimer` | `MainViewModel.cs` / 3D 窗口 | 100ms 状态刷新、33ms 3D 刷新、80ms 轨迹预览、80ms 探针帧计数 |
| WPF 3D | `Robot3DWindow.xaml/.cs` | `Viewport3D`、相机、灯光、`ModelVisual3D`、`Transform3DGroup` |
| 3D 几何生成 | `Robot3DWindow.xaml.cs` / `StlMeshLoader.cs` | STL 解析、网格缓存、路径管、八面体端点标记 |

### 4.2 C# / .NET 工程实践

| 知识点 | 说明 |
|---|---|
| `net8.0-windows` + `UseWPF` | WPF 只能运行在 Windows 目标框架上 |
| Nullable + ImplicitUsings | 现代 C# 工程默认配置，减少样板代码 |
| 接口隔离 | UI 依赖 `IRobotBackend`，3D 依赖 Provider 接口 |
| 委托字段保存事件处理器 | `_backendLogHandler`、`_backendStateHandler` 便于正确退订 |
| `IDisposable` | `SerialAsciiClient` 管理串口资源 |
| `record` | `TrajectoryPoint`、`RrtResult`、`ServoResult`、`CollisionInfo` 等结果类型 |
| `CancellationTokenSource` | 播放、点动、Servo、断线监控、轮询的取消机制 |
| `lock` + 快照 | 后台高频状态先克隆再入锁，UI 定时取走，避免直接冲击界面 |
| `InvariantCulture` | 避免中文/本地化环境把小数点输出成逗号 |
| `double.TryParse` / `Math.Clamp` | 防御性解析与数值边界保护 |
| 失败降级 | 配置读写失败不影响主功能，渲染探测失败切软件渲染 |

### 4.3 dummy / 机器人控制知识点

| 知识点 | 代码位置 | 必须理解的内容 |
|---|---|---|
| 关节语义 | `RobotState.cs` | 6 个固件关节角，单位度 |
| 坐标映射 | `RobotController.cs` | `RViz = 真机 × sign - offset`；默认 offset J3=1.57079rad，sign J5/J6=-1 |
| DH 参数 | `Dof6Kinematics.cs` | `(0.109, 0.035, 0.146, 0.115, 0.052, 0.072)` 米 |
| 正运动学 FK | `Dof6Kinematics.SolveFk()` | 旋转矩阵连乘，叠加基座/肩/肘/腕偏置 |
| 逆运动学 IK | `Dof6Kinematics.SolveIk()` | 肩解、余弦定理、腕部分解，得到 8 组解 |
| IK 选解 | `SolveIkBest()` | 过滤限位，取与当前关节“最大单轴差”最小的解 |
| 数值雅可比 | `ComputeJacobian()` | 每轴微扰 `1e-4` 度，FK 差分得到 6×6 矩阵 |
| 碰撞检测 | `TrajectoryPlanner` | 连杆采样、地面检查、非相邻连杆间距离检查 |
| 时间参数化 | `PlanTimeParameterized()` | 梯形速度曲线：加速段、匀速段、减速段 |
| RRT-Connect | `RrtPlanner.cs` | 双树扩展、`goalBias=0.15`、`stepDeg=12`、贪心平滑 |
| Cartesian Servo | `CartesianServo.cs` | tool twist → base twist；DLS：\(dq=(J^TJ+\lambda^2I)^{-1}J^Tv\) |
| Servo 循环 | `ServoController.cs` | 约 30Hz：twist→关节速度→积分→限位/碰撞检查→`MoveJ` |
| 串口协议 | `SerialAsciiClient.cs` | 8N1、ASCII、CRLF、`ok ...` 查询应答、整数队列余量 ACK |
| 真机后端 | `RobotController.cs` | 限位、重复目标过滤、状态轮询、速度差分、角度映射 |
| 仿真后端 | `SimulationController.cs` | 按最大关节速度向目标逼近，FK 更新末端位姿 |
| 配置持久化 | `PersistenceService.cs` | `%AppData%\DummyWpfUpper\config.json` |
| 3D 模型加载 | `StlMeshLoader.cs` | 二进制/ASCII STL，毫米转米，`Freeze()` 网格 |

---

## 5. 学习计划

### 总览表

| 模块 | 主题 | 建议学时 | 核心产出 |
|---|---|---:|---|
| M0 | 安全、环境与命令行验证 | 2 | 两个工程编译通过，三个离线测试跑通 |
| M1 | 工程结构与启动链 | 4 | 启动流程图，渲染探针说明 |
| M2 | XAML 布局与样式 | 6 | 新增一个绑定控件和一个按钮样式 |
| M3 | MVVM、通知与命令 | 8 | 新增属性 + 命令 + `CanExecute` |
| M4 | 状态模型与后端抽象 | 6 | 真机/仿真调用链对比表 |
| M5 | 串口协议与 DummyCli | 8 | 协议模拟器或新 CLI 命令设计 |
| M6 | 6DOF 运动学 | 10 | IK 回环实验报告 |
| M7 | 碰撞与轨迹规划 | 8 | 梯形速度曲线实验 |
| M8 | RRT 路径规划 | 8 | RRT 参数对比实验 |
| M9 | Cartesian Servo | 8 | base/tool twist 对比实验 |
| M10 | 3D 可视化 | 8 | 自定义路径标记或相机控制 |
| M11 | 线程、持久化与综合实战 | 10 | 综合调试报告与扩展功能 |

---

### M0 安全、环境与命令行验证（2 学时）

**学习目标**：建立“仿真优先”的习惯，确认开发环境可用。

**阅读**：

- 根目录 `README.md`
- `DummyWpfUpper.csproj`
- `DummyCli/DummyCli.csproj`
- `上下文交接.md` 第 1、3、5 节

**掌握知识点**：

- 真机与仿真的边界。
- `WinExe` 与 `Exe` 的差别。
- `Compile Remove` 与 `Compile Include Link`。
- CLI 的三个离线测试入口：`iktest`、`rrttest`、`servotest`。
- 退出码：0 成功、1 参数/测试失败、2 串口/运行异常。

**实验任务**：

1. 编译两个工程。
2. 运行 `iktest 30 -50 90 20 -30 45`，记录回环误差。
3. 运行 `rrttest`，记录迭代次数与路径点数。
4. 运行 `servotest`，记录线速度和角速度误差。

**验收标准**：两个工程 0 错误；三个测试均能成功退出；学生能解释为什么不用 home 奇异姿态做 IK 测试。

---

### M1 工程结构与启动链（4 学时）

**阅读**：

- `App.xaml`
- `App.xaml.cs`
- `MainWindow.xaml.cs`
- `ViewModels/MainViewModel.cs` 构造函数部分

**掌握知识点**：

- `App.xaml` 没有 `StartupUri`，窗口在 `OnStartup()` 中手动创建。
- `MainWindow` 构造时设置 `DataContext = new MainViewModel()`。
- `MainViewModel` 构造时创建串口客户端、真机后端、仿真后端、规划器、Servo 和约 38 个命令。
- `--probe-3d` 是独立子进程模式：主进程启动探针，探针渲染 3 帧后退出；失败则主进程启用软件渲染。
- 后端事件通过固定委托字段订阅，便于退订。

**必须画出的流程图**：

```text
程序启动
  → App 构造函数检查 --probe-3d
  → 普通模式：ProbeCanUseHardwareRendering()
  → OnStartup() 创建 MainWindow
  → MainWindow.InitializeComponent()
  → DataContext = new MainViewModel()
  → MainViewModel 构造：定时器 / 后端 / 规划器 / 命令 / 配置 / 断线监控
  → 显示窗口
```

**实验任务**：

1. 在 `App.xaml` 增加一个全局画刷资源，并在某个按钮背景中引用。
2. 写出普通启动与探针启动的差异。
3. 解释为什么切换后端时要退订旧后端事件。

**验收标准**：能口述启动链；能说明渲染探针存在的意义；修改资源后界面正常显示。

---

### M2 XAML 布局与样式（6 学时）

**阅读**：

- `MainWindow.xaml`
- `Views/Robot3DWindow.xaml`

**掌握知识点**：

- `Grid.RowDefinitions` 六行布局：串口、控制、设置、Servo、状态/点动/示教、日志。
- 第 5 行三列比例 `1.2* / 1* / 1.4*`。
- `StackPanel` 横向按钮组；`WrapPanel` 设置项换行。
- `GroupBox` 卡片式分区；`Expander` 折叠高级参数。
- `ItemsControl + DataTemplate` 根据 `JogRows` 和 `CartesianAxes` 动态生成按钮。
- `ListBox` 显示示教点和日志；`ComboBox` 选择串口、坐标系、命名姿态。
- `x:Key`、`StaticResource`、隐式样式、`BasedOn`。
- `ControlTemplate`、`TemplateBinding Padding`、`ContentPresenter`。
- `Trigger`、`DataTrigger`，例如 `IsConnected=True` 时状态胶囊变绿。

**实验任务**：

1. 在设置区新增一个 `TextBox`，绑定到现有字符串属性。
2. 新增 `WarningButton` 样式，基于 `PrimaryButton`，颜色改为橙色。
3. 给日志 `ListBox` 增加交替行背景，并解释用到了哪些 XAML 概念。
4. 修改 `ItemsControl` 的 `ItemsSource`，观察 J1~J6 按钮生成方式。

**验收标准**：能独立解释一个样式从资源定义到控件应用的全过程；新增控件能正常双向绑定。

---

### M3 MVVM、通知与命令（8 学时）

**阅读**：

- `ViewModels/RelayCommand.cs`
- `ViewModels/MainViewModel.cs` 的属性区、命令注册区、`RefreshCommands()`
- `MainWindow.xaml.cs` 的点动事件

**掌握知识点**：

- `DataContext` 是 XAML 与 ViewModel 的桥梁。
- `INotifyPropertyChanged` 让 UI 感知属性变化。
- `[CallerMemberName]` 自动传入属性名，避免手写字符串。
- `ObservableCollection<T>` 用于会增删的集合。
- `ICommand` 的 `Execute` / `CanExecute` / `CanExecuteChanged`。
- `CanConnect()`、`CanOperate()`、`CanManualMove()` 构成按钮启用逻辑。
- `RefreshCommands()` 统一刷新命令可用状态。
- 普通按钮用 `Command`；按住连续点动用鼠标事件 + `Tag` 参数 + `StartJointJog()` / `StopJog()`。

**实验任务**：

1. 新增 `StatusNote` 属性，在按钮点击后更新 TextBlock。
2. 给新命令增加 `CanExecute`：只有连接后才可执行。
3. 在连接/断开时调用 `RaiseCanExecuteChanged()`，验证按钮灰显/恢复。
4. 解释为什么连续点动没有使用普通 `Command`。

**验收标准**：属性变化立刻反映到 UI；命令灰显逻辑正确；能说清事件与命令的适用边界。

---

### M4 状态模型与后端抽象（6 学时）

**阅读**：

- `Models/RobotState.cs`
- `Services/IRobotBackend.cs`
- `Services/RobotController.cs`
- `Services/SimulationController.cs`
- `Services/IRobotPathProvider.cs`

**掌握知识点**：

- `RobotState` 包含 6 关节角、6 关节速度、6 位姿、队列余量、连接状态。
- `IRobotBackend` 统一 Connect/Disconnect、轮询、Start/Stop/Home、MoveJ/MoveL、坐标映射、电机维护命令。
- 真机后端负责限位、目标去重、状态轮询、速度差分、固件角↔ROS 弧度映射。
- 仿真后端按最大速度逼近目标，并用 FK 生成位姿。
- `IRobotPathProvider` / `IRobotTargetProvider` 将 3D 视图与 ViewModel 解耦。
- 后端状态事件到达 ViewModel 后要克隆，避免共享数组被后续轮询覆盖。

**必须掌握的调用链**：

```text
Connect()
  → SerialAsciiClient.Open(port) 或 SimulationController.Connect()
  → RobotController/SimulationController.StartPolling()
  → 后台循环读取/计算状态
  → StateUpdated 事件
  → MainViewModel.OnStateUpdated() 加锁克隆
  → DispatcherTimer 100ms 取快照
  → JointText / VelocityText / PoseText 更新
```

**实验任务**：

1. 制作“真机 vs 仿真”方法对照表：Start、Stop、Home、MoveJ、MoveL、Configure。
2. 在仿真模式下依次执行连接、Start、发送关节目标、Stop，记录日志。
3. 说明为什么切换后端时要重建 Servo。

**验收标准**：能在不查看代码的情况下画出状态回写链；能解释后端抽象带来的可测试性。

---

### M5 串口协议与 DummyCli（8 学时）

**阅读**：

- `Services/SerialAsciiClient.cs`
- `DummyCli/Program.cs`

**掌握知识点**：

- 串口参数：8N1、`Parity.None`、`StopBits.One`、`Handshake.None`、ASCII、CRLF。
- `SendAndWait()`：`_sync` 锁、清空缓冲、`WriteLine`、按 deadline 循环 `ReadLine`、accept 谓词判断应答、超时异常。
- 查询命令要求 `ok ` 开头且数值个数匹配；查询失败重试一次。
- 运动命令返回整数队列余量。
- `Fmt()` 使用 `F2` 和 `InvariantCulture`。
- CLI 参数解析、顶层语句、命令分派。
- 维护命令：`start`、`stop`、`home`、`reset`、`disable`、`set-home-current`、`cmdmode`、`getjpos`、`getlpos`、`dce-*`、`reboot`。
- 离线命令：`iktest`、`rrttest`、`servotest`。

**协议速查**：

| 方法 | 命令 | 应答 |
|---|---|---|
| `GetJointPositions()` | `#GETJPOS` | `ok` + 6 数 |
| `GetLinearPose()` | `#GETLPOS` | `ok` + 6 数 |
| `MoveJoint()` | `>j1,j2,j3,j4,j5,j6[,speed]` | 整数队列余量 |
| `MovePose()` | `@x,y,z,a,b,c[,speed]` | 整数队列余量 |
| 系统命令 | `!START` / `!STOP` / `!HOME` 等 | 包含 ACK 文本 |
| 电机参数 | `#SET_DCE_KP/KI/KD <node> <value>` | 参数名确认 |
| 重启电机 | `#REBOOT <node>` | `REBOOT MOTOR` 确认 |

**实验任务**：

1. 根据 `#GETJPOS` 应答格式写一个假串口模拟器。
2. 为新命令 `get-status` 设计协议、CLI 分支、预期输出。
3. 修改 `QueryTimeoutInput`，观察超时日志与重试行为。

**验收标准**：学生能解释一条 `sendj` 命令从 CLI 到下位机再回到界面的完整链路。

---

### M6 6DOF 运动学（10 学时）

**阅读**：

- `Services/Dof6Kinematics.cs`
- `DummyCli/Program.cs` 中 `RunIkTest()`

**掌握知识点**：

- `Joint6D`：6 关节角。
- `Pose6D`：位置、欧拉角、9 元素行主序旋转矩阵。
- `IkSolves`：8 组解与有效性标志。
- 底层 `SolveFk()` 使用米/弧度；高层 `SolveFkMm()`、`SolveIkBest()` 使用毫米/度。
- FK：6 个 3×3 旋转矩阵连乘，叠加几何偏置。
- IK：腕点位置求 J1 两解，平面二连杆余弦定理求 J2/J3，腕部矩阵求 J4/J5/J6，输出 8 组解。
- 奇异位形会导致 IK 解失效或跳变。
- 选解策略：过滤限位后取“最大单轴差”最小者，仿固件 `MoveL()`。
- 数值雅可比：位置行 mm/deg，姿态行 deg/deg。

**实验任务**：

1. 对 `{30,-50,90,20,-30,45}` 和其他 3 组非奇异姿态运行 `iktest`。
2. 记录每组的 FK→IK→FK 回环误差和选中解。
3. 修改一个 DH 参数，观察回环是否仍一致。
4. 解释为什么不用欧氏距离选解。

**验收标准**：能说清 8 组解的来源；能解释单位边界；能说明 home 姿态为何奇异。

---

### M7 碰撞与轨迹规划（8 学时）

**阅读**：

- `Services/TrajectoryPlanner.cs`
- `MainViewModel.cs` 中播放路径相关方法

**掌握知识点**：

- 连杆近似为 7 段 L0~L6，每段有半径、安全余量、最低 Z。
- `CheckCollisionFwDeg()` 基于 `Dof6Kinematics.GetLinkPoints()`，输入固件角度，是当前实际使用的碰撞检查。
- 旧的 `CheckCollision()` 使用 URDF 风格坐标链，与下位机 FK 不一致，容易产生误报，学习时只需了解其历史背景。
- 自碰撞检查跳过零长度段与相邻共享端点。
- `PlanTimeParameterized()` 取最大单轴位移，计算加速、匀速、减速时间。
- `TrajectoryPoint(JointsFwDeg, DelayMs)` 描述每个控制周期的目标关节角与延时。
- 普通插值用 `RobotController.InterpolateJointPath()`，包含起终点。

**实验任务**：

1. 修改 `MaxVelocityInput` / `MaxAccelInput`，观察播放耗时变化。
2. 关闭碰撞检查播放一次，再打开播放一次，比较日志。
3. 修改碰撞半径，观察是否出现规划失败。

**验收标准**：能画出梯形速度曲线；能解释为什么碰撞检查要在发送前和 Servo 每步都做。

---

### M8 RRT 路径规划（8 学时）

**阅读**：

- `Services/RrtPlanner.cs`
- `MainViewModel.cs` 中 `PlanPath()` / `PlayPlannedPath()`
- `DummyCli/Program.cs` 中 `RunRrtTest()`

**掌握知识点**：

- 关节空间 6 维规划，路径点单位是固件角度（度）。
- 两棵树：起点树、终点树，交替扩展。
- `goalBias=0.15` 提高向目标采样的概率。
- `Extend()` 单步扩展；`Connect()` 连续连接。
- `EdgeValid()` 按 `stepDeg` 采样，检查限位与外部碰撞函数。
- `ExtractPath()` 通过父节点回溯路径。
- `Smooth()` 贪心跳过中间点，减少路径点数量。
- `RrtResult` 包含 `Path`、`Iterations`、`Success`、`Message`。
- 规划在后台 `Task.Run` 中执行，结果通过 `Dispatcher` 回写 UI。

**实验任务**：

1. 用 `rrttest` 比较 3 组起止点的迭代次数与路径点数。
2. 把 `goalBias`、`stepDeg`、`maxIterations` 改成不同值，观察成功率。
3. 在 UI 中规划路径，打开 3D 窗口查看橙色路径与起终点标记。

**验收标准**：能解释 RRT-Connect 为什么比单树 RRT 更快；能说明路径平滑为何仍要做碰撞检查。

---

### M9 Cartesian Servo（8 学时）

**阅读**：

- `Services/CartesianServo.cs`
- `Services/ServoController.cs`
- `MainViewModel.cs` 中 Servo 相关属性与命令
- `DummyCli/Program.cs` 中 `RunServoTest()`

**掌握知识点**：

- twist：线速度 mm/s，角速度 deg/s；关节速度 deg/s。
- `frame=tool` 时，先用 FK 的旋转矩阵把 twist 旋到 base：`twist_base = R * twist_tool`。
- 数值雅可比 \(J\) 把关节速度映射到末端 twist。
- 阻尼最小二乘：

\[
dq = (J^TJ + \lambda^2 I)^{-1}J^T v
\]

- `damping` 默认 0.05，构造时下限 0.001。
- `SolveLinearSystem()` 使用带部分主元的高斯-约当消元。
- `ServoController` 默认约 33ms 一个周期：
  1. 复制 twist、frame、当前关节；
  2. `CartesianServo.Compute()` 得关节速度；
  3. 欧拉积分得下一关节角；
  4. 限位检查；
  5. `CheckCollisionFwDeg()`；
  6. `backend.MoveJ(next, null)`；
  7. `Task.Delay()`；
  8. `CancellationToken` 停止。

**实验任务**：

1. 运行 `servotest ... 10 0 0 0 0 0 base`，记录误差。
2. 改用 `tool` 坐标系，解释旋转矩阵带来的差异。
3. 增大 damping，观察关节速度与末端验证误差的变化。
4. 在仿真模式下启动/停止 Servo，观察日志和关节角变化。

**验收标准**：能写出 DLS 公式；能解释阻尼为何能改善奇异位形附近的稳定性；能说出 Servo 每周期的安全检查。

---

### M10 3D 可视化（8 学时）

**阅读**：

- `Views/Robot3DWindow.xaml`
- `Views/Robot3DWindow.xaml.cs`
- `Services/StlMeshLoader.cs`

**掌握知识点**：

- `Viewport3D`、`PerspectiveCamera`、`AmbientLight`、两个 `DirectionalLight`。
- 机器人根节点绕 Z 轴 180°，对应 `world_joint` 约定。
- 6 个连杆父子嵌套，每个连杆有 `Transform3DGroup`。
- 变换顺序：先旋转再平移，对应 URDF 语义 `T(J)·R`；顺序写反会导致连杆“散架”。
- 目标关节用半透明橙色虚影显示。
- 规划路径用橙色细管 + 八面体点；示教路径用青色；起点绿色、终点红色。
- `CreateTube()` 用叉积构造局部正交基并生成三角形索引。
- `ComputeToolPoint()` 使用与场景树一致的变换链计算末端点，保证轨迹与模型对齐。
- STL 支持二进制和 ASCII；默认缩放 0.001（毫米转米）；`MeshCache` 避免重复解析；`Freeze()` 提升共享只读资源性能。
- 鼠标左键旋转、右键平移、滚轮缩放相机。
- 轨迹预览 80ms/点，预览时暂停状态更新，结束后恢复。
- 窗口关闭时退订事件并停止定时器。

**实验任务**：

1. 修改初始相机 yaw/pitch/distance，观察视角变化。
2. 把路径点八面体换成球体或立方体。
3. 解释为什么 `Rotate` 必须放在 `Translate` 之前。
4. 加载一个新 STL 并观察缩放是否正确。

**验收标准**：3D 连杆连贯；路径与末端对齐；相机操作正常；窗口关闭无异常。

---

### M11 线程、持久化与综合实战（10 学时）

**阅读**：

- `Services/PersistenceService.cs`
- `MainViewModel.cs` 中 `LoadConfig()`、`BuildConfig()`、`SaveConfig()`、`StartConnectionMonitor()`、`StateUiTimer_Tick()`、`DispatchIfNeeded()`
- `App.xaml.cs` 渲染探针

**掌握知识点**：

- `AppConfig` 字段：串口参数、目标输入、碰撞参数、轮询频率、速度/加速度、示教点、命名姿态、自动重连、offset/sign。
- 配置路径：`%AppData%\DummyWpfUpper\config.json`。
- `Load()` / `Save()` 捕获异常，保证配置问题不影响控制功能。
- 后台任务：真机/仿真轮询、路径播放、连续点动、RRT、Servo、断线监控。
- UI 更新原则：后台线程不直接改 UI，使用 `Dispatcher.Invoke/BeginInvoke`。
- 状态节流：高频事件 → 克隆 → 锁保护缓冲 → UI 定时器消费。
- 自动重连：每秒检查；手动断开、仿真模式、已连接时跳过。
- 日志最多保留 200 条，支持导出为文本。
- 资源生命周期：窗口关闭退订；串口 `IDisposable`；定时器停止；网格冻结与缓存。

**综合任务（三选一）**：

1. **线程审计表**：列出每类后台任务的线程、锁、取消方式、UI 回写方式、异常路径。
2. **配置字段解释器**：读取本机 `config.json`，逐项解释字段含义和对运行的影响。
3. **小功能扩展**：新增一个“安全归零”按钮，依次 Stop → 发送安全姿态 → 写日志 → 保存配置。

**验收标准**：综合任务能运行、能解释、能回退；无跨线程 UI 异常；无事件重复订阅导致的重复日志。

---

## 6. 源码阅读指导

### 6.1 建议阅读顺序

不要一开始就逐行读 `MainViewModel.cs`。推荐按下面顺序建立心智模型：

```text
App.xaml.cs
  → MainWindow.xaml.cs
  → RelayCommand.cs
  → RobotState.cs
  → IRobotBackend.cs / IRobotPathProvider.cs
  → SerialAsciiClient.cs
  → SimulationController.cs
  → Dof6Kinematics.cs
  → TrajectoryPlanner.cs / RrtPlanner.cs
  → CartesianServo.cs / ServoController.cs
  → Robot3DWindow.xaml.cs
  → MainViewModel.cs（最后通读）
```

原因：`MainViewModel` 是集成点，先读会同时遇到 WPF、串口、数学、线程和 3D，认知负担过大。

### 6.2 八条关键调用链

#### 链 1：启动

```text
App.OnStartup → new MainWindow → InitializeComponent
→ new MainViewModel → DataContext 生效 → Window.Show
```

#### 链 2：刷新串口并连接

```text
RefreshPortsCommand → SerialPort.GetPortNames()
→ SelectedPort → ConnectCommand
→ _client.Open(port) → _hardware.Connect()
→ SubscribeBackend → StartPolling → UpdateConnectionState → RefreshCommands
```

#### 链 3：发送关节角

```text
JointTargetInput → TryParseSix → SendJointArray
→ _controller.MoveJ(joints, speed)
→ 限位检查 / 重复目标过滤
→ SerialAsciiClient.MoveJoint → SendAndWait(">...")
→ 返回队列余量 → QueueText
```

#### 链 4：发送笛卡尔位姿

```text
PoseTargetInput → TryParseSix
→ Dof6Kinematics.SolveIkBest(pose, current, min, max)
→ 得到固件关节角 → _controller.MoveJ
```

#### 链 5：RRT 规划与播放

```text
PlanPathCommand → CanManualMove
→ 当前关节 + 目标关节
→ Task.Run(() => _rrt.Plan(...))
→ RrtResult → _plannedPath → PathsChanged
→ PlayPlannedCommand → 插值/时间参数化 → MoveJ 逐点发送
```

#### 链 6：Servo

```text
TwistInput + ServoFrameInput
→ StartServoCommand → ServoController.Start()
→ 后台 33ms 循环
→ CartesianServo.Compute → 关节速度
→ 积分 → 限位/碰撞检查 → _backend.MoveJ
→ StopServoCommand → CancellationToken 取消
```

#### 链 7：3D 视图刷新

```text
后端 StateUpdated → MainViewModel.OnStateUpdated → 克隆
→ Robot3DWindow._poseLock 保存最新关节角
→ 33ms DispatcherTimer → ApplyPose → Transform3DGroup
→ 若目标/路径变化 → BeginInvoke(RefreshPaths / RefreshGhost)
```

#### 链 8：配置保存

```text
界面输入 → ViewModel 属性
→ SaveConfigCommand → BuildConfig()
→ PersistenceService.Save(config.json)
→ LoadConfig() 在下次启动时恢复
```

---

## 7. 必做实验清单

| 编号 | 实验 | 无硬件可做 | 关键验收 |
|---|---|---|---|
| E1 | 三个 CLI 离线测试 | 是 | 输出误差和路径信息 |
| E2 | 启动流程图 | 是 | 说清探针与软件渲染回退 |
| E3 | XAML 新样式与绑定控件 | 是 | 界面刷新正确 |
| E4 | MVVM 新属性 + 命令 + CanExecute | 是 | 按钮灰显逻辑正确 |
| E5 | 仿真模式连接、Start、MoveJ、Stop | 是 | 关节状态变化 |
| E6 | 假串口模拟器 | 是 | 能返回 `#GETJPOS` 格式 |
| E7 | IK 多姿态回环 | 是 | 误差接近 0 |
| E8 | RRT 参数对比 | 是 | 记录迭代次数与路径点数 |
| E9 | Servo base/tool 对比 | 是 | 误差在容差内 |
| E10 | 3D 路径标记替换 | 是 | 路径显示正确 |
| E11 | 示教点保存与播放 | 是 | 重启后配置仍存在 |
| E12 | 线程审计表 | 是 | 每类后台任务有清晰记录 |

---

## 8. 常见错误与排查

| 现象 | 可能原因 | 排查方向 |
|---|---|---|
| 绑定不生效 | `DataContext` 未设置或属性名写错 | 检查 `MainWindow` 构造函数和绑定路径 |
| 修改属性后 UI 不变 | 未调用 `OnPropertyChanged()` | 检查 setter |
| 按钮不灰显 | `CanExecute` 未刷新 | 检查 `RefreshCommands()` |
| 后台线程改 UI 抛异常 | 未使用 `Dispatcher` | 检查 `DispatchIfNeeded()` / `BeginInvoke` |
| IK 回环误差大 | 奇异姿态或单位错误 | 换非奇异姿态；检查 mm/deg/m/rad |
| 真机与 RViz 角度不一致 | offset/sign 配错 | 对照 `RViz = 真机 × sign - offset` |
| 碰撞误报多 | 用了不匹配的坐标链或半径过大 | 使用 `CheckCollisionFwDeg()` |
| RRT 总是失败 | 目标超限位或在障碍中 | 检查 `KinematicMin/Max` 和碰撞参数 |
| Servo 速度异常放大 | DLS 公式写错 | 使用关节空间 `(JᵀJ+λ²I)⁻¹Jᵀv` |
| 3D 连杆散架 | `Transform3DGroup` 中 Rotate/Translate 顺序错 | 先 Rotate 再 Translate |
| STL 模型过大/过小 | 缩放系数错误 | 默认毫米转米 0.001 |
| 小数点变逗号 | 本地化格式影响 | 使用 `CultureInfo.InvariantCulture` |
| 编译重复程序集属性 | CLI 子目录未排除 | 保留 `<Compile Remove="DummyCli/**" />` |
| 事件重复触发 | 切换后端未退订旧事件 | 使用固定委托字段并退订 |

---

## 9. 阶段考核建议

### 9.1 口头检查

随机抽取以下问题，学生需不看代码回答：

1. 为什么 `App.xaml` 没有 `StartupUri`？
2. `MainViewModel` 构造时做了哪些初始化？
3. `IRobotBackend` 为什么同时给真机和仿真实现？
4. `StateUpdated` 到达 ViewModel 后为什么要克隆？
5. 8 组 IK 解从哪里来？如何选解？
6. `CheckCollisionFwDeg()` 为什么是实际使用的碰撞检查？
7. RRT 的 `goalBias` 有什么作用？
8. tool twist 为什么要先转到 base？
9. 为什么连续点动用鼠标事件而不是 `Command`？
10. 3D 中 Rotate 和 Translate 的顺序为什么重要？

### 9.2 实操检查

在仿真模式下完成：

1. 连接仿真、Start、发送一组非奇异关节角、记录位姿。
2. 保存两个示教点并播放。
3. 规划一条 RRT 路径并在 3D 窗口显示。
4. 启动 Servo 5 秒后停止。
5. 导出日志并说明关键事件顺序。

### 9.3 代码修改检查

给出一个未见过的小需求，例如：

- 新增“速度倍率”输入框并影响点动速度。
- 新增“清空日志”按钮。
- 新增一个命名姿态下拉项。
- 给 3D 路径增加线宽或颜色切换。

要求：能定位文件、完成最小修改、编译通过、在仿真中验证。

---

## 10. 术语表

| 术语 | 含义 |
|---|---|
| FK | Forward Kinematics，正运动学，关节角 → 末端位姿 |
| IK | Inverse Kinematics，逆运动学，末端位姿 → 关节角 |
| DH | Denavit-Hartenberg，机器人连杆坐标系参数化方法 |
| Jacobian | 雅可比矩阵，关节速度 ↔ 末端速度 |
| DLS | Damped Least Squares，阻尼最小二乘 |
| twist | 6 维速度：3 维线速度 + 3 维角速度 |
| RRT | Rapidly-exploring Random Tree，快速扩展随机树 |
| RRT-Connect | 双向 RRT 变体，从起点和终点同时建树 |
| Servo | 这里指笛卡尔速度伺服，而非电机底层驱动器 |
| MoveJ | 关节空间运动指令 |
| MoveL | 笛卡尔/直线运动指令，本项目通过 C# IK 转成 MoveJ |
| ACK | 应答确认 |
| 8N1 | 8 数据位、无校验、1 停止位 |
| CRLF | `\r\n` 行结束符 |
| Provider | 为视图提供只读数据的接口 |
| Dispatcher | WPF UI 线程的消息循环与调度器 |

---

## 11. 推荐扩展方向

完成本指导书后，可选择：

1. 手爪 ASCII 接口：打开/关闭、状态反馈、与示教点联动。
2. fibre 二进制协议客户端：对比 ASCII 与二进制协议的帧同步、校验和、吞吐。
3. 路径时间最优平滑：B 样条或三次样条替代简单线性插值。
4. 自定义障碍物避障：把工作区障碍物加入碰撞检查。
5. 更完整的日志系统：结构化日志、等级过滤、按模块检索。
6. 单元测试：为 FK/IK、RRT、DLS、协议解析建立 xUnit 测试工程。
7. 硬件安全反馈：急停状态、限位开关、碰撞停机联动；再次强调软件不能替代安全回路。

---

## 12. 一句话学习路线

**先跑通命令和仿真，再学界面与 MVVM；先理解状态与协议，再进入运动学、规划、Servo；最后把 3D、线程和工程细节串成完整系统。**
