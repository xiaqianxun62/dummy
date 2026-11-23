using System.Collections.ObjectModel;
using System.ComponentModel;
using Microsoft.Win32;
using System.Globalization;
using System.IO;
using System.Runtime.CompilerServices;
using System.Windows.Input;
using DummyWpfUpper.Models;
using DummyWpfUpper.Services;

namespace DummyWpfUpper.ViewModels;

public sealed class MainViewModel : INotifyPropertyChanged, IRobotPathProvider
{
    public event Action? PathsChanged;

    public IReadOnlyList<double[]>? PlannedPath => _plannedPath.Count >= 2
        ? _plannedPath.Select(p => (double[])p.Clone()).ToList()
        : null;

    public IReadOnlyList<double[]>? TaughtPath => _taughtPoints.Count >= 2
        ? _taughtPoints.Select(p => (double[])p.Clone()).ToList()
        : null;

    private readonly SerialAsciiClient _client;
    private readonly RobotController _hardware;
    private readonly SimulationController _simulation;
    private IRobotBackend _controller;
    private bool _useSimulation;
    private string _plannedStatusText = "未规划";
    private string _twistInput = "0,0,0,0,0,0";
    private string _servoFrameInput = "base";
    private string _servoStatusText = "未启动";
    private readonly Action<string> _backendLogHandler;
    private readonly Action<Models.RobotState> _backendStateHandler;
    private readonly TrajectoryPlanner _planner = new();
    private readonly Dof6Kinematics _ik = Dof6Kinematics.CreateDefault();
    private readonly RrtPlanner _rrt;
    private readonly CartesianServo _servoMath = new(Dof6Kinematics.CreateDefault());
    private ServoController? _servo;

    private static readonly double[] KinematicMin = { -170, -73, 35, -180, -120, -720 };
    private static readonly double[] KinematicMax = { 170, 90, 180, 180, 120, 720 };

    private readonly RelayCommand _connectCommand;
    private readonly RelayCommand _disconnectCommand;
    private readonly RelayCommand _refreshPortsCommand;
    private readonly RelayCommand _startCommand;
    private readonly RelayCommand _stopCommand;
    private readonly RelayCommand _disableCommand;
    private readonly RelayCommand _homeCommand;
    private readonly RelayCommand _resetCommand;
    private readonly RelayCommand _calibrationCommand;
    private readonly RelayCommand _sendJointCommand;
    private readonly RelayCommand _sendPoseCommand;
    private readonly RelayCommand _jogMinusCommand;
    private readonly RelayCommand _jogPlusCommand;
    private readonly RelayCommand _setHomeCurrentCommand;
    private readonly RelayCommand _setCommandModeCommand;
    private readonly RelayCommand _getHomeOffsetCommand;
    private readonly RelayCommand _getBasePosCommand;
    private readonly RelayCommand _recordPointCommand;
    private readonly RelayCommand _deletePointCommand;
    private readonly RelayCommand _clearPointsCommand;
    private readonly RelayCommand _playPathCommand;
    private readonly RelayCommand _stopPlayCommand;
    private readonly RelayCommand _stopJogCommand;
    private readonly RelayCommand _applySettingsCommand;
    private readonly RelayCommand _saveConfigCommand;
    private readonly RelayCommand _exportLogsCommand;
    private readonly RelayCommand _savePoseCommand;
    private readonly RelayCommand _recallPoseCommand;
    private readonly RelayCommand _deletePoseCommand;
    private readonly RelayCommand _setDceKpCommand;
    private readonly RelayCommand _setDceKiCommand;
    private readonly RelayCommand _setDceKdCommand;
    private readonly RelayCommand _rebootMotorCommand;
    private readonly RelayCommand _toggleSimulationCommand;
    private readonly RelayCommand _planPathCommand;
    private readonly RelayCommand _playPlannedCommand;
    private readonly RelayCommand _startServoCommand;
    private readonly RelayCommand _stopServoCommand;

    private readonly List<double[]> _taughtPoints = new();
    private readonly List<double[]> _plannedPath = new();
    private readonly Dictionary<string, double[]> _namedPoses = new();
    private CancellationTokenSource? _playCts;
    private CancellationTokenSource? _jogCts;
    private CancellationTokenSource? _monitorCts;
    private double[]? _jogJoints;
    private double[]? _jogPose;

    private string? _selectedPort;
    private bool _isConnected;
    private string _connectionStatus = "未连接";
    private string _jointTargetInput = "0,-73,180,0,0,0";
    private string _poseTargetInput = "0,0,0,0,0,0";
    private string _speedInput = "20";
    private string _jogStepInput = "1";
    private string _jointText = "--";
    private string _poseText = "--";
    private string _queueText = "队列余量: -";
    private string _commandModeInput = "2";
    private string _homeOffsetText = "--";
    private string _basePosText = "--";

    private bool _isPlaying;
    private bool _isJogging;
    private int _selectedPointIndex = -1;
    private string _pathStatusText = "未播放";
    private string _interpolationStepsInput = "10";
    private string _playIntervalInput = "50";
    private string _cartesianJogStepInput = "10";

    private bool _useTimeParameterization;
    private bool _enableCollisionCheck = true;
    private string _collisionRadiusInput = "45,20,25,20,25,30,20";
    private string _collisionMarginInput = "0,0,0,0,0,0,0";
    private string _collisionGroundInput = "-20,-20,-20,-20,-20,-20,-20";
    private string _hardwarePollHzInput = "10";
    private string _simulationPollHzInput = "50";
    private string _maxVelocityInput = "30";
    private string _maxAccelInput = "60";
    private string _collisionText = "未检查";
    private string _velocityText = "--";
    private bool _autoReconnect = true;
    private bool _manualDisconnect;
    private string? _lastConnectedPort;

    private string _namedPoseNameInput = "home";
    private string _motorNodeInput = "1";
    private string _dceValueInput = "100";

    private string _baudRateInput = "115200";
    private string _commandPrefixInput = ">";
    private string _queryTimeoutInput = "300";
    private string _motionAckTimeoutInput = "300";
    private string _slowTimeoutInput = "15000";
    private string _offsetInput = "0,0,1.57079,0,0,0";
    private string _signInput = "1,1,1,1,-1,-1";

    public MainViewModel()
    {
        _client = new SerialAsciiClient();
        _backendLogHandler = message => AppendLog(message);
        _backendStateHandler = state => OnStateUpdated(state);

        _hardware = new RobotController(_client);
        _simulation = new SimulationController();
        _controller = _hardware;
        _rrt = new RrtPlanner(KinematicMin, KinematicMax,
            joints => _planner.CheckCollisionFwDeg(joints) is null);
        CreateServo();
        SubscribeBackend(_controller);

        _connectCommand = new RelayCommand(_ => Connect(), _ => CanConnect());
        _disconnectCommand = new RelayCommand(_ => Disconnect(), _ => CanOperate());
        _refreshPortsCommand = new RelayCommand(_ => RefreshPorts(), _ => !CanOperate());
        _startCommand = new RelayCommand(_ => _controller.Start(), _ => CanOperate());
        _stopCommand = new RelayCommand(_ => _controller.Stop(), _ => CanOperate());
        _disableCommand = new RelayCommand(_ => _controller.Disable(), _ => CanOperate());
        _homeCommand = new RelayCommand(_ => _controller.Home(), _ => CanOperate());
        _resetCommand = new RelayCommand(_ => _controller.Reset(), _ => CanOperate());
        _calibrationCommand = new RelayCommand(_ => _controller.Calibration(), _ => CanOperate());
        _sendJointCommand = new RelayCommand(_ => SendJoint(), _ => CanManualMove());
        _sendPoseCommand = new RelayCommand(_ => SendPose(), _ => CanManualMove());
        _jogMinusCommand = new RelayCommand(p => Jog(ToIndex(p), -1), _ => CanManualMove());
        _jogPlusCommand = new RelayCommand(p => Jog(ToIndex(p), 1), _ => CanManualMove());
        _setHomeCurrentCommand = new RelayCommand(_ => _controller.SetHomeCurrent(), _ => CanOperate());
        _setCommandModeCommand = new RelayCommand(_ => SetCommandMode(), _ => CanOperate());
        _getHomeOffsetCommand = new RelayCommand(_ =>
        {
            double[] values = _controller.GetHomeOffset();
            if (values.Length == 6)
                HomeOffsetText = FormatSix(values);
        }, _ => CanOperate());
        _getBasePosCommand = new RelayCommand(_ =>
        {
            double[] values = _controller.GetBasePosition();
            if (values.Length == 3)
                BasePosText = string.Join(", ", values.Select(v => v.ToString("F2", CultureInfo.InvariantCulture)));
        }, _ => CanOperate());

        _recordPointCommand = new RelayCommand(_ => RecordCurrentPoint(), _ => CanManualMove());
        _deletePointCommand = new RelayCommand(_ => DeleteSelectedPoint(), _ => CanManualMove());
        _clearPointsCommand = new RelayCommand(_ => ClearPoints(), _ => CanManualMove());
        _playPathCommand = new RelayCommand(_ => PlayPath(), _ => CanManualMove() && _taughtPoints.Count >= 2);
        _stopPlayCommand = new RelayCommand(_ => StopPlay(), _ => _isPlaying);
        _stopJogCommand = new RelayCommand(_ => StopJog(), _ => _isJogging);
        _applySettingsCommand = new RelayCommand(_ => ApplySettings());
        _saveConfigCommand = new RelayCommand(_ => SaveConfig());
        _exportLogsCommand = new RelayCommand(_ => ExportLogs());
        _savePoseCommand = new RelayCommand(_ => SaveNamedPose(), _ => CanManualMove());
        _recallPoseCommand = new RelayCommand(_ => RecallNamedPose(), _ => CanManualMove());
        _deletePoseCommand = new RelayCommand(_ => DeleteNamedPose());
        _setDceKpCommand = new RelayCommand(_ => SetDce("KP"), _ => CanOperate());
        _setDceKiCommand = new RelayCommand(_ => SetDce("KI"), _ => CanOperate());
        _setDceKdCommand = new RelayCommand(_ => SetDce("KD"), _ => CanOperate());
        _rebootMotorCommand = new RelayCommand(_ => RebootMotor(), _ => CanOperate());
        _toggleSimulationCommand = new RelayCommand(_ => ToggleSimulation());
        _planPathCommand = new RelayCommand(_ => PlanPath(), _ => CanManualMove());
        _playPlannedCommand = new RelayCommand(_ => PlayPlannedPath(), _ => CanManualMove() && _plannedPath.Count >= 2);
        _startServoCommand = new RelayCommand(_ => StartServo(), _ => CanManualMove());
        _stopServoCommand = new RelayCommand(_ => StopServo(), _ => IsServoRunning);

        for (int i = 0; i < 6; i++)
            JogRows.Add(new JogRow { Index = i, Label = $"J{i + 1}" });

        string[] axes = { "X", "Y", "Z" };
        for (int i = 0; i < axes.Length; i++)
            CartesianAxes.Add(new JogRow { Index = i, Label = axes[i] });

        LoadConfig();
        StartConnectionMonitor();
        RefreshPorts();
        UpdateConnectionState();
    }

    public event PropertyChangedEventHandler? PropertyChanged;

    public IRobotBackend Controller => _controller;

    public ObservableCollection<string> Ports { get; } = new();
    public ObservableCollection<string> Logs { get; } = new();
    public ObservableCollection<JogRow> JogRows { get; } = new();
    public ObservableCollection<JogRow> CartesianAxes { get; } = new();
    public ObservableCollection<string> TaughtPoints { get; } = new();
    public ObservableCollection<string> NamedPoses { get; } = new();

    public ICommand ConnectCommand => _connectCommand;
    public ICommand DisconnectCommand => _disconnectCommand;
    public ICommand RefreshPortsCommand => _refreshPortsCommand;
    public ICommand StartCommand => _startCommand;
    public ICommand StopCommand => _stopCommand;
    public ICommand DisableCommand => _disableCommand;
    public ICommand HomeCommand => _homeCommand;
    public ICommand ResetCommand => _resetCommand;
    public ICommand CalibrationCommand => _calibrationCommand;
    public ICommand SendJointCommand => _sendJointCommand;
    public ICommand SendPoseCommand => _sendPoseCommand;
    public ICommand JogMinusCommand => _jogMinusCommand;
    public ICommand JogPlusCommand => _jogPlusCommand;
    public ICommand SetHomeCurrentCommand => _setHomeCurrentCommand;
    public ICommand SetCommandModeCommand => _setCommandModeCommand;
    public ICommand GetHomeOffsetCommand => _getHomeOffsetCommand;
    public ICommand GetBasePosCommand => _getBasePosCommand;
    public ICommand RecordPointCommand => _recordPointCommand;
    public ICommand DeletePointCommand => _deletePointCommand;
    public ICommand ClearPointsCommand => _clearPointsCommand;
    public ICommand PlayPathCommand => _playPathCommand;
    public ICommand StopPlayCommand => _stopPlayCommand;
    public ICommand StopJogCommand => _stopJogCommand;
    public ICommand ApplySettingsCommand => _applySettingsCommand;
    public ICommand SaveConfigCommand => _saveConfigCommand;
    public ICommand ExportLogsCommand => _exportLogsCommand;
    public ICommand SavePoseCommand => _savePoseCommand;
    public ICommand RecallPoseCommand => _recallPoseCommand;
    public ICommand DeletePoseCommand => _deletePoseCommand;
    public ICommand SetDceKpCommand => _setDceKpCommand;
    public ICommand SetDceKiCommand => _setDceKiCommand;
    public ICommand SetDceKdCommand => _setDceKdCommand;
    public ICommand RebootMotorCommand => _rebootMotorCommand;
    public ICommand ToggleSimulationCommand => _toggleSimulationCommand;
    public ICommand PlanPathCommand => _planPathCommand;
    public ICommand PlayPlannedCommand => _playPlannedCommand;
    public ICommand StartServoCommand => _startServoCommand;
    public ICommand StopServoCommand => _stopServoCommand;

    public string TwistInput
    {
        get => _twistInput;
        set
        {
            if (_twistInput == value) return;
            _twistInput = value;
            OnPropertyChanged();
        }
    }

    public string ServoFrameInput
    {
        get => _servoFrameInput;
        set
        {
            if (_servoFrameInput == value) return;
            _servoFrameInput = value;
            OnPropertyChanged();
        }
    }

    public string ServoStatusText
    {
        get => _servoStatusText;
        private set
        {
            if (_servoStatusText == value) return;
            _servoStatusText = value;
            OnPropertyChanged();
        }
    }

    public bool IsServoRunning => _servo?.IsRunning == true;

    public string PlannedStatusText
    {
        get => _plannedStatusText;
        private set
        {
            if (_plannedStatusText == value)
                return;
            _plannedStatusText = value;
            OnPropertyChanged();
        }
    }

    public bool UseSimulation
    {
        get => _useSimulation;
        private set
        {
            if (_useSimulation == value)
                return;
            _useSimulation = value;
            OnPropertyChanged();
        }
    }

    public string? SelectedPort
    {
        get => _selectedPort;
        set
        {
            if (_selectedPort == value)
                return;
            _selectedPort = value;
            OnPropertyChanged();
            _connectCommand.RaiseCanExecuteChanged();
        }
    }

    public bool IsConnected
    {
        get => _isConnected;
        private set
        {
            if (_isConnected == value)
                return;
            _isConnected = value;
            OnPropertyChanged();
        }
    }

    public string ConnectionStatus
    {
        get => _connectionStatus;
        private set
        {
            if (_connectionStatus == value)
                return;
            _connectionStatus = value;
            OnPropertyChanged();
        }
    }

    public string JointTargetInput
    {
        get => _jointTargetInput;
        set
        {
            if (_jointTargetInput == value)
                return;
            _jointTargetInput = value;
            OnPropertyChanged();
        }
    }

    public string PoseTargetInput
    {
        get => _poseTargetInput;
        set
        {
            if (_poseTargetInput == value)
                return;
            _poseTargetInput = value;
            OnPropertyChanged();
        }
    }

    public string SpeedInput
    {
        get => _speedInput;
        set
        {
            if (_speedInput == value)
                return;
            _speedInput = value;
            OnPropertyChanged();
        }
    }

    public string JogStepInput
    {
        get => _jogStepInput;
        set
        {
            if (_jogStepInput == value)
                return;
            _jogStepInput = value;
            OnPropertyChanged();
        }
    }

    public string JointText
    {
        get => _jointText;
        private set
        {
            if (_jointText == value)
                return;
            _jointText = value;
            OnPropertyChanged();
        }
    }

    public string PoseText
    {
        get => _poseText;
        private set
        {
            if (_poseText == value)
                return;
            _poseText = value;
            OnPropertyChanged();
        }
    }

    public string QueueText
    {
        get => _queueText;
        private set
        {
            if (_queueText == value)
                return;
            _queueText = value;
            OnPropertyChanged();
        }
    }

    public string CommandModeInput
    {
        get => _commandModeInput;
        set
        {
            if (_commandModeInput == value)
                return;
            _commandModeInput = value;
            OnPropertyChanged();
        }
    }

    public string HomeOffsetText
    {
        get => _homeOffsetText;
        private set
        {
            if (_homeOffsetText == value)
                return;
            _homeOffsetText = value;
            OnPropertyChanged();
        }
    }

    public string BasePosText
    {
        get => _basePosText;
        private set
        {
            if (_basePosText == value)
                return;
            _basePosText = value;
            OnPropertyChanged();
        }
    }

    public bool IsPlaying
    {
        get => _isPlaying;
        private set
        {
            if (_isPlaying == value)
                return;
            _isPlaying = value;
            OnPropertyChanged();
            RefreshCommands();
        }
    }

    public bool IsJogging
    {
        get => _isJogging;
        private set
        {
            if (_isJogging == value)
                return;
            _isJogging = value;
            OnPropertyChanged();
            RefreshCommands();
        }
    }

    public int SelectedPointIndex
    {
        get => _selectedPointIndex;
        set
        {
            if (_selectedPointIndex == value)
                return;
            _selectedPointIndex = value;
            OnPropertyChanged();
        }
    }

    public string PathStatusText
    {
        get => _pathStatusText;
        private set
        {
            if (_pathStatusText == value)
                return;
            _pathStatusText = value;
            OnPropertyChanged();
        }
    }

    public string InterpolationStepsInput
    {
        get => _interpolationStepsInput;
        set
        {
            if (_interpolationStepsInput == value)
                return;
            _interpolationStepsInput = value;
            OnPropertyChanged();
        }
    }

    public string PlayIntervalInput
    {
        get => _playIntervalInput;
        set
        {
            if (_playIntervalInput == value)
                return;
            _playIntervalInput = value;
            OnPropertyChanged();
        }
    }

    public string CartesianJogStepInput
    {
        get => _cartesianJogStepInput;
        set
        {
            if (_cartesianJogStepInput == value)
                return;
            _cartesianJogStepInput = value;
            OnPropertyChanged();
        }
    }

    public bool UseTimeParameterization
    {
        get => _useTimeParameterization;
        set
        {
            if (_useTimeParameterization == value)
                return;
            _useTimeParameterization = value;
            OnPropertyChanged();
        }
    }

    public bool EnableCollisionCheck
    {
        get => _enableCollisionCheck;
        set
        {
            if (_enableCollisionCheck == value)
                return;
            _enableCollisionCheck = value;
            OnPropertyChanged();
        }
    }

    public string CollisionRadiusInput
    {
        get => _collisionRadiusInput;
        set
        {
            if (_collisionRadiusInput == value)
                return;
            _collisionRadiusInput = value;
            OnPropertyChanged();
        }
    }

    public string CollisionMarginInput
    {
        get => _collisionMarginInput;
        set
        {
            if (_collisionMarginInput == value)
                return;
            _collisionMarginInput = value;
            OnPropertyChanged();
        }
    }

    public string CollisionGroundInput
    {
        get => _collisionGroundInput;
        set
        {
            if (_collisionGroundInput == value)
                return;
            _collisionGroundInput = value;
            OnPropertyChanged();
        }
    }

    public string HardwarePollHzInput
    {
        get => _hardwarePollHzInput;
        set
        {
            if (_hardwarePollHzInput == value)
                return;
            _hardwarePollHzInput = value;
            OnPropertyChanged();
        }
    }

    public string SimulationPollHzInput
    {
        get => _simulationPollHzInput;
        set
        {
            if (_simulationPollHzInput == value)
                return;
            _simulationPollHzInput = value;
            OnPropertyChanged();
        }
    }

    public string MaxVelocityInput
    {
        get => _maxVelocityInput;
        set
        {
            if (_maxVelocityInput == value)
                return;
            _maxVelocityInput = value;
            OnPropertyChanged();
        }
    }

    public string MaxAccelInput
    {
        get => _maxAccelInput;
        set
        {
            if (_maxAccelInput == value)
                return;
            _maxAccelInput = value;
            OnPropertyChanged();
        }
    }

    public string CollisionText
    {
        get => _collisionText;
        private set
        {
            if (_collisionText == value)
                return;
            _collisionText = value;
            OnPropertyChanged();
        }
    }

    public string VelocityText
    {
        get => _velocityText;
        private set
        {
            if (_velocityText == value)
                return;
            _velocityText = value;
            OnPropertyChanged();
        }
    }

    public bool AutoReconnect
    {
        get => _autoReconnect;
        set
        {
            if (_autoReconnect == value)
                return;
            _autoReconnect = value;
            OnPropertyChanged();
        }
    }

    public string NamedPoseNameInput
    {
        get => _namedPoseNameInput;
        set
        {
            if (_namedPoseNameInput == value)
                return;
            _namedPoseNameInput = value;
            OnPropertyChanged();
        }
    }

    public string MotorNodeInput
    {
        get => _motorNodeInput;
        set
        {
            if (_motorNodeInput == value)
                return;
            _motorNodeInput = value;
            OnPropertyChanged();
        }
    }

    public string DceValueInput
    {
        get => _dceValueInput;
        set
        {
            if (_dceValueInput == value)
                return;
            _dceValueInput = value;
            OnPropertyChanged();
        }
    }

    public string BaudRateInput
    {
        get => _baudRateInput;
        set
        {
            if (_baudRateInput == value)
                return;
            _baudRateInput = value;
            OnPropertyChanged();
        }
    }

    public string CommandPrefixInput
    {
        get => _commandPrefixInput;
        set
        {
            if (_commandPrefixInput == value)
                return;
            _commandPrefixInput = value;
            OnPropertyChanged();
        }
    }

    public string QueryTimeoutInput
    {
        get => _queryTimeoutInput;
        set
        {
            if (_queryTimeoutInput == value)
                return;
            _queryTimeoutInput = value;
            OnPropertyChanged();
        }
    }

    public string MotionAckTimeoutInput
    {
        get => _motionAckTimeoutInput;
        set
        {
            if (_motionAckTimeoutInput == value)
                return;
            _motionAckTimeoutInput = value;
            OnPropertyChanged();
        }
    }

    public string SlowTimeoutInput
    {
        get => _slowTimeoutInput;
        set
        {
            if (_slowTimeoutInput == value)
                return;
            _slowTimeoutInput = value;
            OnPropertyChanged();
        }
    }

    public string OffsetInput
    {
        get => _offsetInput;
        set
        {
            if (_offsetInput == value)
                return;
            _offsetInput = value;
            OnPropertyChanged();
        }
    }

    public string SignInput
    {
        get => _signInput;
        set
        {
            if (_signInput == value)
                return;
            _signInput = value;
            OnPropertyChanged();
        }
    }

    private bool CanConnect() => !CanOperate() && !string.IsNullOrWhiteSpace(SelectedPort);

    private bool CanOperate() => _controller.IsConnected;

    private bool CanManualMove() => _controller.IsConnected && !_isPlaying && !_isJogging;

    private void RefreshPorts()
    {
        Ports.Clear();
        foreach (string port in SerialAsciiClient.GetAvailablePorts())
            Ports.Add(port);

        if (Ports.Count > 0 && string.IsNullOrWhiteSpace(SelectedPort))
            SelectedPort = Ports[0];
    }

    private void Connect()
    {
        try
        {
            ApplySettings(false);
            _controller.Connect(UseSimulation ? "sim" : SelectedPort!);
            int pollHz = CurrentPollHz;
            _controller.StartPolling(1000 / pollHz);
            _manualDisconnect = false;
            _lastConnectedPort = SelectedPort;
            SaveConfig();
            AppendLog(UseSimulation
                ? $"仿真已启动，状态刷新 {pollHz}Hz"
                : $"已连接串口，状态轮询 {pollHz}Hz");
        }
        catch (Exception ex)
        {
            AppendLog($"连接失败: {ex.Message}");
        }
        finally
        {
            UpdateConnectionState();
        }
    }

    private void Disconnect()
    {
        _manualDisconnect = true;
        StopPlay();
        StopJog();
        StopServo();
        _controller.Disconnect();
        UpdateConnectionState();
    }

    private void SendJoint()
    {
        if (!TryParseSix(JointTargetInput, out double[] joints))
        {
            AppendLog("关节目标格式错误，应为 6 个逗号分隔的数字");
            return;
        }
        SendJointArray(joints);
    }

    private void SendJointArray(double[] joints)
    {
        if (EnableCollisionCheck)
        {
            CollisionInfo? hit = _planner.CheckCollisionFwDeg(joints);
            DispatchIfNeeded(() => CollisionText = hit is null ? "安全" : hit.Description);
            if (hit is not null)
            {
                AppendLog($"已阻止发送：{hit.Description}");
                return;
            }
        }

        double? speed = TryParseDouble(SpeedInput, out double parsedSpeed) ? parsedSpeed : null;
        try
        {
            int? queue = _controller.MoveJ(joints, speed);
            DispatchIfNeeded(() => QueueText = queue.HasValue ? $"队列余量: {queue.Value}" : "队列余量: - (未发送)");
            if (queue.HasValue)
                AppendLog($"已发送关节目标，队列余量 {queue.Value}");
        }
        catch (Exception ex)
        {
            AppendLog($"发送关节目标失败: {ex.Message}");
        }
    }

    private void SendPose()
    {
        if (!TryParseSix(PoseTargetInput, out double[] pose))
        {
            AppendLog("笛卡尔目标格式错误，应为 6 个逗号分隔的数字");
            return;
        }
        SendPoseArray(pose);
    }

    private void SendPoseArray(double[] pose)
    {
        var target = new Pose6D(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
        double[] currentFw = _controller.State.JointsDeg.Length == 6
            ? _controller.State.JointsDeg : new double[6];

        Joint6D? solution = _ik.SolveIkBest(target, currentFw, KinematicMin, KinematicMax);
        if (solution is null)
        {
            AppendLog("IK 失败: 8 组解均超出关节限位");
            return;
        }

        double[] jointsFw = solution.A;
        Pose6D checkPose = _ik.SolveFkMm(jointsFw);
        AppendLog($"IK 解: {string.Join(",", jointsFw.Select(v => v.ToString("F2", CultureInfo.InvariantCulture)))} " +
                  $"(回环 {Math.Sqrt(Math.Pow(checkPose.X - pose[0], 2) + Math.Pow(checkPose.Y - pose[1], 2) + Math.Pow(checkPose.Z - pose[2], 2)):F2}mm)");

        if (EnableCollisionCheck)
        {
            CollisionInfo? hit = _planner.CheckCollisionFwDeg(jointsFw);
            DispatchIfNeeded(() => CollisionText = hit is null ? "安全" : hit.Description);
            if (hit is not null)
            {
                AppendLog($"已阻止笛卡尔目标：{hit.Description}");
                return;
            }
        }

        double? speed = TryParseDouble(SpeedInput, out double parsedSpeed) ? parsedSpeed : null;
        try
        {
            int? queue = _controller.MoveL(pose, speed);
            DispatchIfNeeded(() => QueueText = queue.HasValue ? $"队列余量: {queue.Value}" : "队列余量: - (未发送)");
            if (queue.HasValue)
                AppendLog($"已发送笛卡尔目标，队列余量 {queue.Value}");
        }
        catch (Exception ex)
        {
            AppendLog($"发送笛卡尔目标失败: {ex.Message}");
        }
    }

    private void CreateServo()
    {
        StopServo();
        _servo = new ServoController(_controller, _servoMath, _planner, KinematicMin, KinematicMax);
        _servo.Log += message => DispatchIfNeeded(() =>
        {
            AppendLog(message);
            if (message.Contains("启动")) ServoStatusText = "运行中";
            else if (message.Contains("停止")) ServoStatusText = "已停止";
            _startServoCommand.RaiseCanExecuteChanged();
            _stopServoCommand.RaiseCanExecuteChanged();
            OnPropertyChanged(nameof(IsServoRunning));
        });
    }

    private void StartServo()
    {
        if (!TryParseSix(TwistInput, out double[] twist))
        {
            AppendLog("twist 格式错误，应为 6 个数字: vx,vy,vz,wx,wy,wz");
            return;
        }
        ServoFrame frame = ServoFrameInput.Trim().Equals("tool", StringComparison.OrdinalIgnoreCase)
            ? ServoFrame.Tool : ServoFrame.Base;
        _servo?.Start(twist, frame, 33);
    }

    private void StopServo()
    {
        if (_servo?.IsRunning == true)
        {
            _servo.Stop();
            ServoStatusText = "已停止";
        }
    }

    private void SubscribeBackend(IRobotBackend backend)
    {
        backend.Log += _backendLogHandler;
        backend.StateUpdated += _backendStateHandler;
    }

    private void UnsubscribeBackend(IRobotBackend backend)
    {
        backend.Log -= _backendLogHandler;
        backend.StateUpdated -= _backendStateHandler;
    }

    private void ToggleSimulation()
    {
        StopPlay();
        StopJog();
        StopServo();
        _controller.Disconnect();

        UnsubscribeBackend(_controller);
        UseSimulation = !UseSimulation;
        _controller = UseSimulation ? _simulation : _hardware;
        CreateServo();
        SubscribeBackend(_controller);

        if (UseSimulation)
        {
            // 仿真无需串口：切换后立即连接并启动轮询，状态区马上显示仿真姿态
            try
            {
                _controller.Connect("sim");
                int pollHz = SimulationPollHz;
                _controller.StartPolling(1000 / pollHz);
                _manualDisconnect = false;
                AppendLog($"已切换到内置仿真并启动（状态刷新 {pollHz}Hz）");
            }
            catch (Exception ex)
            {
                AppendLog($"仿真启动失败: {ex.Message}");
            }
        }
        else
        {
            // 真机需要用户选端口后手动连接
            AppendLog("已切换到串口真机，请选择端口并点击连接");
        }

        UpdateConnectionState();
    }

    private void SetCommandMode()
    {
        if (!int.TryParse(CommandModeInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int mode))
        {
            AppendLog("命令模式必须是整数 (1=顺序, 2=可打断, 3=连续轨迹, 4=调参)");
            return;
        }
        _controller.SetCommandMode(mode);
    }

    private void Jog(int index, double direction)
    {
        if (index < 0 || index > 5)
            return;

        double[] baseJoints = _controller.State.JointsDeg.Length == 6
            ? (double[])_controller.State.JointsDeg.Clone()
            : new double[6];

        double step = TryParseDouble(JogStepInput, out double parsedStep) ? Math.Abs(parsedStep) : 1.0;
        baseJoints[index] += direction * step;
        JointTargetInput = FormatSix(baseJoints);
        SendJoint();
    }

    private void RecordCurrentPoint()
    {
        if (_controller.State.JointsDeg.Length != 6)
        {
            AppendLog("尚无有效关节状态，无法记录示教点");
            return;
        }

        var point = (double[])_controller.State.JointsDeg.Clone();
        _taughtPoints.Add(point);
        TaughtPoints.Add($"P{_taughtPoints.Count}: {FormatSix(point)}");
        SelectedPointIndex = TaughtPoints.Count - 1;
        AppendLog($"已记录示教点 P{_taughtPoints.Count}");
        SaveConfig();
        RefreshCommands();
        PathsChanged?.Invoke();
    }

    private void DeleteSelectedPoint()
    {
        int index = SelectedPointIndex;
        if (index < 0 || index >= _taughtPoints.Count)
        {
            AppendLog("请先选择一个示教点");
            return;
        }

        _taughtPoints.RemoveAt(index);
        RefreshTaughtPoints();
        SelectedPointIndex = Math.Min(index, TaughtPoints.Count - 1);
        AppendLog("已删除示教点");
        SaveConfig();
        RefreshCommands();
        PathsChanged?.Invoke();
    }

    private void ClearPoints()
    {
        _taughtPoints.Clear();
        TaughtPoints.Clear();
        SelectedPointIndex = -1;
        PathStatusText = "未播放";
        AppendLog("已清空示教点");
        SaveConfig();
        RefreshCommands();
        PathsChanged?.Invoke();
    }

    private void RefreshTaughtPoints()
    {
        TaughtPoints.Clear();
        for (int i = 0; i < _taughtPoints.Count; i++)
            TaughtPoints.Add($"P{i + 1}: {FormatSix(_taughtPoints[i])}");
    }

    private void PlanPath()
    {
        if (_controller.State.JointsDeg.Length != 6)
        {
            AppendLog("尚无有效关节状态，无法规划");
            return;
        }

        double[] goal;
        if (SelectedPointIndex >= 0 && SelectedPointIndex < _taughtPoints.Count)
            goal = (double[])_taughtPoints[SelectedPointIndex].Clone();
        else if (TryParseSix(JointTargetInput, out double[] parsedGoal))
            goal = parsedGoal;
        else
        {
            AppendLog("请选择示教点或输入合法的关节目标");
            return;
        }

        double[] start = (double[])_controller.State.JointsDeg.Clone();
        PlannedStatusText = "规划中...";
        AppendLog($"RRT 规划开始: {FormatSix(start)} → {FormatSix(goal)}");

        _ = Task.Run(() =>
        {
            RrtResult result = _rrt.Plan(start, goal, maxIterations: 4000, stepDeg: 12);
            DispatchIfNeeded(() =>
            {
                _plannedPath.Clear();
                if (!result.Success)
                {
                    PlannedStatusText = "规划失败";
                    AppendLog($"RRT 规划失败: {result.Message}");
                    PathsChanged?.Invoke();
                    return;
                }

                _plannedPath.AddRange(result.Path);
                PlannedStatusText = $"已规划 {result.Path.Count} 点 / {result.Iterations} 次迭代";
                AppendLog($"RRT 规划成功: {result.Path.Count} 个路径点，{result.Iterations} 次迭代");
                _playPlannedCommand.RaiseCanExecuteChanged();
                PathsChanged?.Invoke();
            });
        });
    }

    private void PlayPlannedPath()
    {
        if (_plannedPath.Count < 2)
        {
            AppendLog("请先规划路径");
            return;
        }
        PlayWaypoints(_plannedPath, "规划路径");
    }

    private void PlayPath()
    {
        if (_taughtPoints.Count < 2)
        {
            AppendLog("路径播放至少需要 2 个示教点");
            return;
        }
        PlayWaypoints(_taughtPoints, "示教路径");
    }

    private void PlayWaypoints(List<double[]> waypoints, string pathName)
    {
        if (_isPlaying)
            return;
        if (waypoints.Count < 2)
        {
            AppendLog("路径至少需要 2 个点");
            return;
        }

        StopJog();
        int steps = int.TryParse(InterpolationStepsInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedSteps)
            ? Math.Clamp(parsedSteps, 2, 200) : 10;
        int interval = int.TryParse(PlayIntervalInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedInterval)
            ? Math.Clamp(parsedInterval, 5, 2000) : 50;

        _playCts?.Cancel();
        CancellationTokenSource cts = new();
        _playCts = cts;
        CancellationToken token = cts.Token;
        IsPlaying = true;
        PathStatusText = "播放中";
        AppendLog(_useTimeParameterization
            ? $"开始播放{pathName}：{waypoints.Count} 点，时间参数化，速度上限 {ParseMaxVelocity():F0}deg/s，加速度上限 {ParseMaxAccel():F0}deg/s²，周期 {interval}ms"
            : $"开始播放{pathName}：{waypoints.Count} 点，插值 {steps} 步，间隔 {interval}ms");

        _ = Task.Run(async () =>
        {
            try
            {
                for (int i = 0; i < waypoints.Count - 1; i++)
                {
                    List<TrajectoryPoint> segment = _useTimeParameterization
                        ? _planner.PlanTimeParameterized(
                            waypoints[i], waypoints[i + 1],
                            ParseMaxVelocity(), ParseMaxAccel(), interval)
                        : RobotController.InterpolateJointPath(waypoints[i], waypoints[i + 1], steps)
                            .Select(p => new TrajectoryPoint(p, interval)).ToList();

                    foreach (TrajectoryPoint point in segment)
                    {
                        token.ThrowIfCancellationRequested();
                        if (EnableCollisionCheck)
                        {
                            CollisionInfo? hit = _planner.CheckCollisionFwDeg(point.JointsFwDeg);
                            DispatchIfNeeded(() => CollisionText = hit is null ? "安全" : hit.Description);
                            if (hit is not null)
                            {
                                AppendLog($"路径播放因碰撞风险停止：{hit.Description}");
                                return;
                            }
                        }

                        int? queue = _controller.MoveJ(point.JointsFwDeg, ParseSpeed());
                        if (queue.HasValue && queue.Value <= 0)
                            await Task.Delay(20, token).ConfigureAwait(false);
                        if (point.DelayMs > 0)
                            await Task.Delay((int)Math.Round(point.DelayMs), token).ConfigureAwait(false);
                    }
                }
                DispatchIfNeeded(() => PathStatusText = "播放完成");
                AppendLog("路径播放完成");
            }
            catch (OperationCanceledException)
            {
                DispatchIfNeeded(() => PathStatusText = "已取消");
                AppendLog("路径播放已取消");
            }
            catch (Exception ex)
            {
                DispatchIfNeeded(() => PathStatusText = "播放异常");
                AppendLog($"路径播放异常: {ex.Message}");
            }
            finally
            {
                DispatchIfNeeded(() =>
                {
                    if (ReferenceEquals(_playCts, cts))
                    {
                        IsPlaying = false;
                        _playCts = null;
                    }
                });
            }
        }, token);
    }

    public void StopPlay()
    {
        _playCts?.Cancel();
    }

    public void StartJointJog(int index, double direction)
    {
        if (!CanManualMove())
            return;

        StopJog();
        CancellationTokenSource cts = new();
        _jogCts = cts;
        CancellationToken token = cts.Token;
        _jogJoints = _controller.State.JointsDeg.Length == 6
            ? (double[])_controller.State.JointsDeg.Clone()
            : new double[6];
        IsJogging = true;
        AppendLog($"开始关节连续点动 J{index + 1} {(direction > 0 ? "+" : "-")}");

        _ = Task.Run(async () =>
        {
            DoJointJogStep(index, direction);
            try
            {
                while (!token.IsCancellationRequested)
                {
                    await Task.Delay(100, token).ConfigureAwait(false);
                    DoJointJogStep(index, direction);
                }
            }
            catch (OperationCanceledException)
            {
                // 正常停止
            }
            finally
            {
                DispatchIfNeeded(() =>
                {
                    if (ReferenceEquals(_jogCts, cts))
                    {
                        IsJogging = false;
                        _jogCts = null;
                    }
                });
            }
        }, token);
    }

    private void DoJointJogStep(int index, double direction)
    {
        if (index < 0 || index > 5)
            return;

        _jogJoints ??= _controller.State.JointsDeg.Length == 6
            ? (double[])_controller.State.JointsDeg.Clone()
            : new double[6];
        double step = TryParseDouble(JogStepInput, out double parsedStep) ? Math.Abs(parsedStep) : 1.0;
        _jogJoints[index] += direction * step;
        string text = FormatSix(_jogJoints);
        double[] target = (double[])_jogJoints.Clone();

        DispatchIfNeeded(() => JointTargetInput = text);
        SendJointArray(target);
    }

    public void StartCartesianJog(int axis, double direction)
    {
        if (!CanManualMove())
            return;

        StopJog();
        CancellationTokenSource cts = new();
        _jogCts = cts;
        CancellationToken token = cts.Token;
        _jogPose = _controller.State.Pose.Length == 6
            ? (double[])_controller.State.Pose.Clone()
            : new double[6];
        IsJogging = true;
        AppendLog($"开始笛卡尔连续点动 {new[] { "X", "Y", "Z" }[Math.Clamp(axis, 0, 2)]} {(direction > 0 ? "+" : "-")}");

        _ = Task.Run(async () =>
        {
            DoCartesianJogStep(axis, direction);
            try
            {
                while (!token.IsCancellationRequested)
                {
                    await Task.Delay(100, token).ConfigureAwait(false);
                    DoCartesianJogStep(axis, direction);
                }
            }
            catch (OperationCanceledException)
            {
                // 正常停止
            }
            finally
            {
                DispatchIfNeeded(() =>
                {
                    if (ReferenceEquals(_jogCts, cts))
                    {
                        IsJogging = false;
                        _jogCts = null;
                    }
                });
            }
        }, token);
    }

    private void DoCartesianJogStep(int axis, double direction)
    {
        if (axis < 0 || axis > 2)
            return;

        _jogPose ??= _controller.State.Pose.Length == 6
            ? (double[])_controller.State.Pose.Clone()
            : new double[6];
        double step = TryParseDouble(CartesianJogStepInput, out double parsedStep) ? Math.Abs(parsedStep) : 10.0;
        _jogPose[axis] += direction * step;
        string text = FormatSix(_jogPose);
        double[] target = (double[])_jogPose.Clone();

        DispatchIfNeeded(() => PoseTargetInput = text);
        SendPoseArray(target);
    }

    public void StopJog()
    {
        _jogCts?.Cancel();
    }

    public void ApplySettings(bool log = true)
    {
        int baud = int.TryParse(BaudRateInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedBaud) && parsedBaud > 0
            ? parsedBaud : 115200;
        char prefix = string.IsNullOrWhiteSpace(CommandPrefixInput) ? '>' : CommandPrefixInput.Trim()[0];
        int queryTimeout = int.TryParse(QueryTimeoutInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedQuery) && parsedQuery > 0
            ? parsedQuery : 300;
        int motionTimeout = int.TryParse(MotionAckTimeoutInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedMotion) && parsedMotion > 0
            ? parsedMotion : 300;
        int slowTimeout = int.TryParse(SlowTimeoutInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedSlow) && parsedSlow > 0
            ? parsedSlow : 15000;

        if (!TryParseSix(OffsetInput, out double[] offset) || !TryParseSix(SignInput, out double[] signValues))
        {
            AppendLog("偏移/符号格式错误，应为 6 个逗号分隔数字");
            return;
        }

        int[] signs = signValues.Select(v => v >= 0 ? 1 : -1).ToArray();
        HardwarePollHzInput = HardwarePollHz.ToString(CultureInfo.InvariantCulture);
        SimulationPollHzInput = SimulationPollHz.ToString(CultureInfo.InvariantCulture);
        _client.Configure(baud, prefix, queryTimeout, motionTimeout, queryTimeout);
        _controller.Configure(offset, signs, slowTimeout);
        ConfigureCollisionPlanner(log);
        ApplyPollInterval(log);

        SaveConfig();
        if (log)
            AppendLog(_client.IsOpen ? "设置已应用并保存（波特率将在下次连接后生效）" : "设置已应用并保存");
    }

    private void LoadConfig()
    {
        AppConfig config = PersistenceService.Load();
        SelectedPort = config.SelectedPort;
        JointTargetInput = config.JointTargetInput;
        PoseTargetInput = config.PoseTargetInput;
        SpeedInput = config.SpeedInput;
        JogStepInput = config.JogStepInput;
        CommandModeInput = config.CommandModeInput;
        CartesianJogStepInput = config.CartesianJogStepInput;
        InterpolationStepsInput = config.InterpolationStepsInput;
        PlayIntervalInput = config.PlayIntervalInput;
        UseTimeParameterization = config.UseTimeParameterization;
        EnableCollisionCheck = config.EnableCollisionCheck;
        CollisionRadiusInput = config.CollisionRadiusInput;
        CollisionMarginInput = config.CollisionMarginInput;
        CollisionGroundInput = config.CollisionGroundInput;
        HardwarePollHzInput = config.HardwarePollHzInput;
        SimulationPollHzInput = config.SimulationPollHzInput;
        MaxVelocityInput = config.MaxVelocityInput;
        MaxAccelInput = config.MaxAccelInput;
        BaudRateInput = config.BaudRateInput;
        CommandPrefixInput = config.CommandPrefixInput;
        QueryTimeoutInput = config.QueryTimeoutInput;
        MotionAckTimeoutInput = config.MotionAckTimeoutInput;
        SlowTimeoutInput = config.SlowTimeoutInput;
        OffsetInput = config.OffsetInput;
        SignInput = config.SignInput;
        AutoReconnect = config.AutoReconnect;
        ConfigureCollisionPlanner(false);

        _taughtPoints.Clear();
        foreach (double[] point in config.TaughtPoints)
        {
            if (point.Length == 6)
                _taughtPoints.Add((double[])point.Clone());
        }
        RefreshTaughtPoints();

        _namedPoses.Clear();
        foreach (KeyValuePair<string, double[]> pair in config.NamedPoses)
        {
            if (pair.Value.Length == 6)
                _namedPoses[pair.Key] = (double[])pair.Value.Clone();
        }
        RefreshNamedPoses();
        PathsChanged?.Invoke();
    }

    private AppConfig BuildConfig()
    {
        return new AppConfig
        {
            SelectedPort = SelectedPort ?? string.Empty,
            JointTargetInput = JointTargetInput,
            PoseTargetInput = PoseTargetInput,
            SpeedInput = SpeedInput,
            JogStepInput = JogStepInput,
            CommandModeInput = CommandModeInput,
            CartesianJogStepInput = CartesianJogStepInput,
            InterpolationStepsInput = InterpolationStepsInput,
            PlayIntervalInput = PlayIntervalInput,
            UseTimeParameterization = UseTimeParameterization,
            EnableCollisionCheck = EnableCollisionCheck,
            CollisionRadiusInput = CollisionRadiusInput,
            CollisionMarginInput = CollisionMarginInput,
            CollisionGroundInput = CollisionGroundInput,
            HardwarePollHzInput = HardwarePollHzInput,
            SimulationPollHzInput = SimulationPollHzInput,
            MaxVelocityInput = MaxVelocityInput,
            MaxAccelInput = MaxAccelInput,
            BaudRateInput = BaudRateInput,
            CommandPrefixInput = CommandPrefixInput,
            QueryTimeoutInput = QueryTimeoutInput,
            MotionAckTimeoutInput = MotionAckTimeoutInput,
            SlowTimeoutInput = SlowTimeoutInput,
            OffsetInput = OffsetInput,
            SignInput = SignInput,
            AutoReconnect = AutoReconnect,
            TaughtPoints = _taughtPoints.Select(p => (double[])p.Clone()).ToList(),
            NamedPoses = _namedPoses.ToDictionary(p => p.Key, p => (double[])p.Value.Clone())
        };
    }

    private void SaveConfig()
    {
        PersistenceService.Save(BuildConfig());
        AppendLog($"配置已保存: {PersistenceService.ConfigPath}");
    }

    private void ExportLogs()
    {
        var dialog = new SaveFileDialog
        {
            Filter = "文本文件 (*.txt)|*.txt",
            FileName = $"dummy_log_{DateTime.Now:yyyyMMdd_HHmmss}.txt"
        };
        if (dialog.ShowDialog() != true)
            return;

        try
        {
            File.WriteAllLines(dialog.FileName, Logs.Reverse());
            AppendLog($"日志已导出: {dialog.FileName}");
        }
        catch (Exception ex)
        {
            AppendLog($"日志导出失败: {ex.Message}");
        }
    }

    private void RefreshNamedPoses()
    {
        NamedPoses.Clear();
        foreach (string name in _namedPoses.Keys.OrderBy(k => k))
            NamedPoses.Add(name);
    }

    private void SaveNamedPose()
    {
        string name = NamedPoseNameInput.Trim();
        if (string.IsNullOrWhiteSpace(name))
        {
            AppendLog("请先输入姿态名称");
            return;
        }
        if (_controller.State.JointsDeg.Length != 6)
        {
            AppendLog("尚无有效关节状态，无法保存姿态");
            return;
        }

        _namedPoses[name] = (double[])_controller.State.JointsDeg.Clone();
        RefreshNamedPoses();
        SaveConfig();
        AppendLog($"已保存命名姿态 {name}");
    }

    private void RecallNamedPose()
    {
        string name = NamedPoseNameInput.Trim();
        if (!_namedPoses.TryGetValue(name, out double[]? joints))
        {
            AppendLog($"未找到命名姿态 {name}");
            return;
        }

        JointTargetInput = FormatSix(joints);
        SendJointArray((double[])joints.Clone());
        AppendLog($"已调用命名姿态 {name}");
    }

    private void DeleteNamedPose()
    {
        string name = NamedPoseNameInput.Trim();
        if (!_namedPoses.Remove(name))
        {
            AppendLog($"未找到命名姿态 {name}");
            return;
        }

        RefreshNamedPoses();
        SaveConfig();
        AppendLog($"已删除命名姿态 {name}");
    }

    private void SetDce(string parameter)
    {
        int node = int.TryParse(MotorNodeInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedNode)
            ? Math.Clamp(parsedNode, 1, 6) : 1;
        int value = int.TryParse(DceValueInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedValue)
            ? parsedValue : 0;
        _controller.SetMotorDce(node, parameter, value);
    }

    private void RebootMotor()
    {
        int node = int.TryParse(MotorNodeInput.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsedNode)
            ? Math.Clamp(parsedNode, 1, 6) : 1;
        _controller.RebootMotor(node);
    }

    private void StartConnectionMonitor()
    {
        _monitorCts = new CancellationTokenSource();
        CancellationToken token = _monitorCts.Token;

        _ = Task.Run(async () =>
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                    await Task.Delay(1000, token).ConfigureAwait(false);
                    if (!AutoReconnect || _manualDisconnect || UseSimulation || _client.IsOpen ||
                        string.IsNullOrWhiteSpace(_lastConnectedPort))
                        continue;

                    try
                    {
                        ApplySettings(false);
                        _client.Open(_lastConnectedPort!);
                        _controller.State.IsConnected = true;
                        int pollHz = HardwarePollHz;
                        _controller.StartPolling(1000 / pollHz);
                        DispatchIfNeeded(() =>
                        {
                            UpdateConnectionState();
                            AppendLog($"已自动重连 {_lastConnectedPort}，状态轮询 {pollHz}Hz");
                        });
                    }
                    catch
                    {
                        // 下次循环继续尝试
                    }
                }
            }
            catch (OperationCanceledException)
            {
                // 正常停止
            }
        }, token);
    }

    private double? ParseSpeed()
    {
        return TryParseDouble(SpeedInput, out double speed) ? speed : null;
    }

    private double ParseMaxVelocity()
    {
        return TryParseDouble(MaxVelocityInput, out double value) && value > 0 ? value : 30;
    }

    private double ParseMaxAccel()
    {
        return TryParseDouble(MaxAccelInput, out double value) && value > 0 ? value : 60;
    }

    private void OnStateUpdated(RobotState state)
    {
        void Update()
        {
            JointText = string.Join("   ", state.JointsDeg.Select(v => v.ToString("F1", CultureInfo.InvariantCulture)));
            VelocityText = string.Join("   ", state.JointVelocitiesDegPerSec.Select(v => v.ToString("F1", CultureInfo.InvariantCulture)));
            PoseText = string.Join("   ", state.Pose.Select(v => v.ToString("F1", CultureInfo.InvariantCulture)));
        }

        DispatchIfNeeded(Update);
    }

    private void AppendLog(string message)
    {
        void Add()
        {
            Logs.Insert(0, $"{DateTime.Now:HH:mm:ss}  {message}");
            while (Logs.Count > 200)
                Logs.RemoveAt(Logs.Count - 1);
        }

        DispatchIfNeeded(Add);
    }

    private void UpdateConnectionState()
    {
        IsConnected = _controller.IsConnected;
        ConnectionStatus = IsConnected ? _controller.DisplayName : "未连接";
        RefreshCommands();
    }

    private void RefreshCommands()
    {
        _connectCommand.RaiseCanExecuteChanged();
        _disconnectCommand.RaiseCanExecuteChanged();
        _refreshPortsCommand.RaiseCanExecuteChanged();
        _startCommand.RaiseCanExecuteChanged();
        _stopCommand.RaiseCanExecuteChanged();
        _disableCommand.RaiseCanExecuteChanged();
        _homeCommand.RaiseCanExecuteChanged();
        _resetCommand.RaiseCanExecuteChanged();
        _calibrationCommand.RaiseCanExecuteChanged();
        _sendJointCommand.RaiseCanExecuteChanged();
        _sendPoseCommand.RaiseCanExecuteChanged();
        _jogMinusCommand.RaiseCanExecuteChanged();
        _jogPlusCommand.RaiseCanExecuteChanged();
        _setHomeCurrentCommand.RaiseCanExecuteChanged();
        _setCommandModeCommand.RaiseCanExecuteChanged();
        _getHomeOffsetCommand.RaiseCanExecuteChanged();
        _getBasePosCommand.RaiseCanExecuteChanged();
        _recordPointCommand.RaiseCanExecuteChanged();
        _deletePointCommand.RaiseCanExecuteChanged();
        _clearPointsCommand.RaiseCanExecuteChanged();
        _playPathCommand.RaiseCanExecuteChanged();
        _stopPlayCommand.RaiseCanExecuteChanged();
        _stopJogCommand.RaiseCanExecuteChanged();
        _savePoseCommand.RaiseCanExecuteChanged();
        _recallPoseCommand.RaiseCanExecuteChanged();
        _setDceKpCommand.RaiseCanExecuteChanged();
        _setDceKiCommand.RaiseCanExecuteChanged();
        _setDceKdCommand.RaiseCanExecuteChanged();
        _rebootMotorCommand.RaiseCanExecuteChanged();
        _planPathCommand.RaiseCanExecuteChanged();
        _playPlannedCommand.RaiseCanExecuteChanged();
        _startServoCommand.RaiseCanExecuteChanged();
        _stopServoCommand.RaiseCanExecuteChanged();
    }

    private static void DispatchIfNeeded(Action action)
    {
        System.Windows.Application? app = System.Windows.Application.Current;
        if (app is null || app.Dispatcher.CheckAccess())
            action();
        else
            app.Dispatcher.Invoke(action);
    }

    private static int ToIndex(object? parameter)
    {
        return parameter is int index ? index : -1;
    }

    private int HardwarePollHz => ParsePollHz(HardwarePollHzInput, 10, 1, 30);
    private int SimulationPollHz => ParsePollHz(SimulationPollHzInput, 50, 1, 100);
    private int CurrentPollHz => UseSimulation ? SimulationPollHz : HardwarePollHz;
    private int CurrentPollIntervalMs => 1000 / CurrentPollHz;

    private static int ParsePollHz(string text, int fallback, int min, int max)
    {
        return int.TryParse(text.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out int hz)
               && hz >= min && hz <= max
            ? hz
            : fallback;
    }

    private void ApplyPollInterval(bool log)
    {
        if (!_controller.IsConnected)
            return;

        int hz = CurrentPollHz;
        _controller.StopPolling();
        _controller.StartPolling(1000 / hz);
        if (log)
            AppendLog($"轮询频率已更新为 {hz}Hz");
    }

    private void ConfigureCollisionPlanner(bool log)
    {
        if (!TryParseDoubleArray(CollisionRadiusInput, 7, out double[] radius)
            || !TryParseDoubleArray(CollisionMarginInput, 7, out double[] margin)
            || !TryParseDoubleArray(CollisionGroundInput, 7, out double[] ground))
        {
            if (log)
                AppendLog("碰撞参数格式错误，应为 7 个逗号分隔数字（L0~L6），已保留原参数");
            return;
        }

        try
        {
            _planner.Configure(radius, margin, ground);
        }
        catch (Exception ex)
        {
            if (log)
                AppendLog($"碰撞参数无效，已保留原参数: {ex.Message}");
        }
    }

    private static bool TryParseDoubleArray(string text, int count, out double[] values)
    {
        values = Array.Empty<double>();
        string[] parts = text.Split(',');
        if (parts.Length != count)
            return false;

        var parsed = new double[count];
        for (int i = 0; i < count; i++)
        {
            if (!double.TryParse(parts[i].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out parsed[i])
                || !double.IsFinite(parsed[i]))
                return false;
        }

        values = parsed;
        return true;
    }

    private static bool TryParseSix(string text, out double[] values)
    {
        values = Array.Empty<double>();
        string[] parts = text.Split(',');
        if (parts.Length != 6)
            return false;

        var parsed = new double[6];
        for (int i = 0; i < 6; i++)
        {
            if (!double.TryParse(parts[i].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out parsed[i]))
                return false;
        }

        values = parsed;
        return true;
    }

    private static bool TryParseDouble(string text, out double value)
    {
        return double.TryParse(text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out value);
    }

    private static string FormatSix(double[] values)
    {
        return string.Join(",", values.Select(v => v.ToString("F2", CultureInfo.InvariantCulture)));
    }

    private void OnPropertyChanged([CallerMemberName] string? name = null)
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}

public sealed class JogRow
{
    public int Index { get; init; }
    public string Label { get; init; } = string.Empty;
}
