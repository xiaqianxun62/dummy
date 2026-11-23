using DummyWpfUpper.Models;

namespace DummyWpfUpper.Services;

public sealed class RobotController : IRobotBackend
{
    private double[] _offsetRad = { 0, 0, 1.57079, 0, 0, 0 };
    private int[] _sign = { 1, 1, 1, 1, -1, -1 };
    private int _slowCommandTimeoutMs = 15000;

    private static readonly (double Min, double Max)[] JointLimits =
    {
        (-170.0, 170.0),
        (-73.0, 90.0),
        (35.0, 180.0),
        (-180.0, 180.0),
        (-120.0, 120.0),
        (-720.0, 720.0)
    };

    private const double CommandEpsilon = 1e-4;

    private readonly SerialAsciiClient _client;
    private CancellationTokenSource? _pollCts;
    private double[]? _lastJointSample;
    private DateTime _lastJointSampleTime;
    private double[]? _lastJointTarget;
    private double[]? _lastPoseTarget;

    public RobotState State { get; } = new();

    public bool IsConnected => _client.IsOpen;

    public string DisplayName => $"串口直连 {_client.PortName}";

    public event Action<string>? Log;
    public event Action<RobotState>? StateUpdated;

    public RobotController(SerialAsciiClient client)
    {
        _client = client;
    }

    public void Configure(double[] offsetRad, int[] sign, int slowCommandTimeoutMs)
    {
        if (offsetRad.Length == 6)
            _offsetRad = (double[])offsetRad.Clone();
        if (sign.Length == 6)
            _sign = (int[])sign.Clone();
        _slowCommandTimeoutMs = slowCommandTimeoutMs > 0 ? slowCommandTimeoutMs : 15000;
    }

    public void Connect(string portName)
    {
        _client.Open(portName);
        _lastJointSample = null;
        _lastJointTarget = null;
        _lastPoseTarget = null;
        State.IsConnected = true;
        Log?.Invoke($"已连接 {portName}");
    }

    public void Disconnect()
    {
        StopPolling();

        if (_client.IsOpen)
        {
            try
            {
                _client.SystemCommand("!STOP", "Stopped ok", 500);
                Log?.Invoke("断开前已发送 !STOP 急停");
            }
            catch (Exception ex)
            {
                Log?.Invoke($"断开前 !STOP 失败: {ex.Message}");
            }
        }

        _client.Close();
        State.IsConnected = false;
        State.QueueSpace = -1;
        _lastJointTarget = null;
        _lastPoseTarget = null;
        Log?.Invoke("已断开串口");
    }

    public void StartPolling(int intervalMs = 50)
    {
        if (_pollCts is not null)
            return;

        _pollCts = new CancellationTokenSource();
        CancellationToken token = _pollCts.Token;

        _ = Task.Run(async () =>
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                    try
                    {
                        if (_client.IsOpen)
                        {
                            double[] joints = _client.GetJointPositions();
                            DateTime now = DateTime.UtcNow;
                            if (_lastJointSample is not null)
                            {
                                double dt = (now - _lastJointSampleTime).TotalSeconds;
                                if (dt > 1e-3)
                                {
                                    var velocities = new double[6];
                                    for (int i = 0; i < 6; i++)
                                        velocities[i] = (joints[i] - _lastJointSample[i]) / dt;
                                    State.JointVelocitiesDegPerSec = velocities;
                                }
                            }
                            _lastJointSample = (double[])joints.Clone();
                            _lastJointSampleTime = now;
                            State.JointsDeg = joints;
                            State.Pose = _client.GetLinearPose();
                            StateUpdated?.Invoke(State);
                        }
                    }
                    catch (Exception ex)
                    {
                        Log?.Invoke($"状态轮询失败: {ex.Message}");
                    }

                    await Task.Delay(intervalMs, token).ConfigureAwait(false);
                }
            }
            catch (OperationCanceledException)
            {
                // 正常停止轮询
            }
        }, token);
    }

    public void StopPolling()
    {
        _pollCts?.Cancel();
        _pollCts = null;
    }

    public void Start() => ExecuteSystem("!START", "Started ok", 500, "已使能 (START)");

    public void Stop() => ExecuteSystem("!STOP", "Stopped ok", 500, "已急停 (STOP)");

    public void Disable() => ExecuteSystem("!DISABLE", "Disabled ok", 500, "已失能 (DISABLE)");

    public void Home() => ExecuteSystem("!HOME", "Started ok", _slowCommandTimeoutMs, "回零指令已执行");

    public void Reset() => ExecuteSystem("!RESET", "Started ok", _slowCommandTimeoutMs, "复位指令已执行");

    public void Calibration() => ExecuteSystem("!CALIBRATION", "calibration ok", _slowCommandTimeoutMs + 5000, "标定指令已执行");

    public void SetHomeCurrent() => ExecuteSystem("!SET_HOME_CURRENT", "SET_HOME_CURRENT", 500, "已设置 HOME 电流");

    public void SetCommandMode(int mode)
    {
        try
        {
            _client.SetCommandMode(mode);
            Log?.Invoke($"已设置命令模式 {mode}");
        }
        catch (Exception ex)
        {
            Log?.Invoke($"设置命令模式失败: {ex.Message}");
        }
    }

    public void SetMotorDce(int node, string parameter, int value)
    {
        try
        {
            _client.SetMotorDce(node, parameter, value);
            Log?.Invoke($"已设置电机 {node} 的 DCE_{parameter} = {value}");
        }
        catch (Exception ex)
        {
            Log?.Invoke($"设置 DCE_{parameter} 失败: {ex.Message}");
        }
    }

    public void RebootMotor(int node)
    {
        try
        {
            _client.RebootMotor(node);
            Log?.Invoke($"已请求重启电机 {node}");
        }
        catch (Exception ex)
        {
            Log?.Invoke($"重启电机失败: {ex.Message}");
        }
    }

    public double[] GetHomeOffset()
    {
        try
        {
            double[] values = _client.GetHomeOffset();
            Log?.Invoke($"HOME 偏移: {Format(values)}");
            return values;
        }
        catch (Exception ex)
        {
            Log?.Invoke($"读取 HOME 偏移失败: {ex.Message}");
            return Array.Empty<double>();
        }
    }

    public double[] GetBasePosition()
    {
        try
        {
            double[] values = _client.GetBasePosition();
            Log?.Invoke($"基座位置: {Format(values)}");
            return values;
        }
        catch (Exception ex)
        {
            Log?.Invoke($"读取基座位置失败: {ex.Message}");
            return Array.Empty<double>();
        }
    }

    public static bool IsJointTargetValid(double[] jointsDeg, out string error)
    {
        if (jointsDeg.Length != 6)
        {
            error = "需要 6 个关节角";
            return false;
        }

        for (int i = 0; i < 6; i++)
        {
            if (jointsDeg[i] < JointLimits[i].Min || jointsDeg[i] > JointLimits[i].Max)
            {
                error = $"J{i + 1}={jointsDeg[i]:F2} 超出限位 [{JointLimits[i].Min:F0}, {JointLimits[i].Max:F0}]";
                return false;
            }
        }

        error = string.Empty;
        return true;
    }

    public int? MoveJ(double[] jointsDeg, double? speedDeg)
    {
        if (!IsJointTargetValid(jointsDeg, out string error))
        {
            Log?.Invoke($"关节目标非法，未发送: {error}");
            return null;
        }

        if (IsSame(_lastJointTarget, jointsDeg))
        {
            Log?.Invoke("关节目标与上次相同，跳过发送");
            return null;
        }

        int queueSpace = _client.MoveJoint(jointsDeg, speedDeg);
        _lastJointTarget = (double[])jointsDeg.Clone();
        _lastPoseTarget = null;
        State.QueueSpace = queueSpace;
        return queueSpace;
    }

    public int? MoveL(double[] poseMmDeg, double? speed)
    {
        if (poseMmDeg.Length != 6)
        {
            Log?.Invoke("笛卡尔目标必须是 6 个值，未发送");
            return null;
        }

        if (IsSame(_lastPoseTarget, poseMmDeg))
        {
            Log?.Invoke("位姿目标与上次相同，跳过发送");
            return null;
        }

        int queueSpace = _client.MovePose(poseMmDeg, speed);
        _lastPoseTarget = (double[])poseMmDeg.Clone();
        _lastJointTarget = null;
        State.QueueSpace = queueSpace;
        return queueSpace;
    }

    public double[] RosRadToFirmwareDeg(double[] rosRad)
    {
        if (rosRad.Length != 6)
            throw new ArgumentException("需要 6 个 ROS 关节角(弧度)", nameof(rosRad));

        var result = new double[6];
        for (int i = 0; i < 6; i++)
            result[i] = (rosRad[i] + _offsetRad[i]) * _sign[i] * 180.0 / Math.PI;
        return result;
    }

    public double[] FirmwareDegToRosRad(double[] firmwareDeg)
    {
        if (firmwareDeg.Length != 6)
            throw new ArgumentException("需要 6 个固件关节角(度)", nameof(firmwareDeg));

        var result = new double[6];
        for (int i = 0; i < 6; i++)
            result[i] = firmwareDeg[i] * _sign[i] * Math.PI / 180.0 - _offsetRad[i];
        return result;
    }

    public static List<double[]> InterpolateJointPath(double[] from, double[] to, int steps)
    {
        if (from.Length != 6 || to.Length != 6)
            throw new ArgumentException("起止点都必须是 6 个关节角", nameof(from));
        if (steps < 2)
            throw new ArgumentException("steps 至少为 2", nameof(steps));

        var path = new List<double[]>();
        for (int s = 0; s < steps; s++)
        {
            double t = s / (double)(steps - 1);
            var point = new double[6];
            for (int i = 0; i < 6; i++)
                point[i] = from[i] + (to[i] - from[i]) * t;
            path.Add(point);
        }
        return path;
    }

    private static bool IsSame(double[]? last, double[] current)
    {
        if (last is null || last.Length != current.Length)
            return false;

        for (int i = 0; i < current.Length; i++)
        {
            if (Math.Abs(last[i] - current[i]) > CommandEpsilon)
                return false;
        }
        return true;
    }

    private static string Format(double[] values)
    {
        return string.Join(", ", values.Select(v => v.ToString("F2")));
    }

    private void ExecuteSystem(string command, string expected, int timeoutMs, string successMessage)
    {
        try
        {
            _client.SystemCommand(command, expected, timeoutMs);
            Log?.Invoke(successMessage);
        }
        catch (Exception ex)
        {
            Log?.Invoke($"{command} 失败: {ex.Message}");
        }
    }
}
