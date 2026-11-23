using System.Diagnostics;
using DummyWpfUpper.Models;

namespace DummyWpfUpper.Services;

/// <summary>
/// 纯软件机械臂仿真器：不依赖串口和下位机，用运动学+简单动力学模拟关节运动。
/// 实现了与真机相同的 IRobotBackend 接口，UI 可无缝切换。
/// </summary>
public sealed class SimulationController : IRobotBackend
{
    private static readonly double[] RestPose = { 0, -73, 180, 0, 0, 0 };
    private static readonly double[] OffsetRad = { 0, 0, 1.57079, 0, 0, 0 };
    private static readonly int[] Sign = { 1, 1, 1, 1, -1, -1 };
    private static readonly double[] MinLimits = { -170, -73, 35, -180, -120, -720 };
    private static readonly double[] MaxLimits = { 170, 90, 180, 180, 120, 720 };

    private readonly Dof6Kinematics _kinematics = Dof6Kinematics.CreateDefault();
    private readonly object _sync = new();
    private CancellationTokenSource? _pollCts;

    private double[] _currentJoints = (double[])RestPose.Clone();
    private double[] _targetJoints = (double[])RestPose.Clone();
    private double[] _jointSpeeds = { 60, 60, 60, 90, 90, 120 };
    private bool _enabled;
    private bool _connected;
    private int _queueSpace = 16;

    public RobotState State { get; } = new();

    public bool IsConnected => _connected;

    public string DisplayName => "内置仿真";

    public event Action<string>? Log;
    public event Action<RobotState>? StateUpdated;

    public void Connect(string target)
    {
        _connected = true;
        _targetJoints = (double[])_currentJoints.Clone();
        Log?.Invoke("仿真器已启动");
    }

    public void Disconnect()
    {
        StopPolling();
        _connected = false;
        _enabled = false;
        Log?.Invoke("仿真器已停止");
    }

    public void Configure(double[] offsetRad, int[] sign, int slowCommandTimeoutMs)
    {
        // 仿真器使用自身映射
    }

    public void StartPolling(int intervalMs = 50)
    {
        if (_pollCts is not null)
            return;

        _pollCts = new CancellationTokenSource();
        CancellationToken token = _pollCts.Token;
        var sw = Stopwatch.StartNew();

        _ = Task.Run(async () =>
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                    double dt = Math.Min(0.2, sw.Elapsed.TotalSeconds);
                    sw.Restart();

                    lock (_sync)
                    {
                        var previous = (double[])_currentJoints.Clone();
                        for (int i = 0; i < 6; i++)
                        {
                            if (!_enabled)
                                continue;
                            double delta = _targetJoints[i] - _currentJoints[i];
                            double maxStep = _jointSpeeds[i] * dt;
                            _currentJoints[i] += Math.Clamp(delta, -maxStep, maxStep);
                        }

                        State.JointsDeg = (double[])_currentJoints.Clone();
                        for (int i = 0; i < 6; i++)
                            State.JointVelocitiesDegPerSec[i] = dt > 1e-4
                                ? (_currentJoints[i] - previous[i]) / dt : 0;

                        Pose6D pose = _kinematics.SolveFkMm(_currentJoints);
                        State.Pose = pose.ToArray();
                        State.QueueSpace = _queueSpace;
                        State.IsConnected = _connected;
                    }

                    StateUpdated?.Invoke(State);
                    await Task.Delay(intervalMs, token).ConfigureAwait(false);
                }
            }
            catch (OperationCanceledException)
            {
                // 正常停止
            }
        }, token);
    }

    public void StopPolling()
    {
        _pollCts?.Cancel();
        _pollCts = null;
    }

    public void Start()
    {
        _enabled = true;
        Log?.Invoke("仿真已使能");
    }

    public void Stop()
    {
        _enabled = false;
        _targetJoints = (double[])_currentJoints.Clone();
        Log?.Invoke("仿真已急停");
    }

    public void Disable()
    {
        _enabled = false;
        Log?.Invoke("仿真已失能");
    }

    public void Home()
    {
        SetTarget(RestPose, 60, "仿真回零");
    }

    public void Reset()
    {
        SetTarget(RestPose, 60, "仿真复位");
    }

    public void Calibration() => Log?.Invoke("仿真无需标定");

    public void SetHomeCurrent()
    {
        Log?.Invoke("仿真 HOME 电流设置已忽略");
    }

    public void SetCommandMode(int mode) => Log?.Invoke($"仿真命令模式设为 {mode}");

    public void SetMotorDce(int node, string parameter, int value)
    {
        Log?.Invoke($"仿真电机 {node} DCE_{parameter}={value}");
    }

    public void RebootMotor(int node) => Log?.Invoke($"仿真电机 {node} 重启");

    public double[] GetHomeOffset() => new[] { 0.0, 0.0, 90.0, 0.0, 0.0, 0.0 };

    public double[] GetBasePosition()
    {
        Pose6D pose = _kinematics.SolveFkMm(_currentJoints);
        return new[] { pose.X, pose.Y, pose.Z };
    }

    public int? MoveJ(double[] jointsDeg, double? speedDeg)
    {
        if (jointsDeg.Length != 6)
            return null;
        for (int i = 0; i < 6; i++)
        {
            if (jointsDeg[i] < MinLimits[i] || jointsDeg[i] > MaxLimits[i])
            {
                Log?.Invoke($"仿真目标超出限位: J{i + 1}={jointsDeg[i]:F2}");
                return null;
            }
        }

        SetTarget(jointsDeg, speedDeg ?? 60, "仿真关节目标");
        return _queueSpace;
    }

    public int? MoveL(double[] poseMmDeg, double? speed)
    {
        if (poseMmDeg.Length != 6)
            return null;

        var pose = new Pose6D(poseMmDeg[0], poseMmDeg[1], poseMmDeg[2],
            poseMmDeg[3], poseMmDeg[4], poseMmDeg[5]);
        Joint6D? solution = _kinematics.SolveIkBest(pose, _currentJoints, MinLimits, MaxLimits);
        if (solution is null)
        {
            Log?.Invoke("仿真 IK 失败: 无有效解");
            return null;
        }

        int? result = MoveJ(solution.A, speed);
        if (result.HasValue)
            Log?.Invoke("仿真笛卡尔目标已解算并执行");
        return result;
    }

    public double[] RosRadToFirmwareDeg(double[] rosRad)
    {
        var result = new double[6];
        for (int i = 0; i < 6; i++)
            result[i] = (rosRad[i] + OffsetRad[i]) * Sign[i] * 180.0 / Math.PI;
        return result;
    }

    public double[] FirmwareDegToRosRad(double[] firmwareDeg)
    {
        var result = new double[6];
        for (int i = 0; i < 6; i++)
            result[i] = firmwareDeg[i] * Sign[i] * Math.PI / 180.0 - OffsetRad[i];
        return result;
    }

    private void SetTarget(double[] target, double speedDegPerSec, string logMessage)
    {
        lock (_sync)
        {
            _targetJoints = (double[])target.Clone();
            for (int i = 0; i < 6; i++)
                _jointSpeeds[i] = Math.Max(5, Math.Abs(speedDegPerSec));
        }
        Log?.Invoke(logMessage);
    }
}
