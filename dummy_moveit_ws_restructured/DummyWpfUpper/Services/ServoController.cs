using System.Diagnostics;
using DummyWpfUpper.Models;

namespace DummyWpfUpper.Services;

/// <summary>
/// 笛卡尔速度伺服循环：以固定周期把 twist 转换为关节增量并下发。
/// </summary>
public sealed class ServoController
{
    private readonly IRobotBackend _backend;
    private readonly CartesianServo _servo;
    private readonly TrajectoryPlanner _collision;
    private readonly double[] _min;
    private readonly double[] _max;

    private CancellationTokenSource? _cts;
    private double[] _twist = new double[6];
    private ServoFrame _frame = ServoFrame.Base;
    private double[] _joints = new double[6];

    public bool IsRunning => _cts is not null;

    public event Action<string>? Log;

    public ServoController(IRobotBackend backend, CartesianServo servo, TrajectoryPlanner collision,
        double[] minLimits, double[] maxLimits)
    {
        _backend = backend;
        _servo = servo;
        _collision = collision;
        _min = (double[])minLimits.Clone();
        _max = (double[])maxLimits.Clone();
    }

    public void Start(double[] twist, ServoFrame frame, int intervalMs = 33)
    {
        if (_cts is not null)
            Stop();

        if (_backend.State.JointsDeg.Length != 6)
        {
            Log?.Invoke("Servo 启动失败：无有效关节状态");
            return;
        }

        _twist = (double[])twist.Clone();
        _frame = frame;
        _joints = (double[])_backend.State.JointsDeg.Clone();
        _cts = new CancellationTokenSource();
        CancellationToken token = _cts.Token;
        var sw = Stopwatch.StartNew();

        Log?.Invoke($"Servo 启动：{frame} 系 twist=[{string.Join(",", _twist.Select(v => v.ToString("F1")))}]");

        _ = Task.Run(async () =>
        {
            try
            {
                while (!token.IsCancellationRequested)
                {
                    double dt = Math.Min(0.1, sw.Elapsed.TotalSeconds);
                    sw.Restart();
                    if (dt < 1e-4)
                    {
                        await Task.Delay(intervalMs, token).ConfigureAwait(false);
                        continue;
                    }

                    ServoResult result = _servo.Compute(_joints, _twist, _frame);
                    var next = new double[6];
                    for (int i = 0; i < 6; i++)
                        next[i] = _joints[i] + result.JointVelocitiesDegPerSec[i] * dt;

                    if (!WithinLimits(next))
                    {
                        Log?.Invoke("Servo 停止：目标超出关节限位");
                        break;
                    }
                    if (_collision.CheckCollisionFwDeg(next) is not null)
                    {
                        Log?.Invoke("Servo 停止：检测到碰撞风险");
                        break;
                    }

                    _joints = next;
                    _backend.MoveJ(next, null);
                    await Task.Delay(intervalMs, token).ConfigureAwait(false);
                }
            }
            catch (OperationCanceledException)
            {
                // 正常停止
            }
            finally
            {
                _cts = null;
                Log?.Invoke("Servo 已停止");
            }
        }, token);
    }

    public void Stop()
    {
        _cts?.Cancel();
        _cts = null;
    }

    public void UpdateTwist(double[] twist, ServoFrame frame)
    {
        _twist = (double[])twist.Clone();
        _frame = frame;
    }

    private bool WithinLimits(double[] joints)
    {
        for (int i = 0; i < 6; i++)
            if (joints[i] < _min[i] || joints[i] > _max[i])
                return false;
        return true;
    }
}
