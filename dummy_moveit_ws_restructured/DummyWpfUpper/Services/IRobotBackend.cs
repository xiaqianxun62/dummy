using DummyWpfUpper.Models;

namespace DummyWpfUpper.Services;

/// <summary>
/// 真机与仿真后端的统一接口。
/// 所有命令、状态和事件都通过此接口，UI 无需关心当前是串口还是软件仿真。
/// </summary>
public interface IRobotBackend
{
    RobotState State { get; }

    event Action<string>? Log;
    event Action<RobotState>? StateUpdated;

    bool IsConnected { get; }
    string DisplayName { get; }

    void Connect(string target);
    void Disconnect();
    void StartPolling(int intervalMs = 50);
    void StopPolling();
    void Configure(double[] offsetRad, int[] sign, int slowCommandTimeoutMs);

    void Start();
    void Stop();
    void Disable();
    void Home();
    void Reset();
    void Calibration();
    void SetHomeCurrent();
    void SetCommandMode(int mode);

    void SetMotorDce(int node, string parameter, int value);
    void RebootMotor(int node);

    double[] GetHomeOffset();
    double[] GetBasePosition();

    int? MoveJ(double[] jointsDeg, double? speedDeg);
    int? MoveL(double[] poseMmDeg, double? speed);

    double[] RosRadToFirmwareDeg(double[] rosRad);
    double[] FirmwareDegToRosRad(double[] firmwareDeg);
}