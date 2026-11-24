namespace DummyWpfUpper.Services;

/// <summary>
/// 3D 视图路径可视化数据源。
/// 路径点均为固件语义关节角(度)，与 IRobotBackend.State.JointsDeg 一致。
/// </summary>
public interface IRobotPathProvider
{
    IReadOnlyList<double[]>? PlannedPath { get; }
    IReadOnlyList<double[]>? TaughtPath { get; }
    event Action? PathsChanged;
}

/// <summary>
/// 3D 视图关节目标数据源。
/// 目标点为固件语义关节角(度)，null 表示当前输入无效。
/// </summary>
public interface IRobotTargetProvider
{
    double[]? TargetJointsFwDeg { get; }
    event Action? TargetChanged;
}
