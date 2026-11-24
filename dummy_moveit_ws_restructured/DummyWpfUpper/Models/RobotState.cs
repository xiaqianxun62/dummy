namespace DummyWpfUpper.Models;

public sealed class RobotState
{
    // 关节角度
    public double[] JointsDeg { get; set; } = new double[6];
    // 关节速度
    public double[] JointVelocitiesDegPerSec { get; set; } = new double[6];
    // 笛卡尔位姿
    public double[] Pose { get; set; } = new double[6];
    // 队列空间
    public int QueueSpace { get; set; } = -1;
    // 是否连接
    public bool IsConnected { get; set; }
}
