namespace DummyWpfUpper.Models;

public sealed class RobotState
{
    public double[] JointsDeg { get; set; } = new double[6];
    public double[] JointVelocitiesDegPerSec { get; set; } = new double[6];
    public double[] Pose { get; set; } = new double[6];
    public int QueueSpace { get; set; } = -1;
    public bool IsConnected { get; set; }
}
