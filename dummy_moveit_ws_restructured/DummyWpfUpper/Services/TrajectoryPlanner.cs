using System.Numerics;

namespace DummyWpfUpper.Services;

public sealed record TrajectoryPoint(double[] JointsFwDeg, double DelayMs);

public sealed record CollisionInfo(string Description);

/// <summary>
/// 轨迹规划器，用于规划关节轨迹。
/// </summary>
public sealed class TrajectoryPlanner
{
    private static readonly Vector3[] JointOrigin =
    {
        new(-0.019225f, -0.000523f, 0.100684f),
        new(-0.0155f, 0.035f, 0.0285f),
        new(0.03665f, 0f, 0.146f),
        new(-0.01855f, -0.01f, 0.052f),
        new(0.0168f, 0.127f, 0f),
        new(-0.018506f, 0.077f, -0.000106f)
    };

    private static readonly Vector3[] JointAxis =
    {
        new(0f, 0f, -1f),
        new(-1f, 0f, 0f),
        new(-1f, 0f, 0f),
        new(0f, -1f, 0f),
        new(-1f, 0f, 0f),
        new(0f, -1f, 0f)
    };

    private static readonly Vector3 ToolOffset = new(0.018331f, -0.228477f, -0.327078f);

    private static readonly double[] DefaultRadiusMm = { 45, 20, 25, 20, 25, 30, 20 };
    private static readonly double[] DefaultMarginMm = { 0, 0, 0, 0, 0, 0, 0 };
    private static readonly double[] DefaultGroundMm = { -20, -20, -20, -20, -20, -20, -20 };

    private readonly double[] _segmentRadius = DefaultRadiusMm.Select(v => v / 1000.0).ToArray();
    private readonly double[] _safetyMargin = DefaultMarginMm.Select(v => v / 1000.0).ToArray();
    private readonly double[] _groundZ = DefaultGroundMm.Select(v => v / 1000.0).ToArray();

    private const int SamplesPerSegment = 6;
    private const int SegmentCount = 7;

    /// <summary>
    /// 为 L0~L6 分别设置碰撞检测参数，输入单位均为毫米。
    /// 两段之间的判定阈值为 radius[i] + radius[j] + max(margin[i], margin[j])。
    /// </summary>
    public void Configure(
        IReadOnlyList<double> radiusMm,
        IReadOnlyList<double> marginMm,
        IReadOnlyList<double> groundMm)
    {
        ValidateProfile(radiusMm, nameof(radiusMm), allowNegative: false);
        ValidateProfile(marginMm, nameof(marginMm), allowNegative: false);
        ValidateProfile(groundMm, nameof(groundMm), allowNegative: true);

        for (int i = 0; i < SegmentCount; i++)
        {
            _segmentRadius[i] = radiusMm[i] / 1000.0;
            _safetyMargin[i] = marginMm[i] / 1000.0;
            _groundZ[i] = groundMm[i] / 1000.0;
        }
    }

    private static void ValidateProfile(IReadOnlyList<double> values, string name, bool allowNegative)
    {
        if (values.Count != SegmentCount)
            throw new ArgumentException($"{name} 必须包含 {SegmentCount} 个值（L0~L6）", name);

        for (int i = 0; i < values.Count; i++)
        {
            if (!double.IsFinite(values[i]) || (!allowNegative && values[i] < 0))
                throw new ArgumentException($"{name}[{i}] 不是有效数值", name);
        }
    }

    public List<TrajectoryPoint> PlanTimeParameterized(
        double[] fromFwDeg,
        double[] toFwDeg,
        double maxVelocityDegPerSec,
        double maxAccelDegPerSec2,
        double controlPeriodMs)
    {
        if (fromFwDeg.Length != 6 || toFwDeg.Length != 6)
            throw new ArgumentException("起止点必须是 6 个关节角");

        double velocity = Math.Max(1.0, maxVelocityDegPerSec);
        double accel = Math.Max(1.0, maxAccelDegPerSec2);
        double period = Math.Clamp(controlPeriodMs, 1, 2000) / 1000.0;

        double d = 0;
        for (int i = 0; i < 6; i++)
            d = Math.Max(d, Math.Abs(toFwDeg[i] - fromFwDeg[i]));

        if (d < 1e-6)
            return new List<TrajectoryPoint> { new((double[])toFwDeg.Clone(), 0) };

        double ta;
        double tp;
        if (d <= velocity * velocity / accel)
        {
            ta = Math.Sqrt(d / accel);
            tp = 0;
        }
        else
        {
            ta = velocity / accel;
            tp = d / velocity - ta;
        }
        double total = 2 * ta + tp;

        var result = new List<TrajectoryPoint>();
        for (double t = 0; t < total; t += period)
        {
            double s = ProfileDistance(t, ta, tp, accel, velocity);
            double fraction = Math.Clamp(s / d, 0, 1);
            var point = new double[6];
            for (int i = 0; i < 6; i++)
                point[i] = fromFwDeg[i] + (toFwDeg[i] - fromFwDeg[i]) * fraction;
            result.Add(new TrajectoryPoint(point, period * 1000));
        }

        result.Add(new TrajectoryPoint((double[])toFwDeg.Clone(), 0));
        return result;
    }

    private readonly Dof6Kinematics _kinematics = Dof6Kinematics.CreateDefault();

    /// <summary>
    /// 用与下位机一致的 FK 检查固件语义角姿态的碰撞(地面+自碰撞)。
    /// </summary>
    public CollisionInfo? CheckCollisionFwDeg(double[] jointsFwDeg)
    {
        if (jointsFwDeg.Length != 6)
            return new CollisionInfo("关节数据不是 6 个值");

        double[][] joints = _kinematics.GetLinkPoints(jointsFwDeg);
        var samples = new System.Numerics.Vector3[joints.Length][];
        for (int i = 0; i < joints.Length; i++)
        {
            samples[i] = new System.Numerics.Vector3[SamplesPerSegment + 1];
            System.Numerics.Vector3 start = i == 0
                ? System.Numerics.Vector3.Zero
                : ToVector(joints[i - 1]);
            System.Numerics.Vector3 end = ToVector(joints[i]);
            for (int s = 0; s <= SamplesPerSegment; s++)
            {
                float t = s / (float)SamplesPerSegment;
                samples[i][s] = start + (end - start) * t;
                if (samples[i][s].Z < _groundZ[i])
                    return new CollisionInfo($"连杆 L{i} 低于地面");
            }
        }

        for (int i = 0; i < samples.Length; i++)
        {
            for (int j = i + 2; j < samples.Length; j++)
            {
                if ((samples[i][SamplesPerSegment] - samples[i][0]).Length() < 0.005) continue;
                if ((samples[j][SamplesPerSegment] - samples[j][0]).Length() < 0.005) continue;

                double shared = (samples[i][0] - samples[j][SamplesPerSegment]).Length()
                              + (samples[i][SamplesPerSegment] - samples[j][0]).Length();
                if (shared < 0.001) continue;

                double min = double.MaxValue;
                foreach (System.Numerics.Vector3 a in samples[i])
                    foreach (System.Numerics.Vector3 b in samples[j])
                    {
                        double dist = (a - b).Length();
                        if (dist < min) min = dist;
                    }
                double threshold = _segmentRadius[i] + _segmentRadius[j]
                                   + Math.Max(_safetyMargin[i], _safetyMargin[j]);
                if (min < threshold)
                    return new CollisionInfo(
                        $"连杆 L{i} 与 L{j} 距离 {min * 1000:F0}mm，阈值 {threshold * 1000:F0}mm，存在自碰撞风险");
            }
        }
        return null;
    }

    private static System.Numerics.Vector3 ToVector(double[] point)
    {
        return new System.Numerics.Vector3((float)point[0], (float)point[1], (float)point[2]);
    }

    public CollisionInfo? CheckCollision(double[] rosJointsRad)
    {
        if (rosJointsRad.Length != 6)
            return new CollisionInfo("关节数据不是 6 个值");

        var points = new Vector3[7];
        Matrix4x4 frame = Matrix4x4.Identity;

        for (int i = 0; i < 6; i++)
        {
            points[i] = Vector3.Transform(JointOrigin[i], frame);
            frame = frame * Matrix4x4.CreateTranslation(JointOrigin[i])
                          * Matrix4x4.CreateFromAxisAngle(JointAxis[i], (float)rosJointsRad[i]);
        }
        points[6] = Vector3.Transform(ToolOffset, frame);

        var samples = new Vector3[points.Length][];
        for (int i = 0; i < points.Length; i++)
        {
            samples[i] = new Vector3[SamplesPerSegment + 1];
            Vector3 start = i == 0 ? Vector3.Zero : points[i - 1];
            Vector3 end = points[i];
            for (int s = 0; s <= SamplesPerSegment; s++)
            {
                float t = s / (float)SamplesPerSegment;
                samples[i][s] = start + (end - start) * t;
                if (samples[i][s].Z < _groundZ[i])
                    return new CollisionInfo($"连杆 L{i} 低于地面");
            }
        }

        for (int i = 0; i < samples.Length; i++)
        {
            for (int j = i + 2; j < samples.Length; j++)
            {
                double min = double.MaxValue;
                foreach (Vector3 a in samples[i])
                {
                    foreach (Vector3 b in samples[j])
                    {
                        double dist = (a - b).Length();
                        if (dist < min)
                            min = dist;
                    }
                }
                double threshold = _segmentRadius[i] + _segmentRadius[j]
                                   + Math.Max(_safetyMargin[i], _safetyMargin[j]);
                if (min < threshold)
                    return new CollisionInfo(
                        $"连杆 L{i} 与 L{j} 距离 {min * 1000:F0}mm，阈值 {threshold * 1000:F0}mm，存在自碰撞风险");
            }
        }

        return null;
    }

    private static double ProfileDistance(double t, double ta, double tp, double accel, double velocity)
    {
        if (t <= 0)
            return 0;
        if (t < ta)
            return 0.5 * accel * t * t;

        double plateau = 0.5 * accel * ta * ta;
        if (tp > 0 && t < ta + tp)
            return plateau + velocity * (t - ta);

        double td = Math.Min(t - ta - tp, ta);
        return plateau + velocity * tp + velocity * td - 0.5 * accel * td * td;
    }
}
