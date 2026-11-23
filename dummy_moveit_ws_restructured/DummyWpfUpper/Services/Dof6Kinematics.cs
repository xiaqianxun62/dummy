using System;
using System.Linq;

namespace DummyWpfUpper.Services;

public sealed class Joint6D
{
    public double[] A { get; } = new double[6];

    public Joint6D() { }

    public Joint6D(double a0, double a1, double a2, double a3, double a4, double a5)
    {
        A[0] = a0; A[1] = a1; A[2] = a2; A[3] = a3; A[4] = a4; A[5] = a5;
    }

    public Joint6D(double[] values)
    {
        if (values.Length != 6)
            throw new ArgumentException("关节必须是 6 个值", nameof(values));
        Array.Copy(values, A, 6);
    }

    public Joint6D Clone() => new Joint6D(A);
}

public sealed class Pose6D
{
    public double X, Y, Z;
    public double A, B, C;
    public double[] R = new double[9];
    public bool HasR;

    public Pose6D() { }

    public Pose6D(double x, double y, double z, double a, double b, double c)
    {
        X = x; Y = y; Z = z; A = a; B = b; C = c;
    }

    public double[] ToArray() => new[] { X, Y, Z, A, B, C };
}

public sealed class IkSolves
{
    public Joint6D[] Config { get; } = Enumerable.Range(0, 8).Select(_ => new Joint6D()).ToArray();
    public int[,] SolFlag { get; } = new int[8, 3];
}

/// <summary>
/// 移植自下位机 6dof_kinematic.cpp，DH 参数与算法完全一致。
/// 底层约定：SolveFK 输出位置单位为米，SolveIK 输入位置单位为毫米。
/// 高层方法 *Mm / *Deg 统一对外使用 mm / deg。
/// </summary>
public sealed class Dof6Kinematics
{
    private const double RadToDeg = 57.295777754771045;
    private const double DegToRad = Math.PI / 180.0;
    private const double HalfPi = Math.PI / 2.0;

    private readonly double[,] _dh = new double[6, 4];
    private readonly double[] _l1Base = new double[3];
    private readonly double[] _l2Arm = new double[3];
    private readonly double[] _l3Elbow = new double[3];
    private readonly double[] _l6Wrist = new double[3];

    private readonly double _lBase, _dBase, _lArm, _lForearm, _dElbow, _lWrist;
    private double _lSe2, _lSe, _lEw2, _lEw, _atanE;

    public Dof6Kinematics(double lBase, double dBse, double lArm, double lForearm, double dElbow, double lWrist)
    {
        _lBase = lBase; _dBase = dBse; _lArm = lArm;
        _lForearm = lForearm; _dElbow = dElbow; _lWrist = lWrist;

        _dh[0, 0] = 0;       _dh[0, 1] = lBase;    _dh[0, 2] = dBse;    _dh[0, 3] = -HalfPi;
        _dh[1, 0] = -HalfPi; _dh[1, 1] = 0;        _dh[1, 2] = lArm;    _dh[1, 3] = 0;
        _dh[2, 0] = HalfPi;  _dh[2, 1] = dElbow;   _dh[2, 2] = 0;       _dh[2, 3] = HalfPi;
        _dh[3, 0] = 0;       _dh[3, 1] = lForearm; _dh[3, 2] = 0;       _dh[3, 3] = -HalfPi;
        _dh[4, 0] = 0;       _dh[4, 1] = 0;        _dh[4, 2] = 0;       _dh[4, 3] = HalfPi;
        _dh[5, 0] = 0;       _dh[5, 1] = lWrist;   _dh[5, 2] = 0;       _dh[5, 3] = 0;

        _l1Base[0] = dBse;   _l1Base[1] = -lBase;  _l1Base[2] = 0;
        _l2Arm[0] = lArm;    _l2Arm[1] = 0;        _l2Arm[2] = 0;
        _l3Elbow[0] = -dElbow; _l3Elbow[1] = 0;    _l3Elbow[2] = lForearm;
        _l6Wrist[0] = 0;     _l6Wrist[1] = 0;      _l6Wrist[2] = lWrist;

        _lSe2 = lArm * lArm;
        _lSe = lArm;
        _lEw2 = lForearm * lForearm + dElbow * dElbow;
        _lEw = 0;
        _atanE = 0;
    }

    public static Dof6Kinematics CreateDefault() => new(0.109, 0.035, 0.146, 0.115, 0.052, 0.072);

    // ---------- 高层 API：统一对外使用 mm / deg ----------

    public Pose6D SolveFkMm(double[] jointsDeg)
    {
        SolveFk(new Joint6D(jointsDeg), out Pose6D pose);
        pose.X *= 1000; pose.Y *= 1000; pose.Z *= 1000;
        return pose;
    }

    /// <summary>
    /// 与下位机 MoveL 一致的选解：过滤限位后，取与当前关节角“最大单轴差”最小的解。
    /// </summary>
    public Joint6D? SolveIkBest(Pose6D poseMm, double[] currentDeg,
        double[]? minLimits = null, double[]? maxLimits = null)
    {
        SolveIk(poseMm, new Joint6D(), out IkSolves solves);

        int best = -1;
        double bestMax = double.MaxValue;
        for (int i = 0; i < 8; i++)
        {
            double[] angles = solves.Config[i].A;
            bool valid = true;
            for (int j = 0; j < 6; j++)
            {
                if (minLimits is not null && angles[j] < minLimits[j]) { valid = false; break; }
                if (maxLimits is not null && angles[j] > maxLimits[j]) { valid = false; break; }
            }
            if (!valid) continue;

            double maxDiff = 0;
            for (int j = 0; j < 6; j++)
                maxDiff = Math.Max(maxDiff, Math.Abs(angles[j] - currentDeg[j]));

            if (maxDiff < bestMax)
            {
                bestMax = maxDiff;
                best = i;
            }
        }

        return best < 0 ? null : solves.Config[best].Clone();
    }

    /// <summary>
    /// 返回各连杆关键点(单位 m)，用于碰撞检测：基座、J1~J3 位置、腕部、末端。
    /// 与 SolveFK 使用同一套 DH 变换，保证几何一致。
    /// </summary>
    public double[][] GetLinkPoints(double[] jointsDeg)
    {
        BuildFrames(jointsDeg, out double[][] r, out double[] r02, out double[] r03, out double[] r06);
        double[] l0bs = Mul(r[0], _l1Base, 3, 3, 1);
        double[] l0se = Mul(r02, _l2Arm, 3, 3, 1);
        double[] l0ew = Mul(r03, _l3Elbow, 3, 3, 1);
        double[] l0wt = Mul(r06, _l6Wrist, 3, 3, 1);

        var origin = new double[3];
        var p1 = l0bs;
        var p2 = new double[3];
        var p3 = new double[3];
        var pEnd = new double[3];
        for (int i = 0; i < 3; i++)
        {
            p2[i] = p1[i] + l0se[i];
            p3[i] = p2[i] + l0ew[i];
            pEnd[i] = p3[i] + l0wt[i];
        }
        return new[] { origin, p1, p2, p3, pEnd };
    }

    private void BuildFrames(double[] jointsDeg, out double[][] r, out double[] r02, out double[] r03, out double[] r06)
    {
        r = new double[6][];
        for (int i = 0; i < 6; i++)
        {
            r[i] = new double[9];
            double q = jointsDeg[i] / RadToDeg + _dh[i, 0];
            double cosq = Math.Cos(q), sinq = Math.Sin(q);
            double cosa = Math.Cos(_dh[i, 3]), sina = Math.Sin(_dh[i, 3]);

            r[i][0] = cosq;  r[i][1] = -cosa * sinq; r[i][2] = sina * sinq;
            r[i][3] = sinq;  r[i][4] = cosa * cosq;  r[i][5] = -sina * cosq;
            r[i][6] = 0;     r[i][7] = sina;        r[i][8] = cosa;
        }
        r02 = Mul(r[0], r[1], 3, 3, 3);
        r03 = Mul(r02, r[2], 3, 3, 3);
        double[] r04 = Mul(r03, r[3], 3, 3, 3);
        double[] r05 = Mul(r04, r[4], 3, 3, 3);
        r06 = Mul(r05, r[5], 3, 3, 3);
    }

    /// <summary>
    /// 数值雅可比：行 0-2 为位置(mm/rad输入对应deg输出)，行 3-5 为姿态(deg/deg)。
    /// </summary>
    public double[,] ComputeJacobian(double[] jointsDeg)
    {
        const double eps = 1e-4;
        Pose6D p0 = SolveFkMm(jointsDeg);
        var jac = new double[6, 6];

        for (int i = 0; i < 6; i++)
        {
            double[] q = (double[])jointsDeg.Clone();
            q[i] += eps;
            Pose6D p1 = SolveFkMm(q);

            jac[0, i] = (p1.X - p0.X) / eps;
            jac[1, i] = (p1.Y - p0.Y) / eps;
            jac[2, i] = (p1.Z - p0.Z) / eps;
            jac[3, i] = WrapDeg(p1.A - p0.A) / eps;
            jac[4, i] = WrapDeg(p1.B - p0.B) / eps;
            jac[5, i] = WrapDeg(p1.C - p0.C) / eps;
        }
        return jac;
    }

    // ---------- 底层移植：与 6dof_kinematic.cpp 一一对应 ----------

    public bool SolveFk(Joint6D input, out Pose6D output)
    {
        var qIn = new double[6];
        var q = new double[6];
        var r = new double[6][];
        for (int i = 0; i < 6; i++)
        {
            r[i] = new double[9];
            qIn[i] = input.A[i] / RadToDeg;
        }

        for (int i = 0; i < 6; i++)
        {
            q[i] = qIn[i] + _dh[i, 0];
            double cosq = Math.Cos(q[i]), sinq = Math.Sin(q[i]);
            double cosa = Math.Cos(_dh[i, 3]), sina = Math.Sin(_dh[i, 3]);

            r[i][0] = cosq;  r[i][1] = -cosa * sinq; r[i][2] = sina * sinq;
            r[i][3] = sinq;  r[i][4] = cosa * cosq;  r[i][5] = -sina * cosq;
            r[i][6] = 0;     r[i][7] = sina;        r[i][8] = cosa;
        }

        double[] r02 = Mul(r[0], r[1], 3, 3, 3);
        double[] r03 = Mul(r02, r[2], 3, 3, 3);
        double[] r04 = Mul(r03, r[3], 3, 3, 3);
        double[] r05 = Mul(r04, r[4], 3, 3, 3);
        double[] r06 = Mul(r05, r[5], 3, 3, 3);

        double[] l0bs = Mul(r[0], _l1Base, 3, 3, 1);
        double[] l0se = Mul(r02, _l2Arm, 3, 3, 1);
        double[] l0ew = Mul(r03, _l3Elbow, 3, 3, 1);
        double[] l0wt = Mul(r06, _l6Wrist, 3, 3, 1);

        var p06 = new double[6];
        for (int i = 0; i < 3; i++)
            p06[i] = l0bs[i] + l0se[i] + l0ew[i] + l0wt[i];

        RotMatToEuler(r06, out double ea, out double eb, out double ec);
        p06[3] = ec; p06[4] = eb; p06[5] = ea;

        output = new Pose6D
        {
            X = p06[0], Y = p06[1], Z = p06[2],
            A = p06[3] * RadToDeg, B = p06[4] * RadToDeg, C = p06[5] * RadToDeg
        };
        Array.Copy(r06, output.R, 9);
        return true;
    }

    public bool SolveIk(Pose6D input, Joint6D last, out IkSolves output)
    {
        output = new IkSolves();

        var qs = new double[2];
        var qa = new double[2, 2];
        var qw = new double[2, 3];
        var p06 = new double[6];
        var p0w = new double[3];
        var p1w = new double[3];
        var l0wt = new double[3];
        var l1sw = new double[3];

        if (_lEw == 0)
        {
            _lEw = Math.Sqrt(_lEw2);
            _atanE = Math.Atan(_dElbow / _lForearm);
        }

        p06[0] = input.X / 1000.0;
        p06[1] = input.Y / 1000.0;
        p06[2] = input.Z / 1000.0;
        double[] R06;
        if (!input.HasR)
        {
            p06[3] = input.A / RadToDeg;
            p06[4] = input.B / RadToDeg;
            p06[5] = input.C / RadToDeg;
            EulerToRotMat(p06[3], p06[4], p06[5], out R06);
        }
        else
        {
            R06 = (double[])input.R.Clone();
        }

        for (int i = 0; i < 2; i++)
        {
            qs[i] = last.A[0];
            qa[i, 0] = last.A[1]; qa[i, 1] = last.A[2];
            qw[i, 0] = last.A[3]; qw[i, 1] = last.A[4]; qw[i, 2] = last.A[5];
        }

        l0wt = Mul(R06, _l6Wrist, 3, 3, 1);
        for (int i = 0; i < 3; i++) p0w[i] = p06[i] - l0wt[i];

        if (Math.Sqrt(p0w[0] * p0w[0] + p0w[1] * p0w[1]) <= 0.000001)
        {
            qs[0] = last.A[0]; qs[1] = last.A[0];
            for (int i = 0; i < 4; i++)
            {
                output.SolFlag[i, 0] = -1; output.SolFlag[4 + i, 0] = -1;
            }
        }
        else
        {
            qs[0] = Math.Atan2(p0w[1], p0w[0]);
            qs[1] = Math.Atan2(-p0w[1], -p0w[0]);
            for (int i = 0; i < 4; i++)
            {
                output.SolFlag[i, 0] = 1; output.SolFlag[4 + i, 0] = 1;
            }
        }

        for (int indArm = 0; indArm < 2; indArm++)
        {
            double cosqs = Math.Cos(qs[indArm] + _dh[0, 0]);
            double sinqs = Math.Sin(qs[indArm] + _dh[0, 0]);

            double[] r10 =
            {
                cosqs, sinqs, 0,
                0, 0, -1,
                -sinqs, cosqs, 0
            };

            p1w = Mul(r10, p0w, 3, 3, 1);
            for (int i = 0; i < 3; i++) l1sw[i] = p1w[i] - _l1Base[i];

            double lsw2 = l1sw[0] * l1sw[0] + l1sw[1] * l1sw[1];
            double lsw = Math.Sqrt(lsw2);

            if (Math.Abs(_lSe + _lEw - lsw) <= 0.000001)
            {
                qa[indArm, 0] = Math.Atan2(l1sw[1], l1sw[0]);
                qa[1 - indArm, 0] = qa[indArm, 0];
                qa[indArm, 1] = 0; qa[1 - indArm, 1] = 0;
                for (int i = 0; i < 2; i++)
                {
                    int flag = lsw > _lSe + _lEw ? 0 : 1;
                    output.SolFlag[4 * indArm + i, 1] = flag;
                    output.SolFlag[4 * indArm + 2 + i, 1] = flag;
                }
            }
            else if (Math.Abs(lsw - Math.Abs(_lSe - _lEw)) <= 0.000001)
            {
                qa[indArm, 0] = Math.Atan2(l1sw[1], l1sw[0]);
                qa[1 - indArm, 0] = qa[indArm, 0];
                if (indArm == 0) { qa[0, 1] = Math.PI; qa[1, 1] = -Math.PI; }
                else { qa[0, 1] = -Math.PI; qa[1, 1] = Math.PI; }
                for (int i = 0; i < 2; i++)
                {
                    int flag = lsw < Math.Abs(_lSe - _lEw) ? 0 : 1;
                    output.SolFlag[4 * indArm + i, 1] = flag;
                    output.SolFlag[4 * indArm + 2 + i, 1] = flag;
                }
            }
            else
            {
                double atanA = Math.Atan2(l1sw[1], l1sw[0]);
                double acosA = 0.5 * (_lSe2 + lsw2 - _lEw2) / (_lSe * lsw);
                acosA = acosA >= 1.0 ? 0 : acosA <= -1.0 ? Math.PI : Math.Acos(acosA);
                double acosE = 0.5 * (_lSe2 + _lEw2 - lsw2) / (_lSe * _lEw);
                acosE = acosE >= 1.0 ? 0 : acosE <= -1.0 ? Math.PI : Math.Acos(acosE);

                if (indArm == 0)
                {
                    qa[0, 0] = atanA - acosA + HalfPi;
                    qa[0, 1] = _atanE - acosE + Math.PI;
                    qa[1, 0] = atanA + acosA + HalfPi;
                    qa[1, 1] = _atanE + acosE - Math.PI;
                }
                else
                {
                    qa[0, 0] = atanA + acosA + HalfPi;
                    qa[0, 1] = _atanE + acosE - Math.PI;
                    qa[1, 0] = atanA - acosA + HalfPi;
                    qa[1, 1] = _atanE - acosE + Math.PI;
                }
                for (int i = 0; i < 2; i++)
                {
                    output.SolFlag[4 * indArm + i, 1] = 1;
                    output.SolFlag[4 * indArm + 2 + i, 1] = 1;
                }
            }

            for (int indElbow = 0; indElbow < 2; indElbow++)
            {
                double cq0 = Math.Cos(qa[indElbow, 0] + _dh[1, 0]);
                double sq0 = Math.Sin(qa[indElbow, 0] + _dh[1, 0]);
                double cq1 = Math.Cos(qa[indElbow, 1] + _dh[2, 0]);
                double sq1 = Math.Sin(qa[indElbow, 1] + _dh[2, 0]);

                double[] r31 =
                {
                    cq0 * cq1 - sq0 * sq1, cq0 * sq1 + sq0 * cq1, 0,
                    0, 0, 1,
                    cq0 * sq1 + sq0 * cq1, -cq0 * cq1 + sq0 * sq1, 0
                };

                double[] r30 = Mul(r31, r10, 3, 3, 3);
                double[] r36 = Mul(r30, R06, 3, 3, 3);

                double cosqw;
                if (r36[8] >= 1.0 - 0.000001)
                {
                    cosqw = 1.0; qw[0, 1] = 0; qw[1, 1] = 0;
                }
                else if (r36[8] <= -1.0 + 0.000001)
                {
                    cosqw = -1.0;
                    if (indArm == 0) { qw[0, 1] = Math.PI; qw[1, 1] = -Math.PI; }
                    else { qw[0, 1] = -Math.PI; qw[1, 1] = Math.PI; }
                }
                else
                {
                    cosqw = r36[8];
                    if (indArm == 0)
                    {
                        qw[0, 1] = Math.Acos(cosqw); qw[1, 1] = -Math.Acos(cosqw);
                    }
                    else
                    {
                        qw[0, 1] = -Math.Acos(cosqw); qw[1, 1] = Math.Acos(cosqw);
                    }
                }

                if (cosqw == 1.0 || cosqw == -1.0)
                {
                    if (indArm == 0)
                    {
                        qw[0, 0] = last.A[3];
                        double cw = Math.Cos(last.A[3] + _dh[3, 0]);
                        double sw = Math.Sin(last.A[3] + _dh[3, 0]);
                        qw[0, 2] = Math.Atan2(cw * r36[3] - sw * r36[0], cw * r36[0] + sw * r36[3]);
                        qw[1, 2] = last.A[5];
                        cw = Math.Cos(last.A[5] + _dh[5, 0]);
                        sw = Math.Sin(last.A[5] + _dh[5, 0]);
                        qw[1, 0] = Math.Atan2(cw * r36[3] - sw * r36[0], cw * r36[0] + sw * r36[3]);
                    }
                    else
                    {
                        qw[0, 2] = last.A[5];
                        double cw = Math.Cos(last.A[5] + _dh[5, 0]);
                        double sw = Math.Sin(last.A[5] + _dh[5, 0]);
                        qw[0, 0] = Math.Atan2(cw * r36[3] - sw * r36[0], cw * r36[0] + sw * r36[3]);
                        qw[1, 0] = last.A[3];
                        cw = Math.Cos(last.A[3] + _dh[3, 0]);
                        sw = Math.Sin(last.A[3] + _dh[3, 0]);
                        qw[1, 2] = Math.Atan2(cw * r36[3] - sw * r36[0], cw * r36[0] + sw * r36[3]);
                    }
                    output.SolFlag[4 * indArm + 2 * indElbow, 2] = -1;
                    output.SolFlag[4 * indArm + 2 * indElbow + 1, 2] = -1;
                }
                else
                {
                    if (indArm == 0)
                    {
                        qw[0, 0] = Math.Atan2(r36[5], r36[2]);
                        qw[1, 0] = Math.Atan2(-r36[5], -r36[2]);
                        qw[0, 2] = Math.Atan2(r36[7], -r36[6]);
                        qw[1, 2] = Math.Atan2(-r36[7], r36[6]);
                    }
                    else
                    {
                        qw[0, 0] = Math.Atan2(-r36[5], -r36[2]);
                        qw[1, 0] = Math.Atan2(r36[5], r36[2]);
                        qw[0, 2] = Math.Atan2(-r36[7], r36[6]);
                        qw[1, 2] = Math.Atan2(r36[7], -r36[6]);
                    }
                    output.SolFlag[4 * indArm + 2 * indElbow, 2] = 1;
                    output.SolFlag[4 * indArm + 2 * indElbow + 1, 2] = 1;
                }

                for (int indWrist = 0; indWrist < 2; indWrist++)
                {
                    Joint6D cfg = output.Config[4 * indArm + 2 * indElbow + indWrist];
                    cfg.A[0] = WrapPi(qs[indArm]);
                    cfg.A[1] = WrapPi(qa[indElbow, 0]);
                    cfg.A[2] = WrapPi(qa[indElbow, 1]);
                    cfg.A[3] = WrapPi(qw[indWrist, 0]);
                    cfg.A[4] = WrapPi(qw[indWrist, 1]);
                    cfg.A[5] = WrapPi(qw[indWrist, 2]);
                }
            }
        }

        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 6; j++)
                output.Config[i].A[j] *= RadToDeg;

        return true;
    }

    private static double WrapPi(double value)
    {
        if (value > Math.PI) return value - Math.PI;
        if (value < -Math.PI) return value + Math.PI;
        return value;
    }

    private static double WrapDeg(double value)
    {
        while (value > 180) value -= 360;
        while (value < -180) value += 360;
        return value;
    }

    private static double[] Mul(double[] m1, double[] m2, int m, int l, int n)
    {
        var result = new double[m * n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
            {
                double tmp = 0;
                for (int k = 0; k < l; k++)
                    tmp += m1[l * i + k] * m2[n * k + j];
                result[n * i + j] = tmp;
            }
        return result;
    }

    private static void RotMatToEuler(double[] r, out double a, out double b, out double c)
    {
        if (Math.Abs(r[6]) >= 1.0 - 0.0001)
        {
            if (r[6] < 0)
            {
                a = 0; b = HalfPi; c = Math.Atan2(r[1], r[4]);
            }
            else
            {
                a = 0; b = -HalfPi; c = -Math.Atan2(r[1], r[4]);
            }
        }
        else
        {
            b = Math.Atan2(-r[6], Math.Sqrt(r[0] * r[0] + r[3] * r[3]));
            double cb = Math.Cos(b);
            a = Math.Atan2(r[3] / cb, r[0] / cb);
            c = Math.Atan2(r[7] / cb, r[8] / cb);
        }
    }

    private static void EulerToRotMat(double c, double b, double a, out double[] r)
    {
        double cc = Math.Cos(c), cb = Math.Cos(b), ca = Math.Cos(a);
        double sc = Math.Sin(c), sb = Math.Sin(b), sa = Math.Sin(a);
        r = new double[9];
        r[0] = ca * cb;  r[1] = ca * sb * sc - sa * cc; r[2] = ca * sb * cc + sa * sc;
        r[3] = sa * cb;  r[4] = sa * sb * sc + ca * cc; r[5] = sa * sb * cc - ca * sc;
        r[6] = -sb;      r[7] = cb * sc;                 r[8] = cb * cc;
    }
}
