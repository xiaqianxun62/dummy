namespace DummyWpfUpper.Services;

public enum ServoFrame
{
    Base,
    Tool
}

public sealed record ServoResult(double[] JointVelocitiesDegPerSec, double[] TwistBase);

/// <summary>
/// 笛卡尔速度伺服：基于数值雅可比的阻尼最小二乘伪逆，
/// 把末端 twist (线速度 mm/s + 角速度 deg/s) 映射到关节速度。
/// </summary>
public sealed class CartesianServo
{
    private readonly Dof6Kinematics _kinematics;
    private readonly double _damping;

    public CartesianServo(Dof6Kinematics kinematics, double damping = 0.05)
    {
        _kinematics = kinematics;
        _damping = Math.Max(0.001, damping);
    }

    /// <summary>
    /// 计算给定 twist 对应的关节速度。
    /// </summary>
    /// <param name="jointsDeg">当前关节角(度)</param>
    /// <param name="twist">[vx,vy,vz,wx,wy,wz]，线速度 mm/s，角速度 deg/s</param>
    /// <param name="frame">twist 所在坐标系：基座或工具</param>
    public ServoResult Compute(double[] jointsDeg, double[] twist, ServoFrame frame)
    {
        if (jointsDeg.Length != 6 || twist.Length != 6)
            throw new ArgumentException("关节和 twist 都必须是 6 个值");

        double[] twistBase = frame == ServoFrame.Tool
            ? ToBaseTwist(twist, _kinematics.SolveFkMm(jointsDeg).R)
            : (double[])twist.Clone();

        double[,] jacobian = _kinematics.ComputeJacobian(jointsDeg);
        double[] jointVel = DampedLeastSquares(jacobian, twistBase);
        return new ServoResult(jointVel, twistBase);
    }

    /// <summary>
    /// 工具坐标系 twist 旋转到基座坐标系。
    /// R 为行主序 3x3 旋转矩阵（FK 输出）。
    /// </summary>
    private static double[] ToBaseTwist(double[] twist, double[] rotation)
    {
        var result = new double[6];
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                result[row] += rotation[row * 3 + col] * twist[col];
                result[3 + row] += rotation[row * 3 + col] * twist[3 + col];
            }
        }
        return result;
    }

    /// <summary>
    /// 关节空间阻尼最小二乘：dq = (JᵀJ + λ²I)⁻¹ Jᵀ v
    /// 与任务空间形式 Jᵀ(JJᵀ + λ²I)⁻¹ v 等价，且矩阵维数匹配关节速度。
    /// </summary>
    private double[] DampedLeastSquares(double[,] j, double[] v)
    {
        var a = new double[6, 6];
        var b = new double[6];
        double lambda2 = _damping * _damping;

        for (int i = 0; i < 6; i++)
        {
            for (int k = 0; k < 6; k++)
            {
                double sum = 0;
                for (int row = 0; row < 6; row++)
                    sum += j[row, i] * j[row, k];
                a[i, k] = sum + (i == k ? lambda2 : 0);
            }

            double rhs = 0;
            for (int row = 0; row < 6; row++)
                rhs += j[row, i] * v[row];
            b[i] = rhs;
        }

        return SolveLinearSystem(a, b);
    }

    /// <summary>
    /// 高斯-约当消元求解 Ax=b（带部分主元）。
    /// </summary>
    private static double[] SolveLinearSystem(double[,] a, double[] b)
    {
        int n = b.Length;
        var matrix = new double[n, n + 1];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
                matrix[i, j] = a[i, j];
            matrix[i, n] = b[i];
        }

        for (int col = 0; col < n; col++)
        {
            int pivot = col;
            for (int row = col + 1; row < n; row++)
                if (Math.Abs(matrix[row, col]) > Math.Abs(matrix[pivot, col]))
                    pivot = row;

            if (Math.Abs(matrix[pivot, col]) < 1e-12)
                return new double[n];

            if (pivot != col)
                for (int j = 0; j <= n; j++)
                    (matrix[col, j], matrix[pivot, j]) = (matrix[pivot, j], matrix[col, j]);

            double div = matrix[col, col];
            for (int j = col; j <= n; j++)
                matrix[col, j] /= div;

            for (int row = 0; row < n; row++)
            {
                if (row == col) continue;
                double factor = matrix[row, col];
                if (Math.Abs(factor) < 1e-15) continue;
                for (int j = col; j <= n; j++)
                    matrix[row, j] -= factor * matrix[col, j];
            }
        }

        var solution = new double[n];
        for (int i = 0; i < n; i++)
            solution[i] = matrix[i, n];
        return solution;
    }
}
