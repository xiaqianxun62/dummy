using System.Diagnostics;

namespace DummyWpfUpper.Services;

public sealed record RrtResult(List<double[]> Path, int Iterations, bool Success, string Message)
{
    public static RrtResult Fail(string message) => new(new List<double[]>(), 0, false, message);
}

/// <summary>
/// 关节空间 RRT-Connect 双向路径规划器。
/// 状态为 6 维关节角(度)，边有效性由外部委托判定(限位+碰撞)。
/// </summary>
public sealed class RrtPlanner
{
    private sealed class Node
    {
        public required double[] Config { get; init; }
        public Node? Parent { get; set; }
    }

    private readonly double[] _min;
    private readonly double[] _max;
    private readonly Func<double[], bool> _isValid;
    private readonly Random _random = new();

    public RrtPlanner(double[] minLimits, double[] maxLimits, Func<double[], bool> isValid)
    {
        if (minLimits.Length != 6 || maxLimits.Length != 6)
            throw new ArgumentException("限位必须是 6 个值");
        _min = (double[])minLimits.Clone();
        _max = (double[])maxLimits.Clone();
        _isValid = isValid;
    }

    /// <summary>
    /// 规划从 start 到 goal 的避障路径，返回关节角序列(含起点和终点)。
    /// </summary>
    public RrtResult Plan(double[] start, double[] goal,
        int maxIterations = 4000, double stepDeg = 12, double goalBias = 0.15)
    {
        if (!InLimits(start) || !_isValid(start))
            return RrtResult.Fail("起点非法或与障碍物碰撞");
        if (!InLimits(goal) || !_isValid(goal))
            return RrtResult.Fail("终点非法或与障碍物碰撞");

        stepDeg = Math.Clamp(stepDeg, 1, 60);
        var treeA = new List<Node> { new() { Config = (double[])start.Clone() } };
        var treeB = new List<Node> { new() { Config = (double[])goal.Clone() } };

        var sw = Stopwatch.StartNew();
        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            if (sw.Elapsed.TotalSeconds > 10)
                return RrtResult.Fail($"规划超时 ({iteration} 次迭代)");

            double[] biasTarget = treeB[0].Config;
            double[] sample = Sample(biasTarget, goalBias);

            Node? newNode = Extend(treeA, sample, stepDeg);
            if (newNode is not null)
            {
                Node? joined = Connect(treeB, newNode.Config, stepDeg);
                if (joined is not null)
                {
                    List<double[]> raw = ExtractPath(newNode, joined);
                    List<double[]> smoothed = Smooth(raw, stepDeg);
                    return new RrtResult(smoothed, iteration, true, "找到路径");
                }
            }

            (treeA, treeB) = (treeB, treeA);
        }

        return RrtResult.Fail($"规划失败：{maxIterations} 次迭代内未找到路径");
    }

    /// <summary>
    /// 检查两点间直线是否可直接通过(无碰撞、不超限位)。
    /// </summary>
    public bool IsEdgeValid(double[] from, double[] to, double stepDeg = 12)
    {
        return EdgeValid(from, to, stepDeg);
    }

    private double[] Sample(double[] biasTarget, double goalBias)
    {
        if (_random.NextDouble() < goalBias)
            return (double[])biasTarget.Clone();

        var sample = new double[6];
        for (int i = 0; i < 6; i++)
            sample[i] = _min[i] + _random.NextDouble() * (_max[i] - _min[i]);
        return sample;
    }

    private Node? Extend(List<Node> tree, double[] target, double stepDeg)
    {
        Node nearest = Nearest(tree, target);
        double[] newConfig = Steer(nearest.Config, target, stepDeg);
        if (!EdgeValid(nearest.Config, newConfig, stepDeg))
            return null;

        var node = new Node { Config = newConfig, Parent = nearest };
        tree.Add(node);
        return node;
    }

    private Node? Connect(List<Node> tree, double[] target, double stepDeg)
    {
        while (true)
        {
            Node nearest = Nearest(tree, target);
            if (Distance(nearest.Config, target) < 1e-6)
                return nearest;

            double[] newConfig = Steer(nearest.Config, target, stepDeg);
            if (!EdgeValid(nearest.Config, newConfig, stepDeg))
                return null;

            tree.Add(new Node { Config = newConfig, Parent = nearest });
        }
    }

    private Node Nearest(List<Node> tree, double[] target)
    {
        Node best = tree[0];
        double bestDistance = Distance(best.Config, target);
        foreach (Node node in tree)
        {
            double d = Distance(node.Config, target);
            if (d < bestDistance)
            {
                bestDistance = d;
                best = node;
            }
        }
        return best;
    }

    private static double[] Steer(double[] from, double[] to, double stepDeg)
    {
        double distance = Distance(from, to);
        if (distance <= stepDeg)
            return (double[])to.Clone();

        double ratio = stepDeg / distance;
        var result = new double[6];
        for (int i = 0; i < 6; i++)
            result[i] = from[i] + (to[i] - from[i]) * ratio;
        return result;
    }

    private bool EdgeValid(double[] from, double[] to, double stepDeg)
    {
        double distance = Distance(from, to);
        int samples = Math.Max(1, (int)Math.Ceiling(distance / Math.Max(1.0, stepDeg / 2.0)));
        for (int s = 1; s <= samples; s++)
        {
            double t = s / (double)samples;
            var point = new double[6];
            for (int i = 0; i < 6; i++)
                point[i] = from[i] + (to[i] - from[i]) * t;

            if (!InLimits(point) || !_isValid(point))
                return false;
        }
        return true;
    }

    private List<double[]> ExtractPath(Node nodeA, Node nodeB)
    {
        var pathA = new List<double[]>();
        for (Node? node = nodeA; node is not null; node = node.Parent)
            pathA.Add((double[])node.Config.Clone());
        pathA.Reverse();

        var pathB = new List<double[]>();
        for (Node? node = nodeB; node is not null; node = node.Parent)
            pathB.Add((double[])node.Config.Clone());

        pathA.AddRange(pathB);
        return pathA;
    }

    private List<double[]> Smooth(List<double[]> path, double stepDeg)
    {
        if (path.Count <= 2)
            return path;

        var result = new List<double[]> { path[0] };
        int index = 0;
        while (index < path.Count - 1)
        {
            int next = index + 1;
            for (int candidate = path.Count - 1; candidate > index + 1; candidate--)
            {
                if (EdgeValid(path[index], path[candidate], stepDeg))
                {
                    next = candidate;
                    break;
                }
            }
            result.Add(path[next]);
            index = next;
        }
        return result;
    }

    private bool InLimits(double[] joints)
    {
        for (int i = 0; i < 6; i++)
        {
            if (joints[i] < _min[i] || joints[i] > _max[i])
                return false;
        }
        return true;
    }

    private static double Distance(double[] a, double[] b)
    {
        double sum = 0;
        for (int i = 0; i < 6; i++)
            sum += (a[i] - b[i]) * (a[i] - b[i]);
        return Math.Sqrt(sum);
    }
}
