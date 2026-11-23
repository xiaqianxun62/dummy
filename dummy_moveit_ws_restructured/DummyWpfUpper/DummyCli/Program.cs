using System.Globalization;
using DummyWpfUpper.Services;

if (args.Length == 0)
{
    PrintUsage();
    return 1;
}

string port = "COM3";
int baud = 115200;
var rest = new List<string>();
for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--port" when i + 1 < args.Length:
            port = args[++i];
            break;
        case "--baud" when i + 1 < args.Length:
            int.TryParse(args[++i], NumberStyles.Integer, CultureInfo.InvariantCulture, out baud);
            break;
        default:
            rest.Add(args[i]);
            break;
    }
}

if (rest.Count == 0)
{
    PrintUsage();
    return 1;
}

string command = rest[0].ToLowerInvariant();
string[] parameters = rest.Skip(1).ToArray();

if (command == "iktest")
    return RunIkTest(parameters);

if (command == "rrttest")
    return RunRrtTest(parameters);

if (command == "servotest")
    return RunServoTest(parameters);

using var client = new SerialAsciiClient(baud);
try
{
    client.Open(port);

    switch (command)
    {
        case "start":
            Console.WriteLine(client.SystemCommand("!START", "Started ok", 500));
            break;
        case "stop":
            Console.WriteLine(client.SystemCommand("!STOP", "Stopped ok", 500));
            break;
        case "home":
            Console.WriteLine(client.SystemCommand("!HOME", "Started ok", 15000));
            break;
        case "reset":
            Console.WriteLine(client.SystemCommand("!RESET", "Started ok", 15000));
            break;
        case "disable":
            Console.WriteLine(client.SystemCommand("!DISABLE", "Disabled ok", 500));
            break;
        case "set-home-current":
            Console.WriteLine(client.SystemCommand("!SET_HOME_CURRENT", "SET_HOME_CURRENT", 500));
            break;
        case "calibration":
            Console.WriteLine(client.SystemCommand("!CALIBRATION", "calibration ok", 20000));
            break;
        case "cmdmode":
            if (parameters.Length != 1)
                return Fail("用法: cmdmode <1|2|3|4>");
            Console.WriteLine(client.SetCommandMode(int.Parse(parameters[0], CultureInfo.InvariantCulture)));
            break;
        case "getjpos":
            Console.WriteLine("ok " + string.Join(" ", client.GetJointPositions().Select(v => v.ToString("F2", CultureInfo.InvariantCulture))));
            break;
        case "getlpos":
            Console.WriteLine("ok " + string.Join(" ", client.GetLinearPose().Select(v => v.ToString("F2", CultureInfo.InvariantCulture))));
            break;
        case "get-home-offset":
            Console.WriteLine("ok " + string.Join(" ", client.GetHomeOffset().Select(v => v.ToString("F2", CultureInfo.InvariantCulture))));
            break;
        case "get-base-pos":
            Console.WriteLine("ok " + string.Join(" ", client.GetBasePosition().Select(v => v.ToString("F2", CultureInfo.InvariantCulture))));
            break;
        case "sendj":
            if (parameters.Length is < 6 or > 7)
                return Fail("用法: sendj j1 j2 j3 j4 j5 j6 [speed]");
            Console.WriteLine(client.MoveJoint(ParseSix(parameters[..6]), parameters.Length == 7 ? ParseDouble(parameters[6]) : null));
            break;
        case "sendl":
            if (parameters.Length is < 6 or > 7)
                return Fail("用法: sendl x y z a b c [speed]");
            Console.WriteLine(client.MovePose(ParseSix(parameters[..6]), parameters.Length == 7 ? ParseDouble(parameters[6]) : null));
            break;
        case "dce-kp":
        case "dce-ki":
        case "dce-kd":
            if (parameters.Length != 2)
                return Fail($"用法: {command} <node> <value>");
            string parameter = command[4..].ToUpperInvariant();
            Console.WriteLine(client.SetMotorDce(int.Parse(parameters[0], CultureInfo.InvariantCulture), parameter,
                int.Parse(parameters[1], CultureInfo.InvariantCulture)));
            break;
        case "reboot":
            if (parameters.Length != 1)
                return Fail("用法: reboot <node>");
            Console.WriteLine(client.RebootMotor(int.Parse(parameters[0], CultureInfo.InvariantCulture)));
            break;
        default:
            PrintUsage();
            return 1;
    }

    return 0;
}
catch (Exception ex)
{
    Console.Error.WriteLine($"error: {ex.Message}");
    return 2;
}

static int RunIkTest(string[] parameters)
{
    double[] joints = parameters.Length == 6
        ? ParseSix(parameters)
        : new double[] { 0, -73, 180, 0, 0, 0 };

    var solver = Dof6Kinematics.CreateDefault();
    Pose6D fk = solver.SolveFkMm(joints);

    double[] min = { -170, -73, 35, -180, -120, -720 };
    double[] max = { 170, 90, 180, 180, 120, 720 };
    var solves = new IkSolves();
    solver.SolveIk(fk, new Joint6D(), out solves);

    Console.WriteLine($"原始关节 : {string.Join(", ", joints.Select(v => v.ToString("F3")))}");
    Console.WriteLine($"FK 位姿 : X={fk.X:F2} Y={fk.Y:F2} Z={fk.Z:F2} A={fk.A:F2} B={fk.B:F2} C={fk.C:F2}");
    Console.WriteLine("8 组 IK 解 (位置回环误差 / 是否在限位内):");

    for (int i = 0; i < 8; i++)
    {
        double[] q = solves.Config[i].A;
        Pose6D back = solver.SolveFkMm(q);
        double error = Math.Sqrt(Math.Pow(fk.X - back.X, 2) + Math.Pow(fk.Y - back.Y, 2) + Math.Pow(fk.Z - back.Z, 2));
        bool inLimit = true;
        for (int j = 0; j < 6; j++)
            if (q[j] < min[j] || q[j] > max[j]) { inLimit = false; break; }

        Console.WriteLine($"  [{i}] {string.Join(", ", q.Select(v => v.ToString("F2")))}  err={error:F4}mm  {(inLimit ? "OK" : "超限位")}");
    }

    Joint6D? best = solver.SolveIkBest(fk, joints, min, max);
    if (best is null)
    {
        Console.WriteLine("选解失败: 无满足限位的解");
        return 1;
    }

    Pose6D finalFk = solver.SolveFkMm(best.A);
    double posError = Math.Sqrt(Math.Pow(fk.X - finalFk.X, 2) + Math.Pow(fk.Y - finalFk.Y, 2) + Math.Pow(fk.Z - finalFk.Z, 2));
    double jointError = 0;
    for (int i = 0; i < 6; i++)
        jointError = Math.Max(jointError, Math.Abs(best.A[i] - joints[i]));

    Console.WriteLine($"选中解 : {string.Join(", ", best.A.Select(v => v.ToString("F3")))}");
    Console.WriteLine($"位置回环误差: {posError:F4} mm, 关节最大差: {jointError:F4} deg");
    return posError < 1.0 ? 0 : 1;
}

static int RunRrtTest(string[] parameters)
{
    double[] min = { -170, -73, 35, -180, -120, -720 };
    double[] max = { 170, 90, 180, 180, 120, 720 };
    double[] start = parameters.Length >= 6 ? ParseSix(parameters[..6]) : new double[] { 30, -50, 90, 20, -30, 45 };
    double[] goal = parameters.Length >= 12 ? ParseSix(parameters[6..12]) : new double[] { -60, 20, 120, -30, 40, -60 };

    var collision = new TrajectoryPlanner();

    string? lastCollision = null;
    bool IsValid(double[] joints)
    {
        for (int i = 0; i < 6; i++)
            if (joints[i] < min[i] || joints[i] > max[i]) { lastCollision = "超限位"; return false; }
        var hit = collision.CheckCollisionFwDeg(joints);
        lastCollision = hit?.Description;
        return hit is null;
    }

    var rrt = new RrtPlanner(min, max, IsValid);

    if (parameters.Length < 6)
    {
        double[][] candidates =
        {
            new double[] { 0, -73, 180, 0, 0, 0 },
            new double[] { 0, -30, 90, 0, -30, 0 },
            new double[] { 0, -45, 120, 0, 0, 0 },
            new double[] { 20, -40, 100, 0, 20, 0 },
            new double[] { 0, 0, 90, 0, 0, 0 }
        };
        start = candidates.FirstOrDefault(IsValid) ?? start;
        Console.WriteLine($"自动选择安全起点: {string.Join(", ", start.Select(v => v.ToString("F2")))}");
    }
    if (parameters.Length < 12)
    {
        double[][] candidates =
        {
            new double[] { -20, -30, 100, 0, 20, 0 },
            new double[] { 20, -50, 110, 0, -20, 0 },
            new double[] { 0, -60, 140, 0, 0, 0 },
            new double[] { -30, -40, 80, 0, 30, 0 }
        };
        goal = candidates.FirstOrDefault(IsValid) ?? goal;
        Console.WriteLine($"自动选择安全终点: {string.Join(", ", goal.Select(v => v.ToString("F2")))}");
    }

    Console.WriteLine($"起点: {string.Join(", ", start.Select(v => v.ToString("F2")))}");
    Console.WriteLine($"终点: {string.Join(", ", goal.Select(v => v.ToString("F2")))}");

    Console.WriteLine($"起点有效: {IsValid(start)} ({(lastCollision ?? "安全")})");
    Console.WriteLine($"终点有效: {IsValid(goal)} ({(lastCollision ?? "安全")})");
    RrtResult result = rrt.Plan(start, goal, maxIterations: 4000, stepDeg: 12);
    Console.WriteLine($"结果: {(result.Success ? "成功" : "失败")} - {result.Message}");
    if (!result.Success)
        return 1;

    Console.WriteLine($"路径点数: {result.Path.Count}, 迭代: {result.Iterations}");
    for (int i = 0; i < result.Path.Count; i++)
        Console.WriteLine($"  [{i}] {string.Join(", ", result.Path[i].Select(v => v.ToString("F2")))}");

    for (int i = 0; i < result.Path.Count - 1; i++)
    {
        if (!rrt.IsEdgeValid(result.Path[i], result.Path[i + 1]))
        {
            Console.WriteLine($"路径段 {i} 无效!");
            return 1;
        }
    }
    Console.WriteLine("所有路径段通过碰撞检查");
    return 0;
}

static int RunServoTest(string[] parameters)
{
    var kinematics = Dof6Kinematics.CreateDefault();
    var servo = new CartesianServo(kinematics);

    double[] joints = { 30, -50, 90, 20, -30, 45 };
    double[] twist = { 10, 0, 0, 0, 0, 0 };
    var frame = ServoFrame.Base;

    if (parameters.Length == 0)
    {
        // 使用默认姿态和默认 twist
    }
    else if (parameters.Length == 6)
    {
        joints = ParseSix(parameters);
    }
    else if (parameters.Length is 12 or 13)
    {
        joints = ParseSix(parameters[..6]);
        twist = ParseSix(parameters[6..12]);
        if (parameters.Length == 13 &&
            parameters[12].Equals("tool", StringComparison.OrdinalIgnoreCase))
            frame = ServoFrame.Tool;
    }
    else
    {
        return Fail("用法: servotest [j1 j2 j3 j4 j5 j6 vx vy vz wx wy wz [base|tool]]");
    }

    ServoResult result = servo.Compute(joints, twist, frame);
    double[] dq = result.JointVelocitiesDegPerSec;

    Console.WriteLine($"坐标系 : {frame}");
    Console.WriteLine($"关节角 : {string.Join(", ", joints.Select(v => v.ToString("F3")))}");
    Console.WriteLine($"指令 twist : {string.Join(", ", twist.Select(v => v.ToString("F3")))} (线速度 mm/s, 角速度 deg/s)");
    Console.WriteLine($"关节速度 : {string.Join(", ", dq.Select(v => v.ToString("F3")))} deg/s");
    Console.WriteLine($"基座 twist : {string.Join(", ", result.TwistBase.Select(v => v.ToString("F3")))}");

    // 用 FK 有限差分独立验证：dq 积分一小步后末端速度应接近目标 twist
    const double dt = 0.001;
    double[] jointsNext = joints.Select((v, i) => v + dq[i] * dt).ToArray();
    Pose6D fk0 = kinematics.SolveFkMm(joints);
    Pose6D fk1 = kinematics.SolveFkMm(jointsNext);

    double[] measuredBase =
    {
        (fk1.X - fk0.X) / dt,
        (fk1.Y - fk0.Y) / dt,
        (fk1.Z - fk0.Z) / dt,
        WrapDeg(fk1.A - fk0.A) / dt,
        WrapDeg(fk1.B - fk0.B) / dt,
        WrapDeg(fk1.C - fk0.C) / dt
    };

    double[] expected = frame == ServoFrame.Base ? result.TwistBase : twist;
    double[] measured = frame == ServoFrame.Base
        ? measuredBase
        : ToToolTwist(measuredBase, fk0.R);

    double maxLinearError = 0;
    double maxAngularError = 0;
    for (int i = 0; i < 6; i++)
    {
        double error = Math.Abs(measured[i] - expected[i]);
        if (i < 3) maxLinearError = Math.Max(maxLinearError, error);
        else maxAngularError = Math.Max(maxAngularError, error);
    }

    Console.WriteLine($"实测 twist : {string.Join(", ", measured.Select(v => v.ToString("F3")))}");
    Console.WriteLine($"最大误差   : 线速度 {maxLinearError:F4} mm/s, 角速度 {maxAngularError:F4} deg/s");

    if (!double.IsFinite(maxLinearError) || !double.IsFinite(maxAngularError))
    {
        Console.WriteLine("验证失败: 结果非有限值");
        return 1;
    }
    if (maxLinearError > 0.5 || maxAngularError > 0.5)
    {
        Console.WriteLine("验证失败: 末端速度偏差过大");
        return 1;
    }

    Console.WriteLine("验证通过: 关节速度经 FK 积分后末端速度与目标一致");
    return 0;
}

static double WrapDeg(double value)
{
    while (value > 180) value -= 360;
    while (value < -180) value += 360;
    return value;
}

/// <summary>把基座 twist 旋转回工具坐标系 (twist_tool = Rᵀ * twist_base)。</summary>
static double[] ToToolTwist(double[] twistBase, double[] rotation)
{
    var result = new double[6];
    for (int col = 0; col < 3; col++)
        for (int row = 0; row < 3; row++)
        {
            result[col] += rotation[row * 3 + col] * twistBase[row];
            result[3 + col] += rotation[row * 3 + col] * twistBase[3 + row];
        }
    return result;
}

static int Fail(string message)
{
    Console.Error.WriteLine(message);
    return 1;
}

static double[] ParseSix(string[] values)
{
    return values.Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray();
}

static double ParseDouble(string value)
{
    return double.Parse(value, CultureInfo.InvariantCulture);
}

static void PrintUsage()
{
    Console.WriteLine("用法: DummyCli [--port COMx] [--baud 115200] <command> [args...]");
    Console.WriteLine("命令: start stop home reset disable set-home-current calibration");
    Console.WriteLine("      cmdmode <mode> getjpos getlpos get-home-offset get-base-pos");
    Console.WriteLine("      sendj j1..j6 [speed]   sendl x,y,z,a,b,c [speed]");
    Console.WriteLine("      dce-kp|dce-ki|dce-kd <node> <value>   reboot <node>");
    Console.WriteLine("      iktest [j1 j2 j3 j4 j5 j6]  (离线 FK->IK->FK 回环验证)");
    Console.WriteLine("      rrttest [start6 goal6]     (离线 RRT 路径规划验证)");
    Console.WriteLine("      servotest [j1..j6 vx vy vz wx wy wz [base|tool]]  (离线 Servo 末端速度验证)");
}
