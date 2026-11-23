using System.IO;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using DummyWpfUpper.Services;

namespace DummyWpfUpper.Views;

public partial class Robot3DWindow : Window
{
    private static readonly string[] MeshFiles =
    {
        "base_link.stl",
        "link1_1_1.stl",
        "link2_1_1.stl",
        "link3_1_1.stl",
        "link4_1_1.stl",
        "link5_1_1.stl",
        "link6_1_1.stl"
    };

    private static readonly Vector3D[] JointOrigin =
    {
        new(-0.019225, -0.000523, 0.100684),
        new(-0.0155, 0.035, 0.0285),
        new(0.03665, 0, 0.146),
        new(-0.01855, -0.01, 0.052),
        new(0.0168, 0.127, 0),
        new(-0.018506, 0.077, -0.000106)
    };

    private static readonly Vector3D[] JointAxis =
    {
        new(0, 0, -1),
        new(-1, 0, 0),
        new(-1, 0, 0),
        new(0, -1, 0),
        new(-1, 0, 0),
        new(0, -1, 0)
    };

    private static readonly Vector3D[] LinkVisualOffset =
    {
        new(0, 0, 0),
        new(0.019225, 0.000523, -0.100684),
        new(0.034725, -0.034477, -0.129184),
        new(-0.001925, -0.034477, -0.275184),
        new(0.016625, -0.024477, -0.327184),
        new(-0.000175, -0.151477, -0.327184),
        new(0.018331, -0.228477, -0.327078)
    };

    private static readonly Color[] LinkColors =
    {
        Colors.Silver,
        Colors.LightSteelBlue,
        Colors.Silver,
        Colors.LightSteelBlue,
        Colors.Silver,
        Colors.LightSteelBlue,
        Colors.Gold
    };

    private readonly IRobotBackend _controller;
    private readonly IRobotPathProvider? _pathProvider;
    private readonly AxisAngleRotation3D[] _rotations = new AxisAngleRotation3D[6];
    private readonly PerspectiveCamera _camera;
    private readonly Dof6Kinematics _kinematics = Dof6Kinematics.CreateDefault();
    private readonly Model3DGroup _pathRoot = new();
    private readonly Model3DGroup _plannedPathGroup = new();
    private readonly Model3DGroup _taughtPathGroup = new();
    private readonly object _poseLock = new();
    private double[]? _latestJointsDeg;

    private readonly System.Windows.Threading.DispatcherTimer _poseTimer;
    private System.Windows.Threading.DispatcherTimer? _previewTimer;
    private List<double[]>? _previewPath;
    private int _previewIndex;
    private bool _previewing;
    private double[]? _poseBeforePreview;

    private Point _lastMouse;
    private bool _dragging;
    private double _yaw = -45;
    private double _pitch = 25;
    private double _distance = 1.15;
    private Vector3D _lookAt = new(0, 0, 0.10);

    public Robot3DWindow(IRobotBackend controller, IRobotPathProvider? pathProvider = null)
    {
        _controller = controller;
        _pathProvider = pathProvider;
        InitializeComponent();
        _camera = (PerspectiveCamera)Viewport.Camera;
        UpdateCamera();
        BuildScene();
        UpdatePose();
        _poseTimer = new System.Windows.Threading.DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(33)
        };
        _poseTimer.Tick += PoseTimer_Tick;
        _poseTimer.Start();
        _controller.StateUpdated += OnStateUpdated;
        if (_pathProvider is not null)
            _pathProvider.PathsChanged += OnPathsChanged;
        RefreshPaths();
        Closed += OnClosed;
    }

    private void BuildScene()
    {
        var root = new Model3DGroup();
        root.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 180));

        var baseGroup = new Model3DGroup();
        baseGroup.Children.Add(CreateLinkModel(0));
        root.Children.Add(baseGroup);

        Model3DGroup parent = baseGroup;
        for (int i = 0; i < 6; i++)
        {
            var transform = new Transform3DGroup();
            // URDF joint 语义：先绕关节轴旋转，再平移到关节原点 (T(J)*R)。
            // Transform3DGroup 按添加顺序应用，必须先放 Rotate 再放 Translate，
            // 否则关节原点会被一起旋转 (变成 R*(p+J))，导致连杆"散架"。
            _rotations[i] = new AxisAngleRotation3D(JointAxis[i], 0);
            transform.Children.Add(new RotateTransform3D(_rotations[i]));
            transform.Children.Add(new TranslateTransform3D(JointOrigin[i]));

            var linkGroup = new Model3DGroup { Transform = transform };
            linkGroup.Children.Add(CreateLinkModel(i + 1));
            parent.Children.Add(linkGroup);
            parent = linkGroup;
        }

        // 路径可视化层：与机器人共用 root 坐标系（已含 world_joint Rz180）
        _plannedPathGroup.Transform = Transform3D.Identity;
        _taughtPathGroup.Transform = Transform3D.Identity;
        _pathRoot.Children.Add(_plannedPathGroup);
        _pathRoot.Children.Add(_taughtPathGroup);
        root.Children.Add(_pathRoot);

        RobotRoot.Content = root;
    }

    private static Model3D CreateLinkModel(int index)
    {
        string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Meshes", MeshFiles[index]);
        MeshGeometry3D mesh = StlMeshLoader.Load(path);
        var material = new DiffuseMaterial(new SolidColorBrush(LinkColors[index]));
        var model = new GeometryModel3D(mesh, material)
        {
            Transform = new TranslateTransform3D(LinkVisualOffset[index])
        };
        return model;
    }

    private void UpdatePose()
    {
        ApplyJointAngles(_controller.State.JointsDeg);
    }

    private void PoseTimer_Tick(object? sender, EventArgs e)
    {
        if (_previewing)
            return;

        double[]? joints;
        lock (_poseLock)
        {
            joints = _latestJointsDeg;
            _latestJointsDeg = null;
        }

        if (joints is not null)
            ApplyJointAngles(joints);
    }

    private void ApplyJointAngles(double[] jointsDeg)
    {
        if (jointsDeg.Length != 6)
            return;

        double[] rosRad = _controller.FirmwareDegToRosRad(jointsDeg);
        for (int i = 0; i < 6; i++)
            _rotations[i].Angle = rosRad[i] * 180.0 / Math.PI;
    }

    private void OnStateUpdated(Models.RobotState state)
    {
        if (_previewing)
            return;

        lock (_poseLock)
            _latestJointsDeg = (double[])state.JointsDeg.Clone();
    }

    // ---------- 路径可视化 ----------

    private static readonly Vector3D ToolOffset =
        new(0.018331, -0.228477, -0.327078);

    /// <summary>
    /// 用与 3D 模型相同的 URDF 变换链计算末端工具点。
    /// 返回的是 root(Rz180) 之前的坐标，放入 _pathRoot 后会被 root 自动变换。
    /// </summary>
    private Point3D ComputeToolPoint(double[] jointsFwDeg)
    {
        double[] rosRad = _controller.FirmwareDegToRosRad(jointsFwDeg);
        Matrix3D frame = Matrix3D.Identity;
        for (int i = 0; i < 6; i++)
        {
            double angleDeg = rosRad[i] * 180.0 / Math.PI;
            var rotate = new RotateTransform3D(
                new AxisAngleRotation3D(JointAxis[i], angleDeg)).Value;
            var translate = new TranslateTransform3D(JointOrigin[i]).Value;
            frame = rotate * translate * frame;
        }
        return frame.Transform(new Point3D(ToolOffset.X, ToolOffset.Y, ToolOffset.Z));
    }

    private void OnPathsChanged()
    {
        Dispatcher.BeginInvoke(new Action(RefreshPaths));
    }

    private void RefreshPaths()
    {
        // XAML 初始化时 IsChecked=True 会提前触发 Checked，此时 PathInfoText 尚未生成。
        if (PathInfoText is null)
            return;

        _plannedPathGroup.Children.Clear();
        _taughtPathGroup.Children.Clear();
        if (ShowPathCheck.IsChecked != true)
        {
            PathInfoText.Text = "路径显示已关闭";
            return;
        }

        IReadOnlyList<double[]>? planned = _pathProvider?.PlannedPath;
        IReadOnlyList<double[]>? taught = _pathProvider?.TaughtPath;

        int count = 0;
        if (planned is { Count: >= 2 })
        {
            AddPathToGroup(InterpolateForDisplay(planned), _plannedPathGroup,
                Colors.OrangeRed, 0.0035, planned);
            count += planned.Count;
        }
        if (taught is { Count: >= 2 })
        {
            AddPathToGroup(InterpolateForDisplay(taught), _taughtPathGroup,
                Colors.Cyan, 0.0025, taught);
            count += taught.Count;
        }

        PathInfoText.Text = count > 0
            ? $"轨迹点 {count} 个（规划 {planned?.Count ?? 0} / 示教 {taught?.Count ?? 0}）"
            : "未加载路径";
    }

    private static IReadOnlyList<double[]> InterpolateForDisplay(IReadOnlyList<double[]> waypoints)
    {
        const int stepsPerSegment = 12;
        if (waypoints.Count < 2 || waypoints.Count > 200)
            return waypoints;

        var result = new List<double[]>();
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            List<double[]> segment = RobotController.InterpolateJointPath(
                waypoints[i], waypoints[i + 1], stepsPerSegment);
            int from = i == 0 ? 0 : 1;
            for (int s = from; s < segment.Count; s++)
                result.Add(segment[s]);
        }
        return result;
    }

    private void AddPathToGroup(IReadOnlyList<double[]> jointsPath,
        Model3DGroup group, Color color, double radius,
        IReadOnlyList<double[]>? markerWaypoints = null)
    {
        var points = new Point3D[jointsPath.Count];
        for (int i = 0; i < jointsPath.Count; i++)
            points[i] = ComputeToolPoint(jointsPath[i]);

        var tubeMaterial = new DiffuseMaterial(new SolidColorBrush(color));

        // 相邻点之间画细管；原始路径点画八面体标记
        for (int i = 0; i < points.Length - 1; i++)
        {
            Model3D? segment = CreateTube(points[i], points[i + 1], radius, tubeMaterial);
            if (segment is not null)
                group.Children.Add(segment);
        }

        var markers = markerWaypoints ?? jointsPath;
        var markerPoints = new Point3D[markers.Count];
        for (int i = 0; i < markers.Count; i++)
            markerPoints[i] = ComputeToolPoint(markers[i]);

        var startMaterial = new MaterialGroup();
        startMaterial.Children.Add(new DiffuseMaterial(new SolidColorBrush(Colors.LimeGreen)));
        startMaterial.Children.Add(new EmissiveMaterial(new SolidColorBrush(Colors.LimeGreen)));

        var endMaterial = new MaterialGroup();
        endMaterial.Children.Add(new DiffuseMaterial(new SolidColorBrush(Colors.Crimson)));
        endMaterial.Children.Add(new EmissiveMaterial(new SolidColorBrush(Colors.Crimson)));

        var midMaterial = new DiffuseMaterial(new SolidColorBrush(color));

        for (int i = 0; i < markerPoints.Length; i++)
        {
            bool isEndpoint = i == 0 || i == markerPoints.Length - 1;
            Material markerMat = i == 0 ? startMaterial
                : i == markerPoints.Length - 1 ? endMaterial : midMaterial;
            group.Children.Add(CreateOctahedron(markerPoints[i],
                isEndpoint ? radius * 6 : radius * 1.6, markerMat));
        }
    }

    private static Model3D? CreateTube(Point3D a, Point3D b, double radius, Material material)
    {
        Vector3D axis = b - a;
        double length = axis.Length;
        if (length < 1e-6)
            return null;
        axis.Normalize();
        Vector3D helper = Math.Abs(axis.Z) < 0.9
            ? new Vector3D(0, 0, 1) : new Vector3D(1, 0, 0);
        Vector3D u = Vector3D.CrossProduct(axis, helper);
        u.Normalize();
        Vector3D v = Vector3D.CrossProduct(axis, u);
        v.Normalize();

        const int sides = 6;
        var positions = new Point3DCollection();
        var indices = new Int32Collection();
        for (int i = 0; i < sides; i++)
        {
            double angle = 2 * Math.PI * i / sides;
            Vector3D offset = (u * Math.Cos(angle) + v * Math.Sin(angle)) * radius;
            positions.Add(a + offset);
            positions.Add(b + offset);
        }
        for (int i = 0; i < sides; i++)
        {
            int next = (i + 1) % sides;
            int base0 = i * 2;
            int base1 = next * 2;
            indices.Add(base0); indices.Add(base1); indices.Add(base0 + 1);
            indices.Add(base1); indices.Add(base1 + 1); indices.Add(base0 + 1);
        }
        var mesh = new MeshGeometry3D { Positions = positions, TriangleIndices = indices };
        return new GeometryModel3D(mesh, material) { BackMaterial = material };
    }

    private static Model3D CreateOctahedron(Point3D center, double r, Material material)
    {
        var positions = new Point3DCollection
        {
            center + new Vector3D(r, 0, 0), center + new Vector3D(-r, 0, 0),
            center + new Vector3D(0, r, 0), center + new Vector3D(0, -r, 0),
            center + new Vector3D(0, 0, r), center + new Vector3D(0, 0, -r)
        };
        var indices = new Int32Collection
        {
            0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4,
            2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5
        };
        var mesh = new MeshGeometry3D { Positions = positions, TriangleIndices = indices };
        return new GeometryModel3D(mesh, material) { BackMaterial = material };
    }

    // ---------- 轨迹预览 ----------

    private void ShowPathCheck_Changed(object sender, RoutedEventArgs e)
    {
        RefreshPaths();
    }

    private void PreviewButton_Click(object sender, RoutedEventArgs e)
    {
        if (_previewing)
        {
            StopPreview();
            return;
        }

        IReadOnlyList<double[]>? path = _pathProvider?.PlannedPath;
        if (path is null || path.Count < 2)
            path = _pathProvider?.TaughtPath;
        if (path is null || path.Count < 2)
        {
            PathInfoText.Text = "请先规划或示教至少 2 个点";
            return;
        }

        _previewPath = path.Select(p => (double[])p.Clone()).ToList();
        _poseBeforePreview = (double[])_controller.State.JointsDeg.Clone();
        _previewIndex = 0;
        _previewing = true;
        lock (_poseLock)
            _latestJointsDeg = null;
        PreviewButton.Content = "停止预览";
        ApplyPreviewPose(_previewPath[0]);

        _previewTimer = new System.Windows.Threading.DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(80)
        };
        _previewTimer.Tick += PreviewTimer_Tick;
        _previewTimer.Start();
    }

    private void PreviewTimer_Tick(object? sender, EventArgs e)
    {
        if (_previewPath is null)
            return;
        _previewIndex++;
        if (_previewIndex >= _previewPath.Count)
        {
            StopPreview();
            return;
        }
        ApplyPreviewPose(_previewPath[_previewIndex]);
        PathInfoText.Text = $"预览 {_previewIndex + 1}/{_previewPath.Count}";
    }

    private void ApplyPreviewPose(double[] jointsFwDeg)
    {
        double[] rosRad = _controller.FirmwareDegToRosRad(jointsFwDeg);
        for (int i = 0; i < 6; i++)
            _rotations[i].Angle = rosRad[i] * 180.0 / Math.PI;
    }

    private void StopPreview()
    {
        _previewTimer?.Stop();
        _previewTimer = null;
        _previewing = false;
        PreviewButton.Content = "轨迹预览";
        if (_poseBeforePreview is not null)
            ApplyPreviewPose(_poseBeforePreview);
        _previewPath = null;
        UpdatePose();
    }

    private void Viewport_MouseDown(object sender, MouseButtonEventArgs e)
    {
        _lastMouse = e.GetPosition(Viewport);
        _dragging = true;
        Viewport.CaptureMouse();
    }

    private void Viewport_MouseUp(object sender, MouseButtonEventArgs e)
    {
        _dragging = false;
        Viewport.ReleaseMouseCapture();
    }

    private void Viewport_MouseMove(object sender, MouseEventArgs e)
    {
        if (!_dragging)
            return;

        Point current = e.GetPosition(Viewport);
        double dx = current.X - _lastMouse.X;
        double dy = current.Y - _lastMouse.Y;
        _lastMouse = current;

        if (e.LeftButton == MouseButtonState.Pressed)
        {
            _yaw += dx * 0.5;
            _pitch = Math.Clamp(_pitch - dy * 0.5, -85, 85);
        }
        else if (e.RightButton == MouseButtonState.Pressed)
        {
            double scale = _distance * 0.0015;
            Vector3D right = Vector3D.CrossProduct(_camera.LookDirection, _camera.UpDirection);
            right.Normalize();
            Vector3D up = Vector3D.CrossProduct(right, _camera.LookDirection);
            up.Normalize();
            _lookAt -= right * dx * scale;
            _lookAt += up * dy * scale;
        }

        UpdateCamera();
    }

    private void Viewport_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        _distance = Math.Clamp(_distance * (e.Delta > 0 ? 0.9 : 1.1), 0.2, 5);
        UpdateCamera();
    }

    private void UpdateCamera()
    {
        double yaw = _yaw * Math.PI / 180.0;
        double pitch = _pitch * Math.PI / 180.0;
        double cosPitch = Math.Cos(pitch);
        var direction = new Vector3D(
            -Math.Cos(yaw) * cosPitch,
            -Math.Sin(yaw) * cosPitch,
            -Math.Sin(pitch));

        _camera.LookDirection = direction;
        _camera.Position = new Point3D(
            _lookAt.X - direction.X * _distance,
            _lookAt.Y - direction.Y * _distance,
            _lookAt.Z - direction.Z * _distance);
        _camera.UpDirection = new Vector3D(0, 0, 1);
    }

    private void OnClosed(object? sender, EventArgs e)
    {
        if (_previewing)
            StopPreview();
        _poseTimer.Stop();
        _controller.StateUpdated -= OnStateUpdated;
        if (_pathProvider is not null)
            _pathProvider.PathsChanged -= OnPathsChanged;
    }
}
