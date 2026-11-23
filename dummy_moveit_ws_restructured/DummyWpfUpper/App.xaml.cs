using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Windows;
using System.Windows.Interop;
using System.Windows.Media;
using System.Windows.Threading;
using DummyWpfUpper.Services;
using DummyWpfUpper.Views;

namespace DummyWpfUpper;

public partial class App : Application
{
    private const string ProbeArgument = "--probe-3d";
    private static readonly TimeSpan ProbeTimeout = TimeSpan.FromSeconds(6);

    private readonly bool _isProbeProcess;

    public App()
    {
        _isProbeProcess = Environment.GetCommandLineArgs()
            .Skip(1)
            .Contains(ProbeArgument, StringComparer.OrdinalIgnoreCase);

        if (_isProbeProcess)
            return;

        if (!ProbeCanUseHardwareRendering())
            RenderOptions.ProcessRenderMode = RenderMode.SoftwareOnly;
    }

    protected override void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        if (_isProbeProcess)
        {
            RunProbe();
            return;
        }

        var mainWindow = new MainWindow();
        MainWindow = mainWindow;
        mainWindow.Show();
    }

    private static bool ProbeCanUseHardwareRendering()
    {
        try
        {
            string executable = Environment.ProcessPath
                ?? Assembly.GetEntryAssembly()?.Location
                ?? throw new InvalidOperationException("无法定位可执行文件路径");

            string arguments = ProbeArgument;
            if (Path.GetFileNameWithoutExtension(executable)
                .Equals("dotnet", StringComparison.OrdinalIgnoreCase))
            {
                string assembly = Assembly.GetEntryAssembly()?.Location
                    ?? throw new InvalidOperationException("无法定位程序集路径");
                arguments = $"\"{assembly}\" {ProbeArgument}";
            }

            var startInfo = new ProcessStartInfo(executable, arguments)
            {
                UseShellExecute = false,
                WorkingDirectory = AppDomain.CurrentDomain.BaseDirectory
            };

            using Process? probe = Process.Start(startInfo);
            if (probe is null)
                return false;

            if (!probe.WaitForExit((int)ProbeTimeout.TotalMilliseconds))
            {
                probe.Kill(entireProcessTree: true);
                return false;
            }

            return probe.ExitCode == 0;
        }
        catch
        {
            return false;
        }
    }

    private void RunProbe()
    {
        try
        {
            var simulation = new SimulationController();
            var window = new Robot3DWindow(simulation)
            {
                WindowStyle = WindowStyle.None,
                ShowInTaskbar = false,
                ShowActivated = false,
                Left = 0,
                Top = 0
            };

            int renderedFrames = 0;
            var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(80) };
            timer.Tick += (_, _) =>
            {
                if (++renderedFrames < 3)
                    return;
                timer.Stop();
                window.Close();
                Shutdown(0);
            };

            window.ContentRendered += (_, _) => timer.Start();
            window.Show();
        }
        catch
        {
            Shutdown(1);
        }
    }
}
