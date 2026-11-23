using System.IO;
using System.Text.Json;

namespace DummyWpfUpper.Services;

public sealed class AppConfig
{
    public string SelectedPort { get; set; } = string.Empty;
    public string JointTargetInput { get; set; } = "0,-73,180,0,0,0";
    public string PoseTargetInput { get; set; } = "0,0,0,0,0,0";
    public string SpeedInput { get; set; } = "20";
    public string JogStepInput { get; set; } = "1";
    public string CommandModeInput { get; set; } = "2";
    public string CartesianJogStepInput { get; set; } = "10";
    public string InterpolationStepsInput { get; set; } = "10";
    public string PlayIntervalInput { get; set; } = "50";
    public bool UseTimeParameterization { get; set; }
    public bool EnableCollisionCheck { get; set; } = true;
    public string CollisionRadiusInput { get; set; } = "45,20,25,20,25,30,20";
    public string CollisionMarginInput { get; set; } = "0,0,0,0,0,0,0";
    public string CollisionGroundInput { get; set; } = "-20,-20,-20,-20,-20,-20,-20";
    public string MaxVelocityInput { get; set; } = "30";
    public string MaxAccelInput { get; set; } = "60";
    public string BaudRateInput { get; set; } = "115200";
    public string CommandPrefixInput { get; set; } = ">";
    public string QueryTimeoutInput { get; set; } = "300";
    public string MotionAckTimeoutInput { get; set; } = "300";
    public string SlowTimeoutInput { get; set; } = "15000";
    public string OffsetInput { get; set; } = "0,0,1.57079,0,0,0";
    public string SignInput { get; set; } = "1,1,1,1,-1,-1";
    public List<double[]> TaughtPoints { get; set; } = new();
    public Dictionary<string, double[]> NamedPoses { get; set; } = new();
    public bool AutoReconnect { get; set; } = true;
}

public static class PersistenceService
{
    private static readonly string DirectoryPath = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
        "DummyWpfUpper");

    private static readonly string FilePath = Path.Combine(DirectoryPath, "config.json");

    private static readonly JsonSerializerOptions Options = new()
    {
        WriteIndented = true
    };

    public static AppConfig Load()
    {
        try
        {
            if (!File.Exists(FilePath))
                return new AppConfig();

            AppConfig? config = JsonSerializer.Deserialize<AppConfig>(File.ReadAllText(FilePath), Options);
            return config ?? new AppConfig();
        }
        catch
        {
            return new AppConfig();
        }
    }

    public static void Save(AppConfig config)
    {
        try
        {
            System.IO.Directory.CreateDirectory(DirectoryPath);
            File.WriteAllText(FilePath, JsonSerializer.Serialize(config, Options));
        }
        catch
        {
            // 保存失败不应影响控制功能
        }
    }

    public static string ConfigPath => FilePath;
}
