using System.Globalization;
using System.IO.Ports;
using System.Text;

namespace DummyWpfUpper.Services;

/// <summary>
/// 串口 ASCII 客户端。
/// </summary>
public sealed class SerialAsciiClient : IDisposable
{
    private readonly object _sync = new();
    private SerialPort? _port;

    private char _jointPrefix = '>';
    private int _defaultQueryTimeoutMs = 300;
    private int _defaultMotionAckTimeoutMs = 300;
    private int _defaultSystemTimeoutMs = 500;

    public int BaudRate { get; private set; } = 115200;

    public char JointPrefix => _jointPrefix;

    public bool IsOpen
    {
        get { lock (_sync) return _port?.IsOpen == true; }
    }

    public string PortName
    {
        get { lock (_sync) return _port?.PortName ?? string.Empty; }
    }

    public SerialAsciiClient(int baudRate = 115200)
    {
        BaudRate = baudRate;
    }

    public void Configure(int baudRate, char jointPrefix, int queryTimeoutMs, int motionAckTimeoutMs, int systemTimeoutMs)
    {
        BaudRate = baudRate > 0 ? baudRate : 115200;
        _jointPrefix = jointPrefix;
        _defaultQueryTimeoutMs = queryTimeoutMs > 0 ? queryTimeoutMs : 300;
        _defaultMotionAckTimeoutMs = motionAckTimeoutMs > 0 ? motionAckTimeoutMs : 300;
        _defaultSystemTimeoutMs = systemTimeoutMs > 0 ? systemTimeoutMs : 500;
    }

    public static string[] GetAvailablePorts() => SerialPort.GetPortNames();

    public void Open(string portName)
    {
        lock (_sync)
        {
            if (_port?.IsOpen == true)
                throw new InvalidOperationException("串口已打开");

            _port = new SerialPort(portName, BaudRate, Parity.None, 8, StopBits.One)
            {
                Handshake = Handshake.None,
                NewLine = "\r\n",
                Encoding = Encoding.ASCII,
                ReadTimeout = 200,
                WriteTimeout = 300
            };
            _port.Open();
            DrainLocked();
        }
    }

    public void Close()
    {
        lock (_sync)
        {
            if (_port is null)
                return;

            if (_port.IsOpen)
                _port.Close();
            _port.Dispose();
            _port = null;
        }
    }

    public string SendAndWait(string command, int timeoutMs, Func<string, bool>? accept = null)
    {
        lock (_sync)
        {
            SerialPort port = _port ?? throw new InvalidOperationException("串口未打开");
            DrainLocked();
            port.WriteLine(command);

            DateTime deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
            while (DateTime.UtcNow < deadline)
            {
                int remaining = (int)Math.Max(1, (deadline - DateTime.UtcNow).TotalMilliseconds);
                port.ReadTimeout = Math.Min(remaining, 200);

                string? line;
                try
                {
                    line = port.ReadLine();
                }
                catch (TimeoutException)
                {
                    continue;
                }
                catch (InvalidOperationException)
                {
                    throw new InvalidOperationException("串口在等待应答时被关闭");
                }

                if (string.IsNullOrWhiteSpace(line))
                    continue;

                line = line.Trim();
                if (accept is null || accept(line))
                    return line;
            }

            throw new TimeoutException($"等待串口应答超时: {command}");
        }
    }

    private string SendQuery(string command, int expectedValueCount, int? timeoutMs)
    {
        int timeout = timeoutMs ?? _defaultQueryTimeoutMs;
        bool IsValid(string line) => line.StartsWith("ok ", StringComparison.Ordinal)
                                     && ParseValues(line).Count == expectedValueCount;

        try
        {
            return SendAndWait(command, timeout, IsValid);
        }
        catch (TimeoutException)
        {
            return SendAndWait(command, timeout, IsValid);
        }
    }

    public double[] GetJointPositions(int? timeoutMs = null)
    {
        string line = SendQuery("#GETJPOS", 6, timeoutMs);
        return ParseValues(line).ToArray();
    }

    public double[] GetLinearPose(int? timeoutMs = null)
    {
        string line = SendQuery("#GETLPOS", 6, timeoutMs);
        return ParseValues(line).ToArray();
    }

    public double[] GetHomeOffset(int? timeoutMs = null)
    {
        string line = SendQuery("#GET_HOME_OFFSET", 6, timeoutMs);
        return ParseValues(line).ToArray();
    }

    public double[] GetBasePosition(int? timeoutMs = null)
    {
        string line = SendQuery("#GET_BASE_POS", 3, timeoutMs);
        return ParseValues(line).ToArray();
    }

    public string SetCommandMode(int mode, int? timeoutMs = null)
    {
        return SystemCommand($"#CMDMODE {mode}", "command mode", timeoutMs);
    }

    public string SystemCommand(string command, string expected, int? timeoutMs = null)
    {
        return SendAndWait(
            command,
            timeoutMs ?? _defaultSystemTimeoutMs,
            l => l.Contains(expected, StringComparison.OrdinalIgnoreCase));
    }

    public string SetMotorDce(int node, string parameter, int value, int? timeoutMs = null)
    {
        if (node < 1 || node > 6)
            throw new ArgumentException("电机节点必须是 1~6", nameof(node));
        return SystemCommand($"#SET_DCE_{parameter} {node} {value}", "DCE_" + parameter, timeoutMs);
    }

    public string RebootMotor(int node, int? timeoutMs = null)
    {
        if (node < 1 || node > 6)
            throw new ArgumentException("电机节点必须是 1~6", nameof(node));
        return SystemCommand($"#REBOOT {node}", "REBOOT MOTOR", timeoutMs);
    }

    public int MoveJoint(double[] jointsDeg, double? speedDeg = null, int? timeoutMs = null)
    {
        if (jointsDeg.Length != 6)
            throw new ArgumentException("关节目标必须是 6 个角度", nameof(jointsDeg));

        string body = string.Join(",", jointsDeg.Select(Fmt));
        if (speedDeg.HasValue)
            body += "," + Fmt(speedDeg.Value);

        string line = SendAndWait(
            _jointPrefix + body,
            timeoutMs ?? _defaultMotionAckTimeoutMs,
            l => int.TryParse(l, NumberStyles.Integer, CultureInfo.InvariantCulture, out _));

        return int.Parse(line, CultureInfo.InvariantCulture);
    }

    public int MovePose(double[] poseMmDeg, double? speed = null, int? timeoutMs = null)
    {
        if (poseMmDeg.Length != 6)
            throw new ArgumentException("笛卡尔目标必须是 6 个值", nameof(poseMmDeg));

        string body = string.Join(",", poseMmDeg.Select(Fmt));
        if (speed.HasValue)
            body += "," + Fmt(speed.Value);

        string line = SendAndWait(
            "@" + body,
            timeoutMs ?? _defaultMotionAckTimeoutMs,
            l => int.TryParse(l, NumberStyles.Integer, CultureInfo.InvariantCulture, out _));

        return int.Parse(line, CultureInfo.InvariantCulture);
    }

    private static string Fmt(double value) => value.ToString("F2", CultureInfo.InvariantCulture);

    private static List<double> ParseValues(string line)
    {
        var values = new List<double>();
        string[] parts = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
        foreach (string part in parts)
        {
            if (double.TryParse(part, NumberStyles.Float, CultureInfo.InvariantCulture, out double value))
                values.Add(value);
        }
        return values;
    }

    private void DrainLocked()
    {
        if (_port?.IsOpen == true)
            _port.DiscardInBuffer();
    }

    public void Dispose() => Close();
}
