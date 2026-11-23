using System.Reflection;
using DummyWpfUpper.Services;
using DummyWpfUpper.ViewModels;

MethodInfo parse = typeof(MainViewModel).GetMethod(
    "ParsePollHz", BindingFlags.NonPublic | BindingFlags.Static)!;

object Check(string input, int fallback, int min, int max) =>
    parse.Invoke(null, new object[] { input, fallback, min, max })!;

Console.WriteLine($"sim60={Check("60", 50, 1, 100)}");
Console.WriteLine($"sim200={Check("200", 50, 1, 100)}");
Console.WriteLine($"hwBad={Check("abc", 10, 1, 30)}");

var config = new AppConfig();
Console.WriteLine($"defaults={config.HardwarePollHzInput}/{config.SimulationPollHzInput}Hz");
