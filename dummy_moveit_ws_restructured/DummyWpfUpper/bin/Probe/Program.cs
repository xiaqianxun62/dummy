using System.Reflection;
using DummyWpfUpper.Services;
using DummyWpfUpper.ViewModels;

MethodInfo parse = typeof(MainViewModel).GetMethod(
    "TryParseSix", BindingFlags.NonPublic | BindingFlags.Static)!;

object[] validArgs = { "10,-20,30,40,50,60", Array.Empty<double>() };
bool valid = (bool)parse.Invoke(null, validArgs)!;
Console.WriteLine($"valid={valid} values={string.Join(",", (double[])validArgs[1])}");

object[] invalidArgs = { "bad", Array.Empty<double>() };
bool invalid = (bool)parse.Invoke(null, invalidArgs)!;
Console.WriteLine($"invalidAccepted={invalid}");

bool implements = typeof(MainViewModel).GetInterfaces().Contains(typeof(IRobotTargetProvider));
Console.WriteLine($"implementsTargetProvider={implements}");
