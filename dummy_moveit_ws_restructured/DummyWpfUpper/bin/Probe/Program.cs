using DummyWpfUpper.Services;

double[] joints = { 10, -73, 180, 0, 0, 0 };
var planner = new TrajectoryPlanner();
Console.WriteLine(planner.CheckCollisionFwDeg(joints)?.Description ?? "默认参数：安全");
