using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using DummyWpfUpper.ViewModels;
using DummyWpfUpper.Views;

namespace DummyWpfUpper;

public partial class MainWindow : Window
{
    public MainWindow()
    {
        InitializeComponent();
        DataContext = new MainViewModel();
    }

    private MainViewModel Vm => (MainViewModel)DataContext;

    private void JointJogDown(object sender, MouseButtonEventArgs e)
    {
        if (sender is Button button && button.Tag is int index)
            Vm.StartJointJog(index, DirectionOf(button));
    }

    private void JointJogUp(object sender, MouseEventArgs e)
    {
        Vm.StopJog();
    }

    private void CartesianJogDown(object sender, MouseButtonEventArgs e)
    {
        if (sender is Button button && button.Tag is int axis)
            Vm.StartCartesianJog(axis, DirectionOf(button));
    }

    private void CartesianJogUp(object sender, MouseEventArgs e)
    {
        Vm.StopJog();
    }

    private static double DirectionOf(Button button)
    {
        return button.Content?.ToString() == "+" ? 1.0 : -1.0;
    }

    private void Open3DView_Click(object sender, RoutedEventArgs e)
    {
        new Robot3DWindow(Vm.Controller, Vm).Show();
    }
}
