using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.IO.Ports;
using WpfApplication1.Properties;
using MahApps.Metro.Controls.Dialogs;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using OxyPlot;
using OxyPlot.Axes;

namespace WpfApplication1
{


    public class Destination
    {
        public DateTime timeStarted { get; set; }

        public double KP { get; set; }
        public double KI { get; set; }
        public double KD { get; set; }

        public double error { get; set; }
        public double target { get; set; }
    }

    public static class PositionGraph
    {
        public static void AddPoint(PlotModel plot, double position, double setpoint)
        {
            DataPoint positionPoint = new DataPoint(DateTimeAxis.ToDouble(DateTime.Now), position);
            DataPoint setpointPoint = new DataPoint(DateTimeAxis.ToDouble(DateTime.Now), setpoint);

            (plot.Series[0] as OxyPlot.Series.LineSeries).Points.Add(positionPoint);
            (plot.Series[0] as OxyPlot.Series.LineSeries).Points.Add(setpointPoint);

            plot.InvalidatePlot(true);
        }

        public static PlotModel Setup(string title)
        {
            PlotModel plot = new PlotModel();
            plot.Title = title;
            plot.TitleColor = OxyColors.Gray;
            plot.Series.Add(new OxyPlot.Series.LineSeries()); // for current position
            plot.Series.Add(new OxyPlot.Series.LineSeries()); // for setpoint

            DateTimeAxis xAxis = new DateTimeAxis
            {
                Position = AxisPosition.Bottom,
                StringFormat = "hh-ss-mm",
                Title = "Time",
                AxislineColor = OxyColors.Gray,
                TextColor = OxyColors.LightGray,
                TicklineColor = OxyColors.Gray,
                MinorGridlineColor = OxyColors.Gray,
                MajorGridlineColor = OxyColors.Gray,
                ExtraGridlineColor = OxyColors.Gray,
                IntervalType = DateTimeIntervalType.Months,
                MajorGridlineStyle = LineStyle.Solid,
                TitleColor = OxyColors.Gray
            };

            LinearAxis linearAxis = new LinearAxis
            {
                Position = AxisPosition.Left,
                Title = "Position",
                MajorGridlineStyle = LineStyle.Solid,
                AxislineColor = OxyColors.Gray,
                TextColor = OxyColors.LightGray,
                TicklineColor = OxyColors.Gray,
                MinorGridlineColor = OxyColors.Gray,
                MajorGridlineColor = OxyColors.Gray,
                ExtraGridlineColor = OxyColors.Gray,
                TitleColor = OxyColors.Gray
            };

            plot.Axes.Add(linearAxis);
            plot.Axes.Add(xAxis);
            return plot;
        }
    }


    public class Setpoint
    {
        public string identifier { get; set; }
        public SerialPort port { get; set; }
        public void Send(double value)
        {
            port.WriteLine($"{identifier} {value}");
        }
        public Task<Destination> NewDestination(double setpoint, int seconds)
        {
            Send(setpoint);

            double runningError = 0;
            bool timedOut = false;
            DateTime startTime = DateTime.Now;

            while (!timedOut)
            {
                timedOut = (DateTime.Now - startTime > TimeSpan.FromSeconds(seconds));
            }

            Destination t = new Destination
            {
                timeStarted = startTime,
                error = runningError,
                target = setpoint
            };

            return Task.FromResult(t);
        }
    }

    public class Gains
    {
        public List<string> identifiers { get; set; }
        public SerialPort port { get; set; }
        public void Send(double kp, double ki, double kd)
        {
            port.WriteLine($"{identifiers[0]} {kp}");
            port.WriteLine($"{identifiers[1]} {ki}");
            port.WriteLine($"{identifiers[2]} {kd}");
        }
    }

    public class DegreeOfFreedom : INotifyPropertyChanged
    {
        public SerialPort serialPort { get; set; }
        public Setpoint setpoint = new Setpoint();
        public Gains gains = new Gains();
        private double _currentPosition;

        public double currentPosition
        {
            get { return _currentPosition; }
            set
            {
                if (_currentPosition != value)
                {
                    _currentPosition = value;
                    OnPropertyChanged();
                }
            }
        }


        private double _runningError;
        public double runningError
        {
            get { return _runningError; }
            set
            {
                if (_runningError != value)
                {
                    _runningError = value;
                    OnPropertyChanged();
                }
            }
        }

        private double _currentError;
        public double currentError
        {
            get { return _currentError; }
            set
            {
                if (_currentError != value)
                {
                    _currentError = value;
                    OnPropertyChanged();
                }
            }
        }

        private double _currentOutput;
        public double currentOutput
        {
            get { return _currentOutput; }
            set
            {
                if (_currentOutput != value)
                {
                    _currentOutput = value;
                    OnPropertyChanged();
                }
            }
        }


        private double _currentSetpoint;
        public double currentSetpoint
        {
            get { return _currentSetpoint; }
            set
            {
                if (_currentSetpoint != value)
                {
                    _currentSetpoint = value;
                    OnPropertyChanged();
                }
            }
        }


        private List<Destination> _trials;
        public List<Destination> trials
        {
            get { return _trials; }
            set
            {
                if (_trials != value)
                {
                    _trials = value;
                    OnPropertyChanged();
                }
            }
        }

        private PlotModel _plot;
        public PlotModel Plot
        {
            get { return _plot; }
            set
            {
                if (_plot != value)
                {
                    _plot = value;
                    OnPropertyChanged();
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }


    }


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : INotifyPropertyChanged
    {


        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }

        bool abortTrials = false;
        Settings settings = Settings.Default;
        SerialPort port = new SerialPort(Settings.Default.ComPort, Settings.Default.BaudRate, Parity.None, 8, StopBits.One);


        private DegreeOfFreedom _yaw;
        public DegreeOfFreedom Yaw
        {
            get { return _yaw; }
            set
            {
                if(_yaw != value)
                {
                    _yaw = value;
                    OnPropertyChanged();
                }
            }
        }


        private DegreeOfFreedom _lift;
        public DegreeOfFreedom Lift
        {
            get { return _lift; }
            set
            {
                if (_lift != value)
                {
                    _lift = value;
                    OnPropertyChanged();
                }
            }
        }

        public MainWindow()
        {
            Yaw = new DegreeOfFreedom();
            Lift = new DegreeOfFreedom();

            Yaw.trials = new List<Destination>();
            InitializeComponent();
            port.DataReceived += new SerialDataReceivedEventHandler(serialDataRecieved);

            Yaw.serialPort = port;
            Yaw.setpoint.identifier = settings.YawSetpointIdentifier;
            Yaw.gains.identifiers = new List<string> { "-p", "-i", "-d" };

            Lift.serialPort = port;
            Lift.setpoint.identifier = settings.LiftSetpointIdentifier;
            Lift.gains.identifiers = new List<string> { "-P", "-I", "-D" };

            Yaw.Plot = PositionGraph.Setup("Yaw Position");
            Lift.Plot = PositionGraph.Setup("Lift Position");
        }


        private void serialDataRecieved(object sender, SerialDataReceivedEventArgs e)
        {
            if (!port.IsOpen) return;

            string data = String.Empty;
            try
            {
                data = port.ReadLine();
            }
            catch
            {
                Alert_Async("DAMMIT", "Problem reading data.");
                return;
            }

            List<string> serialIn = data.Split(' ').ToList();
            if (serialIn.Count() < 3) return; // sometimes we get incomplete data, ignore that

            double pos, err, output;
            if (double.TryParse(serialIn[0], out pos))
            {
                Yaw.currentPosition = pos;
                PositionGraph.AddPoint(Yaw.Plot, pos, Yaw.currentSetpoint);
            }
            if (double.TryParse(serialIn[1], out err))
            {
                Yaw.currentError = err;
                Yaw.runningError += Math.Abs(err);
            }

            if (double.TryParse(serialIn[2], out output))
            {
                Yaw.currentOutput = output;
            }
            
       }

        #region EventHandler

        private void ApplyYawSettings_Click(object sender, RoutedEventArgs e)
        {
            if (port.IsOpen)
            {
                Yaw.gains.Send(settings.YawKP, settings.YawKI, settings.YawKD);
            }
            else
            {
                Alert_Async("NOT YET", "You can't do that yet because the port isn't open.");
            }
        }

        private void ApplyLiftSettings_Click(object sender, RoutedEventArgs e)
        {
            if (port.IsOpen)
            {
                Lift.gains.Send(settings.LiftKP, settings.LiftKI, settings.LiftKD);
            }
            else
            {
                Alert_Async("NOT YET", "You can't do that yet because the port isn't open.");
            }
        }

        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            port.Close();
            settings.Save();
            base.OnClosing(e);
        }

        private async void TrialBeginButton_Click(object sender, RoutedEventArgs e)
        {
            abortTrials = false;
            Yaw.currentSetpoint = Yaw.currentPosition;

            int inverter = 1;
            for (double kp = settings.KPFrom; kp <= settings.KPTo; kp = kp + settings.KPBy)
            {
                for (double ki = settings.KIFrom; ki <= settings.KITo; ki = ki + settings.KIBy)
                {
                    for (double kd = settings.KDFrom; kd <= settings.KDTo; kd = kd + settings.KDBy)
                    {
                        if (abortTrials)
                        {
                            Alert_Async("ABORTED", "Trials ended prematurely.");
                            return;
                        }

                        if (settings.InvertOffset)
                            inverter *= -1; // if the offset inverter is on, then setpoint should be -offset then +offset then -offset etc.

                        Yaw.currentSetpoint = Yaw.currentPosition + settings.SetpointOffset * inverter;

                        settings.YawKP = kp;
                        settings.YawKI = ki;
                        settings.YawKD = kd;


                        Destination newTrial = await Task.Run(() =>
                        {
                            return Yaw.setpoint.NewDestination(Yaw.currentSetpoint, settings.TimeoutSeconds);
                        });

                        newTrial.KP = kp;
                        newTrial.KI = ki;
                        newTrial.KD = kd;

                        Yaw.trials.Add(newTrial);
                        TrialsDataGrid.Items.Refresh();
                    }
                }
            }
        }

        private void TrialEndButton_Click(object sender, RoutedEventArgs e)
        {
            abortTrials = true;
            Alert_Async("WILL DO", "The trials will end after the most recently started trial finishes.");
        }
        

        private void ManualSave_Click(object sender, RoutedEventArgs e)
        {
                            
            settings.Save();
            Alert_Async("YOU GOT IT", 
                        @"Your settings are saved. This'll happen automatically on close but it's handy to save manually in case the program crashes."
                            + Environment.NewLine + Environment.NewLine + "Not that it would do that...");
        }

        private void OpenConnection_Click(object sender, RoutedEventArgs e)
        {
            if (!port.IsOpen)
            {
                try {
                    port.Open();
                }
                catch (Exception ex)
                {
                    Alert_Async("WHOOPS", "Sorry, couldn't do that because: " + ex.Message);
                    return;
                }
                var button = sender as Button;
                button.IsEnabled = false;
                button.Content = "Connected";
            }
            else
            {
                Alert_Async("NO NEED", "The port's already open");   
            }
        }
        
        private void RoutineOne_Click(object sender, RoutedEventArgs e)
        {
            Lift.setpoint.NewDestination(100, 30);
            Yaw.setpoint.NewDestination(50, 30);
            Lift.setpoint.NewDestination(0, 30);
        }

        private void RoutineTwo_Click(object sender, RoutedEventArgs e)
        {
            Lift.setpoint.NewDestination(100, 30);
            Yaw.setpoint.NewDestination(100, 60);
            Lift.setpoint.NewDestination(0, 30);
        }

        private async void RoutineThree_Click(object sender, RoutedEventArgs e)
        {
            if (await Confirm_Async("WOAH WOAH WOAH", "You sure you want to do that?"))
            {
                while (true)
                {
                    await Lift.setpoint.NewDestination(randint(-100, 100), randint(0, 10));
                    await Yaw.setpoint.NewDestination(randint(-100, 100), randint(0, 10));
                }
            }
            else
            {
                Alert_Async("OH THANK GOD", "Yeah, probably a good idea.");
            }
        }

        #endregion


        private async void Alert_Async(string title, string message)
        {
            MetroDialogOptions.AffirmativeButtonText = "OK";
            await this.ShowMessageAsync(title, message);
        }

        private async Task<bool> Confirm_Async(string title, string message)
        {
            MetroDialogOptions.AffirmativeButtonText = "OK";
            MetroDialogOptions.NegativeButtonText = "NO";

            var res = await this.ShowMessageAsync(title, message,
                        MessageDialogStyle.AffirmativeAndNegative);

            return (res == MessageDialogResult.Affirmative);
        }

        int randint(int low, int high)
        {
            return new Random().Next(low, high);
        }


    }
}
