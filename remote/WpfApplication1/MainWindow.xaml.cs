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
        public static void AddPoint(PlotModel plot, List<double>values)
        {
            if (!Settings.Default.portOverride) return;

            var i = 0;
            foreach (double value in values)
            {
                DataPoint pnt = new DataPoint(DateTimeAxis.ToDouble(DateTime.Now), value);
                (plot.Series[i] as OxyPlot.Series.LineSeries).Points.Add(pnt);
                i++;
            }

            plot.InvalidatePlot(true);
        }



        public static PlotModel Setup(string title)
        {
            PlotModel plot = new PlotModel();
            plot.Title = title;
            plot.TitleColor = OxyColors.Gray;
            plot.Series.Add(new OxyPlot.Series.LineSeries()); // for current position
            plot.Series.Add(new OxyPlot.Series.LineSeries()); // for setpoint
            plot.Series.Add(new OxyPlot.Series.LineSeries()); // for output

            DateTimeAxis xAxis = new DateTimeAxis
            {
                Position = AxisPosition.Bottom,
                StringFormat = "hh:mm:ss",
                Title = "Time",
                IntervalLength = 60,
                AxislineColor = OxyColors.Gray,
                TextColor = OxyColors.LightGray,
                TicklineColor = OxyColors.Gray,
                MinorGridlineColor = OxyColors.Gray,
                MajorGridlineColor = OxyColors.Gray,
                ExtraGridlineColor = OxyColors.Gray,
                IntervalType = DateTimeIntervalType.Seconds,
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


    public class Setpoint : INotifyPropertyChanged
    {
        public string identifier { get; set; }

        public void Send(SerialPort serial, double value)
        {
            serial.WriteLine($"{identifier} {value}");
        }
        public Task<Destination> NewDestination(SerialPort serial, double setpoint, int seconds)
        {
            Send(serial, setpoint);

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

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
    }

    public class Gains
    {
        public List<string> identifiers { get; set; }
        public void Send(SerialPort serial, double kp, double ki, double kd)
        {
            serial.WriteLine($"{identifiers[0]} {kp}");
            serial.WriteLine($"{identifiers[1]} {ki}");
            serial.WriteLine($"{identifiers[2]} {kd}");
        }
    }

    public class DegreeOfFreedom : INotifyPropertyChanged
    {
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

        bool tryToRead;

        public MainWindow()
        {
            
            InitializeComponent();
            port.DataReceived += new SerialDataReceivedEventHandler(serialDataRecieved);

            tryToRead = true;

            // initialize axes
            Yaw = new DegreeOfFreedom
            {
                trials = new List<Destination>(),
                Plot = PositionGraph.Setup("Yaw Position"),
                currentPosition = 127.5
            };

            Yaw.setpoint.identifier = settings.YawSetpointIdentifier;
            Yaw.currentSetpoint = Yaw.currentPosition;
            Yaw.gains.identifiers = new List<string> { "-p", "-i", "-d" };


            Lift = new DegreeOfFreedom
            {
                trials = new List<Destination>(),
                Plot = PositionGraph.Setup("Lift Position"),
                currentPosition = 0
            };
            Lift.setpoint.identifier = settings.LiftSetpointIdentifier;
            Lift.currentSetpoint = Lift.currentPosition;
            Lift.gains.identifiers = new List<string> { "-P", "-I", "-D" };
        }


        private void serialDataRecieved(object sender, SerialDataReceivedEventArgs e)
        {
            if (!port.IsOpen) return;

            if (!tryToRead) return;

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
            if (serialIn.Count() < 4) return; // sometimes we get incomplete data, ignore that

            // yaw values
            double yawPosition, yawOutput;
            if (double.TryParse(serialIn[0], out yawPosition))
            {
                this.Dispatcher.Invoke((Action)(() => {
                    YawPosition.Value = yawPosition; 
                    YawCurrentError.Value = Yaw.currentSetpoint - yawPosition;
                    YawRunningError.Value += Math.Abs(Yaw.currentSetpoint - yawPosition);
                }));

                // DATABINDING TOO SLOW WHEN GRAPHING, SO REMOVE IT FOR READ-ONLY VALUES
                // IT'S A CRYIN' SHAME BUT OH WELL

                //Yaw.currentPosition = yawPosition;
                //Yaw.currentError = Yaw.currentSetpoint - Yaw.currentPosition;
                //Yaw.runningError += Math.Abs(Yaw.currentError);
                //Yaw.currentOutput = yawOutput;


                PositionGraph.AddPoint(Yaw.Plot, new List<double> { yawPosition, Yaw.currentSetpoint });
            }
            if (double.TryParse(serialIn[1], out yawOutput))
            {
                this.Dispatcher.Invoke((Action)(() => {
                    YawOutput.Value = yawOutput;
                }));

            }

            // lift values
            double liftPosition, liftOutput;
            if (double.TryParse(serialIn[2], out liftPosition))
            {
                this.Dispatcher.Invoke((Action)(() => {
                    LiftPosition.Value = liftPosition;
                    LiftCurrentError.Value = Lift.currentSetpoint - liftPosition;
                    LiftRunningError.Value += Math.Abs(Lift.currentSetpoint - liftPosition);
                }));

                // DATABINDING TOO SLOW WHEN GRAPHING, SO REMOVE IT FOR READ-ONLY VALUES
                // IT'S A CRYIN' SHAME BUT OH WELL

                //Lift.currentPosition = liftPosition;
                //Lift.currentError = Lift.currentSetpoint - Lift.currentPosition;
                //Lift.runningError += Math.Abs(Lift.currentError);
                //Lift.currentOutput = liftOutput;


                PositionGraph.AddPoint(Lift.Plot, new List<double> { liftPosition, Lift.currentSetpoint });
            }
            if (double.TryParse(serialIn[3], out liftOutput))
            {
                this.Dispatcher.Invoke((Action)(() => {
                    LiftOutput.Value = liftOutput;
                }));
            }

        }

        #region EventHandler

        private void ApplyYawSettings_Click(object sender, RoutedEventArgs e)
        {
            if (port.IsOpen)
            {
                Yaw.Plot = PositionGraph.Setup($"Yaw Position, {settings.YawKP}, {settings.YawKI}, {settings.YawKD}");
                Yaw.setpoint.Send(port, Yaw.currentSetpoint);
                Yaw.gains.Send(port, settings.YawKP, settings.YawKI, settings.YawKD);
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
                Lift.Plot = PositionGraph.Setup($"Lift Position, {settings.LiftKP}, {settings.LiftKI}, {settings.LiftKD}");
                Lift.setpoint.Send(port, Lift.currentSetpoint);
                Lift.gains.Send(port, settings.LiftKP, settings.LiftKI, settings.LiftKD);
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
                            return Yaw.setpoint.NewDestination(port, Yaw.currentSetpoint, settings.TimeoutSeconds);
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
                Button button = sender as Button;
                button.Content = "Disconnect";
            }
            else
            {
                try
                {
                    tryToRead = false;
                    port.Close();
                }
                catch (Exception ex)
                {
                    Alert_Async("OOPS", "Couldn't close because: " + ex.Message);
                }
                Button button = sender as Button;
                button.Content = "Connect";
            }
        }
        
        private async void RoutineOne_Click(object sender, RoutedEventArgs e)
        {
            if (port.IsOpen)
            {
                YawTargetMode.IsChecked = false;

                await Task.Run(() =>
                {
                    this.Dispatcher.Invoke((Action)(() =>
                    {
                        YawTarget.Value = -130;
                    }));
                    Yaw.setpoint.NewDestination(port, -130, 5).Wait();
                });

                await Task.Run(() =>
                {
                    this.Dispatcher.Invoke((Action)(() =>
                    {
                        YawTarget.Value = 0;
                    }));
                    Yaw.setpoint.NewDestination(port, 0, 5).Wait();
                });

                await Task.Run(() =>
                {
                    this.Dispatcher.Invoke((Action)(() =>
                    {
                        YawTarget.Value = 130;
                    }));
                    Yaw.setpoint.NewDestination(port, 130, 5).Wait();
                });

                YawTargetMode.IsChecked = true;
            }
            else
            {
                Alert_Async("NOT YET", "Open the port first please.");
            }
        }

        private void RoutineTwo_Click(object sender, RoutedEventArgs e)
        {
            Lift.setpoint.NewDestination(port, 100, 30);
            Yaw.setpoint.NewDestination(port, 100, 60);
            Lift.setpoint.NewDestination(port, 0, 30);
        }

        private async void RoutineThree_Click(object sender, RoutedEventArgs e)
        {
            if (await Confirm_Async("WOAH WOAH WOAH", "You sure you want to do that?"))
            {
                while (true)
                {
                    await Lift.setpoint.NewDestination(port, randint(-100, 100), randint(0, 10));
                    await Yaw.setpoint.NewDestination(port, randint(-100, 100), randint(0, 10));
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

        private void MetroWindow_Closing(object sender, CancelEventArgs e)
        {
            tryToRead = false;
            port.Close();
            settings.Save();
        }

        private void ViewPlotSwitch_Click(object sender, RoutedEventArgs e)
        {
            if (PlotsFlyout.IsOpen)
            {

                for (int i = 0; i < 4; i++)
                {
                    YawTabGrid.ColumnDefinitions.Add(new ColumnDefinition());
                }
            }
            else
            {
                for (int i = 0; i < 4; i++)
                {
                    YawTabGrid.ColumnDefinitions.RemoveAt(YawTabGrid.ColumnDefinitions.Count - 1);
                }
            }
        }
    }
}