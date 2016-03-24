using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using WpfApplication1.Properties;
using MahApps.Metro.Controls.Dialogs;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Threading;
using OxyPlot;
using OxyPlot.Axes;

namespace WpfApplication1
{


    public class Trial
    {
        public DateTime timeStarted { get; set; }
        public DateTime timeFinished { get; set; }
        public TimeSpan timeElapsed { get { return timeFinished - timeStarted; } }

        public double KP {get; set;}
        public double KI { get; set; }
        public double KD { get; set; }

        public double error { get; set; }
        public bool timedOut { get; set;  }
        public bool settled { get; set;  }

        public int direction { get; set; }
        public double target { get; set; }
    }


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow :  INotifyPropertyChanged
    {
        bool abortTrials = false;
        Settings settings = Properties.Settings.Default;
        SerialPort port = new SerialPort(Settings.Default.ComPort, Settings.Default.BaudRate, Parity.None, 8, StopBits.One);

        List<double> recordedPosition = new List<double>();
        List<double> recordedError = new List<double>();
        List<double> recordedOutput = new List<double>();

        private List<Trial> _trials;
        public List<Trial> trials
        {
            get { return _trials; }
            set
            {
                if(_trials != value)
                {
                    _trials = value;
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


        private double _currentPosition;
        public double currentPosition
        {
            get { return _currentPosition; }
            set
            {
                if( _currentPosition != value)
                {
                    _currentPosition = value;
                    OnPropertyChanged();
                }
            }
        }


        private double _averagePosition;
        public double averagePosition
        {
            get { return _averagePosition; }
            set
            {
                if (_averagePosition != value)
                {
                    _averagePosition = value;
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


        private double _averagePercentError;
        public double averagePercentError
        {
            get { return _averagePercentError; }
            set
            {
                if (_averagePercentError != value)
                {
                    _averagePercentError = value;
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

        public MainWindow()
        {
            trials = new List<Trial>();
            InitializeComponent();
            port.DataReceived += new SerialDataReceivedEventHandler(serialDataRecieved);

            StepResponsePlot.Model
        }

        private void serialDataRecieved(object sender, SerialDataReceivedEventArgs e)
        {
            string data = String.Empty;
            try {
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
                recordedPosition.Add(pos); currentPosition = pos;
                averagePosition = recordedPosition.Skip(recordedPosition.Count() - Settings.Default.Samples).Average();
                averagePercentError = Math.Abs(averagePosition - currentSetpoint);
                Settings.Default.ReachedDestination = averagePercentError < Settings.Default.Tolerance;
            }
            if (double.TryParse(serialIn[1], out err))
            {
                recordedError.Add(err); currentError = err;
                runningError += currentError;
            }

            if (double.TryParse(serialIn[2], out output))
            {
                recordedOutput.Add(output); currentOutput = output;
            }
            
       }

        Task<Trial> trial(double kp, double ki, double kd, double setpoint)
        {
            runningError = 0;
            bool exitEarly = false;
            DateTime startTime = DateTime.Now;

            List<double> clonedList = new List<double>(recordedPosition);

            while (!Settings.Default.ReachedDestination && !exitEarly)
            {
                exitEarly = (DateTime.Now - startTime > TimeSpan.FromSeconds(Settings.Default.TimeoutSeconds));
            }

            Trial t = new Trial
            {
                timeStarted = startTime,
                timeFinished = DateTime.Now,
                error = runningError,
                timedOut = exitEarly,
                settled = Settings.Default.ReachedDestination,
                target = setpoint
            };

            t.KP = kp; t.KI = ki; t.KD = kd;

            return Task.FromResult(t);
        }


        void initializePlot()
        {

        }

        private void ApplyYawSettings_Click(object sender, RoutedEventArgs e)
        {
            initializePlot();

            if (portOpen())
            {
                SendYawData();
            }
            else
            {
                Alert_Async("NOT YET", "You can't do that yet because the port isn't open.");
            }
        }
        
        private void SendYawData()
        {
            if(!portOpen()) {
                Alert_Async("CAN'T", "The port isn't open yet.");
                return;
            }

            port.WriteLine($"-s {currentSetpoint}");
            port.WriteLine($"-p {Settings.Default.YawKP}");
            port.WriteLine($"-i {Settings.Default.YawKI}");
            port.WriteLine($"-d {Settings.Default.YawKD}");
        }

        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            port.Close();
            settings.Save();
            base.OnClosing(e);
        }

        bool portOpen()
        {
            if (Settings.Default.portOverride) return true;
            return port.IsOpen;
        }

        private async void TrialBeginButton_Click(object sender, RoutedEventArgs e)
        {
            abortTrials = false;

            if (!portOpen())
            {
                Alert_Async("OPEN THE CONNECTION", "We can't commence trials until you open the port first.");
                return;
            }

            currentSetpoint = 0;

            int inverter = 1;
            for (double kp = settings.KPFrom; kp <= settings.KPTo; kp = kp + settings.KPBy)
                for (double ki = settings.KIFrom; ki <= settings.KITo; ki = ki + settings.KIBy)
                    for (double kd = settings.KDFrom; kd <= settings.KDTo; kd = kd + settings.KDBy)
                    {
                        if (abortTrials)
                        {
                            Alert_Async("ABORTED", "Trials ended prematurely.");
                            return;
                        }

                        if (Settings.Default.InvertOffset)
                            inverter *= -1; // if the offset inverter is on, then setpoint should be -offset then +offset then -offset etc.

                        currentSetpoint += currentPosition + Settings.Default.SetpointOffset * inverter;


                        Settings.Default.YawKP = kp;
                        Settings.Default.YawKI = ki;
                        Settings.Default.YawKD = kd;

                        SendYawData();
                        Trial newTrial = await Task.Run(() =>
                        {
                            return trial(kp, ki, kd, currentSetpoint);
                        });

                        newTrial.direction = inverter;

                        trials.Add(newTrial);

                        

                     
                    }

        }

        private void TrialEndButton_Click(object sender, RoutedEventArgs e)
        {
            abortTrials = true;
            Alert_Async("WILL DO", "The trials will end after the most recently started trial finishes.");
        }

        public PlotModel DataPlot { get; set; }

        private static PlotModel CreatePlotModel()
        {
            var series = new OxyPlot.Series.LineSeries();

            for(int i = 1; i< 100; i++)
            {
                series.Points.Add(new DataPoint(DateTimeAxis.ToDouble(DateTime.Now), i));
            }

            var newPlot = new PlotModel
            {
                Title = ""
            };

            newPlot.Series.Add(series);


            var xAxis = new DateTimeAxis
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

            var linearAxis = new LinearAxis
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


            newPlot.Axes.Add(xAxis);
            newPlot.Axes.Add(linearAxis);


            return newPlot;
        }

        private void ManualSave_Click(object sender, RoutedEventArgs e)
        {
            StepResponsePlot.Model = CreatePlotModel();
            StepResponsePlot.Model.DefaultColors = OxyPalettes.Jet(StepResponsePlot.Series.Count).Colors;
                            
            Settings.Default.Save();
            Alert_Async("YOU GOT IT", 
                        @"Your settings are saved. This'll happen automatically on close but it's handy to save manually in case the program crashes. \n\n Not that it would do that.");
        }

        private void OpenConnection_Click(object sender, RoutedEventArgs e)
        {
            if (!portOpen())
            {
                var button = sender as Button;
                button.IsEnabled = false;
                button.Content = "Connected";
                port.Open();

            }
            else
            {
                Alert_Async("NO NEED", "The port's already open");   
            }
        }

        private async void Alert_Async(string title, string message)
        {
            MetroDialogOptions.AffirmativeButtonText = "OK";
            await this.ShowMessageAsync(title, message);
        }
    }
}
