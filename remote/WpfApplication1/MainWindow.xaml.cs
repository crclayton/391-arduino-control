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

namespace WpfApplication1
{


    public class Trial
    {
        public DateTime timeStarted { get; set; }
        public DateTime timeFinished { get; set; }
        public TimeSpan timeElapsed { get { return timeFinished - timeStarted; } }

        public double YawKP {get; set;}
        public double YawKI { get; set; }
        public double YawKD { get; set; }


        public double LiftKP { get; set; }
        public double LiftKI { get; set; }
        public double LiftKD { get; set; }

        public double error { get; set; }
        public bool timedOut { get; set;  }
    }


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow :  INotifyPropertyChanged
    {


        Settings settings = Properties.Settings.Default;

        List<Trial> trials = new List<Trial>();
        SerialPort port = new SerialPort(Settings.Default.ComPort, Settings.Default.BaudRate, Parity.None, 8, StopBits.One);

        List<double> recordedPosition = new List<double>();
        List<double> recordedError = new List<double>();
        List<double> recordedOutput = new List<double>();


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
            InitializeComponent();
            port.DataReceived += new SerialDataReceivedEventHandler(serialDataRecieved);
            TrialsDataGrid.ItemsSource = trials;
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

        private void serialDataSend(string msg)
        {

        }

        Trial trial(double kp, double ki, double kd, double setpoint)
        {
            runningError = 0;
            bool exitEarly = false;
            DateTime startTime = DateTime.UtcNow;

            while (!settledWithinTolerance(settings.Tolerance, settings.Samples))
            {
                if (DateTime.UtcNow - startTime > TimeSpan.FromSeconds(Settings.Default.TimeoutSeconds))
                {
                    exitEarly = true;
                    break;
                }
            }

            Trial t = new Trial
            {
                timeStarted = startTime,
                timeFinished = DateTime.UtcNow,
                error = runningError,
                timedOut = exitEarly
            };

            t.YawKP = kp; t.YawKI = ki; t.YawKD = kd;

            return t;
        }

        bool settledWithinTolerance(double tolerancePercent, int samples)
        {
            double average = recordedPosition.Sum() / recordedPosition.Count();
            return Math.Abs(average - currentSetpoint) < tolerancePercent;
        }


        private void ApplyYawSettings_Click(object sender, RoutedEventArgs e)
        {
            if (port.IsOpen)
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
        

        private void TrialBeginButton_Click(object sender, RoutedEventArgs e)
        {
            currentSetpoint = 0;

            for (double kp = settings.YawFrom; kp <= settings.YawTo; kp = kp + settings.YawBy)
                for (double ki = settings.YawFrom; ki <= settings.YawTo; ki = ki + settings.YawBy)
                    for (double kd = settings.YawFrom; kd <= settings.YawTo; kd = kp + settings.YawBy)
                    {
                        Settings.Default.YawKP = kp;
                        Settings.Default.YawKI = ki;
                        Settings.Default.YawKD = kd;

                        SendYawData();
                        Trial newTrial = trial(kp, ki, kd, currentSetpoint);
                        trials.Add(newTrial);
                        currentSetpoint += 50;
                      
                    }

        }

        private void TrialEndButton_Click(object sender, RoutedEventArgs e)
        {

        }

        private void OpenConnection_Click(object sender, RoutedEventArgs e)
        {
            if (!port.IsOpen)
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
