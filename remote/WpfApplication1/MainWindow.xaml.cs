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
    public partial class MainWindow 
    {
        Settings settings = Properties.Settings.Default;

        //todo these should be settings!!
        const int SECONDS_TO_TIMEOUT = 10;
        const string COM_PORT = "COM5";
        const int BAUD_RATE = 9600;

        List<Trial> trials = new List<Trial>();
        SerialPort port = new SerialPort(COM_PORT, BAUD_RATE, Parity.None, 8, StopBits.One);

        List<double> recordedPosition;
        List<double> recordedError;
        List<double> recordedOutput;
        double currentSetpoint;
        double totalError;

        public MainWindow()
        {
            InitializeComponent();

            port.DataReceived += new SerialDataReceivedEventHandler(serialDataRecieved);
            //port.Open();
            
            TrialsDataGrid.ItemsSource = trials;

        }

        private void serialDataRecieved(object sender, SerialDataReceivedEventArgs e)
        {
            string data = port.ReadExisting();
            List<string> serialIn = data.Split(' ').ToList();

            double pos, err, output;
            if (double.TryParse(serialIn[0], out pos))      recordedPosition.Add(pos);
            if (double.TryParse(serialIn[1], out err))      recordedError.Add(err);
            if (double.TryParse(serialIn[2], out output))   recordedOutput.Add(output);

       }

        private void serialDataSend(string msg)
        {

        }

        Trial trial(double kp, double ki, double kd, double setpoint)
        {
            totalError = 0;
            bool exitEarly = false;
            DateTime startTime = DateTime.UtcNow;

            while (!settledWithinTolerance())
            {
                if (DateTime.UtcNow - startTime < TimeSpan.FromSeconds(SECONDS_TO_TIMEOUT))
                {
                    exitEarly = true;
                    break;
                }

                totalError += recordedError.Last();
            }

            Trial t = new Trial
            {
                timeStarted = startTime,
                timeFinished = DateTime.UtcNow,
                error = totalError,
                timedOut = exitEarly
            };

            t.YawKP = kp; t.YawKI = ki; t.YawKD = kd;

            return t;
        }

        bool settledWithinTolerance()
        {
            // todo:
            return true;
        }

        private void ApplyYawSettings_Click(object sender, RoutedEventArgs e)
        {
            // todo:
            // send values to arduino microcontroller using serial
        }


        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            settings.Save();
            port.Close();
            base.OnClosing(e);
        }

        private void TrialBeginButton_Click(object sender, RoutedEventArgs e)
        {
            currentSetpoint = 0;

            for (double kp = settings.YawFrom; kp <= settings.YawTo; kp = kp + settings.YawBy)
                for (double ki = settings.YawFrom; ki <= settings.YawTo; ki = ki + settings.YawBy)
                    for (double kd = settings.YawFrom; kd <= settings.YawTo; kd = kp + settings.YawBy)
                    {
                        Trial newTrial = trial(kp, ki, kd, currentSetpoint);
                        trials.Add(newTrial);
                        currentSetpoint += 180;
                    }

        }

        private void TrialEndButton_Click(object sender, RoutedEventArgs e)
        {

        }
    }
}
