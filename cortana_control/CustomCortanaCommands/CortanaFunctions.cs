using System;
using System.Collections.Generic;

using Windows.Storage;
using Windows.ApplicationModel;
using Windows.ApplicationModel.VoiceCommands;

using Windows.Media.SpeechRecognition;
using Windows.ApplicationModel.Activation;
using Windows.Devices.SerialCommunication;
using Windows.Devices.Enumeration;
using Windows.Storage.Streams;

using Windows.UI.Popups;
using System.Threading.Tasks;

namespace CustomCortanaCommands
{

    class CortanaFunctions
    {
        


        /*
        This is the lookup of VCD CommandNames as defined in 
        CustomVoiceCommandDefinitios.xml to their corresponding actions
        */
        public readonly static Dictionary<string, Delegate> vcdLookup = new Dictionary<string, Delegate>{

            /*
            {<command name from VCD>, (Action)(async () => {
                 <code that runs when that commmand is called>
            })}
            */

            {"TurnAround", (Action)(async () => {
                String selector = SerialDevice.GetDeviceSelector("COM3"); //Get the serial port on port '3'
                DeviceInformationCollection devices = await DeviceInformation.FindAllAsync(selector);
                if(devices.Count > 0)
                {
                    try
                    {
                        DeviceInformation deviceInfo = devices[0];
                        SerialDevice serialDevice = await SerialDevice.FromIdAsync(deviceInfo.Id);
                        serialDevice.BaudRate = 9600;
                        serialDevice.DataBits = 8;
                        serialDevice.StopBits = SerialStopBitCount.Two;
                        serialDevice.Parity = SerialParity.None;

                        var dataWriter = new DataWriter(serialDevice.OutputStream);
                        dataWriter.WriteString("-s -127.5");
                        await dataWriter.StoreAsync();
                        dataWriter.DetachStream();
                        dataWriter = null;

                    }
                    catch (Exception ex)
                    {
                        MessageDialog popup2 = new MessageDialog("Your GUI is using the COM port, please close it.");
                        await popup2.ShowAsync();
                    }
                }
                else
                {
                    var popup = new MessageDialog("Sorry, no device found.");
                    await popup.ShowAsync();
                }

            })},

            {"LiftOff", (Action)(async () => {
                      String selector = SerialDevice.GetDeviceSelector("COM3"); //Get the serial port on port '3'
                DeviceInformationCollection devices = await DeviceInformation.FindAllAsync(selector);
                if(devices.Count > 0)
                {
                    try
                    {
                        DeviceInformation deviceInfo = devices[0];
                        SerialDevice serialDevice = await SerialDevice.FromIdAsync(deviceInfo.Id);
                        serialDevice.BaudRate = 9600;
                        serialDevice.DataBits = 8;
                        serialDevice.StopBits = SerialStopBitCount.Two;
                        serialDevice.Parity = SerialParity.None;

                        var dataWriter = new DataWriter(serialDevice.OutputStream);
                        dataWriter.WriteString("-S 1000");
                        await dataWriter.StoreAsync();
                        dataWriter.DetachStream();
                        dataWriter = null;

                    }
                    catch (Exception ex)
                    {
                        MessageDialog popup2 = new MessageDialog("Your GUI is using the COM port, please close it.");
                        await popup2.ShowAsync();
                    }
                }
                else
                {
                    var popup = new MessageDialog("Sorry, no device found.");
                    await popup.ShowAsync();
                }

            })},

            {"Land", (Action)(async () => {
                             String selector = SerialDevice.GetDeviceSelector("COM3"); //Get the serial port on port '3'
                DeviceInformationCollection devices = await DeviceInformation.FindAllAsync(selector);
                if(devices.Count > 0)
                {
                    try
                    {
                        DeviceInformation deviceInfo = devices[0];
                        SerialDevice serialDevice = await SerialDevice.FromIdAsync(deviceInfo.Id);
                        serialDevice.BaudRate = 9600;
                        serialDevice.DataBits = 8;
                        serialDevice.StopBits = SerialStopBitCount.Two;
                        serialDevice.Parity = SerialParity.None;

                        var dataWriter = new DataWriter(serialDevice.OutputStream);
                        dataWriter.WriteString("-S -1000");
                        await dataWriter.StoreAsync();
                        dataWriter.DetachStream();
                        dataWriter = null;

                    }
                    catch (Exception ex)
                    {
                        MessageDialog popup2 = new MessageDialog("Your GUI is using the COM port, please close it.");
                        await popup2.ShowAsync();
                    }
                }
                else
                {
                    var popup = new MessageDialog("Sorry, no device found.");
                    await popup.ShowAsync();
                }
             })},


            {"TurnThreeSixty", (Action)(async() => {
                string selector = SerialDevice.GetDeviceSelector("COM3"); //Get the serial port on port '3'
                DeviceInformationCollection devices = await DeviceInformation.FindAllAsync(selector);
                if(devices.Count > 0)
                {
                    try
                    {
                        DeviceInformation deviceInfo = devices[0];
                        SerialDevice serialDevice = await SerialDevice.FromIdAsync(deviceInfo.Id);
                        serialDevice.BaudRate = 9600;
                        serialDevice.DataBits = 8;
                        serialDevice.StopBits = SerialStopBitCount.Two;
                        serialDevice.Parity = SerialParity.None;

                        DataWriter dataWriter = new DataWriter(serialDevice.OutputStream);
                        dataWriter.WriteString("-s 255");
                        await dataWriter.StoreAsync();
                        dataWriter.DetachStream();
                        dataWriter = null;
                    }
                    catch (Exception ex)
                    {
                        MessageDialog popup2 = new MessageDialog("Your GUI is using the COM port, please close it.");
                        await popup2.ShowAsync();
                    }
                }
                else
                {
                    MessageDialog popup = new MessageDialog("Sorry, no device found.");
                    await popup.ShowAsync();
                }
            })}

        };

        /*
        Register Custom Cortana Commands from VCD file
        */
        public static async void RegisterVCD()
        {
            StorageFile vcd = await Package.Current.InstalledLocation.GetFileAsync(@"CustomVoiceCommandDefinitions.xml");
            await VoiceCommandDefinitionManager.InstallCommandDefinitionsFromStorageFileAsync(vcd);
        }

        /*
        Look up the spoken command and execute its corresponding action
        */
        public static void RunCommand(VoiceCommandActivatedEventArgs cmd)
        {
            SpeechRecognitionResult result = cmd.Result;
            string commandName = result.RulePath[0];
            vcdLookup[commandName].DynamicInvoke();
        }


   
    }
}
