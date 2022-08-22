//tabs=4
// --------------------------------------------------------------------------------
// ASCOM Dome driver for LifeRoof
//
// Description:	Simple ROR driver with commands for open, close and status.
// Expanded to include ability to read independent rain and mount position sensor
// Implements:	ASCOM Dome interface version: <2.1>
// Author:		(C2015) Chris Woodhouse <chris@digitalastrophotography.co.uk>
//
// Edit Log:
//
// Date			Who	Vers	Description
// -----------	---	-----	-------------------------------------------------------
// 10-Nov-2015	CJW	1.0.0	Initial edit, created from ASCOM driver template
// 24-Dec-2015  CJW 2.0     Re-write of Serial handling, using event handler, thanks to an idea from Per 
// 26-Dec-2015  CJW 2.1     Included provision in Arduino and here for direct commands
//                          to bypass Arduino standard safety operations
// 3 Jan        CJW 2.2     Added other fields to communication phrase, for expansion
// Nov 16       CJW 2.2    recompiling 
// Feb 21       CJW 2.3    works with version 2.5 Arduino code, faster status updates
// Feb'21       CJW 2.4    reworked in Ascom 6.5 templates
// Feb'21       CJW 2.5    reworked so that open or close waits for response
// Feb'21       CJW 2.6    updated with different transmission, including sensor overrides
// Aug'22       CJW 2.7    added supported actions documentation (overdue!!) , 19200 baud
// --------------------------------------------------------------------------------



// This is used to define code in the template that is specific to one class implementation
// unused code canbe deleted and this definition removed.
#define Dome

using System;
using System.Runtime.InteropServices;
using ASCOM.Astrometry.AstroUtils;
using ASCOM.Utilities;
using ASCOM.DeviceInterface;
using System.Globalization;
using System.Collections;
using System.IO.Ports;
using System.Threading.Tasks;

namespace ASCOM.MegaRoof
{
    //
    // Your driver's DeviceID is ASCOM.LifeRoof.Dome
    //
    // The Guid attribute sets the CLSID for ASCOM.MegaRoof.Dome
    // The ClassInterface/None addribute prevents an empty interface called
    // _MegaRoof from being created and used as the [default] interface

    /// <summary>
    /// ASCOM Dome Driver for LifeRoof.
    /// </summary>
    [Guid("cf98241a-eef8-431f-b3b0-0d63d235ca20")]
    [ClassInterface(ClassInterfaceType.None)]
    public class Dome : IDomeV2
    {
        /// <summary>
        /// ASCOM DeviceID (COM ProgID) for this driver.
        /// The DeviceID is used by ASCOM applications to load the driver at runtime.
        /// </summary>
        internal static string driverID = "ASCOM.MegaRoof.Dome";
        /// <summary>
        /// Driver description that displays in the ASCOM Chooser.
        /// </summary>
        private static string driverDescription = "MegaRoof ROR Driver";

        internal static string comPortProfileName = "COM Port"; // Constants used for Profile persistence
        internal static string comPortDefault = "COM1";
        internal static string traceStateProfileName = "Trace Level";
        internal static string traceStateDefault = "true";

        internal static string comPort; // Variables to hold the current device configuration
        //internal static bool traceState;  // used to hold setup log file option status
        internal string buffer = "";
        internal bool startCharRx;  // indicates start of message detected
        internal char endChar = '#';  // end character of Arduino response
        internal char startChar = '$';  // start character of Arduino response
        //internal char[] delimeter = new char[] {','};  // delimiter between Arduino response values
        internal char[] delimeter = { ',' };
        internal string arduinoMessage = ""; // string that builds up received message
        internal bool dataRx = false;  // indicates that data is available to read

        /// <summary>
        /// Private variable to hold the connected state
        /// </summary>
        private bool connectedState;
        private string[] arduinoStatus = new string[6];  //  "$dry,roof,park,rainsense,parksense,wind(xx.x)#"

        /// <summary>
        /// Private variable to hold an ASCOM Utilities object
        /// </summary>
        private Util utilities;

        /// <summary>
        /// Private variable to hold an ASCOM AstroUtilities object to provide the Range method
        /// </summary>
        private AstroUtils astroUtilities;

        /// <summary>
        /// Variable to hold the trace logger object (creates a diagnostic log file with information that you specify)
        /// </summary>
        internal TraceLogger tl;
        private SerialPort Serial;  // my serial port instance of ASCOM serial port

        /// <summary>
        /// Initializes a new instance of the <see cref="MegaRoof"/> class.
        /// Must be public for COM registration.
        /// </summary>
        public Dome()
        {
            tl = new TraceLogger("", "MegaRoof");
            ReadProfile(); // Read device configuration from the ASCOM Profile store
            tl.LogMessage("Dome", "Starting initialisation");
            connectedState = false; // Initialise connected to false
            utilities = new Util(); //Initialise util object
            astroUtilities = new AstroUtils(); // Initialise astro-utilities object
            Serial = new SerialPort();  // standard .net serial port
            tl.LogMessage("Dome", "Completed initialisation");
        }
        // OpenArduino initilises serial port and set up an event handler to suck in characters
        // it runs in the background, Arduino broadcasts every 4 seconds
        private bool OpenArduino()
        {
            Serial.BaudRate = 19200;  // note original was 9600
            Serial.PortName = comPort;
            Serial.Parity = Parity.None;
            Serial.DataBits = 8;
            Serial.Handshake = System.IO.Ports.Handshake.None;
            Serial.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(ReceiveData);
            Serial.ReceivedBytesThreshold = 1;
            try
            {
                Serial.Open();              // open port
                Serial.DiscardInBuffer();   // and clear it out just in case
            }
            catch (Exception)
            {
                return false;
            }
            return true;
        }


        // ReceiveData is based on a code fragment suggested by Per and reads characters as they arrive
        // it decodes the messages, looking for framing characters and then splits the CSV string into
        // component parts to represent the status flags from the Arduino 
        private void ReceiveData(object sender, SerialDataReceivedEventArgs e)
        {
            if (e.EventType == System.IO.Ports.SerialData.Chars)
            {
                while (Serial.BytesToRead > 0)
                {
                    char c = (char)Serial.ReadChar();  // wait for start character
                    if (!startCharRx)
                    {
                        if (c == startChar)  // and then initialise the message
                        {
                            startCharRx = true;
                            buffer = "";  // clear buffer
                        }
                    }
                    else
                    {
                        if (c == endChar)
                        {
                            arduinoMessage = buffer;  // transfer the buffer to the message and clear the buffer
                            buffer = "";
                            startCharRx = false;
                            if (arduinoMessage.Length == 14) // check the message length is OK (was 17 with old message)
                            {
                                dataRx = true; // tell the world that data is available
                                arduinoStatus = arduinoMessage.Split(delimeter);  // rain / roof / park / rainsense / mountsense / spare
                            }
                            else  // message was corrupted
                            {
                                dataRx = false;
                                tl.LogMessage("communications", "corrupted message length");
                                arduinoMessage = "";
                            }
                        }
                        else
                        {
                            buffer += c;  // build up message string in buffer
                        }
                    }
                }
            }
        }

        //
        // PUBLIC COM INTERFACE IDomeV2 IMPLEMENTATION
        //

        #region Common properties and methods.

        /// <summary>
        /// Displays the Setup Dialog form.
        /// If the user clicks the OK button to dismiss the form, then
        /// the new settings are saved, otherwise the old values are reloaded.
        /// THIS IS THE ONLY PLACE WHERE SHOWING USER INTERFACE IS ALLOWED!
        /// </summary>
        public void SetupDialog()
        {
            // consider only showing the setup dialog if not connected
            // or call a different dialog if connected
            if (IsConnected)
                System.Windows.Forms.MessageBox.Show("Already connected, just press OK");

            using (SetupMegaRoof F = new SetupMegaRoof(tl))
            {
                var result = F.ShowDialog();
                if (result == System.Windows.Forms.DialogResult.OK)
                {
                    WriteProfile(); // Persist device configuration values to the ASCOM Profile store
                }
            }
        }

        public ArrayList SupportedActions
        {
            get
            {
                ArrayList suptaction =new ArrayList()
                { "INIT","FORCEOPEN","FORCECLOSE","NORAINSENSE","NOPARKSENSE","RAINSENSE","PARKSENSE","PARKSENSOR","RAINSENSOR"};
                    return suptaction;
            }
        }

        public string Action(string actionName, string actionParameters)
        {
            LogMessage("", "Action {0}, parameters {1} not implemented", actionName, actionParameters);
            throw new ASCOM.ActionNotImplementedException("Action " + actionName + " is not implemented by this driver");
        }

        public void CommandBlind(string command, bool raw)
        {
            CheckConnected("CommandBlind");
            CommandString(command, raw);  // don't need the return string
            return;
        }

        public bool CommandBool(string command, bool raw)
        {
            CheckConnected("CommandBool");
            return (CommandString(command, raw) == "1");// return state of command from arduino
        }

        // CommandString expanded to treat commands differently
        // raw parameter is set to false to use # delimiter, or true for none (not used)
        // status commands are only replied upon if data is valid (dataRX ==true)
        public  string CommandString(string command, bool raw)
        {
            CheckConnected("CommandString");
            // this is the customised I/O to the serial port, used by all commands
            // status commands are interpreted from cache variables, and commands
            // are issued
            try
            {                
                if (!raw) // if command uses delimiter
                {
                    tl.LogMessage("attempting commandstring", command);
                    if (IsConnected)  // only if connected  - try and avoid comms error
                    {
                        if (command == "RAIN" && dataRx) return (arduinoStatus[0]);         // these return status from Arduino message array
                        else if (command == "SHUTTERSTATUS" && dataRx) return (arduinoStatus[1]);
                        else if (command == "PARK" && dataRx) return (arduinoStatus[2]);
                        else if (command == "RAINSENSOR" && dataRx) return (arduinoStatus[3]);
                        else if (command == "PARKSENSOR" && dataRx) return (arduinoStatus[4]);
                        else if (command == "SPARE" && dataRx) return (arduinoStatus[5]);  // replace "SPARE" with something else in due course nn.n
                        else
                        {
                            if (command == "SHUTTERSTATUS") tl.LogMessage("comms error", "no shutter data");  // diagnostic
                            Serial.Write(command + "#"); return ("1");  // if not a data request, it sends the string command   (bool/blind/string) 
                        }
                    }
                    throw new ASCOM.NotConnectedException("com port not connected");
                }
                else  // do nothing if the command is not using delimiter
                {
                    tl.LogMessage("commandstring ", "Not implemented without # terminator");
                    throw new ASCOM.MethodNotImplementedException("CommandString");
                }
            }
            catch (Exception)  // better luck next time :)
            {
                System.Windows.Forms.MessageBox.Show("Timed out, press OK to recover");
                return ("comms error");
            }
        }

        public void Dispose()
        {
            // Clean up the trace logger and util objects
            tl.Enabled = false;
            tl.Dispose();
            tl = null;
            utilities.Dispose();
            utilities = null;
            astroUtilities.Dispose();
            astroUtilities = null;
            Serial.Dispose();
            Serial = null;
        }

        public bool Connected
        {
            get
            {
                LogMessage("Connected", "Get {0}", IsConnected);
                return IsConnected;
            }
            set
            {
                LogMessage("Connected", "Set {0}", value);
                if (value == IsConnected)
                    return;

                if (value)
                {
                    connectedState = true;
                    LogMessage("Connected Set", "Connecting to port {0}", comPort);
                    if (!OpenArduino()) Serial.Close();
                }
                else
                {
                    connectedState = false;
                    LogMessage("Connected Set", "Disconnecting from port {0}", comPort);
                    Serial.Close();  //disconnect to serial
                    Serial.Dispose();
                }
            }
        }

        public string Description
        {
            get
            {
                tl.LogMessage("Description Get", driverDescription);
                return driverDescription;
            }
        }

        public string DriverInfo
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverInfo = "MegaRoof Arduino Interface Version: " + String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverInfo Get", driverInfo);
                return driverInfo;
            }
        }

        public string DriverVersion
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverVersion = String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverVersion Get", driverVersion);
                return driverVersion;
            }
        }

        public short InterfaceVersion
        {
            // set by the driver wizard
            get
            {
                LogMessage("InterfaceVersion Get", "2");
                return Convert.ToInt16("2");
            }
        }

        public string Name
        {
            get
            {
                string name = "MegaRoof";
                tl.LogMessage("Name Get", name);
                return name;
            }
        }

        #endregion

        #region IDome Implementation

        // private bool domeShutterState = false; // Variable to hold the open/closed status of the shutter, true = Open

        public void AbortSlew()
        {
            // This is a mandatory parameter, which in this case tells the Arduino to shut off all roof and mount movements
            tl.LogMessage("AbortSlew", "Completed");
            CommandBlind("ABORT", false);  // stops roof move and mount move
        }

        public double Altitude
        {
            get
            {
                //tl.LogMessage("Altitude Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("Altitude", false);
            }
        }

        public bool AtHome
        {
            get
            {
                //tl.LogMessage("AtHome Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("AtHome", false);
            }
        }

        public bool AtPark
        {
            get
            {
                //tl.LogMessage("AtPark Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("AtPark", false);
            }
        }

        public double Azimuth
        {
            get
            {
                //tl.LogMessage("Azimuth Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("Azimuth", false);
            }
        }

        public bool CanFindHome
        {
            get
            {
                //tl.LogMessage("CanFindHome Get", false.ToString());
                return false;
            }
        }

        public bool CanPark
        {
            get
            {
                //tl.LogMessage("CanPark Get", false.ToString());
                return false;
            }
        }

        public bool CanSetAltitude
        {
            get
            {
                //tl.LogMessage("CanSetAltitude Get", false.ToString());
                return false;
            }
        }

        public bool CanSetAzimuth
        {
            get
            {
                //tl.LogMessage("CanSetAzimuth Get", false.ToString());
                return false;
            }
        }

        public bool CanSetPark
        {
            get
            {
                //tl.LogMessage("CanSetPark Get", false.ToString());
                return false;
            }
        }

        public bool CanSetShutter
        {
            get
            {
                tl.LogMessage("CanSetShutter Get", true.ToString());
                return true;
            }
        }

        public bool CanSlave
        {
            get
            {
                //tl.LogMessage("CanSlave Get", false.ToString());
                return false;
            }
        }

        public bool CanSyncAzimuth
        {
            get
            {
               // tl.LogMessage("CanSyncAzimuth Get", false.ToString());
                return false;
            }
        }

        public void CloseShutter()
        {
            try
            {
                var t = Task.Run(async delegate { await Task.Delay(600); });
                dataRx = false;  // new received command will change this
                CommandBlind("CLOSE", false);  // fed through CommandBlind and then CommandString
                tl.LogMessage("CloseShutter", "Shutter asked to close");
                {
                    int counter = 0;
                    while (!dataRx || counter < 5)
                    {
                        t.Wait();
                        counter++;
                    }
                    if (!dataRx) throw new ASCOM.NotConnectedException("no reply!");
                }
            }
            catch (Exception)
            {
                tl.LogMessage("Comms issue", "No response from Roof within poll time");
            }
        }


        public void FindHome()
        {
            //tl.LogMessage("FindHome", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("FindHome");
        }

        public void OpenShutter()
        {
            try
            {
                var t = Task.Run(async delegate { await Task.Delay(600); }); 
                dataRx = false;
                CommandBlind("OPEN", false);  // fed through CommandBlind and then CommandString
                tl.LogMessage("OpenShutter", "Shutter asked to open");
                {
                    int counter = 0;
                    while (!dataRx || counter < 5)
                    {
                        t.Wait();
                        counter++;
                    }
                    if (!dataRx) throw new ASCOM.NotConnectedException("no reply!");
                }
            }
            catch (Exception)
            {
                tl.LogMessage("Comms issue", "No response from Roof within poll time");
            }
        }

        public void Park()
        {
            //tl.LogMessage("Park", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("Park");
        }

        public void SetPark()
        {
            //tl.LogMessage("SetPark", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("SetPark");
        }

        public ShutterState ShutterStatus
        {
            
            get
            {
                    string status = CommandString("SHUTTERSTATUS", false);
                    tl.LogMessage("SHUTTERSTATUS", status);
                    switch (status)
                    {
                        case "0":
                            tl.LogMessage("ShutterStatus", ShutterState.shutterOpen.ToString());
                            return ShutterState.shutterOpen;
                        case "1":
                            tl.LogMessage("ShutterStatus", ShutterState.shutterClosed.ToString());
                            return ShutterState.shutterClosed;
                        case "2":
                            tl.LogMessage("ShutterStatus", ShutterState.shutterOpening.ToString());
                            return ShutterState.shutterOpening;
                        case "3":
                            tl.LogMessage("ShutterStatus", ShutterState.shutterClosing.ToString());
                            return ShutterState.shutterClosing;
                        default:
                            tl.LogMessage("ShutterStatus", ShutterState.shutterError.ToString());
                            return ShutterState.shutterError;
                    }
                }
            }

        public bool Slaved
        {
            get
            {
                //tl.LogMessage("Slaved Get", false.ToString());
                return false;
            }
            set
            {
                //tl.LogMessage("Slaved Set", "not implemented");
                throw new ASCOM.PropertyNotImplementedException("Slaved", true);
            }
        }

        public void SlewToAltitude(double Altitude)
        {
            //tl.LogMessage("SlewToAltitude", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("SlewToAltitude");
        }

        public void SlewToAzimuth(double Azimuth)
        {
            //tl.LogMessage("SlewToAzimuth", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("SlewToAzimuth");
        }

        // late addition - this applies to ROR too
        public bool Slewing
        {
            get
            {
                bool slew = (ShutterStatus == ShutterState.shutterOpening || ShutterStatus==ShutterState.shutterClosing);
                tl.LogMessage("Slewing Get", slew.ToString() );
                return slew;
            }
        }

        public void SyncToAzimuth(double Azimuth)
        {
            //tl.LogMessage("SyncToAzimuth", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("SyncToAzimuth");
        }

        #endregion

        #region Private properties and methods
        // here are some useful properties and methods that can be used as required
        // to help with driver development

        #region ASCOM Registration

        // Register or unregister driver for ASCOM. This is harmless if already
        // registered or unregistered. 
        //
        /// <summary>
        /// Register or unregister the driver with the ASCOM Platform.
        /// This is harmless if the driver is already registered/unregistered.
        /// </summary>
        /// <param name="bRegister">If <c>true</c>, registers the driver, otherwise unregisters it.</param>
        private static void RegUnregASCOM(bool bRegister)
        {
            using (var P = new ASCOM.Utilities.Profile())
            {
                P.DeviceType = "Dome";
                if (bRegister)
                {
                    P.Register(driverID, driverDescription);
                }
                else
                {
                    P.Unregister(driverID);
                }
            }
        }

        /// <summary>
        /// This function registers the driver with the ASCOM Chooser and
        /// is called automatically whenever this class is registered for COM Interop.
        /// </summary>
        /// <param name="t">Type of the class being registered, not used.</param>
        /// <remarks>
        /// This method typically runs in two distinct situations:
        /// <list type="numbered">
        /// <item>
        /// In Visual Studio, when the project is successfully built.
        /// For this to work correctly, the option <c>Register for COM Interop</c>
        /// must be enabled in the project settings.
        /// </item>
        /// <item>During setup, when the installer registers the assembly for COM Interop.</item>
        /// </list>
        /// This technique should mean that it is never necessary to manually register a driver with ASCOM.
        /// </remarks>
        [ComRegisterFunction]
        public static void RegisterASCOM(Type t)
        {
            RegUnregASCOM(true);
        }

        /// <summary>
        /// This function unregisters the driver from the ASCOM Chooser and
        /// is called automatically whenever this class is unregistered from COM Interop.
        /// </summary>
        /// <param name="t">Type of the class being registered, not used.</param>
        /// <remarks>
        /// This method typically runs in two distinct situations:
        /// <list type="numbered">
        /// <item>
        /// In Visual Studio, when the project is cleaned or prior to rebuilding.
        /// For this to work correctly, the option <c>Register for COM Interop</c>
        /// must be enabled in the project settings.
        /// </item>
        /// <item>During uninstall, when the installer unregisters the assembly from COM Interop.</item>
        /// </list>
        /// This technique should mean that it is never necessary to manually unregister a driver from ASCOM.
        /// </remarks>
        [ComUnregisterFunction]
        public static void UnregisterASCOM(Type t)
        {
            RegUnregASCOM(false);
        }

        #endregion

        /// <summary>
        /// Returns true if there is a valid connection to the driver hardware
        /// </summary>
        private bool IsConnected
        {
            get
            {
                // check the actual serial connection (checks for unplugged)
                connectedState = Serial.IsOpen; 
                return connectedState;
            }
        }

        /// <summary>
        /// Use this function to throw an exception if we aren't connected to the hardware
        /// </summary>
        /// <param name="message"></param>
        private void CheckConnected(string message)
        {
            if (!IsConnected)
            {
                throw new ASCOM.NotConnectedException(message);
            }
        }

        /// <summary>
        /// Read the device configuration from the ASCOM Profile store
        /// </summary>
        internal void ReadProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Dome";
                tl.Enabled = Convert.ToBoolean(driverProfile.GetValue(driverID, traceStateProfileName, string.Empty, traceStateDefault));
                comPort = driverProfile.GetValue(driverID, comPortProfileName, string.Empty, comPortDefault);
            }
        }

        /// <summary>
        /// Write the device configuration to the  ASCOM  Profile store
        /// </summary>
        internal void WriteProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Dome";
                driverProfile.WriteValue(driverID, traceStateProfileName, tl.Enabled.ToString());
                driverProfile.WriteValue(driverID, comPortProfileName, comPort.ToString());
            }
        }

        /// <summary>
        /// Log helper function that takes formatted strings and arguments
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        /// <param name="args"></param>
        internal void LogMessage(string identifier, string message, params object[] args)
        {
            var msg = string.Format(message, args);
            tl.LogMessage(identifier, msg);
        }
        #endregion
    }
}
