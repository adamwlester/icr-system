using System;
using MNetCom;
using System.ComponentModel;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;
using System.IO;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Microsoft.VisualBasic.FileIO;
using System.Collections;
using MLApp;

namespace ICR_Run
{

    class ICR_Run
    {

        #region ---------DEBUG SETTINGS---------

        struct DB
        {
            // Run system test
            /*
                0: No test
                1: PID calibration
                2: Halt error test
                3: Simulated rat test
            */
            public double systemTest;
            // Debug matlab
            /*
                true: Dont break on errors
                false: Break on errors
            */
            public bool debugMat;
            // Breakpoint line for matlab debugging
            public int breakLine;
            // Print all blocked vt recs
            public bool printBlockedVt;
            // Print all sent vt recs
            public bool printSentVT;
            // Print feederrobot log
            public bool printRobLog;
            // Print cheetahdue log
            public bool printDueLog;

            // Constructor:
            public DB(
                double system_test,
                bool debug_mat,
                int break_line,
                bool print_blocked_vt,
                bool print_sent_vt,
                bool print_rob_log,
                bool print_due_log
                )
            {
                systemTest = system_test;
                debugMat = debug_mat;
                breakLine = break_line;
                printBlockedVt = print_blocked_vt;
                printSentVT = print_sent_vt;
                printRobLog = print_rob_log;
                printDueLog = print_due_log;
            }
        }
        private static DB db = new DB(
            system_test: 0,
            debug_mat: false,
            break_line: 6160,
            print_blocked_vt: false,
            print_sent_vt: false,
            print_rob_log: false,
            print_due_log: true
            );

        #endregion

        #region ---------TOP LEVEL VARS---------

        // Initialize FC to track program flow
        private static Flow_Control fc = new Flow_Control();

        // Create stop watch object
        private static Stopwatch sw_main = new Stopwatch();
        private static long t_sync = 0;

        // Initialize callback object
        private static MNetCom.MNC_VTCallback deligate_netComCallback;

        // Initialize serial port object for Xbee
        private static System.IO.Ports.SerialPort sp_Xbee = new System.IO.Ports.SerialPort();

        // Initialize serial port object for CheetahDue
        private static System.IO.Ports.SerialPort sp_cheetahDue = new System.IO.Ports.SerialPort();

        // Initialize MATLAB COM object
        private static MLApp.MLApp com_Matlab = new MLApp.MLApp();

        // Initialize ICR_GUI background worker
        private static BackgroundWorker bw_RunGUI = new BackgroundWorker();

        // Initialize MATLAB com background worker
        private static BackgroundWorker bw_MatCOM = new BackgroundWorker();

        // Create lock objects for safe threading
        static readonly object lock_printState = new object();
        static readonly object lock_sendData = new object();
        static readonly object lockRcvdCheck = new object();
        static readonly object lockDoneCheck = new object();
        private static VT_Blocker vtBlocker = new VT_Blocker();

        // Create logging objects
        private static DB_Logger robLogger = new DB_Logger();
        private static DB_Logger dueLogger = new DB_Logger();
        private static DB_Logger csLogger = new DB_Logger();

        // Directories
        private static string matStartDir = @"C:\Users\lester\MeDocuments\AppData\MATLABMO\Startup";
        private static string nlxRecDir = @"C:\CheetahData\Temp\0000-00-00_00-00-00"; // cheetah dir
        private static string logDir = @"C:\CheetahData\Temp\0000-00-00_00-00-00"; // log temp dir
        private static string robLogFi = @"FeederDue_Log.csv"; // log file for FeederDue
        private static string dueLogFi = @"CheetahDue_Log.csv"; // log file for CheetahDue
        private static string csLogFi = @"ICR_Run_Log.csv"; // log file for ICR_Run
        private static string matLogFi = @"ICR_GUI_Log.csv"; // log file for ICR_Run

        // Matlab to CS
        private static Com_Track m2c = new Com_Track(
            _lockRcvd: lockRcvdCheck,
            _lockDone: lockDoneCheck,
            _idList:
            new char[13]{ // prefix giving masage id
            'T', // system test command
            'S', // start session
            'M', // move to position
            'R', // run reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'N', // matlab not loaded
            'G', // matlab gui loaded
            'A', // connected to AC computer
            'F', // data saved
            'X', // confirm quit
            'C', // data saved
             },
            _idNow: 'N'
            );

        // CS to Matlab
        private static Com_Track c2m = new Com_Track(
            _lockRcvd: lockRcvdCheck,
            _lockDone: lockDoneCheck,
            _idList:
            new char[8] {
            'W', // sync time
            'J', // battery voltage
            'Z', // reward zone
            'V', // robot streaming
            'K', // robot in place
            'Y', // enable save
            'E', // enable exit
            'C', // confirm close
             }
        );

        // Robot to Matlab
        private static Com_Track r2m = new Com_Track(
            _lockRcvd: lockRcvdCheck,
            _lockDone: lockDoneCheck,
            _idList:
            new char[2] {
            'J', // battery voltage
            'Z', // reward zone
            }
        );

        // CS to Robot
        private static Com_Track c2r = new Com_Track(
            _lockRcvd: lockRcvdCheck,
            _lockDone: lockDoneCheck,
            _idList:
            new char[12] {
            '+', // Setup handshake
            'T', // system test command
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // run reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'P', // position data
            'V', // request stream status
            'L', // request log send/resend
             },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // Robot to CS
        private static Com_Track r2c = new Com_Track(
            _lockRcvd: lockRcvdCheck,
            _lockDone: lockDoneCheck,
            _idList:
            new char[14] {
            '+', // Setup handshake
            'T', // system test command
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // run reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'D', // execution done
            'V', // connected and streaming
            'L', // request log send/resend
            'J', // battery voltage
            'Z', // reward zone
             },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // Robot to Ard
        private static Com_Track r2a = new Com_Track(
            _lockRcvd: lockRcvdCheck,
            _lockDone: lockDoneCheck,
            _idList:
            new char[5] {
            'q', // quit/reset
	        'r', // reward
	        's', // sound cond [0, 1, 2]
	        'p', // pid mode [0, 1]
	        'b', // bull mode [0, 1]
            },
            _head: (byte)'{',
            _foot: (byte)'}'
        );

        // Robot to Ard
        private static Com_Track a2c = new Com_Track(
            _lockRcvd: new object(),
            _lockDone: new object(),
            _idList:
            new char[1] {
            ' '
            },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // General communication
        private static long dt_sendSent = 5; // (ms)
        private static long dt_sendRcvd = 1; // (ms)
        private static long resendTimeout = 500; // (ms)
        private static long acConnectTimeout = 30000; // (ms)
        private static long importLogTimeout = 15000; // (ms)
        private static int resendMax = 5;
        private static int droppedPacks = 0;

        // Position variables
        private static double X_CENT = 359.5553;
        private static double Y_CENT = 260.2418;
        private static double RADIUS = 179.4922;
        private static double feedDist = 66 * ((2 * Math.PI) / (140 * Math.PI));
        private static byte vtEnt = 0;
        private static float vtX = 0;
        private static float vtY = 0;
        private static float[,] vtRad = new float[2, 2];
        private static float vtCM;
        private static ulong vtStr = 0;
        private static int[,] vtTS = new int[2, 2];

        // Structure used to convert data types to byte
        [StructLayout(LayoutKind.Explicit, Pack = 1)]
        struct UnionHack
        {
            [FieldOffset(0)]
            public int i; // (int) 4 byte
            [FieldOffset(0)]
            public float f; // (float) 4 byte
            [FieldOffset(0)]
            public ushort s1; // (ushort) 2 byte
            [FieldOffset(2)]
            public ushort s2;
            [FieldOffset(0)]
            public char c1; // (char) 2 byte
            [FieldOffset(2)]
            public char c2;
            [FieldOffset(0)]
            public byte b1; // (byte) 1 byte
            [FieldOffset(1)]
            public byte b2;
            [FieldOffset(2)]
            public byte b3;
            [FieldOffset(3)]
            public byte b4;

            // Constructor:
            public UnionHack(int i, float f, ushort s, char c, byte b)
            {
                this.i = i;
                this.f = f;
                this.s1 = s;
                this.s2 = s;
                this.c1 = c;
                this.c2 = c;
                this.b1 = b;
                this.b2 = b;
                this.b3 = b;
                this.b4 = b;
            }
        }

        #endregion

        // MAIN
        static void Main()
        {

            #region --------- [MAIN] SETUP ---------
            LogEvent("[Main] RUNNING: Main...");

            // Start primary timer
            sw_main.Start();

            // Local vars
            bool pass;
            double move_to;
            string msg_str;

            // Setup and start CheetahDue serial
            sp_cheetahDue.ReadTimeout = 100;
            sp_cheetahDue.BaudRate = 57600;
            sp_cheetahDue.PortName = "COM14";
            // Create event handeler for incoming data
            sp_cheetahDue.DataReceived += DataReceived_CheetahDue;
            // Open serial port connection
            sp_cheetahDue.Open();
            LogEvent("[Main] FINISHED: CheetahDue Serial Port Open");

            // Setup and start Xbee serial
            sp_Xbee.ReadTimeout = 100;
            sp_Xbee.BaudRate = 57600;
            sp_Xbee.PortName = "COM92";
            // Create event handeler for incoming data
            sp_Xbee.DataReceived += DataReceived_Xbee;
            // Open serial port connection
            sp_Xbee.Open();
            LogEvent("[Main] FINISHED: Xbee Serial Port Open");

            // TEMP
            while (!sp_Xbee.IsOpen) ;
            // Send FeederDue message
            RepeatSendPack(id: '+');
            // Wait for sync confirmation from robot
            pass = WaitForR2C('+', timeout: 5000, do_abort: true);

            // Start importing robot log on seperate thread
            LogEvent("[Main] RUNNING: Robot Log Import/Store/Save...");
            new Thread(delegate ()
            {
                GetRobotLog();
            }).Start();

            // Wait for robot log save to complete
            LogEvent("[Main] RUNNING: Wait for Robot Log Import/Store/Save...");
            while (!robLogger.isLogFinished) ;

            // Check if complete log was imported
            if (robLogger.isLogComplete)
                msg_str = String.Format("[Main] FINISHED: Robot Log Import/Store/Save: logged={0} dropped={1} dt={2}ms", robLogger.cntLogged, robLogger.cntDropped, robLogger.logDT);
            else
                msg_str = String.Format("!!ERROR!! [Main] ABORTED: Robot Log Import/Store/Save: logged={0} dropped={1} dt={2}ms", robLogger.cntLogged, robLogger.cntDropped, robLogger.logDT);
            LogEvent(msg_str);

            // TEMP
            // Save CS log file
            csLogger.SaveLog(nlxRecDir, csLogFi);
            // Confirm log saved
            LogEvent("[Main] FINISHED: Save CS Log");

            // Initalize Matlab global vars
            com_Matlab.PutWorkspaceData("m2c_id", "global", 'N');
            com_Matlab.PutWorkspaceData("m2c_dat1", "global", -1.00);
            com_Matlab.PutWorkspaceData("m2c_dat2", "global", -1.00);
            com_Matlab.PutWorkspaceData("m2c_dat3", "global", -1.00);
            com_Matlab.PutWorkspaceData("m2c_flag", "global", false);
            com_Matlab.PutWorkspaceData("m2c_dir", "global", " ");
            // c2m vars
            for (int i = 0; i < c2m.idList.Length; i++)
            {
                com_Matlab.PutWorkspaceData(String.Format("c2m_{0}", c2m.idList[i]), "global", 0);
            }

            // Setup debugging
            if (db.systemTest != 0 || db.debugMat)
            {
                // Hide/show matlab app window
                com_Matlab.Visible = 1;

                // Set MATLAB break point
                if (db.debugMat)
                    SendMCOM(String.Format("dbstop in ICR_GUI at {0};", db.breakLine));

                // Start thread to pass simulation rat data
                if (db.systemTest == 4)
                {
                    new Thread(delegate ()
                    {
                        RelaySimRat();
                    }).Start();
                }
            }
            else com_Matlab.Visible = 0;

            // Setup ICR_GUI background worker
            bw_RunGUI.DoWork += DoWork_RunGUI;
            bw_RunGUI.RunWorkerCompleted += RunWorkerCompleted_RunGUI;
            LogEvent("[Main] RUNNING: RunGUI Worker...");
            // Start ICR_GUI worker
            bw_RunGUI.RunWorkerAsync();

            // Setup MATLAB com background worker
            bw_MatCOM.DoWork += DoWork_MatCOM;
            bw_MatCOM.ProgressChanged += ProgressChanged_MatCOM;
            bw_MatCOM.RunWorkerCompleted += RunWorkerCompleted_MatCOM;
            bw_MatCOM.WorkerReportsProgress = true;

            // Run setup handshake start
            LogEvent("[Main] RUNNING: Setup Handshake...");
            // Send CheetahDue message
            byte[] out_byte = new byte[1] { (byte)'+' };
            sp_cheetahDue.Write(out_byte, 0, 1);
            // Send FeederDue message
            new Thread(delegate ()
            {
                RepeatSendPack(id: '+', check_done: true);
            }).Start();
            // Wait for sync confirmation from robot
            pass = WaitForR2C('+', timeout: 5000, do_abort: true);
            if (pass)
            {
                // Store sync time
                t_sync = sw_main.ElapsedMilliseconds;
                LogEvent(String.Format("SET SYNC TIME: {0}ms", t_sync), t_sync);

                // Send to matlab 
                object result;
                com_Matlab.Feval("now", 1, out result);
                object[] res = result as object[];
                double mat_now = (double)res[0];
                double mat_sec = mat_now * 24 * 60 * 60;
                SendMCOM(String.Format("c2m_{0} = {1};", 'W', mat_sec));

                // Log/print success
                LogEvent("[Main] FINISHED: Setup Handshake");
            }
            else LogEvent("!!ERROR!! [Main] ABORTED: Setup Handshake");

            // Start Cheetah if it is not already running
            LogEvent("[Main] RUNNING: Cheetah Open...");
            while (!IsProcessOpen("Cheetah") && !fc.doAbort)
            {
                OpenCheetah("Cheetah.cfg");
            }
            if (!fc.doAbort) LogEvent("[Main] FINISHED: Cheetah Open");

            // Initilize deligate for VT callback
            MNetComClient com_netComClient = new MNetComClient();
            deligate_netComCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            com_netComClient.SetCallbackFunctionVT(deligate_netComCallback, new ICR_Run());

            // Set NetCom parameters
            var NETCOM_APP_ID = "ICR_Run"; // string displayed in Cheetah when connected
            var NETCOM_ACQ_ENT_1 = "VT1"; // aquisition entity to stream
            var NETCOM_ACQ_ENT_2 = "VT2"; // aquisition entity to stream
            var NETCOM_IP = "127.0.0.1"; // host computer IP

            // Start MatCOM worker
            LogEvent("[Main] RUNNING: MatCOM Worker...");
            // Crate argument vars for backround worker
            char id = ' ';
            double dat1 = 0;
            double dat2 = 0;
            double dat3 = 0;
            var bw_args = Tuple.Create(id, dat1, dat2, dat3);
            bw_MatCOM.RunWorkerAsync(bw_args);

            // Wait for ICR_GUI to connect to AC computer
            LogEvent("[Main] RUNNING: Wait for AC Connect...");
            pass = WaitForM2C('A', timeout: acConnectTimeout);
            if (pass) LogEvent("[Main] FINISHED: Wait for AC Connect");
            else
            {
                // Program timed out because matlab was hanging on connect
                LogEvent("!!ERROR!! [Main] ABORTED: Wait for AC Connect");
                fc.isMAThanging = true;
            }

            // Wait for ICR_GUI to load and connect to NLX
            LogEvent("[Main] RUNNING: Wait for ICR_GUI NLX Setup...");
            pass = WaitForM2C('G', do_abort: true);
            if (pass) LogEvent("[Main] FINISHED: Wait for ICR_GUI NLX Setup");
            else LogEvent("!!ERROR!! [Main] ABORTED: Wait for ICR_GUI NLX Setup");

            #endregion

            #region --------- [MAIN] RUN SESSION ---------

            // Setup and begin NetCom streaming
            if (!(com_netComClient.AreWeConnected()))
                fc.isNlxConnected = com_netComClient.ConnectToServer(NETCOM_IP);
            if (fc.isNlxConnected)
            {
                // Begin stream and set app name
                LogEvent("[Main] RUNNING: Nlx Stream...");
                com_netComClient.SetApplicationName(NETCOM_APP_ID);
                // rat vt
                com_netComClient.OpenStream(NETCOM_ACQ_ENT_1);
                // rob vt
                com_netComClient.OpenStream(NETCOM_ACQ_ENT_2);
                LogEvent("[Main] FINISHED: Nlx Stream");

                // Send streaming check request on seperate thread
                LogEvent("[Main] RUNNING: Confirm Robot Streaming...");
                new Thread(delegate ()
                {
                    RepeatSendPack(id: 'V', check_done: true);
                }).Start();
                // Wait for confirmation from robot
                pass = WaitForR2C('V', timeout: 5000, do_abort: true);
                if (pass)
                {
                    // Send confirm stream to Matlbab
                    SendMCOM(String.Format("c2m_{0} = 1;", 'V'));
                    LogEvent("[Main] FINISHED: Confirm Robot Streaming");
                }
                else LogEvent("!!ERROR!! [Main] ABORTED: Confirm Robot Streaming");

                // Wait for setup command confirmation
                LogEvent("[Main] RUNNING: Confirm Setup");
                pass = WaitForM2C('S', do_abort: true);
                if (pass)
                    pass = WaitForR2C('S', do_abort: true);
                if (pass) LogEvent("[Main] FINISHED: Confirm Setup");
                else LogEvent("!!ERROR!! [Main] ABORTED: Confirm Setup");

                // Wait for initial move to command to complete
                LogEvent("[Main] RUNNING: MoveTo Start...");
                pass = WaitForM2C('M', do_abort: true);
                if (pass)
                    pass = WaitForR2C('M', do_abort: true);
                if (pass)
                {
                    // Send confirm robot in place to Matlbab
                    SendMCOM(String.Format("c2m_{0} = 1;", 'K'));
                    fc.isMovedToStart = true;
                    LogEvent("[Main] FINISHED: MoveTo Start");
                }
                else LogEvent("!!ERROR!! [Main] ABORTED: MoveTo Start");

                // Main holding loop
                LogEvent("[Main] RUNNING: Main Loop...");
                // Stay in loop till rat is out
                while (
                    com_netComClient.AreWeConnected() &&
                    !fc.isRatOut &&
                    !fc.doAbort
                    ) ;
                if (!fc.doAbort) LogEvent("[Main] FINISHED: Main Loop");
                else LogEvent("!!ERROR!! [Main] ABORTED: Main Loop");

                // Wait for reply on last sent packet
                LogEvent("[Main] RUNNING: Wait for Last Pack...");
                pass = WaitForR2C(c2r.idList, timeout: 5000);
                if (pass) LogEvent("[Main] FINISHED: Wait for Last Pack");
                else LogEvent("!!ERROR!! [Main] ABORTED: Wait for Last Pack");

            }
            // Failed to connect to NetCom
            else
            {
                fc.doAbort = true;
                LogEvent("!!ERROR!! [Main] FAILED TO CONNECT TO NETCOM");
            }

            #endregion

            #region --------- [MAIN] EXIT ---------

            // Close down everything
            LogEvent("[Main] EXITING...");

            // MoveTo defualt pos
            if (fc.isMovedToStart)
            {
                LogEvent("[Main] RUNNING: MoveTo South...");
                move_to = CalcMove(4.7124 - feedDist);

                // Wait for robot to finish other commands before sending
                Thread.Sleep(100);

                // Send move command on seperate thread and wait for done reply
                new Thread(delegate ()
                {
                    RepeatSendPack(id: 'M', dat1: move_to, check_done: true);
                }).Start();
                // Wait for confirmation from robot
                pass = WaitForR2C('M', timeout: 10000);
                if (pass) LogEvent("[Main] FINISHED: MoveTo South");
                else LogEvent("!!ERROR!! [Main] ABORTED: MoveTo South");

            }

            // Wait 1 second to see move to plot
            Thread.Sleep(1000);

            // Shut down NetCom
            if (IsProcessOpen("Cheetah"))
            {
                LogEvent("[Main] RUNNING: NetCom Close...");
                //// Stop recording aquisition
                string reply = " ";
                com_netComClient.SendCommand("-StopRecording", ref reply);
                com_netComClient.SendCommand("-StopAcquisition", ref reply);

                // Close NetCom sreams
                com_netComClient.CloseStream(NETCOM_ACQ_ENT_1);
                com_netComClient.CloseStream(NETCOM_ACQ_ENT_2);

                // Disconect from NetCom
                do
                {
                    com_netComClient.DisconnectFromServer();
                }
                while (com_netComClient.AreWeConnected() && !fc.doAbort);
                if (!com_netComClient.AreWeConnected())
                {
                    fc.isNlxConnected = false;
                    LogEvent("[Main] FINISHED: NetCom Close");
                }
                else LogEvent("!!ERROR!! [Main] ABBORTED: NetCom Close");
            }

            // Enable ICR_GUI save button
            if (!fc.doAbort)
            {
                SendMCOM(String.Format("c2m_{0} = 1;", 'Y'));
                LogEvent("[Main] FINISHED: Save Enabled");
            }
            else LogEvent("!!ERROR!! [Main] ABORTED: Save Enabled");

            // Wait for last packet
            LogEvent("[Main] RUNNING: Wait for Last Pack...");
            pass = WaitForR2C(c2r.idList, timeout: 5000);
            if (pass) LogEvent("[Main] FINISHED: Wait for Last Pack");
            else LogEvent("!!ERROR!! [Main] ABORTED: Wait for Last Pack");

            // Start importing robot log on seperate thread
            LogEvent("[Main] RUNNING: Import Robot Log...");
            new Thread(delegate ()
            {
                GetRobotLog();
            }).Start();

            // Wait for robot log save to complete
            LogEvent("[Main] RUNNING: Wait for Robot Log to Finish Importing...");
            while (!robLogger.isLogFinished) ;

            // Check if complete log was imported
            if (robLogger.cntLogged > 0)
            {
                if (robLogger.isLogComplete)
                    msg_str = String.Format("[Main] FINISHED: Logging {0} events in {2}ms", robLogger.cntLogged, robLogger.logDT);
                else
                    msg_str = String.Format("!!ERROR!! [Main] PARIALLY FINISHED: Logging {0} events in {2}ms", robLogger.cntLogged, robLogger.logDT);
                LogEvent(msg_str);
            }
            else
                LogEvent("!!ERROR!! [Main] ABORTED: Import Robot Log");

            // Wait for save complete
            LogEvent("[Main] RUNNING: Wait for ICR_GUI to Save...");
            while (!fc.isSesSaved && !fc.doAbort) ;
            if (fc.isSesSaved)
            {
                LogEvent("[Main] FINISHED: Wait for ICR_GUI to Save");

                // Get NLX dir
                dynamic nlx_rec_dir = com_Matlab.GetVariable("m2c_dir", "global");
                nlxRecDir = (string)nlx_rec_dir;

                // Confirm log saved
                msg_str = String.Format("SET RECORDING DIR TO \"{0}\"", nlxRecDir);
                LogEvent(msg_str);

            }
            else LogEvent("!!ERROR!! [Main] ABORTED: Wait for ICR_GUI to Save");

            // Wait for quit command
            LogEvent("[Main] RUNNING: Wait for ICR_GUI Quit command...");
            if (!fc.isGUIquit)
                pass = WaitForM2C('X', do_abort: true);
            else pass = true;
            if (pass)
                LogEvent("[Main] FINISHED: Wait for ICR_GUI Quit command");
            else
                LogEvent("!!ERROR!! [Main] ABORTED: Wait for ICR_GUI Quit command");

            // Send command for arduino to quit on seperate thread
            new Thread(delegate ()
            {
                RepeatSendPack(id: 'Q');
            }).Start();
            // Wait for quit confirmation from robot for fixed period of time
            LogEvent("[Main] RUN: Confirm Robot Quit...");
            pass = WaitForR2C('Q', timeout: 5000);
            if (pass) LogEvent("[Main] FINISHED: Confirm Robot Quit");
            else LogEvent("!!ERROR!! [Main] ABORTED: Confirm Robot Quit");
            fc.isRobStreaming = false;

            // Hold for errors
            if (db.systemTest != 0 || db.debugMat || fc.doAbort)
            {
                Thread.Sleep(1000);
                LogEvent("PRESS ANY KEY TO EXIT");
                Console.ReadKey();
            }

            // Send command for ICR_GUI to exit
            SendMCOM(String.Format("c2m_{0} = 1;", 'E'));
            LogEvent("[Main] FINISHED: Tell ICR_GUI to Close");

            // Wait for GUI to close
            LogEvent("[Main] RUNNING: Confirm ICR_GUI Closed...");
            pass = WaitForM2C('C');
            if (pass) LogEvent("[Main] FINISHED: Confirm ICR_GUI Closed");
            else LogEvent("!!ERROR!! [Main] ABORTED: Confirm ICR_GUI Closed");
            // Tell matlab close confirmation recieved
            SendMCOM(String.Format("c2m_{0} = 1;", 'C'));
            LogEvent("[Main] FINISHED: Tell ICR_GUI Close Confirmed");

            // Set exit flag to exit all threads
            fc.doExit = true;

            // Wait for threads to close down
            Thread.Sleep(100);

            // Dispose of workers
            //bw_RunGUI.CancelAsync();
            bw_RunGUI.Dispose();
            //bw_MatCOM.CancelAsync();
            bw_MatCOM.Dispose();

            // Clear all MatCOM vars
            SendMCOM("clearvars - global;");
            SendMCOM("clearvars;");
            SendMCOM("close all;");
            Thread.Sleep(100);

            // Quit MatCOM
            if (!fc.isMAThanging)
            {
                com_Matlab.Quit();
            }

            // Kill that mother fucker!
            else
            {
                KillMatlab();
                LogEvent("[MAIN] !!ERROR!! HAD TO KILL MATLAB");
            }
            LogEvent("[Main] FINISHED: Close MatCOM COM");

            // Save due log file
            if (dueLogger.cntLogged > 0)
            {
                dueLogger.SaveLog(logDir, dueLogFi);
                LogEvent("[MAIN] FINISHED: Save CheetahDue Log");
            }
            else
                LogEvent("!!ERROR!! [MAIN] ABORTED: Save CheetahDue Log");

            // Save CS log file
            csLogger.SaveLog(nlxRecDir, csLogFi);
            // Confirm log saved
            LogEvent("[Main] FINISHED: Save CS Log");

            // Copy log files to rat specific dir
            if (nlxRecDir != logDir)
            {
                // Confirm logs copied saved
                msg_str = String.Format("COPPIED LOG FILES TO \"{0}\"", nlxRecDir);
                LogEvent(msg_str);
            }

            // Confirm end of program
            LogEvent("[MAIN] FINISHED ALL");

            // Give time for everything to close
            Thread.Sleep(1000);

            return;

            #endregion

        }

        #region ---------COMMUNICATION---------

        // SEND PACK DATA REPEATEDLY TILL RECIEVED CONFIRMED
        public static bool RepeatSendPack(int send_max = 0, char id = ' ', double dat1 = double.NaN, double dat2 = double.NaN, double dat3 = double.NaN, bool do_comf = true, bool check_done = false)
        {
            bool pass_rcvd = false;
            long t_timeout = sw_main.ElapsedMilliseconds + resendTimeout;
            int send_count = 1;
            ushort pack = 0;

            // Specify max send attempts
            send_max = send_max == 0 ? resendMax : send_max;

            // Get new packet number
            c2r.packCnt++;
            pack = c2r.packCnt;

            // Update c2r check flags
            c2r.Update(id: id, pack: pack, t: sw_main.ElapsedMilliseconds);
            c2r.doRcvdCheck[c2r.ID_Ind(id)] = true;
            c2r.doDoneCheck[c2r.ID_Ind(id)] = check_done ? true : false;

            // Send new data with new packet number
            SendPack(id, dat1, dat2, dat3, pack, do_comf);

            // Keep checking mesage was recieved
            if (do_comf)
            {
                while (
                    !pass_rcvd &&
                    send_count < resendMax &&
                    fc.ContinueSerial()
                    )
                {

                    // Search for matching command packet
                    if (r2c.packList[r2c.ID_Ind(id)] == pack)
                    {
                        // Message recieved
                        pass_rcvd = true;
                        continue;
                    }

                    // Need to resend
                    if (
                        !pass_rcvd &&
                        sw_main.ElapsedMilliseconds > t_timeout
                        )
                    {
                        SendPack(id, dat1, dat2, dat2, pack, do_comf);
                        t_timeout = sw_main.ElapsedMilliseconds + resendTimeout;
                        send_count++;
                    }

                }
            }

            // Check if streaming has failed
            if (send_count >= resendMax)
                fc.isRobStreaming = false;

            // forced quit
            return pass_rcvd;
        }

        // SEND PACK DATA
        public static void SendPack(char id, double dat1, double dat2, double dat3, ushort pack, bool do_conf)
        {
            /* 
            SEND DATA TO ROBOT 
            FORMAT: head, id, data, packet_number, do_confirm, footer
            EXAMPLE: ASCII {<,L,r,ÿ,ÿ,\0,>} DEC {60,76,1,255,255,0,60}
            */
            lock (lock_sendData)
            {

                // Block vt sending
                vtBlocker.Block(id);

                // Local vars
                UnionHack u = new UnionHack(0, 0, 0, '0', 0);
                byte[] msg_id = new byte[1];
                byte[] msg_data = null;
                byte[] msg_pack = new byte[2];
                byte[] msg_conf = new byte[1];
                int n_dat_bytes = 0;
                long t_send;
                bool buff_ready;
                bool do_loop;

                // Wait for next safe send time
                do
                {

                    // Delay send time till x ms after last send or rcvd
                    t_send = c2r.t_now > r2c.t_now ? c2r.t_now + dt_sendSent : r2c.t_now + dt_sendRcvd;

                    // Make sure outbut and input buffer have enough space
                    buff_ready = sp_Xbee.BytesToRead == 0 && sp_Xbee.BytesToWrite == 0;

                    // Check if loop should continue
                    do_loop =
                        (sw_main.ElapsedMilliseconds < t_send || !buff_ready) &&
                        fc.ContinueSerial();

                } while (do_loop);

                // Store pack number
                u.s1 = pack;
                msg_pack[0] = u.b1;
                msg_pack[1] = u.b2;

                // Store ID
                u.c1 = id;
                msg_id[0] = u.b1;

                // Send system test command
                if (id == 'T')
                {
                    n_dat_bytes = 2;
                    msg_data = new byte[n_dat_bytes];
                    // Add test id
                    msg_data[0] = (byte)dat1;
                    // Add test parameter
                    msg_data[1] = (byte)dat2;
                }

                // Send setup data
                if (id == 'S')
                {
                    n_dat_bytes = 2;
                    msg_data = new byte[n_dat_bytes];
                    // Add session cond
                    msg_data[0] = (byte)dat1;
                    // Add sound cond
                    msg_data[1] = (byte)dat2;
                }

                // Send MoveTo data
                else if (id == 'M')
                {
                    n_dat_bytes = 4;
                    msg_data = new byte[n_dat_bytes];
                    // Add move pos
                    u.f = (float)dat1;
                    msg_data[0] = u.b1;
                    msg_data[1] = u.b2;
                    msg_data[2] = u.b3;
                    msg_data[3] = u.b4;
                }

                // Send Reward data
                else if (id == 'R')
                {
                    n_dat_bytes = 6;
                    msg_data = new byte[n_dat_bytes];
                    // Add rew pos
                    u.f = (float)dat1;
                    msg_data[0] = u.b1;
                    msg_data[1] = u.b2;
                    msg_data[2] = u.b3;
                    msg_data[3] = u.b4;
                    // Add zone ind
                    msg_data[4] = (byte)dat2;
                    // Add reward delay
                    msg_data[5] = (byte)dat3;
                }

                // Send halt motor data
                if (id == 'H')
                {
                    n_dat_bytes = 1;
                    msg_data = new byte[n_dat_bytes];
                    // Add halt state
                    msg_data[0] = (byte)dat1;
                }

                // Send bulldoze rat data
                if (id == 'B')
                {
                    n_dat_bytes = 2;
                    msg_data = new byte[n_dat_bytes];
                    // Add bull del
                    msg_data[0] = (byte)dat1;
                    // Add bull speed
                    msg_data[1] = (byte)dat2;
                }

                // Send rat in/out
                if (id == 'I')
                {
                    n_dat_bytes = 1;
                    msg_data = new byte[n_dat_bytes];
                    // Add session cond
                    msg_data[0] = (byte)dat1;
                }

                // Send log request
                if (id == 'L')
                {
                    n_dat_bytes = 1;
                    msg_data = new byte[n_dat_bytes];
                    // Add send/resend request
                    msg_data[0] = (byte)dat1;
                }

                // Send pos data
                else if (id == 'P')
                {
                    n_dat_bytes = 9;
                    msg_data = new byte[n_dat_bytes];
                    // Add vtEnt byte
                    msg_data[0] = vtEnt;
                    // Add vtTS int 
                    u.i = vtTS[vtEnt, 1];
                    msg_data[1] = u.b1;
                    msg_data[2] = u.b2;
                    msg_data[3] = u.b3;
                    msg_data[4] = u.b4;
                    // Add vtCM float 
                    u.f = vtCM;
                    msg_data[5] = u.b1;
                    msg_data[6] = u.b2;
                    msg_data[7] = u.b3;
                    msg_data[8] = u.b4;
                }

                // Store do do_conf 
                msg_conf[0] = do_conf ? (byte)1 : (byte)0;

                // Concatinate header and footer
                byte[] msgByteArr = new byte[
                    c2r.head.Length +    // header
                    msg_id.Length +          // id
                    n_dat_bytes +             // data
                    msg_pack.Length + // packet num
                    msg_conf.Length + // do_conf
                    c2r.foot.Length      // footer
                    ];
                // add header
                c2r.head.CopyTo(msgByteArr, 0);
                // add id
                msg_id.CopyTo(msgByteArr, c2r.head.Length);
                // add data
                if (n_dat_bytes > 0)
                {
                    msg_data.CopyTo(msgByteArr, c2r.head.Length + msg_id.Length);
                }
                // add packet number
                msg_pack.CopyTo(msgByteArr, c2r.head.Length + msg_id.Length + n_dat_bytes);
                // add do_conf
                msg_conf.CopyTo(msgByteArr, c2r.head.Length + msg_id.Length + n_dat_bytes + msg_pack.Length);
                // add footer
                c2r.foot.CopyTo(msgByteArr, c2r.head.Length + msg_id.Length + n_dat_bytes + msg_pack.Length + msg_conf.Length);

                // Send to arduino
                if (sp_Xbee.IsOpen) sp_Xbee.Write(msgByteArr, 0, msgByteArr.Length);

                // Print sent data
                string msg_str;

                // Print sent mesage packet
                if (Double.IsNaN(dat1))
                    msg_str = String.Format("   sent_c2r: id={0} pack={1} do_conf={2}", id, pack, do_conf ? "true" : "false");
                else if (Double.IsNaN(dat2))
                    msg_str = String.Format("   sent_c2r: id={0} dat1={1:0.00} pack={2} do_conf={3}", id, dat1, pack, do_conf ? "true" : "false");
                else if (Double.IsNaN(dat3))
                    msg_str = String.Format("   sent_c2r: id={0} dat1={1:0.00} dat2={2:0.00} pack={3} do_conf={4}", id, dat1, dat2, pack, do_conf ? "true" : "false");
                else
                    msg_str = String.Format("   sent_c2r: id={0} dat1={1:0.00} dat2={2:0.00} dat3={3:0.00} pack={4} do_conf={5}", id, dat1, dat2, dat3, pack, do_conf ? "true" : "false");

                // Check if sending pos data
                if (id != 'P')
                {
                    LogEvent(msg_str, c2r.t_now, c2r.t_last);
                }
                else if (db.printSentVT)
                {
                    LogEvent(msg_str, c2r.t_now, c2r.t_last);
                }

                // Unlock vt sending
                vtBlocker.Unblock(id);

            }

        }

        // SEND/STORE DATA FOR MATLAB
        public static void SendMCOM(string msg)
        {
            if (fc.ContinueMatCom())
            {
                // Store value
                com_Matlab.Execute(msg);

                // Print value
                LogEvent("   sent_c2m: dat=\"" + msg + "\"");
            }
        }

        // WAIT FOR M2C CONFIRMATION
        public static bool WaitForM2C(char id, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : sw_main.ElapsedMilliseconds + timeout;
            string str = " ";
            bool pass = false;
            bool do_loop = true;
            bool first_loop = true;

            do
            {

                // Check if pack recieved
                if (m2c.Recent(id, sw_main.ElapsedMilliseconds))
                    pass = true;
                else
                {
                    pass = false;
                    // Print info
                    if (first_loop)
                    {
                        str = String.Format("   waiting for m2c: id={0}...", id);
                        LogEvent(str);
                    }

                }

                // Set flag
                first_loop = false;

                // Check if ready to exit
                if (pass == true)
                {
                    do_loop = false;
                }
                else if (
                    (do_abort && fc.doAbort) ||
                    !fc.ContinueMatCom() ||
                    (sw_main.ElapsedMilliseconds > t_timeout)
                    )
                {
                    pass = false;
                    do_loop = false;
                    fc.doAbort = true;
                    break;
                }

                // Pause thread
                Thread.Sleep(100);

            } while (do_loop);
            return pass;
        }

        // WAIT FOR R2C CONFIRMATION
        public static bool WaitForR2C(char id, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            char[] id_arr = { id };
            return WaitForR2C(id_arr, do_abort, timeout);
        }
        public static bool WaitForR2C(char[] id_arr, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : sw_main.ElapsedMilliseconds + timeout;
            char id = ' ';
            bool pass = false;
            bool do_check_send = false;
            bool do_loop = true;
            bool first_loop = true;
            bool is_sent = false;
            bool is_rcvd = false;
            bool is_done = false;

            // Check for sent if only one id arg
            do_check_send = id_arr.Length == 1 ? true : false;

            // Loop through each id
            for (int i = 0; i < id_arr.Length; i++)
            {
                // Current id
                id = id_arr[i];
                do_loop = true;
                is_sent = false;
                first_loop = true;

                // Wait for confirmation
                do
                {
                    // Check if sent within past second
                    if (do_check_send)
                        is_sent = !is_sent ?
                            c2r.Recent(id, sw_main.ElapsedMilliseconds) ||
                            r2c.Recent(id, sw_main.ElapsedMilliseconds) :
                            is_sent;
                    else
                        is_sent = true;

                    // Check if recieved confirm and/or done
                    is_rcvd = !is_rcvd ? is_sent && !c2r.doRcvdCheck[c2r.ID_Ind(id)] : is_rcvd;
                    is_done = !c2r.doDoneCheck[c2r.ID_Ind(id)];

                    // Check if pack recieved
                    if (!is_rcvd || !is_done)
                    {
                        // Print info
                        if (first_loop)
                        {
                            string str = String.Format("   waiting for r2c: id={0} pack={1}...", id, c2r.packList[c2r.ID_Ind(id)]);
                            LogEvent(str);
                        }
                        first_loop = false;
                    }
                    // Done with this id
                    else
                        pass = true;

                    // Check if ready to exit
                    if (pass == true)
                    {
                        do_loop = false;
                    }
                    // Need to abort
                    else if (
                        (do_abort && fc.doAbort) ||
                        !fc.ContinueSerial() ||
                        sw_main.ElapsedMilliseconds > t_timeout
                        )
                    {
                        pass = false;
                        do_loop = false;
                        fc.doAbort = true;
                        return false;
                    }

                    // Pause thread
                    Thread.Sleep(100);

                } while (do_loop);

            }

            // All confirmed
            return true;
        }

        // PARSE RECIEVED XBEE DATA 
        public static void DataReceived_Xbee(object sender, SerialDataReceivedEventArgs e)
        {
            /* 
            RECIEVE DATA FROM FEEDERDUE 
            FORMAT: head, id, return_confirm, data, packet_number, footer
            */

            // Bail if processing robot log
            if (robLogger.isLogging)
                return;

            // Loop till all data read out
            while (
                sp_Xbee.BytesToRead > 0 &&
                    fc.ContinueSerial()
                    )
            {

                // Local vars
                UnionHack u = new UnionHack(0, 0, 0, '0', 0);
                bool head_found = false;
                bool id_found = false;
                bool foot_found = false;
                bool for_ard = false;
                bool do_dump = false;
                byte[] head_bytes = new byte[1];
                byte[] id_bytes = new byte[1];
                byte[] conf_bytes = new byte[1];
                byte[] dat_bytes = new byte[1];
                byte[] pack_bytes = new byte[2];
                byte[] foot_bytes = new byte[1];
                char head = ' ';
                char id = ' ';
                byte dat = 0;
                ushort pack = 0;
                bool do_conf = false;
                char foot = ' ';
                string msg_str = " ";

                // Dump till header found
                while (
                    !head_found && sp_Xbee.BytesToRead > 0 &&
                    fc.ContinueSerial()
                    )
                {
                    sp_Xbee.Read(head_bytes, 0, 1);
                    // Get header
                    u.b1 = head_bytes[0];
                    u.b2 = 0; // C# chars are 2 bytes
                    head = u.c1;
                    if (head == r2c.head[0])
                    {
                        head_found = true;
                    }
                    else if (head == r2a.head[0])
                    {
                        for_ard = true;
                        do_dump = true;
                        break;
                    }
                }

                // Find id and check message is intended for CS
                if (head_found)
                {
                    if (XbeeBuffReady(1))
                    {
                        sp_Xbee.Read(id_bytes, 0, 1);
                        // Get id
                        u.b1 = id_bytes[0];
                        u.b2 = 0;
                        id = u.c1;
                        // Check for match
                        for (int i = 0; i < r2c.idList.Length; i++)
                        {
                            if (id == r2c.idList[i])
                            {
                                id_found = true;
                            }
                        }
                    }
                }

                // Check if this is a log packet
                if (head_found && id_found)
                {

                    // Get data
                    if (XbeeBuffReady(1))
                    {
                        // Read in data
                        sp_Xbee.Read(dat_bytes, 0, 1);
                        dat = dat_bytes[0];
                    }

                    // Get packet number
                    if (XbeeBuffReady(2))
                    {

                        // Read in data
                        sp_Xbee.Read(pack_bytes, 0, 2);
                        u.b1 = pack_bytes[0];
                        u.b2 = pack_bytes[1];
                        pack = u.s1;
                    }

                    // Get do confirm byte
                    if (XbeeBuffReady(1))
                    {
                        sp_Xbee.Read(conf_bytes, 0, 1);
                        // Get bool
                        do_conf = conf_bytes[0] == 1 ? true : false;
                    }

                    // Find footer
                    if (XbeeBuffReady(1))
                    {
                        // Read in data
                        sp_Xbee.Read(foot_bytes, 0, 1);

                        // Check footer
                        u.b1 = foot_bytes[0];
                        u.b2 = 0;
                        foot = u.c1;
                        if (foot == r2c.foot[0])
                        {
                            foot_found = true;
                        }
                    }

                    // Store packet data
                    if (foot_found)
                    {
                        // Update flags
                        r2c.Update(id: id, pack: pack, t: sw_main.ElapsedMilliseconds);

                        // print data recieved
                        msg_str = String.Format("   rcvd_r2c: id={0} dat={1} pack={2} do_conf={3}", id, dat, pack, do_conf ? "true" : "false");
                        LogEvent(msg_str, r2c.t_now, r2c.t_last);

                        // Check if this is a recieve confirmation
                        if (c2r.ID_Ind(id) != -1)
                        {
                            c2r.doRcvdCheck[c2r.ID_Ind(id)] = false;
                        }
                        else
                        {
                            // Set done confirm flag
                            if (id == 'D')
                            {
                                c2r.doDoneCheck[c2r.PackInd(pack)] = false;
                            }
                            // Send recieve confirmation
                            if (do_conf)
                                RepeatSendPack(send_max: 1, id: id, dat1: dat, do_comf: false);
                        }

                        // Check if data should be relayed to Matlab
                        if (r2m.ID_Ind(id) != -1)
                        {
                            // Update matlab var
                            SendMCOM(String.Format("c2m_{0} = {1};", id, (double)dat));
                        }
                    }
                    else
                    {
                        // Set dump flag 
                        do_dump = true;
                    }
                }

                // Dump incomplete packets
                if (do_dump)
                {
                    if (for_ard)
                    {
                        // Dump till header found
                        while (
                            !foot_found && sp_Xbee.BytesToRead > 0 &&
                            fc.ContinueSerial()
                            )
                        {
                            sp_Xbee.Read(foot_bytes, 0, 1);
                            // Get footer
                            u.b1 = foot_bytes[0];
                            u.b2 = 0; // C# chars are 2 bytes
                            foot = u.c1;
                            if (foot == r2a.foot[0])
                            {
                                foot_found = true;
                            }
                        }
                        LogEvent("   rcvd: cheeta due packet");
                    }
                    else
                    {
                        // Add to count
                        droppedPacks++;

                        // Print packet info
                        msg_str = String.Format("!!ERROR!! R2C PACK LOST: tot={0} tx={1} rx={2} head={3} id={4} dat={5} pack={6} do_conf={7} foot={8}",
                            droppedPacks, sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, head, id, dat, pack, do_conf, foot);

                        // Print
                        LogEvent(msg_str);

                        // Wait for buffer to refil
                        XbeeBuffReady(6);
                    }
                    // dump input buffer
                    //sp_Xbee.DiscardInBuffer();

                }

                // Change streaming status
                if (!do_dump && foot_found && !fc.isRobStreaming)
                {
                    fc.isRobStreaming = true;
                }


            }

        }

        // PARSE RECIEVED CHEETAHDUE DATA 
        public static void DataReceived_CheetahDue(object sender, SerialDataReceivedEventArgs e)
        {
            // Loop till all data read out
            while (
                fc.isHandshook &&
                sp_cheetahDue.BytesToRead > 0 &&
                    fc.ContinueSerial()
                    )
            {
                // Local vars
                UnionHack u = new UnionHack(0, 0, 0, '0', 0);
                bool head_found = false;
                bool foot_found = false;
                bool do_dump = false;
                byte[] head_bytes = new byte[1];
                byte[] id_bytes = new byte[1];
                byte[] conf_bytes = new byte[1];
                byte[] dat_bytes = new byte[1];
                byte[] pack_bytes = new byte[2];
                byte[] foot_bytes = new byte[1];
                byte[] chksum_bytes = new byte[1];
                char head = ' ';
                ushort chksum = 0;
                char foot = ' ';
                string log_str = " ";

                // Dump till header found
                while (
                    !head_found && sp_cheetahDue.BytesToRead > 0 &&
                    fc.ContinueSerial()
                    )
                {
                    sp_cheetahDue.Read(head_bytes, 0, 1);
                    // Get header
                    u.b1 = head_bytes[0];
                    u.b2 = 0; // C# chars are 2 bytes
                    head = u.c1;
                    if (head == a2c.head[0])
                    {
                        head_found = true;
                    }
                }

                // Find id and check message is intended for CS
                if (head_found)
                {
                    // Get check sum
                    if (DueBuffReady(1))
                    {
                        // Read in data
                        sp_cheetahDue.Read(chksum_bytes, 0, 1);
                        chksum = chksum_bytes[0];
                    }

                    // Get complete message
                    byte[] log_bytes = new byte[chksum];
                    if (DueBuffReady(chksum))
                    {
                        // Read in all data
                        sp_cheetahDue.Read(log_bytes, 0, chksum);

                    }
                    // Convert to string
                    log_str = System.Text.Encoding.UTF8.GetString(log_bytes);

                    // Find footer
                    if (DueBuffReady(1))
                    {
                        // Read in data
                        sp_cheetahDue.Read(foot_bytes, 0, 1);

                        // Check footer
                        u.b1 = foot_bytes[0];
                        u.b2 = 0;
                        foot = u.c1;
                        if (foot == a2c.foot[0])
                        {
                            foot_found = true;
                        }
                    }

                    if (foot_found)
                    {
                        // Update list
                        dueLogger.UpdateList(log_str);

                        // print data recieved
                        if (db.printDueLog)
                        {
                            // Update list
                            dueLogger.UpdateList(log_str);

                            // Print
                            if (db.printDueLog)
                                LogEvent(String.Format("   log_a2c[{0}]: message=\"{1}\" chksum={2}", dueLogger.cntLogged, log_str, chksum));
                        }
                    }
                    else
                    {
                        // Set dump flag
                        do_dump = true;
                    }

                    // Dump incomplete packets
                    if (do_dump)
                    {
                        // Print
                        LogEvent(String.Format("!!ERROR!! [DataReceived_CheetahDue] Lost Log: tot={0} tx={1} rx={2} head={3} log_str={4} chksum={5} foot={6}",
                           droppedPacks, sp_cheetahDue.BytesToWrite, sp_cheetahDue.BytesToRead, head, log_str, chksum, foot));
                    }
                }
            }
        }

        // WAIT FOR XBEE BUFFER TO FILL
        public static bool XbeeBuffReady(int min_byte, long wait_max = 1000)
        {
            // Local vars
            long t_timeout = sw_main.ElapsedMilliseconds + wait_max;
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_Xbee.BytesToRead < min_byte &&
                sw_main.ElapsedMilliseconds <= t_timeout &&
                fc.ContinueSerial()
                ) ;

            // Check if buff filled
            pass = sp_Xbee.BytesToRead >= min_byte ? true : false;

            return pass;
        }

        // WAIT FOR CHEETAHDUE BUFFER TO FILL
        public static bool DueBuffReady(int min_byte, long wait_max = 1000)
        {
            // Local vars
            long t_timeout = sw_main.ElapsedMilliseconds + wait_max;
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_cheetahDue.BytesToRead < min_byte &&
                sw_main.ElapsedMilliseconds <= t_timeout &&
                fc.ContinueSerial()
                ) ;

            // Check if buff filled
            pass = sp_cheetahDue.BytesToRead >= min_byte ? true : false;

            return pass;
        }

        // RETRIEVE FEEDERDUE LOG
        public static void GetRobotLog()
        {
            // Local vars
            long t_start = sw_main.ElapsedMilliseconds;
            long t_timeout = t_start + importLogTimeout;
            long t_read_last = 0;
            bool do_timeout = false;
            ushort conf_pack = 0;
            int msg_lng = 0;
            char[] c_arr = new char[3] { '\0', '\0', '\0' };

            // Send log request
            RepeatSendPack(id: 'L', dat1: 1, do_comf: true, check_done: true);
            while (conf_pack == 0)
                conf_pack = c2r.packList[c2r.ID_Ind('L')];

            // Increase timeout
            sp_Xbee.ReadTimeout = 10;

            // Prevent xBee event handeler from running
            robLogger.isLogging = true;

            // Read stream till ">>>" string
            int read_ind = 0;
            char[] in_arr = new char[1000000];
            while (
                    !do_timeout &&
                    fc.isRobStreaming &&
                    conf_pack != r2c.packList[r2c.ID_Ind('D')])
            {
                // Get next byte
                if (sp_Xbee.BytesToRead > 0)
                {
                    // Get next char
                    c_arr[0] = c_arr[1];
                    c_arr[1] = c_arr[2];
                    c_arr[2] = (char)sp_Xbee.ReadByte();
                    in_arr[read_ind] = c_arr[2];

                    // Check for end
                    if (
                        c_arr[0] == '>' &&
                        c_arr[1] == '>' &&
                        c_arr[2] == '>'
                    )
                    {
                        break;
                    }

                    // Itterate
                    read_ind++;

                    // Check for timeout
                    if (sw_main.ElapsedMilliseconds > t_timeout)
                    {
                        if (sw_main.ElapsedMilliseconds - t_read_last > 1000)
                        {
                            long dt_run = sw_main.ElapsedMilliseconds - t_start;
                            long dt_read = t_read_last == 0 ? 0 : sw_main.ElapsedMilliseconds - t_read_last;
                            do_timeout = true;
                            LogEvent(String.Format("!!ERROR!! [GetRobotLog] Read Timedout: dt_read={0}ms dt_run={1}ms", dt_read, dt_run));
                        }
                    }
                    t_read_last = sw_main.ElapsedMilliseconds;
                }
            }

            // Log finished import
            LogEvent(String.Format("[GetRobotLog] FINISHED: Robot Log Import: bytes={0}B dt={1}ms", read_ind + 1, sw_main.ElapsedMilliseconds - t_start));

            // Reset timeout and logging flag
            sp_Xbee.ReadTimeout = 500;
            robLogger.isLogging = false;

            // Store message length
            msg_lng = read_ind + 1;

            // Parse string and store logs
            char[] out_arr = new char[300];
            int write_ind = 0;
            char c = '\0';
            bool do_rec_store = false;
            char[] int_str = new char[5] { '\0', '\0', '\0', '\0', '\0' };
            int int_cnt = 0;
            int rec_now = 0;
            int rec_last = 0;
            for (int i = 0; i < msg_lng; i++)
            {

                // Get next char
                c = in_arr[i];

                // Parse log number
                if (!do_rec_store && write_ind == 0 && c == '[')
                    do_rec_store = true;
                else if (do_rec_store && c == ']')
                {
                    rec_now = int.Parse(new string(int_str, 0, int_cnt));
                    int_cnt = 0;
                    do_rec_store = false;
                }
                else if (do_rec_store)
                {
                    if (int_cnt < 5)
                    {
                        int_str[int_cnt] = c;
                        int_cnt++;
                    }
                    // Lost ']' byte
                    else
                    {
                        rec_now = 0;
                        int_cnt = 0;
                        do_rec_store = false;
                    }
                }

                // Store next char
                if (c != '\r' && c != '\n')
                {
                    out_arr[write_ind] = c;
                    write_ind++;
                }

                // Store complete string
                else if (write_ind > 0)
                {
                    // Create new string to store
                    string new_log = new string(out_arr, 0, write_ind);

                    // Print current log
                    if (db.printRobLog)
                        Console.WriteLine(String.Format("   log_r2c[{0}]: message=\"{1}\"", rec_now, new_log));

                    // Check for missed log
                    if (rec_now != rec_last + 1)
                    {
                        robLogger.cntDropped++;
                        LogEvent(String.Format("!!ERROR!! [GetRobotLog] Lost Log: read={0} stored={1} dropped={2}", rec_now, robLogger.cntLogged, robLogger.cntDropped));
                    }

                    // Update list
                    robLogger.UpdateList(new_log);

                    // Save record number 
                    rec_last = rec_now;

                    // Reset ind
                    write_ind = 0;
                }

                // Continue
                else
                {
                    continue;
                }
            }

            // Log finished import
            LogEvent(String.Format("[GetRobotLog] FINISHED: Robot Log Store: logged={0}B dt={1}ms", robLogger.cntLogged, sw_main.ElapsedMilliseconds - t_start));

            // Save robot log file
            robLogger.SaveLog(logDir, robLogFi);
            // Log finished save
            LogEvent("[GetRobotLog] FINISHED: Robot Log Save");

            // Set flag that log is finished
            robLogger.isLogFinished = true;
        }

        #endregion

        #region --------MOVEMENT AND TRACKING---------

        // COMPUTE POS IN RAD FOR VT DATA
        public static bool CompPos(ushort ent, ulong ts, double x, double y)
        {

            // Get first vtTS once
            if (vtStr == 0)
            {
                vtStr = ts;
            }

            // Get record vtEnt
            vtEnt = (byte)ent;

            // Convert to ms and update vtTS
            int ts_last = vtTS[vtEnt, 1];
            int ts_now = (int)Math.Round((double)((ts - vtStr) / 1000));

            // Rescale y as VT data is compressed in y axis
            y = y * 1.0976;

            // Normalize 
            x = (x - X_CENT) / RADIUS;
            y = (y - Y_CENT) / RADIUS;

            // Flip y 
            y = y * -1;

            // Compute radians
            double rad_last = vtRad[vtEnt, 1];
            double rad_now = Math.Atan2(y, x);
            double roh = Math.Sqrt(Math.Abs(x) * Math.Abs(x) + Math.Abs(y) * Math.Abs(y));

            // Convert radians to range between [0, 2*pi]
            if (rad_now < 0)
            {
                rad_now = rad_now + 2 * Math.PI;
            }

            // Compute velocity (cm/sec)
            double dt = (double)(ts_now - ts_last) / 1000.0;
            double rad_diff = RadDiff(rad_last, rad_now);
            double vel = rad_diff *
                ((140 * Math.PI) / (2 * Math.PI)) /
                dt;
            // Convert back to pixels with lower left = 0
            x = Math.Round(x * RADIUS) + RADIUS;
            y = Math.Round(y * RADIUS) + RADIUS;

            // Convert cart to cm
            x = x * (140 / (RADIUS * 2));
            y = y * (140 / (RADIUS * 2));

            // Convert rad to cm
            double radFlip = Math.Abs(rad_now - (2 * Math.PI)); // flip
            double cm = radFlip * ((140 * Math.PI) / (2 * Math.PI)); // convert

            // Update vars
            // Save old vals
            vtTS[vtEnt, 0] = ts_last;
            vtRad[vtEnt, 0] = (float)rad_last;
            // New vals
            vtX = (float)x;
            vtY = (float)y;
            vtTS[vtEnt, 1] = ts_now;
            vtRad[vtEnt, 1] = (float)rad_now;
            vtCM = (float)cm;

            // Determine if data should be used
            if (roh < 0.7857 || roh > 1.1 || vel > 300)
            {
                return false;
            }
            else
            {
                return true;
            }

        }

        // CALCULATE MOVE TO POS IN RAT
        public static double CalcMove(double pos_rad)
        {
            // Convert from rad to CM
            double flip_rad = Math.Abs(pos_rad - (2 * Math.PI));
            double cm = (flip_rad * ((140 * Math.PI) / (2 * Math.PI)));
            return cm;
        }

        #endregion

        #region ---------ASYNCHRONOUS METHODS---------

        // CALLBACK FOR NETCOM STREAMING
        public static void NetComCallbackVT(object sender, MNetCom.MVideoRec records, int numRecords, string objectName)
        {
            if (!vtBlocker.isBlocked)
            {
                // Compute position
                bool pass = CompPos(records.swid, records.qwTimeStamp, records.dnextracted_x, records.dnextracted_y);
                // Send data
                if (pass)
                {
                    RepeatSendPack(send_max: 1, id: 'P', do_comf: false);
                }
            }
            else if (db.printBlockedVt)
            {
                LogEvent("      vt blocked");
            }
        }

        // DOWORK FOR bw_RunGUI WORKER
        [STAThread]
        public static void DoWork_RunGUI(object sender, DoWorkEventArgs e)
        {
            LogEvent("[DoWork_RunGUI] RUNNING: RunGUI Worker...");
            object result = null;

            // Set Matlab paths
            SendMCOM(@"addpath(genpath('" + matStartDir + "'));");
            com_Matlab.Feval("startup", 0, out result);
            LogEvent("[DoWork_RunGUI] RUNNING: ICR_GUI");

            // Run ICR_GUI.m
            try
            {
                com_Matlab.Feval("ICR_GUI", 0, out result, db.systemTest, db.debugMat, false, sw_main.ElapsedMilliseconds);
            }
            catch
            {
                LogEvent("!!ERROR!! [DoWork_RunGUI] ABBORTED: ICR_GUI");
            }
            LogEvent("[DoWork_RunGUI] FINISHED: ICR_GUI");
            e.Result = " ";
        }

        // RUNWORKERCOMPLETED FOR bw_RunGUI WORKER
        public static void RunWorkerCompleted_RunGUI(object sender, RunWorkerCompletedEventArgs e)
        {
            LogEvent("[RunWorkerCompleted_RunGUI] FINISHED: RunGUI Worker");
        }

        // DOWORK FOR bw_MatCOM WORKER
        [STAThread]
        public static void DoWork_MatCOM(object sender, DoWorkEventArgs e)
        {
            LogEvent("[DoWork_MatCOM] RUNNING: MatCOM Worker...");
            BackgroundWorker worker = (BackgroundWorker)sender;

            // Create tuple to pass args
            Tuple<char, double, double, double> bw_args = (Tuple<char, double, double, double>)e.Argument;

            //Put the arguments into nicely named variables:
            char id = bw_args.Item1;
            double dat1 = bw_args.Item2;
            double dat2 = bw_args.Item3;
            double dat3 = bw_args.Item4;
            bool flag = false;

            // Check for matlab input till quit or abort
            while (fc.ContinueMatCom())
            {

                // Get flag
                flag = (bool)com_Matlab.GetVariable("m2c_flag", "global");

                // Check for new command
                if (flag)
                {
                    LogEvent("   new matcom data");
                    // Get new message id
                    var i = com_Matlab.GetVariable("m2c_id", "global");
                    id = System.Convert.ToChar(i);

                    // Get new data
                    // data1
                    var d1 = com_Matlab.GetVariable("m2c_dat1", "global");
                    dat1 = d1 != -1 ? (double)d1 : Double.NaN;

                    // data2
                    var d2 = com_Matlab.GetVariable("m2c_dat2", "global");
                    dat2 = d2 != -1 ? (double)d2 : Double.NaN;

                    // data3
                    var d3 = com_Matlab.GetVariable("m2c_dat3", "global");
                    dat3 = d3 != -1 ? (double)d3 : Double.NaN;

                    // Reset flag
                    SendMCOM("m2c_flag = false;");

                    // Trigger progress change event
                    worker.ReportProgress(0, new System.Tuple<char, double, double, double>(id, dat1, dat2, dat3));
                }
            }
            // end polling
            e.Result = " ";
        }

        // PROGRESSCHANGED FOR bw_MatCOM WORKER
        public static void ProgressChanged_MatCOM(object sender, ProgressChangedEventArgs e)
        {
            // Pull out tuble vals
            Tuple<char, double, double, double> bw_args = (Tuple<char, double, double, double>)e.UserState;

            // Store id in top level vars
            char id = bw_args.Item1;
            double dat1 = bw_args.Item2;
            double dat2 = bw_args.Item3;
            double dat3 = bw_args.Item4;

            // Store flags
            m2c.Update(id: id, t: sw_main.ElapsedMilliseconds);

            // Print incoming mesage
            string msg_str = String.Format("   rcvd_m2c: id={0} dat1={1:0.00} dat2={2:0.00} dat3={3:0.00}", id, dat1, dat2, dat3);
            LogEvent(msg_str, m2c.t_now, m2c.t_last);

            // Check quit
            if (id == 'X')
            {
                // Check if this is a premature quit
                if (!fc.isSesSaved)
                {
                    // Start exiting early
                    fc.doAbort = true;
                    // Will print once
                    if (!fc.isGUIquit)
                        LogEvent("!!ERROR!! [DoWork_MatCOM] GUI FORCED QUIT");
                }
                // Set flag that GUI has quit
                fc.isGUIquit = true;
            }

            // Check if ses saved command
            if (id == 'F')
            {
                fc.isSesSaved = true;
                LogEvent("[ProgressChanged_MatCOM] ICR_GUI Confirmed Save");
            }

            // Check closed
            if (id == 'C')
            {
                // Check if this is a premature close
                if (!fc.isGUIquit)
                {
                    // Start exiting early
                    fc.doAbort = true;
                    // Will print once
                    if (!fc.isGUIfinished)
                        LogEvent("!!ERROR!! [DoWork_MatCOM] GUI FORCED CLOSE");
                }
                // Set flag that GUI has closed
                fc.isGUIfinished = true;
            }

            // Check if mesage should be relayed to rob
            for (int i = 0; i < c2r.idList.Length - 1; i++)
                if (id == c2r.idList[i])
                {
                    bool do_check_done = false;

                    // Check if move to command
                    if (id == 'M') // move to
                    {
                        // calculate move to pos
                        dat1 = CalcMove(dat1);
                        do_check_done = true;
                    }
                    // Check if reward command with pos given
                    else if (id == 'R' && dat1 > 0)
                    {
                        // calculate target pos
                        dat1 = CalcMove(dat1);
                    }
                    // Check if rat out command
                    else if (id == 'I' && dat1 == 0)
                    {
                        // Rat is out
                        fc.isRatOut = true;
                    }

                    // Send data
                    RepeatSendPack(id: id, dat1: dat1, dat2: dat2, dat3: dat3, do_comf: true, check_done: do_check_done);
                }
        }

        // RUNWORKERCOMPLETED FOR bw_MatCOM WORKER
        public static void RunWorkerCompleted_MatCOM(object sender, RunWorkerCompletedEventArgs e)
        {
            LogEvent("[RunWorkerCompleted_MatCOM] FINISHED: MatCOM Worker");
        }

        #endregion

        #region ---------MINOR METHODS---------

        // COMPUTE DIFF FOR RAD INPUT
        public static double RadDiff(double rad1, double rad2)
        {
            double rad_diff =
             Math.Abs(rad2 - rad1) < (2 * Math.PI - Math.Abs(rad2 - rad1)) ?
             Math.Abs(rad2 - rad1) : (2 * Math.PI - Math.Abs(rad2 - rad1));
            return rad_diff;
        }

        // DETERMINE IF EXE OPEN
        public static bool IsProcessOpen(string name)
        {
            foreach (Process clsProcess in Process.GetProcesses())
            {
                if (clsProcess.ProcessName.Contains(name))
                {
                    return true;
                }
            }
            return false;
        }

        // OPEN/RUN CHEETAH
        public static void OpenCheetah(string config)
        {
            ProcessStartInfo startInfo = new ProcessStartInfo();
            string nowDir = Directory.GetCurrentDirectory();
            string newDir = @"C:\Program Files\Neuralynx\Cheetah";
            Directory.SetCurrentDirectory(newDir);
            startInfo.FileName = @"C:\Program Files\Neuralynx\Cheetah\Cheetah.exe";
            startInfo.Arguments = string.Format("\"C:\\Users\\Public\\Documents\\Cheetah\\Configuration\\{0}\"&", config);
            startInfo.WindowStyle = ProcessWindowStyle.Minimized;
            Process.Start(startInfo);
            Directory.SetCurrentDirectory(nowDir);
        }

        // FORCE KILL MATLAB EXE
        public static void KillMatlab()
        {
            // Kill newest MATLAB instance
            Process[] proc = Process.GetProcessesByName("MATLAB");
            int ind = 0;
            if (proc.Length > 1)
            {
                long[] start_time = new long[proc.Length];
                for (int i = 0; i < proc.Length; i++)
                {
                    start_time[i] = proc[i].StartTime.Ticks;
                }
                long max = start_time[0];
                for (int i = 1; i < proc.Length; i++)
                {
                    if (start_time[i] > max) ind = i;
                }
            }
            proc[ind].Kill();
            ProcessStartInfo startInfo = new ProcessStartInfo();

            // May have an issue with the following too:
            // Microsoft.VsHub.Server.HttpHost.exe
            // Microsoft.VsHub.Server.HttpHostx64.exe
            // wmic process get name,creationdate
        }

        #endregion

        #region ---------DEBUGGING---------

        // LOG EVEN STRING
        public static void LogEvent(string msg_in, long t_now = -1, long t_last = -1)
        {
            lock (lock_printState)
            {
                // Local vars
                long t_m = 0;
                float t_s = 0;
                long dt = 0;
                string msg_print = " ";
                string ts_print = " ";
                string dt_print = " ";

                // Get time from start of Main()
                if (sw_main.IsRunning)
                    t_now = t_now > 0 ? t_now : sw_main.ElapsedMilliseconds;
                else
                    t_now = t_now > 0 ? t_now : 0;

                // Get sync correction
                t_m = t_now - t_sync;

                // Convert to seconds
                t_s = t_m > 0 ? (float)(t_m) / 1000.0f : 0;

                // Convert to string
                ts_print = String.Format("[{0:0.00}s]", t_s);

                // No input time
                if (t_last >= 0)
                {
                    dt = t_now - t_last;
                    dt_print = String.Format(" [dt:{0}ms]", dt);
                }

                // Pad message
                ts_print = ts_print.PadRight(15, ' ');

                // Cat strings
                msg_print = "\n" + ts_print + msg_in + dt_print + "\n";

                // Print message
                Console.Write(msg_print);

                // Store in logger 
                csLogger.UpdateList(msg_in, t_m);
            }
        }

        // SEND SIMULATED RAT DATA FROM MATLAB TO FEEDERDUE
        public static void RelaySimRat()
        {
            // Create global vars
            com_Matlab.PutWorkspaceData("test_simX", "global", 0.0);
            com_Matlab.PutWorkspaceData("test_simY", "global", 0.0);
            com_Matlab.PutWorkspaceData("test_simTS", "global", 0);
            ushort ent = 0;
            ulong ts_last = 0;
            ulong ts_now = 0;
            double x_now;
            double y_now;

            // Wait for streaming to begin
            while (!fc.isRobStreaming && fc.ContinueMatCom()) ;

            // Check for new data till quit time
            while (fc.isRobStreaming && fc.ContinueMatCom())
            {
                // Get ts
                var ts = com_Matlab.GetVariable("test_simTS", "global");
                ts_now = (ulong)System.Convert.ToDouble(ts);

                // Check if different from last
                if (ts_now != ts_last)
                {
                    // Get remaining vars
                    var x = com_Matlab.GetVariable("test_simX", "global");
                    x_now = System.Convert.ToDouble(x);
                    var y = com_Matlab.GetVariable("test_simY", "global");
                    y_now = System.Convert.ToDouble(y);

                    // Run compute pos
                    bool pass = CompPos(ent, ts_now, x_now, y_now);
                    // Send data
                    if (pass)
                    {
                        RepeatSendPack(send_max: 1, id: 'P', do_comf: false);
                    }

                    // Update ts_last
                    ts_last = ts_now;

                    // Pause thread
                    Thread.Sleep(100);
                }
            }

        }

        #endregion

    }

    #region ---------OTHER CLASSES---------

    // Track program flow
    class Flow_Control
    {
        // Private vars
        private static bool _doAbort = false;
        private static bool _isMAThanging = false;
        // Public vars
        public bool isHandshook = false;
        public bool isNlxConnected = false;
        public bool isRobStreaming = false;
        public bool isMovedToStart = false;
        public bool isRatOut = false;
        public bool isSesSaved = false;
        public bool isGUIquit = false;
        public bool isGUIfinished = false;
        public bool doExit = false;
        public bool doAbort
        {
            set { _doAbort = value; }
            get { return _doAbort; }
        }
        public bool isMAThanging
        {
            set
            {
                _isMAThanging = value;
                if (value)
                {
                    _doAbort = value;
                }

            }
            get { return _isMAThanging; }
        }

        // Check if serial coms are available
        public bool ContinueSerial()
        {
            return (isRobStreaming || !_doAbort) && !doExit;
        }

        // Check if matlab coms are available
        public bool ContinueMatCom()
        {
            return !isMAThanging && !doExit;
        }

    }

    // Track program flow
    class Com_Track
    {
        // Private vars
        private object _lockRcvdCheck = new object();
        private object _lockDoneCheck = new object();
        private bool[] _doRcvdCheck;
        private bool[] _doDoneCheck;
        private long _t_now;
        private long _t_last;
        // Public vars
        public char[] idList;
        public int listLength;
        public char idNow;
        public byte[] head = new byte[1] { 0 };
        public byte[] foot = new byte[1] { 0 };
        public ushort[] packList;
        public ushort packCnt = 0;
        public long[] t_list;
        public long t_now
        {
            set
            {
                _t_last = _t_now;
                _t_now = value;
            }
            get { return _t_now; }
        }
        public long t_last
        {
            set { _t_last = value; }
            get { return _t_last; }
        }
        public bool[] doRcvdCheck
        {
            set
            {
                lock (_lockRcvdCheck)
                {
                    _doRcvdCheck = value;
                }

            }
            get { return _doRcvdCheck; }
        }
        public bool[] doDoneCheck
        {
            set
            {
                lock (_lockDoneCheck)
                {
                    _doDoneCheck = value;
                }

            }
            get { return _doDoneCheck; }
        }

        // Constructor
        public Com_Track(
            object _lockRcvd,
            object _lockDone,
            char[] _idList,
            char _idNow = ' ',
            byte _head = 0,
            byte _foot = 0
            )
        {
            _lockRcvdCheck = _lockRcvd;
            _lockDoneCheck = _lockDone;
            idList = _idList;
            idNow = _idNow;
            head[0] = _head;
            foot[0] = _foot;
            listLength = _idList.Length;
            packList = new ushort[listLength];
            t_list = new long[listLength];
            _doRcvdCheck = new bool[listLength];
            doRcvdCheck = new bool[listLength];
            _doDoneCheck = new bool[listLength];
            doDoneCheck = new bool[listLength];
            t_now = 0;
            t_last = 0;

            // Initialize values to zero
            for (int i = 0; i < listLength; i++)
            {
                packList[i] = 0;
                t_list[i] = 0;
                _doRcvdCheck[i] = false;
                _doDoneCheck[i] = false;
            }
        }

        // Check if command was sent recently
        public bool Recent(char id, long t)
        {
            // Local vars
            bool is_recent = false;
            long dt = Math.Abs(t - t_list[ID_Ind(id)]);

            // Check if processed within past second
            if (
                t_list[ID_Ind(id)] != 0 &&
                dt < 1000
                )
                is_recent = true;

            return is_recent;
        }

        // Update packet info
        public void Update(char id, ushort pack = 0, long t = 0)
        {
            // Update vars
            idNow = id;
            packList[ID_Ind(id)] = pack;
            t_last = t_now;
            t_now = t;
            t_list[ID_Ind(id)] = t;
        }

        // Find id index
        public int ID_Ind(char id)
        {
            int ind = -1;
            for (int i = 0; i < idList.Length; i++)
            {
                if (id == idList[i]) ind = i;
            }
            return ind;
        }

        // Find packet index
        public int PackInd(ushort pack)
        {
            int ind = -1;
            ushort pack_comp = 0;
            for (int i = 0; i < packList.Length; i++)
            {
                pack_comp = packList[i];
                if (pack == pack_comp) ind = i;
            }
            return ind;
        }

    }

    // CLASS TO BLOCK SENDIGN VT DATA
    class VT_Blocker
    {
        // Private vars
        private static readonly object _lockBlock = new object();
        private static int _threadCnt = 0;
        private static Stopwatch _sw = new Stopwatch();
        private static long _t_blockTim = 0;
        private static long _blockFor = 60; // (ms)
        private static bool _isBlocked;
        // Public vars
        public bool isBlocked
        { get { return _isBlocked; } }

        // Constructor
        public VT_Blocker()
        {
            _sw.Start();
        }

        // Block sending vt data
        public void Block(char id)
        {
            if (id != 'P')
            {
                lock (_lockBlock)
                {
                    _threadCnt++;
                    _isBlocked = true;
                    _t_blockTim = _sw.ElapsedMilliseconds + _blockFor;
                }
            }
        }

        // Unblock sending vt data
        public void Unblock(char id)
        {
            if (id != 'P')
            {
                while (_sw.ElapsedMilliseconds < _t_blockTim) ;
                lock (_lockBlock)
                {
                    _threadCnt--;
                    if (_threadCnt < 1)
                    {
                        _isBlocked = false;
                    }
                }
            }
        }
    }

    // CLASS TO LOG  DB INFO
    class DB_Logger
    {
        // Private vars
        private string[] _logList = new string[15000];
        private readonly object _lockLog = new object();
        private Stopwatch _sw = new Stopwatch();
        private long _t_logStart = 0;
        private long _t_lastUpdate = 0;
        private string _lastLogStr = " ";
        private bool _isLogging = false;
        private bool _isLogFinished = false;
        // Public vars
        public int cntLogged = 0;
        public int cntDropped = 0;
        public bool isLogging
        {
            set
            {
                // Store total log time
                if (value)
                    _t_logStart = _sw.ElapsedMilliseconds;
                _isLogging = value;
            }
            get { return _isLogging; }
        }
        public bool isLogComplete
        {
            get
            {
                if (cntLogged > 0 && cntDropped == 0)
                    return true;
                else
                    return false;
            }
        }
        public bool isLogFinished
        {
            set
            {
                lock (_lockLog)
                    _isLogFinished = value;
            }
            get { return _isLogFinished; }
        }
        public long logDT
        {
            get { return _sw.ElapsedMilliseconds - _t_logStart; }
        }

        // Constructor
        public DB_Logger()
        {
            _sw.Start();
            _t_lastUpdate = _sw.ElapsedMilliseconds + 1000;
        }

        // Add new log entry
        public void UpdateList(string log_str, long ts = -1)
        {
            lock (_lockLog)
            {
                // Check for repeat
                if (log_str != _lastLogStr)
                {
                    // Save log string
                    _lastLogStr = log_str;

                    // Itterate count
                    cntLogged++;

                    // Add count and time
                    string str;
                    if (ts < 0)
                        // Add count but skip time
                        str = String.Format("[{0}],{1}", cntLogged, log_str);
                    else
                        // Add count and time
                        str = String.Format("[{0}],{1},{2}", cntLogged, ts, log_str);

                    // Add to list
                    _logList[cntLogged - 1] = str;

                }

                // Update last log
                _t_lastUpdate = _sw.ElapsedMilliseconds;
            }
        }

        // Save log data to csv
        public void SaveLog(string log_dir, string log_fi)
        {
            string fi_path = @log_dir + @"\" + @log_fi;
            using (System.IO.StreamWriter file_out = new System.IO.StreamWriter(fi_path))
            {
                int count = 0;
                foreach (string line in _logList)
                {
                    if (count++ == cntLogged) break;
                    file_out.WriteLine(line);
                }
            }
        }

    }

    #endregion

}