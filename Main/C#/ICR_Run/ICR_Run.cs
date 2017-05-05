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

        // Run system test
        /*
            0: No test
            1: Run MATLAB in debug mode
            2: Halt error test
        */
        private static double systemTest = 1;
        // Print all blocked vt recs
        private static bool doPrint_blockedVt = false;
        // Print all sent vt recs
        private static bool doPrint_sentVt = false;
        // Print robot log
        private static bool doPrint_rcvdLog = true;

        #endregion

        #region ---------TOP LEVEL VARS---------

        // To quit exit program smoothly
        private static bool isMAThanging = false;
        private static bool isRobStreaming = false;
        private static bool isMovedToStart = false;
        private static bool isRatOut = false;
        private static bool isSesSaved = false;
        private static bool doQuit = false;
        private static bool doGUIclose = false;
        private static bool isGUIfinished = false;
        private static bool doExit = false;
        private static bool doAbort = false;

        // Create stop watch object
        private static Stopwatch sw_main = new Stopwatch();
        private static long t_sync = 0;

        // Initialize callback object
        private static MNetCom.MNC_VTCallback deligate_netComCallback;

        // Initialize serial port object to Xbee
        private static System.IO.Ports.SerialPort sp_Xbee = new System.IO.Ports.SerialPort();

        // Initialize MATLAB COM object
        private static MLApp.MLApp com_Matlab = new MLApp.MLApp();

        // Initialize ICR_GUI background worker
        private static BackgroundWorker bw_RunGUI = new BackgroundWorker();

        // Initialize MATLAB com background worker
        private static BackgroundWorker bw_MatCOM = new BackgroundWorker();

        // Create lock objects for safe threading
        static readonly object lock_printState = new object();
        static readonly object lock_sendData = new object();
        private static VT_Blocker vtBlocker = new VT_Blocker();
        private static Due_Logger dueLogger = new Due_Logger();
        private static CS_Logger csLogger = new CS_Logger();

        // Directories
        private static string matStartDir = @"C:\Users\lester\MeDocuments\AppData\MATLABMO\Startup";
        private static string nlxRecDir = " "; // cheetah dir
        private static string dueLogFi = @"FeederDueLogFile.csv"; // log file for FeederDue
        private static string csLogFi = @"ICR_RunLogFile.csv"; // log file for ICR_Run

        // Matlab to CS communication
        private static char[] m2c_id = new char[12] { // prefix giving masage id
            'T', // system test command
            'S', // start session
            'M', // move to position
            'R', // run reward
            'C', // cue reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'N', // matlab not loaded
            'G', // matlab gui loaded
            'A', // connected to AC computer
            'F', // data saved
             };
        private static char matIn_id = 'N'; // matlab now
        private static double matIn_dat1; // matlab data
        private static double matIn_dat2; // matlab data

        // CS to robot communication
        private static char[] c2r_id = new char[13] {
            'T', // system test command
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // run reward
            'C', // cue reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'P', // position data
            'V', // request stream status
            'L', // request log send/resend
            'Y', // confirm done recieved
             };
        private static char[] c2r_charHead = new char[] { '_', '<' }; // 2 byte header
        private static char c2r_charFoot = '>'; // 1 byte header
        private static List<char> c2r_idHist = new List<char>();
        private static List<ushort> c2r_packHist = new List<ushort>();
        private static ushort[] c2r_packLast = new ushort[c2r_id.Length];
        private static ushort c2r_packCnt = 0; // packet number
        private static byte[] c2r_head = new byte[2];
        private static byte[] c2r_foot = new byte[1];
        private static byte[] c2r_packNum = new byte[2];

        // Robot to CS communication
        private static char[] r2c_id = new char[15] {
            'T', // system test command
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // run reward
            'C', // cue reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'D', // execution done
            'V', // connected and streaming
            'L', // request log send/resend
            'U', // log pack
            'J', // battery voltage
            'Z', // reward zone
             };
        private static char r2c_head = '{';
        private static char r2c_foot = '}';
        private static List<char> r2c_idHist = new List<char>();
        private static List<ushort> r2c_packHist = new List<ushort>();
        private static ushort[] r2c_packLast = new ushort[r2c_id.Length];

        // Robot to Matlab communication
        private static char[] r2m_id = new char[2] {
            'J', // battery voltage
            'Z', // reward zone
             };

        // CS to Matlab communication
        private static char[] c2m_id = new char[2] {
            'S', // enable save
            'E', // exit
             };

        // General communication
        private static long sendSentDel = 5; // (ms)
        private static long sendRcvdDel = 1; // (ms)
        private static long resendTimeout = 500; // (ms)
        private static long chkDoneTimeout = 10000; // (ms)
        private static long importLogTimeout = 60000; // (ms)
        private static int resendMax = 5;
        private static long t_c2r = 0;
        private static long t_c2rLast = 0;
        private static long t_r2c = 0;
        private static long t_r2cLast = 0;
        private static long t_m2c = 0;
        private static long t_m2cLast = 0;
        private static int droppedPacks = 0;
        // serial to other ard
        private static char r2a_head = '[';

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
            LogEvent("[Main] RUNNING: Main...");

            // Setup debugging
            if (systemTest != 0)
            {
                // Hide/show matlab app window
                com_Matlab.Visible = 1;
                // Set rec dir to test dir
                nlxRecDir = @"C:\CheetahData\Temp\0000-00-00_00-00-00";
            }
            else com_Matlab.Visible = 0;

            // Start primary timer
            sw_main.Start();

            // Local vars
            long t_timeout;
            bool do_loop;
            bool pass;
            ushort last_pack;
            double move_to;
            string msg_str;

            // Create header and footer byte array
            UnionHack u = new UnionHack(0, 0, 0, '0', 0);
            // get header chars
            u.c1 = c2r_charHead[0];
            u.c2 = c2r_charHead[1];
            // use only first byte 
            // becasue arduino uses uni-8
            c2r_head[0] = u.b1;
            c2r_head[1] = u.b3;
            // Get footer byte 
            u.c1 = c2r_charFoot;
            c2r_foot[0] = u.b1;

            // Initialize list entries
            r2c_idHist.Add(' ');
            c2r_idHist.Add(' ');
            r2c_packHist.Add(0);
            c2r_packHist.Add(0);

            // Initialize last packet arrays
            for (int i = 0; i < c2r_packLast.Length; i++)
                c2r_packLast[i] = 0;
            for (int i = 0; i < r2c_packLast.Length; i++)
                r2c_packLast[i] = 0;

            // Setup and start Xbee serial
            sp_Xbee.ReadTimeout = 100;
            sp_Xbee.BaudRate = 57600;
            sp_Xbee.PortName = "COM92";
            // Create event handeler for incoming data
            sp_Xbee.DataReceived += DataReceived_Xbee;
            // Open serial port connection
            sp_Xbee.Open();
            LogEvent("[Main] FINISHED: Xbee Serial Port Open");

            /*
            // TEST
            LogEvent("[Main] RUNNING: Get Robot Log...");
            pass = GetRobotLog();
            if (pass)
            {
                LogEvent("[Main] FINISHED: Get Robot Log");
                // Print
                msg_str = String.Format("[Main] LOGGED {0} Robot Events Dropped: {1} Packs)", dueLogger.logCnt, droppedPacks);
                LogEvent(msg_str, t_r2cLast, t_r2c);
                // Save log file
                if (nlxRecDir != " ")
                    dueLogger.SaveLog(nlxRecDir, dueLogFi);
            }
            else LogEvent("[Main] !!ABORTED: Get Robot Log!!");
            Console.ReadKey();
            return;
            */

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

            // Start Cheetah if it is not already running
            LogEvent("[Main] RUNNING: Cheetah Open...");
            while (!IsProcessOpen("Cheetah") && !doAbort)
            {
                OpenCheetah("Cheetah.cfg");
            }
            if (!doAbort) LogEvent("[Main] FINISHED: Cheetah Open");

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
            char bw_id = ' ';
            double bw_d1 = 0;
            double bw_d2 = 0;
            var bw_args = Tuple.Create(bw_id, bw_d1, bw_d2);
            bw_MatCOM.RunWorkerAsync(bw_args);

            // Wait for ICR_GUI to connect to AC computer
            LogEvent("[Main] RUNNING: Wait for AC Connect...");
            t_timeout = sw_main.ElapsedMilliseconds + importLogTimeout;
            do_loop = true;
            pass = false;
            do
            {
                if (matIn_id == 'A')
                {
                    do_loop = false;
                    pass = true;
                    break;
                }
                else if (doAbort || sw_main.ElapsedMilliseconds > t_timeout)
                {
                    do_loop = false;
                    pass = false;
                    break;
                }
            }
            while (do_loop);
            if (pass) LogEvent("[Main] FINISHED: Wait for AC Connect");
            else
            {
                LogEvent("[Main] !!ABORTED: Wait for AC Connect!!");
                if (!doAbort)
                {
                    // Program timed out because matlab was hanging on connect
                    doAbort = true;
                    isMAThanging = true;
                }
            }

            // Wait for ICR_GUI to load and connect to NLX
            LogEvent("[Main] RUNNING: Wait for ICR_GUI NLX Setup...");
            while (matIn_id != 'G' && !doAbort) ;
            if (!doAbort) LogEvent("[Main] FINISHED: Wait for ICR_GUI NLX Setup");

            // Setup connection
            if (!(com_netComClient.AreWeConnected()))
            {
                if (com_netComClient.ConnectToServer(NETCOM_IP))
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
                    // Run CheckDone on new thread
                    LogEvent("[Main] RUNNING: Confirm Robot Streaming...");
                    new Thread(delegate ()
                    {
                        RepeatSend('V', true);
                    }).Start();
                    // Wait for confirmation from robot
                    while (c2r_packLast[CharInd('V', c2r_id)] != r2c_packLast[CharInd('D', r2c_id)] && !doAbort) ;
                    if (!doAbort)
                    {
                        isRobStreaming = true;
                        LogEvent("[Main] FINISHED: Confirm Robot Streaming");
                    }
                    else LogEvent("[Main] !!ABORTED: Confirm Robot Streaming!!");

                    // Wait for setup command confirmation
                    LogEvent("[Main] RUNNING: Confirm Setup");
                    while (r2c_idHist[r2c_idHist.Count - 1] != 'S' && !doAbort) ;
                    if (!doAbort) LogEvent("[Main] FINISHED: Confirm Setup");
                    else LogEvent("[Main] !!ABORTED: Confirm Setup!!");

                    // Wait for initial move to command to complete
                    LogEvent("[Main] RUNNING: MoveTo Start...");
                    while (c2r_packLast[CharInd('M', c2r_id)] != r2c_packLast[CharInd('D', r2c_id)] && !doAbort) ;
                    // set flag
                    if (!doAbort)
                    {
                        isMovedToStart = true;
                        LogEvent("[Main] FINISHED: MoveTo Start");
                    }
                    else LogEvent("[Main] !!ABORTED: MoveTo Start!!");

                    // Main holding loop
                    LogEvent("[Main] RUNNING: Main Loop...");
                    // Stay in loop till rat is out
                    while (com_netComClient.AreWeConnected() && !(isRatOut || doAbort)) ;
                    if (!doAbort) LogEvent("[Main] FINISHED: Main Loop");
                    else LogEvent("[Main] !!ABORTED: Main Loop!!");

                    // Wait for reply on last sent packet
                    LogEvent("[Main] RUNNING: Wait for Last Pack...");
                    pass = WaitForPack();
                    if (pass) LogEvent("[Main] FINISHED: Wait for Last Pack");
                    else LogEvent("[Main] !!ABORTED: Wait for Last Pack!!");

                }
                // Failed to connect to NetCom
                else
                {
                    doAbort = true;
                    LogEvent("[Main] !!FAILED: TO CONNECT TO NETCOM!!");
                }

                // Close down everything
                LogEvent("[Main] EXITING...");

                // Move back to default pos
                if (isMovedToStart && com_netComClient.AreWeConnected())
                {
                    LogEvent("[Main] RUNNING: MoveTo South...");
                    move_to = CalcMove(4.7124 - feedDist);

                    // Wait for robot to finish other commands before sending
                    Thread.Sleep(100);

                    // Send move command on seperate thread and wait for done reply
                    last_pack = c2r_packLast[CharInd('M', c2r_id)];
                    new Thread(delegate ()
                    {
                        RepeatSend('M', move_to, true);
                    }).Start();
                    // Wait for new sent packet
                    while (c2r_packLast[CharInd('M', c2r_id)] == last_pack) ;

                    // Wait for confirmation from robot
                    pass = false;
                    do_loop = true;
                    t_timeout = sw_main.ElapsedMilliseconds + 3000;
                    while (do_loop)
                    {
                        if (c2r_packLast[CharInd('M', c2r_id)] == r2c_packLast[CharInd('M', r2c_id)])
                        {
                            pass = true;
                            do_loop = false;
                        }
                        else if (sw_main.ElapsedMilliseconds > t_timeout)
                        {
                            pass = false;
                            do_loop = false;
                        }
                    }
                    if (pass) LogEvent("[Main] FINISHED: MoveTo South Confirm");
                    else LogEvent("[Main] !!ABORTED: MoveTo South Confirm!!");

                    // Wait for move finished confirmation
                    if (pass)
                    {
                        // wait for confirmation from robot
                        do_loop = true;
                        t_timeout = sw_main.ElapsedMilliseconds + 7500;
                        while (do_loop)
                        {
                            if (c2r_packLast[CharInd('M', c2r_id)] == r2c_packLast[CharInd('D', r2c_id)])
                            {
                                pass = true;
                                do_loop = false;
                            }
                            else if (sw_main.ElapsedMilliseconds > t_timeout)
                            {
                                pass = false;
                                do_loop = false;
                            }

                        }

                    }
                    if (pass) LogEvent("[Main] FINISHED: MoveTo South");
                    else LogEvent("[Main] !!ABORTED: MoveTo South!!");

                }

                // Wait 1 second to see move to plot
                Thread.Sleep(1000);

                // Shut down netcom
                if (IsProcessOpen("Cheetah"))
                {
                    //// Stop recording aquisition
                    string reply = " ";
                    com_netComClient.SendCommand("-StopRecording", ref reply);
                    com_netComClient.SendCommand("-StopAcquisition", ref reply);

                    // Close NetCom sreams
                    com_netComClient.CloseStream(NETCOM_ACQ_ENT_1);
                    com_netComClient.CloseStream(NETCOM_ACQ_ENT_2);

                    // Disconect from NetCom
                    while (com_netComClient.AreWeConnected() && !doAbort)
                    {
                        com_netComClient.DisconnectFromServer();
                        LogEvent("[Main] FINISHED: NetCom Close");
                    }
                }

                // Enable GUI save button
                if (!isMAThanging && !doAbort)
                {
                    com_Matlab.Execute("c2m_S = true;");
                    LogEvent("[Main] FINISHED: Save Enabled");
                }
                else LogEvent("[Main] !!ABORTED: Save Enabled!!");

                // Wait for reply on last sent packet
                LogEvent("[Main] RUNNING: Wait for Last Pack...");
                pass = WaitForPack();
                if (pass) LogEvent("[Main] FINISHED: Wait for Last Pack");
                else LogEvent("[Main] !!ABORTED: Wait for Last Pack!!");

                // Request log data from robot
                LogEvent("[Main] RUNNING: Import Robot Log...");
                if (isRobStreaming)
                {
                    pass = GetRobotLog();
                }
                else pass = false;
                if (pass)
                {
                    LogEvent("[Main] FINISHED: Import Robot Log");
                    // Print log info
                    msg_str = String.Format("[Main] Logged {0} Robot Events and Dropped: {1} Packs", dueLogger.logCnt, droppedPacks);
                    LogEvent(msg_str);
                }
                else LogEvent("[Main] !!ABORTED: Import Robot Log!!");

                // Wait for save complete
                LogEvent("[Main] RUNNING: Wait for ICR_GUI to Save");
                while (!(isSesSaved || doQuit || doAbort)) ;
                if (isSesSaved)
                {
                    LogEvent("[Main] FINISHED: Wait for ICR_GUI to Save");

                    // Get NLX dir
                    dynamic nlx_rec_dir = com_Matlab.GetVariable("m2c_dir", "global");
                    nlxRecDir = (string)nlx_rec_dir;

                    // Confirm log saved
                    msg_str = String.Format("RECORDING DIR: {0}:", nlxRecDir);
                    LogEvent(msg_str);

                }
                else LogEvent("[Main] !!ABBORTED: Wait for ICR_GUI to Save!!");


                // Wait for quit command
                while (!(doQuit || doAbort)) ;

                // Send command for arduino to quit on seperate thread
                new Thread(delegate ()
                {
                    RepeatSend('Q');
                }).Start();

                // Wait for quit confirmation from robot for fixed period of time
                t_timeout = sw_main.ElapsedMilliseconds + 3000;
                pass = false;
                do_loop = true;
                while (do_loop)
                    if (c2r_packLast[CharInd('Q', c2r_id)] == r2c_packLast[CharInd('Q', r2c_id)])
                    {
                        pass = true;
                        do_loop = false;
                    }
                    else if (sw_main.ElapsedMilliseconds > t_timeout)
                    {
                        pass = false;
                        do_loop = false;
                    }
                if (pass) LogEvent("[Main] FINISHED: Confirm Robot Quit");
                else LogEvent("[Main] !!ABORTED: Confirm Robot Quit!!");

                // Send command for ICR_GUI to exit
                if (!isMAThanging)
                {
                    com_Matlab.Execute("c2m_E = true;");
                    LogEvent("[Main] FINISHED: Tell Mat to Close");

                    // Set flag
                    doGUIclose = true;
                }
                else LogEvent("[Main] !!ABORTED: Tell Mat to Close!!");

                // Wait for GUI to close
                while (!isMAThanging && !isGUIfinished) ;

                // Set exit flag to exit all threads
                doExit = true;

                // Wait for threads to close down
                Thread.Sleep(100);

                // Dispose of workers
                //bw_RunGUI.CancelAsync();
                bw_RunGUI.Dispose();
                //bw_MatCOM.CancelAsync();
                bw_MatCOM.Dispose();

                // Clear all MatCOM vars
                if (!isMAThanging)
                {
                    com_Matlab.Execute("clearvars - global;");
                    com_Matlab.Execute("clearvars;");
                    com_Matlab.Execute("close all;");
                    Thread.Sleep(100);
                }

                // Save robot log file
                if (nlxRecDir != " ")
                {
                    dueLogger.SaveLog(nlxRecDir, dueLogFi);
                    // Confirm log saved
                    LogEvent("[Main] FINISHED: Save Robot Log");
                }

                // Hold for errors
                if ((systemTest != 0) || isMAThanging || (doAbort && !doQuit))
                {
                    LogEvent("PRESS ANY KEY TO EXIT");
                    Console.ReadKey();
                }

                // Quit MatCOM
                if (!isMAThanging)
                {
                    com_Matlab.Quit();
                }
                // Kill that mother fucker!
                else
                {
                    KillMatlab();
                }
                LogEvent("[Main] FINISHED: Close MatCOM COM");

                // Save CS log file
                if (nlxRecDir != " ")
                {
                    csLogger.SaveLog(nlxRecDir, csLogFi);
                    // Confirm log saved
                    LogEvent("[Main] FINISHED: Save CS Log");

                }

                // Confirm end of program
                LogEvent("[MAIN] FINISHED ALL");

                // Give time for everything to close
                Thread.Sleep(1000);

                return;

            }
        }

        #region ---------COMMUNICATION---------

        public static bool RepeatSend(char id)
        {
            return RepeatSend(id, double.NaN, double.NaN, false);
        }
        public static bool RepeatSend(char id, double dat_1)
        {
            return RepeatSend(id, dat_1, double.NaN, false);
        }
        public static bool RepeatSend(char id, double dat_1, double dat_2)
        {
            return RepeatSend(id, dat_1, dat_2, false);
        }
        public static bool RepeatSend(char id, bool check_done)
        {
            return RepeatSend(id, double.NaN, double.NaN, check_done);
        }
        public static bool RepeatSend(char id, double dat_1, bool check_done)
        {
            return RepeatSend(id, dat_1, double.NaN, check_done);
        }
        public static bool RepeatSend(char id, double dat_1, double dat_2, bool check_done)
        {
            bool pass_rcvd = false;
            long t_timeout = sw_main.ElapsedMilliseconds + resendTimeout;
            int send_count = 1;
            ushort pack;

            // Send new data with new packet number
            pack = SendData(id, dat_1, dat_2);

            // Keep checking mesage was recieved
            while (
                send_count < resendMax &&
                !doExit &&
                (isRobStreaming || !doAbort)
                )
            {

                // Search for matching command packet
                for (int i = r2c_packHist.Count - 1; i >= 0; i--)
                {
                    if (r2c_packHist[i] == pack)
                    {
                        // Message recieved
                        pass_rcvd = true;
                        continue;
                    }
                }

                // Check for matching done packet
                if (check_done)
                {
                    // Find last done command packet number for given mesage
                    for (int i = r2c_idHist.Count - 1; i >= 0; i--)
                    {
                        while (
                            r2c_idHist.Count != r2c_packHist.Count &&
                            !doExit && (isRobStreaming || !doAbort)
                            ) ;
                        // Check match for current command
                        if (r2c_idHist[i] == 'D' && r2c_packHist[i] == pack)
                        {
                            // Send back done recieved confirmation
                            SendData('Y', pack);

                            // Run CheckDone on new thread
                            new Thread(delegate ()
                            {
                                CheckDone(pack);
                            }).Start();

                            // got mesage confirm and done confirm
                            return true;
                        }
                    }
                }

                // Dont check for done confirm
                else if (pass_rcvd)
                {
                    // got main mesage confirm
                    return true;
                }

                // Need to resend
                if (!pass_rcvd && sw_main.ElapsedMilliseconds > t_timeout)
                {
                    SendData(id, dat_1, dat_2, pack);
                    t_timeout = sw_main.ElapsedMilliseconds + resendTimeout;
                    send_count++;
                }

            }

            // forced quit
            return false;
        }

        public static ushort SendData(char id)
        {
            return SendData(id, double.NaN, double.NaN, 0);
        }
        public static ushort SendData(char id, double dat_1, double dat_2)
        {
            return SendData(id, dat_1, dat_2, 0);
        }
        public static ushort SendData(char id, ushort pack)
        {
            return SendData(id, double.NaN, double.NaN, pack);
        }
        public static ushort SendData(char id, double dat_1, double dat_2, ushort pack)
        {

            lock (lock_sendData)
            {

                // Block vt sending
                vtBlocker.Block(id);

                // Wait for next safe send time
                long t_send;
                bool buff_ready;
                bool do_loop;
                do
                {

                    // Delay send time till x ms after last send or rcvd
                    t_send = t_c2r > t_r2c ? t_c2r + sendSentDel : t_r2c + sendRcvdDel;

                    // Make sure outbut and input buffer have enough space
                    buff_ready = sp_Xbee.BytesToRead == 0 && sp_Xbee.BytesToWrite == 0;

                    // Check if loop should continue
                    do_loop = (
                        sw_main.ElapsedMilliseconds < t_send || !buff_ready) &&
                        !doExit && (isRobStreaming || !doAbort);

                } while (do_loop);

                UnionHack u = new UnionHack(0, 0, 0, '0', 0);

                // Get new packet number
                if (pack == 0)
                {
                    c2r_packCnt++;
                    pack = c2r_packCnt;
                }

                // Store pack number
                u.s1 = pack;
                c2r_packNum[0] = u.b1;
                c2r_packNum[1] = u.b2;

                // Store ID
                byte[] msg_id = new byte[1];
                u.c1 = id;
                msg_id[0] = u.b1;

                // Will store message data
                byte[] msg_data = null;
                // default no data
                int nDataBytes = 0;

                // Send system test command
                if (id == 'T')
                {
                    nDataBytes = 2;
                    msg_data = new byte[nDataBytes];
                    // Add test id
                    msg_data[0] = (byte)dat_1;
                    // Add test parameter
                    msg_data[1] = (byte)dat_2;
                }

                // Send setup data
                if (id == 'S')
                {
                    nDataBytes = 2;
                    msg_data = new byte[nDataBytes];
                    // Add session cond
                    msg_data[0] = (byte)dat_1;
                    // Add sound cond
                    msg_data[1] = (byte)dat_2;
                }

                // Send MoveTo data
                else if (id == 'M')
                {
                    nDataBytes = 4;
                    msg_data = new byte[nDataBytes];
                    // Add move pos
                    u.f = (float)dat_1;
                    msg_data[0] = u.b1;
                    msg_data[1] = u.b2;
                    msg_data[2] = u.b3;
                    msg_data[3] = u.b4;
                }

                // Send Reward data
                else if (id == 'R')
                {
                    nDataBytes = 5;
                    msg_data = new byte[nDataBytes];
                    // Add stop pos
                    u.f = (float)dat_1;
                    msg_data[0] = u.b1;
                    msg_data[1] = u.b2;
                    msg_data[2] = u.b3;
                    msg_data[3] = u.b4;
                    // Add reward duration
                    msg_data[4] = (byte)dat_2;
                }

                // Send Cue data
                else if (id == 'C')
                {
                    nDataBytes = 5;
                    msg_data = new byte[nDataBytes];
                    // Add targ pos
                    u.f = (float)dat_1;
                    msg_data[0] = u.b1;
                    msg_data[1] = u.b2;
                    msg_data[2] = u.b3;
                    msg_data[3] = u.b4;
                    // Add reward duration
                    msg_data[4] = (byte)dat_2;
                }

                // Send robot halt data
                if (id == 'H')
                {
                    nDataBytes = 1;
                    msg_data = new byte[nDataBytes];
                    // Add halt state
                    msg_data[0] = (byte)dat_1;
                }

                // Send bulldoze rat data
                if (id == 'B')
                {
                    nDataBytes = 2;
                    msg_data = new byte[nDataBytes];
                    // Add bull del
                    msg_data[0] = (byte)dat_1;
                    // Add bull speed
                    msg_data[1] = (byte)dat_2;
                }

                // Send rat in/out
                if (id == 'I')
                {
                    nDataBytes = 1;
                    msg_data = new byte[nDataBytes];
                    // Add session cond
                    msg_data[0] = (byte)dat_1;
                }

                // Send log request
                if (id == 'L')
                {
                    nDataBytes = 1;
                    msg_data = new byte[nDataBytes];
                    // Add send/resend request
                    msg_data[0] = (byte)dat_1;
                }

                // Send pos data
                else if (id == 'P')
                {
                    nDataBytes = 9;
                    msg_data = new byte[nDataBytes];
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

                    // Start sw_sync with first vt
                    if (t_sync == 0)
                    {
                        t_sync = sw_main.ElapsedMilliseconds;
                    }
                }

                // Concatinate header and footer
                byte[] msgByteArr = new byte[
                    c2r_head.Length +    // header
                    c2r_packNum.Length + // packet num
                    msg_id.Length +          // id
                    nDataBytes +             // data
                    c2r_foot.Length      // footer
                    ];
                // add header
                c2r_head.CopyTo(msgByteArr, 0);
                // add packet number
                c2r_packNum.CopyTo(msgByteArr, c2r_head.Length);
                // add id
                msg_id.CopyTo(msgByteArr, c2r_head.Length + c2r_packNum.Length);
                // add data
                if (nDataBytes > 0)
                {
                    msg_data.CopyTo(msgByteArr, c2r_head.Length + c2r_packNum.Length + msg_id.Length);
                }
                // add footer
                c2r_foot.CopyTo(msgByteArr, c2r_head.Length + c2r_packNum.Length + msg_id.Length + nDataBytes);

                // Send to arduino
                if (sp_Xbee.IsOpen) sp_Xbee.Write(msgByteArr, 0, msgByteArr.Length);

                // Update send time
                t_c2rLast = t_c2r;
                t_c2r = sw_main.ElapsedMilliseconds;

                // Print sent data
                string msg_str;
                if (id != 'P')
                {
                    // Update sent history
                    bool was_saved = false;
                    for (int i = c2r_packHist.Count - 1; i >= 0; i--)
                    {
                        if (c2r_packHist[i] == pack) was_saved = true;
                    }
                    if (!was_saved)
                    {
                        c2r_idHist.Add(id);
                        c2r_packHist.Add(pack);
                    }

                    // Print sent mesage packet
                    if (nDataBytes == 0)
                    {
                        msg_str = String.Format("   SENT: [id:{0} pack:{1}]", id, pack);
                    }
                    else msg_str = String.Format("   SENT: [id:{0} dat1:{1:0.00} dat2:{2:0.00} pack:{3}]", id, dat_1, dat_2, pack);
                    LogEvent(msg_str, t_c2rLast, t_c2r);
                }
                else if (doPrint_sentVt)
                {
                    msg_str = String.Format("   SENT: [id:{0} ent:{1} ts:{2} cm:{3:0.00} pack:{4}]", id, vtEnt, vtTS[vtEnt, 1], vtCM, pack);
                    LogEvent(msg_str, t_c2rLast, t_c2r);
                }

                // Update sent last
                c2r_packLast[CharInd(id, c2r_id)] = pack;

                // Unlock vt sending
                vtBlocker.Unblock(id);

                // Return packet number 
                return pack;

            }

        }

        public static void CheckDone(ushort pack)
        {
            // Print worker started
            string msg_str;
            msg_str = String.Format("   Running CheckDone Worker (pack:{0})", pack);
            LogEvent(msg_str);

            // Initialize
            long t_timeout = sw_main.ElapsedMilliseconds + chkDoneTimeout;
            int msg_cnt = r2c_idHist.Count;
            int send_cnt = 1;
            int max_send = 5;
            bool pass = true;

            // Spend 10 sec checking for done confirmation resend
            while (
                sw_main.ElapsedMilliseconds < t_timeout &&
                !doExit && (isRobStreaming || !doAbort)
                )
            {
                if (r2c_idHist.Count > msg_cnt)
                {
                    // update mesage count
                    msg_cnt = r2c_idHist.Count;

                    // Wait for list update complete
                    while (
                        r2c_idHist.Count != r2c_packHist.Count &&
                        !doExit && (isRobStreaming || !doAbort)
                        ) ;

                    // Check match for done id and current packet
                    if (r2c_idHist[r2c_idHist.Count - 1] == 'D' && r2c_packHist[r2c_packHist.Count - 1] == pack)
                    {
                        // Itterate send count
                        send_cnt++;
                        // Bail after too long
                        if (send_cnt > max_send)
                        {
                            pass = false;
                            break;
                        }
                        // Resend done confirmation
                        SendData('Y', pack);
                        // Add 10 more seconds
                        t_timeout = sw_main.ElapsedMilliseconds + 10000;
                    }
                }
                // Pause thread
                Thread.Sleep(100);
            }
            msg_str = String.Format("   Finished CheckDone Worker {0} (pack:{1})", pass ? "Passed" : "Failed", pack);
            LogEvent(msg_str);
        }

        public static bool WaitForPack()
        {

            long t_timeout = sw_main.ElapsedMilliseconds + 3000;
            bool pass = false;
            bool do_loop = true;
            ushort pack;
            char id;
            string str;
            do
            {
                // Get sent id and pack
                id = c2r_idHist[c2r_packHist.Count - 1];
                pack = c2r_packHist[c2r_packHist.Count - 1];

                // If move command wait for done
                id = id == 'M' ? 'D' : id;

                // Print current id
                str = String.Format("   waiting for: [id:{0} pack:{1}]...", id, pack);
                LogEvent(str);

                // Check history
                for (int i = r2c_packHist.Count - 1; i >= 0; i--)
                {
                    if (r2c_packHist[i] == c2r_packHist[c2r_packHist.Count - 1])
                    {
                        pass = true;
                        do_loop = false;
                        break;
                    }
                    else if (doAbort && sw_main.ElapsedMilliseconds > t_timeout)
                    {
                        pass = false;
                        do_loop = false;
                        break;
                    }
                }

                // Pause thread
                Thread.Sleep(100);

            } while (do_loop);
            return pass;
        }

        public static void DataReceived_Xbee(object sender, SerialDataReceivedEventArgs e)
        {

            // Loop till all data read out
            while (sp_Xbee.BytesToRead > 0)
            {

                // Local vars
                UnionHack u = new UnionHack(0, 0, 0, '0', 0);
                bool head_found = false;
                bool id_found = false;
                bool pack_found = false;
                bool foot_found = false;
                bool for_ard = false;
                bool do_dump = false;
                byte[] head_rcvd = new byte[1];
                byte[] id_rcvd = new byte[1];
                byte[] dat_rcvd = new byte[1];
                byte[] pack_rcvd = new byte[2];
                byte[] foot_rcvd = new byte[1];
                byte[] chksum_rcvd = new byte[1];
                char head = ' ';
                char id = ' ';
                char foot = ' ';
                byte dat = 0;
                ushort pack = 0;
                ushort chksum = 0;
                string msg_str;

                // Dump till header found
                while (!head_found && sp_Xbee.BytesToRead > 0 &&
                    !doExit && (isRobStreaming || !doAbort)
                    )
                {
                    sp_Xbee.Read(head_rcvd, 0, 1);
                    // Get header
                    u.b1 = head_rcvd[0];
                    u.b2 = 0; // C# chars are 2 bytes
                    head = u.c1;
                    if (head == r2c_head)
                    {
                        head_found = true;
                    }
                    else if (head == r2a_head)
                    {
                        for_ard = true;
                    }
                }

                // Find id and check message is intended for CS
                if (head_found)
                {
                    // Wait/check for full buffer
                    if (BuffReady(1))
                    {
                        sp_Xbee.Read(id_rcvd, 0, 1);
                        // Get id
                        u.b1 = id_rcvd[0];
                        u.b2 = 0;
                        id = u.c1;
                        // Check for match
                        for (int i = 0; i < r2c_id.Length; i++)
                        {
                            if (id == r2c_id[i])
                            {
                                id_found = true;
                            }
                        }
                    }
                }

                // Check if this is a log packet
                if (id != 'U' && head_found && id_found)
                {

                    // Get data byte
                    // Wait/check for full buffer
                    if (BuffReady(1))
                    {

                        // Read in data
                        sp_Xbee.Read(dat_rcvd, 0, 1);
                        dat = dat_rcvd[0];
                    }

                    // Get first and second part of packet number
                    // Wait/check for full buffer
                    if (BuffReady(2))
                    {

                        // Read in data
                        sp_Xbee.Read(pack_rcvd, 0, 2);
                        u.b1 = pack_rcvd[0];
                        u.b2 = pack_rcvd[1];
                        pack = u.s1;

                        // Check if pack matches sent pack or done related id or one way message
                        for (int i = c2r_packHist.Count - 1; i >= 0; i--)
                        {

                            // Wait for sent list to update
                            while (
                                c2r_packHist.Count != c2r_idHist.Count &&
                                !doExit && (isRobStreaming || !doAbort)
                                ) ;

                            // Check that packet matches sent packet
                            if (
                                (c2r_packHist[i] == pack && (c2r_idHist[i] == id || id == 'D')) ||
                                 (CharFound(id, r2m_id) && pack == 0)
                                )
                            {
                                pack_found = true;
                            }
                        }
                    }

                    // Find footer
                    if (pack_found)
                    {

                        // Wait/check for full buffer
                        if (BuffReady(1))
                        {

                            // Read in data
                            sp_Xbee.Read(foot_rcvd, 0, 1);

                            // Chack footer
                            u.b1 = foot_rcvd[0];
                            u.b2 = 0;
                            foot = u.c1;
                            if (foot == r2c_foot)
                            {
                                foot_found = true;
                            }
                        }
                    }

                    // Print and save if complete packet recieved
                    if (foot_found)
                    {
                        // Update recieve time
                        t_r2cLast = t_r2c;
                        t_r2c = sw_main.ElapsedMilliseconds;

                        // print data recieved
                        msg_str = String.Format("   RCVD: [id:{0} dat:{1} pack:{2}]", id, dat, pack);
                        LogEvent(msg_str, t_r2cLast, t_r2c);

                        // Update last pack
                        r2c_packLast[CharInd(id, r2c_id)] = pack;
                        // Update list
                        r2c_idHist.Add(id);
                        r2c_packHist.Add(pack);

                        // Check if data should be relayed to Matlab
                        for (int i = 0; i < r2m_id.Length - 1; i++)
                        {
                            if (r2m_id[i] == id)
                            {
                                com_Matlab.Execute(String.Format("c2m_{0} = {1}", r2m_id[i], (double)dat));
                            }
                        }
                    }
                    else
                    {
                        // Set dump flag 
                        do_dump = true;
                    }
                }

                // Process log data
                else if (id == 'U' && head_found && id_found)
                {

                    // Get check sum
                    // Wait/check for full buffer
                    if (BuffReady(1))
                    {
                        // Read in data
                        sp_Xbee.Read(chksum_rcvd, 0, 1);
                        chksum = chksum_rcvd[0];
                    }

                    // Get complete message
                    byte[] log_rcvd = new byte[chksum];
                    // Wait/check for full buffer
                    if (BuffReady(chksum))
                    {
                        // Read in all data
                        sp_Xbee.Read(log_rcvd, 0, chksum);

                    }
                    // Convert to string
                    string log_str = System.Text.Encoding.UTF8.GetString(log_rcvd);

                    // Read in footer
                    // Wait/check for full buffer
                    if (BuffReady(1))
                    {
                        // Get data
                        sp_Xbee.Read(foot_rcvd, 0, 1);
                        u.b1 = foot_rcvd[0];
                        u.b2 = 0;
                        foot = u.c1;
                    }

                    // Check if footer found
                    foot_found = foot == r2c_foot ? true : false;

                    // Check if packet complete
                    if (foot_found)
                    {
                        // Update list
                        dueLogger.UpdateList(log_str);

                        // print data recieved
                        if (doPrint_rcvdLog)
                        {
                            msg_str = String.Format("LOG[{0}]: \"{1}\" (chksum = {2})", dueLogger.logCnt, log_str, chksum);
                            LogEvent(msg_str);
                        }
                    }
                    else
                    {
                        // Request resend
                        dueLogger.ResendLast();

                        // Set dump flag
                        do_dump = true;
                    }


                }
                else do_dump = true;

                // Dump incomplete packets
                if (do_dump)
                {
                    if (for_ard)
                    {
                        msg_str = String.Format("skipped cheeta due packet");
                    }
                    else
                    {
                        droppedPacks++;
                        msg_str = String.Format("!!PACK LOST (Total:{0}) BuffTX:{1} BuffRX:{2} [head:{3} id:{4} dat:{5} pack:{6} chksum:{7} foot:{8}]!!",
                            droppedPacks, sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, head, id, dat, pack, chksum, foot);
                        LogEvent(msg_str);
                    }
                    // dump input buffer
                    //sp_Xbee.DiscardInBuffer();

                    // Wait for buffer to refil
                    BuffReady(6);

                }

            }

        }

        public static bool BuffReady(int min_byte)
        {
            return BuffReady(min_byte, 1000);
        }
        public static bool BuffReady(int min_byte, long wait_max)
        {
            // Local vars
            long t_timeout = sw_main.ElapsedMilliseconds + wait_max;
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_Xbee.BytesToRead < min_byte &&
                sw_main.ElapsedMilliseconds <= t_timeout &&
                !doExit && (isRobStreaming || !doAbort)
                ) ;

            // Check if buff filled
            pass = sp_Xbee.BytesToRead >= min_byte ? true : false;

            return pass;
        }

        public static bool GetRobotLog()
        {
            // Local vars
            bool pass = false;
            ushort pack = 0;
            long t_timeout = sw_main.ElapsedMilliseconds + 30000;

            // Loop till complete log recieved
            bool do_loop = true;
            do
            {
                // Check if need to send new or resend
                if (dueLogger.SendReady())
                {
                    double send_what = dueLogger.sendWhat;
                    RepeatSend('L', dueLogger.sendWhat);
                }

                // Check if list complete
                pack = c2r_packLast[CharInd('L', c2r_id)];
                if (pack == r2c_packLast[CharInd('D', r2c_id)])
                {
                    pass = true;
                    do_loop = false;

                    // Send one more log request and wait for done
                    // Note: this is just a hack to get confirmation from both ends
                    RepeatSend('L', dueLogger.sendWhat, true);

                }
                else if (
                    sw_main.ElapsedMilliseconds > t_timeout
                    )
                {
                    do_loop = false;
                }

            } while (do_loop);

            return pass;
        }

        #endregion

        #region --------MOVEMENT AND TRACKING---------

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
            double vel = Math.Abs(rad_now - rad_last) *
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

        public static double CalcMove(double pos_rad)
        {
            // Convert from rad to CM
            double flip_rad = Math.Abs(pos_rad - (2 * Math.PI));
            double cm = (flip_rad * ((140 * Math.PI) / (2 * Math.PI)));
            return cm;
        }

        #endregion

        #region ---------ASYNCHRONOUS METHODS---------

        public static void NetComCallbackVT(object sender, MNetCom.MVideoRec records, int numRecords, string objectName)
        {
            if (!vtBlocker.isBlocked)
            {
                // Compute position
                bool pass = CompPos(records.swid, records.qwTimeStamp, records.dnextracted_x, records.dnextracted_y);
                // Send data
                if (pass)
                {
                    SendData('P');
                }
            }
            else if (doPrint_blockedVt)
            {
                LogEvent("      vt blocked");
            }
        }

        [STAThread]
        public static void DoWork_RunGUI(object sender, DoWorkEventArgs e)
        {
            LogEvent("[DoWork_RunGUI] RUNNING: RunGUI Worker...");
            object result = null;

            // Set Matlab paths
            com_Matlab.Execute(@"addpath(genpath('" + matStartDir + "'));");
            com_Matlab.Feval("startup", 0, out result);
            LogEvent("[DoWork_RunGUI] RUNNING: ICR_GUI");

            // Debugging

            // Set MATLAB break point
            //com_Matlab.Execute(@"dbstop in ICR_GUI at 2491");

            // Run ICR_GUI.m
            try
            {
                com_Matlab.Feval("ICR_GUI", 0, out result, systemTest);
            }
            catch
            {
                LogEvent("[DoWork_RunGUI] !!ABBORT: ICR_GUI!!");
            }
            LogEvent("[DoWork_RunGUI] FINISHED: ICR_GUI");
            e.Result = " ";
        }

        public static void RunWorkerCompleted_RunGUI(object sender, RunWorkerCompletedEventArgs e)
        {

            // Set flag that GUI has closed
            isGUIfinished = true;

            // Check if do quit flag has been set
            if (doQuit && doGUIclose)
            {
                LogEvent("[RunWorkerCompleted_RunGUI] FINISHED: RunGUI Worker");
            }
            else
            {
                // GUI was closed prematurely
                doQuit = true;
                doAbort = true;
                LogEvent("[RunWorkerCompleted_RunGUI] !!ABORT: RunGUI Worker!!");
            }
        }

        [STAThread]
        public static void DoWork_MatCOM(object sender, DoWorkEventArgs e)
        {
            LogEvent("[DoWork_MatCOM] RUNNING: MatCOM Worker...");
            BackgroundWorker worker = (BackgroundWorker)sender;

            Tuple<char, double, double> bw_args = (Tuple<char, double, double>)e.Argument;
            //Put the arguments into nicely named variables:
            char bw_id = bw_args.Item1;
            double bw_d1 = bw_args.Item2;
            double bw_d2 = bw_args.Item3;
            bool flag = false;
            bool quit = false;

            // Initalize Matlab global vars
            com_Matlab.PutWorkspaceData("m2c_id", "global", 'N');
            com_Matlab.PutWorkspaceData("m2c_dat1", "global", 9999.0);
            com_Matlab.PutWorkspaceData("m2c_dat2", "global", 9999.0);
            com_Matlab.PutWorkspaceData("m2c_flag", "global", false);
            com_Matlab.PutWorkspaceData("m2c_dir", "global", " ");
            com_Matlab.PutWorkspaceData("m2c_quit", "global", false);
            // r2m vars
            for (int i = 0; i < r2m_id.Length - 1; i++)
            {
                com_Matlab.PutWorkspaceData(String.Format("r2m_{0}", r2m_id[i]), "global", 0.0);
            }
            // c2m vars
            for (int i = 0; i < c2m_id.Length - 1; i++)
            {
                com_Matlab.PutWorkspaceData(String.Format("c2m_{0}", c2m_id[i]), "global", false);
            }

            while (!doQuit && !isMAThanging)
            {

                // Get flag
                flag = (bool)com_Matlab.GetVariable("m2c_flag", "global");

                // Get check quit
                quit = (bool)com_Matlab.GetVariable("m2c_quit", "global");

                // Exicute quit
                if (quit)
                {
                    doQuit = true;
                    // Check if this is a premature quit
                    if (!isSesSaved)
                    {
                        // Start exiting early
                        doAbort = true;
                        LogEvent("[ProgressChanged_MatCOM] !!FORCED EXIT!!");
                    }
                }

                // Check for new command
                if (flag)
                {
                    LogEvent("   New MatCom Data");
                    // Get new message id
                    var id = com_Matlab.GetVariable("m2c_id", "global");
                    bw_id = System.Convert.ToChar(id);

                    // Get new data
                    // data1
                    var d1 = com_Matlab.GetVariable("m2c_dat1", "global");
                    bw_d1 = d1 != 9999 ? (double)d1 : Double.NaN;

                    // data2
                    var d2 = com_Matlab.GetVariable("m2c_dat2", "global");
                    bw_d2 = d2 != 9999 ? (double)d2 : Double.NaN;

                    // Reset flag
                    com_Matlab.Execute("m2c_flag = false;");

                    // Trigger progress change event
                    worker.ReportProgress(0, new System.Tuple<char, double, double>(bw_id, bw_d1, bw_d2));
                }
            }
            // end polling
            e.Result = " ";
        }

        public static void ProgressChanged_MatCOM(object sender, ProgressChangedEventArgs e)
        {
            Tuple<char, double, double> bw_args = (Tuple<char, double, double>)e.UserState;

            // Store id in top level vars
            matIn_id = bw_args.Item1;
            matIn_dat1 = bw_args.Item2;
            matIn_dat2 = bw_args.Item3;

            // Print incoming mesage
            string msg_str = String.Format("   mCOM: [id:{0} dat1:{1:0.00} dat2:{2:0.00}]", matIn_id, matIn_dat1, matIn_dat2);
            t_m2cLast = t_m2c;
            t_m2c = sw_main.ElapsedMilliseconds;
            LogEvent(msg_str, t_m2cLast, t_m2c);

            bool do_send = false;
            bool do_check_done = false;

            // Relay all matlab data but the following
            if (
                matIn_id == 'G' || // loaded
                matIn_id == 'A' || // ac comp
                matIn_id == 'F'    // saved
                )
            {
                // Check if ses saved command
                if (matIn_id == 'F')
                {
                    isSesSaved = true;
                    LogEvent("[ProgressChanged_MatCOM] ICR_GUI Confirmed Save");
                }
            }
            else
            {
                // Check if mesage should be relayed to rob
                for (int i = 0; i < c2r_id.Length - 1; i++)
                    if (matIn_id == c2r_id[i]) do_send = true;

                // Send to arduino and wait for exicution confirmation
                if (do_send)
                {
                    // Check if move to command
                    if (matIn_id == 'M') // move to
                    {
                        // calculate move to pos
                        matIn_dat1 = CalcMove(matIn_dat1);
                        do_check_done = true;
                    }
                    // Check if reward command with pos given
                    else if (
                        (matIn_id == 'R' && matIn_dat1 > 0) || // reward
                        matIn_id == 'C') // cue
                    {
                        // calculate target pos
                        matIn_dat1 = CalcMove(matIn_dat1);
                    }
                    // Check if rat out command
                    else if (matIn_id == 'I' && matIn_dat1 == 0)
                    {
                        // Rat is out
                        isRatOut = true;
                    }

                    RepeatSend(matIn_id, matIn_dat1, matIn_dat2, do_check_done);
                }
            }
        }

        public static void RunWorkerCompleted_MatCOM(object sender, RunWorkerCompletedEventArgs e)
        {
            LogEvent("[RunWorkerCompleted_MatCOM] FINISHED: MatCOM Worker");
        }

        #endregion

        #region ---------MINOR METHODS---------

        public static int CharInd(char id, char[] arr)
        {
            int ind = -1;
            for (int i = 0; i < arr.Length; i++)
            {
                if (id == arr[i]) ind = i;
            }
            return ind;
        }

        public static bool CharFound(char id, char[] arr)
        {
            bool found = false;
            for (int i = 0; i < arr.Length; i++)
            {
                if (id == arr[i]) found = true;
            }
            return found;
        }

        public static bool IsEmptyMat(object dynamicVariable)
        {
            return dynamicVariable.GetType() == typeof(System.Reflection.Missing);
        }

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

        public static void OpenCheetah(string config)
        {
            ProcessStartInfo startInfo = new ProcessStartInfo();
            string nowDir = Directory.GetCurrentDirectory();
            string newDir = @"C:\Program Files\Neuralynx\Cheetah5";
            Directory.SetCurrentDirectory(newDir);
            startInfo.FileName = @"C:\Program Files\Neuralynx\Cheetah5\Cheetah.exe";
            startInfo.Arguments = string.Format("\"C:\\Program Files\\Neuralynx\\Cheetah5\\Configuration\\{0}\"&", config);
            startInfo.WindowStyle = ProcessWindowStyle.Minimized;
            Process.Start(startInfo);
            Directory.SetCurrentDirectory(nowDir);
        }

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
                    Console.WriteLine(proc[i].StartTime);
                    Console.WriteLine(start_time[i]);
                }
                long max = start_time[0];
                for (int i = 1; i < proc.Length; i++)
                {
                    if (start_time[i] > max) ind = i;
                }
                Console.WriteLine(proc[ind].StartTime);
                Console.WriteLine(start_time[ind]);
            }
            proc[ind].Kill();
            ProcessStartInfo startInfo = new ProcessStartInfo();

            // May have an issue with the following too:
            // Microsoft.VsHub.Server.HttpHost.exe
            // Microsoft.VsHub.Server.HttpHostx64.exe
            // wmic process get name,creationdate
        }

        public static void LogEvent(string msg_in, long t1 = -1, long t2 = -1)
        {
            lock (lock_printState)
            {
                // Local vars
                long t_m = 0;
                float t_c = 0;
                long dt = 0;
                string msg_print = " ";
                string ts_print = " ";
                string dt_print = " ";

                // Get time from start of Main()
                t_m = t2 > 0 ? t2 : sw_main.ElapsedMilliseconds;
                t_c = (float)(t_m - t_sync) / 1000.0f;
                ts_print = String.Format("[{0:0.00}s]", t_c);

                // No input time
                if (t1 >= 0)
                {
                    dt = t2 - t1;
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

        #endregion

    }

    #region ---------OTHER CLASSES---------

    class VT_Blocker
    {
        static readonly object _lockBlock = new object();
        private static int _threadCnt = 0;
        private static Stopwatch _sw = new Stopwatch();
        private static long _t_blockTim = 0;
        private static long _blockFor = 60; // (ms)
        private static bool _isBlocked;
        public bool isBlocked
        { get { return _isBlocked; } }

        public VT_Blocker()
        {
            _sw.Start();
        }

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

    class Due_Logger
    {
        private static List<string> _logList = new List<string>();
        static readonly object _lockLog = new object();
        private static Stopwatch _sw = new Stopwatch();
        private static long _t_lastUpdate = 0;
        private static long _t_lastSend = 0;
        private static long _sendDel = 1; // (ms)
        private static long _rcvdDel = 1; // (ms)
        private static long _resendTimeout = 10000; // (ms)
        private static string _lastLogStr = " ";
        public double sendWhat;
        private static bool _doSend;
        private static int _logCnt;
        public int logCnt
        { get { return _logCnt; } }

        public Due_Logger()
        {
            _sw.Start();
            _doSend = true;
            sendWhat = 0;
            _logCnt = 0;
            _t_lastUpdate = _sw.ElapsedMilliseconds + 1000;
        }

        public bool SendReady()
        {
            lock (_lockLog)
            {
                bool do_send = false;

                // Check if too much time has ellapsed since last send    
                if (
                    _sw.ElapsedMilliseconds > _t_lastUpdate + _resendTimeout &&
                    _logCnt > 0
                    )
                {
                    ResendLast();
                }

                // Wait for enough time to ellapse for next send
                if (_doSend)
                {
                    while (
                        (_sw.ElapsedMilliseconds - _t_lastSend) < _sendDel &&
                        (_sw.ElapsedMilliseconds - _t_lastUpdate) < _rcvdDel
                        ) ;

                    // Reset flag
                    do_send = _doSend;
                    _doSend = false;

                    // Update send time
                    _t_lastSend = _sw.ElapsedMilliseconds;
                }

                return do_send;
            }
        }

        public void UpdateList(string log_str)
        {
            lock (_lockLog)
            {
                // Check for repeat
                if (log_str != _lastLogStr)
                {
                    // Save log string
                    _lastLogStr = log_str;

                    // Itterate count
                    _logCnt++;

                    // Add count
                    string str = String.Format("[{0}],", _logCnt) + log_str;

                    // Add to list
                    _logList.Add(str);

                }

                // Update last log
                _t_lastUpdate = _sw.ElapsedMilliseconds;

                // Flag to send next
                _doSend = true;
                sendWhat = 0;
            }
        }

        public void ResendLast()
        {
            lock (_lockLog)
            {
                // Flag to send last
                _doSend = true;
                sendWhat = 1;
            }
        }

        public void SaveLog(string log_dir, string log_fi)
        {
            string fi_path = @log_dir + @"\" + @log_fi;
            using (System.IO.StreamWriter file_out = new System.IO.StreamWriter(fi_path))
            {
                foreach (string line in _logList)
                {
                    file_out.WriteLine(line);
                }
            }
        }

    }

    class CS_Logger
    {
        private static List<string> _logList = new List<string>();
        static readonly object _lockLog = new object();
        private static string _lastLogStr = " ";
        private static int _logCnt;
        public int logCnt
        { get { return _logCnt; } }

        public CS_Logger()
        {
            _logCnt = 0;
        }

        public void UpdateList(string log_str, long ts)
        {
            lock (_lockLog)
            {

                // Itterate count
                _logCnt++;

                // Add time and count
                string str = String.Format("[{0}],{1},", _logCnt, ts) + log_str;

                // Add to list
                _logList.Add(str);

                // Save log string
                _lastLogStr = str;

            }
        }

        public void SaveLog(string log_dir, string log_fi)
        {
            string fi_path = @log_dir + @"\" + @log_fi;
            using (System.IO.StreamWriter file_out = new System.IO.StreamWriter(fi_path))
            {
                foreach (string line in _logList)
                {
                    file_out.WriteLine(line);
                }
            }
        }

    }

    #endregion

}