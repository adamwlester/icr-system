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

        #region ============ DEBUG SETTINGS =============

        struct DB
        {
            // Set System Test
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
            public bool do_debugMat;
            // Breakpoint line for matlab debugging
            public int breakLine;


            // Print all blocked vt recs
            public bool do_printBlockedVt;
            // Print all sent rat vt recs
            public bool do_printSentRatVT;
            // Print all sent rob vt recs
            public bool do_printSentRobVT;
            // Print feederrobot log
            public bool do_printRobLog;
            // Print cheetahdue log
            public bool do_printDueLog;

            // Constructor:
            public DB(
                double system_test,
                bool do_debug_mat,
                int break_line,
                bool do_print_blocked_vt,
                bool do_print_sent_rat_vt,
                bool do_print_sent_rob_vt,
                bool do_print_rob_log,
                bool do_print_due_log
                )
            {
                systemTest = system_test;
                do_debugMat = do_debug_mat;
                breakLine = break_line;
                do_printBlockedVt = do_print_blocked_vt;
                do_printSentRatVT = do_print_sent_rat_vt;
                do_printSentRobVT = do_print_sent_rob_vt;
                do_printRobLog = do_print_rob_log;
                do_printDueLog = do_print_due_log;
            }
        }
        private static DB db = new DB(
            system_test: 0,
            do_debug_mat: true,
            break_line: 0, // 10000
            do_print_blocked_vt: true,
            do_print_sent_rat_vt: false,
            do_print_sent_rob_vt: false,
            do_print_rob_log: false,
            do_print_due_log: false
            );

        #endregion

        #region ============= TOP LEVEL VARS ============

        // Create stop watch object
        private static Stopwatch sw_main = new Stopwatch();
        private static long t_sync = 0;

        // Create lock objects and thread lists for safe threading
        static readonly object lock_sendMCOM = new object();
        static readonly object lock_getMCOM = new object();
        static readonly object lock_sendPack = new object();
        static readonly object lock_queue_getMCOM = new object();
        static readonly object lock_queue_sendMCOM = new object();
        static readonly object lock_queue_sendPack = new object();
        private static int queue_getMCOM = 0;
        private static int queue_sendMCOM = 0;
        private static int queue_sendPack = 0;
        static readonly object lock_checkConf = new object();
        static readonly object lock_checkDone = new object();
        static readonly object lock_printLog = new object();

        // Initialize vt blocking object
        private static VT_Blocker vtBlocker = new VT_Blocker(_stop_watch: sw_main);

        // Initialize FC to track program flow
        private static Flow_Control fc = new Flow_Control(_lock_print_log: lock_printLog);

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

        // Define NetCom vars
        private static MNetComClient com_netComClient = new MNetComClient();
        private static string NETCOM_APP_ID = "ICR_Run"; // string displayed in Cheetah when connected
        private static string NETCOM_ACQ_ENT_1 = "VT1"; // aquisition entity to stream
        private static string NETCOM_ACQ_ENT_2 = "VT2"; // aquisition entity to stream
        private static string NETCOM_IP = "127.0.0.1"; // host computer IP

        // Create logging objects
        private static DB_Logger robLogger = new DB_Logger(_stop_watch: sw_main);
        private static DB_Logger dueLogger = new DB_Logger(_stop_watch: sw_main);
        private static DB_Logger csLogger = new DB_Logger(_stop_watch: sw_main);
        private static UnionHack logBytes = new UnionHack(0, 0, 0, '0', 0);

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
            _stop_watch: sw_main,
            _lock_check_conf: new object(),
            _lock_check_done: new object(),
            _id:
            new char[16]{ // prefix giving masage id
            'h', // setup handshake [NA]
            'p', // simulation data [ts, x, y]
            'G', // matlab gui loaded [NA]
            'A', // connected to AC computer [NA]
            'N', // netcom setup [NA]
            'F', // data saved [NA]
            'X', // confirm quit
            'C', // confirm close
            'T', // system test command [(byte)test]
            'S', // setup session [(byte)ses_cond, (byte)sound_cond]
            'M', // move to position [(float)targ_pos]
            'R', // run reward [(float)rew_pos, (byte)zone_ind, (byte)rew_delay]
            'H', // halt movement [(byte)halt_state]
            'B', // bulldoze rat [(byte)bull_delay, (byte)bull_speed]
            'I', // rat in/out [(byte)in/out]
            'O'  // confirm rat out [NA]
             }
            );

        // CS to Matlab
        private static Com_Track c2m = new Com_Track(
            _stop_watch: sw_main,
            _lock_check_conf: new object(),
            _lock_check_done: new object(),
            _id:
            new char[9] {
            'g', // request m2c data
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

        // CS to Robot
        private static Com_Track c2r = new Com_Track(
            _stop_watch: sw_main,
            _lock_check_conf: lock_checkConf,
            _lock_check_done: lock_checkDone,
            _id:
            new char[15] {
            'h', // setup handshake
            'T', // system test command
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // run reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'V', // request stream status
            'L', // request log conf/send
            'J', // battery voltage
            'Z', // reward zone
            'U', // log size
            'P', // position data
             },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // Robot to CS
        private static Com_Track r2c = new Com_Track(
            _stop_watch: sw_main,
            _lock_check_conf: lock_checkConf,
            _lock_check_done: lock_checkDone,
            _id:
            new char[16] {
            'h', // setup handshake
            'T', // system test command
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // run reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'V', // connected and streaming
            'L', // request log conf/send
            'J', // battery voltage
            'Z', // reward zone
            'U', // log size
            'P', // position data
            'D', // execution done
             },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // Robot to Ard
        private static Com_Track r2a = new Com_Track(
            _stop_watch: sw_main,
            _lock_check_conf: new object(),
            _lock_check_done: new object(),
            _id:
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
            _stop_watch: sw_main,
            _lock_check_conf: new object(),
            _lock_check_done: new object(),
            _id:
            new char[1] {
            ' '
            },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // General communication
        private static long dt_sendSent = 5; // (ms)
        private static long dt_sendRcvd = 1; // (ms)
        private static long dt_resend = 500; // (ms)
        private static int resendMax = 5;
        private static long timeoutLoadGUI = 15000; // (ms)
        private static long timeoutConnectAC = 15000; // (ms)
        private static long timeoutConnectMatNLX = 30000; // (ms)
        private static long timeoutMatCloseConfirm = 10000; // (ms)
        private static long timeoutImportLog = 10000; // (ms)

        // Position variables
        private static double X_CENT = 359.5553;
        private static double Y_CENT = 260.2418;
        private static double RADIUS = 179.4922;
        private static double feedDist = 66 * ((2 * Math.PI) / (140 * Math.PI));
        private static ulong vtStr = 0;
        private static float[,] vtRad = new float[2, 2];
        private static float[] vtCM = new float[2];
        private static int[,] vtTS = new int[2, 2];

        #endregion

        #region ================= MAIN ==================

        // MAIN
        static void Main()
        {

            // SETUP
            LogEvent("[MAIN] RUNNING: SETUP...");
            bool passed_setup = Setup();
            if (passed_setup)
                LogEvent("[MAIN] FINISHED: SETUP");
            else if (!fc.isRunError)
                LogEvent("**WARNING!! [MAIN] ABORTED: SETUP");
            else
                fc.errStr = LogEvent("!!ERROR!! [MAIN] FAILED: SETUP");

            // RUN 
            if (passed_setup)
            {
                LogEvent("[MAIN] RUNNING: RUN...");
                bool passed_run = Run();
                if (passed_run)
                    LogEvent("[MAIN] FINISHED: RUN");
                else if (!fc.isRunError)
                    LogEvent("**WARNING!! [MAIN] ABORTED: RUN");
                else
                    fc.errStr = LogEvent("!!ERROR!! [MAIN] FAILED: RUN");
            }

            // EXIT 
            LogEvent("[MAIN] RUNNING: EXIT...");
            Exit();
            LogEvent("[MAIN] FINISHED: EXIT");

            // Pause before final exit
            Thread.Sleep(1000);

        }

        // SETUP
        public static bool Setup()
        {
            // Local vars
            bool pass;
            char id = ' ';
            double[] dat = new double[3] { 0, 0, 0 };
            ushort pack = 0;

            // Start timer
            sw_main.Start();

            // Set buffer height
            Console.BufferHeight = Int16.MaxValue - 1;

            // Create temp directory
            System.IO.Directory.CreateDirectory(logDir);
            LogEvent(String.Format("[Setup] FINISHED: Create Temporary Log Directory: \"{0}\"", logDir));

            // Initalize Matlab global vars
            LogEvent("[Setup] RUNNNING: Create mCOM Global Variables...");
            lock (lock_queue_sendMCOM) queue_sendMCOM++;
            lock (lock_sendMCOM)
            {
                System.Array _m2c_pack = new double[5] { 0, 0, 0, 0, 0 };
                com_Matlab.PutWorkspaceData("m2c_pack", "global", _m2c_pack);
                com_Matlab.PutWorkspaceData("m2c_dir", "global", " ");
            }
            lock (lock_queue_sendMCOM) queue_sendMCOM--;
            LogEvent("[Setup] FINISHED: Create mCOM Global Variables");

            // Setup and start CheetahDue serial
            LogEvent("[Setup] RUNNING: Setup CheetahDue Serial Coms and Logging...");
            sp_cheetahDue.ReadTimeout = 100;
            sp_cheetahDue.BaudRate = 57600;
            sp_cheetahDue.PortName = "COM14";
            // Open serial port connection
            sp_cheetahDue.Open();
            // Start getting new data on seperate thread
            new Thread(delegate ()
            {
                ParserA2C();
            }).Start();
            LogEvent("[Setup] FINISHED: Setup CheetahDue Serial Coms and Logging");

            // Setup and start Xbee serial
            LogEvent("[Setup] RUNNING: Setup Xbee Serial Coms");
            sp_Xbee.ReadTimeout = 100;
            sp_Xbee.BaudRate = 57600;
            sp_Xbee.PortName = "COM92";
            // Create event handeler for incoming data
            //sp_Xbee.DataReceived += DataReceived_Xbee;
            // Set byte threshold to max packet size
            sp_Xbee.ReceivedBytesThreshold = 7;
            // Open serial port connection
            sp_Xbee.Open();
            // Spin up parser thread
            new Thread(delegate ()
            {
                ParserR2C();
            }).Start();
            LogEvent("[Setup] FINISHED: Setup Xbee Serial Coms");

            // Setup debugging
            if (db.systemTest != 0 || db.do_debugMat)
            {
                LogEvent("[Setup] RUNNING: Setup Debugging...");

                // Hide/show matlab app window
                com_Matlab.Visible = 1;

                // Set MATLAB break point
                if (db.breakLine != 0)
                    SendMCOM(msg: String.Format("dbstop in ICR_GUI at {0};", db.breakLine));

                LogEvent("[Setup] FINISHED: Setup Debugging");
            }
            else com_Matlab.Visible = 0;

            // Setup ICR_GUI background worker
            LogEvent("[Setup] RUNNING: Start RunGUI Worker...");
            bw_RunGUI.DoWork += DoWork_RunGUI;
            bw_RunGUI.RunWorkerCompleted += RunWorkerCompleted_RunGUI;
            // Start ICR_GUI worker
            bw_RunGUI.RunWorkerAsync();
            LogEvent("[Setup] FINISHED: Start RunGUI Worker");

            // Setup MATLAB com background worker
            LogEvent("[Setup] START: Start MatCOM Worker...");
            bw_MatCOM.DoWork += DoWork_MatCOM;
            bw_MatCOM.ProgressChanged += ProgressChanged_MatCOM;
            bw_MatCOM.RunWorkerCompleted += RunWorkerCompleted_MatCOM;
            bw_MatCOM.WorkerReportsProgress = true;
            var bw_args = Tuple.Create(id, dat[0], dat[1], dat[2], pack);
            bw_MatCOM.RunWorkerAsync(bw_args);
            LogEvent("[Setup] FINISHED: Start MatCOM Worker...");

            // Wait for matlab handshake
            LogEvent("[Setup] RUNNING: Wait for ICR_GUI Handshake...");
            pass = WaitForMCOM(id: 'h', do_abort: true, timeout: 15000);
            if (pass)
            {
                LogEvent("[Setup] FINISHED: Wait for ICR_GUI Handshake...");
                fc.isMatComActive = true;
            }
            else
            {
                LogEvent("**WARNING** [Setup] ABORTED: Wait for ICR_GUI Handshake...");
                return false;
            }

            // Send CheetahDue message
            LogEvent("[Setup] RUNNING: Wait for Robot Handshake...");
            byte[] out_byte = new byte[1] { (byte)'h' };
            sp_cheetahDue.Write(out_byte, 0, 1);

            // Wait for sync confirmation from robot
            pass = WaitForSerial(id: 'h', do_abort: true, timeout: 5000);
            if (pass)
            {
                // Store sync time based on send time
                t_sync = c2r.t_sentRcvd[c2r.ID_Ind('h')];

                // Signal matlab to store sync time
                SendMCOM(id: 'W', dat_num: 1);

                // Log/print sync time
                LogEvent(String.Format("SET SYNC TIME: {0}ms", t_sync), t_sync);

                // Log/print success
                LogEvent("[Setup] FINISHED: Robot Handshake");
            }
            else
            {
                LogEvent("**WARNING** [Setup] ABORTED: Robot Handshake");
                return false;
            }

            // Start Cheetah if it is not already running
            LogEvent("[Setup] RUNNING: Run Cheetah.exe...");
            while (!IsProcessOpen("Cheetah") && !fc.doAbort)
            {
                OpenCheetah("Cheetah.cfg");
            }
            if (IsProcessOpen("Cheetah"))
                LogEvent("[Setup] FINISHED: Run Cheetah.exe");
            else
            {
                if (!fc.doAbort)
                {
                    fc.errStr = LogEvent("!!ERROR!! [Setup] FAILED: Run Cheetah.exe");
                    fc.isRunError = true;
                }
                else
                    LogEvent("**WARNING** [Setup] ABORTED: Run Cheetah.exe");
                return false;
            }

            // Initilize deligate for VT callback
            deligate_netComCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            com_netComClient.SetCallbackFunctionVT(deligate_netComCallback, new ICR_Run());

            // Wait for ICR_GUI to load 
            LogEvent("[Setup] RUNNING: Wait for ICR_GUI to Load...");
            pass = WaitForMCOM(id: 'G', timeout: timeoutLoadGUI);
            if (pass)
            {
                LogEvent("[Setup] FINISHED: Wait for ICR_GUI to Load");
            }
            else
            {
                LogEvent("**WARNING** [Setup] ABORTED: Wait for ICR_GUI to Load");
                return false;
            }

            // Wait for ICR_GUI to connect to AC computer
            LogEvent("[Setup] RUNNING: Wait for AC Connect...");
            pass = WaitForMCOM(id: 'A', do_abort: true, timeout: timeoutConnectAC);
            if (pass)
                LogEvent("[Setup] FINISHED: Wait for AC Connect");
            else
            {
                // Program timed out because matlab was hanging on connect
                LogEvent("**WARNING** [Setup] ABORTED: Wait for AC Connect");
                fc.isMAThanging = true;
                return false;
            }

            // Wait for ICR_GUI to connect to NLX
            LogEvent("[Setup] RUNNING: Wait for ICR_GUI NLX Setup...");
            pass = WaitForMCOM(id: 'N', do_abort: true, timeout: timeoutConnectMatNLX);
            if (pass)
                LogEvent("[Setup] FINISHED: Wait for ICR_GUI NLX Setup");
            else
            {
                LogEvent("**WARNING** [Setup] ABORTED: Wait for ICR_GUI NLX Setup");
                return false;
            }

            // Setup and begin NetCom streaming
            LogEvent("[Setup] RUNNING: Connect to NLX...");
            if (!(com_netComClient.AreWeConnected()))
                fc.isNlxConnected = com_netComClient.ConnectToServer(NETCOM_IP);
            if (fc.isNlxConnected)
                LogEvent("[Setup] FINISHED: Connect to NLX");
            else
            {
                fc.errStr = LogEvent("!!ERROR!! [Setup] FAILED: Connect to NLX");
                fc.isRunError = true;
                return false;
            }

            // Begin stream and set app name
            LogEvent("[Run] STARTING: Nlx Stream");
            com_netComClient.SetApplicationName(NETCOM_APP_ID);

            // Stream rat vt
            if (db.systemTest != 3)
            {
                LogEvent("[Run] STARTING: VT1 Stream");
                com_netComClient.OpenStream(NETCOM_ACQ_ENT_1);
            }

            // Stream rob vt
            LogEvent("[Run] STARTING: VT2 Stream");
            com_netComClient.OpenStream(NETCOM_ACQ_ENT_2);

            // Send streaming check request on seperate thread
            LogEvent("[Run] RUNNING: Confirm Robot Streaming...");
            RepeatSendPack(id: 'V', do_check_done: true);
            // Wait for confirmation from robot
            pass = WaitForSerial(id: 'V', do_abort: true, timeout: 5000);
            if (pass)
            {
                // Send confirm stream to Matlbab
                SendMCOM(id: 'V', dat_num: 1);
                LogEvent("[Run] FINISHED: Confirm Robot Streaming");
            }
            else
            {
                LogEvent("**WARNING** [Run] ABORTED: Confirm Robot Streaming");
                return false;
            }

            // Setup succesfull
            return true;

        }

        // RUN
        public static bool Run()
        {
            // Local vars
            bool pass;

            // Wait for setup command confirmation
            LogEvent("[Run] RUNNING: Confirm Setup...");
            pass = WaitForMCOM(id: 'S', do_abort: true);
            if (pass)
                pass = WaitForSerial(id: 'S', do_abort: true);
            if (pass) LogEvent("[Run] FINISHED: Confirm Setup");
            else
            {
                LogEvent("**WARNING** [Run] ABORTED: Confirm Setup");
                return false;
            }

            // Wait for initial move to command to complete
            LogEvent("[Run] RUNNING: MoveTo Start...");
            pass = WaitForMCOM(id: 'M', do_abort: true);
            if (pass)
                pass = WaitForSerial(id: 'M', do_abort: true);
            if (pass)
            {
                // Send confirm robot in place to Matlbab
                SendMCOM(id: 'K', dat_num: 1);
                fc.isMovedToStart = true;
                LogEvent("[Run] FINISHED: MoveTo Start");
            }
            else
            {
                LogEvent("**WARNING** [Run] ABORTED: MoveTo Start");
                return false;
            }

            // Main holding loop
            LogEvent("[Run] RUNNING: Main Session Loop...");
            // Stay in loop till rat is out or error
            while (
                com_netComClient.AreWeConnected() &&
                !fc.isRatOut &&
                !fc.doAbort
                ) ;
            if (!fc.doAbort) LogEvent("[Run] FINISHED: Main Session Loop");
            else
            {
                if (com_netComClient.AreWeConnected())
                    LogEvent("**WARNING** [Run] ABORTED: Main Session Loop");
                else
                {
                    fc.errStr = LogEvent("!!ERROR!! [Run] FAILED: Main Session Loop Because NLX Disconnected");
                    fc.isRunError = true;
                }
                return false;
            }

            // Run succesfull
            return true;
        }

        // EXIT
        public static void Exit()
        {
            // Local vars
            bool pass;

            // Check if we have confirmed rat is out of icr
            if (fc.isRatIn && !fc.isRatOut)
            {
                LogEvent("[Run] RUNNING: Wait for Last Confirmation Rat is Out...");
                pass = WaitForMCOM(id: 'O', timeout: 10000);
                if (pass)
                    LogEvent("[Exit] FINISHED: Wait for Last Confirmation Rat is Out");
                else
                    LogEvent("**WARNING** [Exit] ABORTED: Wait for Last Confirmation Rat is Out");
            }

            // Wait for reply on any remaining sent packets
            LogEvent("[Run] RUNNING: Wait for Last Packets...");
            pass = WaitForSerial(id_arr: c2r.id, timeout: 5000);
            if (pass) LogEvent("[Run] FINISHED: Wait for Last Packets");
            else
            {
                LogEvent("**WARNING** [Run] ABORTED: Wait for Last Packets");
            }

            // MoveTo defualt pos
            if (fc.isMovedToStart)
            {
                LogEvent("[Exit] RUNNING: MoveTo South...");
                double move_to = CalcMove(4.7124 - feedDist);

                // Send move command on seperate thread and wait for done reply
                RepeatSendPack(id: 'M', dat1: move_to, do_check_done: true);

                // Wait for confirmation from robot
                pass = WaitForSerial(id: 'M', timeout: 10000);
                if (pass)
                    LogEvent("[Exit] FINISHED: MoveTo South");
                else
                {
                    LogEvent("**WARNING** [Exit] ABORTED: MoveTo South");
                }
            }

            // Wait 1 second to see move to plot
            Thread.Sleep(1000);

            // Shut down NetCom
            if (IsProcessOpen("Cheetah"))
            {
                LogEvent("[Exit] RUNNING: NetCom Disconnect...");
                //// Stop recording aquisition
                string reply = " ";
                com_netComClient.SendCommand("-StopRecording", ref reply);
                com_netComClient.SendCommand("-StopAcquisition", ref reply);

                // Close NetCom sreams
                com_netComClient.CloseStream(NETCOM_ACQ_ENT_1);
                com_netComClient.CloseStream(NETCOM_ACQ_ENT_2);

                // Disconnect from NetCom
                do { com_netComClient.DisconnectFromServer(); }
                while (com_netComClient.AreWeConnected() && !fc.doAbort);

                // Check if disconnect succesful
                if (!com_netComClient.AreWeConnected())
                {
                    fc.isNlxConnected = false;
                    LogEvent("[Exit] FINISHED: NetCom Disconnect");
                }
                else
                {
                    fc.errStr = LogEvent("!!ERROR!! [Exit] FAILED: NetCom Disconnect");
                    fc.isRunError = true;
                }
            }

            // Enable ICR_GUI save button
            if (!fc.doAbort)
            {
                SendMCOM(id: 'Y', dat_num: 1);
                LogEvent("[Exit] FINISHED: Save Enabled");
                fc.isSaveEnabled = true;
            }
            else
                LogEvent("**WARNING** [Exit] ABORTED: Save Enabled");

            // Wait for last packet
            LogEvent("[Exit] RUNNING: Wait for Last Pack...");
            pass = WaitForSerial(id_arr: c2r.id, timeout: 5000);
            if (pass)
                LogEvent("[Exit] FINISHED: Wait for Last Pack");
            else
                LogEvent("**WARNING** [Exit] ABORTED: Wait for Last Pack");

            // Send initial robot log request
            LogEvent("[Exit] RUNNING: Request Robot Log...");
            RepeatSendPack(id: 'L', dat1: 0);
            pass = WaitForSerial(id: 'L', timeout: 5000);
            if (pass)
            {
                // Wait for bytes to receive messages to be received
                LogEvent("[Exit] RUNNING: Request Robot Log...");
                long t_byte_cnt_timeout = sw_main.ElapsedMilliseconds + 5000;
                while (robLogger.bytesToRcv == 0 && sw_main.ElapsedMilliseconds < t_byte_cnt_timeout)
                    Thread.Sleep(1);
                pass = robLogger.bytesToRcv > 0;
            }

            // Start importing log on seperate thread
            if (pass)
            {
                // Start importing
                new Thread(delegate ()
                {
                    GetRobotLog();
                }).Start();

                // Tell robot to begin streaming log and wait for message to send
                RepeatSendPack(id: 'L', dat1: 1, do_conf: false);
                pass = WaitForSerial(id: 'L', timeout: 5000);
                LogEvent("[Exit] FINISHED: Request Robot Log");
            }
            else
            {
                fc.errStr = LogEvent("!!ERROR!! [Exit] FAILED: Request Robot Log");
                fc.isRunError = true;
            }

            // Wait for save complete
            if (fc.isSaveEnabled)
            {
                LogEvent("[Exit] RUNNING: Wait for ICR_GUI to Save...");
                while (!fc.isSesSaved && !fc.doAbort) ;
                if (fc.isSesSaved)
                {
                    LogEvent("[Exit] FINISHED: Wait for ICR_GUI to Save");

                    // Get NLX dir
                    dynamic nlx_rec_dir = GetMCOM(msg: "m2c_dir");
                    nlxRecDir = (string)nlx_rec_dir;

                    // Confirm log saved
                    LogEvent(String.Format("SET RECORDING DIR TO \"{0}\"", nlxRecDir));

                }
                else
                {
                    LogEvent("**WARNING** [Exit] ABORTED: Wait for ICR_GUI to Save");
                }
            }

            // Wait for quit command
            LogEvent("[Exit] RUNNING: Wait for ICR_GUI Quit command...");
            if (!fc.isGUIquit)
                pass = WaitForMCOM(id: 'X', do_abort: true);
            else
                pass = true;
            if (pass)
                LogEvent("[Exit] FINISHED: Wait for ICR_GUI Quit command");
            else
                LogEvent("**WARNING** [Exit] ABORTED: Wait for ICR_GUI Quit command");

            // Wait for robot log save to complete
            LogEvent("[Exit] RUNNING: Wait for Robot Log Save...");
            while (!robLogger.isFinished && !robLogger.isImportTimedout)
                Thread.Sleep(10);

            // Check if complete log was imported
            if (robLogger.isLogComplete)
                LogEvent(String.Format("[Exit] FINISHED: Wait for Robot Log Save: logged={0} dropped={1} dt_run={2}",
                    robLogger.cnt_logged, robLogger.cnt_dropped[1], robLogger.logDT));
            else if (robLogger.cnt_logged > 0)
                LogEvent(String.Format("**WARNING** [Exit] FINISHED: Wait for Robot Log Save: logged={0} dropped={1} dt_run={2}",
                    robLogger.cnt_logged, robLogger.cnt_dropped[1], robLogger.logDT));
            else
            {
                fc.errStr = LogEvent(String.Format("!!ERROR!! [Exit] FAILED: Wait for Robot Log Save: logged={0} dropped={1} dt_run={2}",
                    robLogger.cnt_logged, robLogger.cnt_dropped[1], robLogger.logDT));
                fc.isRunError = true;
            }

            // Send command for arduino to quit on seperate thread
            RepeatSendPack(id: 'Q');
            // Wait for quit confirmation from robot for fixed period of time
            LogEvent("[Exit] RUN: Confirm Robot Quit...");
            pass = WaitForSerial(id: 'Q', timeout: 5000);
            if (pass)
                LogEvent("[Exit] FINISHED: Confirm Robot Quit");
            else
                LogEvent("**WARNING** [Exit] ABORTED: Confirm Robot Quit");
            // Set flags
            fc.isRobComActive = false;
            fc.isArdComActive = false;

            // Send command for ICR_GUI to exit
            SendMCOM(id: 'E', dat_num: 1);
            LogEvent("[Exit] FINISHED: Tell ICR_GUI to Close");

            // Wait for GUI to close
            LogEvent("[Exit] RUNNING: Confirm ICR_GUI Closed...");
            pass = WaitForMCOM(id: 'C', timeout: timeoutMatCloseConfirm);
            if (pass)
                LogEvent("[Exit] FINISHED: Confirm ICR_GUI Closed");
            else
                LogEvent("**WARNING** [Exit] ABORTED: Confirm ICR_GUI Closed");

            // Tell Matlab close confirmation received
            LogEvent("[Exit] RUNNING: Send ICR_GUI Close Confirmation Received...");
            SendMCOM(id: 'C', dat_num: 1);
            pass = WaitForMCOM(id: 'C', do_send_check: true, timeout: timeoutMatCloseConfirm);
            if (pass)
                LogEvent("[Exit] FINISHED: Send ICR_GUI Close Confirmation Received");
            else
                LogEvent("**WARNING** [Exit] ABORTED: Send ICR_GUI Close Confirmation Received");

            // Set exit flag to exit all threads
            fc.doExit = true;

            // Wait for Matlab and threads to close down
            Thread.Sleep(1000);

            // Dispose of workers
            //bw_RunGUI.CancelAsync();
            bw_RunGUI.Dispose();
            LogEvent("[Exit] FINISHED: Dispose RunGUI Worker");
            //bw_MatCOM.CancelAsync();
            bw_MatCOM.Dispose();
            LogEvent("[Exit] FINISHED: Dispose MatCOM Worker");

            // Clear all MatCOM vars
            SendMCOM(msg: "clearvars - global;");
            SendMCOM(msg: "clearvars;");
            SendMCOM(msg: "close all;");
            LogEvent("[Exit] FINISHED: Clear MatCom Globals");

            // Hold for debugging or errors errors
            if (db.systemTest != 0 || db.do_debugMat || fc.isRunError)
            {
                fc.RunPauseForDB();
            }

            // Quit MatCOM
            if (!fc.isMAThanging)
            {
                Thread.Sleep(100);
                com_Matlab.Quit();
                LogEvent("[Exit] FINISHED: Close MatCOM");
            }

            // Kill that mother fucker!
            else
            {
                KillMatlab();
                LogEvent("**WARNING** [Exit] HAD TO KILL MATLAB");
            }

            // Save CheetahDue log file
            LogEvent("[Exit] RUNNING: Save CheetahDue Log...");
            dueLogger.SaveLog(logDir, dueLogFi);
            if (dueLogger.isLogComplete)
                LogEvent(String.Format("[Exit] FINISHED: Save CheetahDue Log: logged={0} dropped={1}",
                    dueLogger.cnt_logged, dueLogger.cnt_dropped[1]));
            else if (dueLogger.cnt_logged > 0)
                LogEvent(String.Format("**WARNING** [Exit] FINISHED: Save CheetahDue Log: logged={0} dropped={1}",
                    dueLogger.cnt_logged, dueLogger.cnt_dropped[1]));
            else
            {
                fc.errStr = LogEvent(String.Format("!!ERROR!! [Exit] FAILED: Save CheetahDue Log: logged={0} dropped={1}",
                    dueLogger.cnt_logged, dueLogger.cnt_dropped[1]));
                fc.isRunError = true;
            }

            // Save CS log file
            LogEvent("[Exit] RUNNING: Save CS Log...");
            csLogger.SaveLog(logDir, csLogFi);
            LogEvent(String.Format("[Exit] FINISHED: Save CS Log: logged={0}", csLogger.cnt_logged));

            // Copy log files to rat specific dir
            if (nlxRecDir != logDir)
            {
                // Create copies
                System.IO.File.Copy(System.IO.Path.Combine(logDir, robLogFi), System.IO.Path.Combine(nlxRecDir, robLogFi), true);
                System.IO.File.Copy(System.IO.Path.Combine(logDir, dueLogFi), System.IO.Path.Combine(nlxRecDir, dueLogFi), true);
                System.IO.File.Copy(System.IO.Path.Combine(logDir, csLogFi), System.IO.Path.Combine(nlxRecDir, csLogFi), true);
                System.IO.File.Copy(System.IO.Path.Combine(logDir, matLogFi), System.IO.Path.Combine(nlxRecDir, matLogFi), true);

                // Confirm logs copied saved
                LogEvent(String.Format("COPPIED LOG FILES TO \"{0}\"", nlxRecDir));
            }
        }

        #endregion

        #region ========= SERIAL COMMUNICATION ==========

        // SEND PACK DATA REPEATEDLY TILL RECIEVED CONFIRMED
        public static void RepeatSendPack(int send_max = 0, char id = ' ', double dat1 = double.NaN, double dat2 = double.NaN, double dat3 = double.NaN, ushort pack = 0, bool do_conf = true, bool do_check_done = false)
        {
            // Local vars
            double[] dat = new double[3] { dat1, dat2, dat3 };

            // Update c2r queue info
            c2r.SetCheckFor(id: id, do_check_sent_rcvd: true, do_check_conf: do_conf, do_check_done: do_check_done);

            // Run method on seperate thread
            new Thread(delegate ()
            {
                RepeatSendPack_Thread(send_max: send_max, id: id, dat: dat, pack: pack, do_conf: do_conf, do_check_done: do_check_done);
            }).Start();

        }
        public static void RepeatSendPack_Thread(int send_max, char id, double[] dat, ushort pack, bool do_conf, bool do_check_done)
        {
            long t_resend = sw_main.ElapsedMilliseconds + dt_resend;
            int send_count = 1;

            // Specify max send attempts
            send_max = send_max == 0 ? resendMax : send_max;

            // Send new data with new packet number
            pack = SendPack(id: id, dat: dat, pack: pack, do_conf: do_conf, do_check_done: do_check_done);

            // Bail if not checking for confirmation
            if (!do_conf)
                return;

            // Keep checking mesage was received
            else
            {
                while (fc.ContinueRobCom())
                {

                    // Message confirmed
                    if (c2r.IsConfirmed(id))
                        return;

                    // Check if streaming has failed
                    else if (send_count >= resendMax)
                    {
                        fc.isRobComActive = false;
                        return;
                    }

                    // Need to resend
                    else if (sw_main.ElapsedMilliseconds > t_resend)
                    {
                        // Log/print
                        LogEvent(String.Format("**WARNING** [RepeatSendPack_Thread] Resending c2r: id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} do_conf={5} do_check_done={6}",
                            id, dat[0], dat[1], dat[2], pack, do_conf ? "true" : "false", do_check_done ? "true" : "false"));

                        // Resend
                        SendPack(id: id, dat: dat, pack: pack, do_conf: do_conf, do_check_done: do_check_done);
                        t_resend = sw_main.ElapsedMilliseconds + dt_resend;
                        send_count++;
                    }
                }
            }
        }

        // SEND PACK DATA
        public static ushort SendPack(char id, double[] dat, ushort pack, bool do_conf, bool do_check_done)
        {
            /* 
            SEND DATA TO ROBOT 
            FORMAT: head, id, data, packet_number, do_confirm, footer
            EXAMPLE: ASCII {'<','L','r','ÿ','ÿ','\0','>'} DEC {60,76,1,255,255,0,60}
            */

            // Track when data queued
            long t_queued = sw_main.ElapsedMilliseconds;
            lock (lock_queue_sendPack) queue_sendPack++;

            lock (lock_sendPack)
            {
                // Block vt sending
                vtBlocker.Block(id);

                // Local vars
                UnionHack U = new UnionHack(0, 0, 0, '0', 0);
                byte[] msg_id = new byte[1];
                byte[] msg_data = null;
                byte[] msg_pack = new byte[2];
                byte[] msg_conf = new byte[1];
                int n_dat_bytes = 0;
                long t_send = 0;
                bool do_loop = false;
                bool buff_ready = false;
                bool is_clogged = false;
                bool is_hanging = false;

                // Wait for next safe send time
                do
                {
                    // Delay send time till x ms after last send or rcvd
                    t_send = c2r.t_new > r2c.t_new ? c2r.t_new + dt_sendSent : r2c.t_new + dt_sendRcvd;

                    // Make sure outbut and input buffer have enough space
                    buff_ready = sp_Xbee.BytesToWrite < 1 && sp_Xbee.BytesToRead < 1;

                    // Check if loop should continue
                    do_loop =
                        (sw_main.ElapsedMilliseconds < t_send || !buff_ready) &&
                        fc.ContinueRobCom();

                    // Get status
                    is_clogged = queue_sendPack >= 3;
                    is_hanging = sw_main.ElapsedMilliseconds > t_queued + 100;

                    // Check if queue backed up or hanging
                    if (is_clogged || is_hanging)
                    {
                        // Log/print
                        if (is_clogged)
                            LogEvent(String.Format("**WARNING** [SendPack] c2r Queue Clogged: id=\'{0}\' tx={1} rx={2} queued={3} queue_dt={4}",
                                id, sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, queue_sendPack, sw_main.ElapsedMilliseconds - t_queued));
                        if (is_hanging)
                            LogEvent(String.Format("**WARNING** [SendPack] c2r Queue Hanging: id=\'{0}\' tx={1} rx={2} queued={3} queue_dt={4}",
                            id, sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, queue_sendPack, sw_main.ElapsedMilliseconds - t_queued));

                        // Bail if this is pos data
                        if (id == 'P')
                        {
                            lock (lock_queue_sendPack) queue_sendPack--;
                            return pack;
                        }
                    }

                } while (do_loop);

                // Get new packet number
                if (pack == 0)
                {
                    c2r.cnt_pack++;
                    pack = c2r.cnt_pack;
                }

                // Store pack number
                U.s1 = pack;
                msg_pack[0] = U.b1;
                msg_pack[1] = U.b2;

                // Store ID
                U.c1 = id;
                msg_id[0] = U.b1;

                // Send system test command
                if (id == 'T')
                {
                    n_dat_bytes = 2;
                    msg_data = new byte[n_dat_bytes];
                    // Add test id
                    msg_data[0] = (byte)dat[0];
                    // Add test parameter
                    msg_data[1] = (byte)dat[1];
                }

                // Send setup data
                if (id == 'S')
                {
                    n_dat_bytes = 2;
                    msg_data = new byte[n_dat_bytes];
                    // Add session cond
                    msg_data[0] = (byte)dat[0];
                    // Add sound cond
                    msg_data[1] = (byte)dat[1];
                }

                // Send MoveTo data
                else if (id == 'M')
                {
                    n_dat_bytes = 4;
                    msg_data = new byte[n_dat_bytes];
                    // Add move pos
                    U.f = (float)dat[0];
                    msg_data[0] = U.b1;
                    msg_data[1] = U.b2;
                    msg_data[2] = U.b3;
                    msg_data[3] = U.b4;
                }

                // Send Reward data
                else if (id == 'R')
                {
                    n_dat_bytes = 6;
                    msg_data = new byte[n_dat_bytes];
                    // Add rew pos
                    U.f = (float)dat[0];
                    msg_data[0] = U.b1;
                    msg_data[1] = U.b2;
                    msg_data[2] = U.b3;
                    msg_data[3] = U.b4;
                    // Add zone ind
                    msg_data[4] = (byte)dat[1];
                    // Add reward delay
                    msg_data[5] = (byte)dat[2];
                }

                // Send halt motor data
                if (id == 'H')
                {
                    n_dat_bytes = 1;
                    msg_data = new byte[n_dat_bytes];
                    // Add halt state
                    msg_data[0] = (byte)dat[0];
                }

                // Send bulldoze rat data
                if (id == 'B')
                {
                    n_dat_bytes = 2;
                    msg_data = new byte[n_dat_bytes];
                    // Add bull del
                    msg_data[0] = (byte)dat[0];
                    // Add bull speed
                    msg_data[1] = (byte)dat[1];
                }

                // Send rat in/out
                if (id == 'I')
                {
                    n_dat_bytes = 1;
                    msg_data = new byte[n_dat_bytes];
                    // Add session cond
                    msg_data[0] = (byte)dat[0];
                }

                // Send log request
                if (id == 'L')
                {
                    n_dat_bytes = 1;
                    msg_data = new byte[n_dat_bytes];
                    // Add conf/send request
                    msg_data[0] = (byte)dat[0];
                }

                // Send pos data
                else if (id == 'P')
                {
                    n_dat_bytes = 9;
                    msg_data = new byte[n_dat_bytes];
                    // Add vtEnt byte
                    msg_data[0] = (byte)dat[0];
                    // Add vtTS int 
                    U.i = (int)dat[1];
                    msg_data[1] = U.b1;
                    msg_data[2] = U.b2;
                    msg_data[3] = U.b3;
                    msg_data[4] = U.b4;
                    // Add vtCM float 
                    U.f = (float)dat[2];
                    msg_data[5] = U.b1;
                    msg_data[6] = U.b2;
                    msg_data[7] = U.b3;
                    msg_data[8] = U.b4;
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
                sp_Xbee.Write(msgByteArr, 0, msgByteArr.Length);

                // Update c2r info
                c2r.UpdateSentRcvd(id: id, pack: pack, t: sw_main.ElapsedMilliseconds);

                // Print sent data
                string dat_str;

                // Print sent mesage packet
                dat_str = String.Format("id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}|", id, dat[0], dat[1], dat[2]);

                // Store common info
                string end_str = String.Format(" pack={0} do_conf={1} do_check_done={2} bytes_sent={3} tx={4} rx={5} queued={6} queue_dt={7}",
                pack, do_conf ? "true" : "false", do_check_done ? "true" : "false", msgByteArr.Length, sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, queue_sendPack, sw_main.ElapsedMilliseconds - t_queued);

                // Check if sending pos data
                if (id != 'P')
                {
                    // Check for resend
                    if (pack != c2r.packLast[c2r.ID_Ind(id)])
                        LogEvent("   [SENT] c2r: " + dat_str + end_str, c2r.t_new, c2r.t_last);
                    else
                    {
                        c2r.cnt_repeat++;
                        LogEvent(String.Format("   [*RE-SENT*] c2r: cnt={0} ", c2r.cnt_repeat) + dat_str + end_str, c2r.t_new, c2r.t_last);
                    }

                }
                else
                {
                    // Track send rate
                    vtBlocker.StoreSendTime((int)dat[0], sw_main.ElapsedMilliseconds);

                    // Log/print
                    if ((db.do_printSentRatVT && (int)dat[0] == 0) ||
                        (db.do_printSentRobVT && (int)dat[0] == 1))
                    {
                        dat_str = String.Format("id=\'{0}\' vtEnt={1} vtTS={2} vtCM={3:0.00} dt_send_mu={4}",
                            id, (int)dat[0], vtTS[(int)dat[0], 1], vtCM[(int)dat[0]], vtBlocker.GetSendDT((int)dat[0], "avg"));
                        LogEvent("   [SENT] c2r: " + dat_str + end_str, c2r.t_new, c2r.t_last);
                    }
                }

                // Unlock vt sending
                vtBlocker.Unblock(id);

            }
            lock (lock_queue_sendPack) queue_sendPack--;

            // Return packet number
            return pack;

        }

        // WAIT FOR R2C CONFIRMATION
        public static bool WaitForSerial(char id, bool do_send_check = true, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            char[] id_arr = { id };
            return WaitForSerial(id_arr: id_arr, do_send_check: do_send_check, do_abort: do_abort, timeout: timeout);
        }
        public static bool WaitForSerial(char[] id_arr, bool do_send_check = false, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_start = sw_main.ElapsedMilliseconds;
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : t_start + timeout;
            char id = ' ';
            bool first_loop = true;
            bool is_sent = false;
            bool is_conf = false;
            bool is_done = false;
            string wait_str = " ";

            // Loop through each id
            for (int i = 0; i < id_arr.Length; i++)
            {
                // Current id
                id = id_arr[i];
                is_sent = false;
                first_loop = true;

                // Wait for confirmation
                do
                {
                    // Check if sent
                    if (do_send_check)
                        is_sent = !is_sent ? c2r.IsSentRcvd(id) : is_sent;
                    else
                        is_sent = true;

                    // Check if received confirm and/or done
                    is_conf = !is_conf ? is_sent && c2r.IsConfirmed(id) : is_conf;
                    is_done = !is_done ? is_sent && c2r.IsDone(id) : is_done;

                    // Check if waiting on current id
                    if (first_loop)
                    {
                        wait_str = String.Format("|{0}{1}{2}", do_send_check ? "Sent|" : "", !c2r.IsConfirmed(id) ? "Conf|" : "", !c2r.IsDone(id) ? "Done|" : "");
                        if (!(is_conf && is_done))
                            LogEvent(String.Format("[WaitForSerial] RUNNING: Wait for {0}: id=\'{1}\' do_send_check=%s do_abort=%s timeout=%d...",
                                wait_str, id, do_send_check ? "true" : "false", do_abort ? "true" : "false", timeout == long.MaxValue ? 0 : timeout));
                        else break;
                        first_loop = false;
                    }

                    // Check if received
                    if (is_conf && is_done)
                    {
                        LogEvent(String.Format("[WaitForSerial] FINISHED: Wait for {0}: id=\'{1}\' pack={2} is_sent={3} is_conf={4} is_done={5} dt_wait={6}",
                            wait_str, id, c2r.pack[c2r.ID_Ind(id)], is_sent ? "true" : "false", is_conf ? "true" : "false", is_done ? "true" : "false", sw_main.ElapsedMilliseconds - t_start));
                        break;
                    }

                    // Check if need to abort
                    else if (
                        (do_abort && fc.doAbort) ||
                        !fc.ContinueRobCom() ||
                        sw_main.ElapsedMilliseconds > t_timeout
                        )
                    {

                        // Store details
                        string dat_str = String.Format("id=\'{0}\' pack={1} is_sent={2} is_conf={3} is_done={4} dt_wait={5}",
                            id, c2r.pack[c2r.ID_Ind(id)], is_sent ? "true" : "false", is_conf ? "true" : "false", is_done ? "true" : "false", sw_main.ElapsedMilliseconds - t_start);


                        // External forced abort
                        if (do_abort && fc.doAbort)
                            LogEvent(String.Format("**WARNING** [WaitForSerial] Forced Abort: Wait for {0}: {1}", wait_str, dat_str));
                        else
                        {
                            // Coms failed
                            if (!fc.ContinueRobCom())
                                fc.errStr = LogEvent(String.Format("!!ERROR!! [WaitForSerial] Lost Comms: Wait for {0}: {1}", wait_str, dat_str));

                            // Timedout
                            else if (sw_main.ElapsedMilliseconds > t_timeout)
                                fc.errStr = LogEvent(String.Format("!!ERROR!! [WaitForSerial] Timedout: Wait for {0}: {1}", wait_str, dat_str));

                            // Set error flag
                            fc.isRunError = true;
                        }

                        // Reset flags so we dont check for this again
                        c2r.ResetCheckFor(id: id, is_conf: true, is_done: true);

                        // Set abort flag and bail  
                        return false;
                    }

                    // Pause thread
                    else

                        Thread.Sleep(1);

                } while (true);

            }

            // All confirmed
            return true;
        }

        //// EVENT HANDELER FOR RECIEVED XBEE DATA 
        //public static void DataReceived_Xbee(object sender, SerialDataReceivedEventArgs e)
        //{
        //    // Bail if processing robot log
        //    if (robLogger.isLogging)
        //        return;

        //    // Run method on seperate thread
        //    new Thread(delegate ()
        //    {
        //        ParseR2C();
        //    }).Start();

        //}

        // PARSE RECIEVED XBEE DATA 
        public static void ParserR2C()
        {
            /* 
            RECIEVE DATA FROM FEEDERDUE 
            FORMAT: head, id, return_confirm, data, packet_number, footer
            */

            // Loop till all data read out
            while (!fc.doExit)
            {

                // Bail if no new data or processing robot log
                if (sp_Xbee.BytesToRead < 1 ||
                    !fc.ContinueRobCom() ||
                    robLogger.isLogging)
                {
                    Thread.Sleep(1);
                    continue;
                }

                // Local vars
                long t_parse_str = sw_main.ElapsedMilliseconds;
                UnionHack U = new UnionHack(0, 0, 0, '0', 0);
                bool head_found = false;
                bool id_found = false;
                bool foot_found = false;
                bool for_ard = false;
                byte[] head_bytes = new byte[1];
                byte[] id_bytes = new byte[1];
                byte[] conf_bytes = new byte[1];
                byte[] dat_bytes = new byte[3];
                byte[] pack_bytes = new byte[2];
                byte[] foot_bytes = new byte[1];
                int bytes_read = 0;
                char head = ' ';
                char id = ' ';
                byte[] dat = new byte[3];
                ushort pack = 0;
                bool do_conf = false;
                char foot = ' ';

                // Get header
                if (XbeeBuffReady(1, t_parse_str))
                {
                    sp_Xbee.Read(head_bytes, 0, 1);
                    bytes_read += 1;
                    // Get header
                    U.b1 = head_bytes[0];
                    U.b2 = 0; // C# chars are 2 bytes
                    head = U.c1;
                    if (head == r2c.head[0])
                    {
                        head_found = true;
                    }
                    else if (head == r2a.head[0])
                    {
                        for_ard = true;
                    }
                }

                // Find id and check message is intended for CS
                if (head_found)
                {
                    if (XbeeBuffReady(1, t_parse_str))
                    {
                        if (head_found)
                        {
                            sp_Xbee.Read(id_bytes, 0, 1);
                            bytes_read += 1;
                            // Get id
                            U.b1 = id_bytes[0];
                            U.b2 = 0;
                            id = U.c1;
                            // Check for match
                            for (int i = 0; i < r2c.id.Length; i++)
                            {
                                if (id == r2c.id[i])
                                {
                                    id_found = true;
                                }
                            }
                        }
                    }
                }

                // Check if this is a log packet
                if (head_found && id_found)
                {

                    // Get data
                    if (XbeeBuffReady(3, t_parse_str))
                    {
                        // Read in data
                        sp_Xbee.Read(dat_bytes, 0, 3);
                        bytes_read += 3;
                        dat[0] = dat_bytes[0];
                        dat[1] = dat_bytes[1];
                        dat[2] = dat_bytes[2];
                    }

                    // Get packet number
                    if (XbeeBuffReady(2, t_parse_str))
                    {

                        // Read in data
                        sp_Xbee.Read(pack_bytes, 0, 2);
                        bytes_read += 2;
                        U.b1 = pack_bytes[0];
                        U.b2 = pack_bytes[1];
                        pack = U.s1;
                    }

                    // Get do confirm byte
                    if (XbeeBuffReady(1, t_parse_str))
                    {
                        sp_Xbee.Read(conf_bytes, 0, 1);
                        bytes_read += 1;
                        // Get bool
                        do_conf = conf_bytes[0] == 1 ? true : false;
                    }

                    // Find footer
                    if (XbeeBuffReady(1, t_parse_str))
                    {
                        // Read in data
                        sp_Xbee.Read(foot_bytes, 0, 1);
                        bytes_read += 1;

                        // Check footer
                        U.b1 = foot_bytes[0];
                        U.b2 = 0;
                        foot = U.c1;
                        if (foot == r2c.foot[0])
                        {
                            foot_found = true;
                        }
                    }

                    // Store packet data
                    if (foot_found)
                    {
                        // Update flags
                        r2c.UpdateSentRcvd(id: id, pack: pack, t: sw_main.ElapsedMilliseconds);

                        // Check for repeat packet
                        string msg_str;
                        if (pack != r2c.packLast[r2c.ID_Ind(id)])
                        {
                            msg_str = String.Format("   [RCVD] r2c: id=\'{0}\' dat=|{1}|{2}|{3}| pack={4} do_conf={5} bytes_read={6} rx={7} tx={8} parse_dt={9}",
                            id, dat[0], dat[1], dat[2], pack, do_conf ? "true" : "false", bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, sw_main.ElapsedMilliseconds - t_parse_str);
                        }
                        else
                        {
                            r2c.cnt_repeat++;
                            msg_str = String.Format("   [*RE-RCVD*] r2c: cnt={0} id=\'{1}\' dat=|{2}|{3}|{4}| pack={5} do_conf={6} bytes_read={7} rx={8} tx={9} parse_dt={10}",
                            r2c.cnt_repeat, id, dat[0], dat[1], dat[2], pack, do_conf ? "true" : "false", bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, sw_main.ElapsedMilliseconds - t_parse_str);
                        }

                        // Log/print rcvd details
                        LogEvent(msg_str, r2c.t_new, r2c.t_last);

                        // Check if this is a recieve confirmation
                        if (c2r.ID_Ind(id) != -1)
                            c2r.ResetCheckFor(id: id, is_conf: true);

                        // Check if this is a done confirmation
                        if (id == 'D')
                            c2r.ResetCheckFor(id: c2r.PackID(pack), is_done: true);

                        // Send recieve confirmation
                        if (do_conf)
                            RepeatSendPack(send_max: 1, id: id, dat1: dat[0], dat2: dat[1], dat3: dat[2], pack: pack, do_conf: false);

                        // Check if data should be relayed to Matlab
                        if (c2m.ID_Ind(id) != -1)
                            SendMCOM(id: id, dat_num: dat[0]);

                        // Store data bytes to be sent for logging
                        if (id == 'U')
                            // Store data byte
                            robLogger.UpdateBytesToRcv(dat);

                    }
                }

                // If no bytes read restart loop
                if (bytes_read == 0)
                    continue;

                // Dump incomplete packets
                if (!head_found || !id_found || !foot_found)
                {

                    // Check if data intended for CheetahDue
                    if (for_ard)
                    {
                        // Dump till header found
                        while (
                            !foot_found && sp_Xbee.BytesToRead > 0 &&
                            fc.ContinueRobCom()
                            )
                        {
                            sp_Xbee.Read(foot_bytes, 0, 1);
                            bytes_read += 1;
                            // Get footer
                            U.b1 = foot_bytes[0];
                            U.b2 = 0; // C# chars are 2 bytes
                            foot = U.c1;
                            if (foot == r2a.foot[0])
                            {
                                foot_found = true;
                            }
                        }

                        // Log/print ard message
                        LogEvent(String.Format("[ParseR2C] Received CheetaDue Packet: bytes_read={0} rx={1} tx={2} parse_dt={3}",
                        id, dat, pack, do_conf ? "true" : "false", bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, sw_main.ElapsedMilliseconds - t_parse_str));
                    }
                    else
                    {
                        // Add to count
                        r2c.AddDropped(1);

                        // Log/print available info
                        LogEvent(String.Format("**WARNING** [ParseR2C] Dropped r2c Packet: dropped={0}|{1} head={2} id=\'{3}\' dat=|{4}|{5}|{6}| pack={7} do_conf={8} foot={9} bytes_read={10} rx={11} tx={12} parse_dt={13}",
                            r2c.cnt_dropped[0], r2c.cnt_dropped[1], head, id, dat[0], dat[1], dat[2], pack, do_conf, foot, bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, sw_main.ElapsedMilliseconds - t_parse_str));

                        // Dump buffer if > 1 consecutive drops and no bytes read
                        if (r2c.cnt_dropped[0] > 1 && bytes_read == 0)
                        {
                            LogEvent("**WARNING** [ParseR2C] Dumping r2c Input Buffer");
                            sp_Xbee.DiscardInBuffer();
                        }

                        // Wait for buffer to refil
                        //XbeeBuffReady(6, t_parse_str); TEMP
                    }
                    // dump input buffer
                    //sp_Xbee.DiscardInBuffer();

                }

                // Change streaming status
                else if (!fc.isRobComActive)
                {
                    fc.isRobComActive = true;
                }
            }

        }

        // WAIT FOR XBEE BUFFER TO FILL
        public static bool XbeeBuffReady(int min_byte, long t_str, long timeout = 1000)
        {
            // Local vars
            long t_timeout = sw_main.ElapsedMilliseconds + timeout;
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_Xbee.BytesToRead < min_byte &&
                sw_main.ElapsedMilliseconds <= t_timeout &&
                fc.ContinueRobCom()
                ) ;

            // Check if timedout
            if (sw_main.ElapsedMilliseconds > t_timeout)
                LogEvent(String.Format("**WARNING** [XbeeBuffReady] r2c Hanging: min_byte={0} dt_check={1} dt_queue={2} rx={3} tx={4}",
                     min_byte, (sw_main.ElapsedMilliseconds - t_timeout) + timeout, sw_main.ElapsedMilliseconds - t_str, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite));

            // Check if buff filled
            pass = sp_Xbee.BytesToRead >= min_byte ? true : false;

            return pass;
        }

        // CONTINUALLY CHECK FOR NEW CHEETAHDUE LOG DATA
        public static void ParserA2C()
        {
            // Loop till all data read out
            while (fc.ContinueArdCom())
            {
                // Local vars
                UnionHack U = new UnionHack(0, 0, 0, '0', 0);
                bool head_found = false;
                bool foot_found = false;
                byte[] head_bytes = new byte[1];
                byte[] foot_bytes = new byte[1];
                byte[] chksum_bytes = new byte[1];
                int bytes_read = 0;
                char head = ' ';
                ushort chksum = 0;
                char foot = ' ';
                string log_str = " ";

                // Wait for new header
                while (!head_found && fc.ContinueArdCom())
                {
                    // Bail till new data found
                    if (sp_cheetahDue.BytesToRead < 1)
                        continue;

                    sp_cheetahDue.Read(head_bytes, 0, 1);
                    bytes_read += 1;
                    // Get header
                    U.b1 = head_bytes[0];
                    U.b2 = 0; // C# chars are 2 bytes
                    head = U.c1;
                    if (head == a2c.head[0])
                    {
                        head_found = true;
                    }
                }

                // Find id and check message is intended for CS
                if (head_found)
                {
                    // Get check sum
                    if (ArdBuffReady(1))
                    {
                        // Read in data
                        sp_cheetahDue.Read(chksum_bytes, 0, 1);
                        bytes_read += 1;
                        chksum = chksum_bytes[0];
                    }

                    // Get complete message
                    byte[] log_bytes = new byte[chksum];
                    if (ArdBuffReady(chksum))
                    {
                        // Read in all data
                        sp_cheetahDue.Read(log_bytes, 0, chksum);
                        bytes_read += chksum;

                    }
                    // Convert to string
                    log_str = System.Text.Encoding.UTF8.GetString(log_bytes);

                    // Find footer
                    if (ArdBuffReady(1))
                    {
                        // Read in data
                        sp_cheetahDue.Read(foot_bytes, 0, 1);
                        bytes_read += 1;

                        // Check footer
                        U.b1 = foot_bytes[0];
                        U.b2 = 0;
                        foot = U.c1;
                        if (foot == a2c.foot[0])
                        {
                            foot_found = true;
                        }
                    }
                }

                if (head_found && foot_found)
                {
                    // Update list
                    dueLogger.UpdateList(log_str);

                    // print data received
                    if (db.do_printDueLog)
                    {
                        // Update list
                        dueLogger.UpdateList(log_str);

                        // Print
                        if (db.do_printDueLog)
                            LogEvent(String.Format("   [LOG] a2c[{0}]: message=\"{1}\" chksum={2} bytes_read={3} rx={4} tx={5}",
                                dueLogger.cnt_logged, log_str, chksum, bytes_read, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));
                    }

                    // Change streaming status
                    if (!fc.isArdComActive)
                    {
                        fc.isArdComActive = true;
                    }
                }

                // Dump incomplete packets
                else if (bytes_read > 0)
                {
                    // Add to count
                    dueLogger.AddDropped(1);

                    // Print
                    LogEvent(String.Format("**WARNING** [GetArdLog] Dropped a2c Log: logged={0} dropped={1}|{2} head={3} message=\"{4}\" chksum={5} foot={6} bytes_read={7} rx={8} tx={9}",
                       dueLogger.cnt_logged, dueLogger.cnt_dropped[0], dueLogger.cnt_dropped[1], head, log_str, chksum, foot, bytes_read, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));

                    // Dump buffer if > 1 consecutive drops and no bytes read
                    if (dueLogger.cnt_dropped[0] > 1 && bytes_read == 0)
                    {
                        LogEvent("**WARNING** [GetArdLog] Dumping a2c Input Buffer");
                        sp_cheetahDue.DiscardInBuffer();
                    }
                }

            }
        }

        // WAIT FOR CHEETAHDUE BUFFER TO FILL
        public static bool ArdBuffReady(int min_byte, long timeout = 1000)
        {
            // Local vars
            long t_timeout = sw_main.ElapsedMilliseconds + timeout;
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_cheetahDue.BytesToRead < min_byte &&
                sw_main.ElapsedMilliseconds <= t_timeout &&
                (sp_cheetahDue.BytesToRead > 0 || fc.ContinueArdCom())
                ) ;

            // Check if buff filled
            pass = sp_cheetahDue.BytesToRead >= min_byte ? true : false;

            // Check for errors
            if (!pass)
            {
                // Timedout
                if (sw_main.ElapsedMilliseconds > t_timeout)
                    LogEvent(String.Format("**WARNING** [ArdBuffReady] a2c Hanging: dt_check={0} rx={1} tx={2}",
                      (sw_main.ElapsedMilliseconds - t_timeout) + timeout, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));
                else
                    LogEvent(String.Format("**WARNING** [ArdBuffReady] ABORTED: dt_check={0} rx={1} tx={2}",
                   (sw_main.ElapsedMilliseconds - t_timeout) + timeout, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));
            }

            return pass;
        }

        // RETRIEVE FEEDERDUE LOG
        public static void GetRobotLog()
        {
            // Local vars
            long t_stream_str = sw_main.ElapsedMilliseconds;
            long t_timeout = t_stream_str + timeoutImportLog;
            long t_read_last = t_stream_str;
            ushort conf_pack = 0;
            int msg_lng = 0;
            char[] c_arr = new char[3] { '\0', '\0', '\0' };

            // Prevent xBee event handeler from running
            robLogger.isLogging = true;

            // Start log import
            LogEvent("[GetRobotLog] RUNNING: Robot Log Import...");

            // Read stream till ">>>" string
            int read_ind = 0;
            char[] in_arr = new char[1000000];
            long dt_run = 0;
            long dt_read = 0;
            while (
                    !robLogger.isImportTimedout &&
                    fc.isRobComActive &&
                    conf_pack != r2c.pack[r2c.ID_Ind('D')])
            {
                // Check for timeout
                if (sw_main.ElapsedMilliseconds > t_timeout)
                {
                    if (sw_main.ElapsedMilliseconds - t_read_last > 1000)
                    {
                        dt_run = sw_main.ElapsedMilliseconds - t_stream_str;
                        dt_read = t_read_last == 0 ? 0 : sw_main.ElapsedMilliseconds - t_read_last;
                        robLogger.isImportTimedout = true;
                    }
                }

                // Get next byte
                if (sp_Xbee.BytesToRead > 0)
                {
                    // Check progress
                    string status_str = robLogger.GetImportStatus(read_ind);
                    if (status_str != " ")
                    {
                        // Print progress on seperate thread
                        new Thread(delegate ()
                        {
                            LogEvent(String.Format("[GetRobotLog] Log Import {0}", status_str));
                        }).Start();
                    }

                    // Get next char
                    c_arr[0] = c_arr[1];
                    c_arr[1] = c_arr[2];
                    c_arr[2] = (char)sp_Xbee.ReadByte();
                    in_arr[read_ind] = c_arr[2];
                    t_read_last = sw_main.ElapsedMilliseconds;

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
                }
            }

            // Reset logging flag
            robLogger.isLogging = false;

            // Check if logging timed out
            if (robLogger.isImportTimedout)
            {
                fc.errStr = LogEvent(String.Format("!!ERROR!! [GetRobotLog] FAILED: Robot Log Import: Read Timedout: dt_read={0}ms dt_run={1}", dt_read, dt_run));
                fc.isRunError = true;

                // Bail
                return;
            }
            else
            {
                // Print final status
                LogEvent(String.Format("[GetRobotLog] Log Import {0}", robLogger.prcnt_str[robLogger.prcnt_str.Length - 1]));

                // Finished log import
                LogEvent(String.Format("[GetRobotLog] FINISHED: Robot Log Import: bytes_read={0} rx={1} tx={2} dt_stream={3}",
                    read_ind + 1, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, robLogger.logDT));
            }

            // Store message length
            msg_lng = read_ind + 1;

            // Start log store
            LogEvent("[GetRobotLog] RUNNING: Robot Log Store...");

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
                    if (db.do_printRobLog)
                        Console.WriteLine(String.Format("   [LOG] r2c[{0}]: message=\"{1}\"",
                            rec_now, new_log));

                    // Check for missed log
                    if (rec_now != rec_last + 1)
                    {
                        int cnt_dropped = rec_last - rec_now - 1;
                        robLogger.AddDropped(cnt_dropped);
                        LogEvent(String.Format("**WARNING** [GetRobotLog] Dropped r2c Log: logged_expected={0} logs_stored={1} dropped={2}|{3}",
                            rec_now, robLogger.cnt_logged, r2c.cnt_dropped[0], r2c.cnt_dropped[1]));
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

            // Finished log store
            LogEvent(String.Format("[GetRobotLog] FINISHED: Robot Log Store: logs_stored={0} dt_run={1}",
                robLogger.cnt_logged, robLogger.logDT));

            // Save robot log file
            LogEvent("[GetRobotLog] RUNNING: Robot Log Save...");
            robLogger.SaveLog(logDir, robLogFi);
            LogEvent("[GetRobotLog] FINISHED: Robot Log Save");
        }

        #endregion

        #region ========== MATLAB COMMUNICATION =========

        // DOWORK FOR bw_RunGUI WORKER
        [STAThread]
        public static void DoWork_RunGUI(object sender, DoWorkEventArgs e)
        {
            // Local vars
            object startup_result = null;
            object icr_gui_result = null;
            string status = " ";

            // Set Matlab paths
            LogEvent("[DoWork_RunGUI] RUNNING: Setup Matlab paths...");
            SendMCOM(msg: @"addpath(genpath('" + matStartDir + "'));");
            LogEvent("[DoWork_RunGUI] FINISHED: Setup Matlab paths");

            // Run startup.m
            LogEvent("[DoWork_RunGUI] RUNNING: startup.m...");
            com_Matlab.Feval("startup", 0, out startup_result);
            LogEvent("[DoWork_RunGUI] FINISHED: startup.m...");

            // Wait for queue to clear
            while (queue_sendMCOM > 0)
                Thread.Sleep(10);

            // Run ICR_GUI.m
            LogEvent("[DoWork_RunGUI] RUNNING: ICR_GUI.m...");
            com_Matlab.Feval("ICR_GUI", 1, out icr_gui_result, db.systemTest, db.do_debugMat, false);

            // Get status
            object[] res = icr_gui_result as object[];
            status = res[0] as string;
            if (status == null)
                status = " ";

            // Print status
            if (status == "succeeded")
                LogEvent("[DoWork_RunGUI] FINISHED: ICR_GUI.m");
            else if (status != " ")
            {
                fc.errStr = LogEvent(String.Format("!!ERROR!! [DoWork_RunGUI] FAILED: ICR_GUI.m Error: {0}", status));
                fc.isRunError = true;
            }

            // Pass on results
            e.Result = status;
        }

        // RUNWORKERCOMPLETED FOR bw_RunGUI WORKER
        public static void RunWorkerCompleted_RunGUI(object sender, RunWorkerCompletedEventArgs e)
        {
            // Local vars
            string status = " ";

            // See if valid handeler returned
            try
            {
                status = e.Result as string;

                if (status == null)
                    status = " ";
            }
            catch
            {
                status = " ";
            }

            // Run succeeded
            if (status == "succeeded")
                LogEvent("[RunWorkerCompleted_RunGUI] FINISHED: RunGUI Worker");
            else if (status != " ")
            {
                // Run failed but errors were caught
                LogEvent("**WARNING** [RunWorkerCompleted_RunGUI] ABORTED: RunGUI Worker");
            }
            else
            {
                // Run failed completely
                fc.errStr = LogEvent("!!ERROR!! [RunWorkerCompleted_RunGUI] FAILED WITHOUT CATCHING ERRORS: RunGUI Worker");
                fc.isRunError = true;
            }

        }

        // DOWORK FOR bw_MatCOM WORKER
        [STAThread]
        public static void DoWork_MatCOM(object sender, DoWorkEventArgs e)
        {
            // Local vars
            long t_check = 0;
            long dt_check = 0;

            LogEvent("[DoWork_MatCOM] RUNNING: MatCOM Worker...");
            BackgroundWorker worker = (BackgroundWorker)sender;

            // Create tuple to pass args
            Tuple<char, double, double, double, ushort> bw_args = (Tuple<char, double, double, double, ushort>)e.Argument;

            //Put the arguments into nicely named variables:
            char id = bw_args.Item1;
            double dat1 = bw_args.Item2;
            double dat2 = bw_args.Item3;
            double dat3 = bw_args.Item4;
            ushort pack = bw_args.Item5;
            ushort pack_last = 0;

            // Check for matlab input till quit or abort
            while (fc.ContinueMatCom())
            {
                // Pause thread
                if (dt_check < 5)
                    Thread.Sleep((int)(5 - dt_check));

                // Wait for queue to clear
                if (queue_getMCOM > 1)
                    continue;

                // Store check time
                dt_check = sw_main.ElapsedMilliseconds - t_check;

                // Get current packet
                var _m2c_pack = GetMCOM(msg: "m2c_pack");
                id = (char)(byte)_m2c_pack.GetValue(0, 0);
                dat1 = _m2c_pack.GetValue(0, 1) != -1 ? (double)_m2c_pack.GetValue(0, 1) : Double.NaN;
                dat2 = _m2c_pack.GetValue(0, 2) != -1 ? (double)_m2c_pack.GetValue(0, 2) : Double.NaN;
                dat3 = _m2c_pack.GetValue(0, 3) != -1 ? (double)_m2c_pack.GetValue(0, 3) : Double.NaN;
                pack = (ushort)_m2c_pack.GetValue(0, 4);

                // Check for new packet number
                if (pack != 0 &&
                    pack != pack_last)
                {

                    // Trigger progress change event
                    worker.ReportProgress(0, new System.Tuple<char, double, double, double, ushort>(id, dat1, dat2, dat3, pack));

                    // Store packet number
                    pack_last = pack;

                    // Set pack back to zero
                    SendMCOM(msg: "m2c_pack(5) = 0;", do_print: false);

                }
            }

            // end polling
            e.Result = " ";
        }

        // PROGRESSCHANGED FOR bw_MatCOM WORKER
        public static void ProgressChanged_MatCOM(object sender, ProgressChangedEventArgs e)
        {
            // Pull out tuble vals
            Tuple<char, double, double, double, ushort> bw_args = (Tuple<char, double, double, double, ushort>)e.UserState;

            // Store id in top level vars
            char id = bw_args.Item1;
            double dat1 = bw_args.Item2;
            double dat2 = bw_args.Item3;
            double dat3 = bw_args.Item4;
            ushort pack = bw_args.Item5;

            // Update com info
            m2c.UpdateSentRcvd(id: id, pack: pack, t: sw_main.ElapsedMilliseconds);

            // Handle simulation data
            if (id == 'p' && db.systemTest == 3)
            {
                // Store matlab position data
                ulong ts = (ulong)(dat1) + vtStr;
                double x = dat2;
                double y = dat3;

                // Run compute pos
                bool pass = CompPos(0, ts, x, y);

                // Send data
                if (pass)
                    RepeatSendPack(send_max: 1, id: 'P', dat1: 0, dat2: (double)vtTS[0, 1], dat3: (double)vtCM[0], do_conf: false);

                // Bail to avoid printing
                return;
            }

            // Check for repeat packet
            string msg_str;
            if (pack != m2c.packLast[m2c.ID_Ind(id)])
            {
                // Print received data
                msg_str = String.Format("   [RCVD] m2c: id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4}",
                    id, dat1, dat2, dat3, pack);
                LogEvent(msg_str, m2c.t_new, m2c.t_last);
            }
            else
            {
                m2c.cnt_repeat++;
                // Print received data
                msg_str = String.Format("   [*RE-RCVD*] m2c: cnt={0} id=\'{1}\' dat=|{2:0.00}|{3:0.00}|{4:0.00}| pack={5}",
                    m2c.cnt_repeat, id, dat1, dat2, dat3, pack);
                LogEvent(msg_str, m2c.t_new, m2c.t_last);
                // Bail without processing
                return;
            }

            // Check for ses saved command
            if (id == 'F')
            {
                fc.isSesSaved = true;
                LogEvent("[ProgressChanged_MatCOM] ICR_GUI Confirmed Save");
            }

            // Check for rat in command
            else if (id == 'I' && dat1 == 1)
            {
                fc.isRatIn = true;
                LogEvent("[ProgressChanged_MatCOM] ICR_GUI Confirmed Rat Taken Into ICR");
            }

            // Check for rat out command
            else if (id == 'O')
            {
                fc.isRatOut = true;
                LogEvent("[ProgressChanged_MatCOM] ICR_GUI Confirmed Rat Taken Out of ICR");
            }

            // Check for quit
            else if (id == 'X')
            {
                // Check if this is a premature quit
                if (!fc.isSesSaved)
                {
                    // Start exiting early
                    fc.doAbort = true;
                    // Will print once
                    if (!fc.isGUIquit)
                        LogEvent("**WARNING** [DoWork_MatCOM] ICR_GUI FORCED QUIT");
                }
                // Set flag that GUI has quit
                fc.isGUIquit = true;
            }

            // Check for close confirmation
            else if (id == 'C')
            {
                // Check if this is a premature close
                if (!fc.isGUIquit)
                {
                    // Will print once
                    if (!fc.isGUIfinished)
                        fc.errStr = LogEvent("!!ERROR!! [DoWork_MatCOM] ICR_GUI FORCED CLOSE");
                    // Start exiting early
                    fc.isRunError = true;
                }
                // Set flag that GUI has closed
                fc.isGUIfinished = true;
            }

            // Check if mesage should be relayed to rob
            for (int i = 0; i < c2r.id.Length - 1; i++)
            {
                if (id == c2r.id[i])
                {
                    bool do_check_done = false;

                    // Check if handshake command
                    if (id == 'h') // move to
                    {
                        do_check_done = true;
                    }
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

                    // Send data
                    RepeatSendPack(id: id, dat1: dat1, dat2: dat2, dat3: dat3, do_check_done: do_check_done);
                }
            }
        }

        // RUNWORKERCOMPLETED FOR bw_MatCOM WORKER
        public static void RunWorkerCompleted_MatCOM(object sender, RunWorkerCompletedEventArgs e)
        {
            LogEvent("[RunWorkerCompleted_MatCOM] FINISHED: MatCOM Worker");
        }

        // SEND/STORE DATA FOR MATLAB
        /// <summary>
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="dat_num"></param>
        /// <param name="dat_char"></param>
        public static void SendMCOM(char id, byte dat_num = 0, char dat_char = ' ')
        {
            // Local vars
            ushort pack = 0;

            // Itterate packet
            pack = c2m.cnt_pack++;

            // Set in case we want to check that send finished
            c2m.SetCheckFor(id: id, do_check_sent_rcvd: true);

            // Will check for reply on id we are waiting on
            if (id == 'g')
                m2c.SetCheckFor(id: dat_char, do_check_sent_rcvd: true);

            // Run method on seperate thread
            new Thread(delegate ()
            {
                SendMCOM_Thread(id: id, dat_num: dat_num, dat_char: dat_char, pack: pack);
            }).Start();

        }
        public static void SendMCOM(string msg, bool do_print = true)
        {
            // Run method on seperate thread
            new Thread(delegate ()
            {
                SendMCOM_Thread(msg: msg, do_print: do_print);
            }).Start();
        }
        public static void SendMCOM_Thread(string msg = " ", char id = ' ', byte dat_num = 0, char dat_char = ' ', ushort pack = 0, bool do_print = true)
        {
            // Local vars
            long t_queued = sw_main.ElapsedMilliseconds;
            string dat_str = " ";

            // Format packet data strings
            if (id != ' ')
            {
                if (dat_char == ' ')
                {
                    msg = String.Format("[c2m.{0}.dat1, c2m.{0}.pack] =  deal({1}, {2});", id, dat_num, pack);
                    dat_str = String.Format("id=\'{0}\' dat={1} pack={2}", id, dat_num, pack);
                }
                else
                {
                    msg = String.Format("[c2m.{0}.dat1, c2m.{0}.pack] =  deal(\'{1}\', {2});", id, dat_char, pack);
                    dat_str = String.Format("id=\'{0}\' dat={1} pack={2}", id, dat_char, pack);
                }
            }

            // Check if queue backed up
            if (queue_sendMCOM >= 3)
                LogEvent(String.Format("**WARNING** [SendMCOM_Thread] c2m Queue Clogged: msg=\"{0}\" queued={1} queue_dt={2}",
                                msg, queue_sendMCOM, sw_main.ElapsedMilliseconds - t_queued));

            // Add to queue
            lock (lock_queue_sendMCOM) queue_sendMCOM++;
            lock (lock_sendMCOM)
            {

                // Check if queue hanging
                if (sw_main.ElapsedMilliseconds > t_queued + 100)
                    // Log/print error
                    LogEvent(String.Format("**WARNING** [SendMCOM_Thread] c2m Queue Hanging: msg=\"{0}\" queued={1} queue_dt={2}",
                                    msg, queue_sendMCOM, sw_main.ElapsedMilliseconds - t_queued));


                if (fc.ContinueMatCom())
                {
                    // Sending packet data
                    if (id != ' ')
                    {
                        // Execute matlab command
                        com_Matlab.Execute(msg);

                        // Update sent
                        c2m.UpdateSentRcvd(id: id, pack: pack, t: sw_main.ElapsedMilliseconds);

                        // Log/print sent
                        LogEvent(String.Format("   [SENT] c2m: {0} queued={1} queue_dt={2}",
                                dat_str, queue_sendMCOM, sw_main.ElapsedMilliseconds - t_queued));
                    }

                    // Sending simple command
                    else
                    {
                        // Execute matlab command
                        com_Matlab.Execute(msg);

                        // Log/print message
                        if (do_print)
                            LogEvent(String.Format("   [mCOM] c2m: msg=\"{0}\" queued={1} queue_dt={2}",
                                msg, queue_sendMCOM, sw_main.ElapsedMilliseconds - t_queued));
                    }

                }
            }
            lock (lock_queue_sendMCOM) queue_sendMCOM--;
        }

        // SEND/STORE DATA FOR MATLAB
        public static dynamic GetMCOM(string msg)
        {
            // Local vars
            dynamic mat_var = 0;
            long t_queued = sw_main.ElapsedMilliseconds;

            // Check if queue backed up
            if (queue_getMCOM >= 3)
                LogEvent(String.Format("**WARNING** [GetMCOM] m2c Queue Clogged: msg=\"{0}\" queued={1} queue_dt={2}",
                                msg, queue_getMCOM, sw_main.ElapsedMilliseconds - t_queued));

            // Add to queue
            lock (lock_queue_getMCOM) queue_getMCOM++;
            lock (lock_getMCOM)
            {

                // Check if queue hanging
                if (sw_main.ElapsedMilliseconds > t_queued + 100)
                    // Log/print error
                    LogEvent(String.Format("**WARNING** [GetMCOM] m2c Queue Hanging: msg=\"{0}\" queued={1} queue_dt={2}",
                                    msg, queue_getMCOM, sw_main.ElapsedMilliseconds - t_queued));

                if (fc.ContinueMatCom())
                    // Store value
                    mat_var = com_Matlab.GetVariable(msg, "global");
            }
            lock (lock_queue_getMCOM) queue_getMCOM--;

            // Return value
            return mat_var;
        }

        // WAIT FOR M2C CONFIRMATION
        public static bool WaitForMCOM(char id, bool do_send_check = false, bool do_send_request = true, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_start = sw_main.ElapsedMilliseconds;
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : t_start + timeout;
            bool is_sent = false;
            bool is_rcvd = false;
            string wait_str = " ";

            // Dont do send request if just checking that message sent
            do_send_request = do_send_check ? false : do_send_request;

            // Print info
            wait_str = do_send_check ? "|Sent|" : "|Rcvd|";
            LogEvent(String.Format("[WaitForMCOM] RUNNING: Wait for {0}: id=\'{1}\' do_send_check=%s do_send_request=%s do_abort=%s timeout=%d...",
                wait_str, id, do_send_check ? "true" : "false", do_send_request ? "true" : "false", do_abort ? "true" : "false", timeout == long.MaxValue ? 0 : timeout));

            // Send resend request
            if (do_send_request)
                SendMCOM(id: 'g', dat_char: id);

            do
            {
                // Check if sent 
                if (do_send_check)
                    is_sent = !is_sent ? c2m.IsSentRcvd(id) : is_sent;

                // Check if received
                else
                    is_rcvd = !is_rcvd ? m2c.IsSentRcvd(id) : is_rcvd;

                // Check if sent/rcvd confirmed
                if (is_sent || is_rcvd)
                {
                    LogEvent(String.Format("[WaitForMCOM] FINISHED: Wait for {0}: id=\'{1}\' dt_wait={2}",
                        wait_str, id, sw_main.ElapsedMilliseconds - t_start));
                    return true;
                }

                // Check if need to abort
                else if (
                    (do_abort && fc.doAbort) ||
                    !fc.ContinueMatCom() ||
                    (sw_main.ElapsedMilliseconds > t_timeout)
                    )
                {

                    // External forced abort
                    if (do_abort && fc.doAbort)
                        LogEvent(String.Format("**WARNING** [WaitForMCOM] Forced Abort: Wait for {0}: id=\'{1}\' dt_wait={2}",
                            wait_str, id, sw_main.ElapsedMilliseconds - t_start));
                    else
                    {
                        // Coms failed
                        if (!fc.ContinueMatCom())
                            fc.errStr = LogEvent(String.Format("!!ERROR!! [WaitForMCOM] Lost Comms: Wait for {0}: id=\'{1}\' dt_wait={2}",
                                wait_str, id, sw_main.ElapsedMilliseconds - t_start));

                        // Timedout
                        else if (sw_main.ElapsedMilliseconds > t_timeout)
                            fc.errStr = LogEvent(String.Format("!!ERROR!! [WaitForMCOM] Timedout: Wait for {0}: id=\'{1}\' dt_wait={2}",
                                wait_str, id, sw_main.ElapsedMilliseconds - t_start));

                        // Check if this is first packet
                        if (do_send_request && m2c.packTot == 0)
                            fc.isMAThanging = true;

                        // Set error flag
                        fc.isRunError = true;
                    }

                    // Set flags and bail
                    return false;
                }

                // Pause thread
                else
                    Thread.Sleep(1);

            } while (true);

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

        #region ========= CHEETAH COMMUNICATION =========

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

        // CALLBACK FOR NETCOM STREAMING
        public static void NetComCallbackVT(object sender, MNetCom.MVideoRec records, int numRecords, string objectName)
        {
            // Local vars
            ushort ent = records.swid;
            ulong ts = records.qwTimeStamp;
            double x = records.dnextracted_x;
            double y = records.dnextracted_y;

            if (!vtBlocker.CheckBlock(ent))
            {
                // Compute position
                bool pass = CompPos(ent, ts, x, y);

                // Send data
                if (pass)
                    RepeatSendPack(send_max: 1, id: 'P', dat1: (double)ent, dat2: (double)vtTS[ent, 1], dat3: (double)vtCM[ent], do_conf: false);

            }
            else if (db.do_printBlockedVt)
            {
                LogEvent(String.Format("**WARNING** [NetComCallbackVT] VT Blocked: ent={0} cnt={1} dt_send={2}|{3}|{4}",
                    ent, vtBlocker.cnt_block[ent], vtBlocker.GetSendDT(ent), vtBlocker.GetSendDT(ent, "avg"), sw_main.ElapsedMilliseconds - vtBlocker.t_sent[ent]));
            }
        }

        #endregion

        #region ========= MOVEMENT AND TRACKING =========

        // COMPUTE POS IN RAD FOR VT DATA
        public static bool CompPos(ushort ent, ulong ts, double x, double y)
        {

            // Get first vtTS once
            if (vtStr == 0)
            {
                // Save first ts and bail
                vtStr = ts;
                return false;
            }

            // Convert ts from us to ms and subtract firts record ts
            int ts_last = vtTS[ent, 1];
            int ts_now = (int)Math.Round((double)((ts - vtStr) / 1000));

            // Rescale y as VT data is compressed in y axis
            y = y * 1.0976;

            // Normalize 
            x = (x - X_CENT) / RADIUS;
            y = (y - Y_CENT) / RADIUS;

            // Flip y 
            y = y * -1;

            // Compute radians
            double rad_last = vtRad[ent, 1];
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
            vtTS[ent, 0] = ts_last;
            vtRad[ent, 0] = (float)rad_last;
            // New vals
            vtTS[ent, 1] = ts_now;
            vtRad[ent, 1] = (float)rad_now;
            vtCM[ent] = (float)cm;

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

        #region ============= MINOR METHODS =============

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

        #endregion

        #region =============== DEBUGGING ===============

        // LOG EVEN STRING
        public static string LogEvent(string msg_in, long t_now = -1, long t_last = -1)
        {
            // Local vars
            long t_m = 0;
            float t_s = 0;
            long t_m_sync = 0;
            float t_s_sync = 0;
            long dt = 0;
            string msg_print = " ";
            string msg_log = " ";
            string ts_str = " ";
            string dt_str = " ";

            // Get time from start of Main()
            t_now = t_now > 0 ? t_now : sw_main.ElapsedMilliseconds;

            lock (lock_printLog)
            {
                // Get sync correction
                t_m = t_now;
                t_m_sync = t_now - t_sync;

                // Convert to seconds
                t_s = t_m > 0 ? (float)(t_m) / 1000.0f : 0;
                t_s_sync = t_m_sync > 0 ? (float)(t_m_sync) / 1000.0f : 0;

                // Convert to string
                ts_str = String.Format("[{0:0.000}][{1:0.000}]", t_s_sync, t_s);

                // No input time
                if (t_last >= 0)
                {
                    dt = t_now - t_last;
                    dt_str = String.Format(" [dt={0}]", dt);
                }
                else
                    dt_str = "";

                // Pad ts string
                ts_str = ts_str.PadRight(20, ' ');

                // Cat strings
                msg_print = "\n" + ts_str + msg_in + dt_str + "\n";

                // Print message
                Console.Write(msg_print);

                // Remove cammas from message
                msg_log = msg_in.Replace(",", string.Empty) + dt_str;

                // Store in logger 
                csLogger.UpdateList(msg_log, t_m_sync);
            }

            // Return printed message
            return msg_print;
        }

        #endregion

    }

    #region =============== OTHER CLASSES ===============

    // CLASS TO TRACK PROGRAM FLAGS
    class Flow_Control
    {
        // Private vars
        private static object _lock_printLog = new object();
        private static object _lock_runPauseForDB = new object();
        private static bool _isRunError = false;
        private static bool _doAbort = false;
        private static bool _isMAThanging = false;
        private static int _cnt_err = 0;
        private string[] _err_list = new string[100];
        // Public vars
        public bool isNlxConnected = false;
        public bool isMatComActive = false;
        public bool isRobComActive = false;
        public bool isArdComActive = false;
        public bool isMovedToStart = false;
        public bool isRatIn = false;
        public bool isRatOut = false;
        public bool isSaveEnabled = false;
        public bool isSesSaved = false;
        public bool isGUIquit = false;
        public bool isGUIfinished = false;
        public bool doExit = false;
        public string errStr
        {
            set
            {
                // Store current error string
                _err_list[_cnt_err++] = value;
            }
        }
        public bool isRunError
        {
            set
            {
                _isRunError = value;
                if (value)
                {
                    _doAbort = true;
                }
            }
            get
            { return _isRunError; }
        }
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
                    _doAbort = true;
                    _isRunError = true;
                }

            }
            get { return _isMAThanging; }
        }

        // Constructor
        public Flow_Control(
            object _lock_print_log
            )
        {
            _lock_printLog = _lock_print_log;
        }

        // Run erro hold
        public void RunPauseForDB()
        {
            // Make sure runs consecutively
            lock (_lock_runPauseForDB)
            {

                // Print messeage with error
                lock (_lock_printLog)
                {
                    Thread.Sleep(500);
                    Console.WriteLine("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    if (isRunError)
                    {
                        Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PAUSED FOR ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                        // Print all errors
                        for (int i = 0; i < _cnt_err; i++)
                            Console.WriteLine(_err_list[i]);
                    }
                    else
                        Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PAUSED FOR DEBUGGING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PRESS ANY KEY TO EXIT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
                }

                // Lock printing
                lock (_lock_printLog)
                    Console.ReadKey();
            }
        }

        // Check if Matlab coms are active
        public bool ContinueMatCom()
        {
            return (isMatComActive || !_doAbort) && !isMAThanging && !doExit;
        }

        // Check if serial Xbee coms are active
        public bool ContinueRobCom()
        {
            return (isRobComActive || !_doAbort) && !doExit;
        }

        // Check if serial CheetahDue coms active
        public bool ContinueArdCom()
        {
            return (isArdComActive || !_doAbort) && !doExit;
        }

    }

    // CLASS TO TRACK COMS
    class Com_Track
    {
        // Private vars
        private static Stopwatch _sw = new Stopwatch();
        private object _lock_checkSentRcvd = new object();
        private object _lock_checkConf = new object();
        private object _lock_checkDone = new object();
        private bool[] _doCheckSentRcvd;
        private bool[] _doCheckConf;
        private bool[] _doCheckDone;
        // Public vars
        public char[] id;
        public byte[] head = new byte[1] { 0 };
        public byte[] foot = new byte[1] { 0 };
        public ushort[] pack;
        public ushort[] packLast;
        public ushort packTot = 0;
        public ushort cnt_pack = 0;
        public int[] cnt_dropped = new int[2] { 0, 0 };
        public int cnt_repeat = 0;
        public long t_new = 0;
        public long t_last = 0;
        public long[] t_sentRcvd;

        // Constructor
        public Com_Track(
            Stopwatch _stop_watch,
            object _lock_check_conf,
            object _lock_check_done,
            char[] _id,
            byte _head = 0,
            byte _foot = 0
            )
        {
            _sw = _stop_watch;
            _lock_checkConf = _lock_check_conf;
            _lock_checkDone = _lock_check_done;
            id = _id;
            head[0] = _head;
            foot[0] = _foot;
            pack = new ushort[_id.Length];
            packLast = new ushort[_id.Length];
            t_sentRcvd = new long[_id.Length];
            _doCheckSentRcvd = new bool[_id.Length];
            _doCheckConf = new bool[_id.Length];
            _doCheckDone = new bool[_id.Length];

            // Initialize values to zero
            for (int i = 0; i < _id.Length; i++)
            {
                pack[i] = 0;
                packLast[i] = 0;
                t_sentRcvd[i] = 0;
                _doCheckSentRcvd[i] = false;
                _doCheckConf[i] = false;
                _doCheckDone[i] = false;
            }
        }

        // Check if command was sent recently
        public void SetCheckFor(char id, bool do_check_sent_rcvd = false, bool do_check_conf = false, bool do_check_done = false)
        {
            // Set sent/received check flag
            lock (_lock_checkSentRcvd)
                _doCheckSentRcvd[ID_Ind(id)] = do_check_sent_rcvd;

            // Set received check flag
            lock (_lock_checkConf)
                _doCheckConf[ID_Ind(id)] = do_check_conf;

            // Set done check flag
            lock (_lock_checkDone)
                _doCheckDone[ID_Ind(id)] = do_check_done;
        }

        // Update packet info
        public void ResetCheckFor(char id, bool is_conf = false, bool is_done = false)
        {
            // Reset conf check
            if (is_conf)
                lock (_lock_checkConf)
                    _doCheckConf[ID_Ind(id)] = false;

            // Reset done check
            if (is_done)
                lock (_lock_checkDone)
                    _doCheckDone[ID_Ind(id)] = false;
        }

        // Update packet info
        public void UpdateSentRcvd(char id, ushort pack = 0, long t = 0)
        {
            lock (_lock_checkSentRcvd)
            {
                // Update packet number
                packTot = pack;
                packLast[ID_Ind(id)] = this.pack[ID_Ind(id)];
                this.pack[ID_Ind(id)] = pack;

                // Update check flag
                _doCheckSentRcvd[ID_Ind(id)] = false;

                // Update timers
                t_last = t_new;
                t_new = t;
                t_sentRcvd[ID_Ind(id)] = t;

                // Reset consecutive dropped packs
                cnt_dropped[0] = 0;
            }
        }

        // Check if command sent or recieved
        public bool IsSentRcvd(char id)
        {
            lock (_lock_checkSentRcvd)
                return !_doCheckSentRcvd[ID_Ind(id)];
        }

        // Check if should continue to wait for confirmation command was recieved
        public bool IsConfirmed(char id)
        {
            lock (_lock_checkConf)
                return !_doCheckConf[ID_Ind(id)];
        }

        // Check if should continue to wait for  command done
        public bool IsDone(char id)
        {
            lock (_lock_checkDone)
                return !_doCheckDone[ID_Ind(id)];
        }

        // Track dropped packets
        public void AddDropped(int cnt)
        {
            cnt_dropped[0] += cnt;
            cnt_dropped[1] += cnt;
        }

        // Find id index
        public int ID_Ind(char id)
        {
            int ind = -1;
            for (int i = 0; i < this.id.Length; i++)
            {
                if (id == this.id[i]) ind = i;
            }
            return ind;
        }

        // Find packet index
        public char PackID(ushort pack)
        {
            int ind = -1;
            for (int i = 0; i < this.pack.Length; i++)
            {
                if (pack == this.pack[i]) ind = i;
            }
            return id[ind];
        }

    }

    // CLASS TO LOG DB INFO
    class DB_Logger
    {
        // Private vars
        private Stopwatch _sw = new Stopwatch();
        private string[] _logList = new string[50000];
        private readonly object _lock_logFlags = new object();
        private readonly object _lock_updateList = new object();
        private readonly object _lock_bytesToRcv = new object();
        private long _t_logStart = 0;
        private string _lastLogStr = " ";
        private bool _isStarted = false;
        private bool _isLogging = false;
        private bool _isSaved = false;
        public UnionHack _u_bytesToRcv;
        private int _bytesToRcv = 0;
        private int next_milestone = 0;
        private const int _n_updates = 10;
        private int[] _import_update_bytes = new int[_n_updates];
        // Public vars
        public bool isImportTimedout = false;
        public string[] prcnt_str = new string[_n_updates + 1];
        public int cnt_logged = 0;
        public int[] cnt_dropped = new int[2] { 0, 0 };
        public bool isLogging
        {
            set
            {
                // Store total log time
                if (value)
                {
                    lock (_lock_logFlags)
                    {
                        _t_logStart = _sw.ElapsedMilliseconds;
                        _isStarted = true;
                    }
                }
                _isLogging = value;
            }
            get { return _isLogging; }
        }
        public bool isLogComplete
        {
            get
            {
                if (cnt_logged > 0 && cnt_dropped[1] == 0)
                    return true;
                else
                    return false;
            }
        }
        public bool isFinished
        {
            get
            {
                lock (_lock_logFlags)
                    return _isSaved || !_isStarted;
            }
        }
        public long logDT
        {
            get { return _sw.ElapsedMilliseconds - _t_logStart; }
        }
        public int bytesToRcv
        {
            set { lock (_lock_bytesToRcv) _bytesToRcv = value; }
            get { lock (_lock_bytesToRcv) return _bytesToRcv; }
        }

        // Constructor
        public DB_Logger(
            Stopwatch _stop_watch
            )
        {
            _sw = _stop_watch;
        }

        // Add new log entry
        public void UpdateList(string log_str, long ts = -1)
        {
            lock (_lock_updateList)
            {
                // Check for repeat
                if (log_str != _lastLogStr)
                {
                    // Save log string
                    _lastLogStr = log_str;

                    // Itterate count
                    cnt_logged++;

                    // Reset consecutive dropped logs
                    cnt_dropped[0] = 0;

                    // Add count and time
                    string str;
                    if (ts < 0)
                        // Add count but skip time
                        str = String.Format("[{0}],{1}", cnt_logged, log_str);
                    else
                        // Add count and time
                        str = String.Format("[{0}],{1},{2}", cnt_logged, ts, log_str);

                    // Add to list
                    if (cnt_logged - 1 < 50000)
                        _logList[cnt_logged - 1] = str;
                    else if (cnt_logged - 1 == 50000)
                        _logList[cnt_logged - 1] = String.Format("**WARNING** Log Maxed out at {0} entries", 50000);

                }
            }
        }

        // Track dropped packets
        public void AddDropped(int cnt)
        {
            cnt_dropped[0] += cnt;
            cnt_dropped[1] += cnt;
        }

        // Store bytes to be reveived sent mesage bytes
        public void UpdateBytesToRcv(byte[] new_byte)
        {
            // Local vars
            string str;

            // Add bytes to UnionHack var
            _u_bytesToRcv.i = 0;
            _u_bytesToRcv.b1 = new_byte[0];
            _u_bytesToRcv.b2 = new_byte[1];
            _u_bytesToRcv.b3 = new_byte[2];

            // Get total bytes
            _bytesToRcv = _u_bytesToRcv.i;

            // Compute print update milestones
            for (int i = 0; i <= _n_updates; i++)
            {
                // Get milestone value
                if (i < _n_updates)
                    _import_update_bytes[i] = (int)((double)_bytesToRcv * i / 10);

                // Get string
                str = String.Format("{0}% Complete", i * _n_updates);
                str = str.PadRight(str.Length + _n_updates - i, '.');
                str = str.PadLeft(str.Length + i, '.');
                prcnt_str[i] = str;
            }
        }

        // Pass string giving import status
        public string GetImportStatus(int cnt_bytes)
        {
            // Local vars
            string msg = " ";

            // Check if milestone reached
            if (next_milestone < _n_updates)
                if (cnt_bytes == _import_update_bytes[next_milestone])
                {
                    msg = prcnt_str[next_milestone];
                    next_milestone++;
                }

            // Return string
            return msg;
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
                    if (count++ == cnt_logged)
                        break;
                    file_out.WriteLine(line);
                }
            }
            // Set flag
            _isSaved = true;
        }

    }

    // CLASS TO BLOCK SENDIGN VT DATA
    class VT_Blocker
    {
        // Private vars
        private static Stopwatch _sw = new Stopwatch();
        private static readonly object _lockBlock = new object();
        private static int _cntThread = 0;
        private static long _t_blockTim = 0;
        private static long _blockFor = 60; // (ms) // TEMP
                                            // Public vars
        public long[] t_sent = new long[2] { 0, 0 };
        public long[] t_sent_last = new long[2] { 0, 0 };
        public int[,] dt_hist = new int[2, 10];
        public int[] cnt_sent = new int[2] { 0, 0 };
        public int[] cnt_block = new int[2] { 0, 0 };

        // Constructor
        public VT_Blocker(
            Stopwatch _stop_watch
            )
        {
            _sw = _stop_watch;
        }

        // Block sending vt data
        public void Block(char id)
        {
            if (id != 'P')
            {
                lock (_lockBlock)
                {
                    _cntThread++;
                    _t_blockTim = _sw.ElapsedMilliseconds + _blockFor;
                }
            }
        }

        // Unblock sending vt data
        public void Unblock(char id)
        {
            if (id != 'P')
            {
                lock (_lockBlock)
                {
                    _cntThread--;
                }
            }
        }

        // Check if currently blocking
        public bool CheckBlock(ushort ent)
        {
            if (_sw.ElapsedMilliseconds > _t_blockTim && _cntThread <= 0)
                return false;
            else
            {
                // Add to counter
                cnt_block[ent]++;
                return true;
            }
        }

        // Update send time
        public int GetSendDT(int ent, string what = "now")
        {
            if (what == "avg")
            {
                // Get number of samples stored
                int dt_sum = 0;
                int dt_mu = 0;
                int cnt = cnt_sent[ent] < 10 ? cnt_sent[ent] : 10;

                // Bail if no samples stored
                if (cnt == 0)
                    return 0;

                // Compute average dt_send
                for (int i = 0; i < cnt; i++)
                    dt_sum += dt_hist[ent, i];
                dt_mu = dt_sum / cnt;
                return dt_mu;
            }
            else
                return (int)(t_sent[ent] - t_sent_last[ent]);
        }
        public void StoreSendTime(int ent, long t)
        {
            // Update time
            t_sent_last[ent] = t_sent[ent];
            t_sent[ent] = t;

            // Update count and sum
            cnt_sent[ent]++;

            // Shift and update dt hist
            for (int i = 0; i < 10 - 1; i++)
                dt_hist[ent, i] = dt_hist[ent, i + 1];

            // Store current dt
            dt_hist[ent, 9] = (int)(t_sent[ent] - t_sent_last[ent]);
        }
    }

    // USE TO CONVERT BETWEEN DATA TYPE FOR SERIAL COMS
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

}