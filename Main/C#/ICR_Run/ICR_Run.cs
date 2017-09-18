using System;
using MNetCom;
using System.ComponentModel;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;

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
            do_debug_mat: false,
            break_line: 0, // 0
            do_print_blocked_vt: false,
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
        static readonly object lock_sendPack = new object();
        static readonly object lock_queue_sendMCOM = new object();
        static readonly object lock_queue_sendPack = new object();
        private static int queue_sendMCOM = 0;
        private static int queue_SendXBee = 0;
        static readonly object lock_checkConf = new object();
        static readonly object lock_checkDone = new object();
        static readonly object lock_printLog = new object();

        // Initialize vt blocking object
        private static VT_Handler vtHandler = new VT_Handler(_stop_watch: sw_main);

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
        private static UnionHack logBytes = new UnionHack(0, '0', 0, 0, 0);

        // Directories
        private static string matStartDir = @"C:\Users\lester\MeDocuments\AppData\MATLAB\Startup";
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
            _flag_send_rcv: true,
            _id:
            new char[16]{ // prefix giving masage id
            'i', // handshake request [NA]
            'p', // simulation data [ts, x, y]
            'G', // matlab gui loaded [NA]
            'A', // connected to AC computer [NA]
            'N', // netcom setup [NA]
            'F', // data saved [NA]
            'X', // confirm quit
            'C', // confirm close
            'T', // system test command [test]
            'S', // setup session [ses_cond, sound_cond]
            'M', // move to position [targ_pos]
            'R', // run reward [rew_pos, zone_ind, rew_delay]
            'H', // halt movement [halt_state]
            'B', // bulldoze rat [bull_delay, bull_speed]
            'I', // rat in/out [in/out]
            'O'  // confirm rat out [NA]
             }
            );

        // CS to Matlab
        private static Com_Track c2m = new Com_Track(
            _stop_watch: sw_main,
            _lock_check_conf: new object(),
            _lock_check_done: new object(),
            _flag_send_rcv: false,
            _id:
            new char[9] {
            'g', // request m2c data
            'h', // setup handshake
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
            _flag_send_rcv: false,
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
            'V', // request stream status
            'L', // request log conf/send
            'J', // battery voltage
            'Z', // reward zone
            'U', // log size
            'D', // execution done
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
            _flag_send_rcv: true,
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
            'D', // execution done
            'P', // position data
             },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // Robot to Ard
        private static Com_Track r2a = new Com_Track(
            _stop_watch: sw_main,
            _lock_check_conf: new object(),
            _lock_check_done: new object(),
            _flag_send_rcv: false,
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
            _flag_send_rcv: false,
            _id:
            new char[1] {
            ' '
            },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // Communication general
        private static long dt_sendSent = 5; // (ms)
        private static long dt_sendRcvd = 1; // (ms)
        private static long dt_resend = 500; // (ms)
        private static int resendMax = 5;
        private static long timeoutLoadGUI = 15000; // (ms)
        private static long timeoutConnectAC = 15000; // (ms)
        private static long timeoutConnectMatNLX = 60000; // (ms) 
        private static long timeoutMatCloseConfirm = 10000; // (ms)
        private static long timeoutImportLog = 10000; // (ms)

        // Position variables
        private static double X_CENT = 359.5553;
        private static double Y_CENT = 260.2418;
        private static double RADIUS = 179.4922;
        private static double feedDist = 66 * ((2 * Math.PI) / (140 * Math.PI));
        private static UInt64 vtStr = 0;
        private static double[,] vtRad = new double[2, 2];
        private static double[] vtCM = new double[2];
        private static UInt32[,] vtTS = new UInt32[2, 2];

        #endregion

        #region ================= MAIN ==================

        // MAIN
        static void Main()
        {

            // SETUP
            LogEvent_Thread("[MAIN] RUNNING: SETUP...");
            bool passed_setup = Setup();
            if (passed_setup)
                LogEvent_Thread("[MAIN] SUCCEEDED: SETUP");
            else if (!fc.isRunError)
                LogEvent_Thread("**WARNING!! [MAIN] ABORTED: SETUP", is_warning: true);
            else
                LogEvent_Thread("!!ERROR!! [MAIN] FAILED: SETUP", is_error: true);

            // RUN 
            if (passed_setup)
            {
                LogEvent_Thread("[MAIN] RUNNING: RUN...");
                bool passed_run = Run();
                if (passed_run)
                    LogEvent_Thread("[MAIN] SUCCEEDED: RUN");
                else if (!fc.isRunError)
                    LogEvent_Thread("**WARNING!! [MAIN] ABORTED: RUN", is_warning: true);
                else
                    LogEvent_Thread("!!ERROR!! [MAIN] FAILED: RUN", is_error: true);
            }

            // EXIT 
            LogEvent_Thread("[MAIN] RUNNING: EXIT...");
            Exit();
            LogEvent_Thread("[MAIN] FINISHED: EXIT");

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
            UInt16 pack = 0;

            // Start timer
            sw_main.Start();

            // Set buffer height
            Console.BufferHeight = Int16.MaxValue - 1;

            // Create temp directory
            System.IO.Directory.CreateDirectory(logDir);
            LogEvent_Thread(String.Format("[Setup] FINISHED: Create Temporary Log Directory: \"{0}\"", logDir));

            // Start Cheetah if it is not already running
            LogEvent_Thread("[Setup] RUNNING: Run Cheetah.exe...");
            while (!IsProcessOpen("Cheetah") && !fc.doAbort)
            {
                OpenCheetah("Cheetah.cfg");
            }
            if (IsProcessOpen("Cheetah"))
                LogEvent_Thread("[Setup] SUCCEEDED: Run Cheetah.exe");
            else
            {
                if (!fc.doAbort)
                {
                    LogEvent_Thread("!!ERROR!! [Setup] FAILED: Run Cheetah.exe", is_error: true);
                    fc.isRunError = true;
                }
                else
                    LogEvent_Thread("**WARNING** [Setup] ABORTED: Run Cheetah.exe", is_warning: true);
                return false;
            }

            // Initalize Matlab global vars
            LogEvent_Thread("[Setup] RUNNNING: Create mCOM Global Variables...");
            lock (lock_queue_sendMCOM) queue_sendMCOM++;
            lock (lock_sendMCOM)
            {
                System.Array _m2c_pack = new double[6] { 0, 0, 0, 0, 0, 0 };
                com_Matlab.PutWorkspaceData("m2c_pack", "global", _m2c_pack);
                com_Matlab.PutWorkspaceData("m2c_dir", "global", " ");
            }
            lock (lock_queue_sendMCOM) queue_sendMCOM--;
            LogEvent_Thread("[Setup] FINISHED: Create mCOM Global Variables");

            // Setup and start CheetahDue serial
            LogEvent_Thread("[Setup] RUNNING: Setup CheetahDue Serial Coms and Logging...");
            sp_cheetahDue.ReadTimeout = 100;
            sp_cheetahDue.BaudRate = 57600;
            sp_cheetahDue.PortName = "COM5";
            // Open serial port connection
            sp_cheetahDue.Open();
            // Start getting new data on seperate thread
            new Thread(delegate ()
            {
                ParserA2C();
            }).Start();
            LogEvent_Thread("[Setup] FINISHED: Setup CheetahDue Serial Coms and Logging");

            // Setup and start Xbee serial
            LogEvent_Thread("[Setup] RUNNING: Setup Xbee Serial Coms");
            sp_Xbee.ReadTimeout = 100;
            sp_Xbee.BaudRate = 57600;
            sp_Xbee.PortName = "COM4";
            // Create event handeler for incoming data
            sp_Xbee.DataReceived += DataReceived_Xbee;
            // Set byte threshold to max packet size
            sp_Xbee.ReceivedBytesThreshold = 1;
            // Open serial port connection
            sp_Xbee.Open();
            // Spin up parser thread
            new Thread(delegate ()
            {
                ParserR2C();
            }).Start();
            LogEvent_Thread("[Setup] FINISHED: Setup Xbee Serial Coms");

            // Setup debugging
            if (db.systemTest != 0 || db.do_debugMat)
            {
                LogEvent_Thread("[Setup] RUNNING: Setup Debugging...");

                // Hide/show matlab app window
                com_Matlab.Visible = 1;

                // Set MATLAB break point
                if (db.breakLine != 0)
                    SendMCOM_Thread(msg: String.Format("dbstop in ICR_GUI at {0};", db.breakLine));

                LogEvent_Thread("[Setup] FINISHED: Setup Debugging");

                // Log/print db settings
                LogEvent_Thread(String.Format("[Setup] RUNNING IN DEBUG MODE: systemTest={0} do_debugMat={1}",
                    db.systemTest, db.do_debugMat));
            }
            else
                com_Matlab.Visible = 0;


            // Setup ICR_GUI background worker
            LogEvent_Thread("[Setup] RUNNING: Start RunGUI Worker...");
            bw_RunGUI.DoWork += DoWork_RunGUI;
            bw_RunGUI.RunWorkerCompleted += RunWorkerCompleted_RunGUI;
            // Start ICR_GUI worker
            bw_RunGUI.RunWorkerAsync();
            LogEvent_Thread("[Setup] FINISHED: Start RunGUI Worker");

            // Setup MATLAB com background worker
            LogEvent_Thread("[Setup] START: Start MatCOM Worker...");
            bw_MatCOM.DoWork += DoWork_MatCOM;
            bw_MatCOM.ProgressChanged += ProgressChanged_MatCOM;
            bw_MatCOM.RunWorkerCompleted += RunWorkerCompleted_MatCOM;
            bw_MatCOM.WorkerReportsProgress = true;
            var bw_args = Tuple.Create(id, dat[0], dat[1], dat[2], pack);
            bw_MatCOM.RunWorkerAsync(bw_args);
            LogEvent_Thread("[Setup] FINISHED: Start MatCOM Worker...");

            //// TEMP
            //double[] d = new double[3] { 2.3, 3.2, 4.3 };
            //RepeatSendPack(send_max: 1, id: 'V', dat: d, pack: 1, do_conf: false, do_check_done: false);
            //KillMatlab();

            // Wait for ICR_GUI to connect to AC computer
            LogEvent_Thread("[Setup] RUNNING: Wait for AC Connect...");
            pass = WaitForMCOM(id: 'A', do_abort: true, timeout: timeoutConnectAC);
            if (pass)
                LogEvent_Thread("[Setup] SUCCEEDED: Wait for AC Connect");
            else
            {
                // Program timed out because matlab was hanging on connect
                LogEvent_Thread("**WARNING** [Setup] ABORTED: Wait for AC Connect", is_warning: true);
                fc.isMAThanging = true;
                return false;
            }

            // Wait for matlab handshake request
            LogEvent_Thread("[Setup] RUNNING: Wait for ICR_GUI Handshake...");
            pass = WaitForMCOM(id: 'i', do_abort: true, timeout: 15000);
            if (pass)
            {
                LogEvent_Thread("[Setup] SUCCEEDED: Wait for ICR_GUI Handshake...");
                fc.isMatComActive = true;
            }
            else
            {
                LogEvent_Thread("**WARNING** [Setup] ABORTED: Wait for ICR_GUI Handshake...", is_warning: true);
                return false;
            }

            // Send CheetahDue handshake request
            LogEvent_Thread("[Setup] RUNNING: Wait for Robot Handshake...");
            byte[] out_byte = new byte[1] { (byte)'i' };
            sp_cheetahDue.Write(out_byte, 0, 1);

            // Wait for handshake from robot
            pass = WaitForSerial(id: 'h', check_for: "rcv", do_abort: true, timeout: 5000);
            if (pass)
            {
                // Store sync time based on send time
                t_sync = r2c.t_sentRcvd[c2r.ID_Ind('h')];

                // Log/print sync time
                LogEvent_Thread(String.Format("SET SYNC TIME: {0}ms", t_sync), t: t_sync);

                // Log/print success
                LogEvent_Thread("[Setup] SUCCEEDED: Robot Handshake");
            }
            else
            {
                LogEvent_Thread("!!ERROR!! [Setup] ABORTED: Robot Handshake", is_error: true);
                return false;
            }

            // Initilize deligate for VT callback
            deligate_netComCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            com_netComClient.SetCallbackFunctionVT(deligate_netComCallback, new ICR_Run());

            // Wait for ICR_GUI to load 
            LogEvent_Thread("[Setup] RUNNING: Wait for ICR_GUI to Load...");
            pass = WaitForMCOM(id: 'G', timeout: timeoutLoadGUI);
            if (pass)
            {
                LogEvent_Thread("[Setup] SUCCEEDED: Wait for ICR_GUI to Load");
            }
            else
            {
                LogEvent_Thread("**WARNING** [Setup] ABORTED: Wait for ICR_GUI to Load", is_warning: true);
                return false;
            }

            // Wait for ICR_GUI to connect to NLX
            LogEvent_Thread("[Setup] RUNNING: Wait for ICR_GUI NLX Setup...");
            pass = WaitForMCOM(id: 'N', do_abort: true, timeout: timeoutConnectMatNLX);
            if (pass)
                LogEvent_Thread("[Setup] SUCCEEDED: Wait for ICR_GUI NLX Setup");
            else
            {
                LogEvent_Thread("**WARNING** [Setup] ABORTED: Wait for ICR_GUI NLX Setup", is_warning: true);
                return false;
            }

            // Setup and begin NetCom streaming
            LogEvent_Thread("[Setup] RUNNING: Connect to NLX...");
            if (!(com_netComClient.AreWeConnected()))
                fc.isNlxConnected = com_netComClient.ConnectToServer(NETCOM_IP);
            if (fc.isNlxConnected)
                LogEvent_Thread("[Setup] SUCCEEDED: Connect to NLX");
            else
            {
                LogEvent_Thread("!!ERROR!! [Setup] FAILED: Connect to NLX", is_error: true);
                fc.isRunError = true;
                return false;
            }

            // Begin stream and set app name
            LogEvent_Thread("[Run] STARTING: Nlx Stream");
            com_netComClient.SetApplicationName(NETCOM_APP_ID);

            // Stream rat vt
            if (db.systemTest != 3)
            {
                LogEvent_Thread("[Run] STARTING: VT1 Stream");
                com_netComClient.OpenStream(NETCOM_ACQ_ENT_1);
            }

            // Stream rob vt
            LogEvent_Thread("[Run] STARTING: VT2 Stream");
            com_netComClient.OpenStream(NETCOM_ACQ_ENT_2);

            // Send streaming check request on seperate thread
            LogEvent_Thread("[Run] RUNNING: Confirm Robot Streaming...");
            RepeatSendPack_Thread(id: 'V', do_check_done: true);
            // Wait for confirmation from robot
            pass = WaitForSerial(id: 'V', do_abort: true, timeout: 5000);
            if (pass)
            {
                // Send confirm stream to Matlbab
                SendMCOM_Thread(id: 'V', dat_num: 1);
                LogEvent_Thread("[Run] SUCCEEDED: Confirm Robot Streaming");
            }
            else
            {
                LogEvent_Thread("**WARNING** [Run] ABORTED: Confirm Robot Streaming", is_warning: true);
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
            LogEvent_Thread("[Run] RUNNING: Confirm Setup...");
            pass = WaitForMCOM(id: 'S', do_abort: true);
            if (pass)
                pass = WaitForSerial(id: 'S', do_abort: true);
            if (pass)
                LogEvent_Thread("[Run] SUCCEEDED: Confirm Setup");
            else
            {
                LogEvent_Thread("**WARNING** [Run] ABORTED: Confirm Setup", is_warning: true);
                return false;
            }

            // Wait for initial move to command to complete
            LogEvent_Thread("[Run] RUNNING: MoveTo Start...");
            // Wait for matlab
            pass = WaitForMCOM(id: 'M', do_abort: true);
            // Wait for move done
            if (pass)
                pass = WaitForSerial(id: 'M', do_abort: true);
            if (pass)
            {
                // Send confirm robot in place to Matlbab
                SendMCOM_Thread(id: 'K', dat_num: 1);
                fc.isMovedToStart = true;
                LogEvent_Thread("[Run] SUCCEEDED: MoveTo Start");
            }
            else
            {
                LogEvent_Thread("**WARNING** [Run] ABORTED: MoveTo Start", is_warning: true);
                return false;
            }

            // Main holding loop
            LogEvent_Thread("[Run] RUNNING: Main Session Loop...");
            // Stay in loop till rat is out or error
            while (
                com_netComClient.AreWeConnected() &&
                !fc.isRecDone &&
                !fc.doAbort
                ) ;
            if (!fc.doAbort)
                LogEvent_Thread("[Run] SUCCEEDED: Main Session Loop");
            else
            {
                if (com_netComClient.AreWeConnected())
                    LogEvent_Thread("**WARNING** [Run] ABORTED: Main Session Loop", is_warning: true);
                else
                {
                    LogEvent_Thread("!!ERROR!! [Run] FAILED: Main Session Loop Because NLX Disconnected", is_error: true);
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

            // Check if we have confirmed rat is out and recording is done
            if (fc.isRatIn && !fc.isRecDone)
            {
                LogEvent_Thread("[Run] RUNNING: Wait for Last Confirmation Rat is Out...");
                pass = WaitForMCOM(id: 'O', timeout: 10000);
                if (pass)
                {
                    // Wait for all the other crap to be relayed from Matlab
                    Thread.Sleep(1000);
                    LogEvent_Thread("[Exit] SUCCEEDED: Wait for Last Confirmation Rat is Out");
                }
                else
                    LogEvent_Thread("**WARNING** [Exit] ABORTED: Wait for Last Confirmation Rat is Out", is_warning: true);
            }

            // Wait for reply on any remaining sent packets
            LogEvent_Thread("[Run] RUNNING: Wait for Last Packets...");
            pass = WaitForSerial(id_arr: c2r.id, timeout: 5000);
            if (pass)
                LogEvent_Thread("[Run] SUCCEEDED: Wait for Last Packets");
            else
            {
                LogEvent_Thread("**WARNING** [Run] ABORTED: Wait for Last Packets", is_warning: true);
            }

            // MoveTo defualt pos
            if (fc.isMovedToStart)
            {
                LogEvent_Thread("[Exit] RUNNING: MoveTo South...");
                double move_to = CalcMove(4.7124 - feedDist);

                // Send move command on seperate thread and wait for done reply
                RepeatSendPack_Thread(id: 'M', dat1: move_to, do_check_done: true);

                // Wait for confirmation from robot
                pass = WaitForSerial(id: 'M', timeout: 10000);
                if (pass)
                    LogEvent_Thread("[Exit] SUCCEEDED: MoveTo South");
                else
                {
                    LogEvent_Thread("**WARNING** [Exit] ABORTED: MoveTo South", is_warning: true);
                }
            }

            // Wait 1 second to see move to plot
            Thread.Sleep(1000);

            // Shut down NetCom
            if (IsProcessOpen("Cheetah"))
            {
                LogEvent_Thread("[Exit] RUNNING: NetCom Disconnect...");
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
                    LogEvent_Thread("[Exit] SUCCEEDED: NetCom Disconnect");
                }
                else
                {
                    LogEvent_Thread("!!ERROR!! [Exit] FAILED: NetCom Disconnect", is_error: true);
                    fc.isRunError = true;
                }
            }

            // Enable ICR_GUI save button
            if (fc.isRecDone && !fc.isGUIquit)
            {
                SendMCOM_Thread(id: 'Y', dat_num: 1);
                LogEvent_Thread("[Exit] SUCCEEDED: Save Enabled");
                fc.isSaveEnabled = true;
            }
            else
                LogEvent_Thread("**WARNING** [Exit] ABORTED: Save Enabled", is_warning: true);

            // Wait for last packet
            LogEvent_Thread("[Exit] RUNNING: Wait for Last Pack...");
            pass = WaitForSerial(id_arr: c2r.id, timeout: 5000);
            if (pass)
                LogEvent_Thread("[Exit] SUCCEEDED: Wait for Last Pack");
            else
                LogEvent_Thread("**WARNING** [Exit] ABORTED: Wait for Last Pack", is_warning: true);

            // Send initial robot log request
            LogEvent_Thread("[Exit] RUNNING: Request Robot Log...");
            RepeatSendPack_Thread(id: 'L', dat1: 0);
            pass = WaitForSerial(id: 'L', timeout: 5000);
            if (pass)
            {
                // Tell CheetahDue logging beggining
                //byte[] out_byte = new byte[1] { (byte)'l' };
                //sp_cheetahDue.Write(out_byte, 0, 1);

                // Wait for bytes to receive messages to be received
                LogEvent_Thread("[Exit] RUNNING: Wait for Robot Log Bytes...");
                pass = WaitForSerial(id: 'U', check_for: "rcv", timeout: 5000);
                if (pass)
                {
                    // Store data byte
                    robLogger.UpdateBytesToRcv(r2c.dat[r2c.ID_Ind('U')]);
                    LogEvent_Thread(String.Format("[Exit] SUCCEEDED: Wait for Robot Log Bytes: bytes_expected={0}", robLogger.bytesToRcv));

                    // Flag logging started and block ParserR2C()
                    robLogger.isLogging = true;

                    // Start importing
                    Thread get_log_thread = new Thread(delegate ()
                    {
                        GetRobotLog();
                    });
                    get_log_thread.Priority = ThreadPriority.Highest;
                    get_log_thread.Start();

                    // Tell robot to begin streaming log and wait for message to send
                    RepeatSendPack_Thread(id: 'L', dat1: 1, do_conf: false);
                    pass = WaitForSerial(id: 'L', timeout: 5000);
                    LogEvent_Thread("[Exit] SUCCEEDED: Request Robot Log");
                }
                else
                {
                    LogEvent_Thread(String.Format("[Exit] !!ERROR!! ABORTED: Wait for Robot Log Bytes: bytes_expected={0}", robLogger.bytesToRcv), is_error: true);
                    fc.isRunError = true;
                }
            }
            else
            {
                LogEvent_Thread("!!ERROR!! [Exit] FAILED: Request Robot Log", is_error: true);
                fc.isRunError = true;
            }


            // Wait for save complete
            if (fc.isSaveEnabled && !fc.isGUIquit)
            {
                LogEvent_Thread("[Exit] RUNNING: Wait for ICR_GUI to Save...");
                pass = WaitForMCOM(id: 'F', do_abort: true);
                if (pass)
                {
                    LogEvent_Thread("[Exit] SUCCEEDED: Wait for ICR_GUI to Save");

                    // Get NLX dir
                    dynamic nlx_rec_dir = GetMCOM(msg: "m2c_dir");
                    nlxRecDir = (string)nlx_rec_dir;

                    // Confirm log saved
                    LogEvent_Thread(String.Format("SET RECORDING DIR TO \"{0}\"", nlxRecDir));

                }
                else
                {
                    LogEvent_Thread("**WARNING** [Exit] ABORTED: Wait for ICR_GUI to Save", is_warning: true);
                }
            }

            // Wait for quit command
            LogEvent_Thread("[Exit] RUNNING: Wait for ICR_GUI Quit command...");
            if (!fc.isGUIquit)
                pass = WaitForMCOM(id: 'X', do_abort: true);
            else
                pass = true;
            if (pass)
                LogEvent_Thread("[Exit] SUCCEEDED: Wait for ICR_GUI Quit command");
            else
                LogEvent_Thread("**WARNING** [Exit] ABORTED: Wait for ICR_GUI Quit command", is_warning: true);

            // Wait for robot log save to complete
            LogEvent_Thread("[Exit] RUNNING: Wait for Robot Log Save...");
            while (!robLogger.isFinished && !robLogger.isImportTimedout)
                Thread.Sleep(10);

            // Check if complete log was imported
            if (robLogger.isLogComplete)
                LogEvent_Thread(String.Format("[Exit] SUCCEEDED: Wait for Robot Log Save: logged={0} dropped={1} bytes_read={2} bytes_expected={3} dt_run={4}",
                    robLogger.cnt_logsStored, robLogger.cnt_dropped[1], robLogger.bytesRead, robLogger.bytesToRcv, robLogger.logDT));
            else if (robLogger.cnt_logsStored > 0)
                LogEvent_Thread(String.Format("**WARNING** [Exit] PARTIALLY SUCCEEDED: Wait for Robot Log Save: logged={0} dropped={1} bytes_read={2} bytes_expected={3} dt_run={4}",
                    robLogger.cnt_logsStored, robLogger.cnt_dropped[1], robLogger.bytesRead, robLogger.bytesToRcv, robLogger.logDT), is_warning: true);
            else
            {
                LogEvent_Thread(String.Format("!!ERROR!! [Exit] FAILED: Wait for Robot Log Save: logged={0} dropped={1} bytes_read={2} bytes_expected={3} dt_run={4}",
                    robLogger.cnt_logsStored, robLogger.cnt_dropped[1], robLogger.bytesRead, robLogger.bytesToRcv, robLogger.logDT), is_error: true);
                fc.isRunError = true;
            }

            // Send command for arduino to quit on seperate thread
            RepeatSendPack_Thread(id: 'Q');
            // Wait for quit confirmation from robot for fixed period of time
            LogEvent_Thread("[Exit] RUN: Confirm Robot Quit...");
            pass = WaitForSerial(id: 'Q', timeout: 5000);
            if (pass)
                LogEvent_Thread("[Exit] SUCCEEDED: Confirm Robot Quit");
            else
                LogEvent_Thread("**WARNING** [Exit] ABORTED: Confirm Robot Quit", is_warning: true);
            // Set flags
            fc.isRobComActive = false;
            fc.isArdComActive = false;

            // Send command for ICR_GUI to exit
            SendMCOM_Thread(id: 'E', dat_num: 1);
            LogEvent_Thread("[Exit] FINISHED: Tell ICR_GUI to Close");

            // Wait for GUI to close
            LogEvent_Thread("[Exit] RUNNING: Confirm ICR_GUI Closed...");
            pass = WaitForMCOM(id: 'C', timeout: timeoutMatCloseConfirm);
            if (pass)
                LogEvent_Thread("[Exit] SUCCEEDED: Confirm ICR_GUI Closed");
            else
                LogEvent_Thread("**WARNING** [Exit] ABORTED: Confirm ICR_GUI Closed", is_warning: true);

            // Tell Matlab close confirmation received
            LogEvent_Thread("[Exit] RUNNING: Send ICR_GUI Close Confirmation Received...");
            SendMCOM_Thread(id: 'C', dat_num: 1);
            pass = WaitForMCOM(id: 'C', check_for: "send", timeout: timeoutMatCloseConfirm);
            if (pass)
                LogEvent_Thread("[Exit] SUCCEEDED: Send ICR_GUI Close Confirmation Received");
            else
                LogEvent_Thread("**WARNING** [Exit] ABORTED: Send ICR_GUI Close Confirmation Received", is_warning: true);

            // Set exit flag to exit all threads
            fc.doExit = true;

            // Wait for Matlab and threads to close down
            Thread.Sleep(1000);

            // Dispose of workers
            //bw_RunGUI.CancelAsync();
            bw_RunGUI.Dispose();
            LogEvent_Thread("[Exit] FINISHED: Dispose RunGUI Worker");
            //bw_MatCOM.CancelAsync();
            bw_MatCOM.Dispose();
            LogEvent_Thread("[Exit] FINISHED: Dispose MatCOM Worker");

            // Save CheetahDue log file
            LogEvent_Thread("[Exit] RUNNING: Save CheetahDue Log...");
            dueLogger.SaveLog(logDir, dueLogFi);
            if (dueLogger.isLogComplete)
                LogEvent_Thread(String.Format("[Exit] SUCCEEDED: Save CheetahDue Log: logged={0} dropped={1}",
                    dueLogger.cnt_logsStored, dueLogger.cnt_dropped[1]));
            else if (dueLogger.cnt_logsStored > 0)
                LogEvent_Thread(String.Format("**WARNING** [Exit] PARTIALLY SUCCEEDED: Save CheetahDue Log: logged={0} dropped={1}",
                    dueLogger.cnt_logsStored, dueLogger.cnt_dropped[1]), is_warning: true);
            else
            {
                LogEvent_Thread(String.Format("!!ERROR!! [Exit] FAILED: Save CheetahDue Log: logged={0} dropped={1}",
                    dueLogger.cnt_logsStored, dueLogger.cnt_dropped[1]), is_error: true);
                fc.isRunError = true;
            }

            // Clear all MatCOM vars
            SendMCOM_Thread(msg: "clearvars -global -except 'ME' 'startTime';");
            SendMCOM_Thread(msg: "close all;");
            LogEvent_Thread("[Exit] FINISHED: Clear MatCom Globals");

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
                LogEvent_Thread("[Exit] FINISHED: Close MatCOM");
            }

            // Kill that mother fucker!
            else
            {
                KillMatlab();
                LogEvent_Thread("**WARNING** [Exit] HAD TO KILL MATLAB", is_warning: true);
            }

            // Log/print run summary
            LogEvent(msg_in: csLogger.GetSummary("warnings"));
            LogEvent(msg_in: csLogger.GetSummary("errors"));

            // Save CS log file
            LogEvent_Thread("[Exit] RUNNING: Save CS Log...");
            csLogger.SaveLog(logDir, csLogFi);
            LogEvent_Thread(String.Format("[Exit] FINISHED: Save CS Log: logged={0}", csLogger.cnt_logsStored));

            // Copy log files to rat specific dir
            if (nlxRecDir != logDir)
            {
                // Create copies
                System.IO.File.Copy(System.IO.Path.Combine(logDir, robLogFi), System.IO.Path.Combine(nlxRecDir, robLogFi), true);
                System.IO.File.Copy(System.IO.Path.Combine(logDir, dueLogFi), System.IO.Path.Combine(nlxRecDir, dueLogFi), true);
                System.IO.File.Copy(System.IO.Path.Combine(logDir, csLogFi), System.IO.Path.Combine(nlxRecDir, csLogFi), true);
                System.IO.File.Copy(System.IO.Path.Combine(logDir, matLogFi), System.IO.Path.Combine(nlxRecDir, matLogFi), true);

                // Confirm logs copied saved
                LogEvent_Thread(String.Format("COPPIED LOG FILES TO \"{0}\"", nlxRecDir));
            }

        }

        #endregion

        #region ========= SERIAL COMMUNICATION ==========

        // SEND PACK DATA REPEATEDLY TILL RECIEVED CONFIRMED
        public static void RepeatSendPack_Thread(int send_max = 0, char id = ' ', double dat1 = double.NaN, double dat2 = double.NaN, double dat3 = double.NaN, UInt16 pack = 0, bool do_conf = true, bool do_check_done = false)
        {
            // Local vars
            double[] dat = new double[3] { dat1, dat2, dat3 };

            // Update c2r queue info
            c2r.SetCheckFor(id: id, do_check_sent_rcvd: true, do_check_conf: do_conf, do_check_done: do_check_done);

            // Run method on seperate thread
            new Thread(delegate ()
            {
                RepeatSendPack(send_max: send_max, id: id, dat: dat, pack: pack, do_conf: do_conf, do_check_done: do_check_done);
            }).Start();

        }
        public static void RepeatSendPack(int send_max, char id, double[] dat, UInt16 pack, bool do_conf, bool do_check_done)
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
                    if (!c2r.Wait4RcvConf(id))
                        return;

                    // Check if streaming has failed
                    else if (send_count >= resendMax)
                    {
                        // Log/print
                        LogEvent_Thread(String.Format("!!ERROR!! [RepeatSendPack_Thread] ABBORTED: Resending c2r: cnt={0} id=\'{1}\' dat=|{2:0.00}|{3:0.00}|{4:0.00}| pack={5} do_conf={6} do_check_done={7}",
                            send_count, id, dat[0], dat[1], dat[2], pack, do_conf, do_check_done), is_error: true);

                        // Set error flags
                        fc.isRunError = true;
                        fc.isRobComActive = false;
                        return;
                    }

                    // Need to resend
                    else if (sw_main.ElapsedMilliseconds > t_resend)
                    {
                        // Log/print
                        LogEvent_Thread(String.Format("**WARNING** [RepeatSendPack_Thread] Resending c2r: cnt={0} id=\'{1}\' dat=|{2:0.00}|{3:0.00}|{4:0.00}| pack={5} do_conf={6} do_check_done={7}",
                            send_count, id, dat[0], dat[1], dat[2], pack, do_conf, do_check_done), is_warning: true);

                        // Resend
                        SendPack(id: id, dat: dat, pack: pack, do_conf: do_conf, do_check_done: do_check_done);
                        t_resend = sw_main.ElapsedMilliseconds + dt_resend;
                        send_count++;
                    }
                }
            }
        }

        // SEND PACK DATA
        public static UInt16 SendPack(char id, double[] dat, UInt16 pack, bool do_conf, bool do_check_done)
        {
            /* 
            SEND DATA TO ROBOT 
            FORMAT: [0]head, [1]id, [2:5]dat1, [6:9]dat2, [10:13]dat2, [14:15]pack, [16]do_conf, [17]footer
            EXAMPLE: ASCII {'<','L','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','\0','>','c'} DEC {60,76,1,255,255,0,60}
            */

            // Track when data queued
            long t_queued = sw_main.ElapsedMilliseconds;
            lock (lock_queue_sendPack) queue_SendXBee++;

            lock (lock_sendPack)
            {
                // Block vt sending
                vtHandler.Block(id);

                // Local vars
                UnionHack U = new UnionHack(0, '0', 0, 0, 0);
                byte[] msg_id = new byte[1];
                byte[] msg_data = new byte[12];
                byte[] msg_pack = new byte[2];
                byte[] msg_conf = new byte[1];
                long t_send = 0;
                bool do_loop = true;
                bool buff_ready = false;
                bool is_recieving = false;
                bool is_clogged = false;
                bool is_hanging = false;
                string dat_str = "";
                string buff_str = "";
                long dt_rcvd = 0;

                // Wait for next safe send time
                while (do_loop)
                {
                    // Delay send time till x ms after last send or rcvd
                    t_send = c2r.t_new > r2c.t_new ? c2r.t_new + dt_sendSent : r2c.t_new + dt_sendRcvd;

                    // Make sure outbut and input buffer have enough space
                    buff_ready = sp_Xbee.BytesToWrite == 0 && sp_Xbee.BytesToRead == 0;

                    // Check if loop should continue
                    do_loop =
                        (sw_main.ElapsedMilliseconds < t_send || !buff_ready) &&
                        fc.ContinueRobCom();

                    // Get status
                    is_recieving = fc.isXbeeBusy;
                    is_clogged = queue_SendXBee >= 3;
                    is_hanging = sw_main.ElapsedMilliseconds > t_queued + 100;

                    // Abort if sending pos data
                    if ((is_clogged || is_hanging || is_recieving) &&
                        id == 'P')
                        break;

                }

                // Get new packet number
                if (pack == 0)
                {
                    c2r.cnt_pack++;
                    pack = c2r.cnt_pack;
                }

                // Format data string
                dat_str = String.Format("id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} do_conf={5} do_check_done={6} ",
                    id, dat[0], dat[1], dat[2], pack, do_conf, do_check_done);

                // Check if queue backed up or hanging
                if (is_clogged || is_hanging || is_recieving)
                {
                    // Get status info
                    dt_rcvd = Math.Max(r2c.DT_SentRcvd(sw_main.ElapsedMilliseconds), r2a.DT_SentRcvd(sw_main.ElapsedMilliseconds));
                    buff_str = String.Format("tx={0} rx={1} queued={2} dt_queue={3} dt_send={4} dt_rcvd={5}",
                   sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, queue_SendXBee, sw_main.ElapsedMilliseconds - t_queued, c2r.DT_SentRcvd(), dt_rcvd);

                    // Log/print
                    LogEvent_Thread(String.Format("**WARNING** [SendPack] c2r Queue |{0}{1}{2}: {3}",
                        is_recieving ? "Holding for RX|" : "", is_hanging ? "Hanging|" : "", is_clogged ? "Clogged|" : "", dat_str + buff_str), is_warning: true);

                    // Bail if this is pos data
                    if (id == 'P')
                    {
                        lock (lock_queue_sendPack) queue_SendXBee--;
                        c2r.cnt_pack--;
                        return pack;
                    }
                }

                // Store pack number
                U.i16_0 = pack;
                msg_pack[0] = U.b_0;
                msg_pack[1] = U.b_1;

                // Store ID
                U.c_0 = id;
                msg_id[0] = U.b_0;

                // Store data
                int b_ind = 0;
                for (int i = 0; i < 3; i++)
                {
                    U.f = (float)dat[i];
                    msg_data[b_ind++] = U.b_0;
                    msg_data[b_ind++] = U.b_1;
                    msg_data[b_ind++] = U.b_2;
                    msg_data[b_ind++] = U.b_3;
                }

                // Store do do_conf 
                msg_conf[0] = do_conf ? (byte)1 : (byte)0;

                // Concatinate header and footer
                byte[] msgByteArr = new byte[
                    c2r.head.Length +    // header
                    msg_id.Length +          // id
                    msg_data.Length +             // data
                    msg_pack.Length + // packet num
                    msg_conf.Length + // do_conf
                    c2r.foot.Length      // footer
                    ];
                // add header
                c2r.head.CopyTo(msgByteArr, 0);
                // add id
                msg_id.CopyTo(msgByteArr, c2r.head.Length);
                // add data
                msg_data.CopyTo(msgByteArr, c2r.head.Length + msg_id.Length);
                // add packet number
                msg_pack.CopyTo(msgByteArr, c2r.head.Length + msg_id.Length + msg_data.Length);
                // add do_conf
                msg_conf.CopyTo(msgByteArr, c2r.head.Length + msg_id.Length + msg_data.Length + msg_pack.Length);
                // add footer
                c2r.foot.CopyTo(msgByteArr, c2r.head.Length + msg_id.Length + msg_data.Length + msg_pack.Length + msg_conf.Length);

                // Send to arduino
                while (fc.isXbeeBusy) ;
                sp_Xbee.Write(msgByteArr, 0, msgByteArr.Length);

                // Update c2r info
                c2r.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: sw_main.ElapsedMilliseconds);

                // Store final status info
                dt_rcvd = Math.Max(r2c.DT_SentRcvd(sw_main.ElapsedMilliseconds), r2a.DT_SentRcvd(sw_main.ElapsedMilliseconds));
                buff_str = String.Format("bytes_sent={0} tx={1} rx={2} queued={3} dt_queue={4} dt_send={5} dt_rcvd={6}",
                    msgByteArr.Length, sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, queue_SendXBee, sw_main.ElapsedMilliseconds - t_queued, c2r.DT_SentRcvd(), dt_rcvd);

                // Check if sending pos data
                if (id != 'P')
                {
                    // Check for resend
                    if (pack != c2r.packLast[c2r.ID_Ind(id)])
                        LogEvent_Thread("   [SENT] c2r: " + dat_str + buff_str, t: c2r.t_new);
                    else
                    {
                        c2r.cnt_repeat++;
                        LogEvent_Thread(String.Format("   [*RE-SENT*] c2r: cnt={0} ", c2r.cnt_repeat) + dat_str + buff_str, t: c2r.t_new);
                    }

                }
                else
                {
                    // Track send rate
                    vtHandler.StoreSendTime((int)dat[0], sw_main.ElapsedMilliseconds);

                    // Log/print
                    if ((db.do_printSentRatVT && (int)dat[0] == 0) ||
                        (db.do_printSentRobVT && (int)dat[0] == 1))
                    {
                        U.f = (float)dat[2];
                        string dt_vt = String.Format(" ts_int={0} dt_send_mu={1}",
                            U.i32, vtHandler.GetSendDT((int)dat[0], "avg"));
                        LogEvent_Thread("   [SENT] c2r: " + dat_str + buff_str + dt_vt, t: c2r.t_new);
                    }
                }

                // Unlock vt sending
                vtHandler.Unblock(id);

            }
            lock (lock_queue_sendPack) queue_SendXBee--;

            // Return packet number
            return pack;

        }

        // WAIT FOR R2C CONFIRMATION
        public static bool WaitForSerial(char id, string check_for = "send", bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            char[] id_arr = { id };
            return WaitForSerial(id_arr: id_arr, check_for: check_for, do_abort: do_abort, timeout: timeout);
        }
        public static bool WaitForSerial(char[] id_arr, string check_for = "send", bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_start = sw_main.ElapsedMilliseconds;
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : t_start + timeout;
            char id = ' ';
            bool first_loop = true;
            bool wait_4_rcv = false;
            bool wait_4_send = false;
            bool wait_4_conf = false;
            bool wait_4_done = false;
            bool pass = false;
            string wait_str = " ";

            // Loop through each id
            for (int i = 0; i < id_arr.Length; i++)
            {
                // Current id
                id = id_arr[i];

                // Preset flags
                first_loop = true;
                wait_4_rcv = check_for == "rcv";
                wait_4_send = check_for == "send";
                wait_4_conf = check_for == "send";
                wait_4_done = check_for == "send";

                // Wait for confirmation
                do
                {

                    // Check if received
                    wait_4_rcv = wait_4_rcv ? r2c.Wait4SendRcv(id) : wait_4_rcv;

                    // Check if sent
                    wait_4_send = wait_4_send ? c2r.Wait4SendRcv(id) : wait_4_send;

                    // Wont check flag till sent
                    if (!wait_4_send)
                    {
                        wait_4_conf = wait_4_conf ? c2r.Wait4RcvConf(id) : wait_4_conf;
                        wait_4_done = wait_4_done ? c2r.Wait4DoneConf(id) : wait_4_done;
                    }

                    // Check if all conditions confirmed
                    pass = !wait_4_rcv && !wait_4_send && !wait_4_conf && !wait_4_done;

                    // Bail if nothing to wait for
                    if (first_loop && pass)
                        break;

                    // Check if anything to wait for
                    if (first_loop)
                        wait_str = String.Format("|{0}{1}{2}{3}",
                            check_for == "send" ? "Send|" : "", check_for == "rcv" ? "Rcv|" : "", c2r.Wait4RcvConf(id) ? "Conf|" : "", c2r.Wait4DoneConf(id) ? "Done|" : "");

                    // Get current status string
                    string dat_str = String.Format("Wait for {0}: id=\'{1}\' pack={2} check_for={3} do_abort={4} timeout={5} wait_4_conf={6} wait_4_done={7} dt_wait={8}",
                            wait_str, id, c2r.pack[c2r.ID_Ind(id)], check_for, do_abort, timeout == long.MaxValue ? 0 : timeout, wait_4_conf, wait_4_done, sw_main.ElapsedMilliseconds - t_start);

                    // Print what we are waiting on
                    if (first_loop)
                    {
                        LogEvent_Thread(String.Format("[WaitForSerial] RUNNING: {0}", dat_str));
                        first_loop = false;
                    }

                    // Check if all confirmed
                    if (pass)
                    {
                        LogEvent_Thread(String.Format("[WaitForSerial] SUCCEEDED: {0}", dat_str));
                        break;
                    }

                    // Check if need to abort
                    else if (
                        (do_abort && fc.doAbort) ||
                        !fc.ContinueRobCom() ||
                        sw_main.ElapsedMilliseconds > t_timeout
                        )
                    {

                        // External forced abort
                        if (do_abort && fc.doAbort)
                            LogEvent_Thread(String.Format("**WARNING** [WaitForSerial] Forced Abort: {0}", dat_str), is_warning: true);
                        else
                        {
                            // Coms failed
                            if (!fc.ContinueRobCom())
                                LogEvent_Thread(String.Format("!!ERROR!! [WaitForSerial] Lost Comms: {0}", dat_str), is_error: true);

                            // Timedout
                            else if (sw_main.ElapsedMilliseconds > t_timeout)
                                LogEvent_Thread(String.Format("!!ERROR!! [WaitForSerial] Timedout: {0}", dat_str), is_error: true);

                            // Set error flag
                            fc.isRunError = true;
                        }

                        // Reset flags so we dont check for this again
                        c2r.ResetCheckFor(id: id, reset_check_conf: true, reset_check_done: true);

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

        // HANDLE INCOMING XBEE DATA
        public static void DataReceived_Xbee(object sender, SerialDataReceivedEventArgs e)
        {
            // Prevent multiple intances running
            if (fc.isXbeeBusy ||
                robLogger.isLogging)
                return;

            // Block any xbee writing till complete packet received
            fc.isXbeeBusy = true;
            while (sp_Xbee.BytesToRead > 0)
                Thread.Sleep(1);

            // Flag xbee free
            fc.isXbeeBusy = false;
        }

        // PARSE RECIEVED XBEE DATA 
        public static void ParserR2C()
        {
            /* 
            RECIEVE DATA FROM FEEDERDUE 
            FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
            */

            // Loop till all data read out
            while (!fc.doExit)
            {

                // Bail if no new data or processing robot log
                if (sp_Xbee.BytesToRead < 1 ||
                    robLogger.isLogging)
                {
                    Thread.Sleep(1);
                    continue;
                }

                // Local vars
                long t_parse_str = sw_main.ElapsedMilliseconds;
                UnionHack U = new UnionHack(0, '0', 0, 0, 0);
                bool r2c_head_found = false;
                bool r2a_head_found = false;
                bool r2c_id_found = false;
                bool r2a_id_found = false;
                bool r2c_foot_found = false;
                bool r2a_foot_found = false;
                byte[] head_bytes = new byte[1];
                byte[] id_bytes = new byte[1];
                byte[] conf_bytes = new byte[1];
                byte[] dat_bytes = new byte[4];
                byte[] pack_bytes = new byte[2];
                byte[] foot_bytes = new byte[1];
                int bytes_read = 0;
                char head = ' ';
                char id = ' ';
                double[] dat = new double[3];
                UInt16 pack = 0;
                bool do_conf = false;
                char foot = ' ';

                // Get header
                if (XbeeBuffReady(1, t_parse_str, "head"))
                {
                    sp_Xbee.Read(head_bytes, 0, 1);
                    bytes_read += 1;
                    // Get header
                    U.b_0 = head_bytes[0];
                    U.b_1 = 0; // C# chars are 2 bytes
                    head = U.c_0;

                    // Check if for r2c head
                    if (head == r2c.head[0])
                        r2c_head_found = true;

                    // Check if for r2a head
                    if (head == r2a.head[0])
                        r2a_head_found = true;

                }

                // Find id and check message is intended for CS
                if (r2c_head_found || r2a_head_found)
                {

                    if (XbeeBuffReady(1, t_parse_str, "id"))
                    {

                        sp_Xbee.Read(id_bytes, 0, 1);
                        bytes_read += 1;
                        // Get id
                        U.b_0 = id_bytes[0];
                        U.b_1 = 0;
                        id = U.c_0;

                        // Check for r2c id 
                        for (int i = 0; i < r2c.id.Length; i++)
                        {
                            if (id == r2c.id[i])
                            {
                                r2c_id_found = true;
                                break;
                            }
                        }

                        // Check for r2a id 
                        for (int i = 0; i < r2a.id.Length; i++)
                        {
                            if (id == r2a.id[i])
                            {
                                r2a_id_found = true;
                                break;
                            }
                        }

                    }
                }

                // Get data, packet number and do_conf flag
                if ((r2c_head_found && r2c_id_found) ||
                    (r2a_head_found && r2a_id_found))
                {

                    // Get data
                    for (int i = 0; i < 3; i++)
                    {
                        if (XbeeBuffReady(4, t_parse_str, String.Format("dat{0}", i + 1)))
                        {
                            sp_Xbee.Read(dat_bytes, 0, 4);
                            U.b_0 = dat_bytes[0];
                            U.b_1 = dat_bytes[1];
                            U.b_2 = dat_bytes[2];
                            U.b_3 = dat_bytes[3];
                            dat[i] = (double)U.f;
                        }
                        bytes_read += 4;
                    }

                    // Get packet number
                    if (XbeeBuffReady(2, t_parse_str, "pack"))
                    {

                        // Read in data
                        sp_Xbee.Read(pack_bytes, 0, 2);
                        bytes_read += 2;
                        U.b_0 = pack_bytes[0];
                        U.b_1 = pack_bytes[1];
                        pack = U.i16_0;
                    }

                    // Get do confirm byte
                    if (XbeeBuffReady(1, t_parse_str, "do_comf"))
                    {
                        sp_Xbee.Read(conf_bytes, 0, 1);
                        bytes_read += 1;
                        // Get bool
                        do_conf = conf_bytes[0] == 1 ? true : false;
                    }

                    // Find footer
                    if (XbeeBuffReady(1, t_parse_str, "foot"))
                    {
                        // Read in data
                        sp_Xbee.Read(foot_bytes, 0, 1);
                        bytes_read += 1;

                        // Check footer
                        U.b_0 = foot_bytes[0];
                        U.b_1 = 0;
                        foot = U.c_0;

                        // Check for r2c foot
                        if (foot == r2c.foot[0])
                            r2c_foot_found = true;

                        // Check for r2a foot
                        if (foot == r2a.foot[0])
                            r2a_foot_found = true;

                    }
                }

                // Format data string
                long dt_rcvd = Math.Max(r2c.DT_SentRcvd(sw_main.ElapsedMilliseconds), r2a.DT_SentRcvd(sw_main.ElapsedMilliseconds));
                long dt_parse = sw_main.ElapsedMilliseconds - t_parse_str;
                string dat_str = String.Format("id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} do_conf={5} bytes_read={6} rx={7} tx={8} dt_parse={9} dt_send={10} dt_rcv={11}",
                        id, dat[0], dat[1], dat[2], pack, do_conf, bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, dt_parse, c2r.DT_SentRcvd(), dt_rcvd);

                // Store r2c packet data
                if (r2c_foot_found)
                {
                    // Update flags
                    r2c.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: sw_main.ElapsedMilliseconds);

                    // Check for repeat packet
                    string msg_str;
                    if (pack != r2c.packLast[r2c.ID_Ind(id)])
                    {
                        msg_str = "   [RCVD] r2c: ";
                    }
                    else
                    {
                        r2c.cnt_repeat++;
                        msg_str = String.Format("   [*RE-RCVD*] r2c: cnt={0} ", r2c.cnt_repeat);
                    }

                    // Log/print rcvd details
                    LogEvent_Thread(msg_str + dat_str, t: r2c.t_new);

                    // Check if this is a recieve confirmation
                    if (c2r.ID_Ind(id) != -1)
                        c2r.ResetCheckFor(id: id, reset_check_conf: true);

                    // Check if this is a done confirmation
                    if (id == 'D')
                        c2r.ResetCheckFor(id: c2r.PackID(pack), reset_check_done: true);

                    // Send recieve confirmation
                    if (do_conf)
                        RepeatSendPack_Thread(send_max: 1, id: id, dat1: dat[0], dat2: dat[1], dat3: dat[2], pack: pack, do_conf: false);

                    // Check if data should be relayed to Matlab
                    if (c2m.ID_Ind(id) != -1)
                        SendMCOM_Thread(id: id, dat_num: dat[0]);

                }

                // Handle r2a packet
                else if (r2a_foot_found)
                {
                    // Update flags
                    r2a.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: sw_main.ElapsedMilliseconds);

                    // Log/print rcvd details
                    LogEvent_Thread("   [RCVD] r2a: " + dat_str, t: r2a.t_new);
                }

                // If all data found restart loop
                if ((r2c_head_found && r2c_id_found && r2c_foot_found) ||
                    (r2a_head_found && r2a_id_found && r2a_foot_found))
                {
                    // Change streaming status
                    if (!fc.isRobComActive)
                        fc.isRobComActive = true;

                    continue;
                }

                // If no bytes read restart loop
                else if (bytes_read == 0)
                {
                    continue;
                }

                // Otherwise handle dropped data
                else
                {
                    // Add to dropped count
                    if (r2c_head_found)
                        r2c.AddDropped(1);
                    else if (r2a_head_found)
                        r2a.AddDropped(1);
                    else
                        r2c.AddDropped(1);

                    // Get from info
                    char from = r2c_head_found ? 'c' : r2a_head_found ? 'a' : '?';

                    // Get drop count info
                    int[] cnt_dropped = new int[2] { r2c_head_found ? r2c.cnt_dropped[0] : r2a_head_found ? r2a.cnt_dropped[0] : r2c.cnt_dropped[0],
                        r2c_head_found ? r2c.cnt_dropped[1] : r2a_head_found ? r2a.cnt_dropped[1] : r2c.cnt_dropped[1] };

                    // Get found flags info
                    string found = String.Format("|{0}|{1}|{2}|",
                        r2c_head_found ? "r2c_head" : r2a_head_found ? "r2a_head" : "no_head",
                        r2c_id_found ? "r2c_id" : r2a_id_found ? "r2a_id" : "no_id",
                        r2c_foot_found ? "r2c_foot" : r2a_foot_found ? "r2a_foot" : "no_foot");

                    // Log/print available info
                    LogEvent_Thread(String.Format("**WARNING** [ParseR2C] Dropped r2{0} Packet: dropped={1}|{2} found={3} head={4} id=\'{5}\' dat=|{6:0.00}|{7:0.00}|{8:0.00}| pack={9} do_conf={10} foot={11} bytes_read={12} rx={13} tx={14} parse_dt={15}",
                        from, cnt_dropped[0], cnt_dropped[1], found, head, id, dat[0], dat[1], dat[2], pack, do_conf, foot, bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, sw_main.ElapsedMilliseconds - t_parse_str), is_warning: true);

                    // Dump buffer if > 1 consecutive drops and no bytes read
                    if (cnt_dropped[0] > 1)
                    {
                        LogEvent_Thread("**WARNING** [ParseR2C] Dumping r2 Input Buffer", is_warning: true);
                        sp_Xbee.DiscardInBuffer();
                    }
                }

            }

        }

        // WAIT FOR XBEE BUFFER TO FILL
        public static bool XbeeBuffReady(int min_byte, long t_parse_str, string getting)
        {
            // Local vars
            long t_str = sw_main.ElapsedMilliseconds;
            long t_timeout = t_parse_str + 1000;
            string dat_str = "";
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_Xbee.BytesToRead < min_byte &&
                sw_main.ElapsedMilliseconds <= t_timeout &&
                fc.ContinueRobCom()
                ) ;

            // Store data string
            dat_str = String.Format("getting=\"{0}\" min_byte={1} dt_check={2} dt_queue={3} rx={4} tx={5}",
                     getting, min_byte, sw_main.ElapsedMilliseconds - t_str, sw_main.ElapsedMilliseconds - t_parse_str, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite);

            // Check if hanging
            if (sw_main.ElapsedMilliseconds > t_str + 25)
            {
                LogEvent_Thread("**WARNING** [XbeeBuffReady] XBee Read Hanging: " + dat_str, is_warning: true);
            }

            // Check if timedout
            if (sw_main.ElapsedMilliseconds > t_timeout)
            {
                LogEvent_Thread("**WARNING** [XbeeBuffReady] TIMEDOUT: " + dat_str, is_warning: true);
            }

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
                UnionHack U = new UnionHack(0, '0', 0, 0, 0);
                bool head_found = false;
                bool foot_found = false;
                byte[] head_bytes = new byte[1];
                byte[] foot_bytes = new byte[1];
                byte[] chksum_bytes = new byte[1];
                int bytes_read = 0;
                char head = ' ';
                UInt16 chksum = 0;
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
                    U.b_0 = head_bytes[0];
                    U.b_1 = 0; // C# chars are 2 bytes
                    head = U.c_0;
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
                        U.b_0 = foot_bytes[0];
                        U.b_1 = 0;
                        foot = U.c_0;
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
                            LogEvent_Thread(String.Format("   [LOG] a2c[{0}]: message=\"{1}\" chksum={2} bytes_read={3} rx={4} tx={5}",
                                dueLogger.cnt_logsStored, log_str, chksum, bytes_read, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));
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
                    LogEvent_Thread(String.Format("**WARNING** [GetArdLog] Dropped a2c Log: logged={0} dropped={1}|{2} head={3} message=\"{4}\" chksum={5} foot={6} bytes_read={7} rx={8} tx={9}",
                       dueLogger.cnt_logsStored, dueLogger.cnt_dropped[0], dueLogger.cnt_dropped[1], head, log_str, chksum, foot, bytes_read, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite), is_warning: true);

                    // Dump buffer if > 1 consecutive drops and no bytes read
                    if (dueLogger.cnt_dropped[0] > 1 && bytes_read == 0)
                    {
                        LogEvent_Thread("**WARNING** [GetArdLog] Dumping a2c Input Buffer", is_warning: true);
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
                    LogEvent_Thread(String.Format("**WARNING** [ArdBuffReady] a2c Hanging: dt_check={0} rx={1} tx={2}",
                      (sw_main.ElapsedMilliseconds - t_timeout) + timeout, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite), is_warning: true);
                else
                    LogEvent_Thread(String.Format("**WARNING** [ArdBuffReady] ABORTED: dt_check={0} rx={1} tx={2}",
                   (sw_main.ElapsedMilliseconds - t_timeout) + timeout, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite), is_warning: true);
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
            char[] c_arr = new char[3] { '\0', '\0', '\0' };

            // Start log import
            LogEvent_Thread("[GetRobotLog] RUNNING: Robot Log Import...");

            // Read stream vars
            int read_ind = 0;
            char[] in_arr = new char[1000000];
            long dt_run = 0;
            long dt_read = 0;
            bool is_timedout = false;
            bool is_robot_abort = false;
            bool send_complete = false;

            // Read stream till ">>>" string
            while (!is_timedout)
            {

                // Check for timeout
                if (sw_main.ElapsedMilliseconds > t_timeout)
                {
                    if (sw_main.ElapsedMilliseconds - t_read_last > 5000)
                    {
                        dt_run = sw_main.ElapsedMilliseconds - t_stream_str;
                        dt_read = t_read_last == 0 ? 0 : sw_main.ElapsedMilliseconds - t_read_last;
                        is_timedout = true;
                    }
                }

                // Get next byte
                if (sp_Xbee.BytesToRead < 1)
                {
                    continue;
                }

                // Check progress
                string status_str = robLogger.GetImportStatus(read_ind);
                if (status_str != " ")
                {
                    // Print progress on seperate thread
                    LogEvent_Thread(String.Format("[GetRobotLog] Log Import {0}", status_str));
                }

                // Get next char
                c_arr[0] = c_arr[1];
                c_arr[1] = c_arr[2];
                c_arr[2] = (char)sp_Xbee.ReadByte();
                in_arr[read_ind] = c_arr[2];
                t_read_last = sw_main.ElapsedMilliseconds;

                // Itterate count
                read_ind++;

                // Check for end
                if (
                    c_arr[0] == '>' &&
                    c_arr[1] == '>'
                )
                {
                    if (c_arr[2] == '>')
                    {
                        // Print final status
                        LogEvent_Thread(String.Format("[GetRobotLog] Log Import {0}",
                            robLogger.prcnt_str[robLogger.prcnt_str.Length - 1]));

                        // Print success termination string received
                        LogEvent_Thread(String.Format("[GetRobotLog] Received Send Success Termination String: \"{0}{1}{2}\" rx={3} tx={4}",
                            c_arr[0], c_arr[1], c_arr[2], sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite));
                        send_complete = true;

                        // Break out of loop
                        break;
                    }
                    else if (c_arr[2] == '!')
                    {
                        // Print abort termination string received
                        LogEvent_Thread(String.Format("**WARNING** [GetRobotLog] Received Abort Termination String: \"{0}{1}{2}\" rx={3} tx={4}",
                            c_arr[0], c_arr[1], c_arr[2], sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite), is_warning: true);
                        is_robot_abort = true;

                        // Break out of loop
                        break;
                    }
                }
            }

            // Dump whatever is left in buffer
            sp_Xbee.DiscardInBuffer();

            // Reset logging flag
            robLogger.isLogging = false;

            // Store bytes read
            robLogger.bytesRead = read_ind;

            // Store data string
            string dat_str = String.Format("bytes_read={0} bytes_expected=~{1} rx={2} tx={3} dt_stream={4}",
                    robLogger.bytesRead, robLogger.bytesToRcv, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, robLogger.logDT);

            // Check if logging timed out
            if (!send_complete)
            {
                // Bail if no bytes read
                if (robLogger.bytesRead == 0)
                {
                    LogEvent_Thread(String.Format("!!ERROR!! [GetRobotLog] FAILED: Robot Log Import: {0}: dt_read_last={1} dt_run={2} {3}",
                        is_timedout ? "Read Timedout" : is_robot_abort ? "Robot Aborted" : "Reason Unknown", dt_read, dt_run, dat_str), is_error: true);
                    fc.isRunError = true;
                    robLogger.isImportTimedout = true;

                    // Bail
                    return;
                }
                else
                {
                    LogEvent_Thread(String.Format("**WARNING** [GetRobotLog] ABORTED: Robot Log Import: {0}: dt_read_last={1} dt_run={2} {3}",
                        is_timedout ? "Read Timedout" : is_robot_abort ? "Robot Aborted" : "Reason Unknown", dt_read, dt_run, dat_str), is_warning: true);
                }
            }
            else
            {
                // Finished log import
                LogEvent_Thread(String.Format("[GetRobotLog] SUCCEEDED: Robot Log Import: {0}", dat_str));
            }

            // Start log store
            LogEvent_Thread("[GetRobotLog] RUNNING: Robot Log Store...");

            // Parse string and store logs
            char[] out_arr = new char[1000];
            int write_ind = 0;
            char c = '\0';
            bool do_rec_store = false;
            char[] int_str = new char[5] { '\0', '\0', '\0', '\0', '\0' };
            int int_cnt = 0;
            int rec_now = 0;
            int rec_last = 0;
            for (int i = 0; i < robLogger.bytesRead; i++)
            {

                // Get next char
                c = in_arr[i];

                // Check for count head
                if (!do_rec_store && write_ind == 0 && c == '[')
                    do_rec_store = true;

                // Check for count foot
                else if (do_rec_store && c == ']')
                {
                    // Try to parse record number
                    try
                    {
                        rec_now = int.Parse(new string(int_str, 0, int_cnt));
                    }
                    catch
                    {
                        LogEvent_Thread(String.Format("**WARNING** [GetRobotLog] Failed to Parse r2c Log Number: rec_last={0}",
                                rec_now), is_warning: true);
                    }

                    // Reset flags
                    int_cnt = 0;
                    do_rec_store = false;
                }

                // Store new record
                else if (do_rec_store)
                {
                    if (int_cnt < 5)
                    {
                        int_str[int_cnt] = c;
                        int_cnt++;
                    }
                    // Lost foot byte
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
                        LogEvent_Thread(String.Format("**WARNING** [GetRobotLog] Dropped r2c Log: logged_expected={0} logs_stored={1} dropped={2}|{3}",
                            rec_now, robLogger.cnt_logsStored, robLogger.cnt_dropped[0], robLogger.cnt_dropped[1]), is_warning: true);
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
            LogEvent_Thread(String.Format("[GetRobotLog] FINISHED: Robot Log Store: logs_stored={0} dt_run={1}",
                robLogger.cnt_logsStored, robLogger.logDT));

            // Save robot log file
            LogEvent_Thread("[GetRobotLog] RUNNING: Robot Log Save...");
            robLogger.SaveLog(logDir, robLogFi);
            LogEvent_Thread("[GetRobotLog] FINISHED: Robot Log Save");
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
            LogEvent_Thread("[DoWork_RunGUI] RUNNING: Setup Matlab paths...");
            SendMCOM(msg: @"addpath(genpath('" + matStartDir + "'));");
            LogEvent_Thread("[DoWork_RunGUI] FINISHED: Setup Matlab paths");

            // Run startup.m
            LogEvent_Thread("[DoWork_RunGUI] RUNNING: startup.m...");
            com_Matlab.Feval("startup", 0, out startup_result);
            LogEvent_Thread("[DoWork_RunGUI] FINISHED: startup.m...");

            // Wait for queue to clear
            while (queue_sendMCOM > 0)
                Thread.Sleep(10);

            // Run ICR_GUI.m
            LogEvent_Thread("[DoWork_RunGUI] RUNNING: ICR_GUI.m...");
            com_Matlab.Feval("ICR_GUI", 1, out icr_gui_result, db.systemTest, db.do_debugMat, false);

            // Get status
            object[] res = icr_gui_result as object[];
            status = res[0] as string;
            if (status == null)
                status = " ";

            // Print status
            if (status == "succeeded")
                LogEvent_Thread("[DoWork_RunGUI] SUCCEEDED: ICR_GUI.m");
            else if (status != " ")
            {
                LogEvent_Thread(String.Format("!!ERROR!! [DoWork_RunGUI] FAILED: ICR_GUI.m Error: {0}", status), is_error: true);
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
                LogEvent_Thread("[RunWorkerCompleted_RunGUI] SUCCEEDED: RunGUI Worker");
            else if (status != " ")
            {
                // Run failed but errors were caught
                LogEvent_Thread("**WARNING** [RunWorkerCompleted_RunGUI] ABORTED: RunGUI Worker", is_warning: true);
            }
            else
            {
                // Run failed completely
                LogEvent_Thread("!!ERROR!! [RunWorkerCompleted_RunGUI] FAILED WITHOUT CATCHING ERRORS: RunGUI Worker", is_error: true);
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
            double flag = 0;

            LogEvent_Thread("[DoWork_MatCOM] RUNNING: MatCOM Worker...");
            BackgroundWorker worker = (BackgroundWorker)sender;

            // Create tuple to pass args
            Tuple<char, double, double, double, UInt16> bw_args = (Tuple<char, double, double, double, UInt16>)e.Argument;

            //Put the arguments into nicely named variables:
            char id = bw_args.Item1;
            double dat1 = bw_args.Item2;
            double dat2 = bw_args.Item3;
            double dat3 = bw_args.Item4;
            UInt16 pack = bw_args.Item5;

            // Check for matlab input till quit or abort
            while (fc.ContinueMatCom())
            {
                // Pause thread
                if (dt_check < 5)
                    Thread.Sleep((int)(5 - dt_check));

                // Store check time
                dt_check = sw_main.ElapsedMilliseconds - t_check;

                // Get current packet
                var _m2c_pack = GetMCOM(msg: "m2c_pack");
                id = (char)(byte)_m2c_pack.GetValue(0, 0);
                dat1 = _m2c_pack.GetValue(0, 1) != -1 ? (double)_m2c_pack.GetValue(0, 1) : Double.NaN;
                dat2 = _m2c_pack.GetValue(0, 2) != -1 ? (double)_m2c_pack.GetValue(0, 2) : Double.NaN;
                dat3 = _m2c_pack.GetValue(0, 3) != -1 ? (double)_m2c_pack.GetValue(0, 3) : Double.NaN;
                pack = (UInt16)_m2c_pack.GetValue(0, 4);
                flag = (double)_m2c_pack.GetValue(0, 5);

                // Check for new data
                if (flag == 1)
                {

                    // Trigger progress change event
                    worker.ReportProgress(0, new System.Tuple<char, double, double, double, UInt16>(id, dat1, dat2, dat3, pack));

                    // Set flag back to zero
                    SendMCOM(msg: "m2c_pack(6) = 0;");

                }
            }

            // end polling
            e.Result = " ";
        }

        // PROGRESSCHANGED FOR bw_MatCOM WORKER
        public static void ProgressChanged_MatCOM(object sender, ProgressChangedEventArgs e)
        {
            // Pull out tuble vals
            Tuple<char, double, double, double, UInt16> bw_args = (Tuple<char, double, double, double, UInt16>)e.UserState;

            // Store id in top level vars
            char id = bw_args.Item1;
            double[] dat = new double[3] { bw_args.Item2, bw_args.Item3, bw_args.Item4 };
            UInt16 pack = bw_args.Item5;

            // Update com info
            m2c.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: sw_main.ElapsedMilliseconds);

            // Handle simulation data
            if (id == 'p' && db.systemTest == 3)
            {
                // Store matlab position data
                UnionHack U = new UnionHack(0, '0', 0, 0, 0);
                UInt64 ts = (UInt64)(dat[0]) + vtStr;
                double x = dat[1];
                double y = dat[2];

                // Run compute pos
                bool pass = CompPos(0, ts, x, y);

                // Convert pos data to double
                double dat1 = 0;
                double dat2 = vtCM[0];
                U.i32 = vtTS[0, 1];
                double dat3 = (double)U.f;

                // Send data
                if (pass)
                    RepeatSendPack_Thread(send_max: 1, id: 'P', dat1: dat1, dat2: dat2, dat3: dat3, do_conf: false);

                // Bail to avoid printing
                return;
            }

            // Store commmon data
            string dat_str = String.Format("id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} dt_rcvd={5}",
                    id, dat[0], dat[1], dat[2], pack, m2c.DT_SentRcvd());

            // Check for repeat packet
            string msg_str;
            if (pack != m2c.packLast[m2c.ID_Ind(id)])
            {
                // Print received data
                msg_str = "   [RCVD] m2c: ";
                LogEvent_Thread(msg_str + dat_str, t: m2c.t_new);
            }
            else
            {
                m2c.cnt_repeat++;
                // Print received data
                msg_str = String.Format("   [*RE-RCVD*] m2c: cnt={0} ", m2c.cnt_repeat);
                LogEvent_Thread(msg_str + dat_str, t: m2c.t_new);
                // Bail without processing
                return;
            }

            // Check for ses saved command
            if (id == 'F')
            {
                fc.isSesSaved = true;
                LogEvent_Thread("[ProgressChanged_MatCOM] ICR_GUI Confirmed Save");
            }

            // Check for rat in command
            else if (id == 'I' && dat[0] == 1)
            {
                fc.isRatIn = true;
                LogEvent_Thread("[ProgressChanged_MatCOM] ICR_GUI Confirmed Rat Taken Into ICR");
            }

            // Check for Recording done command
            else if (id == 'O')
            {
                fc.isRecDone = true;
                LogEvent_Thread("[ProgressChanged_MatCOM] ICR_GUI Confirmed Recording Done and Rat Out");
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
                        LogEvent_Thread("**WARNING** [DoWork_MatCOM] ICR_GUI FORCED QUIT", is_warning: true);
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
                        LogEvent_Thread("!!ERROR!! [DoWork_MatCOM] ICR_GUI FORCED CLOSE", is_error: true);
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

                    // Check if move to command
                    if (id == 'M') // move to
                    {
                        // calculate move to pos
                        dat[0] = CalcMove(dat[0]);
                        do_check_done = true;
                    }
                    // Check if free or cue reward command
                    else if (id == 'R' && (dat[1] == 2 || dat[1] == 3))
                    {
                        // calculate target pos
                        dat[0] = CalcMove(dat[0]);
                    }

                    // Send data
                    RepeatSendPack_Thread(id: id, dat1: dat[0], dat2: dat[1], dat3: dat[2], do_check_done: do_check_done);
                }
            }
        }

        // RUNWORKERCOMPLETED FOR bw_MatCOM WORKER
        public static void RunWorkerCompleted_MatCOM(object sender, RunWorkerCompletedEventArgs e)
        {
            LogEvent_Thread("[RunWorkerCompleted_MatCOM] FINISHED: MatCOM Worker");
        }

        // SEND/STORE DATA FOR MATLAB
        /// <summary>
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="dat_num"></param>
        /// <param name="dat_char"></param>
        public static void SendMCOM_Thread(char id, double dat_num = 0, char dat_char = ' ')
        {
            // Local vars
            UInt16 pack = 0;

            // Itterate packet
            pack = ++c2m.cnt_pack;

            // Set in case we want to check that send finished
            c2m.SetCheckFor(id: id, do_check_sent_rcvd: true);

            // Will check for reply on id we are waiting on
            if (id == 'g')
                m2c.SetCheckFor(id: dat_char, do_check_sent_rcvd: true);

            // Run method on seperate thread
            new Thread(delegate ()
            {
                SendMCOM(id: id, dat_num: dat_num, dat_char: dat_char, pack: pack);
            }).Start();

        }
        public static void SendMCOM_Thread(string msg, bool do_print = true)
        {
            // Run method on seperate thread
            new Thread(delegate ()
            {
                SendMCOM(msg: msg, do_print: do_print);
            }).Start();
        }
        public static void SendMCOM(string msg = " ", char id = ' ', double dat_num = 0, char dat_char = ' ', UInt16 pack = 0, bool do_print = true)
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
                    dat_str = String.Format("id=\'{0}\' dat={1:0.00} pack={2}", id, dat_num, pack);
                }
                else
                {
                    msg = String.Format("[c2m.{0}.dat1, c2m.{0}.pack] =  deal(\'{1}\', {2});", id, dat_char, pack);
                    dat_str = String.Format("id=\'{0}\' dat=\'{1}\' pack={2}", id, dat_char, pack);
                }
            }

            // Check if queue backed up
            if (queue_sendMCOM >= 3)
                LogEvent_Thread(String.Format("**WARNING** [SendMCOM_Thread] c2m Queue Clogged: msg=\"{0}\" queued={1} dt_queue={2}",
                                msg, queue_sendMCOM, sw_main.ElapsedMilliseconds - t_queued), is_warning: true);

            // Add to queue
            lock (lock_queue_sendMCOM) queue_sendMCOM++;
            lock (lock_sendMCOM)
            {

                // Check if queue hanging
                if (sw_main.ElapsedMilliseconds > t_queued + 100)
                    // Log/print error
                    LogEvent_Thread(String.Format("**WARNING** [SendMCOM_Thread] c2m Queue Hanging: msg=\"{0}\" queued={1} dt_queue={2}",
                                    msg, queue_sendMCOM, sw_main.ElapsedMilliseconds - t_queued), is_warning: true);


                if (fc.ContinueMatCom())
                {
                    // Sending packet data
                    if (id != ' ')
                    {
                        // Execute matlab command
                        com_Matlab.Execute(msg);

                        // Update sent
                        double[] dat = new double[3] { dat_num, 0, 0 };
                        c2m.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: sw_main.ElapsedMilliseconds);

                        // Log/print sent
                        LogEvent_Thread(String.Format("   [SENT] c2m: {0} queued={1} dt_queue={2}",
                                dat_str, queue_sendMCOM, sw_main.ElapsedMilliseconds - t_queued));
                    }

                    // Sending simple command
                    else
                    {
                        // Execute matlab command
                        com_Matlab.Execute(msg);

                        // Log/print message
                        if (do_print)
                            LogEvent_Thread(String.Format("   [mCOM] c2m: msg=\"{0}\" queued={1} dt_queue={2}",
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

            // Get value
            if (fc.ContinueMatCom())
                // Store value
                mat_var = com_Matlab.GetVariable(msg, "global");

            // Return value
            return mat_var;
        }

        // WAIT FOR M2C CONFIRMATION
        public static bool WaitForMCOM(char id, string check_for = "rcv", bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_start = sw_main.ElapsedMilliseconds;
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : t_start + timeout;
            bool first_loop = true;
            bool wait_4_rcv = false;
            bool wait_4_send = false;
            bool pass = false;
            string wait_str = " ";

            //// Send resend request
            //if (!check_for == "send")
            //    SendMCOM(id: 'g', dat_char: id);

            // Preset flags
            wait_4_rcv = check_for == "rcv";
            wait_4_send = check_for == "send";

            do
            {

                // Check if received
                wait_4_rcv = wait_4_rcv ? m2c.Wait4SendRcv(id) : wait_4_rcv;

                // Check if sent 
                wait_4_send = wait_4_send ? c2m.Wait4SendRcv(id) : wait_4_send;

                // Check if done
                pass = !wait_4_send && !wait_4_rcv;

                // Bail if nothing to wait for
                if (first_loop && pass)
                    return true;

                // Check if anything to wait for
                if (first_loop)
                    wait_str = check_for == "send" ? "|Send|" : "|Rcv|";

                // Get current status string
                string dat_str = String.Format("Wait for {0}: id=\'{1}\' check_for={2} do_abort={3} timeout={4} wait_4_send={5} wait_4_rcv={6} dt_wait={7}",
                        wait_str, id, check_for, do_abort, timeout == long.MaxValue ? 0 : timeout, wait_4_send, wait_4_rcv, sw_main.ElapsedMilliseconds - t_start);

                // Print what we are waiting on
                if (first_loop)
                {
                    LogEvent_Thread(String.Format("[WaitForMCOM] RUNNING: {0}", dat_str));
                    first_loop = false;
                }

                // Check if sent/rcvd confirmed
                if (pass)
                {
                    LogEvent_Thread(String.Format("[WaitForMCOM] SUCCEEDED: {0}", dat_str));
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
                        LogEvent_Thread(String.Format("**WARNING** [WaitForMCOM] Forced Abort: {0}", dat_str), is_warning: true);
                    else
                    {
                        // Coms failed
                        if (!fc.ContinueMatCom())
                            LogEvent_Thread(String.Format("!!ERROR!! [WaitForMCOM] Lost Comms: {0}", dat_str), is_error: true);

                        // Timedout
                        else if (sw_main.ElapsedMilliseconds > t_timeout)
                            LogEvent_Thread(String.Format("!!ERROR!! [WaitForMCOM] Timedout: {0}", dat_str), is_error: true);

                        // Check if this is first packet
                        if (m2c.packTot == 0)
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
            UnionHack U = new UnionHack(0, '0', 0, 0, 0);
            UInt16 ent = records.swid;
            double x = records.dnextracted_x;
            double y = records.dnextracted_y;
            UInt64 ts = records.qwTimeStamp;

            if (!vtHandler.CheckBlock(ent))
            {
                // Compute position
                bool pass = CompPos(ent, ts, x, y);

                // Send data
                if (pass)
                {
                    // Convert pos data to double
                    double dat1 = ent;
                    double dat2 = vtCM[ent];
                    U.i32 = vtTS[ent, 1];
                    double dat3 = (double)U.f;

                    RepeatSendPack_Thread(send_max: 1, id: 'P', dat1: dat1, dat2: dat2, dat3: dat3, do_conf: false);

                    // Log/print first record received
                    if (!vtHandler.is_streamStarted[ent])
                    {
                        LogEvent_Thread(String.Format("[NetComCallbackVT] FIRST {0} VT RECORD", ent == 0 ? "RAT" : "ROBOT"));
                        vtHandler.is_streamStarted[ent] = true;
                    }
                }

            }
            else if (db.do_printBlockedVt)
            {
                LogEvent_Thread(String.Format("**WARNING** [NetComCallbackVT] VT Blocked: ent={0} cnt={1} dt_send={2}|{3}|{4}",
                    ent, vtHandler.cnt_block[ent], vtHandler.GetSendDT(ent), vtHandler.GetSendDT(ent, "avg"), sw_main.ElapsedMilliseconds - vtHandler.t_sent[ent]), is_warning: true);
            }
        }

        #endregion

        #region ========= MOVEMENT AND TRACKING =========

        // COMPUTE POS IN RAD FOR VT DATA
        public static bool CompPos(UInt16 ent, UInt64 ts, double x, double y)
        {

            // Get first vtTS once
            if (vtStr == 0)
            {
                // Save first ts and bail
                vtStr = ts;
                return false;
            }

            // Convert ts from us to ms and subtract firts record ts
            UInt64 ts_last = vtTS[ent, 1];
            UInt32 ts_now = (UInt32)Math.Round((double)((ts - vtStr) / 1000));

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

            // Check for negative dt
            if (dt < 0)
            {
                LogEvent_Thread(String.Format("**WARNING** [CompPos] Strange TS Values: ent={0} ts_now={1} ts_last={2} dt={3}",
                    ent, ts_now, ts_last, dt), is_warning: true);
            }

            // Convert cart to cm
            x = x * (140 / (RADIUS * 2));
            y = y * (140 / (RADIUS * 2));

            // Convert rad to cm
            double radFlip = Math.Abs(rad_now - (2 * Math.PI)); // flip
            double cm = radFlip * ((140 * Math.PI) / (2 * Math.PI)); // convert

            // Update vars
            // Save old vals
            vtTS[ent, 0] = vtTS[ent, 1];
            vtRad[ent, 0] = rad_last;
            // New vals
            vtTS[ent, 1] = ts_now;
            vtRad[ent, 1] = rad_now;
            vtCM[ent] = cm;

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

        // LOG EVEN STRING
        public static void LogEvent_Thread(string msg_in, bool is_warning = false, bool is_error = false, long t = -1)
        {
            // Print event on seperate thread
            new Thread(delegate ()
            {
                LogEvent(msg_in: msg_in, is_warning: is_warning, is_error: is_error, t: t);
            }).Start();
        }
        public static void LogEvent(string msg_in, bool is_warning = false, bool is_error = false, long t = -1)
        {
            // Local vars
            long t_m = 0;
            float t_s = 0;
            long t_m_sync = 0;
            float t_s_sync = 0;
            string msg_print = " ";
            string msg_log = " ";
            string ts_str = " ";

            // Get time from start of Main()
            t = t > 0 ? t : sw_main.ElapsedMilliseconds;

            lock (lock_printLog)
            {
                // Get sync correction
                t_m = t;
                t_m_sync = t - t_sync;

                // Convert to seconds
                t_s = t_m > 0 ? (float)(t_m) / 1000.0f : 0;
                t_s_sync = t_m_sync > 0 ? (float)(t_m_sync) / 1000.0f : 0;

                // Convert to string
                ts_str = String.Format("[{0:0.000}][{1:0.000}]", t_s_sync, t_s);

                // Pad ts string
                ts_str = ts_str.PadRight(20, ' ');

                // Cat strings
                msg_print = "\n" + ts_str + msg_in + "\n";

                // Print message
                Console.Write(msg_print);

                // Remove cammas from message
                msg_log = msg_in.Replace(",", string.Empty);

                // Store in logger 
                csLogger.UpdateList(msg: msg_log, is_warning: is_warning, is_error: is_error, t: t_m_sync);
            }

            // Store error string
            if (is_error)
                fc.errStr = msg_print;
        }

        #endregion

    }

    #region =============== OTHER CLASSES ===============

    // CLASS TO TRACK PROGRAM FLAGS
    class Flow_Control
    {
        // Private vars
        private static object _lock_printLog = new object();
        private static object _lock_isXbeeBusy = new object();
        private static bool _isRunError = false;
        private static bool _doAbort = false;
        private static bool _isMAThanging = false;
        private static bool _isXbeeBusy = false;
        private static int _cnt_err = 0;
        private string[] _err_list = new string[100];
        // Public vars
        public bool isNlxConnected = false;
        public bool isMatComActive = false;
        public bool isRobComActive = false;
        public bool isArdComActive = false;
        public bool isMovedToStart = false;
        public bool isRatIn = false;
        public bool isRecDone = false;
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
        public bool isXbeeBusy
        {
            set
            {
                lock (_lock_isXbeeBusy)
                    _isXbeeBusy = value;

            }
            get
            {
                lock (_lock_isXbeeBusy)
                    return _isXbeeBusy;
            }
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
            // Pause to let printing finish
            Thread.Sleep(500);

            // Print messeage with error
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


            // Wait for key press
            lock (_lock_printLog)
                Console.ReadKey();
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
        private bool[] _doCheckSendRcv;
        private bool[] _doCheckConf;
        private bool[] _doCheckDone;
        // Public vars
        public char[] id;
        public byte[] head = new byte[1] { 0 };
        public byte[] foot = new byte[1] { 0 };
        public double[][] dat;
        public UInt16[] pack;
        public UInt16[] packLast;
        public UInt16 packTot = 0;
        public UInt16 cnt_pack = 0;
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
            bool _flag_send_rcv,
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
            dat = new double[_id.Length][];
            pack = new UInt16[_id.Length];
            packLast = new UInt16[_id.Length];
            t_sentRcvd = new long[_id.Length];
            _doCheckSendRcv = new bool[_id.Length];
            _doCheckConf = new bool[_id.Length];
            _doCheckDone = new bool[_id.Length];

            // Initialize values to zero
            for (int i = 0; i < _id.Length; i++)
            {
                pack[i] = 0;
                dat[i] = new double[3] { 0, 0, 0 };
                packLast[i] = 0;
                t_sentRcvd[i] = 0;
                _doCheckSendRcv[i] = _flag_send_rcv;
                _doCheckConf[i] = false;
                _doCheckDone[i] = false;
            }
        }

        // Check if command was sent recently
        public void SetCheckFor(char id, bool do_check_sent_rcvd = false, bool do_check_conf = false, bool do_check_done = false)
        {
            // Set sent/received check flag
            lock (_lock_checkSentRcvd)
                _doCheckSendRcv[ID_Ind(id)] = do_check_sent_rcvd;

            // Set received check flag
            lock (_lock_checkConf)
                _doCheckConf[ID_Ind(id)] = do_check_conf;

            // Set done check flag
            lock (_lock_checkDone)
                _doCheckDone[ID_Ind(id)] = do_check_done;
        }

        // Update packet info
        public void ResetCheckFor(char id, bool reset_check_conf = false, bool reset_check_done = false)
        {
            // Reset conf check
            if (reset_check_conf)
                lock (_lock_checkConf)
                    _doCheckConf[ID_Ind(id)] = false;

            // Reset done check
            if (reset_check_done)
                lock (_lock_checkDone)
                    _doCheckDone[ID_Ind(id)] = false;
        }

        // Update packet info
        public void UpdateSentRcvd(char id, double[] dat, UInt16 pack = 0, long t = 0)
        {
            lock (_lock_checkSentRcvd)
            {
                // Get id ind
                int id_ind = ID_Ind(id);

                // Update data
                this.dat[id_ind][0] = dat[0];
                this.dat[id_ind][1] = dat[1];
                this.dat[id_ind][2] = dat[2];

                // Update packet number
                packTot = pack;
                packLast[id_ind] = this.pack[id_ind];
                this.pack[id_ind] = pack;

                // Update check flag
                _doCheckSendRcv[id_ind] = false;

                // Update timers
                t_last = t_new;
                t_new = t;
                t_sentRcvd[id_ind] = t;

                // Reset consecutive dropped packs
                cnt_dropped[0] = 0;
            }
        }

        // Check if command sent or recieved
        public bool Wait4SendRcv(char id)
        {
            lock (_lock_checkSentRcvd)
                return _doCheckSendRcv[ID_Ind(id)];
        }

        // Check if should continue to wait for confirmation command was recieved
        public bool Wait4RcvConf(char id)
        {
            lock (_lock_checkConf)
                return _doCheckConf[ID_Ind(id)];
        }

        // Check if should continue to wait for  command done
        public bool Wait4DoneConf(char id)
        {
            lock (_lock_checkDone)
                return _doCheckDone[ID_Ind(id)];
        }

        // Track dropped packets
        public void AddDropped(int cnt)
        {
            cnt_dropped[0] += cnt;
            cnt_dropped[1] += cnt;
        }

        // Get dt sent or received
        public long DT_SentRcvd(long t = 0)
        {
            if (t == 0)
                return t_new - t_last;
            else
                return t - t_new;
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
        public char PackID(UInt16 pack)
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
        private int _bytesToRcv = 0;
        private int next_milestone = 0;
        private const int _n_updates = 10;
        private int[] _import_update_bytes = new int[_n_updates];
        private long _cnt_warn = 0;
        private long _cnt_err = 0;
        private long[] _warn_line = new long[100];
        private long[] _err_line = new long[100];
        // Public vars
        public bool isImportTimedout = false;
        public string[] prcnt_str = new string[_n_updates + 1];
        public int cnt_logsStored = 0;
        public int[] cnt_dropped = new int[2] { 0, 0 };
        public int bytesRead = 0;
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
                if (cnt_logsStored > 0 &&
                    cnt_dropped[1] == 0 &&
                    (_bytesToRcv == 0 || bytesRead > _bytesToRcv))
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
        public void UpdateList(string msg, bool is_warning = false, bool is_error = false, long t = -1)
        {
            lock (_lock_updateList)
            {
                // Check for repeat
                if (msg != _lastLogStr)
                {
                    // Save log string
                    _lastLogStr = msg;

                    // Itterate count
                    cnt_logsStored++;

                    // Reset consecutive dropped logs
                    cnt_dropped[0] = 0;

                    // Add count and time
                    string str;
                    if (t < 0)
                        // Add count but skip time
                        str = String.Format("{0},{1}", cnt_logsStored, msg);
                    else
                        // Add count and time
                        str = String.Format("{0},{1},{2}", cnt_logsStored, t, msg);

                    // Add to list
                    if (cnt_logsStored - 1 < 50000)
                        _logList[cnt_logsStored - 1] = str;
                    else if (cnt_logsStored - 1 == 50000)
                        _logList[cnt_logsStored - 1] = String.Format("**WARNING** Log Maxed out at {0} entries", 50000);

                    // Store error info
                    if (is_error)
                    {
                        _err_line[_cnt_err++] = cnt_logsStored;
                    }
                    else if (is_warning)
                    {
                        _warn_line[_cnt_warn++] = cnt_logsStored;
                    }
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
        public void UpdateBytesToRcv(double[] dat)
        {
            // Local vars
            string str;

            // Add bytes to UnionHack var
            _bytesToRcv = (int)dat[0];

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

        // Get summary
        public string GetSummary(string get_what)
        {
            // Local vars
            string summary_str = "";

            // Warnings
            if (get_what == "warnings")
            {
                string warn_lines = "ON LINES |";
                for (int i = 0; i < _cnt_warn; i++)
                {
                    warn_lines = String.Format("{0}{1}|", warn_lines, _warn_line[i]);
                }
                summary_str = String.Format("TOTAL WARNINGS: {0} {1}", _cnt_warn, _cnt_warn > 0 ? warn_lines : "");
            }

            // Errors
            else if (get_what == "errors")
            {
                string err_lines = "ON LINES |";
                for (int i = 0; i < _cnt_err; i++)
                {
                    err_lines = String.Format("{0}{1}|", err_lines, _err_line[i]);
                }
                summary_str = String.Format("TOTAL ERRORS: {0} {1}", _cnt_err, _cnt_err > 0 ? err_lines : "");
            }

            // Return string
            return summary_str;
        }

        // Save log data to csv
        public void SaveLog(string log_dir, string log_fi)
        {
            // Write log to file
            string fi_path = @log_dir + @"\" + @log_fi;
            using (System.IO.StreamWriter file_out = new System.IO.StreamWriter(fi_path))
            {
                int count = 0;
                foreach (string line in _logList)
                {
                    if (count++ == cnt_logsStored)
                        break;
                    file_out.WriteLine(line);
                }
            }
            // Set flag
            _isSaved = true;
        }

    }

    // CLASS TO HANDLE VT DATA
    class VT_Handler
    {
        // Private vars
        private static Stopwatch _sw = new Stopwatch();
        private static readonly object _lockBlock = new object();
        private static int _cntThread = 0;
        private static long _t_blockTim = 0;
        private static long _blockFor = 60; // (ms) 
        // Public vars
        public bool[] is_streamStarted = new bool[2] { false, false };
        public long[] t_sent = new long[2] { 0, 0 };
        public long[] t_sent_last = new long[2] { 0, 0 };
        public int[,] dt_hist = new int[2, 10];
        public int[] cnt_sent = new int[2] { 0, 0 };
        public int[] cnt_block = new int[2] { 0, 0 };

        // Constructor
        public VT_Handler(
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
        public bool CheckBlock(UInt16 ent)
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
        public byte b_0; // (byte) 1 byte
        [FieldOffset(1)]
        public byte b_1;
        [FieldOffset(2)]
        public byte b_2;
        [FieldOffset(3)]
        public byte b_3;
        [FieldOffset(0)]
        public char c_0; // (char) 2 byte
        [FieldOffset(2)]
        public char c_1;
        [FieldOffset(0)]
        public UInt16 i16_0; // (UInt16) 2 byte
        [FieldOffset(2)]
        public UInt16 i16_1;
        [FieldOffset(0)]
        public UInt32 i32; // (UInt32) 4 byte
        [FieldOffset(0)]
        public float f; // (float) 4 byt

        // Constructor:
        public UnionHack(byte b, char c, UInt16 i16, UInt32 i32, float f)
        {
            this.b_0 = b;
            this.b_1 = b;
            this.b_2 = b;
            this.b_3 = b;
            this.c_0 = c;
            this.c_1 = c;
            this.i16_0 = i16;
            this.i16_1 = i16;
            this.i32 = i32;
            this.f = f;
        }
    }

    #endregion

}