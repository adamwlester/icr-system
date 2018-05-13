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
                1: Simulated rat test
                2: PID calibration
                3: VT calibration
                4: Halt error test
                5: Wall image IR sync timing
                6: IR sync timing
                7: Robot hardware test
            */
            public double systemTest;

            // Debug matlab
            /*
                [0]: Dont break on errors
                [1]: Break on errors
                [>1]: Break on line
            */
            public int breakDebug;

            // Autoload rat data
            /*
                true: Load rat data based on ICR_GUI hardcoded values
                false: Start normally
            */
            public bool do_autoloadUI;


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
            // Flag if doing any debugging
            public bool is_debugRun;

            // CONSTRUCTOR:
            public DB(
                double system_test,
                int break_debug,
                bool do_autoload_ui,
                bool do_print_blocked_vt,
                bool do_print_sent_rat_vt,
                bool do_print_sent_rob_vt,
                bool do_print_rob_log,
                bool do_print_due_log
                )
            {
                systemTest = system_test;
                breakDebug = break_debug;
                do_autoloadUI = do_autoload_ui;
                do_printBlockedVt = do_print_blocked_vt;
                do_printSentRatVT = do_print_sent_rat_vt;
                do_printSentRobVT = do_print_sent_rob_vt;
                do_printRobLog = do_print_rob_log;
                do_printDueLog = do_print_due_log;
                is_debugRun = system_test != 0 || break_debug > 0 || do_autoload_ui;
            }
        }
        private static DB db = new DB(
            system_test: 0, // 0
            break_debug: 1, // 0
            do_autoload_ui: true, // false
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
        static readonly object lock_m2cPack = new object();
        static readonly object lock_c2mPack = new object();
        static readonly object lock_sendPack = new object();
        static readonly object lock_isConf = new object();
        static readonly object lock_isDone = new object();

        // Initialize vt blocking object
        private static VT_Handler vtHandler = new VT_Handler(stop_watch: sw_main);

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

        // Create logging objects
        private static DB_Logger robLog = new DB_Logger(stop_watch: sw_main);
        private static DB_Logger dueLog = new DB_Logger(stop_watch: sw_main);
        private static DB_Logger csLog = new DB_Logger(stop_watch: sw_main);
        private static UnionHack logBytes = new UnionHack(0, '0', 0, 0, 0);

        // Initialize FC to track program flow
        private static Flow_Control fc = new Flow_Control(cs_log: ref csLog);

        // Define NetCom vars
        private static MNetComClient com_netComClient = new MNetComClient();
        private static string NETCOM_APP_ID = "ICR_Run"; // string displayed in Cheetah when connected
        private static string NETCOM_ACQ_ENT_VT1 = "VT1"; // aquisition entity to stream
        private static string NETCOM_ACQ_ENT_VT2 = "VT2"; // aquisition entity to stream
        private static string NETCOM_ServerIP = "192.168.3.100"; // host computer IP 9"127.0.0.1")

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
            obj_id: "m2c",
            _lock_is_conf: new object(),
            _lock_is_done: new object(),
            _id:
            new char[16]{ // prefix giving masage id
            'i', // setup handshake [NA]
            'p', // simulation data [ts, x, y]
            'G', // session type [1:3]
            'A', // connected to AC computer [NA]
            'N', // netcom setup [NA]
            'F', // data saved [NA]
            'X', // confirm quit
            'C', // confirm close
            'T', // system test command [test]
            'S', // setup task session [ses_cond, task_cond, sound_cond]
            'M', // move to position [targ_pos]
            'R', // run reward [rew_pos, zone_ind, rew_delay]
            'H', // halt movement [halt_state]
            'B', // bulldoze rat [bull_delay, bull_speed]
            'I', // rat in/out [in/out]
            'O'  // confirm task done
             }
            );

        // CS to Matlab
        private static Com_Track c2m = new Com_Track(
            obj_id: "c2m",
            _lock_is_conf: new object(),
            _lock_is_done: new object(),
            _id:
            new char[7] {
            'h', // setup handshake
            'J', // battery voltage
            'Z', // reward zone
            'K', // robot status
            'Y', // task done
            'E', // enable exit
            'C', // confirm close
             },
            _dt_min_sent_rcvd: 100
        );

        // CS to Matlba queue
        private static Com_Track c2m_queue = c2m;

        // CS to Robot
        private static Com_Track c2r = new Com_Track(
            obj_id: "c2r",
            _lock_is_conf: lock_isConf,
            _lock_is_done: lock_isDone,
            _id:
            new char[18] {
            'h', // setup handshake
			't', // hardware ping test
			'T', // system test
			'S', // setup session
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
            'O', // confirm task done
			'U', // log size
            'D', // execution done
			'P', // position data
             },
            _head: (byte)'<',
            _foot: (byte)'>',
            _dt_min_sent_rcvd: 5,
            _dt_resend: 500,
            _resend_max: 5
        );

        // Robot to CS
        private static Com_Track r2c = new Com_Track(
            obj_id: "r2c",
            _lock_is_conf: lock_isConf,
            _lock_is_done: lock_isDone,
            _id:
            new char[18] {
            'h', // setup handshake
			't', // hardware ping test
			'T', // system test command
			'S', // setup session
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
            'O', // confirm task done
			'U', // log size
			'D', // execution done
			'P', // position data
             },
            _head: (byte)'<',
            _foot: (byte)'>',
            _dt_min_sent_rcvd: 5
        );

        // Ard to CS
        private static Com_Track a2c = new Com_Track(
            obj_id: "a2c",
            _lock_is_conf: new object(),
            _lock_is_done: new object(),
            _id:
            new char[1] {
            ' '
            },
            _head: (byte)'<',
            _foot: (byte)'>'
        );

        // Timeouts
        private static long timeoutConnectAC = 15000; // (ms)
        private static long timeoutConnectMatNLX = 60000; // (ms) 
        private static long timeoutImportLog = 10000; // (ms)

        // Position variables
        private static double vt_R;
        private static double vt_XC;
        private static double vt_YC;
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

            // Print debug status
            if (System.Diagnostics.Debugger.IsAttached)
            {
                csLog.Print("RUN MODE = DEBUG");
                db.is_debugRun = true;
            }
            else
            {
                csLog.Print("RUN MODE = RELEASE");
            }

            // SETUP
            csLog.Print("[MAIN] RUNNING: SETUP...");
            bool passed_setup = Setup();
            if (passed_setup)
                csLog.Print("[MAIN] SUCCEEDED: SETUP");
            else
            {
                fc.SetAbort(set_abort_mat: true);
                fc.LogWarning("**WARNING** [MAIN] ABORTED: SETUP");
            }


            // RUN 
            if (passed_setup)
            {
                csLog.Print("[MAIN] RUNNING: RUN...");
                if (fc.doSessionICR)
                {
                    bool passed_run = Run();
                    if (passed_run)
                        csLog.Print("[MAIN] SUCCEEDED: RUN");
                    else
                        fc.LogWarning("**WARNING** [MAIN] ABORTED: RUN");
                }
                else
                {
                    csLog.Print("[MAIN] SKIPPED: RUN");
                }
            }

            // EXIT 
            csLog.Print("[MAIN] RUNNING: EXIT...");
            Exit();
            csLog.Print("[MAIN] FINISHED: EXIT");

            // Pause before final exit
            Thread.Sleep(1000);

        }

        // SETUP
        public static bool Setup()
        {
            // LOCAL VARS
            bool pass;
            char id = ' ';
            double[] dat = new double[3] { 0, 0, 0 };
            UInt16 pack = 0;

            // START TIMER
            sw_main.Start();

            // SET BUFFER SIZE
            Console.BufferHeight = Int16.MaxValue - 1;

            // INITIALIZE MATLAB GLOBAL VARS

            csLog.Print("[Setup] RUNNNING: Create mCOM Global Variables...");
            System.Array m2c_pack = new double[6] { 0, 0, 0, 0, 0, 0 };
            com_Matlab.PutWorkspaceData("m2c_pack", "global", m2c_pack);
            com_Matlab.PutWorkspaceData("m2c_dir", "global", " ");
            csLog.Print("[Setup] FINISHED: Create mCOM Global Variables");

            // SETUP/START ICR_GUI BACKROUND WORKER

            csLog.Print("[Setup] RUNNING: Start RunGUI Worker...");
            bw_RunGUI.DoWork += DoWork_RunGUI;
            bw_RunGUI.RunWorkerCompleted += RunWorkerCompleted_RunGUI;
            // Start ICR_GUI worker
            bw_RunGUI.RunWorkerAsync();
            csLog.Print("[Setup] FINISHED: Start RunGUI Worker");

            // SETUP/RUN MATLAB COM BACKROUND WORKER

            csLog.Print("[Setup] START: Start MatCOM Worker...");
            bw_MatCOM.DoWork += DoWork_MatCOM;
            bw_MatCOM.ProgressChanged += ProgressChanged_MatCOM;
            bw_MatCOM.RunWorkerCompleted += RunWorkerCompleted_MatCOM;
            bw_MatCOM.WorkerReportsProgress = true;
            // Set com flag
            fc.isMatComActive = true;
            // Start running com worker
            var bw_args = Tuple.Create(id, dat[0], dat[1], dat[2], pack);
            bw_MatCOM.RunWorkerAsync(bw_args);
            csLog.Print("[Setup] FINISHED: Start MatCOM Worker...");

            // SETUP DEBUGGING

            if (db.is_debugRun)
            {
                csLog.Print("[Setup] RUNNING: Setup Debugging...");

                // Show matlab app window

                com_Matlab.Visible = 1;

                // Check for break line input
                if (db.is_debugRun)
                {
                    // Keep checking on seperate thread
                    new Thread(delegate ()
                    {
                        CheckBreakInput();
                    }).Start();
                    csLog.Print("[Setup] START: CheckBreakInput()");
                }

                csLog.Print("[Setup] FINISHED: Setup Debugging");

                // Log/print db settings
                csLog.Print(String.Format("[Setup] RUNNING IN DEBUG MODE: systemTest={0} breakDebug={1} do_autoloadUI={2}",
                    db.systemTest, db.breakDebug, db.do_autoloadUI));
            }
            else
            {
                // Hide matlab app window
                com_Matlab.Visible = 0;
            }

            // WAIT FOR SESSION TYPE INFO FROM MATLAB
            csLog.Print("[Setup] RUNNING: WAIT FOR...: ICR_GUI to Load...");
            pass = WaitForMatCom(id: 'G', chk_rcv: true, do_abort: true);
            if (pass)
            {
                csLog.Print("[Setup] SUCCEEDED: WAIT FOR: ICR_GUI to Load");

                // Store session type parameters
                fc.doSessionICR = m2c.datMat[m2c.ID_Ind('G')][0] == 1;
                fc.doSessionTurnTT = m2c.datMat[m2c.ID_Ind('G')][1] == 1;
                fc.doSessionUpdateTable = m2c.datMat[m2c.ID_Ind('G')][2] == 1;

                // Print session type
                csLog.Print(String.Format("[Setup] SESSION TYPE: \"{0}{1}{2}\"",
                    fc.doSessionICR ? "ICR Session" : "", fc.doSessionTurnTT ? "TT Turne" : "", fc.doSessionUpdateTable ? "Update Table" : ""));
            }
            else
            {
                fc.LogWarning("**WARINING**  [Setup] ABORTED: WAIT FOR: ICR_GUI to Load");
                fc.isMatComActive = false;
                return false;
            }

            // WAIT FOR MATLAB TO CONNECT TO AC COMPUTER

            csLog.Print("[Setup] RUNNING: WAIT FOR...: AC Connect...");
            pass = WaitForMatCom(id: 'A', chk_rcv: true, do_abort: true, timeout: timeoutConnectAC);
            if (pass)
                csLog.Print("[Setup] SUCCEEDED: WAIT FOR: AC Connect");
            else
            {
                // Program timed out because matlab was hanging on connect
                if (!fc.doAbortCS)
                {
                    fc.LogWarning("**WARINING**  [Setup] ABORTED: WAIT FOR: AC Connect");
                }
                else
                {
                    fc.LogError("!!ERROR!! [Setup] ABORTED: WAIT FOR: AC Connect");
                }
                fc.SetAbort(set_abort_cs: true, is_mat_failed: true);
                return false;
            }

            // WAIT FOR INITIAL MATLAB HANDSHAKE REQUEST

            csLog.Print("[Setup] RUNNING: WAIT FOR...: ICR_GUI Handshake Request...");
            pass = WaitForMatCom(id: 'i', chk_rcv: true, do_abort: true, timeout: 15000);
            if (pass)
            {
                csLog.Print("[Setup] SUCCEEDED: WAIT FOR: ICR_GUI Handshake Request");
            }
            else
            {
                fc.LogWarning("**WARINING** [Setup] ABORTED: WAIT FOR: ICR_GUI Handshake Request");
                return false;
            }

            // HANDLE NON-ICR SESSION

            if (!fc.doSessionICR)
            {

                // Send back handshake
                csLog.Print("[Setup] RUNNING: Send Handshake From CS");
                SendMatCom_Thread(id: 'h', dat1: 1);

                // Wait for send and store time
                csLog.Print("[Setup] RUNNING: WAIT FOR...: ICR_GUI Handshake to be Sent...");
                pass = WaitForMatCom(id: 'h', chk_send: true, do_abort: true);
                if (pass)
                {
                    csLog.Print("[Setup] SUCCEEDED: WAIT FOR: ICR_GUI Handshake to be Sent");
                }
                else
                {
                    fc.LogWarning("**WARINING** [Setup] ABORTED: WAIT FOR: ICR_GUI Handshake to be Sent");
                    return false;
                }

                // Wait for send and store time
                t_sync = c2m.t_sentRcvd[c2m.ID_Ind('h')];
                csLog.t_sync = t_sync;

                // Bail
                csLog.Print("[Setup] FINISHED: Send Handshake From CS");
                return true;
            }

            // SETUP/START CHEETAHDUE SERIAL

            csLog.Print("[Setup] RUNNING: Setup CheetahDue Serial Coms and Logging...");
            sp_cheetahDue.ReadTimeout = 100;
            sp_cheetahDue.BaudRate = 57600;
            sp_cheetahDue.PortName = "COM5";
            // Set com flag
            fc.isArdComActive = true;
            // Open serial port connection
            sp_cheetahDue.Open();
            // Start getting new data on seperate thread
            new Thread(delegate ()
            {
                ParseArdCom();
            }).Start();
            csLog.Print("[Setup] FINISHED: Setup CheetahDue Serial Coms and Logging");

            // SETUP/START XBEE SERIAL

            csLog.Print("[Setup] RUNNING: Setup Xbee Serial Coms");
            sp_Xbee.ReadTimeout = 100;
            sp_Xbee.BaudRate = 57600;
            sp_Xbee.PortName = "COM4";
            // Set com flag
            fc.isRobComActive = true;
            // Set byte threshold to max packet size
            sp_Xbee.ReceivedBytesThreshold = 1;
            // Open serial port connection
            sp_Xbee.Open();
            // Spin up parser thread
            new Thread(delegate ()
            {
                ParseRobCom();
            }).Start();
            csLog.Print("[Setup] FINISHED: Setup Xbee Serial Coms");

            // SEND CHEETAHDUE HANDSHAKE
            csLog.Print("[Setup] RUNNING: WAIT FOR...: Robot Handshake...");
            byte[] out_byte = new byte[1] { (byte)'i' };
            sp_cheetahDue.Write(out_byte, 0, 1);

            // WAIT FOR ROBOT HANDSHAKE
            pass = WaitForRobCom(id: 'h', chk_rcv: true, do_abort: true, timeout: 5000);
            if (pass)
            {
                // Store sync time based on send time
                t_sync = r2c.t_sentRcvd[r2c.ID_Ind('h')];
                csLog.t_sync = t_sync;

                // Log/print sync time
                csLog.Print(String.Format("SET SYNC TIME: {0}ms", t_sync), t: t_sync);

                // Log/print success
                csLog.Print("[Setup] SUCCEEDED: Robot Handshake");

            }
            else
            {
                fc.LogWarning("**WARINING** [Setup] ABORTED: Robot Handshake");
                fc.isRobComActive = false;
                fc.isArdComActive = false;
                return false;
            }

            // SEND TEST SETUP INFO TO ROBOT

            csLog.Print("[Setup] RUNNING: Test Setup...");
            RepeatSendRobCom_Thread(id: 'T', dat1: db.systemTest, dat2: 0, dat3: 0);
            // Wait for recieve confirmation
            pass = WaitForRobCom(id: 'T', chk_send: true, chk_conf: true, timeout: 5000);
            if (pass)
            {
                csLog.Print("[Setup] SUCCEEDED: Test Setup");
            }
            else
            {
                fc.LogError("!!ERROR!! [Setup] ABORTED: Test Setup");
                return false;
            }

            // RUN PING TIME TEST

            csLog.Print("[Setup] RUNNING: Hardware Test...");

            // Wait for x pings
            pass = WaitForRobCom(id: 't', dat1: r2c.datMat[r2c.ID_Ind('h')][0] + 1, chk_rcv: true, do_abort: true, timeout: 20000);
            if (pass)
            {
                // Log/print success
                csLog.Print("[Setup] FINISHED: Hardware Test");

                // Log/print average ping time
                double dt_ping_r2c = r2c.datMat[r2c.ID_Ind('t')][1];
                double dt_ping_r2a = r2c.datMat[r2c.ID_Ind('t')][2];
                csLog.Print(String.Format("PING TIMES: r2c={0:0.00}ms r2a={1:0.00}ms", dt_ping_r2c, dt_ping_r2a));
            }
            else
            {
                fc.LogWarning("**WARINING** [Setup] ABORTED: Hardware Test");
                return false;
            }

            // SEND ROBOT SETUP CONFIRMATION TO MATLAB 

            csLog.Print("[Setup] RUNNING: Send Robot Setup Confirmation");
            SendMatCom_Thread(id: 'K', dat1: 1);

            // WAIT FOR MATLAB NETCOM SETUP CONFIRMATION
            csLog.Print("[Setup] RUNNING: WAIT FOR...: ICR_GUI NLX Setup...");
            pass = WaitForMatCom(id: 'N', chk_rcv: true, do_abort: true, timeout: timeoutConnectMatNLX);
            if (pass)
            {
                csLog.Print("[Setup] SUCCEEDED: WAIT FOR: ICR_GUI NLX Setup");


                // Store vt pixel parameters
                vt_R = m2c.datMat[m2c.ID_Ind('N')][0];
                vt_XC = m2c.datMat[m2c.ID_Ind('N')][1];
                vt_YC = m2c.datMat[m2c.ID_Ind('N')][2];

                // Print values
                csLog.Print(String.Format("[Setup] RECIEVED VT FOV INFO: vt_R={0:0.00} vt_XC={1:0.00} vt_YC={2:0.00}", vt_R, vt_XC, vt_YC));
            }
            else
            {
                fc.LogWarning("**WARINING** [Setup] ABORTED: WAIT FOR: ICR_GUI NLX Setup");
                return false;
            }

            // SETUP NETCOM

            // Initilize deligate for VT callback
            deligate_netComCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            com_netComClient.SetCallbackFunctionVT(deligate_netComCallback, new ICR_Run());

            // Connect to NetCom
            csLog.Print("[Setup] RUNNING: Connect to NLX...");
            fc.isNlxConnected = com_netComClient.AreWeConnected();
            while (!fc.isNlxConnected && !fc.doAbortCS)
            {
                fc.isNlxConnected = com_netComClient.ConnectToServer(NETCOM_ServerIP);
            }
            if (fc.isNlxConnected)
                csLog.Print("[Setup] SUCCEEDED: Connect to NLX");
            else
            {
                fc.LogWarning("**WARINING** [Setup] FAILED: Connect to NLX");
                return false;
            }

            // START NETCOM STREAMING

            csLog.Print("[Setup] STARTING: Nlx Stream");

            // Send App name
            com_netComClient.SetApplicationName(NETCOM_APP_ID);

            // Start robot vt streaming
            csLog.Print("[Setup] RUNNING: Open VT2 Stream...");
            while (!fc.isRobStreaming && !fc.doAbortCS)
                fc.isRobStreaming = com_netComClient.OpenStream(NETCOM_ACQ_ENT_VT2);
            if (fc.isRobStreaming)
                csLog.Print("[Setup] FINISHED: Open VT2 Stream");
            else
            {
                fc.LogWarning("**WARINING** [Setup] ABORTED: Open VT2 Stream");
                return false;
            }

            // SEND STREAM STATUS REQUEST TO ROBOT

            csLog.Print("[Setup] RUNNING: Confirm Robot Streaming...");
            RepeatSendRobCom_Thread(id: 'V', do_check_done: true);
            // Wait for confirmation from robot
            pass = WaitForRobCom(id: 'V', chk_send: true, chk_conf: true, chk_done: true, do_abort: true);
            if (pass)
            {
                // Send confirm robot streaming to MATLAB
                SendMatCom_Thread(id: 'K', dat1: 2);
                csLog.Print("[Setup] SUCCEEDED: Confirm Robot Streaming");
            }
            else
            {
                fc.LogWarning("**WARINING** [Setup] ABORTED: Confirm Robot Streaming");
                return false;
            }

            // WAIT FOR SESSION SETUP CONFIRMATION

            csLog.Print("[Setup] RUNNING: WAIT FOR...: Setup Parameters from MATLAB...");
            pass = WaitForMatCom(id: 'S', dat1: 2, chk_rcv: true, do_abort: true);
            if (pass)
            {
                pass = WaitForRobCom(id: 'S', dat1: 2, chk_send: true, chk_conf: true, do_abort: true);
                if (pass)
                    csLog.Print("[Setup] SUCCEEDED: WAIT FOR: Setup Parameters");
                else
                {
                    fc.LogWarning("**WARINING** [Setup] ABORTED: WAIT FOR: Setup Parameters");
                    return false;
                }
            }
            else
            {
                fc.LogWarning("**WARINING** [Setup] ABORTED: WAIT FOR: Setup Parameters");
                return false;
            }

            // RETURN SETUP SUCCESS

            return true;

        }

        // RUN
        public static bool Run()
        {
            // LOCAL VARS
            bool run_pass = true;
            bool pass;

            // WAIT FOR COMPLETION OF INITIAL ROBOT MOVE

            csLog.Print("[Run] RUNNING: WAIT FOR...: MoveTo Start Command from MATLAB...");
            // Wait for matlab
            pass = WaitForMatCom(id: 'M', chk_rcv: true, do_abort: true);
            // Wait for move done
            if (pass)
            {
                csLog.Print("[Run] RUNNING: WAIT FOR...: MoveTo Start...");
                pass = WaitForRobCom(id: 'M', chk_send: true, chk_conf: true, chk_done: true, do_abort: true);
                if (pass)
                {
                    // Send confirm robot first move done to Matlbab
                    SendMatCom_Thread(id: 'K', dat1: 3);
                    fc.isMovedToStart = true;
                    csLog.Print("[Run] SUCCEEDED: WAIT FOR: MoveTo Start");
                }
                else
                {
                    // Bail if first move fails
                    fc.LogError("!!ERROR!! [Run] ABORTED: WAIT FOR: MoveTo Start");
                    return false;
                }
            }
            else
            {
                fc.LogWarning("**WARINING** [Run] ABORTED: WAIT FOR: MoveTo Start Command from MATLAB");
                return false;
            }

            // WAIT FOR RAT IN CONFIRMATION

            csLog.Print("[Run] RUNNING: WAIT FOR...: Rat In...");
            pass = WaitForMatCom(id: 'I', chk_rcv: true, do_abort: true);
            if (pass)
            {
                csLog.Print("[Run] FINISHED: WAIT FOR: Rat In");
                // Stream rat vt if not forage task or simulation test
                if (db.systemTest != 1 && fc.isRatOnTrack)
                {
                    csLog.Print("[Run] RUNNING: Open VT1 Stream...");
                    while (!fc.isRatStreaming && !fc.doAbortCS)
                        fc.isRatStreaming = com_netComClient.OpenStream(NETCOM_ACQ_ENT_VT1);
                    if (fc.isRatStreaming)
                        csLog.Print("[Setup] FINISHED: Open VT1 Stream");
                    else
                    {
                        fc.LogWarning("**WARINING** [Setup] ABORTED: Open VT1 Stream");
                        return false;
                    }
                }
                else
                    csLog.Print("[Run] SKIPPED: Open VT1 Stream");
            }
            else
            {
                // Bail if rat in check fails
                fc.LogWarning("**WARNING** [Run] ABORTED: WAIT FOR: Rat In");
                run_pass = false;
            }

            // HOLD HERE TILL TASK COMPLETE

            csLog.Print("[Run] RUNNING: Main Session Loop...");
            // Stay in loop till task is done or error
            while (
                com_netComClient.AreWeConnected() &&
                fc.isRatInArena &&
                !fc.isGUIclosed &&
                !fc.isTaskDone &&
                !fc.doAbortCS
                ) ;
            if (!fc.doAbortCS)
                csLog.Print("[Run] SUCCEEDED: Main Session Loop");
            else
            {
                if (com_netComClient.AreWeConnected())
                    fc.LogWarning("**WARNING** [Run] ABORTED: Main Session Loop");
                else
                {
                    fc.LogError("!!ERROR!! [Run] FAILED: Main Session Loop: NLX Disconnected");
                    fc.SetAbort(set_abort_mat: true);
                }
                run_pass = false;
            }

            // WAIT FOR TASK DONE CONFIRMATION FROM MATLAB

            if (!fc.isTaskDone && !fc.isGUIclosed)
            {
                csLog.Print("[Run] RUNNING: WAIT FOR...: Task Finished...");
                pass = WaitForMatCom(id: 'O', chk_rcv: true, timeout: !fc.isAbortRun ? 30000 : 10000);
                if (pass)
                {
                    // Wait for all the other crap to be relayed from Matlab
                    Thread.Sleep(1000);
                    csLog.Print("[Run] SUCCEEDED: WAIT FOR: Task Finished");
                }
                else
                {
                    fc.LogError("!!ERROR!! [Run] ABORTED: WAIT FOR: Task Finished");
                    run_pass = false;
                }
            }
            else
                fc.LogWarning("**WARNING** [Run] SKIPPED: WAIT FOR: Task Finished");

            // WAIT FOR TASK DONE ROBOT CONFIRMATION

            // Wait for task done recieve confirmation
            if (fc.isTaskDone)
            {
                csLog.Print("[Run] RUNNING: WAIT FOR...: Confirmation Task Finished Received by Robot...");
                pass = WaitForRobCom(id: 'O', chk_send: true, chk_conf: true, timeout: 5000);
                if (pass)
                    csLog.Print("[Run] SUCCEEDED: WAIT FOR: Confirmation Task Finished Received by Robot");
                else
                {
                    fc.LogError("!!ERROR!! [Run] ABORTED: WAIT FOR: Confirmation Task Finished Received by Robot");
                    run_pass = false;
                }
            }
            else
                fc.LogWarning("**WARNING** [Run] SKIPPED: WAIT FOR: Confirmation Task Finished Received by Robot");

            // SEND/WAIT FOR FINAL MOVE TO COMPLETE

            if (fc.isMovedToStart)
            {
                csLog.Print("[Run] RUNNING: MoveTo South...");
                double move_to = CalcMove(4.7124 - feedDist);

                // Clear check state from previous 'M' message
                c2r.GetMsgState(id: 'M', get_sent_rcvd: true, get_conf: true, get_done: true);

                // Send move command on seperate thread and wait for done reply
                RepeatSendRobCom_Thread(id: 'M', dat1: 0, dat2: move_to, do_check_done: true);

                // Wait for confirmation from robot
                pass = WaitForRobCom(id: 'M', dat1: 0, chk_send: true, chk_conf: true, chk_done: true, timeout: 10000);
                if (pass)
                {
                    csLog.Print("[Run] SUCCEEDED: MoveTo South");
                }
                else
                {
                    fc.LogWarning("**WARNING** [Run] ABORTED: MoveTo South");
                    run_pass = false;
                }
            }
            else
                fc.LogWarning("**WARNING** [Run] SKIPPED:  MoveTo South");

            // WAIT FOR FINAL MOVEMENT TO PLOT

            // Wait also cause fucking 'Done' Check still does not work
            Thread.Sleep(1000);

            // SEND TASK DONE CONFIRMATION TO MATLAB

            if (fc.isTaskDone)
            {
                // Send task done confirmation to Matlab
                SendMatCom_Thread(id: 'Y', dat1: 1);

                // Wait for send and store time
                csLog.Print("[Setup] RUNNING: WAIT FOR...: Send ICR_GUI Confirmation Task Finished...");
                pass = WaitForMatCom(id: 'h', chk_send: true, do_abort: true);
                if (pass)
                    csLog.Print("[Setup] SUCCEEDED: WAIT FOR: Send ICR_GUI Confirmation Task Finished");
                else
                    fc.LogWarning("**WARINING** [Setup] ABORTED: WAIT ICR_GUI: Send ICR Confirmation Task Finished");
            }
            else
                fc.LogWarning("**WARNING** [Run] SKIPPED: Send ICR_GUI Confirmation Task Finished");


            // RETURN RUN STATUS

            return run_pass;
        }

        // EXIT
        public static void Exit()
        {
            // LOCAL VARS
            bool pass;

            // SEND ROBOT LOG REQUEST

            if (!fc.doSessionICR || !sp_Xbee.IsOpen)
            {
                csLog.Print("[Exit] SKIPPED: Request Robot Log");
            }
            else
            {
                csLog.Print("[Exit] RUNNING: Request Robot Log...");
                RepeatSendRobCom_Thread(id: 'L', dat1: 0);
                pass = WaitForRobCom(id: 'L', chk_send: true, chk_conf: true, timeout: 5000);
                if (pass)
                {
                    // Wait for bytes to receive messages to be received
                    csLog.Print("[Exit] RUNNING: WAIT FOR...: Robot Log Bytes...");
                    pass = WaitForRobCom(id: 'U', chk_rcv: true, timeout: 5000);
                    if (pass)
                    {
                        // Store data byte
                        robLog.UpdateBytesToRcv(r2c.datMat[r2c.ID_Ind('U')]);
                        csLog.Print(String.Format("[Exit] SUCCEEDED: WAIT FOR: Robot Log Bytes: bytes_expected={0}", robLog.bytesToRcv));

                        // Flag logging started and block ParseRobCom()
                        robLog.isLogging = true;

                        // Start importing
                        Thread get_log_thread = new Thread(delegate ()
                        {
                            GetRobotLog();
                        });
                        get_log_thread.Priority = ThreadPriority.Highest;
                        get_log_thread.Start();

                        // Tell robot to begin streaming log and wait for message to send
                        RepeatSendRobCom_Thread(id: 'L', dat1: 1, do_conf: false);
                        pass = WaitForRobCom(id: 'L', chk_send: true, timeout: 5000);
                        csLog.Print("[Exit] SUCCEEDED: Request Robot Log");
                    }
                    else
                    {
                        fc.LogError(String.Format("!!ERROR!! [Exit] ABORTED: WAIT FOR: Robot Log Bytes: bytes_expected={0}", robLog.bytesToRcv));
                    }
                }
                else
                {
                    fc.LogError("!!ERROR!! [Exit] FAILED: Request Robot Log");
                }
            }

            // SHUT DOWN NETCOM

            if (!fc.doSessionICR || !IsProcessOpen("Cheetah"))
            {
                csLog.Print("[Exit] SKIPPED: NetCom Disconnect");
            }
            else
            {
                csLog.Print("[Exit] RUNNING: NetCom Disconnect...");

                //// Stop recording aquisition if GUI closed early
                if (fc.isGUIclosed)
                {
                    fc.LogWarning("**WARNING** [Exit] STOPPING NLX RECORDING AND ACQUISITION");
                    string reply = " ";
                    com_netComClient.SendCommand("-StopRecording", ref reply);
                    com_netComClient.SendCommand("-StopAcquisition", ref reply);
                }

                // Close NetCom sreams
                com_netComClient.CloseStream(NETCOM_ACQ_ENT_VT1);
                com_netComClient.CloseStream(NETCOM_ACQ_ENT_VT2);

                // Disconnect from NetCom
                do { com_netComClient.DisconnectFromServer(); }
                while (com_netComClient.AreWeConnected() && !fc.doAbortCS);

                // Check if disconnect succesful
                if (!com_netComClient.AreWeConnected())
                {
                    fc.isNlxConnected = false;
                    csLog.Print("[Exit] SUCCEEDED: NetCom Disconnect");
                }
                else
                {
                    fc.LogError("!!ERROR!! [Exit] FAILED: NetCom Disconnect");
                }
            }

            // WAIT FOR ROBOT LOG SAVE TO COMPLETE

            if (!fc.doSessionICR || !sp_Xbee.IsOpen)
            {
                csLog.Print("[Exit] SKIPPED: WAIT FOR Robot Log Save");
            }
            else
            {
                csLog.Print("[Exit] RUNNING: WAIT FOR...: Robot Log Save...");
                while (!robLog.isFinished && !robLog.isImportTimedout)
                    Thread.Sleep(10);

                // Check if complete log was imported
                if (robLog.isLogComplete)
                    csLog.Print(String.Format("[Exit] SUCCEEDED: WAIT FOR: Robot Log Save: logged={0} dropped={1} b_read={2} bytes_expected={3} dt_run={4}",
                        robLog.cnt_logsStored, robLog.cnt_dropped[1], robLog.bytesRead, robLog.bytesToRcv, robLog.logDT));
                else if (robLog.cnt_logsStored > 0)
                    fc.LogWarning(String.Format("**WARNING** [Exit] PARTIALLY SUCCEEDED: WAIT FOR: Robot Log Save: logged={0} dropped={1} b_read={2} bytes_expected={3} dt_run={4}",
                        robLog.cnt_logsStored, robLog.cnt_dropped[1], robLog.bytesRead, robLog.bytesToRcv, robLog.logDT));
                else
                {
                    fc.LogError(String.Format("!!ERROR!! [Exit] FAILED: WAIT FOR: Robot Log Save: logged={0} dropped={1} b_read={2} bytes_expected={3} dt_run={4}",
                        robLog.cnt_logsStored, robLog.cnt_dropped[1], robLog.bytesRead, robLog.bytesToRcv, robLog.logDT));
                }
            }

            // SEND/WAIT FOR ROBOT QUIT COMMAND

            if (!fc.doSessionICR || !sp_Xbee.IsOpen)
            {
                csLog.Print("[Exit] SKIPPED: Confirm Robot Quit");
            }
            else
            {
                RepeatSendRobCom_Thread(id: 'Q', do_check_done: true);

                // Wait for quit confirmation from robot for fixed period of time
                csLog.Print("[Exit] RUN: Confirm Robot Quit...");
                pass = WaitForRobCom(id: 'Q', chk_send: true, chk_conf: true, chk_done: true, timeout: 15000);
                if (pass)
                    csLog.Print("[Exit] SUCCEEDED: Confirm Robot Quit");
                else
                    fc.LogError("!!ERROR!! [Exit] FAILED: Confirm Robot Quit");
            }

            // Set flags
            fc.isRobComActive = false;
            fc.isArdComActive = false;

            // SAVE CHEETAH DUE LOG FILE
            if (!fc.doSessionICR || !sp_Xbee.IsOpen)
            {
                csLog.Print("[Exit] SKIPPED: Save CheetahDue Log");
            }
            else
            {
                csLog.Print("[Exit] RUNNING: Save CheetahDue Log...");
                dueLog.SaveLog(logDir, dueLogFi);
                if (dueLog.isLogComplete)
                    csLog.Print(String.Format("[Exit] SUCCEEDED: Save CheetahDue Log: logged={0} dropped={1}",
                        dueLog.cnt_logsStored, dueLog.cnt_dropped[1]));
                else if (dueLog.cnt_logsStored > 0)
                    fc.LogWarning(String.Format("**WARNING** [Exit] PARTIALLY SUCCEEDED: Save CheetahDue Log: logged={0} dropped={1}",
                        dueLog.cnt_logsStored, dueLog.cnt_dropped[1]));
                else
                {
                    fc.LogError(String.Format("!!ERROR!! [Exit] FAILED: Save CheetahDue Log: logged={0} dropped={1}",
                        dueLog.cnt_logsStored, dueLog.cnt_dropped[1]));
                }
            }

            // WAIT FOR MATLAB TO SAVE

            if (fc.isGUIquit || fc.isGUIclosed)
            {
                csLog.Print("[Exit] SKIPPED: WAIT FOR...: ICR_GUI to Save");
            }
            else
            {
                csLog.Print("[Exit] RUNNING: WAIT FOR...: ICR_GUI to Save...");
                pass = WaitForMatCom(id: 'F', chk_rcv: true, do_abort: true);
                if (pass)
                {
                    csLog.Print("[Exit] SUCCEEDED: WAIT FOR: ICR_GUI to Save");

                    // Get NLX dir
                    lock (lock_m2cPack)
                    {
                        dynamic nlx_rec_dir = com_Matlab.GetVariable("m2c_dir", "global");
                        nlxRecDir = (string)nlx_rec_dir;
                    }

                    // Confirm log saved
                    csLog.Print(String.Format("SET RECORDING DIR TO \"{0}\"", nlxRecDir));

                }
                else if (fc.isGUIquit)
                    fc.LogWarning("**WARNING** [Exit] ABORTED: WAIT FOR: ICR_GUI to Save");
                else
                {
                    fc.LogError("!!ERROR!! [Exit] ABORTED: WAIT FOR: ICR_GUI to Save");
                }
            }

            // WAIT FOR MATLAB QUIT COMMAND

            csLog.Print("[Exit] RUNNING: WAIT FOR...: ICR_GUI Quit command...");
            if (!fc.isGUIquit && !fc.isGUIclosed)
                pass = WaitForMatCom(id: 'X', chk_rcv: true, do_abort: true);
            else
                pass = fc.isGUIquit;
            if (pass)
                csLog.Print("[Exit] SUCCEEDED: WAIT FOR: ICR_GUI Quit command");
            else
                fc.LogError("!!ERROR!! [Exit] ABORTED: WAIT FOR: ICR_GUI Quit command");

            // SEND COMMAND FOR MATLAB TO EXIT

            SendMatCom_Thread(id: 'E', dat1: 1);
            csLog.Print("[Exit] FINISHED: Tell ICR_GUI to Close");

            // Wait for GUI to close

            csLog.Print("[Exit] RUNNING: Confirm ICR_GUI Closed...");
            pass = WaitForMatCom(id: 'C', chk_rcv: true, timeout: fc.isMatComActive && !(fc.isMatFailed || db.is_debugRun) ? 30000 : 10000);
            if (pass)
                csLog.Print("[Exit] SUCCEEDED: Confirm ICR_GUI Closed");
            else
            {
                fc.LogError("!!ERROR!! [Exit] ABORTED: Confirm ICR_GUI Closed");

                // Set abort all
                fc.SetAbort(set_abort_cs: true, set_abort_mat: true, is_mat_failed: true);
            }

            // SEND CLOSE CONFIRM TO MATLAB
            csLog.Print("[Exit] RUNNING: Send ICR_GUI Close Confirmation Received...");
            SendMatCom_Thread(id: 'C', dat1: 1);
            pass = WaitForMatCom(id: 'C', chk_send: true, timeout: fc.isMatComActive && !fc.isMatFailed ? 10000 : 50000);
            if (pass)
                csLog.Print("[Exit] SUCCEEDED: Send ICR_GUI Close Confirmation Received");
            else
            {
                fc.LogError("!!ERROR!! [Exit] ABORTED: Send ICR_GUI Close Confirmation Received");
                // Set error flag
                fc.SetAbort(set_abort_mat: true);
            }

            // Set exit flag to exit all threads
            fc.doExit = true;

            // Wait for Matlab and threads to close down
            Thread.Sleep(1000);

            // CLOSE DOWN BACKROUND WORKERS

            //bw_RunGUI.CancelAsync();
            bw_RunGUI.Dispose();
            csLog.Print("[Exit] FINISHED: Dispose RunGUI Worker");
            //bw_MatCOM.CancelAsync();
            bw_MatCOM.Dispose();
            csLog.Print("[Exit] FINISHED: Dispose MatCOM Worker");

            // CLEAR ALL MATLAB VARS
            string msg = "clearvars -global; close all;";
            if (!fc.isMatFailed)
            {
                csLog.Print(String.Format("[Exit] RUNNING: Clear MatCom Globals: msg=\"{0}\"", msg));
                com_Matlab.Execute(msg);
            }
            else
            {
                fc.LogWarning(String.Format("**WARNING** [Exit] ABORTED: Clear MatCom Globals: msg=\"{0}\"", msg));
            }

            // HOLD FOR DEBUGGING OR ERRRORS

            if (db.is_debugRun || fc.isAbortRun || fc.isErrorRun)
            {
                // Show Matlab window
                if (!fc.isMatFailed)
                    com_Matlab.Visible = 1;

                // Pause to let printing finish
                Thread.Sleep(1000);

                // Print messeage with error
                Console.WriteLine("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                if (fc.isErrorRun)
                {
                    Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PAUSED FOR ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

                    // Print all errors
                    for (int i = 0; i < csLog.cnt_err; i++)
                    {
                        Console.WriteLine(csLog.err_list[i]);
                    }
                }
                else if (fc.isAbortRun)
                {
                    Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PAUSED FOR ABORT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                }
                else
                {
                    Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PAUSED FOR DEBUGGING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                }

                Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PRESS ANY KEY TO EXIT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                Console.WriteLine("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");

                // Pause to let printing finish
                Thread.Sleep(1000);

                // Wait for key press
                Console.ReadKey();
            }

            // QUIT MATCOM

            if (!fc.isMatFailed)
            {
                Thread.Sleep(100);
                com_Matlab.Quit();
                csLog.Print("[Exit] FINISHED: Close MatCOM");
            }

            // KILL THAT MOTHER FUCKER!

            else
            {
                KillMatlab();
                fc.LogWarning("**WARNING** [Exit] HAD TO KILL MATLAB");
            }

            // LOG/PRINT RUN SUMMARY

            // Error and warning summary
            csLog.Print(msg_in: csLog.GetErrorSummary("warnings"));
            csLog.Print(msg_in: csLog.GetErrorSummary("errors"));
            
            // Com summary info
            csLog.Print(msg_in: String.Format(
                "COM SUMMARY: c2r=|p_ind={0}|p_tot={1}|resnd={2}| r2c=|p_ind={3}|p_tot={4}|rercv={5}|drops={6}| " +
                "c2m=|p_ind={7}|p_tot={8}|resnd={9}|  m2c=|p_ind={10}|p_tot={11}|rercv={12}|",
                c2r.packInd, c2r.packTot, c2r.cnt_repeat, r2c.packInd - UInt16.MaxValue / 2, r2c.packTot, r2c.cnt_repeat, r2c.cnt_dropped,
                c2m.packInd, c2m.packTot, c2m.cnt_repeat, m2c.packInd, m2c.packTot, m2c.cnt_repeat));

            // SAVE CS LOG FILE

            csLog.Print("[Exit] RUNNING: Save CS Log...");
            csLog.SaveLog(logDir, csLogFi);
            csLog.Print(String.Format("[Exit] FINISHED: Save CS Log: logged={0}", csLog.cnt_logsStored));

            // COPY LOG FILES TO CURRENT LOG DIRECTORY

            if (nlxRecDir != logDir)
            {
                // Create copies
                if (fc.doSessionICR)
                {
                    System.IO.File.Copy(System.IO.Path.Combine(logDir, robLogFi), System.IO.Path.Combine(nlxRecDir, robLogFi), true);
                    System.IO.File.Copy(System.IO.Path.Combine(logDir, dueLogFi), System.IO.Path.Combine(nlxRecDir, dueLogFi), true);
                }
                System.IO.File.Copy(System.IO.Path.Combine(logDir, csLogFi), System.IO.Path.Combine(nlxRecDir, csLogFi), true);
                System.IO.File.Copy(System.IO.Path.Combine(logDir, matLogFi), System.IO.Path.Combine(nlxRecDir, matLogFi), true);

                // Confirm logs copied saved
                csLog.Print(String.Format("COPPIED LOG FILES TO \"{0}\"", nlxRecDir));
            }

        }

        #endregion

        #region ========= SERIAL COMMUNICATION ==========

        // SEND PACK DATA REPEATEDLY TILL RECIEVED CONFIRMED
        public static void RepeatSendRobCom_Thread(int send_max = 0, char id = ' ', double dat1 = double.NaN, double dat2 = double.NaN, double dat3 = double.NaN, UInt16 pack = 0, bool do_conf = true, bool do_check_done = false)
        {
            // Local vars
            double[] dat = new double[3] { dat1, dat2, dat3 };

            // Run method on seperate thread
            new Thread(delegate ()
            {
                RepeatSendRobCom(send_max: send_max, id: id, dat: dat, pack: pack, do_conf: do_conf, do_check_done: do_check_done);
            }).Start();

        }
        public static void RepeatSendRobCom(int send_max, char id, double[] dat, UInt16 pack, bool do_conf, bool do_check_done)
        {
            long t_resend = sw_main.ElapsedMilliseconds + c2r.dt_resend;
            int send_count = 1;

            // Specify max send attempts
            send_max = send_max == 0 ? c2r.resendMax : send_max;

            // Send new data with new packet number
            pack = SendRobCom(id: id, dat: dat, pack: pack, do_conf: do_conf, do_check_done: do_check_done);

            // Bail if not checking for confirmation
            if (!do_conf)
                return;

            // Keep checking mesage was received
            else
            {
                while (fc.ContinueRobCom())
                {

                    // Confirmation recieved
                    if (r2c.GetMsgState(id: id, get_sent_rcvd: true))
                    {
                        // Bail
                        return;
                    }

                    // Need to resend
                    else if (sw_main.ElapsedMilliseconds > t_resend)
                    {
                        // Log/print
                        fc.LogWarning_Thread(String.Format("**WARNING** [RepeatSendRobCom_Thread] Resending c2r: cnt={0} id=\'{1}\' dat=|{2:0.00}|{3:0.00}|{4:0.00}| pack={5} do_conf={6} do_check_done={7}",
                            send_count, id, dat[0], dat[1], dat[2], pack, do_conf, do_check_done));

                        // Resend
                        SendRobCom(id: id, dat: dat, pack: pack, do_conf: do_conf, do_check_done: do_check_done);
                        t_resend = sw_main.ElapsedMilliseconds + c2r.dt_resend;
                        send_count++;
                    }

                    // Check if coms have failed
                    else if (send_count >= c2r.resendMax)
                    {
                        // Log/print
                        fc.LogError_Thread(String.Format("!!ERROR!! [RepeatSendRobCom_Thread] ABBORTED: Resending c2r: cnt={0} id=\'{1}\' dat=|{2:0.00}|{3:0.00}|{4:0.00}| pack={5} do_conf={6} do_check_done={7}",
                            send_count, id, dat[0], dat[1], dat[2], pack, do_conf, do_check_done));

                        // Set error flags
                        fc.SetAbort(set_abort_mat: true);
                        fc.isRobComActive = false;

                        // Bail
                        return;
                    }
                }
            }
        }

        // SEND PACK DATA
        public static UInt16 SendRobCom(char id, double[] dat, UInt16 pack, bool do_conf, bool do_check_done)
        {
            /* 
            SEND DATA TO ROBOT 
            FORMAT: [0]head, [1]id, [2:5]dat1, [6:9]dat2, [10:13]dat2, [14:15]pack, [16]do_conf, [17]footer
            EXAMPLE: ASCII {'<','L','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','\0','>','c'} DEC {60,76,1,255,255,0,60}
            */

            // Track when data queued
            long t_queued = sw_main.ElapsedMilliseconds;

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
                bool do_loop = true;
                bool is_min_dt_send_rcv = false;
                bool is_buff_ready = false;
                bool is_hanging = false;
                string dat_str = "";
                string buff_str = "";
                long dt_rcvd = 0;

                // Wait for next safe send time
                while (do_loop)
                {
                    // Delay send time till x ms after last send or rcvd
                    is_min_dt_send_rcv =
                        c2r.DT_SentRcvd(t: sw_main.ElapsedMilliseconds) < c2r.dt_minSentRcvd &&
                        r2c.DT_SentRcvd(t: sw_main.ElapsedMilliseconds) < r2c.dt_minSentRcvd;

                    // Make sure outbut and input buffer have enough space
                    is_buff_ready = sp_Xbee.BytesToWrite == 0 && sp_Xbee.BytesToRead == 0;

                    // Check if loop should continue
                    do_loop =
                        (is_min_dt_send_rcv || !is_buff_ready) &&
                        fc.ContinueRobCom();

                    // Get status
                    is_hanging = sw_main.ElapsedMilliseconds > t_queued + 100;

                    // Abort if sending pos data
                    if (is_hanging && id == 'P')
                        break;

                }

                // Get new packet number
                if (pack == 0)
                {
                    c2r.packInd++;
                    pack = c2r.packInd;
                }

                // Format data string
                dat_str = String.Format("id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} do_conf={5} do_check_done={6} ",
                    id, dat[0], dat[1], dat[2], pack, do_conf, do_check_done);

                // Check if queue backed up or hanging
                if (is_hanging)
                {
                    // Get status info
                    dt_rcvd = r2c.DT_SentRcvd(sw_main.ElapsedMilliseconds);
                    buff_str = String.Format("tx={0} rx={1} dt_q={2} dt_snd={3} dt_rcv={4}",
                   sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, sw_main.ElapsedMilliseconds - t_queued, c2r.DT_SentRcvd(), dt_rcvd);

                    // Log/print
                    fc.LogWarning_Thread(String.Format("**WARNING** [SendRobCom] c2r Queue HANGING: {0}", dat_str + buff_str));

                    // Bail if this is pos data
                    if (id == 'P')
                    {
                        c2r.packInd--;
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

                // Store do_conf 
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
                sp_Xbee.Write(msgByteArr, 0, msgByteArr.Length);

                // Update c2r info
                c2r.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: sw_main.ElapsedMilliseconds);
                c2r.SetMsgState(id: id, set_sent_rcvd: true);

                // Store final status info
                dt_rcvd = r2c.DT_SentRcvd(sw_main.ElapsedMilliseconds);
                buff_str = String.Format("bytes_sent={0} tx={1} rx={2} dt_q={3} dt_snd={4} dt_rcv={5}",
                    msgByteArr.Length, sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, sw_main.ElapsedMilliseconds - t_queued, c2r.DT_SentRcvd(), dt_rcvd);

                // Check if sending pos data
                if (id != 'P')
                {
                    // Check for resend
                    if (pack != c2r.packLastArr[c2r.ID_Ind(id)])
                        csLog.Print_Thread("   [SENT] c2r: " + dat_str + buff_str, t: c2r.t_new);
                    else
                    {
                        c2r.cnt_repeat++;
                        csLog.Print_Thread(String.Format("   [*RE-SENT*] c2r: cnt={0} ", c2r.cnt_repeat) + dat_str + buff_str, t: c2r.t_new);
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
                        csLog.Print_Thread("   [SENT] c2r: " + dat_str + buff_str + dt_vt, t: c2r.t_new);
                    }
                }

                // Unlock vt sending
                vtHandler.Unblock(id);

            }

            // Return packet number
            return pack;

        }

        // WAIT FOR R2C CONFIRMATION
        public static bool WaitForRobCom(char id, double dat1 = double.NaN, bool chk_send = false, bool chk_rcv = false, bool chk_conf = false, bool chk_done = false, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_start = sw_main.ElapsedMilliseconds;
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : t_start + timeout;
            bool first_loop = true;
            bool pass = false;
            string wait_str = " ";

            // Create wait list strings
            string[] wait_list =
                new string[5] { "Send", "Rcv", "Conf", "Done", String.Format("Dat={0:0.00}", dat1) };

            // Track check flags
            bool chk_dat = !Double.IsNaN(dat1);
            bool[] check_4 = new bool[5] { chk_send, chk_rcv, chk_conf, chk_done, chk_dat };

            // Track wait flags
            bool[] wait_4_now = new bool[check_4.Length];
            check_4.CopyTo(wait_4_now, 0);
            bool[] wait_4_last = new bool[check_4.Length];
            check_4.CopyTo(wait_4_last, 0);

            // Wait for confirmation
            while (true)
            {

                // SENT
                if (chk_send)
                {
                    bool wait_other = false;
                    wait_4_now[0] = wait_4_now[0] ? wait_other || !c2r.GetMsgState(id: id, get_sent_rcvd: true) : wait_4_now[0];
                }

                // RECIEVED
                if (chk_rcv)
                {
                    bool wait_other = wait_4_now[0];
                    wait_4_now[1] = wait_4_now[1] ? wait_other || !r2c.GetMsgState(id: id, get_sent_rcvd: true) : wait_4_now[1];
                }

                // CONFIRMED
                if (chk_conf)
                {
                    bool wait_other = wait_4_now[0];
                    wait_4_now[2] = wait_4_now[2] ? wait_other || !c2r.GetMsgState(id: id, get_conf: true) : wait_4_now[2];
                }

                // DONE
                if (chk_done)
                {
                    bool wait_other = wait_4_now[0] || wait_4_now[1] || wait_4_now[2];
                    wait_4_now[3] = wait_4_now[3] ? wait_other || !c2r.GetMsgState(id: id, get_done: true) : wait_4_now[3];
                }

                // DATA
                if (chk_dat)
                {
                    bool wait_other = wait_4_now[0] || wait_4_now[1] || wait_4_now[2] || wait_4_now[3];
                    // Check for r2c dat match
                    if (chk_rcv)
                        wait_4_now[4] = wait_4_now[4] ? wait_other || r2c.datMat[r2c.ID_Ind(id)][0] != dat1 : wait_4_now[4];
                    // Check cor c2r dat dat match
                    else
                        wait_4_now[4] = wait_4_now[4] ? wait_other || c2r.datMat[r2c.ID_Ind(id)][0] != dat1 : wait_4_now[4];

                }

                // Format check for string
                if (first_loop)
                    wait_str = String.Format("|{0}{1}{2}{3}{4}",
                    wait_4_now[0] ? wait_list[0] + "|" : "",
                    wait_4_now[1] ? wait_list[1] + "|" : "",
                    wait_4_now[2] ? wait_list[2] + "|" : "",
                    wait_4_now[3] ? wait_list[3] + "|" : "",
                    wait_4_now[4] ? wait_list[4] + "|" : "");

                // Get current status string
                long t_out = timeout == long.MaxValue ? 0 : timeout;
                long dt_wait = sw_main.ElapsedMilliseconds - t_start;
                string dat_str = String.Format("id=\'{0}\' chk_flags=|s={1}|r={2}|c={3}|n={4}|d={5}| wait_flags=|s={6}|r={7}|c={8}|n={9}|d={10}| do_abort={11} timeout={12} dt_wait={13}",
                         id, check_4[0] ? "T" : "F", check_4[1] ? "T" : "F", check_4[2] ? "T" : "F", check_4[3] ? "T" : "F", check_4[4] ? "T" : "F",
                         wait_4_now[0] ? "T" : "F", wait_4_now[1] ? "T" : "F", wait_4_now[2] ? "T" : "F", wait_4_now[3] ? "T" : "F", wait_4_now[4] ? "T" : "F",
                         do_abort, t_out, dt_wait);

                // Print what we are waiting on
                if (first_loop)
                {
                    csLog.Print(String.Format("   [WaitForRobCom] RUNNING: WAIT FOR...: {0}...: {1}", wait_str, dat_str));
                    first_loop = false;
                }

                // Check for changes
                for (int i = 0; i < wait_list.Length; i++)
                {
                    if (wait_4_now[i] != wait_4_last[i])
                    {
                        string str = wait_list[i];
                        csLog.Print(String.Format("   [WaitForRobCom] CONFIRMED: \'{0}\' \"{1}\"", id, str));
                    }
                    wait_4_last[i] = wait_4_now[i];
                }

                // Check if all conditions confirmed
                pass = !wait_4_now[0] && !wait_4_now[1] && !wait_4_now[2] && !wait_4_now[3] && !wait_4_now[4];

                // Return success
                if (pass)
                {
                    csLog.Print(String.Format("   [WaitForRobCom] SUCCEEDED: WAIT FOR: {0}: {1}", wait_str, dat_str));
                    return true;
                }

                // Check if need to abort
                else if (
                    (do_abort && fc.doAbortCS) ||
                    !fc.ContinueRobCom() ||
                    sw_main.ElapsedMilliseconds > t_timeout
                    )
                {

                    // External forced abort
                    if (do_abort && fc.doAbortCS)
                        fc.LogWarning_Thread(String.Format("**WARNING** [WaitForRobCom] ABORTED: WAIT FOR: FORCED ABORT: WAIT FOR: {0}: {1}", wait_str, dat_str));
                    else
                    {
                        // Coms failed
                        if (!fc.ContinueRobCom())
                            fc.LogError_Thread(String.Format("!!ERROR!! [WaitForRobCom] ABORTED: WAIT FOR: LOST COMS: WAIT FOR: {0}: {1}", wait_str, dat_str));

                        // Timedout
                        else if (sw_main.ElapsedMilliseconds > t_timeout)
                            fc.LogError_Thread(String.Format("!!ERROR!! [WaitForRobCom] ABORTED: WAIT FOR: TIMEDOUT: WAIT FOR: {0}: {1}", wait_str, dat_str));

                        // Set error flag
                        fc.SetAbort(set_abort_mat: true);
                    }

                    // Set abort flag and bail  
                    return false;
                }

                // Pause thread
                else

                    Thread.Sleep(10);

            }
        }

        // PARSE RECIEVED XBEE DATA 
        public static void ParseRobCom()
        {
            /* 
            RECIEVE DATA FROM FEEDERDUE 
            FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
            */

            // Dump anything in buffers on first run
            sp_Xbee.DiscardInBuffer();
            sp_Xbee.DiscardOutBuffer();

            // Loop till all data read out
            while (!fc.doExit)
            {

                // Bail if no new data or processing robot log
                if (sp_Xbee.BytesToRead < 1 ||
                    robLog.isLogging)
                {
                    Thread.Sleep(1);
                    continue;
                }

                // Local vars
                UnionHack U = new UnionHack(0, '0', 0, 0, 0);
                bool r2c_head_found = false;
                bool r2c_id_found = false;
                bool r2c_foot_found = false;
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

                // Store parse start time
                r2c.t_parse_str = sw_main.ElapsedMilliseconds;

                // Get header
                if (XbeeBuffReady(1, r2c.t_parse_str, "head"))
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

                }

                // Find id and check message is intended for CS
                if (r2c_head_found)
                {

                    if (XbeeBuffReady(1, r2c.t_parse_str, "id"))
                    {

                        sp_Xbee.Read(id_bytes, 0, 1);
                        bytes_read += 1;
                        // Get id
                        U.b_0 = id_bytes[0];
                        U.b_1 = 0;
                        id = U.c_0;

                        // Check for r2c id 
                        for (int i = 0; i < r2c.idArr.Length; i++)
                        {
                            if (id == r2c.idArr[i])
                            {
                                r2c_id_found = true;
                                break;
                            }
                        }

                    }
                }

                // Get data, packet number and do_conf flag
                if (r2c_head_found && r2c_id_found)
                {

                    // Get data
                    for (int i = 0; i < 3; i++)
                    {
                        if (XbeeBuffReady(4, r2c.t_parse_str, String.Format("dat{0}", i + 1)))
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
                    if (XbeeBuffReady(2, r2c.t_parse_str, "pack"))
                    {

                        // Read in data
                        sp_Xbee.Read(pack_bytes, 0, 2);
                        bytes_read += 2;
                        U.b_0 = pack_bytes[0];
                        U.b_1 = pack_bytes[1];
                        pack = U.i16_0;
                    }

                    // Get do confirm byte
                    if (XbeeBuffReady(1, r2c.t_parse_str, "do_comf"))
                    {
                        sp_Xbee.Read(conf_bytes, 0, 1);
                        bytes_read += 1;
                        // Get bool
                        do_conf = conf_bytes[0] == 1 ? true : false;
                    }

                    // Find footer
                    if (XbeeBuffReady(1, r2c.t_parse_str, "foot"))
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

                    }
                }

                // Format data string
                long dt_rcvd = r2c.DT_SentRcvd(sw_main.ElapsedMilliseconds);
                long dt_parse = sw_main.ElapsedMilliseconds - r2c.t_parse_str;
                string dat_str = String.Format("id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} do_conf={5} b_read={6} rx={7} tx={8} dt_prs={9} dt_snd={10} dt_rcv={11}",
                        id, dat[0], dat[1], dat[2], pack, do_conf, bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, dt_parse, c2r.DT_SentRcvd(), dt_rcvd);

                // Store r2c packet data
                if (r2c_foot_found)
                {
                    // Update message info
                    r2c.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: r2c.t_parse_str);
                    r2c.SetMsgState(id: id, set_sent_rcvd: true);
                    c2r.SetMsgState(id: id, set_conf: true);

                    // Check for repeat packet
                    string msg_str;
                    if (pack != r2c.packLastArr[r2c.ID_Ind(id)])
                    {
                        msg_str = "   [RCVD] r2c: ";
                    }
                    else
                    {
                        r2c.cnt_repeat++;
                        msg_str = String.Format("   [*RE-RCVD*] r2c: cnt={0} ", r2c.cnt_repeat);
                    }

                    // Log/print rcvd details
                    csLog.Print_Thread(msg_str + dat_str, t: r2c.t_new);

                    // Check if this is a done confirmation
                    if (id == 'D')
                        c2r.SetMsgState(pack: pack, set_done: true);

                    // Send recieve confirmation
                    if (do_conf)
                        RepeatSendRobCom_Thread(send_max: 1, id: id, dat1: dat[0], dat2: dat[1], dat3: dat[2], pack: pack, do_conf: false);

                    // Check if data should be relayed to Matlab
                    if (c2m.ID_Ind(id) != -1)
                        SendMatCom_Thread(id: id, dat1: dat[0]);

                }

                // If all data found restart loop
                if (r2c_head_found && r2c_id_found && r2c_foot_found)
                {
                    // Change com status
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
                    r2c.AddDropped(1);

                    // Get from info
                    char from = r2c_head_found ? 'c' : '?';

                    // Get found flags info
                    string found = String.Format("|{0}|{1}|{2}|",
                        r2c_head_found ? "r2c_head" : "no_head",
                        r2c_id_found ? "r2c_id" : "no_id",
                        r2c_foot_found ? "r2c_foot" : "no_foot");

                    // Log/print available info
                    fc.LogWarning_Thread(String.Format("**WARNING** [ParseRobCom] Dropped r2{0} Packs: dropped={1}|{2} found={3} head={4} id=\'{5}\' dat=|{6:0.00}|{7:0.00}|{8:0.00}| pack={9} do_conf={10} foot={11} b_read={12} rx={13} tx={14} dt_prs={15}",
                        from, r2c.cnt_dropped[0], r2c.cnt_dropped[1], found, head, id, dat[0], dat[1], dat[2], pack, do_conf, foot, bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, sw_main.ElapsedMilliseconds - r2c.t_parse_str));

                    // Dump buffer if > 1 consecutive drops and no bytes read
                    if (r2c.cnt_dropped[0] > 1)
                    {
                        fc.LogWarning_Thread("**WARNING** [ParseRobCom] Dumping r2c Input Buffer");
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
            dat_str = String.Format("get=\"{0}\" min_byte={1} dt_chk={2} dt_q={3} rx={4} tx={5}",
                     getting, min_byte, sw_main.ElapsedMilliseconds - t_str, sw_main.ElapsedMilliseconds - t_parse_str, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite);

            // Check if hanging
            if (sw_main.ElapsedMilliseconds > t_str + 25)
            {
                fc.LogWarning_Thread("**WARNING** [XbeeBuffReady] XBee Read HANGING: " + dat_str);
            }

            // Check if timedout
            if (sw_main.ElapsedMilliseconds > t_timeout)
            {
                fc.LogWarning_Thread("**WARNING** [XbeeBuffReady] TIMEDOUT: " + dat_str);
            }

            // Check if buff filled
            pass = sp_Xbee.BytesToRead >= min_byte ? true : false;

            return pass;
        }

        // CONTINUALLY CHECK FOR NEW CHEETAHDUE LOG DATA
        public static void ParseArdCom()
        {

            // Dump anything in buffers on first run
            sp_cheetahDue.DiscardInBuffer();
            sp_cheetahDue.DiscardOutBuffer();

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
                    // Keep waiting
                    if (!ArdBuffReady(1, fc.ContinueArdCom() ? 500 : 100, do_print: false))
                    {
                        continue;
                    }

                    // Read in data
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
                    if (ArdBuffReady(1, fc.ContinueArdCom() ? 500 : 100))
                    {
                        // Read in data
                        sp_cheetahDue.Read(chksum_bytes, 0, 1);
                        bytes_read += 1;
                        chksum = chksum_bytes[0];
                    }

                    // Get complete message
                    byte[] log_bytes = new byte[chksum];
                    if (ArdBuffReady(chksum, fc.ContinueArdCom() ? 1000 : 100))
                    {
                        // Read in all data
                        sp_cheetahDue.Read(log_bytes, 0, chksum);
                        bytes_read += chksum;

                    }
                    // Convert to string
                    log_str = System.Text.Encoding.UTF8.GetString(log_bytes);

                    // Find footer
                    if (ArdBuffReady(1, fc.ContinueArdCom() ? 500 : 100))
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
                    dueLog.UpdateList(log_str);

                    // print data received
                    if (db.do_printDueLog)
                    {
                        // Update list
                        dueLog.UpdateList(log_str);

                        // Print
                        if (db.do_printDueLog)
                            csLog.Print_Thread(String.Format("   [LOG] a2c[{0}]: message=\"{1}\" chksum={2} b_read={3} rx={4} tx={5}",
                                dueLog.cnt_logsStored, log_str, chksum, bytes_read, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));
                    }

                    // Change com status
                    if (!fc.isArdComActive)
                    {
                        fc.isArdComActive = true;
                    }
                }

                // Dump incomplete packets
                else if (bytes_read > 0)
                {
                    // Add to count
                    dueLog.AddDropped(1);

                    // Print
                    fc.LogWarning_Thread(String.Format("**WARNING** [ParseArdCom] Dropped a2c Log: logged={0} dropped={1}|{2} head={3} message=\"{4}\" chksum={5} foot={6} b_read={7} rx={8} tx={9}",
                       dueLog.cnt_logsStored, dueLog.cnt_dropped[0], dueLog.cnt_dropped[1], head, log_str, chksum, foot, bytes_read, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));

                    // Dump buffer if > 1 consecutive drops and no bytes read
                    if (dueLog.cnt_dropped[0] > 1 && bytes_read == 0)
                    {
                        fc.LogWarning_Thread("**WARNING** [ParseArdCom] Dumping a2c Input Buffer");
                        sp_cheetahDue.DiscardInBuffer();
                    }
                }

            }
        }

        // WAIT FOR CHEETAHDUE BUFFER TO FILL
        public static bool ArdBuffReady(int min_byte, long timeout, bool do_print = true)
        {
            // Local vars
            long t_timeout = sw_main.ElapsedMilliseconds + timeout;
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_cheetahDue.BytesToRead < min_byte &&
                sw_main.ElapsedMilliseconds <= t_timeout
                ) ;

            // Check if buff filled
            pass = sp_cheetahDue.BytesToRead >= min_byte ? true : false;

            // Check for errors
            if (!pass && do_print)
            {
                // Timedout
                if (sw_main.ElapsedMilliseconds > t_timeout)
                    fc.LogWarning_Thread(String.Format("**WARNING** [ArdBuffReady] a2c HANGING: dt_chk={0} rx={1} tx={2}",
                      (sw_main.ElapsedMilliseconds - t_timeout) + timeout, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));
                else
                    fc.LogWarning_Thread(String.Format("**WARNING** [ArdBuffReady] ABORTED: dt_chk={0} rx={1} tx={2}",
                   (sw_main.ElapsedMilliseconds - t_timeout) + timeout, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));
            }

            return pass;
        }

        // RETRIEVE FEEDERDUE LOG
        public static void GetRobotLog()
        {
            // Local vars
            long t_stream_str = sw_main.ElapsedMilliseconds;
            long t_stream_timeout = t_stream_str + timeoutImportLog;
            long dt_read_timeout = 10000;
            long t_read_last = t_stream_str;
            char[] c_arr = new char[3] { '\0', '\0', '\0' };

            // Start log import
            csLog.Print_Thread("[GetRobotLog] RUNNING: Robot Log Import...");

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

                // Check for run timeout
                if (sw_main.ElapsedMilliseconds > t_stream_timeout)
                {
                    // Check for dt read timeout
                    if (sw_main.ElapsedMilliseconds - t_read_last > dt_read_timeout)
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
                string status_str = robLog.GetImportStatus(read_ind);
                if (status_str != " ")
                {
                    // Print progress on seperate thread
                    csLog.Print_Thread(String.Format("[GetRobotLog] Log Import {0}", status_str));
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
                        csLog.Print_Thread(String.Format("[GetRobotLog] Log Import {0}",
                            robLog.prcnt_str[robLog.prcnt_str.Length - 1]));

                        // Print success termination string received
                        csLog.Print_Thread(String.Format("[GetRobotLog] Received Send Success Termination String: \"{0}{1}{2}\" rx={3} tx={4}",
                            c_arr[0], c_arr[1], c_arr[2], sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite));
                        send_complete = true;

                        // Break out of loop
                        break;
                    }
                    else if (c_arr[2] == '!')
                    {
                        // Print abort termination string received
                        fc.LogWarning_Thread(String.Format("**WARNING** [GetRobotLog] Received Abort Termination String: \"{0}{1}{2}\" rx={3} tx={4}",
                            c_arr[0], c_arr[1], c_arr[2], sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite));
                        is_robot_abort = true;

                        // Break out of loop
                        break;
                    }
                }
            }

            // Dump whatever is left in buffer
            sp_Xbee.DiscardInBuffer();

            // Reset logging flag
            robLog.isLogging = false;

            // Store bytes read
            robLog.bytesRead = read_ind;

            // Store data string
            string dat_str = String.Format("b_read={0} bytes_expected=~{1} rx={2} tx={3} dt_stream={4}",
                    robLog.bytesRead, robLog.bytesToRcv, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, robLog.logDT);

            // Check if logging timed out
            if (!send_complete)
            {
                // Bail if no bytes read
                if (robLog.bytesRead == 0)
                {
                    fc.LogError_Thread(String.Format("!!ERROR!! [GetRobotLog] FAILED: Robot Log Import: {0}: dt_read_last={1} dt_run={2} {3}",
                        is_timedout ? "Read Timedout" : is_robot_abort ? "Robot Aborted" : "Reason Unknown", dt_read, dt_run, dat_str));
                    fc.SetAbort(set_abort_mat: true);
                    robLog.isImportTimedout = true;

                    // Bail
                    return;
                }
                else
                {
                    fc.LogWarning_Thread(String.Format("**WARNING** [GetRobotLog] ABORTED: Robot Log Import: {0}: dt_read_last={1} dt_run={2} {3}",
                        is_timedout ? "Read Timedout" : is_robot_abort ? "Robot Aborted" : "Reason Unknown", dt_read, dt_run, dat_str));
                }
            }
            else
            {
                // Finished log import
                csLog.Print_Thread(String.Format("[GetRobotLog] SUCCEEDED: Robot Log Import: {0}", dat_str));
            }

            // Start log store
            csLog.Print_Thread("[GetRobotLog] RUNNING: Robot Log Store...");

            // Parse string and store logs
            char[] out_arr = new char[1000];
            int write_ind = 0;
            char c = '\0';
            bool do_rec_store = false;
            char[] int_str = new char[5] { '\0', '\0', '\0', '\0', '\0' };
            int int_cnt = 0;
            int rec_now = 0;
            int rec_last = 0;
            for (int i = 0; i < robLog.bytesRead; i++)
            {

                // Get next char
                c = in_arr[i];

                // Check for count head
                if (!do_rec_store && write_ind == 0 && c == '[')
                {
                    do_rec_store = true;
                }

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
                        fc.LogWarning_Thread(String.Format("**WARNING** [GetRobotLog] Failed to Parse r2c Log Number: rec_last={0}",
                                rec_now));
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
                        int cnt_dropped = rec_now - rec_last - 1;
                        robLog.AddDropped(cnt_dropped);
                        fc.LogWarning_Thread(String.Format("**WARNING** [GetRobotLog] Dropped r2c Log: expected={0} stored={1} dropped={2}|{3}",
                            rec_now, robLog.cnt_logsStored, robLog.cnt_dropped[0], robLog.cnt_dropped[1]));
                    }

                    // Update list
                    robLog.UpdateList(new_log);

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
            csLog.Print_Thread(String.Format("[GetRobotLog] FINISHED: Robot Log Store: logs_stored={0} dt_run={1}",
                robLog.cnt_logsStored, robLog.logDT));

            // Save robot log file
            csLog.Print_Thread("[GetRobotLog] RUNNING: Robot Log Save...");
            robLog.SaveLog(logDir, robLogFi);
            csLog.Print_Thread("[GetRobotLog] FINISHED: Robot Log Save");
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
            csLog.Print_Thread("[DoWork_RunGUI] RUNNING: Setup Matlab paths...");
            SendMatCom(msg: @"addpath(genpath('" + matStartDir + "'));");
            csLog.Print_Thread("[DoWork_RunGUI] FINISHED: Setup Matlab paths");

            // Run startup.m
            csLog.Print_Thread("[DoWork_RunGUI] RUNNING: startup.m...");
            com_Matlab.Feval("startup", 0, out startup_result);
            csLog.Print_Thread("[DoWork_RunGUI] FINISHED: startup.m...");

            // Run ICR_GUI.m
            csLog.Print_Thread("[DoWork_RunGUI] RUNNING: ICR_GUI.m...");
            com_Matlab.Feval("ICR_GUI", 1, out icr_gui_result, db.systemTest, db.breakDebug, db.do_autoloadUI);

            // Get status
            object[] res = icr_gui_result as object[];
            status = res[0] as string;
            if (status == null)
                status = " ";

            // Print status
            if (status == "succeeded")
                csLog.Print_Thread("[DoWork_RunGUI] SUCCEEDED: ICR_GUI.m");
            else if (status != " ")
            {
                fc.LogError_Thread(String.Format("!!ERROR!! [DoWork_RunGUI] FAILED: ICR_GUI.m Error: {0}", status));
                fc.SetAbort(set_abort_mat: true);
            }

            // Pass on results
            e.Result = status;
        }

        // RUNWORKERCOMPLETED FOR bw_RunGUI WORKER
        public static void RunWorkerCompleted_RunGUI(object sender, RunWorkerCompletedEventArgs e)
        {
            // Local vars
            string status = " ";

            // Set flag that GUI has closed
            fc.isGUIclosed = true;

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
                csLog.Print_Thread("[RunWorkerCompleted_RunGUI] SUCCEEDED: RunGUI Worker");

            // Run failed
            else
            {
                // Run failed but errors were caught
                if (status != " ")
                    fc.LogWarning_Thread("**WARNING** [RunWorkerCompleted_RunGUI] ABORTED: RunGUI Worker");

                // Run failed completely
                else
                    fc.LogError_Thread("!!ERROR!! [RunWorkerCompleted_RunGUI] FAILED WITHOUT CATCHING ERRORS: RunGUI Worker");

                // Flag Matlab has issues
                fc.SetAbort(is_mat_failed: true);
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
            char id = ' ';
            double dat1 = 0;
            double dat2 = 0;
            double dat3 = 0;
            UInt16 pack = 0;

            csLog.Print_Thread("[DoWork_MatCOM] RUNNING: MatCOM Worker...");
            BackgroundWorker worker = (BackgroundWorker)sender;

            // Create tuple to pass args
            Tuple<char, double, double, double, UInt16> bw_args = (Tuple<char, double, double, double, UInt16>)e.Argument;

            // Store arguments 
            id = bw_args.Item1;
            dat1 = bw_args.Item2;
            dat2 = bw_args.Item3;
            dat3 = bw_args.Item4;
            pack = bw_args.Item5;

            // Check for matlab input till quit or abort
            while (fc.ContinueMatCom())
            {

                // Pause thread
                if (dt_check < 5)
                    Thread.Sleep((int)(5 - dt_check));

                // Store check time
                dt_check = sw_main.ElapsedMilliseconds - t_check;

                // Check for abort flag
                if (fc.doAbortMat)
                {
                    if (fc.doSoftAbortMat)
                    {
                        fc.LogWarning_Thread("**WARNING** [DoWork_MatCOM] SENDING FLAG FOR ICR_GUI TO SAVE AND ABORT");
                        SendMatCom_Thread(id: 'E', dat1: 2);
                    }
                    else if (fc.doHardAbortMat)
                    {
                        fc.LogWarning_Thread("**WARNING** [DoWork_MatCOM] SENDING FLAG FOR ICR_GUI TO FORCE ABORT");
                        SendMatCom_Thread(id: 'E', dat1: 3);
                    }

                    // Reset flag so only read once
                    fc.doAbortMat = false;
                }

                // Get global variable
                dynamic _m2c_pack;
                lock (lock_m2cPack)
                    _m2c_pack = com_Matlab.GetVariable("m2c_pack", "global");

                // Check for incoming packet
                id = (char)(byte)_m2c_pack.GetValue(0, 0);
                dat1 = _m2c_pack.GetValue(0, 1) != -1 ? (double)_m2c_pack.GetValue(0, 1) : Double.NaN;
                dat2 = _m2c_pack.GetValue(0, 2) != -1 ? (double)_m2c_pack.GetValue(0, 2) : Double.NaN;
                dat3 = _m2c_pack.GetValue(0, 3) != -1 ? (double)_m2c_pack.GetValue(0, 3) : Double.NaN;
                pack = (UInt16)_m2c_pack.GetValue(0, 4);
                flag = (double)_m2c_pack.GetValue(0, 5);

                // Get incoming data
                if (flag == 1)
                {

                    // Change com status
                    if (!fc.isMatComActive)
                        fc.isMatComActive = true;

                    // Trigger progress change event
                    worker.ReportProgress(0, new System.Tuple<char, double, double, double, UInt16>(id, dat1, dat2, dat3, pack));

                    // Set pack and flag back to zero
                    lock (lock_m2cPack)
                        SendMatCom(msg: "m2c_pack(6) = 0;", do_print: false);

                }

            }

            // end polling
            e.Result = " ";
        }

        // PROGRESSCHANGED FOR bw_MatCOM WORKER
        public static void ProgressChanged_MatCOM(object sender, ProgressChangedEventArgs e)
        {
            // Pull out argument vals
            Tuple<char, double, double, double, UInt16> bw_args = (Tuple<char, double, double, double, UInt16>)e.UserState;

            // Store id in top level vars
            char id = bw_args.Item1;
            double[] dat = new double[3] { bw_args.Item2, bw_args.Item3, bw_args.Item4 };
            UInt16 pack = bw_args.Item5;

            // Update com info
            m2c.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: sw_main.ElapsedMilliseconds);
            m2c.SetMsgState(id: id, set_sent_rcvd: true);

            // Handle simulation data
            if (id == 'p' && db.systemTest == 1)
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
                    RepeatSendRobCom_Thread(send_max: 1, id: 'P', dat1: dat1, dat2: dat2, dat3: dat3, do_conf: false);

                // Bail to avoid printing
                return;
            }

            // Store commmon data
            string dat_str = String.Format("id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} dt_rcv={5}",
                    id, dat[0], dat[1], dat[2], pack, m2c.DT_SentRcvd());

            // Check for repeat packet
            string msg_str;
            if (pack != m2c.packLastArr[m2c.ID_Ind(id)])
            {
                // Print received data
                msg_str = "   [RCVD] m2c: ";
                csLog.Print_Thread(msg_str + dat_str, t: m2c.t_new);
            }
            else
            {
                m2c.cnt_repeat++;
                // Print received data
                msg_str = String.Format("   [*RE-RCVD*] m2c: cnt={0} ", m2c.cnt_repeat);
                csLog.Print_Thread(msg_str + dat_str, t: m2c.t_new);
                // Bail without processing
                return;
            }

            // Check for ses saved command
            if (id == 'F')
            {
                fc.isSesSaved = true;
                csLog.Print_Thread("[ProgressChanged_MatCOM] ICR_GUI Confirmed Save");
            }

            // Check for rat in command
            else if (id == 'I')
            {
                fc.isRatInArena = true;
                if (dat[0] == 0)
                {
                    csLog.Print_Thread("[ProgressChanged_MatCOM] ICR_GUI Confirmed Rat On Forage Platform");
                }
                if (dat[0] == 1)
                {
                    fc.isRatOnTrack = true;
                    csLog.Print_Thread("[ProgressChanged_MatCOM] ICR_GUI Confirmed Rat On Track");
                }
            }

            // Check for task status command
            else if (id == 'O')
            {
                fc.isTaskDone = true;
                csLog.Print_Thread("[ProgressChanged_MatCOM] ICR_GUI Confirmed Task Done and Rat Out");
            }

            // Check for quit
            else if (id == 'X')
            {
                // Check if this is a premature quit
                if (!fc.isSesSaved)
                {
                    // Start exiting early
                    fc.SetAbort(set_abort_cs: true);

                    // Print warning
                    if (!fc.isGUIquit)
                        fc.LogWarning_Thread("**WARNING** [DoWork_MatCOM] ICR_GUI QUIT EARLY");
                }

                // Check if this is a forced quit
                if (dat[0] == 2)
                {
                    // Start exiting early
                    fc.SetAbort(set_abort_cs: true);

                    // Print error
                    fc.LogWarning_Thread("**WARNING** [DoWork_MatCOM] ICR_GUI FORCED QUIT");
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
                    if (!fc.isGUIclosed)
                        fc.LogWarning_Thread("**WARNING** [DoWork_MatCOM] ICR_GUI FORCED CLOSE");
                    // Start exiting early
                    fc.SetAbort(set_abort_mat: true);
                }
                // Set flag that GUI has closed
                fc.isGUIclosed = true;
            }

            // Check if mesage should be relayed to rob
            for (int i = 0; i < c2r.idArr.Length - 1; i++)
            {
                if (id == c2r.idArr[i])
                {
                    bool do_check_done = false;

                    // Check if move to command
                    if (id == 'M') // move to
                    {
                        // calculate move to pos
                        dat[1] = CalcMove(dat[1]);

                        // Only check done on first and last
                        if (dat[0] == 0 || dat[0] == 1)
                            do_check_done = true;
                    }
                    // Check if free or cue reward command
                    else if (id == 'R' && (dat[1] == 2 || dat[1] == 3))
                    {
                        // calculate target pos
                        dat[0] = CalcMove(dat[0]);
                    }

                    // Send data
                    RepeatSendRobCom_Thread(id: id, dat1: dat[0], dat2: dat[1], dat3: dat[2], do_check_done: do_check_done);
                }
            }
        }

        // RUNWORKERCOMPLETED FOR bw_MatCOM WORKER
        public static void RunWorkerCompleted_MatCOM(object sender, RunWorkerCompletedEventArgs e)
        {
            csLog.Print_Thread("[RunWorkerCompleted_MatCOM] FINISHED: MatCOM Worker");
        }

        // SEND/STORE DATA FOR MATLAB
        public static void SendMatCom_Thread(string msg = " ", char id = ' ', double dat1 = 0, double dat2 = 0, double dat3 = 0, UInt16 pack = 0, bool do_print = true)
        {
            // Itterate packet
            if (id != ' ')
            {
                pack = ++c2m.packInd;
            }

            // Run method on seperate thread
            new Thread(delegate ()
            {
                SendMatCom(msg: msg, id: id, dat1: dat1, dat2: dat2, dat3: dat3, pack: pack, do_print: do_print);
            }).Start();

        }
        public static void SendMatCom(string msg = " ", char id = ' ', double dat1 = 0, double dat2 = 0, double dat3 = 0, UInt16 pack = 0, bool do_print = true)
        {
            // Local vars
            long t_queued = sw_main.ElapsedMilliseconds;
            string dat_str = " ";

            // Sending packet data
            if (id != ' ')
            {

                if (fc.ContinueMatCom())
                {

                    // Store string to print
                    dat_str = String.Format("id=\'{0}\' dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4}", id, dat1, dat2, dat3, pack);

                    // Log/print packet queued
                    if (do_print)
                        csLog.Print_Thread(String.Format("   [QUEUED] c2m: {0} dt_send={1} dt_q={2}",
                            dat_str, c2m.DT_SentRcvd(sw_main.ElapsedMilliseconds), sw_main.ElapsedMilliseconds - t_queued));

                    // Wait for flag to be reset
                    while (c2m.DT_SentRcvd(sw_main.ElapsedMilliseconds) < c2m.dt_minSentRcvd)
                    {
                        Thread.Sleep(10);
                    }

                    // Create message string
                    msg = String.Format("[c2m_com.{0}.dat1, c2m_com.{0}.dat2, c2m_com.{0}.dat3, c2m_com.{0}.pack] =  deal({1}, {2}, {3}, {4});",
                            id, dat1, dat2, dat3, pack);

                    // Update Matlab variable
                    lock (lock_c2mPack)
                        com_Matlab.Execute(msg);

                    // Update sent
                    double[] dat = new double[3] { dat1, 0, 0 };
                    c2m.UpdateSentRcvd(id: id, dat: dat, pack: pack, t: sw_main.ElapsedMilliseconds);
                    c2m.SetMsgState(id: id, set_sent_rcvd: true);


                    // Log/print sent
                    if (do_print)
                        csLog.Print_Thread(String.Format("   [SENT] c2m: {0} dt_send={1} dt_q={2}",
                            dat_str, c2m.DT_SentRcvd(sw_main.ElapsedMilliseconds), sw_main.ElapsedMilliseconds - t_queued));

                }

                // Log/print warning that packet was not sent
                else
                {
                    fc.LogWarning_Thread(String.Format("**WARNING** [SendMatCom] ABORTED: Send c2m Packet: {0} dt_send={1} dt_q={2}",
                            dat_str, c2m.DT_SentRcvd(sw_main.ElapsedMilliseconds), sw_main.ElapsedMilliseconds - t_queued));
                }

            }

            // Sending simple command
            else
            {

                if (fc.ContinueMatCom())
                {

                    // Execute command
                    com_Matlab.Execute(msg);

                    // Log/print queued
                    if (do_print)
                        csLog.Print_Thread(String.Format("[SendMatCom] Executed Matlab Command: msg=\"{0}\" dt_q={1}",
                            msg, sw_main.ElapsedMilliseconds - t_queued));

                }

                // Log/print warning that message was not sent
                else
                {
                    fc.LogWarning_Thread(String.Format("**WARNING** [SendMatCom] ABORTED: Executed Matlab Command: msg=\"{0}\" dt_q={1}",
                                 msg, sw_main.ElapsedMilliseconds - t_queued));
                }

            }

        }


        // WAIT FOR M2C CONFIRMATION
        public static bool WaitForMatCom(char id, double dat1 = double.NaN, bool chk_send = false, bool chk_rcv = false, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_start = sw_main.ElapsedMilliseconds;
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : t_start + timeout;
            bool first_loop = true;
            bool pass = false;
            string wait_str = " ";

            // Create wait list strings
            string[] wait_list =
                new string[3] { "Send", "Rcv", String.Format("Dat={0:0.00}", dat1) };

            // Track check flags
            bool chk_dat = !Double.IsNaN(dat1);
            bool[] check_4 = new bool[3] { chk_send, chk_rcv, chk_dat };

            // Track wait flags
            bool[] wait_4_now = new bool[check_4.Length];
            check_4.CopyTo(wait_4_now, 0);
            bool[] wait_4_last = new bool[check_4.Length];
            check_4.CopyTo(wait_4_last, 0);

            do
            {

                // SENT
                if (chk_send)
                {
                    bool wait_other = false;
                    wait_4_now[0] = wait_4_now[0] ? wait_other || !c2m.GetMsgState(id: id, get_sent_rcvd: true) : wait_4_now[0];
                }

                // RECIEVED
                if (chk_rcv)
                {
                    bool wait_other = wait_4_now[0];
                    wait_4_now[1] = wait_4_now[1] ? wait_other || !m2c.GetMsgState(id: id, get_sent_rcvd: true) : wait_4_now[1];
                }

                // DATA
                if (chk_dat)
                {
                    bool wait_other = wait_4_now[0] || wait_4_now[1];
                    wait_4_now[2] = wait_4_now[2] ? wait_other || m2c.datMat[m2c.ID_Ind(id)][0] != dat1 : wait_4_now[2];
                }

                // Format check for string
                if (first_loop)
                    wait_str = String.Format("|{0}{1}{2}",
                       wait_4_now[0] ? wait_list[0] + "|" : "",
                       wait_4_now[1] ? wait_list[1] + "|" : "",
                       wait_4_now[2] ? wait_list[2] + "|" : "");

                // Get current status string
                long t_out = timeout == long.MaxValue ? 0 : timeout;
                long dt_wait = sw_main.ElapsedMilliseconds - t_start;
                string dat_str = String.Format("id=\'{0}\' chk_flags=|s={1}|r={2}|d={3}| wait_flags=|s={4}|r={5}|d={6}| do_abort={7} timeout={8} dt_wait={9}",
                         id, check_4[0] ? "T" : "F", check_4[1] ? "T" : "F", check_4[2] ? "T" : "F",
                         wait_4_now[0] ? "T" : "F", wait_4_now[1] ? "T" : "F", wait_4_now[2] ? "T" : "F",
                         do_abort, t_out, dt_wait);

                // Print what we are waiting on
                if (first_loop)
                {
                    csLog.Print(String.Format("   [WaitForMatCom] RUNNING: WAIT FOR...: {0}...: {1}", wait_str, dat_str));
                    first_loop = false;
                }

                // Check for changes
                for (int i = 0; i < wait_list.Length; i++)
                {
                    if (wait_4_now[i] != wait_4_last[i])
                    {
                        string str = wait_list[i];
                        csLog.Print(String.Format("   [WaitForMatCom] CONFIRMED: \'{0}\' \"{1}\"", id, str));
                    }
                    wait_4_last[i] = wait_4_now[i];
                }

                // Check if done
                pass = !wait_4_now[0] && !wait_4_now[1] && !wait_4_now[2];

                // Return success
                if (pass)
                {
                    csLog.Print(String.Format("   [WaitForMatCom] SUCCEEDED: WAIT FOR: {0}: {1}", wait_str, dat_str));
                    return true;
                }

                // Check if need to abort
                else if (
                    (do_abort && fc.doAbortCS) ||
                    !fc.ContinueMatCom() ||
                    (sw_main.ElapsedMilliseconds > t_timeout)
                    )
                {

                    // External forced abort
                    if (do_abort && fc.doAbortCS)
                        fc.LogWarning_Thread(String.Format("**WARNING** [WaitForMatCom] ABORTED: WAIT FOR: FORCED ABORT: {0}: {1}", wait_str, dat_str));
                    else
                    {
                        // Coms failed
                        if (!fc.ContinueMatCom())
                            fc.LogError_Thread(String.Format("!!ERROR!! [WaitForMatCom] ABORTED: WAIT FOR: LOST COMS: {0}: {1}", wait_str, dat_str));

                        // Timedout
                        else if (sw_main.ElapsedMilliseconds > t_timeout)
                            fc.LogError_Thread(String.Format("!!ERROR!! [WaitForMatCom] ABORTED: WAIT FOR: TIMEDOUT: {0}: {1}", wait_str, dat_str));

                        // Check if this is first packet
                        if (m2c.packTot == 0)
                            fc.SetAbort(set_abort_cs: true, is_mat_failed: true);

                        // Set error flag
                        fc.SetAbort(set_abort_mat: true);
                    }

                    // Set flags and bail
                    return false;
                }

                // Pause thread
                else
                    Thread.Sleep(10);

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

                    // Send vt data to robot
                    RepeatSendRobCom_Thread(send_max: 1, id: 'P', dat1: dat1, dat2: dat2, dat3: dat3, do_conf: false);

                    // Log/print first record received
                    if (!vtHandler.is_streamStarted[ent])
                    {
                        csLog.Print_Thread(String.Format("[NetComCallbackVT] FIRST {0} VT RECORD", ent == 0 ? "RAT" : "ROBOT"));
                        vtHandler.is_streamStarted[ent] = true;
                    }
                }

            }
            else if (db.do_printBlockedVt)
            {
                fc.LogWarning_Thread(String.Format("**WARNING** [NetComCallbackVT] VT Blocked: ent={0} cnt={1} dt_snd={2}|{3}|{4}",
                    ent, vtHandler.cnt_block[ent], vtHandler.GetSendDT(ent), vtHandler.GetSendDT(ent, "avg"), vtHandler.GetSendDT(ent, "now")));
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
            x = (x - vt_XC) / vt_R;
            y = (y - vt_YC) / vt_R;

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
            x = Math.Round(x * vt_R) + vt_R;
            y = Math.Round(y * vt_R) + vt_R;

            // Check for negative dt
            if (dt < 0)
            {
                fc.LogWarning_Thread(String.Format("**WARNING** [CompPos] Strange TS Values: ent={0} ts_now={1} ts_last={2} dt={3}",
                    ent, ts_now, ts_last, dt));
            }

            // Convert cart to cm
            x = x * (140 / (vt_R * 2));
            y = y * (140 / (vt_R * 2));

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

        // CHECK CONSOLE FOR BREAK DEBUG REQUEST
        public static void CheckBreakInput()
        {
            // Local vars
            long t_check_enter = sw_main.ElapsedMilliseconds + 1000;
            long t_check_answer = sw_main.ElapsedMilliseconds + 10000;
            string msg;

            // Keep looping
            while (!fc.doExit)
            {

                // Reinitialize vars
                string cmd = null;
                int x;

                // Check for input
                cmd = Console.ReadLine();

                // Bail if no input
                if (cmd == null)
                {
                    // Pause and bail
                    continue;
                }

                // Check for 'b'
                if (cmd == "db")
                {

                    // Print message
                    msg = "\nDO YOU WANT TO DEBUG MATLAB (Y/N + ENTER): ";
                    Console.Write(msg);

                    // Check for input
                    cmd = Console.ReadLine();

                    // Handle response
                    if (cmd == "Y" || cmd == "y")
                    {
                        // Print next message
                        msg = "\nENTER BREAK LINE NUMBER: ";
                        Console.Write(msg);

                        // Get line number
                        x = Convert.ToInt32(Console.ReadLine());

                        // Break matlab at this line
                        msg = String.Format("dbstop at {0} in ICR_GUI", x);
                        SendMatCom(msg: msg);
                    }
                }
            }

        }

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

    }

    #region =============== OTHER CLASSES ===============

    // CLASS TO TRACK PROGRAM FLAGS
    class Flow_Control
    {

        // PRIVATE VARS
        private static DB_Logger _csLog;

        // PUBLIC VARS
        public bool doSessionICR = false;
        public bool doSessionTurnTT = false;
        public bool doSessionUpdateTable = false;
        public bool isNlxConnected = false;
        public bool isRatStreaming = false;
        public bool isRobStreaming = false;
        public bool isMatComActive = false;
        public bool isRobComActive = false;
        public bool isArdComActive = false;
        public bool isMovedToStart = false;
        public bool isRatInArena = false;
        public bool isRatOnTrack = false;
        public bool isTaskDone = false;
        public bool isSesSaved = false;
        public bool isGUIquit = false;
        public bool isGUIclosed = false;
        public bool doAbortMat = false;
        public bool doSoftAbortMat = false;
        public bool doHardAbortMat = false;
        public bool isMatFailed = false;
        public bool doAbortCS = false;
        public bool isAbortRun = false;
        public bool isErrorRun = false;
        public bool doExit = false;

        // CONSTRUCTOR
        public Flow_Control(
            ref DB_Logger cs_log
            )
        {
            _csLog = cs_log;
        }

        // Set error status
        public void SetAbort(bool set_abort_cs = false, bool set_abort_mat = false, bool is_mat_failed = false)
        {

            // Set to abourt CS script
            if (set_abort_cs)
            {
                doAbortCS = true;
                LogWarning("**WARNING** [Flow_Control::SetAbort] SET ABORT CS");
            }

            // Set received check flag
            else if (set_abort_mat)
            {
                // Set flag to trigger sending abort message
                doAbortMat = true;

                // Do hard abourt
                if (!isRatInArena && !isGUIquit)
                {
                    doHardAbortMat = true;
                    doAbortCS = true;
                    LogWarning("**WARNING** [Flow_Control::SetAbort] SET HARD ABORT MATLAB");
                }

                // Attempt to save data
                else if (isRatInArena && !isGUIquit)
                {
                    doSoftAbortMat = true;
                    LogWarning("**WARNING** [Flow_Control::SetAbort] SET SOFT ABORT MATLAB");
                }

            }

            // Check if matlab hanging
            if (is_mat_failed)
            {
                isMatFailed = true;
                doAbortCS = true;
                doAbortMat = true;
                LogError("!!ERROR!! [Flow_Control::SetAbort] MATLAB HAS FAILED/CRASHED");
            }

            // Set main flag
            isAbortRun = true;

        }

        // Log warning
        public void LogWarning_Thread(string msg)
        {
            _csLog.Print_Thread(msg, is_warning: true);
        }
        public void LogWarning(string msg)
        {
            _csLog.Print(msg, is_warning: true);
        }

        // Log error
        public void LogError_Thread(string msg)
        {
            _csLog.Print_Thread(msg, is_error: true);
            isErrorRun = true;
        }
        public void LogError(string msg)
        {
            _csLog.Print(msg, is_error: true);
            isErrorRun = true;
        }

        // Check if Matlab coms are active
        public bool ContinueMatCom()
        {
            bool do_cont = isMatComActive && !isMatFailed && !doExit;
            if (!do_cont)
            {
                string flag_str = String.Format("isMatComActive={0} isMatFailed={1} doExit={2}",
                    isMatComActive, isMatFailed, doExit);
                _csLog.Print("   [Flow_Control::ContinueMatCom] RETURNED DISCONTINUE FLAG: " + flag_str);
            }
            return do_cont;
        }

        // Check if serial Xbee coms are active
        public bool ContinueRobCom()
        {
            bool do_cont = isRobComActive && !doExit;
            if (!do_cont)
            {
                string flag_str = String.Format("isRobComActive={0} doExit={1}",
                    isRobComActive, doExit);
                _csLog.Print("   [Flow_Control::ContinueRobCom] RETURNED DISCONTINUE FLAG: " + flag_str);
            }
            return do_cont;
        }

        // Check if serial CheetahDue coms active
        public bool ContinueArdCom()
        {
            bool do_cont = isArdComActive && !doExit;
            if (!do_cont)
            {
                string flag_str = String.Format("isArdComActive={0} doExit={1}",
                    isArdComActive, doExit);
                _csLog.Print("   [Flow_Control::ContinueArdCom] RETURNED DISCONTINUE FLAG: " + flag_str);
            }
            return do_cont;
        }

    }

    // CLASS TO LOG DB INFO
    class DB_Logger
    {

        // PRIVATE VARS
        private Stopwatch _sw = new Stopwatch();
        private string[] _logList = new string[100000];
        private readonly object _lock_logFlags = new object();
        private readonly object _lock_updateList = new object();
        private readonly object _lock_bytesToRcv = new object();
        private readonly object _lock_console = new object();
        private long _t_logStart = 0;
        private string _lastLogStr = " ";
        private bool _isStarted = false;
        private bool _isLogging = false;
        private bool _isSaved = false;
        private int _bytesToRcv = 0;
        private int next_milestone = 0;
        private const int _n_updates = 10;
        private int[] _import_update_bytes = new int[_n_updates];
        private long[] _warn_line = new long[1000];
        private long[] _err_line = new long[1000];

        // PUBLIC VARS
        public long t_sync;
        public bool isImportTimedout = false;
        public string[] prcnt_str = new string[_n_updates + 1];
        public int cnt_logsStored = 0;
        public int[] cnt_dropped = new int[2] { 0, 0 };
        public int bytesRead = 0;
        public long cnt_warn = 0;
        public int cnt_err = 0;
        public string[] err_list = new string[100];

        // Special public vars
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

        // CONSTRUCTOR
        public DB_Logger(
            Stopwatch stop_watch
            )
        {
            _sw = stop_watch;
        }

        // Print events to console
        public void Print_Thread(string msg_in, bool is_warning = false, bool is_error = false, long t = -1)
        {
            // Print event on seperate thread
            new Thread(delegate ()
            {
                Print(msg_in: msg_in, is_warning: is_warning, is_error: is_error, t: t);
            }).Start();
        }
        public void Print(string msg_in, bool is_warning = false, bool is_error = false, long t = -1)
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
            t = t > 0 ? t : _sw.ElapsedMilliseconds;

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
            lock (_lock_console)
                Console.Write(msg_print);

            // Remove cammas from message
            msg_log = msg_in.Replace(",", string.Empty);

            // Store in logger 
            UpdateList(msg: msg_log, is_warning: is_warning, is_error: is_error, t: t_m_sync);

            // Store error string
            if (is_error)
            {
                err_list[cnt_err] = msg_print;
            }
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
                    if (cnt_logsStored <= _logList.Length)
                    {
                        _logList[cnt_logsStored - 1] = str;
                    }

                    // Max logs stored and end reachedd
                    else
                    {
                        // Set to last entry in array
                        cnt_logsStored = _logList.Length;

                        // Store error in last entry
                        _logList[cnt_logsStored - 1] = String.Format("**WARNING** Log Maxed out at {0} entries", _logList.Length);
                    }

                    // Store error info
                    if (is_error)
                    {
                        _err_line[cnt_err < 1000 ? cnt_err++ : 999] = cnt_logsStored;
                    }
                    else if (is_warning)
                    {
                        _warn_line[cnt_warn < 1000 ? cnt_warn++ : 999] = cnt_logsStored;
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
        public string GetErrorSummary(string get_what)
        {
            // Local vars
            string summary_str = "";

            // Warnings
            if (get_what == "warnings")
            {
                string warn_lines = "ON LINES |";
                for (int i = 0; i < cnt_warn; i++)
                {
                    warn_lines = String.Format("{0}{1}|", warn_lines, _warn_line[i]);
                }
                summary_str = String.Format("TOTAL WARNINGS: {0} {1}", cnt_warn, cnt_warn > 0 ? warn_lines : "");
            }

            // Errors
            else if (get_what == "errors")
            {
                string err_lines = "ON LINES |";
                for (int i = 0; i < cnt_err; i++)
                {
                    err_lines = String.Format("{0}{1}|", err_lines, _err_line[i]);
                }
                summary_str = String.Format("TOTAL ERRORS: {0} {1}", cnt_err, cnt_err > 0 ? err_lines : "");
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

    // CLASS TO TRACK COMS
    class Com_Track
    {

        // PRIVATE VARS
        private string _objID;
        private object _lock_isSentRcv = new object();
        private object _lock_isConf = new object();
        private object _lock_isDone = new object();
        private bool[] _isSentRcv;
        private bool[] _isConf;
        private bool[] _isDone;

        // PUBLIC VARS
        public char[] idArr;
        public byte[] head = new byte[1] { 0 };
        public byte[] foot = new byte[1] { 0 };
        public double[][] datMat;
        public UInt16[] packArr;
        public UInt16[] packLastArr;
        public long dt_minSentRcvd;
        public long dt_resend;
        public int resendMax = 5;
        public UInt16 packTot = 0;
        public UInt16 packInd = 0;
        public int[] cnt_dropped = new int[2] { 0, 0 };
        public int cnt_repeat = 0;
        public long t_new = 0;
        public long t_last = 0;
        public long t_parse_str = 0;
        public long[] t_sentRcvd;

        // CONSTRUCTOR
        public Com_Track(
            string obj_id,
            object _lock_is_conf,
            object _lock_is_done,
            char[] _id,
            byte _head = 0,
            byte _foot = 0,
            long _dt_min_sent_rcvd = 0,
            long _dt_resend = 0,
            int _resend_max = 0
            )
        {
            _objID = obj_id;
            _lock_isConf = _lock_is_conf;
            _lock_isDone = _lock_is_done;
            idArr = _id;
            head[0] = _head;
            foot[0] = _foot;
            dt_minSentRcvd = _dt_min_sent_rcvd;
            dt_resend = _dt_resend;
            resendMax = _resend_max;
            datMat = new double[_id.Length][];
            packArr = new UInt16[_id.Length];
            packLastArr = new UInt16[_id.Length];
            t_sentRcvd = new long[_id.Length];
            _isSentRcv = new bool[_id.Length];
            _isConf = new bool[_id.Length];
            _isDone = new bool[_id.Length];

            // Initialize values to zero
            for (int i = 0; i < _id.Length; i++)
            {
                packArr[i] = 0;
                datMat[i] = new double[3] { 0, 0, 0 };
                packLastArr[i] = 0;
                t_sentRcvd[i] = 0;
                _isSentRcv[i] = false;
                _isConf[i] = false;
                _isDone[i] = false;
            }
        }

        // Set check status
        public void SetMsgState(char id = ' ', UInt16 pack = 0, bool set_sent_rcvd = false, bool set_conf = false, bool set_done = false, bool state = true)
        {
            // Local vars
            int id_ind = id != ' ' ? ID_Ind(id) : PackID_Ind(pack);

            // Set sent/received check flag
            if (set_sent_rcvd)
            {
                lock (_lock_isSentRcv)
                    _isSentRcv[id_ind] = state;
            }

            // Set received check flag
            else if (set_conf)
            {
                lock (_lock_isConf)
                    _isConf[id_ind] = state;
            }

            // Set done check flag
            else if (set_done)
            {
                lock (_lock_isDone)
                    _isDone[id_ind] = state;
            }

        }

        // Get check status
        public bool GetMsgState(char id = ' ', bool get_sent_rcvd = false, bool get_conf = false, bool get_done = false)
        {
            // Local vars
            int id_ind = ID_Ind(id);
            bool val = false;

            // Set sent/received check flag
            if (get_sent_rcvd)
            {
                lock (_lock_isSentRcv)
                {
                    val = _isSentRcv[id_ind];
                    _isSentRcv[id_ind] = false;
                }
            }

            // Set received check flag
            else if (get_conf)
            {
                lock (_lock_isConf)
                {
                    val = _isConf[id_ind];
                    _isConf[id_ind] = false;
                }
            }

            // Set done check flag
            else if (get_done)
            {
                lock (_lock_isDone)
                {
                    val = _isDone[id_ind];
                    _isDone[id_ind] = false;
                }
            }

            // Return last value
            return val;
        }

        // Update packet info
        public void UpdateSentRcvd(char id, double[] dat, UInt16 pack = 0, long t = 0)
        {
            // Get id ind
            int id_ind = ID_Ind(id);

            // Update data
            this.datMat[id_ind][0] = dat[0];
            this.datMat[id_ind][1] = dat[1];
            this.datMat[id_ind][2] = dat[2];

            // Itterate packet total
            packTot++;

            // Update packet history
            packLastArr[id_ind] = this.packArr[id_ind];
            this.packArr[id_ind] = pack;

            // Update recieved packet ind
            if (_objID != "c2r" && _objID != "c2m")
            {
                if (_objID == "m2c" || (_objID == "r2c" && pack > UInt16.MaxValue / 2))
                {
                    packInd = pack > packInd ? pack : packInd;
                }
            }

            // Update timers
            t_last = t_new;
            t_new = t;
            t_sentRcvd[id_ind] = t;

            // Reset consecutive dropped packs
            cnt_dropped[0] = 0;
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
            // Set t2 to input time if time given or greater of t_parse_str or newest sent/rcvd
            long t2 = t != 0 ? t :
                t_parse_str > t_new ? t_parse_str : t_new;

            // Set t1 to last sent/rcvd if no input geven otherwise set to greater of t_parse_str or newest sent/rcvd
            long t1 = t == 0 ? t_last :
                t_parse_str > t_new ? t_parse_str : t_new;

            // Return t2-t1
            return t2 - t1;
        }

        // Find id index
        public int ID_Ind(char id)
        {
            int ind = -1;
            for (int i = 0; i < this.idArr.Length; i++)
            {
                if (id == this.idArr[i])
                {
                    ind = i;
                }
            }
            return ind;
        }

        // Find packet index
        public int PackID_Ind(UInt16 pack)
        {
            // Local vars
            bool[] flag_arr = new bool[this.packArr.Length];
            int id_ind = 0;
            int cnt = 0;

            // Flag matching packets
            for (int i = 0; i < this.packArr.Length; i++)
            {
                if (pack == this.packArr[i])
                {
                    flag_arr[i] = true;
                    cnt++;
                }
                else
                    flag_arr[i] = false;
            }

            // Store newest packet id ind
            for (int i = 0; i < flag_arr.Length; i++)
            {
                if (flag_arr[i])
                {
                    // Check if this is newest
                    for (int j = 0; j < this.packArr.Length; j++)
                    {
                        if (!flag_arr[j])
                            continue;
                        else if (this.t_sentRcvd[i] >= this.t_sentRcvd[j])
                            id_ind = i;
                    }
                }
            }

            // Return array
            return id_ind;
        }

    }

    // CLASS TO HANDLE VT DATA
    class VT_Handler
    {

        // PRIVATE VARS
        private static Stopwatch _sw = new Stopwatch();
        private static readonly object _lockBlock = new object();
        private static int _cntThread = 0;
        private static long _t_blockTim = 0;
        private static long _blockFor = 60; // (ms) 

        // PUBLIC VARS
        public bool[] is_streamStarted = new bool[2] { false, false };
        public long[] t_sent = new long[2] { 0, 0 };
        public long[] t_sent_last = new long[2] { 0, 0 };
        public int[,] dt_hist = new int[2, 10];
        public int[] cnt_sent = new int[2] { 0, 0 };
        public int[] cnt_block = new int[2] { 0, 0 };

        // CONSTRUCTOR
        public VT_Handler(
            Stopwatch stop_watch
            )
        {
            _sw = stop_watch;
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
        public int GetSendDT(int ent, string what = "last")
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
            else if (what == "last")
            {
                return (int)(t_sent[ent] - t_sent_last[ent]);
            }
            else if (what == "now")
            {
                return (int)(_sw.ElapsedMilliseconds - t_sent[ent]);
            }
            else
                return 0;
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

        // CONSTRUCTOR:
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