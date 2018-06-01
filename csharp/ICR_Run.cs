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

    public class ICR_Run
    {

        #region ============ DEBUG SETTINGS =============

        public static DEBUG.DB_FLAG flag = new DEBUG.DB_FLAG(

        /* Set System Test
            0: No test
            1: Simulated rat test
            2: PID calibration
            3: VT calibration
            4: Halt error test
            5: Wall image IR sync timing
            6: IR sync timing
            7: Hardware test */
        systemTest: 0, // 0

        /* Debug matlab
            [0]: Dont break on errors
            [1]: Break on errors
            [>1]: Break on line */
        breakDebug: 1, // 0

        // Number of test serial packets [1:50]
        n_testPings: 5, // 5

        /*Autoload rat data
            true: Load rat data based on ICR_GUI hardcoded values
            false: Start normally */
        do_autoloadUI: false, // false

        // Print all blocked vt recs
        do_printBlockedVT: false, // false

        // Print all sent rat vt recs
        do_printSentRatVT: false, // false

        // Print all sent rob vt recs
        do_printSentRobVT: false, // false

        // Print all sent rat vt recs
        do_printRobLog: false, // false

        // Print CheetahDue logs
        do_printDueLog: false, // false

        // Print com flags
        do_printComFlags: false // false

        );

        #endregion

        #region ============= TOP LEVEL VARS ============

        // Create lock objects and thread lists for safe threading
        static readonly object lock_console = new object();
        static readonly object lock_matCom = new object();
        static readonly object lock_sendPack = new object();

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

        // Intialize logging objects
        private static LOGGER logger_feederDue = new LOGGER(header: "RCVD, LOG, TS (ms), TS (string), LOOP, TYPE, MESSAGE");
        private static LOGGER logger_cheetahDue = new LOGGER(header: "RCVD, LOG, TS (ms), TS (string), LOOP, TYPE, MESSAGE");
        private static UNION_HACK u_logBytes = new UNION_HACK(0, '0', 0, 0, 0);

        // MATLAB Directories
        private static string matStartDir = @"C:\Users\lester\MeDocuments\AppData\MATLAB\Startup";

        // Neuralynx Directories
        private static string nlxTempTop = @"C:\CheetahData\Temp"; // cheetah temp dir
        private static string nlxTempSub = @"0000-00-00_00-00-00"; // cheetah temp sub dir
        private static string nlxSaveTop = @"E:\BehaviorPilot"; // cheetah recording dir
        private static string nlxRatSub = nlxTempTop; // cheetah recording dir
        private static string nlxRecSub = nlxTempSub; // cheetah sub dir

        // Log Directories
        private static string logTempDir = Path.Combine(nlxTempTop, nlxTempSub); // log temp dir
        private static string feederDueLogFi = @"FeederDue_Log.csv"; // log file for FeederDue
        private static string cheetahDueLogFi = @"CheetahDue_Log.csv"; // log file for CheetahDue
        private static string csLogFi = @"ICR_Run_Log.csv"; // log file for ICR_Run
        private static string matLogFi = @"ICR_GUI_Log.csv"; // log file for ICR_Run

        // Timeouts
        private static long TIMEOUT_5 = 5000; // (ms)
        private static long TIMEOUT_15 = 15000; // (ms) 
        private static long TIMEOUT_30 = 30000; // (ms)
        private static long TIMEOUT_60 = 60000; // (ms)
        private static long TIMEOUT_120 = 120000; // (ms)
        private static long TIMEOUT_FeederDueLogRead = 5000; // (ms)
        private static long TIMEOUT_FeederDueLogImport = 120000; // (ms)

        // Initialize vt handler object
        private static VT_HANDLER vt = new VT_HANDLER(
            dt_block: 60,
            feedDist: 66 * ((2 * Math.PI) / (140 * Math.PI))
            );

        // Define Com vars
        private static MNetComClient netcomClient = new MNetComClient();
        private static string netcomAppID = "ICR_Run"; // string displayed in Cheetah when connected
        private static string netcomAcqEntVT1 = "VT1"; // aquisition entity to stream
        private static string netcomAcqEntVT2 = "VT2"; // aquisition entity to stream
        private static string netcomServerIP = "192.168.3.100"; // host computer IP 9"127.0.0.1")
        private static string csXbeePort = "COM22";
        private static string csCheetahDuePort = "COM3";
        static UInt16[] pack_range = { 1, UInt16.MaxValue - 1 };

        // Matlab to CS
        private static COM_TRACK m2c = new COM_TRACK(
            objID: "m2c",
            packRange: pack_range,
            id:
            new char[18]{ // prefix giving masage id
            'i', // gui initialized
            'h', // setup handshake
            'p', // simulation data 
            'G', // session type
            'A', // connected to AC computer 
            'T', // system test command 
            'N', // netcom setup 
            'F', // data saved 
            'X', // confirm quit
            'C', // confirm close
            'W', // setup selection done
            'S', // setup task session 
            'M', // move to position
            'R', // run reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // rat in/out
            'O'  // confirm task done
             },
            dt_minSentRcvd: 0
            );

        // CS to Matlab
        private static COM_TRACK c2m = new COM_TRACK(
            objID: "c2m",
            packRange: pack_range,
            id:
            new char[9] {
            'h', // setup handshake
            'N', // netcom setup confirmation
            'J', // battery voltage
            'Z', // reward zone
            'K', // feederdue status
            'Y', // task done
            'F', // confirm save
            'E', // enable exit
            'C', // confirm close
             },
            dt_minSentRcvd: 100
        );

        // CS to Matlbab queue
        private static COM_TRACK c2m_queue = c2m;

        // CS to FeederDue
        private static COM_TRACK c2r = new COM_TRACK(
            objID: "c2r",
            packRange: pack_range,
            id:
            new char[17] {
            'h', // setup handshake
			'n', // ping test packets
			'T', // system test
            'K', // feederdue status
			'S', // setup session
			'Q', // quit session
			'M', // move to position
			'R', // run reward
			'H', // halt movement
			'B', // bulldoze rat
			'I', // rat in/out
			'L', // request log conf/send
			'J', // battery voltage
			'Z', // reward zone
            'O', // confirm task done
			'U', // log size
			'P', // position data
             },
            head: (byte)'<',
            foot: (byte)'>',
            dt_minSentRcvd: 5,
            dt_resend: 500,
            resendMax: 20
        );

        // FeederDue to CS
        private static COM_TRACK r2c = new COM_TRACK(
            objID: "r2c",
            packRange: pack_range,
            id:
            new char[17] {
            'h', // setup handshake
			'n', // ping test packets
			'T', // system test command
            'K', // feederdue status
			'S', // setup session
			'Q', // quit session
			'M', // move to position
			'R', // run reward
			'H', // halt movement
			'B', // bulldoze rat
			'I', // rat in/out
			'L', // request log conf/send
			'J', // battery voltage
			'Z', // reward zone
            'O', // confirm task done
			'U', // log size
			'P', // position data
             },
            head: (byte)'<',
            foot: (byte)'>',
            dt_minSentRcvd: 5
        );

        // CheetaDue to CS
        private static COM_TRACK a2c = new COM_TRACK(
            objID: "a2c",
            packRange: pack_range,
            id:
            new char[1] {
            ' '
            },
            head: (byte)'<',
            foot: (byte)'>'
        );

        #endregion

        #region ================= MAIN ==================

        // MAIN
        static void main()
        {

            // Print debug status
            if (System.Diagnostics.Debugger.IsAttached)
            {
                DEBUG.DB_General("RUN MODE = DEBUG");
                DEBUG.flag.is_debugRun = true;
            }
            else
            {
                DEBUG.DB_General("RUN MODE = RELEASE");
            }

            // SETUP
            DEBUG.DB_General("Setup() RUNNING");
            bool passed_setup = Setup();
            if (passed_setup)
                DEBUG.DB_General("Setup() SUCCEEDED");
            else
            {
                DEBUG.DB_Warning("Setup() ABORTED");
            }


            // RUN 
            if (passed_setup)
            {
                DEBUG.DB_General("Run() RUNNING");
                if (FC.do_SessionICR)
                {
                    bool passed_run = Run();
                    if (passed_run)
                        DEBUG.DB_General("Run() SUCCEEDED");
                    else
                        DEBUG.DB_Warning("Run() ABORTED");
                }
                else
                {
                    DEBUG.DB_Warning("Run() SKIPPED");
                }
            }

            // EXIT 
            DEBUG.DB_General("Exit() RUNNING");
            Exit();
            DEBUG.DB_General("Exit() FINISHED");

            // Pause before final exit
            Thread.Sleep(1000);

        }

        // SETUP
        public static bool Setup()
        {
            // LOCAL VARS
            DEBUG.STATUS status;

            // START DEBUGGING
            DEBUG.DB_Setup(ref com_Matlab);

            // INITIALIZE MATLAB GLOBAL VARS

            DEBUG.DB_Status(msg: DEBUG.msg = " Create mCOM Global Variables", status: DEBUG.STATUS.RUNNING);
            System.Array m2c_pack = new double[6] { 0, 0, 0, 0, 0, 0 };
            com_Matlab.PutWorkspaceData("m2c_pack", "global", m2c_pack);
            com_Matlab.PutWorkspaceData("m2c_rat_sub", "global", " ");
            com_Matlab.PutWorkspaceData("m2c_rec_sub", "global", " ");
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);

            // SETUP/START ICR_GUI BACKROUND WORKER

            DEBUG.DB_Status(msg: DEBUG.msg = "Start RunGUI Worker", status: DEBUG.STATUS.RUNNING);
            // Set com flag
            FC.is_MatComActive = true;
            // Setup RunGUI worker
            bw_RunGUI.DoWork += DoWork_RunGUI;
            bw_RunGUI.RunWorkerCompleted += RunWorkerCompleted_RunGUI;
            // Start RunGUI worker
            bw_RunGUI.RunWorkerAsync();
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);

            // SETUP/RUN MATLAB COM BACKROUND WORKER

            DEBUG.DB_Status(msg: DEBUG.msg = "Start MatCOM Worker", status: DEBUG.STATUS.RUNNING);
            // Setup MatCOM worker
            bw_MatCOM.DoWork += DoWork_MatCOM;
            bw_MatCOM.ProgressChanged += ProgressChanged_MatCOM;
            bw_MatCOM.RunWorkerCompleted += RunWorkerCompleted_MatCOM;
            bw_MatCOM.WorkerReportsProgress = true;
            // Start MatCOM worker
            var bw_args = Tuple.Create(' ', (double)0, (double)0, (double)0, (UInt16)0);
            bw_MatCOM.RunWorkerAsync(bw_args);
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);

            // WAIT ICR_GUI INITIALIZED CONFIRMATION

            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Initialize Confirmation", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'i', chk_rcv: true, do_abort: true, timeout: TIMEOUT_30);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
            {
                // Set matlab coms flag to inactive
                FC.is_MatComActive = false;
                return false;
            }

            // WAIT FOR SESSION TYPE INFO FROM MATLAB
            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Session Type Selection", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'G', chk_rcv: true, do_abort: true);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
                return false;

            // WAIT FOR MATLAB TO CONNECT TO AC COMPUTER

            DEBUG.DB_Status(msg: DEBUG.msg = "AC Connect Confirmation", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'A', chk_rcv: true, do_abort: true, timeout: TIMEOUT_30);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
            {
                // Flag to abort CS and flag MATLAB failed
                FC.SetAbort(set_abort_cs: true, is_mat_failed: true);
                return false;
            }

            // WAIT FOR MATLAB HANDSHAKE REQUEST

            DEBUG.DB_Status(msg: DEBUG.msg = "Initial ICR_GUI Handshake", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'h', dat1: 0, chk_rcv: true, do_abort: true, timeout: TIMEOUT_15);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
                return false;

            // HANDLE NON-ICR SESSION

            if (!FC.do_SessionICR)
            {

                // Send final handshake directory to ICR_GUI
                DEBUG.DB_Status(msg: DEBUG.msg = "Send ICR_GUI Handshake Command", status: DEBUG.STATUS.AWAITING);
                SendMatCom_Thread(id: 'h', dat1: 2);
                status = WaitForMatCom(id: 'h', chk_send: true, do_abort: true);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
                if (status.ToString() == "SUCCEEDED")
                {
                    // Store sync time based on send time
                    DEBUG.t_sync = c2m.ID_T_Last('h');

                    // Log sync time
                    DEBUG.DB_General(String.Format("SET SYNC TIME: {0}ms", DEBUG.t_sync), ts: DEBUG.t_sync);

                    // Exit out of startup
                    DEBUG.DB_General("HANDSHAKE COMPLETE");
                    return true;
                }

                else
                {
                    DEBUG.DB_Error("HANDSHAKE FAILED");
                    return false;
                }

            }

            // SETUP/START CHEETAHDUE SERIAL

            DEBUG.DB_Status(msg: DEBUG.msg = "Setup CheetahDue Serial Coms and Logging", status: DEBUG.STATUS.RUNNING);
            sp_cheetahDue.ReadTimeout = 100;
            sp_cheetahDue.BaudRate = 57600;
            sp_cheetahDue.PortName = csCheetahDuePort;
            // Set com flag
            FC.is_CheetahDueComActive = true;
            // Open serial port connection
            sp_cheetahDue.Open();
            // Start getting new data on seperate thread
            new Thread(delegate ()
            {
                ParseCheetahDueCom();
            }).Start();
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);

            // SETUP/START XBEE SERIAL

            DEBUG.DB_Status(msg: DEBUG.msg = "Setup Xbee Serial Coms", status: DEBUG.STATUS.RUNNING);
            sp_Xbee.ReadTimeout = 100;
            sp_Xbee.BaudRate = 57600;
            sp_Xbee.PortName = csXbeePort;
            // Set com flag
            FC.is_FeederDueComActive = true;
            // Set byte threshold to max packet size
            sp_Xbee.ReceivedBytesThreshold = 1;
            // Open serial port connection
            sp_Xbee.Open();
            // Spin up parser thread
            new Thread(delegate ()
            {
                ParseFeederDueCom();
            }).Start();
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);

            // SEND CHEETAHDUE HANDSHAKE

            DEBUG.DB_Status(msg: DEBUG.msg = "Send CheetahDue Handshake Command", status: DEBUG.STATUS.RUNNING);
            byte[] out_byte = new byte[1] { (byte)'h' };
            sp_cheetahDue.Write(out_byte, 0, 1);
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);

            // WAIT FOR SYNC TIME HANDSHAKE

            DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue Sync Time Handshake", status: DEBUG.STATUS.AWAITING);
            status = WaitForFeederDueCom(id: 'h', dat1: 1, chk_rcv: true, do_abort: true, timeout: TIMEOUT_5);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() == "SUCCEEDED")
            {
                // Store sync time based on recieve time
                DEBUG.t_sync = r2c.ID_T_Last('h');

                // Log success and sync time
                DEBUG.DB_General(String.Format("SET SYNC TIME: {0}ms", DEBUG.t_sync), ts: DEBUG.t_sync);

                // Wait for final confirmation
                DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue Handshake Finished Command", status: DEBUG.STATUS.AWAITING);
                status = WaitForFeederDueCom(id: 'h', dat1: 2, chk_rcv: true, do_abort: true, timeout: TIMEOUT_5);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);

                // Get feederdue log number
                double log_num = r2c.ID_Dat('h', 1);
                DEBUG.DB_General(String.Format("FEEDERDUE LOGGING TO \"LOG{0:00000}.CSV\"", log_num));

            }

            // Get final handshake status
            if (status.ToString() == "SUCCEEDED")
                DEBUG.DB_General("HANDSHAKE COMPLETE");
            else
            {
                DEBUG.DB_Error("HANDSHAKE FAILED");

                // Unset coms active flag
                FC.is_FeederDueComActive = false;
                FC.is_CheetahDueComActive = false;
                return false;
            }

            // SEND TEST SETUP INFO TO FEEDERDUE

            // Send test setup command
            DEBUG.DB_Status(msg: DEBUG.msg = "Send FeederDue Test Setup Command", status: DEBUG.STATUS.RUNNING);
            RepeatSendFeederDueCom_Thread(id: 'T', dat1: DEBUG.flag.systemTest, dat2: 0, dat3: DEBUG.flag.n_testPings);

            // Wait for testing setup to finish
            DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue Test Setup Done", status: DEBUG.STATUS.AWAITING);
            status = WaitForFeederDueCom(id: 'T', chk_send: true, chk_conf: true, chk_done: true, timeout: DEBUG.flag.n_testPings <= 50 ? TIMEOUT_30 : TIMEOUT_120);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() == "SUCCEEDED")
            {

                // Log average ping time
                double dt_ping_r2c = r2c.ID_Dat('n', 0);
                double dt_ping_r2a = r2c.ID_Dat('n', 1);
                double cnt_resend = r2c.ID_Dat('n', 2);
                DEBUG.DB_General(String.Format("PING SUMMARY: n_pings={0} r2c={1:0.00}ms r2a={2:0.00}ms resend={3}",
                    DEBUG.flag.n_testPings, dt_ping_r2c, dt_ping_r2a, cnt_resend));
            }
            else
                return false;

            // WAIT FOR MATLAB UI SELECTION
            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI UI Selection", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'W', chk_rcv: true, do_abort: true);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
                return false;

            // WAIT FOR MATLAB NETCOM CONNECT CONFIRMATION
            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI NLX Connect Confirmation", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'N', dat1: 1, chk_rcv: true, do_abort: true, timeout: TIMEOUT_60);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() == "SUCCEEDED")
                // Send NLX setup confirmation 
                SendMatCom_Thread(id: 'N', dat1: 1);
            else
                return false;

            // SETUP NETCOM

            // Initilize deligate for VT callback
            deligate_netComCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            netcomClient.SetCallbackFunctionVT(deligate_netComCallback, new ICR_Run());

            // Connect to NetCom
            DEBUG.DB_Status(msg: DEBUG.msg = "Connect to NetCom", status: DEBUG.STATUS.RUNNING);
            FC.is_NlxConnected = netcomClient.AreWeConnected();
            while (!FC.is_NlxConnected && !FC.do_AbortCS)
            {
                FC.is_NlxConnected = netcomClient.ConnectToServer(netcomServerIP);
                // Pause loop
                Thread.Sleep(10);
            }
            if (FC.is_NlxConnected)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SUCCEEDED);
            else
            {
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);
                return false;
            }

            // START NETCOM STREAMING

            // Send App name
            netcomClient.SetApplicationName(netcomAppID);

            // Start feederdue vt streaming
            DEBUG.DB_Status(msg: DEBUG.msg = "Open VT2 Stream", status: DEBUG.STATUS.RUNNING);
            while (!FC.is_RobStreaming && !FC.do_AbortCS)
            {
                FC.is_RobStreaming = netcomClient.OpenStream(netcomAcqEntVT2);
                // Pause loop
                Thread.Sleep(10);
            }
            if (FC.is_RobStreaming)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SUCCEEDED);
            else
            {
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);
                return false;
            }

            // WAIT FOR MATLAB TO FINISH CONFIGURING CHEETAH
            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Cheetah Configured Confirmation", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'N', dat1: 2, chk_rcv: true, do_abort: true, timeout: TIMEOUT_30);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
            {
                // Flag to abort CS and flag MATLAB failed because likely hanging trying to send NLX command
                FC.SetAbort(set_abort_cs: true, is_mat_failed: true);
                return false;
            }

            // SEND STREAM STATUS REQUEST TO FEEDERDUE

            DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue Streaming Confirmation", status: DEBUG.STATUS.AWAITING);
            status = WaitForFeederDueCom(id: 'K', chk_rcv: true, do_abort: true, timeout: TIMEOUT_30);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
                return false;

            // WAIT FOR SESSION SETUP PARAMETERS

            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Setup Parameters Command", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'S', dat1: 2, chk_rcv: true, do_abort: true);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() == "SUCCEEDED")
            {
                status = WaitForFeederDueCom(id: 'S', dat1: 2, chk_send: true, chk_conf: true, do_abort: true);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            }
            else
                return false;

            // RETURN SETUP SUCCESS

            return true;

        }

        // RUN
        public static bool Run()
        {
            // LOCAL VARS
            DEBUG.STATUS status;
            bool run_pass = true;

            // WAIT FOR COMPLETION OF INITIAL FEEDERDUE MOVE

            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI MoveTo Start Command", status: DEBUG.STATUS.AWAITING);
            // Wait for matlab
            status = WaitForMatCom(id: 'M', chk_rcv: true, do_abort: true);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            // Wait for move done
            if (status.ToString() == "SUCCEEDED")
            {
                DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue MoveTo Start Done", status: DEBUG.STATUS.AWAITING);
                status = WaitForFeederDueCom(id: 'M', chk_send: true, chk_conf: true, chk_done: true, do_abort: true, timeout: TIMEOUT_30);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
                if (status.ToString() == "SUCCEEDED")
                {
                    // Send confirm feederdue first move done to Matlbab
                    SendMatCom_Thread(id: 'K', dat1: 2);
                    FC.is_MovedToStart = true;
                }
            }
            if (status.ToString() != "SUCCEEDED")
            {
                // ABORT RUN
                run_pass = false;
                FC.SetAbort(set_abort_cs: true, set_abort_mat: true);
            }

            // WAIT FOR RAT IN CONFIRMATION

            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Rat In Confirmation", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'I', chk_rcv: true, do_abort: true);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() == "SUCCEEDED")
            {
                // Pause for flags to be set
                Thread.Sleep(100);

                // Stream rat vt if not forage task or simulation test
                if (DEBUG.flag.systemTest != 1 && FC.is_RatOnTrack)
                {
                    DEBUG.DB_Status(msg: DEBUG.msg = "Open VT1 Stream", status: DEBUG.STATUS.RUNNING);
                    while (!FC.is_RatStreaming && !FC.do_AbortCS)
                    {
                        FC.is_RatStreaming = netcomClient.OpenStream(netcomAcqEntVT1);
                        // Pause loop
                        Thread.Sleep(10);
                    }
                    if (FC.is_RatStreaming)
                        DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SUCCEEDED);
                    else
                    {
                        DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);
                        run_pass = false;
                    }
                }
                else
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);
            }
            else
                run_pass = false;

            // HOLD HERE TILL TASK COMPLETE

            DEBUG.DB_Status(msg: DEBUG.msg = "MAIN SESSION LOOP", status: DEBUG.STATUS.RUNNING);
            // Stay in loop till task is done or error
            while (
                netcomClient.AreWeConnected() &&
                FC.is_RatInArena &&
                !FC.is_GUIclosed &&
                !FC.is_TaskDone &&
                !FC.do_AbortCS
                )
            {
                // Pause loop
                Thread.Sleep(10);
            }
            if (!FC.do_AbortCS)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SUCCEEDED);

            else
            {
                if (!netcomClient.AreWeConnected())
                {
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);
                    DEBUG.DB_Error("NLX DISCONECTED");
                    FC.SetAbort(set_abort_cs: true, set_abort_mat: true);
                }
                else
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.ABORTED);
                run_pass = false;
            }

            // WAIT FOR TASK DONE CONFIRMATION FROM MATLAB
            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Task Finished Command", status: DEBUG.STATUS.AWAITING);
            if (!FC.is_TaskDone && !FC.is_GUIclosed)
            {
                status = WaitForMatCom(id: 'O', chk_rcv: true, timeout: !FC.do_AbortCS ? TIMEOUT_30 : TIMEOUT_5);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
                if (status.ToString() == "SUCCEEDED")
                    // Wait for all the other crap to be relayed from Matlab
                    Thread.Sleep(1000);
                else
                    run_pass = false;
            }
            else
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            // WAIT FOR TASK DONE FEEDERDUE CONFIRMATION

            // Wait for task done recieve confirmation
            DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue Task Finished Confirmation", status: DEBUG.STATUS.AWAITING);
            if (FC.is_TaskDone)
            {
                status = WaitForFeederDueCom(id: 'O', chk_send: true, chk_conf: true, timeout: TIMEOUT_5);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
                if (status.ToString() != "SUCCEEDED")
                    run_pass = false;
            }
            else
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            // SEND/WAIT FOR FINAL MOVE TO COMPLETE

            DEBUG.DB_Status(msg: DEBUG.msg = "Send FeederDue MoveTo South Command", status: DEBUG.STATUS.RUNNING);
            if (FC.is_MovedToStart)
            {
                // Calculate move to position
                double move_to = CalcMove(4.7124 - vt.feedDist);

                // Send move command on seperate thread and wait for done reply
                RepeatSendFeederDueCom_Thread(id: 'M', dat1: 0, dat2: move_to, do_check_done: true);

                // Wait for confirmation from feederdue
                DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue MoveTo South Done", status: DEBUG.STATUS.AWAITING);
                status = WaitForFeederDueCom(id: 'M', dat1: 0, chk_send: true, chk_conf: true, chk_done: true, timeout: TIMEOUT_15);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
                if (status.ToString() != "SUCCEEDED")
                    run_pass = false;
            }
            else
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);


            // SEND TASK DONE CONFIRMATION TO MATLAB

            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Task Finished Confirmation", status: DEBUG.STATUS.AWAITING);
            if (FC.is_TaskDone)
            {
                // Send task done confirmation to Matlab
                SendMatCom_Thread(id: 'Y', dat1: 1);

                // Wait for send
                status = WaitForMatCom(id: 'Y', chk_send: true, do_abort: true);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            }
            else
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);



            // RETURN RUN STATUS
            return run_pass;
        }

        // EXIT
        public static void Exit()
        {
            // LOCAL VARS
            DEBUG.STATUS status;

            // SEND FEEDERDUE LOG REQUEST

            DEBUG.DB_Status(msg: DEBUG.msg = "Send FeederDue Log Bytes Command", status: DEBUG.STATUS.RUNNING);
            if (!FC.do_SessionICR || !sp_Xbee.IsOpen)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            else
            {
                RepeatSendFeederDueCom_Thread(id: 'L', dat1: 0);
                status = WaitForFeederDueCom(id: 'L', chk_send: true, chk_conf: true, timeout: TIMEOUT_5);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
                if (status.ToString() == "SUCCEEDED")
                {
                    // Wait for bytes to receive messages to be received
                    DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue Log Bytes Message", status: DEBUG.STATUS.AWAITING);
                    status = WaitForFeederDueCom(id: 'U', chk_rcv: true, timeout: TIMEOUT_5);
                    DEBUG.DB_Status(msg: DEBUG.msg, status: status);
                    if (status.ToString() == "SUCCEEDED")
                    {
                        // Store data byte
                        double[] dat_arr =
                            new double[3] { r2c.ID_Dat('U', 0), r2c.ID_Dat('U', 1), r2c.ID_Dat('U', 2) };
                        logger_feederDue.UpdateBytesToRcv(dat_arr);
                        DEBUG.DB_General(String.Format("FeederDue Log Bytes Expeted {0}", logger_feederDue.bytesToRcv));

                        // Flag importing started
                        logger_feederDue.is_streaming = true;

                        // Start importing
                        Thread get_log_thread = new Thread(delegate ()
                        {
                            GetFeederDueLog(TIMEOUT_FeederDueLogImport, TIMEOUT_FeederDueLogRead);
                        });
                        get_log_thread.Priority = ThreadPriority.Highest;
                        get_log_thread.Start();

                        // Tell feederdue to begin streaming log and wait for message to send
                        DEBUG.DB_Status(msg: DEBUG.msg = "Send FeederDue Send Logs Command", status: DEBUG.STATUS.RUNNING);
                        RepeatSendFeederDueCom_Thread(id: 'L', dat1: 1, do_conf: false);
                        status = WaitForFeederDueCom(id: 'L', chk_send: true, timeout: TIMEOUT_5);
                        DEBUG.DB_Status(msg: DEBUG.msg, status: status);
                    }
                }
            }

            // SHUT DOWN NETCOM

            DEBUG.DB_Status(msg: DEBUG.msg = "NetCom Disconnect", status: DEBUG.STATUS.RUNNING);
            if (!FC.do_SessionICR || !IsProcessOpen("Cheetah"))
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            else
            {
                /// Stop recording aquisition if GUI closed early
                if (FC.is_GUIclosed)
                {
                    DEBUG.DB_Warning("CS STOPPING NLX RECORDING AND ACQUISITION");
                    string reply = " ";
                    netcomClient.SendCommand("-StopRecording", ref reply);
                    netcomClient.SendCommand("-StopAcquisition", ref reply);
                }

                // Close NetCom sreams
                netcomClient.CloseStream(netcomAcqEntVT1);
                netcomClient.CloseStream(netcomAcqEntVT2);

                // Disconnect from NetCom
                do
                {
                    netcomClient.DisconnectFromServer();
                    // Pause loop
                    Thread.Sleep(10);
                }
                while (netcomClient.AreWeConnected() && !FC.do_AbortCS);

                // Check if disconnect succesful
                if (!netcomClient.AreWeConnected())
                {
                    FC.is_NlxConnected = false;
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SUCCEEDED);
                }
                else
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);
            }

            // WAIT FOR FEEDERDUE LOG SAVE TO COMPLETE

            DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue Log Import", status: DEBUG.STATUS.AWAITING);
            if (!FC.do_SessionICR || !sp_Xbee.IsOpen)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            else
            {
                while (!logger_feederDue.is_importFinished)
                    // Pause loop
                    Thread.Sleep(10);

                if (!logger_feederDue.is_imported)
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);

                // Save if importing completed
                DEBUG.DB_Status(msg: DEBUG.msg = "Save FeederDue Log", status: DEBUG.STATUS.RUNNING);
                if (!logger_feederDue.is_imported)
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

                else
                {
                    // Save log file
                    logger_feederDue.SaveLog(logTempDir, feederDueLogFi);

                    // Check if complete log was imported
                    if (logger_feederDue.is_logComplete)
                        DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SUCCEEDED);
                    else if (logger_feederDue.cnt_logsStored > 0)
                        DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.ABORTED);
                    else
                        DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);
                }

                // Print import summary
                DEBUG.DB_General(String.Format("FEEDERDUE LOGGING SUMMARY: logged={0} dropped={1} b_read={2} bytes_expected={3} dt_run={4}",
                        logger_feederDue.cnt_logsStored, logger_feederDue.cnt_dropped[1], logger_feederDue.bytesRead, logger_feederDue.bytesToRcv, logger_feederDue.logDT));
            }

            // Wait for robot to wrap up logging stuff
            Thread.Sleep(1000);

            // SEND/WAIT FOR FEEDERDUE QUIT COMMAND

            DEBUG.DB_Status(msg: DEBUG.msg = "Send FeederDue Quit Command", status: DEBUG.STATUS.RUNNING);
            if (!FC.do_SessionICR || !sp_Xbee.IsOpen)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            else
            {
                // Send FeederDue quit command
                RepeatSendFeederDueCom_Thread(id: 'Q', do_check_done: true);

                // Wait for quit confirmation from feederdue for fixed period of time
                DEBUG.DB_Status(msg: DEBUG.msg = "FeederDue Quit Confirmation", status: DEBUG.STATUS.AWAITING);
                status = WaitForFeederDueCom(id: 'Q', chk_send: true, chk_conf: true, chk_done: true, timeout: TIMEOUT_15);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            }

            // Set flags to end feederdue serial coms
            FC.is_FeederDueComActive = false;

            // WAIT FOR ICR_GUI TO SAVE

            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Save Confirmation", status: DEBUG.STATUS.AWAITING);
            if (FC.is_GUIquit || FC.is_GUIclosed)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            else
            {
                status = WaitForMatCom(id: 'F', chk_rcv: true, do_abort: true);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            }

            // SEND SAVE CONFIRMATION TO MATLAB
            DEBUG.DB_Status(msg: DEBUG.msg = "Send ICR_GUI Save Confirmation", status: DEBUG.STATUS.RUNNING);
            SendMatCom_Thread(id: 'F', dat1: 1);

            // WAIT FOR MATLAB QUIT COMMAND

            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Quit Command", status: DEBUG.STATUS.AWAITING);
            if (!FC.is_GUIquit && !FC.is_GUIclosed)
            {
                status = WaitForMatCom(id: 'X', chk_rcv: true, do_abort: true);
                DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            }
            else
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            // STORE NLX DIRECTORY INFO

            // Check if nlx dir created
            DEBUG.DB_Status(msg: DEBUG.msg = "Get NLX Recording Directory", status: DEBUG.STATUS.RUNNING);
            if (!FC.is_NlxDirCreated)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            else
            {
                // Get NLX rec sub directory
                dynamic nlx_rec_dir;
                lock (lock_matCom)
                    nlx_rec_dir = com_Matlab.GetVariable("m2c_rec_sub", "global");
                nlxRecSub = (string)nlx_rec_dir;

                // Print rec directories
                DEBUG.DB_General(String.Format("SET REC DIR \"{0}\"", nlxRecSub));

            }

            // Check if nlx data saved
            if (FC.is_NlxDataSaved)
            {

                // Get NLX rat sub directory
                dynamic nlx_rat_dir;
                lock (lock_matCom)
                    nlx_rat_dir = com_Matlab.GetVariable("m2c_rat_sub", "global");
                nlxRatSub = (string)nlx_rat_dir;

                // Print rat directories
                DEBUG.DB_General(String.Format("SET RAT DIR \"{0}\"", nlxRatSub));

            }

            // SEND COMMAND FOR MATLAB TO EXIT
            DEBUG.DB_Status(msg: DEBUG.msg = "Send ICR_GUI Exit Command", status: DEBUG.STATUS.RUNNING);
            SendMatCom_Thread(id: 'E', dat1: 1);

            // Wait for GUI to close

            DEBUG.DB_Status(msg: DEBUG.msg = "ICR_GUI Exit Confirmation.", status: DEBUG.STATUS.AWAITING);
            status = WaitForMatCom(id: 'C', chk_rcv: true, timeout: FC.is_MatComActive && !(FC.is_MatFailed || DEBUG.flag.is_debugRun) ? TIMEOUT_30 : TIMEOUT_5);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
            {
                // Set abort all
                FC.SetAbort(set_abort_cs: true, set_abort_mat: true, is_mat_failed: true);
            }

            // SEND CLOSE CONFIRM TO MATLAB
            DEBUG.DB_Status(msg: DEBUG.msg = "Send ICR_GUI Exit Confirmation", status: DEBUG.STATUS.AWAITING);
            SendMatCom_Thread(id: 'C', dat1: 1);
            status = WaitForMatCom(id: 'C', chk_send: true, timeout: FC.is_MatComActive && !FC.is_MatFailed ? TIMEOUT_15 : TIMEOUT_5);
            DEBUG.DB_Status(msg: DEBUG.msg, status: status);
            if (status.ToString() != "SUCCEEDED")
            {
                // Set error flag
                FC.SetAbort(set_abort_mat: true);
            }

            // Pause for any remaining CheetahDue data
            Thread.Sleep(100);

            // SAVE CHEETAH DUE LOG FILE
            DEBUG.DB_Status(msg: DEBUG.msg = "Save CheetahDue Log", status: DEBUG.STATUS.RUNNING);
            if (!FC.do_SessionICR || !sp_Xbee.IsOpen)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SKIPPED);

            else
            {
                // Save logs
                logger_cheetahDue.SaveLog(logTempDir, cheetahDueLogFi);

                // Check if complete log was imported
                if (logger_cheetahDue.is_logComplete)
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SUCCEEDED);
                else if (logger_cheetahDue.cnt_logsStored > 0)
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.ABORTED);
                else
                    DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);

                // Print logging summary
                DEBUG.DB_General(String.Format("CHEETAHDUE LOGGING SUMMARY: logged={0} dropped={1}",
                        logger_cheetahDue.cnt_logsStored, logger_cheetahDue.cnt_dropped[1]));

            }

            // Set flags to end cheetahdue serial coms
            FC.is_CheetahDueComActive = false;

            // Set exit flag to exit all threads
            DEBUG.DB_Status(msg: DEBUG.msg = "SET EXIT FLAG", status: DEBUG.STATUS.RUNNING);
            FC.do_Exit = true;
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);

            // Pause for Matlab and threads to close down
            Thread.Sleep(1000);

            // CLOSE DOWN BACKROUND WORKERS

            // Close bw_RunGUI
            DEBUG.DB_Status(msg: DEBUG.msg = "Dispose RunGUI Worker", status: DEBUG.STATUS.RUNNING);
            bw_RunGUI.Dispose();
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);
            // Close bw_MatCOM
            DEBUG.DB_Status(msg: DEBUG.msg = "Dispose MatCOM Worker", status: DEBUG.STATUS.RUNNING);
            bw_MatCOM.Dispose();
            DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);

            // CLEAR ALL MATLAB VARS
            string str_mat_msg = "clearvars -global; close all;";
            if (!FC.is_MatFailed)
            {
                DEBUG.DB_General(String.Format("RUNNING: Clear MatCom Globals: msg=\"{0}\"", str_mat_msg));
                com_Matlab.Execute(str_mat_msg);
            }
            else
                DEBUG.DB_Warning(String.Format("ABORTED: Clear MatCom Globals: msg=\"{0}\"", str_mat_msg));

            // HOLD FOR ERRRORS

            if (FC.is_RunErrors)
            {
                // Show Matlab window
                if (!FC.is_MatFailed)
                    com_Matlab.Visible = 1;

                // Pause to let printing finish
                Thread.Sleep(1000);

                lock (lock_console)
                    Console.Write(
                        "\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" +
                        "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PAUSED FOR ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

                // Print all errors
                for (int i = 0; i < DEBUG.cnt_err; i++)
                {
                    lock (lock_console)
                        Console.WriteLine(DEBUG.err_list[i]);
                }

                lock (lock_console)
                    Console.Write(
                   "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PRESS ANY KEY TO EXIT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" +
                   "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");

                // Wait for key press
                while (!Console.KeyAvailable)
                {
                    // Pause loop
                    Thread.Sleep(10);
                }

            }

            // QUIT MATCOM

            if (!FC.is_MatFailed)
            {
                DEBUG.DB_Status(msg: DEBUG.msg = "Close MatCOM", status: DEBUG.STATUS.RUNNING);
                Thread.Sleep(100);
                com_Matlab.Quit();
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FINISHED);
            }

            // FORCE KILL MATLAB

            else
            {
                DEBUG.DB_Warning("FORCE KILLING MATLAB");
                string kill_mat_status = KillMatlabProcess();
                DEBUG.DB_General(String.Format("FORCE KILLED MATLAB: {0}", kill_mat_status));
            }

            // LOG/PRINT RUN SUMMARY

            // Com summary info 1
            DEBUG.DB_General(msg: String.Format(
                "COM SUMMARY FEEDERDUE: C2R=|pind={0}|psent={1}|resnd={2}| R2C=|pind={3}|psent={4}|prcvd={5}|rercv={6}|drop={7}|",
                c2r.packInd, c2r.packSentAll, c2r.cnt_repeat,
                r2c.packInd, r2c.packSentAll, r2c.packRcvdAll, r2c.cnt_repeat, r2c.cnt_dropped));

            // Com summary info 2
            DEBUG.DB_General(msg: String.Format(
                "COM SUMMARY MATLAB: C2M=|pind={0}|psent={1}|resnd={2}|  M2C=|pind={3}|psent={4}|prcvd={5}|rercv={6}|drop={7}|",
                c2m.packInd, c2m.packSentAll, c2m.cnt_repeat,
                m2c.packInd, m2c.packSentAll, m2c.packRcvdAll, m2c.cnt_repeat, m2c.cnt_dropped));

            // Error and warning summary
            DEBUG.DB_General(msg: DEBUG.GetErrWarnSummary("warnings"));
            DEBUG.DB_General(msg: DEBUG.GetErrWarnSummary("errors"));

            // SAVE CS LOG FILE

            DEBUG.DB_Status(msg: DEBUG.msg = "Save CS Log", status: DEBUG.STATUS.RUNNING);
            DEBUG.logger.SaveLog(logTempDir, csLogFi);

            // Check if complete log was imported
            if (DEBUG.logger.is_logComplete)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.SUCCEEDED);
            else if (DEBUG.logger.cnt_logsStored > 0)
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.ABORTED);
            else
                DEBUG.DB_Status(msg: DEBUG.msg, status: DEBUG.STATUS.FAILED);

            // Print logging summary
            DEBUG.DB_General(String.Format("CS LOGGING SUMMARY: logged={0}", logger_cheetahDue.cnt_logsStored));

            // COPY LOG FILES TO TEMPORARY REC DIRECTORY
            if (FC.is_NlxDirCreated)
                // Copy all files
                CopyLogFiles(System.IO.Path.Combine(nlxTempTop, nlxRecSub));

            // COPY LOG FILES TO SAVE REC DIRECTORY
            if (FC.is_NlxDataSaved)
                // Copy all files
                CopyLogFiles(System.IO.Path.Combine(nlxSaveTop, nlxRatSub, nlxRecSub));

        }

        #endregion

        #region ========= SERIAL COMMUNICATION ==========

        // SEND PACK DATA REPEATEDLY TILL RECIEVED CONFIRMED
        public static void RepeatSendFeederDueCom_Thread(int send_max = 0, char id = ' ', double dat1 = double.NaN, double dat2 = double.NaN, double dat3 = double.NaN, UInt16 pack = 0, bool do_conf = true, bool is_conf = false, bool do_check_done = false)
        {
            // Local vars
            double[] dat = new double[3] { dat1, dat2, dat3 };

            // Reset check flags 
            if (!is_conf)
                c2r.ResetComFlags(id: id, do_print: id != 'P');

            // Run method on seperate thread
            new Thread(delegate ()
            {
                RepeatSendFeederDueCom(send_max: send_max, id: id, dat: dat, pack: pack, do_conf: do_conf, is_conf: is_conf, do_check_done: do_check_done);
            }).Start();

        }
        public static void RepeatSendFeederDueCom(int send_max, char id, double[] dat, UInt16 pack, bool do_conf, bool is_conf, bool do_check_done)
        {
            long t_resend = DEBUG.DT() + c2r.dt_resend;
            int send_count = 1;

            // Specify max send attempts
            send_max = send_max == 0 ? c2r.resendMax : send_max;

            // Send new data with new packet number
            pack = SendFeederDueCom(id: id, dat: dat, pack: pack, do_conf: do_conf, is_conf: is_conf, do_check_done: do_check_done);

            // Keep checking mesage was received
            while (FC.ContinueFeederDueCom())
            {

                // Bail if not checking for confirmation or confirmation recieved
                if (!do_conf || r2c.GetComFlags(id: id, get_is_sent_rcvd: true))
                {
                    // Bail
                    return;
                }

                // Need to resend
                else if (DEBUG.DT() > t_resend)
                {
                    // Log
                    DEBUG.DB_Warning_Thread(String.Format("Resending c2r: cnt={0} id='{1}' dat=|{2:0.00}|{3:0.00}|{4:0.00}| pack={5} do_conf={6} is_conf={7} do_check_done={8}",
                        send_count, id, dat[0], dat[1], dat[2], pack, do_conf, is_conf, do_check_done));

                    // Resend with same packet number
                    SendFeederDueCom(id: id, dat: dat, pack: pack, do_conf: do_conf, is_conf: is_conf, do_check_done: do_check_done, is_resend: true);
                    t_resend =
                        send_count > 5 ? DEBUG.DT() + c2r.dt_resend * 2 :
                        send_count > 10 ? DEBUG.DT() + c2r.dt_resend * 3 :
                        send_count > 15 ? DEBUG.DT() + c2r.dt_resend * 4 :
                        DEBUG.DT() + c2r.dt_resend;
                    send_count++;
                }

                // Check if coms have failed
                else if (send_count >= c2r.resendMax)
                {
                    // Log
                    DEBUG.DB_Error_Thread(String.Format("ABBORTED: Resending c2r: cnt={0} id='{1}' dat=|{2:0.00}|{3:0.00}|{4:0.00}| pack={5} do_conf={6} is_conf={7} do_check_done={8}",
                        send_count, id, dat[0], dat[1], dat[2], pack, do_conf, is_conf, do_check_done));

                    // Set error flags
                    FC.SetAbort(set_abort_mat: true);

                    // Bail
                    return;
                }

                else
                {
                    // Pause loop
                    Thread.Sleep(10);
                }

            }
        }

        // SEND PACK DATA
        public static UInt16 SendFeederDueCom(char id, double[] dat, UInt16 pack, bool do_conf, bool is_conf, bool do_check_done, bool is_resend = false)
        {
            /* 
            SEND DATA TO FEEDERDUE 
                FORMAT: [0]head, [1]id, [2:5]dat1, [6:9]dat2, [10:13]dat2, [14:15]pack, [16]flag_byte, [17]footer
                EXAMPLE: ASCII {'<','L','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','ÿ','\0','>','c'} DEC {60,76,1,255,255,0,60}
                "flag_byte" = [0, 0, 0, 0, is_resend, is_done, is_conf, do_conf]
            */

            // Local vars
            UNION_HACK U = new UNION_HACK(0, '0', 0, 0, 0);
            byte[] msg_head = new byte[1];
            byte[] msg_foot = new byte[1];
            byte[] msg_id = new byte[1];
            byte[] msg_data = new byte[12];
            byte[] msg_pack = new byte[2];
            byte[] msg_conf = new byte[1];
            bool do_loop = true;
            bool is_repeat = false;
            bool is_min_dt_send_rcv = false;
            bool is_buff_ready = false;
            bool is_hanging = false;
            string buff_dat_0 = "";
            string buff_dat_1 = "";
            string buff_dat_2 = "";
            long dt_rcvd = 0;
            long dt_queued = 0;
            byte flag_byte = 0;

            // Set flag byte
            GetSetByteBit(ref flag_byte, 0, do_conf);
            GetSetByteBit(ref flag_byte, 1, is_conf);
            GetSetByteBit(ref flag_byte, 3, is_resend);

            // Track when data queued
            long t_queued = DEBUG.DT();

            // Get message size
            int msg_size =
                msg_head.Length +
                msg_id.Length +
                msg_data.Length +
                msg_pack.Length +
                msg_conf.Length +
                msg_foot.Length;

            // Initialize byte array
            byte[] msgByteArr = new byte[msg_size];

            // Format queued data
            buff_dat_0 = String.Format("'{0}': dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} do_conf={5} is_conf={6} do_check_done={7} is_resend={8}",
                id, dat[0], dat[1], dat[2], pack, do_conf, is_conf, do_check_done, is_resend);

            // Log packet queued
            if (id != 'P')
                DEBUG.DB_General_Thread(buff_dat_0, msg_type: "COM", str_prfx: "[SEND-QUEUED:c2r]", indent: 5);

            lock (lock_sendPack)
            {
                // Block vt sending
                vt.Block(id);

                // Get new packet number
                pack = pack == 0 ? c2r.PackIncriment(1) : pack;

                // Format data string
                buff_dat_1 = String.Format("'{0}': dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} flag_byte={5} do_check_done={6}",
                    id, dat[0], dat[1], dat[2], pack, DEBUG.FormatBinary(flag_byte), do_check_done);

                // Wait for next safe send time
                while (do_loop)
                {
                    // Delay send time till x ms after last send or rcvd
                    is_min_dt_send_rcv =
                        c2r.DT_SentRcvd(t2: DEBUG.DT()) < c2r.dt_minSentRcvd &&
                        r2c.DT_SentRcvd(t2: DEBUG.DT()) < r2c.dt_minSentRcvd;

                    // Make sure outbut and input buffer have enough space
                    is_buff_ready = sp_Xbee.BytesToWrite == 0 && sp_Xbee.BytesToRead == 0;

                    // Check if loop should continue
                    do_loop =
                        (is_min_dt_send_rcv || !is_buff_ready) &&
                        FC.ContinueFeederDueCom();

                    // Get status
                    is_hanging = DEBUG.DT() > t_queued + 100;

                    // Abort if sending pos data
                    if (is_hanging && id == 'P')
                        break;

                    // Pause loop
                    Thread.Sleep(1);

                }

                // Get dt info
                dt_rcvd = r2c.DT_SentRcvd(DEBUG.DT());
                dt_queued = DEBUG.DT(ts1: t_queued);

                // Format data string
                buff_dat_2 = String.Format("b_sent={0} tx={1} rx={2} dt(snd|rcv|q)=|{3}|{4}|{5}|",
                    msg_size, sp_Xbee.BytesToWrite, sp_Xbee.BytesToRead, c2r.DT_SentRcvd(), dt_rcvd, dt_queued);

                // Check if queue backed up or hanging
                if (is_hanging)
                {

                    // Log
                    DEBUG.DB_Warning_Thread(String.Format("c2r Queue HANGING: {0} {1}", buff_dat_1, buff_dat_2));

                    // Bail for pos data if recently updated
                    if (id == 'P')
                    {
                        if (vt.GetSendDT((int)dat[0], "now") < 150)
                        {
                            // Decriment packet back
                            pack = c2r.PackIncriment(-1);
                            return pack;
                        }
                    }
                }

                // Store head and foot
                msg_head[0] = c2r.head;
                msg_foot[0] = c2r.foot;

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

                // Store flag_byte 
                msg_conf[0] = flag_byte;

                // Add header
                msg_head.CopyTo(msgByteArr, 0);
                // Add id
                msg_id.CopyTo(msgByteArr, msg_head.Length);
                // Add data
                msg_data.CopyTo(msgByteArr, msg_head.Length + msg_id.Length);
                // Add packet number
                msg_pack.CopyTo(msgByteArr, msg_head.Length + msg_id.Length + msg_data.Length);
                // Add flag_byte
                msg_conf.CopyTo(msgByteArr, msg_head.Length + msg_id.Length + msg_data.Length + msg_pack.Length);
                // Add footer
                msg_foot.CopyTo(msgByteArr, msg_head.Length + msg_id.Length + msg_data.Length + msg_pack.Length + msg_conf.Length);

                // Send to arduino
                sp_Xbee.Write(msgByteArr, 0, msgByteArr.Length);

                // Update c2r info
                var result = c2r.UpdateSent(id: id, dat: dat, pack: pack, ts: DEBUG.DT(), is_conf: is_conf, is_resend: is_resend);
                string str_prfx = result.Item1;
                is_repeat = result.Item2;

                // Check for vt data
                if (id != 'P')
                {
                    // Log send info
                    string str_print = String.Format("{0} {1}", buff_dat_1, buff_dat_2);
                    if (!is_repeat && !is_resend)
                        DEBUG.DB_General_Thread(str_print, ts: c2r.t_new, msg_type: "COM", str_prfx: str_prfx, indent: 5);
                    else
                        DEBUG.DB_Warning_Thread(str_print, ts: c2r.t_new, str_prfx: str_prfx);

                }

                // Update vt info
                else
                {
                    // Track send rate
                    vt.StoreSendTime((int)dat[0], DEBUG.DT());

                    // Log
                    if ((DEBUG.flag.do_printSentRatVT && (int)dat[0] == 0) ||
                        (DEBUG.flag.do_printSentRobVT && (int)dat[0] == 1))
                    {
                        U.f = (float)dat[2];
                        string str_print = String.Format("{0} {1} dt_send_mu={2}", buff_dat_1, buff_dat_2, vt.GetSendDT((int)dat[0], "avg"));
                        DEBUG.DB_General_Thread(str_print, ts: c2r.t_new, msg_type: "COM", str_prfx: "[SENT-VT:c2r]", indent: 5);
                    }
                }

                // Update com check state
                c2r.SetComFlags(id: id, set_is_sent_rcvd: true, do_print: id != 'P');

                // Unlock vt sending
                vt.Unblock(id);

            }

            // Return packet number
            return pack;

        }

        // WAIT FOR R2C CONFIRMATION
        public static DEBUG.STATUS WaitForFeederDueCom(char id, double dat1 = double.NaN, bool chk_send = false, bool chk_rcv = false, bool chk_conf = false, bool chk_done = false, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_start = DEBUG.DT();
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : t_start + timeout;
            bool first_loop = true;
            string str_wait = " ";
            bool pass = false;

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

            // Format wait for string
            str_wait = String.Format("'{0}' |{1}{2}{3}{4}{5}",
            id,
            check_4[0] ? wait_list[0] + "|" : "",
            check_4[1] ? wait_list[1] + "|" : "",
            check_4[2] ? wait_list[2] + "|" : "",
            check_4[3] ? wait_list[3] + "|" : "",
            check_4[4] ? wait_list[4] + "|" : "");

            // Wait for confirmation
            while (true)
            {

                // SENT
                if (chk_send)
                {
                    bool wait_other = false;
                    wait_4_now[0] = wait_4_now[0] ? wait_other || !c2r.GetComFlags(id: id, get_is_sent_rcvd: true) : wait_4_now[0];
                }

                // RECIEVED
                if (chk_rcv)
                {
                    bool wait_other = wait_4_now[0];
                    wait_4_now[1] = wait_4_now[1] ? wait_other || !r2c.GetComFlags(id: id, get_is_sent_rcvd: true) : wait_4_now[1];
                }

                // CONFIRMED
                if (chk_conf)
                {
                    bool wait_other = wait_4_now[0];
                    wait_4_now[2] = wait_4_now[2] ? wait_other || !c2r.GetComFlags(id: id, get_is_conf: true) : wait_4_now[2];
                }

                // DONE
                if (chk_done)
                {
                    bool wait_other = wait_4_now[0] || wait_4_now[1] || wait_4_now[2];
                    wait_4_now[3] = wait_4_now[3] ? wait_other || !c2r.GetComFlags(id: id, get_is_done: true) : wait_4_now[3];
                }

                // DATA
                if (chk_dat)
                {
                    // Make sure check for dat is last
                    bool wait_other = wait_4_now[0] || wait_4_now[1] || wait_4_now[2] || wait_4_now[3];
                    // Check for r2c dat match
                    if (chk_rcv)
                        wait_4_now[4] = wait_4_now[4] ? wait_other || r2c.ID_Dat(id, 0) != dat1 : wait_4_now[4];
                    // Check cor c2r dat dat match
                    else
                        wait_4_now[4] = wait_4_now[4] ? wait_other || c2r.ID_Dat(id, 0) != dat1 : wait_4_now[4];

                }

                // Get current status string
                long t_out = timeout == long.MaxValue ? 0 : timeout;
                long dt_wait = DEBUG.DT(ts1: t_start);
                string buff_dat = String.Format("chk_flags=|snd={0}|rcv={1}|cnf={2}|dn={3}|dat={4}| wait_flags=|snd={5}|rcv={6}|cnf={7}|dn={8}|dat={9}| do_abort={9} timeout={11} dt_wait={12}",
                         check_4[0] ? "T" : "F", check_4[1] ? "T" : "F", check_4[2] ? "T" : "F", check_4[3] ? "T" : "F", check_4[4] ? "T" : "F",
                         wait_4_now[0] ? "T" : "F", wait_4_now[1] ? "T" : "F", wait_4_now[2] ? "T" : "F", wait_4_now[3] ? "T" : "F", wait_4_now[4] ? "T" : "F",
                         do_abort, t_out, dt_wait);

                // Print what we are waiting on
                if (first_loop)
                {
                    DEBUG.DB_General(String.Format("Awaiting {0}...: {1}", str_wait, buff_dat));
                    first_loop = false;
                }

                // Check for changes
                for (int i = 0; i < wait_list.Length; i++)
                {
                    if (wait_4_now[i] != wait_4_last[i])
                    {
                        string str = wait_list[i];
                        DEBUG.DB_General(String.Format("Confirmed: '{0}' |{1}|", id, str), indent: 10);
                    }
                    wait_4_last[i] = wait_4_now[i];
                }

                // Check if all conditions confirmed
                pass = !wait_4_now[0] && !wait_4_now[1] && !wait_4_now[2] && !wait_4_now[3] && !wait_4_now[4];

                // Return success
                if (pass)
                {
                    DEBUG.DB_General(String.Format("Finished Await: {0}: {1}", str_wait, buff_dat));
                    return DEBUG.STATUS.SUCCEEDED;
                }

                // Check if need to abort
                else if (
                    (do_abort && FC.do_AbortCS) ||
                    !FC.ContinueFeederDueCom() ||
                    DEBUG.DT() > t_timeout
                    )
                {

                    // External forced abort
                    if (do_abort && FC.do_AbortCS)
                    {
                        DEBUG.DB_Warning_Thread(String.Format("ABORTED: FORCED ABORT: Await {0}: {1}", str_wait, buff_dat));
                        return DEBUG.STATUS.ABORTED;
                    }

                    // Internal error
                    else
                    {
                        // Set error flag
                        FC.SetAbort(set_abort_mat: true);

                        // Coms failed
                        if (!FC.ContinueFeederDueCom())
                        {
                            DEBUG.DB_Error_Thread(String.Format("FAILED: LOST COMS: Await {0}: {1}", str_wait, buff_dat));
                            return DEBUG.STATUS.FAILED;
                        }

                        // Timedout
                        else if (DEBUG.DT() > t_timeout)
                        {
                            DEBUG.DB_Error_Thread(String.Format("FAILED: TIMEDOUT: Await {0}: {1}", str_wait, buff_dat));
                            return DEBUG.STATUS.TIMEDOUT;
                        }
                    }
                }

                else
                {
                    // Pause loop
                    Thread.Sleep(10);
                }

            }
        }

        // PARSE RECIEVED XBEE DATA 
        public static void ParseFeederDueCom()
        {
            /* 
            RECIEVE DATA FROM FEEDERDUE 
            FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer
            */

            // Dump anything in buffers on first run
            sp_Xbee.DiscardInBuffer();
            sp_Xbee.DiscardOutBuffer();

            // Loop till all data read out
            while (!FC.do_Exit)
            {

                // Bail if no new data or processing feederdue log
                if (sp_Xbee.BytesToRead < 1 ||
                    logger_feederDue.is_streaming)
                {
                    // Pause loop
                    Thread.Sleep(1);
                    continue;
                }

                // Local vars
                UNION_HACK U = new UNION_HACK(0, '0', 0, 0, 0);
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
                byte flag_byte = 0;
                bool do_conf = false;
                bool is_conf = false;
                bool is_done = false;
                bool is_resend = false;
                char foot = ' ';

                // Store parse start time
                r2c.t_parseStart = DEBUG.DT();

                // Get header
                if (FeederDueBuffReady(1, r2c.t_parseStart, "head"))
                {
                    sp_Xbee.Read(head_bytes, 0, 1);
                    bytes_read += 1;
                    // Get header
                    U.b_0 = head_bytes[0];
                    U.b_1 = 0; // C# chars are 2 bytes
                    head = U.c_0;

                    // Check if for r2c head
                    if (head == r2c.head)
                        r2c_head_found = true;

                }

                // Find id and check message is intended for CS
                if (r2c_head_found)
                {

                    if (FeederDueBuffReady(1, r2c.t_parseStart, "id"))
                    {

                        sp_Xbee.Read(id_bytes, 0, 1);
                        bytes_read += 1;
                        // Get id
                        U.b_0 = id_bytes[0];
                        U.b_1 = 0;
                        id = U.c_0;

                        // Check for r2c id 
                        r2c_id_found = r2c.ID_Ind(id) != -1;

                    }
                }

                // Get data, packet number and do_conf flag
                if (r2c_head_found && r2c_id_found)
                {

                    // Get data
                    for (int i = 0; i < 3; i++)
                    {
                        if (FeederDueBuffReady(4, r2c.t_parseStart, String.Format("dat{0}", i + 1)))
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
                    if (FeederDueBuffReady(2, r2c.t_parseStart, "pack"))
                    {

                        // Read in data
                        sp_Xbee.Read(pack_bytes, 0, 2);
                        bytes_read += 2;
                        U.b_0 = pack_bytes[0];
                        U.b_1 = pack_bytes[1];
                        pack = U.i16_0;
                    }

                    // Get do confirm byte
                    if (FeederDueBuffReady(1, r2c.t_parseStart, "flag_byte"))
                    {
                        sp_Xbee.Read(conf_bytes, 0, 1);
                        bytes_read += 1;
                        // Get confirm flags
                        flag_byte = conf_bytes[0];
                        do_conf = GetSetByteBit(ref flag_byte, 0, false);
                        is_conf = GetSetByteBit(ref flag_byte, 1, false);
                        is_done = GetSetByteBit(ref flag_byte, 2, false);
                        is_resend = GetSetByteBit(ref flag_byte, 3, false);
                    }

                    // Find footer
                    if (FeederDueBuffReady(1, r2c.t_parseStart, "foot"))
                    {
                        // Read in data
                        sp_Xbee.Read(foot_bytes, 0, 1);
                        bytes_read += 1;

                        // Check footer
                        U.b_0 = foot_bytes[0];
                        U.b_1 = 0;
                        foot = U.c_0;

                        // Check for r2c foot
                        if (foot == r2c.foot)
                            r2c_foot_found = true;

                    }
                }

                // Get dt info
                long dt_rcvd = r2c.DT_SentRcvd(DEBUG.DT());
                long dt_parse = DEBUG.DT(ts1: r2c.t_parseStart);

                // Format data string
                string buff_dat_1 = String.Format("'{0}': dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} flag_byte={5}",
                        id, dat[0], dat[1], dat[2], pack, DEBUG.FormatBinary(flag_byte));
                string buff_dat_2 = String.Format("b_read={0} rx={1} tx={2} dt(snd|rcv|prs)=|{3}|{4}|{5}|",
                    bytes_read, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, c2r.DT_SentRcvd(), dt_rcvd, dt_parse);

                // Store r2c packet data
                if (r2c_foot_found)
                {
                    // Update message info
                    var result = r2c.UpdateRcvd(id: id, dat: dat, pack: pack, ts: r2c.t_parseStart, is_conf: is_conf, is_done: is_done, is_resend: is_resend);
                    string str_prfx = result.Item1;
                    bool is_repeat = result.Item2;
                    int dropped = result.Item3;

                    // Update check com flags
                    r2c.SetComFlags(id: id, set_is_sent_rcvd: true);
                    c2r.SetComFlags(id: id, set_is_conf: is_conf, set_is_done: is_done);

                    // Send recieve confirmation
                    if (do_conf)
                        RepeatSendFeederDueCom_Thread(send_max: 1, id: id, dat1: dat[0], dat2: dat[1], dat3: dat[2], pack: pack, do_conf: false, is_conf: true);

                    // Check if data should be relayed to Matlab
                    if (c2m.ID_Ind(id) != -1)
                        SendMatCom_Thread(id: id, dat1: dat[0], dat2: dat[1], dat3: dat[2]);

                    // Log missed packets
                    if (dropped > 0)
                    {
                        DEBUG.DB_Warning_Thread(String.Format("Missed r2c Packs: (cns|tot)=|{0}|{1}| {2} {3}",
                        dropped, r2c.cnt_dropped, buff_dat_1, buff_dat_2));
                    }

                    // Log rcvd details
                    string str_print = String.Format("{0} {1}", buff_dat_1, buff_dat_2);
                    if (!is_repeat && !is_resend)
                        DEBUG.DB_General_Thread(str_print, ts: r2c.t_new, msg_type: "COM", str_prfx: str_prfx, indent: 5);
                    else
                        DEBUG.DB_Warning_Thread(str_print, ts: r2c.t_new, str_prfx: str_prfx);

                }

                // If all data found restart loop
                if (r2c_head_found && r2c_id_found && r2c_foot_found)
                {
                    // Change com status
                    if (!FC.is_FeederDueComActive)
                        FC.is_FeederDueComActive = true;

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

                    // Get found flags info
                    string found = String.Format("|{0}|{1}|{2}|",
                        r2c_head_found ? "r2c_head" : "no_head",
                        r2c_id_found ? "r2c_id" : "no_id",
                        r2c_foot_found ? "r2c_foot" : "no_foot");

                    // Log available info
                    DEBUG.DB_Warning_Thread(String.Format("Dropped r2r Packs: cnt={0} found={1} {2} {3}",
                        r2c.cnt_dropped, found, buff_dat_1, buff_dat_2));
                }

            }

        }

        // WAIT FOR XBEE BUFFER TO FILL
        public static bool FeederDueBuffReady(int min_byte, long t_parse_start, string getting)
        {
            // Local vars
            string buff_dat = "";
            long t_start = DEBUG.DT();
            long t_timeout = t_parse_start + 1000;
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_Xbee.BytesToRead < min_byte &&
                DEBUG.DT() <= t_timeout &&
                FC.ContinueFeederDueCom()
                )
            {
                // Pause loop
                Thread.Sleep(1);
            }

            // Format data string
            buff_dat = String.Format("get=\"{0}\" min_byte={1} rx={2} tx={3} dt(chk|q)=|{4}|{5}|",
                     getting, min_byte, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, DEBUG.DT(ts1: t_start), DEBUG.DT(ts1: t_parse_start));

            // Check if hanging
            if (DEBUG.DT() > t_start + 25)
            {
                DEBUG.DB_Warning_Thread("XBee Read HANGING: " + buff_dat);
            }

            // Check if timedout
            if (DEBUG.DT() > t_timeout)
            {
                DEBUG.DB_Warning_Thread("TIMEDOUT: " + buff_dat);
            }

            // Check if buff filled
            pass = sp_Xbee.BytesToRead >= min_byte ? true : false;

            return pass;
        }

        // CONTINUALLY CHECK FOR NEW CHEETAHDUE LOG DATA
        public static void ParseCheetahDueCom()
        {

            // Dump anything in buffers on first run
            sp_cheetahDue.DiscardInBuffer();
            sp_cheetahDue.DiscardOutBuffer();

            // Loop till all data read out
            while (FC.ContinueCheetahDueCom())
            {
                // Local vars
                UNION_HACK U = new UNION_HACK(0, '0', 0, 0, 0);
                bool head_found = false;
                bool foot_found = false;
                byte[] head_bytes = new byte[1];
                byte[] foot_bytes = new byte[1];
                byte[] chksum_bytes = new byte[1];
                int bytes_read = 0;
                char head = ' ';
                UInt16 chksum = 0;
                char foot = ' ';
                string str_log = " ";

                // Wait for new header
                while (!head_found && FC.ContinueCheetahDueCom())
                {
                    // Keep waiting
                    if (!CheetahDueBuffReady(1, FC.ContinueCheetahDueCom() ? 500 : 100, do_print: false))
                    {
                        // Pause loop
                        Thread.Sleep(1);
                        continue;
                    }

                    // Read in data
                    sp_cheetahDue.Read(head_bytes, 0, 1);
                    bytes_read += 1;
                    // Get header
                    U.b_0 = head_bytes[0];
                    U.b_1 = 0; // C# chars are 2 bytes
                    head = U.c_0;
                    if (head == a2c.head)
                    {
                        head_found = true;
                    }
                    else
                    {
                        // Pause loop
                        Thread.Sleep(1);
                    }
                }

                // Find id and check message is intended for CS
                if (head_found)
                {
                    // Get check sum
                    if (CheetahDueBuffReady(1, FC.ContinueCheetahDueCom() ? 500 : 100))
                    {
                        // Read in data
                        sp_cheetahDue.Read(chksum_bytes, 0, 1);
                        bytes_read += 1;
                        chksum = chksum_bytes[0];
                    }

                    // Get complete message
                    byte[] log_bytes = new byte[chksum];
                    if (CheetahDueBuffReady(chksum, FC.ContinueCheetahDueCom() ? 1000 : 100))
                    {
                        // Read in all data
                        sp_cheetahDue.Read(log_bytes, 0, chksum);
                        bytes_read += chksum;

                    }
                    // Convert to string
                    str_log = System.Text.Encoding.UTF8.GetString(log_bytes);

                    // Find footer
                    if (CheetahDueBuffReady(1, FC.ContinueCheetahDueCom() ? 500 : 100))
                    {
                        // Read in data
                        sp_cheetahDue.Read(foot_bytes, 0, 1);
                        bytes_read += 1;

                        // Check footer
                        U.b_0 = foot_bytes[0];
                        U.b_1 = 0;
                        foot = U.c_0;
                        if (foot == a2c.foot)
                        {
                            foot_found = true;
                        }
                    }
                }

                if (head_found && foot_found)
                {
                    // Update list
                    logger_cheetahDue.UpdateLog(str_log);

                    // print data received
                    if (DEBUG.flag.do_printDueLog)
                    {
                        string str_prfx = String.Format("[LOG] a2c[{0}]", logger_cheetahDue.cnt_logsStored);
                        string str_print = String.Format("message=\"{0}\" chksum={1} b_read={2} rx={3} tx={4}", str_log, chksum, bytes_read, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite);
                        DEBUG.DB_General_Thread(str_print, msg_type: "COM", str_prfx: str_prfx, indent: 5);

                    }

                    // Change com status
                    if (!FC.is_CheetahDueComActive)
                    {
                        FC.is_CheetahDueComActive = true;
                    }
                }

                // Dump incomplete packets
                else if (bytes_read > 0)
                {
                    // Add to count
                    logger_cheetahDue.AddDropped(1);

                    // Print
                    DEBUG.DB_Warning_Thread(String.Format("Dropped a2c Log: logged={0} dropped={1}|{2} head={3} message=\"{4}\" chksum={5} foot={6} b_read={7} rx={8} tx={9}",
                       logger_cheetahDue.cnt_logsStored, logger_cheetahDue.cnt_dropped[0], logger_cheetahDue.cnt_dropped[1], head, str_log, chksum, foot, bytes_read, sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite));

                    // Dump buffer if > 1 consecutive drops and no bytes read
                    if (logger_cheetahDue.cnt_dropped[0] > 1 && bytes_read == 0)
                    {
                        DEBUG.DB_Warning_Thread("Dumping a2c Input Buffer");
                        sp_cheetahDue.DiscardInBuffer();
                    }
                }

            }
        }

        // WAIT FOR CHEETAHDUE BUFFER TO FILL
        public static bool CheetahDueBuffReady(int min_byte, long timeout, bool do_print = true)
        {
            // Local vars
            string buff_dat = "";
            long t_start = DEBUG.DT();
            long t_timeout = t_start + timeout;
            bool pass = false;

            // Wait for buffer to fill or time to ellapse
            while (
                sp_cheetahDue.BytesToRead < min_byte &&
                DEBUG.DT() <= t_timeout
                )
            {
                // Pause loop
                Thread.Sleep(1);
            }

            // Check if buff filled
            pass = sp_cheetahDue.BytesToRead >= min_byte ? true : false;

            // Check for errors
            if (!pass && do_print)
            {
                // Format data string
                buff_dat = String.Format(String.Format("a2c HANGING: rx={0} tx={1} dt_chk={2}",
                      sp_cheetahDue.BytesToRead, sp_cheetahDue.BytesToWrite, DEBUG.DT(ts1: t_start)));

                // Timedout
                if (DEBUG.DT() > t_timeout)
                    DEBUG.DB_Warning_Thread(String.Format("a2c HANGING: {0}", buff_dat));
                else
                    DEBUG.DB_Warning_Thread(String.Format("ABORTED: {0}", buff_dat));
            }

            return pass;
        }

        // RETRIEVE FEEDERDUE LOG
        public static void GetFeederDueLog(long timeout_stream, long timeout_read)
        {
            // Local vars
            long str_steam_start = DEBUG.DT();
            long str_steam_timeout = str_steam_start + timeout_stream;
            long dt_read_timeout = timeout_read;
            long t_read_last = str_steam_start;
            char[] c_arr = new char[3] { '\0', '\0', '\0' };

            // Start log import
            DEBUG.DB_General_Thread("Running: Import FeederDue Logs");

            // Read stream vars
            int read_ind = 0;
            char[] stream_arr = new char[1000000];
            long dt_run = 0;
            long dt_read = 0;
            bool is_timedout = false;
            bool is_stream_success = false;
            bool is_stream_warning = false;
            bool is_stream_abort = false;

            // Read stream till ">>>" string
            while (!is_timedout)
            {

                // Check for run or read timeout
                if (DEBUG.DT() > str_steam_timeout ||
                    DEBUG.DT() - t_read_last > dt_read_timeout)
                {
                    dt_run = DEBUG.DT(ts1: str_steam_start);
                    dt_read = t_read_last == 0 ? 0 : DEBUG.DT(ts1: t_read_last);
                    is_timedout = true;
                }

                // Get next byte
                if (sp_Xbee.BytesToRead < 1)
                {
                    // Pause loop
                    Thread.Sleep(1);
                    continue;
                }

                // Check progress
                string str_status = logger_feederDue.GetImportStatus(read_ind);
                if (str_status != " ")
                {
                    // Print progress on seperate thread
                    DEBUG.DB_General_Thread(String.Format("Log Import {0}", str_status));
                }

                // Get next char
                c_arr[0] = c_arr[1];
                c_arr[1] = c_arr[2];
                c_arr[2] = (char)sp_Xbee.ReadByte();
                stream_arr[read_ind] = c_arr[2];
                t_read_last = DEBUG.DT();

                // Incriment count
                read_ind++;

                // Check for end
                if (
                    c_arr[0] == '>' &&
                    c_arr[1] == '>'
                )
                {

                    // Check for success flag
                    if (c_arr[2] == '>')
                    {
                        // Print final status
                        DEBUG.DB_General_Thread(String.Format("Log Import {0}",
                            logger_feederDue.prcntPrintStr[logger_feederDue.prcntPrintStr.Length - 1]));

                        // Print success termination string received
                        DEBUG.DB_General_Thread(String.Format("Received Send Success Termination String: \"{0}{1}{2}\" rx={3} tx={4}",
                            c_arr[0], c_arr[1], c_arr[2], sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite));

                        // Set success flag and break
                        is_stream_success = true;
                        break;
                    }

                    // Check for warning flag
                    else if (c_arr[2] == '*')
                    {
                        // Print abort termination string received
                        DEBUG.DB_Warning_Thread(String.Format("Received Warning Termination String: \"{0}{1}{2}\" rx={3} tx={4}",
                            c_arr[0], c_arr[1], c_arr[2], sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite));

                        // Set success and warning flag and break
                        is_stream_success = true;
                        is_stream_warning = true;
                        break;
                    }

                    // Check for abort flag
                    else if (c_arr[2] == '!')
                    {
                        // Print abort termination string received
                        DEBUG.DB_Warning_Thread(String.Format("Received Abort Termination String: \"{0}{1}{2}\" rx={3} tx={4}",
                            c_arr[0], c_arr[1], c_arr[2], sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite));

                        // Set abort flag and break
                        is_stream_abort = true;
                        break;
                    }
                }
            }

            // Dump whatever is left in buffer
            sp_Xbee.DiscardInBuffer();

            // Unset importing flag
            logger_feederDue.is_streaming = false;

            // Store bytes read
            logger_feederDue.bytesRead = read_ind;

            // Store data string
            string buff_dat = String.Format("b_read={0} bytes_expected=~{1} rx={2} tx={3} str_dteam={4}",
                    logger_feederDue.bytesRead, logger_feederDue.bytesToRcv, sp_Xbee.BytesToRead, sp_Xbee.BytesToWrite, logger_feederDue.logDT);
            string str_err = String.Format("FeederDue Log Import: {0}: dt_read_last={1} dt_run={2} {3}",
                        is_timedout ? "Read Timedout" : is_stream_abort ? "FeederDue Aborted" : "Reason Unknown", dt_read, dt_run, buff_dat);

            // Check if logging timed out
            if (!is_stream_success)
            {
                // Bail if no bytes read
                if (logger_feederDue.bytesRead == 0)
                {
                    DEBUG.DB_Error_Thread(String.Format("FAILED: Import FeederDue Logs: {0}", str_err));
                    logger_feederDue.is_importTimedout = true;

                    // Bail
                    return;
                }
                else
                {
                    DEBUG.DB_Warning_Thread(String.Format("ABORTED: Import FeederDue Logs: {0}", str_err));
                }
            }
            else if (is_stream_warning)
            {
                DEBUG.DB_Warning_Thread(String.Format("Succeeded With Warnings: Import FeederDue Logs: {0}", buff_dat));
            }
            else
            {
                // Finished log import
                DEBUG.DB_General_Thread(String.Format("Succeeded: Import FeederDue Logs: {0}", buff_dat));
            }

            // Start log store
            if (logger_feederDue.bytesRead > 0)
            {
                DEBUG.DB_General_Thread("Running: Store FeederDue Logs");

                // Parse string and store logs
                char[] out_arr = new char[1000];
                int write_ind = 0;
                char c = '\0';
                bool do_log_num_store = false;
                char[] buff_int = new char[5] { '\0', '\0', '\0', '\0', '\0' };
                int cnt_byte = 0;
                int rec_now = 0;
                int rec_last = 0;
                for (int i = 0; i < logger_feederDue.bytesRead; i++)
                {

                    // Get next char
                    c = stream_arr[i];

                    // Check for count head
                    if (!do_log_num_store && write_ind == 0 && c == '[')
                    {
                        do_log_num_store = true;
                    }

                    // Check for count foot
                    else if (do_log_num_store && c == ']')
                    {
                        // Try to parse record number
                        try
                        {
                            rec_now = int.Parse(new string(buff_int, 0, cnt_byte));
                        }
                        catch
                        {
                            DEBUG.DB_Warning_Thread(String.Format("Failed to Parse r2c Log Number: rec_last={0}",
                                    rec_now));
                        }

                        // Reset flags
                        cnt_byte = 0;
                        do_log_num_store = false;
                    }

                    // Store new record
                    else if (do_log_num_store)
                    {
                        if (cnt_byte < 5)
                        {
                            buff_int[cnt_byte] = c;
                            cnt_byte++;
                        }
                        // Lost foot byte
                        else
                        {
                            rec_now = 0;
                            cnt_byte = 0;
                            do_log_num_store = false;
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

                        // Remove brackets around log number
                        if (new_log.IndexOf('[') >= 0)
                            new_log = new_log.Remove(new_log.IndexOf('['), 1);
                        if (new_log.IndexOf(']') >= 0)
                            new_log = new_log.Remove(new_log.IndexOf(']'), 1);

                        // Print current log
                        if (DEBUG.flag.do_printRobLog)
                        {
                            string str_prfx = String.Format("[LOG] r2c[{0}]", rec_now);
                            DEBUG.DB_General_Thread(String.Format("message=\"{0}\"", new_log), msg_type: "COM", str_prfx: str_prfx, indent: 5);
                        }

                        // Check for missed log
                        if (rec_now != rec_last + 1)
                        {
                            int dropped = rec_now - rec_last - 1;
                            logger_feederDue.AddDropped(dropped);
                            DEBUG.DB_Warning_Thread(String.Format("Dropped r2c Log: expected={0} stored={1} dropped={2}|{3}",
                                rec_now, logger_feederDue.cnt_logsStored, logger_feederDue.cnt_dropped[0], logger_feederDue.cnt_dropped[1]));
                        }

                        // Update list
                        logger_feederDue.UpdateLog(new_log);

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
                if (logger_feederDue.cnt_logsStored > 0)
                    DEBUG.DB_General_Thread(String.Format("Succeeded: Store FeederDue Logs: logs_stored={0} dt_run={1}",
                        logger_feederDue.cnt_logsStored, logger_feederDue.logDT));
                else
                    DEBUG.DB_Error_Thread("FAILED: Store FeederDue Logs");

            }
            else
                DEBUG.DB_Warning_Thread("SKIPPED: Store FeederDue Logs");

            // Set import finished flag
            logger_feederDue.is_imported = true;

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
            DEBUG.DB_General_Thread("Running: Setup Matlab paths");
            SendMatCom(msg: @"addpath(genpath('" + matStartDir + "'));");
            DEBUG.DB_General_Thread("Finished: Setup Matlab paths");

            // Run startup.m
            DEBUG.DB_General_Thread("Running: Run startup.m");
            lock (lock_matCom)
                com_Matlab.Feval("startup", 0, out startup_result);
            DEBUG.DB_General_Thread("Finished: Run startup.m");

            // Run ICR_GUI.m
            DEBUG.DB_General_Thread("Running: Run ICR_GUI.m");
            com_Matlab.Feval("ICR_GUI", 1, out icr_gui_result, DEBUG.flag.systemTest, DEBUG.flag.breakDebug, DEBUG.flag.do_autoloadUI);
            DEBUG.DB_General_Thread("Finished: Run ICR_GUI.m");

            // Get status
            object[] res = icr_gui_result as object[];
            status = res[0] as string;
            if (status == null)
                status = " ";

            // Print status
            if (status == "succeeded")
                DEBUG.DB_General_Thread("SUCCEEDED: ICR_GUI.m");
            else if (status != " ")
            {
                DEBUG.DB_Error_Thread(String.Format("FAILED: ICR_GUI.m Error: {0}", status));
                FC.SetAbort(set_abort_mat: true);
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
                DEBUG.DB_General_Thread("SUCCEEDED: RunGUI Worker");

            // Run failed
            else
            {
                // Run failed but errors were caught
                if (status != " ")
                    DEBUG.DB_Warning_Thread("ABORTED: RunGUI Worker");

                // Run failed completely
                else
                    DEBUG.DB_Error_Thread("FAILED WITHOUT CATCHING ERRORS: RunGUI Worker");

                // Flag Matlab has issues
                FC.SetAbort(is_mat_failed: true);
            }

            DEBUG.DB_General_Thread("EXITING: RunGUI Worker");
            FC.is_GUIclosed = true;
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

            DEBUG.DB_General_Thread("Running: MatCOM Worker");
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
            while (FC.ContinueMatCom())
            {

                // Pause loop
                if (dt_check < 5)
                {
                    Thread.Sleep((int)(5 - dt_check));
                }

                // Store check time
                dt_check = DEBUG.DT(ts1: t_check);

                // Check for abort flag
                if (FC.do_SendAbortMat)
                {
                    if (FC.do_SoftAbortMat && !FC.is_SoftAbortMat && !FC.is_HardAbortMat)
                    {
                        DEBUG.DB_Warning_Thread("SENDING MATLAB SOFT ABORT");
                        SendMatCom_Thread(id: 'E', dat1: 2);
                        FC.is_SoftAbortMat = true;
                    }
                    else if (FC.do_HardAbortMat && !FC.is_HardAbortMat)
                    {
                        DEBUG.DB_Warning_Thread("SENDING MATLAB HARD ABORT");
                        SendMatCom_Thread(id: 'E', dat1: 3);
                        FC.is_HardAbortMat = true;
                    }

                    // Reset flag so only read once
                    FC.do_SendAbortMat = false;
                }

                // Get global variable
                dynamic _m2c_pack;
                lock (lock_matCom)
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
                    if (!FC.is_MatComActive)
                        FC.is_MatComActive = true;

                    // Trigger progress change event
                    worker.ReportProgress(0, new System.Tuple<char, double, double, double, UInt16>(id, dat1, dat2, dat3, pack));

                    // Set pack and flag back to zero
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

            // Initialize flags
            bool do_block_relay = false;
            bool do_check_done = false;

            // Update com info
            var result = m2c.UpdateRcvd(id: id, dat: dat, pack: pack, ts: DEBUG.DT());
            string str_prfx = result.Item1;
            bool is_repeat = result.Item2;

            // Handle simulation data
            if (id == 'p' && DEBUG.flag.systemTest == 1)
            {
                // Store matlab position data
                UNION_HACK U = new UNION_HACK(0, '0', 0, 0, 0);
                UInt64 ts = (UInt64)(dat[0]) + vt.streamStart;
                double x = dat[1];
                double y = dat[2];

                // Run compute pos
                bool pass = CompPos(0, ts, x, y);

                // Convert pos data to double
                double dat1 = 0;
                double dat2 = vt.cm[0];
                U.i32 = vt.ts[0, 1];
                double dat3 = (double)U.f;

                // Send data
                if (pass)
                    RepeatSendFeederDueCom_Thread(send_max: 1, id: 'P', dat1: dat1, dat2: dat2, dat3: dat3, do_conf: false);

                // Bail to avoid printing
                return;
            }

            // Store commmon data
            string buff_dat = String.Format("'{0}': dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} dt_rcv={5}",
                    id, dat[0], dat[1], dat[2], pack, m2c.DT_SentRcvd());

            // Log info
            DEBUG.DB_General_Thread(buff_dat, ts: m2c.t_new, msg_type: "COM", str_prfx: str_prfx, indent: 5);

            // Process new packet info
            if (!is_repeat)
            {

                // Handle netcom initialized command
                if (id == 'i')
                {

                    // Store vt pixel parameters
                    vt.R = m2c.ID_Dat('i', 0);
                    vt.XC = m2c.ID_Dat('i', 1);
                    vt.YC = m2c.ID_Dat('i', 2);

                    // Print values
                    DEBUG.DB_General(String.Format("RECIEVED VT FOV INFO: R={0:0.00} XC={1:0.00} YC={2:0.00}", vt.R, vt.XC, vt.YC));

                }

                // Handle session type command
                else if (id == 'G')
                {

                    // Store session type parameters
                    FC.do_SessionICR = m2c.ID_Dat('G', 0) == 1;
                    FC.do_SessionTurnTT = m2c.ID_Dat('G', 1) == 1;
                    FC.do_SessionUpdateTable = m2c.ID_Dat('G', 2) == 1;

                    // Print session type
                    DEBUG.DB_General(String.Format("SESSION TYPE: \"{0}{1}{2}\"",
                        FC.do_SessionICR ? "ICR Session" : "", FC.do_SessionTurnTT ? "TT Turne" : "", FC.do_SessionUpdateTable ? "Update Table" : ""));

                }

                // Handle rat in command
                else if (id == 'I')
                {
                    if (dat[0] == 0)
                    {
                        DEBUG.DB_General_Thread("ICR_GUI Confirmed Rat On Forage Platform");
                    }
                    if (dat[0] == 1)
                    {
                        FC.is_RatOnTrack = true;
                        DEBUG.DB_General_Thread("ICR_GUI Confirmed Rat On Track");
                    }
                    FC.is_RatInArena = true;
                }

                // Handle ses saved command
                else if (id == 'F')
                {

                    // Set mat data saved flag
                    FC.is_MatDataSaved = true;
                    DEBUG.DB_General_Thread("ICR_GUI Confirmed Save Done");
                }

                // Check for task status command
                else if (id == 'O')
                {
                    FC.is_TaskDone = true;
                    DEBUG.DB_General_Thread("ICR_GUI Confirmed Task Done and Rat Out");
                }

                // Check for quit
                else if (id == 'X')
                {

                    // Check if nlx dir created 
                    if (dat[1] == 1)
                    {
                        FC.is_NlxDirCreated = true;
                        DEBUG.DB_General_Thread("ICR_GUI Confirmed NLX Dir Created");
                    }

                    // Check if nlx data saved
                    if (dat[2] == 1)
                    {
                        FC.is_NlxDataSaved = true;
                        DEBUG.DB_General_Thread("ICR_GUI Confirmed NLX Data Saved");
                    }

                    // Check if this is a premature quit
                    if (!FC.is_MatDataSaved)
                    {
                        // Start exiting early
                        FC.SetAbort(set_abort_cs: true);

                        // Print warning
                        if (!FC.is_GUIquit)
                            DEBUG.DB_Warning_Thread("ICR_GUI QUIT EARLY");
                    }

                    // Check if this is a forced quit
                    if (dat[0] == 2)
                    {
                        // Start exiting early
                        FC.SetAbort(set_abort_cs: true);

                        // Print error
                        DEBUG.DB_Warning_Thread("ICR_GUI FORCED QUIT");
                    }


                    // Set flag that GUI has quit
                    FC.is_GUIquit = true;
                }

                // Handle close confirmation
                else if (id == 'C')
                {
                    // Check if this is a premature close
                    if (!FC.is_GUIquit)
                    {
                        // Check for force close
                        if (dat[0] == 1)
                            DEBUG.DB_Warning_Thread("ICR_GUI FORCED CLOSE");
                        // Start exiting early
                        FC.SetAbort(set_abort_mat: true);
                    }
                }

                // Check if mesage should be relayed to rob
                if (c2r.ID_Ind(id) != -1)
                {

                    // Handle handshake command
                    if (id == 'h')
                    {
                        // Block initial handshake request
                        if (dat[0] == 0)
                            do_block_relay = true;
                    }

                    // Handle move to command
                    if (id == 'M')
                    {
                        // Calculate move to pos
                        dat[1] = CalcMove(dat[1]);

                        // Only check done on first and last move commands
                        if (dat[0] == 0 || dat[0] == 1)
                            do_check_done = true;
                    }

                    // Handle reward command 
                    else if (id == 'R')
                    {
                        // Calculate target pos for free or cue reward
                        if (dat[0] == 2 || dat[0] == 3)
                        {
                            dat[1] = CalcMove(dat[1]);
                        }
                    }

                    // Send data
                    if (!do_block_relay)
                        RepeatSendFeederDueCom_Thread(id: id, dat1: dat[0], dat2: dat[1], dat3: dat[2], do_check_done: do_check_done);

                }

            }

            // Reset check flag
            m2c.SetComFlags(id: id, set_is_sent_rcvd: true);

        }

        // RUNWORKERCOMPLETED FOR bw_MatCOM WORKER
        public static void RunWorkerCompleted_MatCOM(object sender, RunWorkerCompletedEventArgs e)
        {
            DEBUG.DB_General_Thread("EXITING: MatCOM Worker");
        }

        // SEND/STORE DATA FOR MATLAB
        public static void SendMatCom_Thread(string msg = " ", char id = ' ', double dat1 = 0, double dat2 = 0, double dat3 = 0, UInt16 pack = 0, bool do_print = true)
        {
            // Incriment packet
            if (id != ' ')
                pack = c2m.PackIncriment(1);

            // Run method on seperate thread
            new Thread(delegate ()
            {
                SendMatCom(msg: msg, id: id, dat1: dat1, dat2: dat2, dat3: dat3, pack: pack, do_print: do_print);
            }).Start();

        }
        public static void SendMatCom(string msg = " ", char id = ' ', double dat1 = 0, double dat2 = 0, double dat3 = 0, UInt16 pack = 0, bool do_print = true)
        {
            // Local vars
            string buff_dat = "";
            long t_queued = DEBUG.DT();

            // Sending packet data
            if (id != ' ')
            {

                if (FC.ContinueMatCom())
                {

                    // Format data string to print
                    buff_dat = String.Format("'{0}': dat=|{1:0.00}|{2:0.00}|{3:0.00}| pack={4} dt(snd|q)=|{5}|{6}|)",
                        id, dat1, dat2, dat3, pack, c2r.DT_SentRcvd(DEBUG.DT()), DEBUG.DT(ts1: t_queued));

                    // Log packet queued
                    if (do_print)
                        DEBUG.DB_General_Thread(buff_dat, msg_type: "COM", str_prfx: "[SEND-QUEUED:c2m]", indent: 5);

                    // Wait for flag to be reset
                    while (c2m.DT_SentRcvd(DEBUG.DT()) < c2m.dt_minSentRcvd ||
                        m2c.DT_SentRcvd(DEBUG.DT()) < m2c.dt_minSentRcvd)
                    {
                        Thread.Sleep(1);
                    }

                    // Create message string
                    msg = String.Format("[c2m_com.{0}.dat1, c2m_com.{0}.dat2, c2m_com.{0}.dat3, c2m_com.{0}.pack] =  deal({1}, {2}, {3}, {4});",
                            id, dat1, dat2, dat3, pack);

                    // Update Matlab variable
                    lock (lock_matCom)
                        com_Matlab.Execute(msg);

                    // Update com info
                    double[] dat = new double[3] { dat1, 0, 0 };
                    var result = c2m.UpdateSent(id: id, dat: dat, pack: pack, ts: DEBUG.DT());
                    string str_prfx = result.Item1;
                    c2m.SetComFlags(id: id, set_is_sent_rcvd: true);

                    // Log sent
                    if (do_print)
                        DEBUG.DB_General_Thread(buff_dat, msg_type: "COM", str_prfx: str_prfx, indent: 5);

                }

                // Log warning that packet was not sent
                else
                    DEBUG.DB_Warning_Thread(buff_dat, str_prfx: "[SEND:c2m]");

            }

            // Sending simple command
            else
            {

                // Format data string to print
                buff_dat = String.Format("msg=\"{0}\" dt_q={1}", msg, DEBUG.DT(ts1: t_queued));

                if (FC.ContinueMatCom())
                {

                    // Execute command
                    lock (lock_matCom)
                        com_Matlab.Execute(msg);

                    // Log message sent
                    if (do_print)
                        DEBUG.DB_General_Thread(buff_dat, msg_type: "COM", str_prfx: "[SEND-MESAGE:c2m]", indent: 5);

                }

                // Log warning that message was not sent
                else
                    DEBUG.DB_Warning_Thread(buff_dat, str_prfx: "[SEND-MESAGE:c2m]");

            }

        }

        // WAIT FOR M2C CONFIRMATION
        public static DEBUG.STATUS WaitForMatCom(char id, double dat1 = double.NaN, bool chk_send = false, bool chk_rcv = false, bool do_abort = false, long timeout = long.MaxValue)
        {
            // Local vars
            long t_start = DEBUG.DT();
            long t_timeout = timeout == long.MaxValue ? long.MaxValue : t_start + timeout;
            bool first_loop = true;
            bool pass = false;
            string str_wait = " ";

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

            // Format check for string
            str_wait = String.Format("'{0}' |{1}{2}{3}",
               id,
               check_4[0] ? wait_list[0] + "|" : "",
               check_4[1] ? wait_list[1] + "|" : "",
               check_4[2] ? wait_list[2] + "|" : "");

            while (true)
            {

                // SENT
                if (chk_send)
                {
                    bool wait_other = false;
                    wait_4_now[0] = wait_4_now[0] ? wait_other || !c2m.GetComFlags(id: id, get_is_sent_rcvd: true) : wait_4_now[0];
                }

                // RECIEVED
                if (chk_rcv)
                {
                    bool wait_other = wait_4_now[0];
                    wait_4_now[1] = wait_4_now[1] ? wait_other || !m2c.GetComFlags(id: id, get_is_sent_rcvd: true) : wait_4_now[1];
                }

                // DATA
                if (chk_dat)
                {
                    bool wait_other = wait_4_now[0] || wait_4_now[1];
                    wait_4_now[2] = wait_4_now[2] ? wait_other || m2c.ID_Dat(id, 0) != dat1 : wait_4_now[2];
                }

                // Get current status string
                long t_out = timeout == long.MaxValue ? 0 : timeout;
                long dt_wait = DEBUG.DT(ts1: t_start);
                string buff_dat = String.Format("chk_flags=|snd={0}|rcv={1}|dat={2}| wait_flags=|snd={3}|rcv={4}|dat={5}| do_abort={6} timeout={7} dt_wait={8}",
                         check_4[0] ? "T" : "F", check_4[1] ? "T" : "F", check_4[2] ? "T" : "F",
                         wait_4_now[0] ? "T" : "F", wait_4_now[1] ? "T" : "F", wait_4_now[2] ? "T" : "F",
                         do_abort, t_out, dt_wait);

                // Print what we are waiting on
                if (first_loop)
                {
                    DEBUG.DB_General(String.Format("Awaiting {0}...: {1}", str_wait, buff_dat));
                    first_loop = false;
                }

                // Check for changes
                for (int i = 0; i < wait_list.Length; i++)
                {
                    if (wait_4_now[i] != wait_4_last[i])
                    {
                        string str = wait_list[i];
                        DEBUG.DB_General(String.Format("Confirmed: '{0}' |{1}|", id, str), indent: 10);
                    }
                    wait_4_last[i] = wait_4_now[i];
                }

                // Check if done
                pass = !wait_4_now[0] && !wait_4_now[1] && !wait_4_now[2];

                // Return success
                if (pass)
                {
                    DEBUG.DB_General(String.Format("Finished Awaiting {0}: {1}", str_wait, buff_dat));
                    return DEBUG.STATUS.SUCCEEDED;
                }

                // Check if need to abort
                else if (
                    (do_abort && FC.do_AbortCS) ||
                    !FC.ContinueMatCom() ||
                    (DEBUG.DT() > t_timeout)
                    )
                {

                    // External forced abort
                    if (do_abort && FC.do_AbortCS)
                    {
                        DEBUG.DB_Warning_Thread(String.Format("ABORTED: FORCED ABORT: Await {0}: {1}", str_wait, buff_dat));
                        return DEBUG.STATUS.ABORTED;
                    }
                    else
                    {

                        // Set MATLAB failed if first packet
                        if (m2c.packSentAll == 0)
                            FC.SetAbort(set_abort_cs: true, is_mat_failed: true);

                        // Set error flag
                        FC.SetAbort(set_abort_mat: true);

                        // Coms failed
                        if (!FC.ContinueMatCom())
                        {
                            DEBUG.DB_Error_Thread(String.Format("FAILED: LOST COMS: Await {0}: {1}", str_wait, buff_dat));
                            return DEBUG.STATUS.FAILED;
                        }

                        // Timedout
                        else
                        {
                            DEBUG.DB_Error_Thread(String.Format("FAILED: TIMEDOUT: Await {0}: {1}", str_wait, buff_dat));
                            return DEBUG.STATUS.TIMEDOUT;
                        }

                    }
                }

                else
                {
                    // Pause loop
                    Thread.Sleep(10);
                }
            }

        }

        // KILL ANY ONGOING MATLAB EXE PROCESS
        public static string KillMatlabProcess()
        {
            // Get Matlab process
            Process[] proc = Process.GetProcessesByName("MATLAB");

            // Find newest MATLAB instance
            int ind = 0;
            if (proc.Length > 1)
            {
                // Store start times
                long[] start_time = new long[proc.Length];
                for (int i = 0; i < proc.Length; i++)
                {
                    start_time[i] = proc[i].StartTime.Ticks;
                }

                // Find newest instance
                long max = start_time[0];
                for (int i = 1; i < proc.Length; i++)
                {
                    ind = start_time[i] > max ? i : ind;
                }
            }

            // Kill newest instance
            if (proc.Length > 0)
            {
                proc[ind].Kill();
                ProcessStartInfo startInfo = new ProcessStartInfo();
                return String.Format("KILLED MATLAB PROCESSES {0} OF {1}", ind + 1, proc.Length);
            }
            else
                return "NO MATLAB PROCESSES ACTIVE";

            // May have an issue with the following too:
            // Microsoft.VsHub.Server.HttpHost.exe
            // Microsoft.VsHub.Server.HttpHostx64.exe
            // wmic process get name.creationdate
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
            startInfo.Arguments = string.Format("\"C:\\Users\\Public\\Documents\\Cheetah\\configuration\\{0}\"&", config);
            startInfo.WindowStyle = ProcessWindowStyle.Minimized;
            Process.Start(startInfo);
            Directory.SetCurrentDirectory(nowDir);
        }

        // CALLBACK FOR NETCOM STREAMING
        public static void NetComCallbackVT(object sender, MNetCom.MVideoRec records, int numRecords, string objectName)
        {
            // Local vars
            UNION_HACK U = new UNION_HACK(0, '0', 0, 0, 0);
            UInt16 ent = records.swid;
            double x = records.dnextracted_x;
            double y = records.dnextracted_y;
            UInt64 ts = records.qwTimeStamp;
            bool skip = false;

            // Skip if being blocked or sent in last 30 ms
            skip = vt.CheckBlock(ent) || vt.GetSendDT(ent, "now") < 30;

            // Send pos data
            if (!skip)
            {

                // Send data
                if (CompPos(ent, ts, x, y))
                {
                    // Convert pos data to double
                    double dat1 = ent;
                    double dat2 = vt.cm[ent];
                    U.i32 = vt.ts[ent, 1];
                    double dat3 = (double)U.f;

                    // Send vt data to feederdue
                    RepeatSendFeederDueCom_Thread(send_max: 1, id: 'P', dat1: dat1, dat2: dat2, dat3: dat3, do_conf: false);

                    // Log first record received
                    if (!vt.is_streamStarted[ent])
                    {
                        DEBUG.DB_General_Thread(String.Format("FIRST {0} VT RECORD", ent == 0 ? "RAT" : "ROBOT"));
                        vt.is_streamStarted[ent] = true;
                    }
                }

            }
            // Log print skipped
            else if (DEBUG.flag.do_printSkippedVT)
            {
                DEBUG.DB_Warning_Thread(String.Format("VT Skpped: ent={0} cnt={1} dt_snd={2}|{3}|{4}",
                    ent, vt.cnt_block[ent], vt.GetSendDT(ent), vt.GetSendDT(ent, "avg"), vt.GetSendDT(ent, "now")));
            }
        }

        #endregion

        #region ========= MOVEMENT AND TRACKING =========

        // COMPUTE POS IN RAD FOR VT DATA
        public static bool CompPos(UInt16 ent, UInt64 ts, double x, double y)
        {

            // Get first vtTS once
            if (vt.streamStart == 0)
            {
                // Save first ts and bail
                vt.streamStart = ts;
                return false;
            }

            // Convert ts from us to ms and subtract firts record ts
            UInt64 ts_last = vt.ts[ent, 1];
            UInt32 ts_now = (UInt32)Math.Round((double)((ts - vt.streamStart) / 1000));

            // Rescale y as VT data is compressed in y axis
            y = y * 1.0976;

            // Normalize 
            x = (x - vt.XC) / vt.R;
            y = (y - vt.YC) / vt.R;

            // Flip y 
            y = y * -1;

            // Compute radians
            double rad_last = vt.rad[ent, 1];
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
            x = Math.Round(x * vt.R) + vt.R;
            y = Math.Round(y * vt.R) + vt.R;

            // Check for negative dt
            if (dt < 0)
            {
                DEBUG.DB_Warning_Thread(String.Format("Strange TS Values: ent={0} ts_now={1} ts_last={2} dt={3}",
                    ent, ts_now, ts_last, dt));
            }

            // Convert cart to cm
            x = x * (140 / (vt.R * 2));
            y = y * (140 / (vt.R * 2));

            // Convert rad to cm
            double radFlip = Math.Abs(rad_now - (2 * Math.PI)); // flip
            double cm = radFlip * ((140 * Math.PI) / (2 * Math.PI)); // convert

            // Update vars
            // Save old vals
            vt.ts[ent, 0] = vt.ts[ent, 1];
            vt.rad[ent, 0] = rad_last;
            // New vals
            vt.ts[ent, 1] = ts_now;
            vt.rad[ent, 1] = rad_now;
            vt.cm[ent] = cm;

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

        // COPY LOG FILES
        public static void CopyLogFiles(string copy_dir)
        {

            // Create copies
            if (FC.do_SessionICR)
            {
                // Copy feederDueLogFi
                if (System.IO.File.Exists(System.IO.Path.Combine(logTempDir, feederDueLogFi)))
                {
                    System.IO.File.Copy(System.IO.Path.Combine(logTempDir, feederDueLogFi), System.IO.Path.Combine(copy_dir, feederDueLogFi), true);
                    DEBUG.DB_General(String.Format("Finished: COPY LOG \"{0}\" TO \"{1}\"", feederDueLogFi, copy_dir));
                }
                else
                    DEBUG.DB_Warning_Thread(String.Format("SKIPPED: COPY LOG \"{0}\" TO \"{1}\"", feederDueLogFi, copy_dir));

                // Copy cheetahDueLogFi
                if (System.IO.File.Exists(System.IO.Path.Combine(logTempDir, cheetahDueLogFi)))
                {
                    System.IO.File.Copy(System.IO.Path.Combine(logTempDir, cheetahDueLogFi), System.IO.Path.Combine(copy_dir, cheetahDueLogFi), true);
                    DEBUG.DB_General(String.Format("Finished: COPY LOG \"{0}\" TO \"{1}\"", cheetahDueLogFi, copy_dir));
                }
                else
                    DEBUG.DB_Warning_Thread(String.Format("SKIPPED: COPY LOG \"{0}\" TO \"{1}\"", cheetahDueLogFi, copy_dir));
            }

            // Copy csLogFi
            if (System.IO.File.Exists(System.IO.Path.Combine(logTempDir, csLogFi)))
            {
                System.IO.File.Copy(System.IO.Path.Combine(logTempDir, csLogFi), System.IO.Path.Combine(copy_dir, csLogFi), true);
                DEBUG.DB_General(String.Format("Finished: COPY LOG \"{0}\" TO \"{1}\"", csLogFi, copy_dir));
            }
            else
                DEBUG.DB_Warning_Thread(String.Format("SKIPPED: COPY LOG \"{0}\" TO \"{1}\"", csLogFi, copy_dir));

            // Copy matLogFi
            if (System.IO.File.Exists(System.IO.Path.Combine(logTempDir, matLogFi)))
            {
                System.IO.File.Copy(System.IO.Path.Combine(logTempDir, matLogFi), System.IO.Path.Combine(copy_dir, matLogFi), true);
                DEBUG.DB_General(String.Format("Finished: COPY LOG \"{0}\" TO \"{1}\"", matLogFi, copy_dir));
            }
            else
                DEBUG.DB_Warning_Thread(String.Format("SKIPPED: COPY LOG \"{0}\" TO \"{1}\"", matLogFi, copy_dir));

        }

        // GET/SET BYTE BIT VALUE
        public static bool GetSetByteBit(ref byte b_set, int bit, bool do_set)
        {
            // Local vars
            bool is_set = false;
            int b_ind = 0;
            byte mask = 0;

            // Get bit ind and mask
            b_ind = bit % 8;
            mask = (byte)(1 << b_ind);

            // Get state
            is_set = (b_set & mask) != 0;

            // Set value
            if (do_set)
            {
                b_set |= mask;
                is_set = true;
            }

            // Return state
            return is_set;

        }

        #endregion

    }

    #region ======= TOP LEVEL CLASSES/STRUCTS/ETC =======

    // CLASS TO TRACK PROGRAM FLAGS
    public class DEBUG
    {

        // PRIVATE VARS
        private static ICR_Run _ICR_Run = new ICR_Run();
        private static Stopwatch _stop_watch = new Stopwatch();
        private static int max_err_warn = 1000;
        private static long[] _warn_line = new long[max_err_warn];
        private static long[] _err_line = new long[max_err_warn];

        // PUBLIC VARS
        public static LOGGER logger = new LOGGER(header: "LOG, TS (ms), TS (string),  TYPE, MESSAGE");
        public static readonly object lock_console = new object();
        public static DB_FLAG flag;
        public static string msg = "";
        public static long t_sync = 0;
        public static long cnt_warn = 0;
        public static int cnt_err = 0;
        public static string[] err_list = new string[max_err_warn];
        public static string[] warn_list = new string[max_err_warn];

        // Coms Check Status
        public enum STATUS
        {
            SKIPPED,
            RUNNING,
            AWAITING,
            FINISHED,
            SUCCEEDED,
            ABORTED,
            FAILED,
            TIMEDOUT
        }

        // DB Flag Struct
        public struct DB_FLAG
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
                7: Hardware test
            */
            public double systemTest;
            // Debug matlab
            /*
                [0]: Dont break on errors
                [1]: Break on errors
                [>1]: Break on line
            */
            public int breakDebug;
            // Number of test packets [1:50]
            public double n_testPings;
            // Autoload rat data
            /*
                true: Load rat data based on ICR_GUI hardcoded values
                false: Start normally
            */
            public bool do_autoloadUI;
            // Print all blocked vt recs
            public bool do_printSkippedVT;
            // Print all sent rat vt recs
            public bool do_printSentRatVT;
            // Print all sent rob vt recs
            public bool do_printSentRobVT;
            // Print feederdue log
            public bool do_printRobLog;
            // Print cheetahdue log
            public bool do_printDueLog;
            // Print com flags
            public bool do_printComFlags;
            // Flag if doing any debugging
            public bool is_debugRun;

            // CONSTRUCTOR:
            public DB_FLAG(
                double systemTest,
                int breakDebug,
                double n_testPings,
                bool do_autoloadUI,
                bool do_printBlockedVT,
                bool do_printSentRatVT,
                bool do_printSentRobVT,
                bool do_printRobLog,
                bool do_printDueLog,
                bool do_printComFlags
                )
            {
                this.systemTest = systemTest;
                this.breakDebug = breakDebug;
                this.n_testPings = n_testPings;
                this.do_autoloadUI = do_autoloadUI;
                this.do_printSkippedVT = do_printBlockedVT;
                this.do_printSentRatVT = do_printSentRatVT;
                this.do_printSentRobVT = do_printSentRobVT;
                this.do_printRobLog = do_printRobLog;
                this.do_printDueLog = do_printDueLog;
                this.do_printComFlags = do_printComFlags;
                this.is_debugRun = systemTest > 0 || breakDebug > 0 || do_autoloadUI;
            }
        }

        // Setup Debugging
        public static void DB_Setup(ref MLApp.MLApp com_matlab)
        {
            // Start timer
            _stop_watch.Start();

            // Setup Conslole
            Console.BufferHeight = Int16.MaxValue - 1;

            // Copy flag
            flag = ICR_Run.flag;

            // Get build date/time
            DateTime buildDate = new FileInfo(System.Reflection.Assembly.GetExecutingAssembly().Location).LastWriteTime;
            DB_General(String.Format("BUILD DATE: {0}", buildDate));

            // Setup Session specific debugging
            if (flag.is_debugRun)
            {
                DB_General("SETTING UP DEBUGGING");

                // Show matlab app window
                com_matlab.Visible = 1;

                // Check for break line input
                if (flag.is_debugRun)
                {
                    // Keep checking on seperate thread
                    new Thread(delegate ()
                    {
                        CheckBreakInput();
                    }).Start();
                    DB_General("STARTING CheckBreakInput()");
                }

                // Log db settings
                DB_General(String.Format("RUNNING IN DEBUG MODE: systemTest={0} breakDebug={1} do_autoloadUI={2}",
                    flag.systemTest, flag.breakDebug, flag.do_autoloadUI));
            }
            else
            {
                // Hide matlab app window
                com_matlab.Visible = 0;
            }

        }

        // Log/Print Events
        public static void DB_General_Thread(string msg, long ts = 0, string msg_type = "NOTICE", string fun = "", int line = 0, string str_prfx = "", int indent = 0)
        {

            // Get calling function info
            if (fun == "")
            {
                var result = GetCallFun();
                fun = result.Item1;
                line = result.Item2;
            }

            // Get current time
            ts = ts > 0 ? ts : DT();

            // Run on seperate thread
            new Thread(delegate ()
            {
                DB_General(msg: msg, ts: ts, msg_type: msg_type, fun: fun, line: line, str_prfx: str_prfx, indent: indent);
            }).Start();
        }
        public static void DB_General(string msg, long ts = 0, string msg_type = "NOTICE", string fun = "", int line = 0, string str_prfx = "", int indent = 0)
        {
            // Get calling function info
            if (fun == "")
            {
                var result = GetCallFun();
                fun = result.Item1;
                line = result.Item2;
            }

            // Call to log on current thread
            DB_LogPrint(msg, ts: ts, msg_type: msg_type, fun: fun, line: line, str_prfx: str_prfx, indent: indent);

        }


        // Log/Print Warnings
        public static void DB_Warning_Thread(string msg, long ts = 0, string fun = "", int line = 0, string str_prfx = "")
        {

            // Get calling function info
            if (fun == "")
            {
                var result = GetCallFun();
                fun = result.Item1;
                line = result.Item2;
            }

            // Set to current time
            ts = ts > 0 ? ts : DT();

            // Run on seperate thread
            new Thread(delegate ()
            {
                DB_Warning(msg: msg, ts: ts, fun: fun, line: line, str_prfx: str_prfx);
            }).Start();

        }
        public static void DB_Warning(string msg, long ts = 0, string fun = "", int line = 0, string str_prfx = "")
        {

            // Get calling function info
            if (fun == "")
            {
                var result = GetCallFun();
                fun = result.Item1;
                line = result.Item2;
            }

            // Call to log on current thread with warning
            DB_LogPrint(msg, ts: ts, msg_type: "**WARNING**", fun: fun, line: line, str_prfx: str_prfx, indent: 0);

        }

        // Log/Print Errors
        public static void DB_Error_Thread(string msg, long ts = 0, string fun = "", int line = 0)
        {

            // Get calling function info
            if (fun == "")
            {
                var result = GetCallFun();
                fun = result.Item1;
                line = result.Item2;
            }

            // Set to current time
            ts = ts > 0 ? ts : DT();

            // Run on seperate thread
            new Thread(delegate ()
            {
                DB_Error(msg: msg, ts: ts, fun: fun, line: line);
            }).Start();

        }
        public static void DB_Error(string msg, long ts = 0, string fun = "", int line = 0)
        {
            // Local vars

            // Get calling function info
            if (fun == "")
            {
                var result = GetCallFun();
                fun = result.Item1;
                line = result.Item2;
            }

            // Call to log on current thread with error
            DB_LogPrint(msg, ts: ts, msg_type: "!!ERROR!!", fun: fun, line: line, indent: 0);

            // Set run error flag flag
            FC.is_RunErrors = true;
        }

        // Log/Print Com Status
        public static void DB_Status_Thread(string msg, STATUS status, long ts = 0, string fun = "", int line = 0)
        {
            // Get current time
            ts = ts > 0 ? ts : DT();

            // Get calling function info
            if (fun == "")
            {
                var result = GetCallFun();
                fun = result.Item1;
                line = result.Item2;
            }

            // Run on seperate thread
            new Thread(delegate ()
            {
                DB_Status(msg: msg, status: status, ts: ts, fun: fun, line: line);
            }).Start();
        }
        public static void DB_Status(string msg, STATUS status, long ts = 0, string fun = "", int line = 0)
        {
            // Local vars
            string str_print = "";

            // Get calling function info
            if (fun == "")
            {
                var result = GetCallFun();
                fun = result.Item1;
                line = result.Item2;
            }

            // Format print string
            str_print = String.Format("{0}: {1}", status.ToString(), msg);

            switch (status)
            {
                case STATUS.RUNNING:
                    {
                        DEBUG.DB_General(str_print, msg_type: "STATUS", fun: fun, line: line);
                        break;
                    }
                case STATUS.AWAITING:
                    {
                        DEBUG.DB_General(str_print + "...", msg_type: "STATUS", fun: fun, line: line);
                        break;
                    }
                case STATUS.FINISHED:
                    {
                        DEBUG.DB_General(str_print, msg_type: "STATUS", fun: fun, line: line);
                        break;
                    }
                case STATUS.SUCCEEDED:
                    {
                        DEBUG.DB_General(str_print, msg_type: "STATUS", fun: fun, line: line);
                        break;
                    }
                case STATUS.ABORTED:
                    {
                        DEBUG.DB_Warning(str_print, fun: fun, line: line);
                        break;
                    }
                case STATUS.TIMEDOUT:
                    {
                        DEBUG.DB_Error(str_print, fun: fun, line: line);
                        break;
                    }
                case STATUS.FAILED:
                    {
                        DEBUG.DB_Error(str_print, fun: fun, line: line);
                        break;
                    }
                default:
                    {
                        break;
                    }

            }

        }

        // Log/Print event
        public static void DB_LogPrint(string msg, long ts = 0, string msg_type = "NOTICE", string fun = "", int line = 0, string str_prfx = "", int indent = 0)
        {
            // Local vars
            int t_m = 0;
            string str_print = " ";
            string str_log = " ";
            string str_ts = " ";
            string str_ts_pad = " ";
            string str_prfx_pad = " ";

            // Get time from start of main()
            ts = ts > 0 ? ts : DT();

            // Get sync correction
            t_m = (int)(ts - t_sync);

            // Convert to string
            str_ts = FormatTimestamp(t_m);

            // Pad ts string
            str_ts_pad = str_ts.PadRight(25, ' ');

            // Format prefix sting with calling method and line number
            if (str_prfx == "")
                str_prfx = String.Format("[{0}:{1}]", fun, line);

            // Add additional indent
            str_prfx_pad = str_prfx;
            if (indent > 0)
                str_prfx_pad = str_prfx_pad.PadLeft(indent + str_prfx_pad.Length, ' ');

            // Indent functions > 2 levels from main()
            if (
                fun != "Main" && fun != "Setup" && fun != "Run" && fun != "Exit" &&
                !(msg_type == "!!ERROR!!" || msg_type == "**WARNING**"))
            {
                str_prfx_pad = str_prfx_pad.PadLeft(5 + str_prfx_pad.Length, ' ');
            }

            // Format string for printing
            string print_type = msg_type == "!!ERROR!!" || msg_type == "**WARNING**" ? " " + msg_type : "";
            str_print = String.Format("\n{0}{1} {2} {3}\n", str_ts_pad, print_type, str_prfx_pad, msg);

            // Print message
            lock (lock_console)
                Console.Write(str_print);

            // Remove cammas from message
            str_log = String.Format("{0} {1}", str_prfx, msg);
            str_log = str_log.Replace(",", string.Empty);

            // Store in logger 
            logger.UpdateLog(msg: str_log, str_ts: str_ts, ts: t_m, msg_type: msg_type);

            // Store error info
            if (msg_type == "!!ERROR!!")
                UpdateErrWarn(msg: str_print, is_error: true);

            // Store warning info
            if (msg_type == "**WARNING**")
                UpdateErrWarn(msg: str_print, is_warning: true);
        }

        // Update Error/Warning List and Count
        public static void UpdateErrWarn(string msg, bool is_error = false, bool is_warning = false)
        {

            // Store error info
            if (is_error)
            {
                err_list[cnt_err < max_err_warn ? cnt_err : max_err_warn - 1] = msg;
                _err_line[cnt_err < max_err_warn ? cnt_err : max_err_warn - 1] = logger.cnt_logsStored;
                cnt_err++;
            }

            // Store warning info
            if (is_warning)
            {
                warn_list[cnt_warn < max_err_warn ? cnt_warn : max_err_warn - 1] = msg;
                _warn_line[cnt_warn < max_err_warn ? cnt_warn : max_err_warn - 1] = logger.cnt_logsStored;
                cnt_warn++;
            }
        }

        // Get Error Summary
        public static string GetErrWarnSummary(string get_what)
        {
            // Local vars
            string str_summary = "";

            // Warnings
            if (get_what == "warnings")
            {
                string warn_lines = "ON LINES |";
                for (int i = 0; i < cnt_warn; i++)
                {
                    warn_lines = String.Format("{0}{1}|", warn_lines, _warn_line[i]);
                }
                str_summary = String.Format("TOTAL WARNINGS: {0} {1}", cnt_warn, cnt_warn > 0 ? warn_lines : "");
            }

            // Errors
            else if (get_what == "errors")
            {
                string err_lines = "ON LINES |";
                for (int i = 0; i < cnt_err; i++)
                {
                    err_lines = String.Format("{0}{1}|", err_lines, _err_line[i]);
                }
                str_summary = String.Format("TOTAL ERRORS: {0} {1}", cnt_err, cnt_err > 0 ? err_lines : "");
            }

            // Return string
            return str_summary;
        }

        // Get Calling Function Info
        public static Tuple<string, int> GetCallFun()
        {
            // Local vars
            StackTrace stack_trace = new StackTrace(true);
            string fun = "";
            int line = 0;

            // Get calling function name and line
            fun = stack_trace.GetFrame(2).GetMethod().Name;
            line = stack_trace.GetFrame(2).GetFileLineNumber();

            // Format tuple 
            var tuple = new Tuple<string, int>(fun, line);

            // Return outputs
            return tuple;

        }

        // Get Ellapsed Milliseconds
        public static long DT(long ts1 = 0, long ts2 = 0)
        {
            // Local vars
            long t_1 = ts1 > 0 ? ts1 : 0;
            long t_2 = ts2 > 0 ? ts2 : _stop_watch.ElapsedMilliseconds;

            // Return DT
            return t_2 - t_1;

        }

        // FORMAT INT AS BINARY
        public static string FormatBinary(UInt32 int_in)
        {
            string bit_str = "";
            UNION_HACK U = new UNION_HACK(0, '0', 0, 0, 0);
            U.i32 = int_in;
            int n_bytes = 0;

            // Get number of bytes
            n_bytes = U.b_3 > 0 ? 4 :
                U.b_2 > 0 ? 3 :
                U.b_1 > 0 ? 2 :
                U.b_0 > 0 ? 1 :
                0;

            if (n_bytes > 0)
            {
                bool do_write = false;
                for (int i = n_bytes; i >= 0; i--)
                {
                    byte b = i == 0 ? U.b_0 :
                        i == 1 ? U.b_1 :
                        i == 2 ? U.b_2 :
                        U.b_3;

                    do_write = do_write || b > 0;
                    if (!do_write)
                        continue;

                    for (int j = 7; j >= 0; j--)
                        bit_str += (b & (byte)(1 << j)) != 0 ? "1" : "0";

                    if (i > 0)
                        bit_str += " ";
                }
            }

            return bit_str;

        }

        // FORMAT TIME STAMP STRING
        public static string FormatTimestamp(int ts)
        {

            // Local vars
            string str_ts;
            int ts_m = 0;
            int s = 0;
            int ts_s = 0;
            int ts_ms = 0;

            // Get minutes
            ts_m = (ts - (ts % (60 * 1000))) / (60 * 1000);

            // Get seconds
            s = ts - (ts_m * 60 * 1000);
            ts_s = (s - (s % 1000)) / 1000;

            // Get milliseconds
            ts_ms = ts - (ts_m * 60 * 1000) - (ts_s * 1000);

            // Format string
            str_ts = String.Format("{0:00}:{1:00}:{2:000}", ts_m, ts_s, ts_ms);
            return str_ts;

        }

        // CHECK CONSOLE FOR BREAK DEBUG REQUEST
        public static void CheckBreakInput()
        {
            // Local vars
            int timeout_enter = 5000;
            int timeout_answer = 10000;
            string msg;

            // Keep looping
            while (!FC.do_Exit)
            {

                // Reinitialize vars
                string cmd = null;
                int line = 0;

                // Check for key press
                if (!Console.KeyAvailable)
                {
                    // Pause and bail
                    Thread.Sleep(10);
                    continue;
                }

                // Check for input
                cmd = READER.ReadLine(timeout_enter);

                // Bail if no input
                if (cmd == null)
                {
                    // Pause and bail
                    Thread.Sleep(10);
                    continue;
                }

                // Check for 'b'
                if (cmd == "db")
                {

                    // Print message
                    Console.Write("\nDO YOU WANT TO DEBUG MATLAB (Y/N + ENTER): ");

                    // Check for input
                    cmd = READER.ReadLine(timeout_answer);
                    if (cmd == null)
                        continue;

                    // Handle response
                    if (cmd == "Y" || cmd == "y")
                    {
                        // Print next message
                        Console.Write("\nENTER BREAK LINE NUMBER: ");

                        // Check for input
                        cmd = READER.ReadLine(timeout_answer);
                        if (cmd == null || !Int32.TryParse(cmd, out line))
                        {
                            Console.Write("\nABORTING: ENTRY NOT NUMERIC\n");
                            continue;
                        }

                        // Break matlab at this line
                        Console.Write(String.Format("\n SETTING MATLAB BREAKPOINT AT LINE {0}\n", line));
                        msg = String.Format("dbstop at {0} in ICR_GUI", line);
                        ICR_Run.SendMatCom(msg: msg);

                    }
                    else if (cmd == "N" || cmd == "n")
                        Console.Write(String.Format("\nSELECTED '{0}': EXITING\n", cmd));
                    else
                        Console.Write("\nABORTING: INVALID ENTRY");
                }
            }

        }

    }

    // CLASS TO TRACK PROGRAM FLAGS
    public class FC
    {

        // PRIVATE VARS
        private static bool _do_SessionICR = false;
        private static bool _do_SessionTurnTT = false;
        private static bool _do_SessionUpdateTable = false;
        private static bool _is_NlxConnected = false;
        private static bool _is_RatStreaming = false;
        private static bool _is_RobStreaming = false;
        private static bool _is_MatComActive = false;
        private static bool _is_FeederDueComActive = false;
        private static bool _is_CheetahDueComActive = false;
        private static bool _is_MovedToStart = false;
        private static bool _is_RatInArena = false;
        private static bool _is_RatOnTrack = false;
        private static bool _is_TaskDone = false;
        private static bool _is_MatDataSaved = false;
        private static bool _is_NlxDirCreated = false;
        private static bool _is_NlxDataSaved = false;
        private static bool _is_GUIquit = false;
        private static bool _is_GUIclosed = false;
        private static bool _do_SendAbortMat = false;
        private static bool _do_SoftAbortMat = false;
        private static bool _is_SoftAbortMat = false;
        private static bool _do_HardAbortMat = false;
        private static bool _is_HardAbortMat = false;
        private static bool _is_MatFailed = false;
        private static bool _do_AbortCS = false;
        private static bool _is_RunErrors = false;
        private static bool _do_Exit = false;

        // PUBLIC VARS
        public static bool do_SessionICR
        {
            set { DebugFlagChange(ref _do_SessionICR, "do_SessionICR", value); }
            get { return _do_SessionICR; }
        }
        public static bool do_SessionTurnTT
        {
            set { DebugFlagChange(ref _do_SessionTurnTT, "do_SessionTurnTT", value); }
            get { return _do_SessionTurnTT; }
        }
        public static bool do_SessionUpdateTable
        {
            set { DebugFlagChange(ref _do_SessionUpdateTable, "do_SessionUpdateTable", value); }
            get { return _do_SessionUpdateTable; }
        }
        public static bool is_NlxConnected
        {
            set { DebugFlagChange(ref _is_NlxConnected, "is_NlxConnected", value); }
            get { return _is_NlxConnected; }
        }
        public static bool is_RatStreaming
        {
            set { DebugFlagChange(ref _is_RatStreaming, "is_RatStreaming", value); }
            get { return _is_RatStreaming; }
        }
        public static bool is_RobStreaming
        {
            set { DebugFlagChange(ref _is_RobStreaming, "is_RobStreaming", value); }
            get { return _is_RobStreaming; }
        }
        public static bool is_MatComActive
        {
            set { DebugFlagChange(ref _is_MatComActive, "is_MatComActive", value); }
            get { return _is_MatComActive; }
        }
        public static bool is_FeederDueComActive
        {
            set { DebugFlagChange(ref _is_FeederDueComActive, "is_FeederDueComActive", value); }
            get { return _is_FeederDueComActive; }
        }
        public static bool is_CheetahDueComActive
        {
            set { DebugFlagChange(ref _is_CheetahDueComActive, "is_CheetahDueComActive", value); }
            get { return _is_CheetahDueComActive; }
        }
        public static bool is_MovedToStart
        {
            set { DebugFlagChange(ref _is_MovedToStart, "is_MovedToStart", value); }
            get { return _is_MovedToStart; }
        }
        public static bool is_RatInArena
        {
            set { DebugFlagChange(ref _is_RatInArena, "is_RatInArena", value); }
            get { return _is_RatInArena; }
        }
        public static bool is_RatOnTrack
        {
            set { DebugFlagChange(ref _is_RatOnTrack, "is_RatOnTrack", value); }
            get { return _is_RatOnTrack; }
        }
        public static bool is_TaskDone
        {
            set { DebugFlagChange(ref _is_TaskDone, "is_TaskDone", value); }
            get { return _is_TaskDone; }
        }
        public static bool is_MatDataSaved
        {
            set { DebugFlagChange(ref _is_MatDataSaved, "is_MatDataSaved", value); }
            get { return _is_MatDataSaved; }
        }
        public static bool is_NlxDataSaved
        {
            set { DebugFlagChange(ref _is_NlxDataSaved, "is_NlxDataSaved", value); }
            get { return _is_NlxDataSaved; }
        }
        public static bool is_NlxDirCreated
        {
            set { DebugFlagChange(ref _is_NlxDirCreated, "is_NlxDirCreated", value); }
            get { return _is_NlxDirCreated; }
        }
        public static bool is_GUIquit
        {
            set { DebugFlagChange(ref _is_GUIquit, "is_GUIquit", value); }
            get { return _is_GUIquit; }
        }
        public static bool is_GUIclosed
        {
            set { DebugFlagChange(ref _is_GUIclosed, "is_GUIclosed", value); }
            get { return _is_GUIclosed; }
        }
        public static bool do_SendAbortMat
        {
            set { DebugFlagChange(ref _do_SendAbortMat, "do_SendAbortMat", value); }
            get { return _do_SendAbortMat; }
        }
        public static bool do_SoftAbortMat
        {
            set { DebugFlagChange(ref _do_SoftAbortMat, "do_SoftAbortMat", value); }
            get { return _do_SoftAbortMat; }
        }
        public static bool is_SoftAbortMat
        {
            set { DebugFlagChange(ref _is_SoftAbortMat, "is_SoftAbortMat", value); }
            get { return _is_SoftAbortMat; }
        }
        public static bool do_HardAbortMat
        {
            set { DebugFlagChange(ref _do_HardAbortMat, "do_HardAbortMat", value); }
            get { return _do_HardAbortMat; }
        }
        public static bool is_HardAbortMat
        {
            set { DebugFlagChange(ref _is_HardAbortMat, "is_HardAbortMat", value); }
            get { return _is_HardAbortMat; }
        }
        public static bool is_MatFailed
        {
            set { DebugFlagChange(ref _is_MatFailed, "is_MatFailed", value); }
            get { return _is_MatFailed; }
        }
        public static bool do_AbortCS
        {
            set { DebugFlagChange(ref _do_AbortCS, "do_AbortCS", value); }
            get { return _do_AbortCS; }
        }
        public static bool is_RunErrors
        {
            set { DebugFlagChange(ref _is_RunErrors, "is_RunErrors", value); }
            get { return _is_RunErrors; }
        }
        public static bool do_Exit
        {
            set { DebugFlagChange(ref _do_Exit, "do_Exit", value); }
            get { return _do_Exit; }
        }

        // Set error status
        public static void SetAbort(bool set_abort_cs = false, bool set_abort_mat = false, bool is_mat_failed = false)
        {

            // Set to abourt CS script
            if (set_abort_cs)
            {
                do_AbortCS = true;
                DEBUG.DB_Warning_Thread("SET ABORT CS");
            }

            // Set received check flag
            else if (set_abort_mat)
            {
                // Set flag to trigger sending abort message
                do_SendAbortMat = true;

                // Do hard abort if rat not in arena yet
                if (!is_RatInArena && !is_GUIquit)
                {
                    do_HardAbortMat = true;
                    do_AbortCS = true;
                    DEBUG.DB_Warning_Thread("SET HARD ABORT MATLAB");
                }

                // Attempt to save data if rat in arena
                else if (is_RatInArena && !is_GUIquit)
                {
                    do_SoftAbortMat = true;
                    DEBUG.DB_Warning_Thread("SET SOFT ABORT MATLAB");
                }

            }

            // Check if matlab hanging
            if (is_mat_failed)
            {
                is_MatFailed = true;
                do_AbortCS = true;
                do_SendAbortMat = true;
                DEBUG.DB_Error_Thread("MATLAB HAS FAILED/CRASHED");
            }

        }

        // Debug flag change
        public static void DebugFlagChange(ref bool flag_loc, string var_name, bool value)
        {

            // Bail if value already set
            if (flag_loc == value)
            {
                return;
            }

            // Log flag change
            DEBUG.DB_General_Thread(String.Format("Set \"{0}\" to {1}",
                   var_name, value), indent: 15);

            // Change local value
            flag_loc = value;
        }

        // Check if Matlab coms are active
        public static bool ContinueMatCom()
        {
            bool do_cont = is_MatComActive && !is_MatFailed && !do_Exit;
            if (!do_cont)
            {
                string str_flag = String.Format("RETURNING DISCONTINUE FLAG: \"is_MatComActive\"={0} \"is_MatFailed\"={1} \"do_Exit\"={2}",
                    is_MatComActive, is_MatFailed, do_Exit);
                DEBUG.DB_General_Thread(str_flag, indent: 15);
            }
            return do_cont;
        }

        // Check if serial Xbee coms are active
        public static bool ContinueFeederDueCom()
        {
            bool do_cont = is_FeederDueComActive && !do_Exit;
            if (!do_cont)
            {
                string str_flag = String.Format("RETURNING DISCONTINUE FLAG: \"is_FeederDueComActive\"={0} \"do_Exit\"={1}",
                    is_FeederDueComActive, do_Exit);
                DEBUG.DB_General_Thread(str_flag, indent: 15);
            }
            return do_cont;
        }

        // Check if serial CheetahDue coms active
        public static bool ContinueCheetahDueCom()
        {
            bool do_cont = is_CheetahDueComActive && !do_Exit;
            if (!do_cont)
            {
                string str_flag = String.Format("RETURNING DISCONTINUE FLAG: \"is_CheetahDueComActive\"={0} \"do_Exit\"={1}",
                    is_CheetahDueComActive, do_Exit);
                DEBUG.DB_General_Thread(str_flag, indent: 15);
            }
            return do_cont;
        }

    }

    // CLASS TO LOG DB INFO
    public class LOGGER
    {

        // PRIVATE VARS
        private string[] _logList = new string[100000];
        private readonly object _lock_updateLog = new object();
        private long _t_logStart = 0;
        private string _lastLogStr = " ";
        private bool _is_started = false;
        private bool _is_streaming = false;
        private bool _is_imported = false;
        private bool _is_importTimedout = false;
        private bool _is_aborted = false;
        private int next_milestone = 0;
        private const int _n_updates = 10;
        private int[] _importUpdateBytes = new int[_n_updates];

        // PUBLIC VARS
        public long t_sync;
        public string[] prcntPrintStr = new string[_n_updates + 1];
        public int cnt_logsStored = 0;
        public int[] cnt_dropped = new int[2] { 0, 0 };
        public int bytesRead = 0;
        public int bytesToRcv = 0;

        // Special public vars
        public bool is_streaming
        {
            set
            {
                // Store total log time
                if (value)
                {
                    // Store log start time
                    _t_logStart = DEBUG.DT();

                    // Set flag once
                    _is_started = true;
                }
                _is_streaming = value;
            }
            get { return _is_streaming; }
        }
        public bool is_imported
        {
            set { _is_imported = value; }
            get { return _is_imported; }
        }
        public bool is_importFinished
        {
            get
            {
                return _is_imported || _is_importTimedout || _is_aborted || !_is_started;
            }
        }
        public bool is_logComplete
        {
            get
            {
                if (cnt_logsStored > 0 &&
                    cnt_dropped[1] == 0 &&
                    (bytesToRcv == 0 || bytesRead > bytesToRcv))
                    return true;
                else
                    return false;
            }
        }
        public bool is_importTimedout
        {
            set { _is_importTimedout = value; }
            get { return _is_importTimedout; }
        }
        public long logDT
        {
            get { return DEBUG.DT(ts1: _t_logStart); }
        }

        // CONSTRUCTOR
        public LOGGER(
        string header
        )
        {
            this._logList[cnt_logsStored] = header;
        }

        // Add new log entry
        public void UpdateLog(string msg, string str_ts = "", long ts = 0, string msg_type = "NOTICE")
        {

            // Local vars
            string str = "";
            string str_err = "";

            lock (_lock_updateLog)
            {

                // Check for repeat
                if (msg != _lastLogStr)
                {
                    // Save log string
                    _lastLogStr = msg;

                    // Incriment count
                    cnt_logsStored++;

                    // Reset consecutive dropped logs
                    cnt_dropped[0] = 0;

                    // Indent "COM" mesages
                    if (msg_type == "COM")
                        msg = msg.PadLeft(3 + msg.Length, ' ');

                    // Add count and time
                    if (str_ts != "")
                        str = String.Format("{0},{1},=\"{2}\",{3},{4}", cnt_logsStored, ts, str_ts, msg_type, msg);
                    else
                        str = String.Format("{0},{1}", cnt_logsStored, msg);

                    // Add to list
                    if (cnt_logsStored <= _logList.Length)
                    {
                        _logList[cnt_logsStored] = str;
                    }

                    // Max logs stored and end reachedd
                    else
                    {
                        // Set to last entry in array
                        cnt_logsStored = _logList.Length;

                        // Format error string
                        str_err = String.Format("!!ERROR!! LOG OVERFLOWED {0} ENTRIES", _logList.Length);

                        // Store error in last entry
                        _logList[cnt_logsStored - 1] = str_err;

                        // Add to error list
                        DEBUG.UpdateErrWarn(str_err, is_error: true);

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
            bytesToRcv = (int)dat[0];

            // Compute print update milestones
            for (int i = 0; i <= _n_updates; i++)
            {
                // Get milestone value
                if (i < _n_updates)
                    _importUpdateBytes[i] = (int)((double)bytesToRcv * i / 10);

                // Get string
                str = String.Format("{0}% Complete", i * _n_updates);
                str = str.PadRight(str.Length + _n_updates - i, '.');
                str = str.PadLeft(str.Length + i, '.');
                prcntPrintStr[i] = str;
            }
        }

        // Pass string giving import status
        public string GetImportStatus(int cnt_bytes)
        {
            // Local vars
            string msg = " ";

            // Check if milestone reached
            if (next_milestone < _n_updates)
                if (cnt_bytes == _importUpdateBytes[next_milestone])
                {
                    msg = prcntPrintStr[next_milestone];
                    next_milestone++;
                }

            // Return string
            return msg;
        }

        // Save log data to csv
        public bool SaveLog(string log_dir, string log_fi)
        {
            // Local vars
            string fi_path = @log_dir + @"\" + @log_fi;

            // Check for directory
            if (!Directory.Exists(log_dir))
            {
                // Set flag
                _is_aborted = true;
                DEBUG.DB_Error(String.Format("DIRECTORY \"{0}\" DOES NOT EXIST", log_dir));
                return false;
            }

            // Check if file locked
            if (IsFileLocked(fi_path))
            {
                // Set flag
                _is_aborted = true;
                DEBUG.DB_Error(String.Format("FILE \"{0}\" IS LOCKED", log_fi));
                return false;
            }

            // Write log to file
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

            // Return success
            return true;
        }

        // Check if file locked
        public bool IsFileLocked(string filename)
        {
            bool Locked = false;
            try
            {
                FileStream fs =
                    File.Open(filename, FileMode.OpenOrCreate,
                    FileAccess.ReadWrite, FileShare.None);
                fs.Close();
            }
            catch (IOException)
            {
                Locked = true;
            }
            return Locked;
        }

    }

    // CLASS TO TRACK COMS
    class COM_TRACK
    {

        // PRIVATE VARS
        private string _objID;
        private UInt16[] _packRange = new UInt16[2] { 0, 0 };
        private char[] _idArr;
        private double[][] _datMat;
        private UInt16[] _packArr;
        private UInt16[] _packConfArr;
        private long[] _t_sentRcvd;
        private object _lock_isSentRcv = new object();
        private object _lock_isConf = new object();
        private object _lock_isDone = new object();
        private bool[] _isSentRcv;
        private bool[] _isConf;
        private bool[] _isDone;

        // PUBLIC VARS
        public byte head = 0;
        public byte foot = 0;
        public long dt_minSentRcvd = 0;
        public long dt_resend = 0;
        public int resendMax = 0;
        public UInt16 packInd = 0;
        public UInt32 packSentAll = 0;
        public UInt32 packRcvdAll = 0;
        public int cnt_dropped = 0;
        public int cnt_repeat = 0;
        public long t_new = 0;
        public long t_last = 0;
        public long t_parseStart = 0;

        // CONSTRUCTOR
        public COM_TRACK(
        string objID,
        UInt16[] packRange,
        char[] id,
        byte head = 0,
        byte foot = 0,
        long dt_minSentRcvd = 0,
        long dt_resend = 0,
        int resendMax = 0
        )
        {
            this._objID = objID;
            this._packRange[0] = packRange[0];
            this._packRange[1] = packRange[1];
            this.packInd = packRange[0];
            this.packInd--;
            this._idArr = id;
            this.head = head;
            this.foot = foot;
            this.dt_minSentRcvd = dt_minSentRcvd;
            this.dt_resend = dt_resend;
            this.resendMax = resendMax;
            _datMat = new double[id.Length][];
            _packArr = new UInt16[id.Length];
            _packConfArr = new UInt16[id.Length];
            _t_sentRcvd = new long[id.Length];
            _isSentRcv = new bool[id.Length];
            _isConf = new bool[id.Length];
            _isDone = new bool[id.Length];

            // Initialize values to zero
            for (int i = 0; i < id.Length; i++)
            {
                _packArr[i] = 0;
                _datMat[i] = new double[3] { 0, 0, 0 };
                _packConfArr[i] = 0;
                _t_sentRcvd[i] = 0;
                _isSentRcv[i] = false;
                _isConf[i] = false;
                _isDone[i] = false;
            }
        }

        // Set check status
        public void SetComFlags(char id = ' ', UInt16 pack = 0, bool set_is_sent_rcvd = false, bool set_is_conf = false, bool set_is_done = false, bool do_print = true)
        {
            // Local vars
            int id_ind = id != ' ' ? ID_Ind(id) : ID_Pack_Ind(pack);

            // Handle print flag
            do_print = do_print && DEBUG.flag.do_printComFlags;

            // Set sent/received check flag
            if (set_is_sent_rcvd)
            {
                lock (_lock_isSentRcv)
                    _isSentRcv[id_ind] = true;
                if (do_print)
                    DEBUG.DB_General_Thread(String.Format("SET {0} '{1}' |Send/Rcv| {2}", _objID, id, true), indent: 15);
            }

            // Set received check flag
            if (set_is_conf)
            {
                lock (_lock_isConf)
                    _isConf[id_ind] = true;
                if (do_print)
                    DEBUG.DB_General_Thread(String.Format("SET {0} '{1}' |Conf| {2}", _objID, id, true), indent: 15);
            }

            // Set done check flag
            if (set_is_done)
            {
                lock (_lock_isDone)
                    _isDone[id_ind] = true;
                if (do_print)
                    DEBUG.DB_General_Thread(String.Format("SET {0} '{1}' |Done| {2}", _objID, id, true), indent: 15);
            }

        }

        // Get check status
        public bool GetComFlags(char id = ' ', bool get_is_sent_rcvd = false, bool get_is_conf = false, bool get_is_done = false)
        {
            // Local vars
            int id_ind = ID_Ind(id);
            bool val = false;

            // Set sent/received check flag
            if (get_is_sent_rcvd)
            {
                lock (_lock_isSentRcv)
                {
                    val = _isSentRcv[id_ind];
                    _isSentRcv[id_ind] = false;
                }
            }

            // Set received check flag
            else if (get_is_conf)
            {
                lock (_lock_isConf)
                {
                    val = _isConf[id_ind];
                    _isConf[id_ind] = false;
                }
            }

            // Set done check flag
            else if (get_is_done)
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

        // Reset Status Flags
        public void ResetComFlags(char id = ' ', bool do_print = true)
        {
            // Local vars
            int id_ind = ID_Ind(id);

            // Handle print flag
            do_print = do_print && DEBUG.flag.do_printComFlags;

            // Reset all flags
            lock (_lock_isSentRcv)
                _isSentRcv[id_ind] = false;
            lock (_lock_isConf)
                _isConf[id_ind] = false;
            lock (_lock_isDone)
                _isDone[id_ind] = false;

            // Log
            if (do_print)
                DEBUG.DB_General_Thread(String.Format("RESET {0} '{1}'", _objID, id), indent: 15);
        }

        // Update sent packet info
        public Tuple<string, bool> UpdateSent(char id, double[] dat, UInt16 pack, long ts, bool is_conf = false, bool is_resend = false)
        {

            // Local vars
            int id_ind = 0;
            bool is_repeat = false;
            UInt16 pack_last = 0;
            string str_prfx = "";

            // Get id ind
            id_ind = ID_Ind(id);

            // Update data
            _datMat[id_ind][0] = dat[0];
            _datMat[id_ind][1] = dat[1];
            _datMat[id_ind][2] = dat[2];

            // Update timers
            t_last = t_new;
            t_new = ts;
            _t_sentRcvd[id_ind] = ts;

            // Get last pack
            if (!is_conf)
                pack_last = _packArr[id_ind];
            else
                pack_last = _packConfArr[id_ind];

            // Flag repeat pack
            is_repeat = pack == pack_last;

            // Incriment repeat
            cnt_repeat += is_repeat || is_resend ? 1 : 0;

            // Update packet history
            if (!is_conf)
                _packArr[id_ind] = pack;
            else
                _packConfArr[id_ind] = pack;

            // Incriment total sent packet count
            packSentAll++;

            // Format prefix string
            str_prfx =
                String.Format("[{0}SENT{1}:{2}]",
                is_resend ? "RSND-" : is_repeat ? "RPT-" : "",
                is_conf ? "-CONF" : "",
                _objID);

            // Format tuple 
            var tuple = new Tuple<string, bool>(str_prfx, is_repeat);

            // Return outputs
            return tuple;

        }

        // Update recived packet info
        public Tuple<string, bool, int> UpdateRcvd(char id, double[] dat, UInt16 pack, long ts, bool is_conf = false, bool is_done = false, bool is_resend = false)
        {

            // Local vars
            int id_ind = 0;
            bool is_repeat = false;
            int dropped = 0;
            UInt16 pack_last = 0;
            string str_prfx = "";

            // Get id ind
            id_ind = ID_Ind(id);

            // Update data
            _datMat[id_ind][0] = dat[0];
            _datMat[id_ind][1] = dat[1];
            _datMat[id_ind][2] = dat[2];

            // Update timers
            t_last = t_new;
            t_new = ts;
            _t_sentRcvd[id_ind] = ts;

            // Get last pack
            pack_last = is_conf ? _packConfArr[id_ind] : _packArr[id_ind];

            // Flag resent pack
            is_repeat = pack == pack_last;

            // Incriment repeat
            cnt_repeat += is_repeat || is_resend ? 1 : 0;

            // Update packet history
            if (!is_conf)
                _packArr[id_ind] = pack;
            else
                _packConfArr[id_ind] = pack;

            // Update for new packets
            if (!is_repeat && !is_conf && !is_done)
            {

                // Get pack diff accounting for packet rollover
                int pack_diff =
                    Math.Abs(pack - packInd) < (_packRange[1] - _packRange[0]) ?
                    pack - packInd :
                    pack - (_packRange[0] - 1);

                // Update dropped packets
                dropped = (pack - packInd) - 1;
                AddDropped(dropped);

                // Update packets sent
                packSentAll += (uint)pack_diff;

                // Update packet ind 
                packInd = pack;

                // Incriment packets recieved
                packRcvdAll++;

            }

            // Format prefix string
            str_prfx =
               String.Format("[{0}RCVD{1}{2}:{3}]",
               is_resend ? "RSND-" : is_repeat ? "RPT-" : "",
               is_conf ? "-CONF" : "",
               is_done ? "-DONE" : "",
               _objID);

            // Format tuple 
            var tuple = new Tuple<string, bool, int>(str_prfx, is_repeat, dropped);

            // Return outputs
            return tuple;


        }

        // Track dropped packets
        public void AddDropped(int cnt)
        {
            cnt_dropped += cnt;
        }

        // Incriment packet
        public UInt16 PackIncriment(int inc)
        {

            // Incriment packet
            packInd = (UInt16)((int)packInd + inc);

            // Reset to lowest range value if out of range
            if (packInd > _packRange[1])
            {
                string str_print = String.Format("RESETTING {0} PACKET INDEX FROM {1} TO {2}", _objID, packInd, _packRange[0]);
                DEBUG.DB_General_Thread(str_print);

                // Set to lowest range value
                packInd = _packRange[0];
            }

            // Return number
            return packInd;

        }

        // Get dt sent or received
        public long DT_SentRcvd(long t2 = 0)
        {

            // Set t1 to greater of str_t_parse or new sent/rcvd if input geven otherwise set to last sent/rcvd 
            long t_1 = t2 > 0 ?
                t_parseStart > t_new ? t_parseStart : t_new :
                t_last;

            // Set t2 to input time if time given or greater of str_t_parse or newest sent/rcvd
            long t_2 = t2 > 0 ?
                t2 :
                t_parseStart > t_new ? t_parseStart : t_new;

            // Return t2-t1
            return t_2 - t_1;
        }

        // Get last ts send/revieve
        public long ID_T_Last(char id)
        {
            return _t_sentRcvd[ID_Ind(id)];
        }

        // Get data
        public double ID_Dat(char id, int dat_ind)
        {
            return _datMat[ID_Ind(id)][dat_ind];
        }

        // Find id index
        public int ID_Ind(char id)
        {
            int ind = -1;
            for (int i = 0; i < _idArr.Length; i++)
            {
                if (id == _idArr[i])
                {
                    ind = i;
                }
            }
            return ind;
        }

        // Find packet index
        public int ID_Pack_Ind(UInt16 pack)
        {
            // Local vars
            bool[] flag_arr = new bool[_packArr.Length];
            int id_ind = 0;
            int cnt = 0;

            // Flag matching packets
            for (int i = 0; i < _packArr.Length; i++)
            {
                if (pack == _packArr[i])
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
                    for (int j = 0; j < _packArr.Length; j++)
                    {
                        if (!flag_arr[j])
                            continue;
                        else if (_t_sentRcvd[i] >= _t_sentRcvd[j])
                            id_ind = i;
                    }
                }
            }

            // Return array
            return id_ind;
        }

    }

    // CLASS TO HANDLE VT DATA
    class VT_HANDLER
    {

        // PRIVATE VARS
        private static int _cntThread = 0;
        private static long _t_blockTim = 0;
        private long _dt_block = 0; // (ms) 

        // PUBLIC VARS
        public double feedDist = 0; // (rad)
        public double R;
        public double XC;
        public double YC;
        public UInt64 streamStart = 0;
        public double[,] rad = new double[2, 2];
        public double[] cm = new double[2];
        public UInt32[,] ts = new UInt32[2, 2];
        public bool[] is_streamStarted = new bool[2] { false, false };
        public long[] t_sent = new long[2] { 0, 0 };
        public long[] t_sent_last = new long[2] { 0, 0 };
        public int[,] dt_hist = new int[2, 10];
        public int[] cnt_sent = new int[2] { 0, 0 };
        public int[] cnt_block = new int[2] { 0, 0 };

        // CONSTRUCTOR
        public VT_HANDLER(
        long dt_block,
        double feedDist
        )
        {
            this._dt_block = dt_block;
            this.feedDist = feedDist;
        }

        // Block sending vt data
        public void Block(char id)
        {
            if (id != 'P')
            {
                _cntThread++;
                _t_blockTim = DEBUG.DT() + _dt_block;
            }
        }

        // Unblock sending vt data
        public void Unblock(char id)
        {
            if (id != 'P')
            {
                _cntThread--;
            }
        }

        // Check if currently blocking
        public bool CheckBlock(UInt16 ent)
        {
            if (DEBUG.DT() > _t_blockTim && _cntThread <= 0)
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
                return (int)(DEBUG.DT(ts1: t_sent[ent]));
            }
            else
                return 0;
        }
        public void StoreSendTime(int ent, long ts)
        {
            // Update time
            t_sent_last[ent] = t_sent[ent];
            t_sent[ent] = ts;

            // Update count and sum
            cnt_sent[ent]++;

            // Shift and update dt hist
            for (int i = 0; i < 10 - 1; i++)
                dt_hist[ent, i] = dt_hist[ent, i + 1];

            // Store current dt
            dt_hist[ent, 9] = (int)(t_sent[ent] - t_sent_last[ent]);
        }
    }

    // CLASS READLINE WITH TIMEOUT
    public class READER
    {
        private static Thread inputThread;
        private static AutoResetEvent getInput, gotInput;
        private static string input;

        static READER()
        {
            getInput = new AutoResetEvent(false);
            gotInput = new AutoResetEvent(false);
            inputThread = new Thread(reader);
            inputThread.IsBackground = true;
            inputThread.Start();
        }

        private static void reader()
        {
            while (true)
            {
                getInput.WaitOne();
                input = Console.ReadLine();
                gotInput.Set();
            }
        }

        // omit the parameter to read a line without a timeout
        public static string ReadLine(int timeOutMillisecs = Timeout.Infinite)
        {
            getInput.Set();
            bool success = gotInput.WaitOne(timeOutMillisecs);
            if (success)
                return input;
            else
            {
                DEBUG.DB_Warning(String.Format("ABORTED: Did Not Provide Input Within {0} ms", timeOutMillisecs));
                return null;
            }
        }
    }

    // USE TO CONVERT BETWEEN DATA TYPE FOR SERIAL COMS
    [StructLayout(LayoutKind.Explicit, Pack = 1)]
    struct UNION_HACK
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
        public UNION_HACK(byte b, char c, UInt16 i16, UInt32 i32, float f)
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