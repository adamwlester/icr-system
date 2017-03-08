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

namespace MeStreamNLX
{

    class MeStreamNLX
    {

        #region ---------TOP LEVEL VARS---------

        // To quit exit program smoothly
        private static bool doQuit = false;
        private static bool doExit = false;
        private static bool robotStreaming = false;
        private static bool printSentVt = false;

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

        // Matlab communication
        private static char[] m2c_id = new char[9] { // prefix giving masage id
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // dispense reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // start/end pid
            'N', // matlab not loaded
            'L', // matlab loaded
             };
        private static char matIn_id = 'N'; // matlab now
        private static double matIn_dat1; // matlab data
        private static double matIn_dat2; // matlab data

        // CS to robot communication
        private static char[] c2r_id = new char[10] {
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // dispense reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // start/end pid
            'P', // position data
            'C', // request stream status
            'Y', // confirm done recieved
             };
        private static char[] char_head = new char[] { '_', '<' }; // 2 byte header
        private static char char_foot = '>'; // 1 byte header
        private static List<char> c2r_idHist = new List<char>();
        private static List<short> c2r_packHist = new List<short>();
        private static short c2r_packCnt = 0; // packet number
        private static byte[] c2r_head = new byte[2];
        private static byte[] c2r_foot = new byte[1];
        private static byte[] c2r_packNum = new byte[2];

        // Robot to CS communication
        private static char[] r2c_id = new char[9] {
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // dispense reward
            'H', // halt movement
            'B', // bulldoze rat
            'I', // start/end pid
            'D', // execution done
            'C', // connected and streaming
             };
        private static char r2c_head = '{';
        private static char r2c_foot = '}';
        private static List<char> r2c_idHist = new List<char>();
        private static List<short> r2c_packHist = new List<short>();
        private static short[] r2c_packLast = new short[r2c_id.Length];

        // General communication
        private static long t_c2rDel = 15;
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
            public short s1; // (short) 2 byte
            [FieldOffset(2)]
            public short s2;
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
            public UnionHack(int i, float f, short s, char c, byte b)
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
            PrintAction("[Main] BEGIN Main");
            // Hide matlab app window
            com_Matlab.Visible = 0;

            // Start primary timer
            sw_main.Start();

            // Create header and footer byte array
            UnionHack u = new UnionHack(0, 0, 0, '0', 0);
            // get header chars
            u.c1 = char_head[0];
            u.c2 = char_head[1];
            // use only first byte 
            // becasue arduino uses uni-8
            c2r_head[0] = u.b1;
            c2r_head[1] = u.b3;
            // Get footer byte 
            u.c1 = char_foot;
            c2r_foot[0] = u.b1;

            // Initialize list entries
            r2c_idHist.Add(' ');
            c2r_idHist.Add(' ');
            r2c_packHist.Add(0);
            c2r_packHist.Add(0);

            // Initialize last packet array
            for (int i = 0; i < r2c_packLast.Length; i++)
                r2c_packLast[i] = (short)(i * -1);

            // Setup ICR_GUI background worker
            bw_RunGUI.DoWork += DoWork_RunGUI;
            bw_RunGUI.RunWorkerCompleted += RunWorkerCompleted_RunGUI;
            PrintAction("[Main] RUN RunGUI Worker");
            // Start ICR_GUI worker
            bw_RunGUI.RunWorkerAsync();

            // Setup MATLAB com background worker
            bw_MatCOM.DoWork += DoWork_MatCOM;
            bw_MatCOM.ProgressChanged += ProgressChanged_MatCOM;
            bw_MatCOM.RunWorkerCompleted += RunWorkerCompleted_MatCOM;
            bw_MatCOM.WorkerReportsProgress = true;

            // Start Cheetah if it is not already running
            PrintAction("[Main] OPEN Cheetah");
            while (!IsProcessOpen("Cheetah"))
            {
                OpenCheetah("Cheetah.cfg");
            }

            // Initilize deligate for VT callback
            MNetComClient com_netComClient = new MNetComClient();
            deligate_netComCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            com_netComClient.SetCallbackFunctionVT(deligate_netComCallback, new MeStreamNLX());

            // Set sp_Xbee parameters
            sp_Xbee.ReadTimeout = 100;
            sp_Xbee.BaudRate = 57600;
            sp_Xbee.PortName = "COM16";
            // Create event handeler for incoming data
            sp_Xbee.DataReceived += DataReceived_Xbee;
            // Open serial port connection
            PrintAction("[Main] OPEN Xbee Serial Port");
            sp_Xbee.Open();

            // Set NetCom parameters
            var NETCOM_APP_ID = "MeStreamNLX"; // string displayed in Cheetah when connected
            var NETCOM_ACQ_ENT_1 = "VT1"; // aquisition entity to stream
            var NETCOM_ACQ_ENT_2 = "VT2"; // aquisition entity to stream
            var NETCOM_IP = "127.0.0.1"; // host computer IP

            // Start MatCOM worker
            PrintAction("[Main] RUN MatCOM Worker");
            // Crate argument vars for backround worker
            char bw_id = ' ';
            double bw_d1 = 0;
            double bw_d2 = 0;
            var bw_args = Tuple.Create(bw_id, bw_d1, bw_d2);
            bw_MatCOM.RunWorkerAsync(bw_args);

            // Wait for ICR_GUI to load
            while (matIn_id != 'L' && !(doQuit || doExit)) ;
            PrintAction("[Main] DONE ICR_GUI Loaded");


            // Setup connection
            if (!(com_netComClient.AreWeConnected()))
            {
                if (com_netComClient.ConnectToServer(NETCOM_IP))
                {

                    // Begin stream and set app name
                    PrintAction("[Main] BEGIN Nlx Stream");
                    com_netComClient.SetApplicationName(NETCOM_APP_ID);
                    // rat vt
                    com_netComClient.OpenStream(NETCOM_ACQ_ENT_1);
                    // rob vt
                    com_netComClient.OpenStream(NETCOM_ACQ_ENT_2);

                    // Send request for check streaming reply
                    RepeatSend('C', 0, true);
                    robotStreaming = true;
                    PrintAction("[Main] DONE Robot Recieved Confirm Streaming Command");

                    // Wait for initial move to command from Matlab
                    while (r2c_packLast[CharInd('M', r2c_id)] != r2c_packLast[CharInd('D', r2c_id)] && !(doQuit || doExit)) ;
                    PrintAction("[Main] DONE Robot Recieved MoveTo Start Quiadrant Command");

                    // Wait for setup command from Matlab
                    while (r2c_packLast[CharInd('S', r2c_id)] != r2c_packLast[CharInd('D', r2c_id)] && !(doQuit || doExit)) ;
                    PrintAction("[Main] DONE Robot Recieved Setup Command");

                    // While connected
                    while (com_netComClient.AreWeConnected() && !(doQuit || doExit)) ;

                    // Close down everything
                    PrintAction("[Main] EXITING.........");

                    // Move back to start pos
                    if (!doExit)
                    {
                        matIn_dat1 = CalcMove(4.7124 - 0.9714);
                        RepeatSend('M', 0, true);
                        PrintAction("[Main] DONE Robot Finished MovedTo South Quiadrant");
                    }

                    // Wait 1 second
                    Thread.Sleep(1000);

                    // Send command for ICR_GUI to exit
                    com_Matlab.Execute("shouldExit = true;");

                    // Wait for GUI to close
                    while (!doExit) ;

                    //// Stop recording aquisition
                    string reply = " ";
                    com_netComClient.SendCommand("-StopRecording", ref reply);
                    com_netComClient.SendCommand("-StopAcquisition", ref reply);

                    // Close NetCom sreams
                    PrintAction("[Main] CLOSE NetCom");
                    com_netComClient.CloseStream(NETCOM_ACQ_ENT_1);
                    com_netComClient.CloseStream(NETCOM_ACQ_ENT_2);

                    // Disconect from NetCom
                    while (com_netComClient.AreWeConnected())
                    {
                        com_netComClient.DisconnectFromServer();
                    }

                    // Send command for arduino to quit
                    RepeatSend('Q');
                    PrintAction("[Main] DONE Robot Recieved Quit Command");

                    // Close port
                    //PrintAction("[Main] CLOSE Xbee Serial Port");
                    //sp_Xbee.DiscardInBuffer();
                    //sp_Xbee.DiscardOutBuffer();
                    //Thread.Sleep(1000);
                    //sp_Xbee.Close();

                    // Dispose of workers
                    bw_RunGUI.Dispose();
                    bw_MatCOM.Dispose();

                    // Quit matlab 
                    com_Matlab.Quit();
                    PrintAction("[Main] CLOSE MatCOM COM");
                    PrintAction("FINISHED");
                    Thread.Sleep(1000);
                    //Console.ReadKey();
                    return;

                }
            }
        }

        #region ---------COMMUNICATION---------

        private static bool RepeatSend(char id, short pack = 0, bool check_done = false)
        {
            bool pass_rcvd = false;
            long resend_tim = sw_main.ElapsedMilliseconds + 100;

            // Send new data with new packet number
            if (pack == 0)
            {
                pack = SendData(id);
            }
            // Check mesage was recieved
            while (!pass_rcvd && !doExit)
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
                // Need to resend
                if (!pass_rcvd && sw_main.ElapsedMilliseconds > resend_tim)
                {
                    SendData(id, pack);
                    resend_tim = sw_main.ElapsedMilliseconds + 500;
                }
            }

            // Check command was completed/done
            if (check_done)
            {
                while (!doExit)
                {
                    // Find last done command packet number for given mesage
                    for (int i = r2c_idHist.Count - 1; i >= 0; i--)
                    {
                        while (r2c_idHist.Count != r2c_packHist.Count) ;
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
            }
            // got mesage confirm
            else return true;
            // forced quit
            return false;
        }

        private static void CheckDone(short pack)
        {
            // Print worker started
            string msg_str;
            msg_str = String.Format("[CheckDone] RUN CheckDone Worker (pack:{0})", pack);
            PrintAction(msg_str);

            // Initialize
            long t_resend_check = sw_main.ElapsedMilliseconds + 10000;
            int msg_cnt = r2c_idHist.Count;
            int send_cnt = 1;
            int max_send = 5;
            bool pass = true;

            // Spend 10 sec checking for done confirmation resend
            while (sw_main.ElapsedMilliseconds < t_resend_check && (!doExit || !doQuit))
            {
                if (r2c_idHist.Count > msg_cnt)
                {
                    // update mesage count
                    msg_cnt = r2c_idHist.Count;

                    // Wait for list update complete
                    while (r2c_idHist.Count != r2c_packHist.Count) ;

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
                        t_resend_check = sw_main.ElapsedMilliseconds + 10000;
                    }
                }
                // Pause thread
                Thread.Sleep(100);
            }
            msg_str = String.Format("[CheckDone] DONE CheckDone Worker {0} (pack:{1})", pass ? "Passed" : "Failed", pack);
            PrintAction(msg_str);
        }

        public static short SendData(char id, short pack = 0)
        {

            lock (lock_sendData)
            {

                // Block vt sending
                vtBlocker.Block(id);

                // Wait for next safe send time
                long t_send;
                do
                {
                    // delay send time till x ms after last send and rcvd
                    t_send = t_c2r > t_r2c ? t_c2r + t_c2rDel : t_r2c + t_c2rDel;
                } while (sw_main.ElapsedMilliseconds < t_send);

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
                int nDataBytes = 0;

                // Send setup data
                if (id == 'S')
                {
                    nDataBytes = 2;
                    msg_data = new byte[nDataBytes];
                    // Add session cond
                    msg_data[0] = (byte)matIn_dat1;
                    // Add tone cond
                    msg_data[1] = (byte)matIn_dat2;
                }

                // Send MoveTo data
                else if (id == 'M')
                {
                    nDataBytes = 4;
                    msg_data = new byte[nDataBytes];
                    // Add move pos
                    u.f = (float)matIn_dat1;
                    msg_data[0] = u.b1;
                    msg_data[1] = u.b2;
                    msg_data[2] = u.b3;
                    msg_data[3] = u.b4;
                }

                // Send Reward data
                else if (id == 'R')
                {
                    nDataBytes = 4;
                    msg_data = new byte[nDataBytes];
                    // Add move pos
                    u.f = (float)matIn_dat1;
                    msg_data[0] = u.b1;
                    msg_data[1] = u.b2;
                    msg_data[2] = u.b3;
                    msg_data[3] = u.b4;
                }

                // Send robot halt data
                if (id == 'H')
                {
                    nDataBytes = 1;
                    msg_data = new byte[nDataBytes];
                    // Add halt state
                    msg_data[0] = (byte)matIn_dat1;
                }

                // Send bulldoze rat data
                if (id == 'B')
                {
                    nDataBytes = 2;
                    msg_data = new byte[nDataBytes];
                    // Add bull del
                    msg_data[0] = (byte)matIn_dat1;
                    // Add bull speed
                    msg_data[1] = (byte)matIn_dat2;
                }

                // Send start/end pid data
                if (id == 'I')
                {
                    nDataBytes = 1;
                    msg_data = new byte[nDataBytes];
                    // Add session cond
                    msg_data[0] = (byte)matIn_dat1;
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
                    if (nDataBytes == 0)
                    {
                        msg_str = String.Format("   Sent: [id:{0} pack:{1}]", id, pack);
                    }
                    else msg_str = String.Format("   Sent: [id:{0} dat1:{1:0.00} dat2:{2:0.00} pack:{3}]", id, matIn_dat1, matIn_dat2, pack);
                    PrintAction(msg_str, t_c2rLast, t_c2r);
                }
                else if (printSentVt)
                {
                    msg_str = String.Format("   Sent: [id:{0} ent:{1} ts:{2} cm:{3:0.00} pack:{4}]", id, vtEnt, vtTS[vtEnt, 1], vtCM, pack);
                    PrintAction(msg_str, t_c2rLast, t_c2r);
                }

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

                // Unlock vt sending
                vtBlocker.Unblock(id);

                // Return packet number 
                return pack;

            }

        }

        private static void DataReceived_Xbee(object sender, SerialDataReceivedEventArgs e)
        {
            UnionHack u = new UnionHack(0, 0, 0, '0', 0);
            bool head_found = false;
            bool id_found = false;
            bool pack_found = false;
            bool foot_found = false;
            bool for_ard = false;
            byte[] head_rcvd = new byte[1];
            byte[] id_rcvd = new byte[1];
            byte[] pack_rcvd = new byte[2];
            byte[] foot_rcvd = new byte[1];
            char head = ' ';
            char id = ' ';
            char foot = ' ';
            short pack = 0;
            string msg_str;

            // Find header
            while (!head_found && sp_Xbee.BytesToRead > 0)
            {
                sp_Xbee.Read(head_rcvd, 0, 1);
                // Get header
                u.b1 = head_rcvd[0];
                u.b2 = 0;
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
            while (head_found && !id_found && sp_Xbee.BytesToRead > 0)
            {
                sp_Xbee.Read(id_rcvd, 0, 1);
                // Get id
                u.b1 = id_rcvd[0];
                u.b2 = 0;
                id = u.c1;
                //
                for (int i = 0; i < r2c_id.Length; i++)
                {
                    if (id == r2c_id[i])
                    {
                        id_found = true;
                    }
                }
            }

            // Get first and second part of packet number
            if (head_found && id_found)
            {
                // Get first byte of packet number
                while (sp_Xbee.BytesToRead < 1) ; // wait
                sp_Xbee.Read(pack_rcvd, 0, 2);
                u.b1 = pack_rcvd[0];
                while (sp_Xbee.BytesToRead < 1) ; // wait
                u.b2 = pack_rcvd[1];
                pack = u.s1;

                // Check if pack matches sent pack or done related id
                for (int i = c2r_packHist.Count - 1; i >= 0; i--)
                {
                    while (c2r_packHist.Count != c2r_idHist.Count) ;
                    if (
                        c2r_packHist[i] == pack &&
                        (c2r_idHist[i] == id ||
                        id == 'D')
                        )
                    {
                        pack_found = true;
                    }
                }
            }

            // Find footer
            if (head_found && id_found && pack_found)
            {
                while (!foot_found && sp_Xbee.BytesToRead > 0)
                {
                    sp_Xbee.Read(foot_rcvd, 0, 1);
                    // Get header
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
            if (head_found && id_found && pack_found && foot_found)
            {
                // Update recieve time
                t_r2cLast = t_r2c;
                t_r2c = sw_main.ElapsedMilliseconds;

                // print data recieved
                msg_str = String.Format("   Rsvd: [id:{0} pack:{1}]", id, pack);
                PrintAction(msg_str, t_r2cLast, t_r2c);

                // Update last pack
                r2c_packLast[CharInd(id, r2c_id)] = pack;
                // Update list
                r2c_idHist.Add(id);
                r2c_packHist.Add(pack);
            }
            // Dump
            else if (!for_ard)
            {
                droppedPacks++;
                msg_str = String.Format("!!PACK LOST!! (Total:{0})", droppedPacks);
                PrintAction(msg_str);
                // dump input buffer
                //sp_Xbee.DiscardInBuffer();
            }
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
            else PrintAction("      vt blocked");
        }

        [STAThread]
        private static void DoWork_RunGUI(object sender, DoWorkEventArgs e)
        {
            PrintAction("[DoWork_RunGUI] RUN RunGUI Worker");
            object result = null;
            if (!doExit && !doQuit)
            {
                com_Matlab.Execute(@"addpath(genpath('C:\Users\lester\MeDocuments\AppData\MATLAB\Code'));");
                com_Matlab.Execute(@"addpath(genpath('C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\MATLAB\ICR_ARENA'));");
                com_Matlab.Execute(@"cd C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\MATLAB\ICR_ARENA");
                PrintAction("[DoWork_RunGUI] BEGIN ICR_GUI");
                com_Matlab.Feval("ICR_GUI", 0, out result);
                PrintAction("[DoWork_RunGUI] FINISHED ICR_GUI");
            }
            e.Result = " ";
        }

        private static void RunWorkerCompleted_RunGUI(object sender, RunWorkerCompletedEventArgs e)
        {
            PrintAction("[RunWorkerCompleted_RunGUI] DONE RunGUI Worker");
            doExit = true;
        }

        [STAThread]
        private static void DoWork_MatCOM(object sender, DoWorkEventArgs e)
        {
            PrintAction("[DoWork_MatCOM] RUN MatCOM Worker");
            BackgroundWorker worker = (BackgroundWorker)sender;

            Tuple<char, double, double> bw_args = (Tuple<char, double, double>)e.Argument;
            //Put the arguments into nicely named variables:
            char bw_id = bw_args.Item1;
            double bw_d1 = bw_args.Item2;
            double bw_d2 = bw_args.Item3;
            bool new_dat;

            // Initalize Matlab global vars
            com_Matlab.PutWorkspaceData("m2c_id", "global", 'N');
            com_Matlab.PutWorkspaceData("m2c_dat1", "global", 9999.0);
            com_Matlab.PutWorkspaceData("m2c_dat2", "global", 9999.0);
            com_Matlab.PutWorkspaceData("m2c_flag", "global", 0.0);

            while (!doQuit && !doExit)
            {
                // Get flag
                var flag = com_Matlab.GetVariable("m2c_flag", "global");
                while (IsEmptyMat(flag)) flag = com_Matlab.GetVariable("m2c_flag", "global");
                flag = (double)flag;
                new_dat = flag > 0 ? true : false;

                // Check for new command
                if (new_dat)
                {
                    PrintAction("[DoWork_MatCOM] RUN Get New MatCom Data");
                    // Get new message id
                    var id = com_Matlab.GetVariable("m2c_id", "global");
                    while (IsEmptyMat(id)) id = com_Matlab.GetVariable("m2c_id", "global");
                    bw_id = System.Convert.ToChar(id);

                    // Get new data
                    // data1
                    var d1 = com_Matlab.GetVariable("m2c_dat1", "global");
                    while (IsEmptyMat(d1)) d1 = com_Matlab.GetVariable("m2c_dat1", "global");
                    if (d1 != 9999)
                    {
                        bw_d1 = (double)d1;
                    }
                    else bw_d1 = Double.NaN;
                    // data2
                    var d2 = com_Matlab.GetVariable("m2c_dat2", "global");
                    while (IsEmptyMat(d2)) d2 = com_Matlab.GetVariable("m2c_dat2", "global");
                    if (d2 != 9999)
                    {
                        bw_d2 = (double)d2;
                    }
                    else bw_d2 = Double.NaN;

                    // Reset flag
                    com_Matlab.Execute("m2c_flag = 0;");

                    // Trigger progress change event
                    worker.ReportProgress(0, new System.Tuple<char, double, double>(bw_id, bw_d1, bw_d2));
                }
            }
            // end polling
            e.Result = " ";
        }

        private static void ProgressChanged_MatCOM(object sender, ProgressChangedEventArgs e)
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
            PrintAction(msg_str, t_m2cLast, t_m2c);

            bool do_send = false;
            bool do_check_done = false;

            // Relay all matlab data to arduino except quit
            if (matIn_id == 'Q') // quit
            {
                // end polling
                doQuit = true;
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
                    if (matIn_id == 'R') // move to
                    {
                        if (matIn_dat1 > 0)
                        {
                            // calculate move to pos
                            matIn_dat1 = CalcMove(matIn_dat1);
                        }
                        //do_check_done = true;
                    }

                    RepeatSend(matIn_id, 0, do_check_done);
                }
            }
        }

        private static void RunWorkerCompleted_MatCOM(object sender, RunWorkerCompletedEventArgs e)
        {
            PrintAction("[RunWorkerCompleted_MatCOM] DONE MatCOM Worker");
            // Set to exit if robot has not started streaming
            if (!robotStreaming) doExit = true;
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

        private static void PrintAction(string msg_str, long t1 = -1, long t2 = -1)
        {
            lock (lock_printState)
            {
                // Initialize
                float t;

                // No input time
                if (t1 < 0)
                {
                    if (t_sync == 0) t = 0;
                    else t = (float)(sw_main.ElapsedMilliseconds - t_sync) / 1000.0f;
                    Console.Write("\n" + msg_str + " ({0:0.000}sec)\n", t);
                }
                // Use input time
                else
                {
                    if (t_sync == 0) t = 0;
                    else t = (float)(t2 - t_sync) / 1000.0f;
                    long dt = t1 > 0 ? t2 - t1 : 0;
                    Console.Write("\n" + msg_str + " (dt:{0}ms tot:{1:0.000}sec)\n", dt, t);

                }
            }
        }

        #endregion

    }

    class VT_Blocker
    {
        static readonly object lockBlock = new object();
        private static int threadCnt = 0;
        private static Stopwatch sw = new Stopwatch();
        private static long t_blockTim = 0;
        private static long blockFor = 60; // (ms)

        private static bool _isBlocked;
        public bool isBlocked
        { get { return _isBlocked; } }

        public VT_Blocker()
        {
            sw.Start();
        }

        public void Block(char id)
        {
            if (id != 'P')
            {
                lock (lockBlock)
                {
                    threadCnt++;
                    _isBlocked = true;
                    t_blockTim = sw.ElapsedMilliseconds + blockFor;
                }
            }
        }

        public void Unblock(char id)
        {
            if (id != 'P')
            {
                while (sw.ElapsedMilliseconds < t_blockTim) ;
                lock (lockBlock)
                {
                    threadCnt--;
                    if (threadCnt < 1)
                    {
                        _isBlocked = false;
                    }
                }
            }
        }
    }

}