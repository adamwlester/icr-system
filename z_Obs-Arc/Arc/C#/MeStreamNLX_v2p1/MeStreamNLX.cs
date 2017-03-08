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

        // Create stop watch object
        private static Stopwatch stopWatch = new Stopwatch();

        // Initialize callback object
        private static MNetCom.MNC_VTCallback deligate_netComCallback;

        // Initialize serial port object to Xbee
        private static System.IO.Ports.SerialPort sp_Xbee = new System.IO.Ports.SerialPort();

        // Initialize MATLAB COM object
        private static MLApp.MLApp com_Matlab = new MLApp.MLApp();

        // Matlab communication
        private static char[] m2c_id = new char[] { // prefix giving masage id
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // dispense reward
            'H', // halt movement
            'N', // matlab not loaded
            'L', // matlab loaded
             };
        private static char matIn_id = m2c_id[5]; // matlab now
        private static double matIn_dat1; // matlab data
        private static double matIn_dat2; // matlab data

        // Arduino serial communication
        private static char[] c2r_id = new char[8] {
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // dispense reward
            'H', // halt movement
            'P', // position data
            'C', // request stream status
            'Z', // request done resend
             };
        private static char[] r2c_id = new char[7] {
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // dispense reward
            'H', // halt movement
            'D', // execution done
            'C', // connected and streaming
             };
        private static char[] char_head = new char[] { '<', '_' }; // 2 byte header
        private static char char_foot = '>'; // 1 byte header
        private static short cnt_packNum = 0; // packet number
        private static byte[] c2r_head = new byte[2];
        private static byte[] c2r_foot = new byte[1];
        private static byte[] c2r_packNum = new byte[2];
        private static List<char> r2c_idHist = new List<char>();
        private static List<short> r2c_packHist = new List<short>();
        private static short[] c2r_packLast = new short[r2c_id.Length];

        // Position variables
        private static double X_CENT = 359.5553;
        private static double Y_CENT = 260.2418;
        private static double RADIUS = 179.4922;
        private static byte vtEnt = 0;
        private static short vtRec = (short)0;
        private static float vtX = 0;
        private static float vtY = 0;
        private static float[,] vtRad = new float[2, 2];
        private static float vtCM;
        private static ulong vtStr = 0;
        private static int[,] vtTS = new int[2, 2];
        private static float vtVel;

        // PID tuning simulation
        private static bool doSim = false;
        private static double simMovePos = Math.PI; // + Math.PI/2;
        private static String csvFi = "simConstVel_0.csv"; // simConstVel_40.csv simRatPos.csv
        private static String csvPath =
            @"C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\C#\Testing\" + csvFi;
        private static TextFieldParser parser = new TextFieldParser(csvPath);
        private static int simIndTS = 0;
        private static int simIndPos = 0;
        private static int nSimSamp = 30 * 60 * 4;
        private static int simSampCnt = 0;
        private static ulong simStrTS = 0;
        private static ulong[] simArrTS = new ulong[nSimSamp];
        private static float[] simArrX = new float[nSimSamp];
        private static float[] simArrY = new float[nSimSamp];

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
            PrintState("[Main] BEGIN Main");
            // Hide matlab app window
            com_Matlab.Visible = 0;

            // Start stopwatch
            stopWatch.Start();

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
            r2c_idHist.Add('N');
            r2c_packHist.Add(0);

            // Initialize last packet array
            for (int i = 0; i < c2r_packLast.Length; i++)
                c2r_packLast[i] = (short)(i * -1);

            // Initialize ICR_GUI background worker
            var bw_RunGUI = new BackgroundWorker();
            bw_RunGUI.DoWork += DoWork_RunGUI;
            bw_RunGUI.RunWorkerCompleted += RunWorkerCompleted_RunGUI;
            PrintState("[Main] RUN RunGUI Worker");
            bw_RunGUI.RunWorkerAsync(); // start worker

            // Start Cheetah if it is not already running
            PrintState("[Main] OPEN Cheetah");
            while (!IsProcessOpen("Cheetah"))
            {
                OpenCheetah("Cheetah.cfg");
            }

            // Crate argument vars for backround worker
            char bw_id = ' ';
            double bw_d1 = 0;
            double bw_d2 = 0;
            var bw_args = Tuple.Create(bw_id, bw_d1, bw_d2);

            // Initialize MATLAB com background worker
            var bw_MatCOM = new BackgroundWorker();
            bw_MatCOM.DoWork += DoWork_MatCOM;
            bw_MatCOM.ProgressChanged += ProgressChanged_MatCOM;
            bw_MatCOM.RunWorkerCompleted += RunWorkerCompleted_MatCOM;
            bw_MatCOM.WorkerReportsProgress = true;

            // Initilize deligate for VT callback
            MNetComClient com_netComClient = new MNetComClient();
            deligate_netComCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            com_netComClient.SetCallbackFunctionVT(deligate_netComCallback, new MeStreamNLX());

            // Set sp_Xbee parameters
            sp_Xbee.ReadTimeout = 100;
            sp_Xbee.BaudRate = 111111;
            sp_Xbee.PortName = "COM16";
            // Create event handeler for incoming data
            sp_Xbee.DataReceived += DataReceived_Xbee;
            // Open serial port connection
            PrintState("[Main] OPEN Xbee Serial Port");
            sp_Xbee.Open();

            // Set NetCom parameters
            var NETCOM_APP_ID = "MeStreamNLX"; // string displayed in Cheetah when connected
            var NETCOM_ACQ_ENT_1 = "VT1"; // aquisition entity to stream
            var NETCOM_ACQ_ENT_2 = "VT2"; // aquisition entity to stream
            var NETCOM_IP = "127.0.0.1"; // host computer IP

            // Start backround MatCOM worker
            PrintState("[Main] RUN MatCOM Worker");
            bw_MatCOM.RunWorkerAsync(bw_args);

            // Wait for ICR_GUI to load
            while (matIn_id != m2c_id[6]) ;
            PrintState("[Main] DONE ICR_GUI Loaded");


            // Load CSV data for simulation
            if (doSim)
            {
                GetSimCSV();
            }

            // Setup connection
            if (!(com_netComClient.AreWeConnected()))
            {
                if (com_netComClient.ConnectToServer(NETCOM_IP))
                {

                    // Begin stream and set app name
                    PrintState("[Main] BEGIN Nlx Stream");
                    com_netComClient.SetApplicationName(NETCOM_APP_ID);
                    if (!doSim)
                    {
                        com_netComClient.OpenStream(NETCOM_ACQ_ENT_1);
                    }
                    com_netComClient.OpenStream(NETCOM_ACQ_ENT_2);

                    // Send request for check streaming reply
                    RepeatSend(c2r_id[CharInd('C', r2c_id)], 0, true);
                    PrintState("[Main] DONE Robot Recieved Confirm Streaming Command");

                    // Wait for initial move to command from Matlab
                    while (c2r_packLast[CharInd('M', r2c_id)] != c2r_packLast[CharInd('D', r2c_id)] && !(doQuit || doExit)) ;
                    PrintState("[Main] DONE Robot Recieved MoveTo Start Quiadrant Command");

                    // Wait for start (rat in) command from Matlab
                    while (c2r_packLast[CharInd('S', r2c_id)] != c2r_packLast[CharInd('D', r2c_id)] && !(doQuit || doExit)) ;
                    PrintState("[Main] DONE Robot Recieved Setup Command");

                    // While connected
                    while (com_netComClient.AreWeConnected() && !(doQuit || doExit)) ;

                    // Close down everything
                    PrintState("[Main] EXITING.........");

                    // Move back to start pos
                    if (!doExit)
                    {
                        matIn_dat1 = CalcMove(3.9124);
                        RepeatSend(c2r_id[2], 0, true);
                        PrintState("[Main] DONE Robot Finished MovedTo South Quiadrant");
                    }

                    // Send command for arduino to quit
                    RepeatSend(c2r_id[CharInd('Q', r2c_id)]);
                    PrintState("[Main] DONE Robot Recieved Quit Command");

                    // Send command for ICR_GUI to exit
                    com_Matlab.Execute("cmdMatIn = 'E';");

                    // Wait for GUI to close
                    while (!doExit) ;

                    //// Stop recording aquisition
                    string reply = " ";
                    com_netComClient.SendCommand("-StopRecording", ref reply);
                    com_netComClient.SendCommand("-StopAcquisition", ref reply);

                    // Close NetCom sreams
                    PrintState("[Main] CLOSE NetCom");
                    com_netComClient.CloseStream(NETCOM_ACQ_ENT_1);
                    com_netComClient.CloseStream(NETCOM_ACQ_ENT_2);

                    // Disconect from NetCom
                    while (com_netComClient.AreWeConnected())
                    {
                        com_netComClient.DisconnectFromServer();
                    }

                    // Close port
                    PrintState("[Main] CLOSE Xbee Serial Port");
                    sp_Xbee.Close();

                    // Dispose of workers
                    bw_RunGUI.Dispose();
                    bw_MatCOM.Dispose();

                    // Quit matlab 
                    com_Matlab.Quit();
                    PrintState("[Main] CLOSE MatCOM COM");
                    PrintState("FINISHED");
                    //Console.ReadKey();
                    return;

                }
            }
        }

        #region ---------COMMUNICATION---------

        private static bool RepeatSend(char id, short pack_num = 0, bool check_done = false)
        {
            bool pass_rcvd = false;
            bool pass_done = false;
            long resend_tim = stopWatch.ElapsedMilliseconds + 250;
            long check_done_tim;
            if (pack_num == 0)
            {
                pack_num = SendData(id);
            }
            while (!(pass_rcvd && pass_done) && !doExit)
            {

                // Check mesage was revieved
                if (!pass_rcvd)
                {
                    // Search for matching command packet
                    for (int i = r2c_packHist.Count - 1; i >= 0; i--)
                    {
                        if (r2c_packHist[i] == pack_num)
                        {
                            // Message recieved
                            pass_rcvd = true;
                            continue;
                        }
                    }
                    // Need to resend
                    if (!pass_rcvd && stopWatch.ElapsedMilliseconds > resend_tim)
                    {
                        SendData(id, pack_num);
                        resend_tim = stopWatch.ElapsedMilliseconds + 250;
                    }
                }

                // Check command was completed done
                if (check_done)
                {
                    if (pass_rcvd && !pass_done)
                    {
                        check_done_tim = stopWatch.ElapsedMilliseconds + 1000;
                        // Check till time elapsed
                        while (stopWatch.ElapsedMilliseconds < check_done_tim)
                        {
                            while (r2c_idHist.Count != r2c_packHist.Count) ;
                            // Find last done command packet number for given mesage
                            for (int i = r2c_packHist.Count - 1; i >= 0; i--)
                            {
                                // Check match for current command
                                if (r2c_idHist[i] == r2c_id[5] && r2c_packHist[i] == pack_num)
                                {
                                    pass_done = true;
                                    return true; // got mesage confirm and done confirm
                                }
                            }
                        }
                        // Send done resend request
                        SendData(c2r_id[7], pack_num);
                    }
                }
                else return true; // got mesage confirm
            }
            return false; // forced quit
        }

        public static short SendData(char id, short pack_num = 0)
        {

            UnionHack u = new UnionHack(0, 0, 0, '0', 0);

            // Get new packet number
            if (pack_num == 0)
            {
                cnt_packNum++;
                pack_num = cnt_packNum;
            }

            // Store pack number
            u.s1 = pack_num;
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
            if (id == c2r_id[0])
            {
                nDataBytes = 2;
                msg_data = new byte[nDataBytes];
                // Add session cond
                msg_data[0] = (byte)matIn_dat1;
                // Add tone cond
                msg_data[1] = (byte)matIn_dat2;
            }

            // Send MoveTo data
            else if (id == c2r_id[2])
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

            // Send pos data
            else if (id == c2r_id[5])
            {
                nDataBytes = 11;
                msg_data = new byte[nDataBytes];
                // Add vtRec
                u.s1 = vtRec;
                msg_data[0] = u.b1;
                msg_data[1] = u.b2;
                // Add vtEnt byte
                msg_data[2] = vtEnt;
                // Add vtTS int 
                u.i = vtTS[vtEnt, 1];
                msg_data[3] = u.b1;
                msg_data[4] = u.b2;
                msg_data[5] = u.b3;
                msg_data[6] = u.b4;
                // Add vtCM float 
                u.f = vtCM;
                msg_data[7] = u.b1;
                msg_data[8] = u.b2;
                msg_data[9] = u.b3;
                msg_data[10] = u.b4;

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

            // Print sent data
            if (id != c2r_id[5])
            {
                string str;
                if (nDataBytes == 0)
                {
                    str = String.Format("   Sent: [id:{0} pack:{1}]", id, pack_num);
                }
                else str = String.Format("   Sent: [id:{0} dat1:{1:0.00} dat2:{2:0.00} pack:{3}]", id, matIn_dat1, matIn_dat2, pack_num);
                PrintState(str);
            }

            // Send to arduino
            sp_Xbee.Write(msgByteArr, 0, msgByteArr.Length);

            // Return packet number 
            return pack_num;

            //string str;
            //str = String.Format("Bytes: {0}  Pack: {1}  ID: {2}  Rec: {3}  Ent: {4}  TS: {5}  CM: {6}\n", msgByteArr.Length, c2r_packNum[0], id, vtRec, vtEnt, vtTS[vtEnt, 1], vtCM);
            //Console.Write(str);

        }

        public static void CompPos(ushort ent, ulong ts, double x, double y)
        {

            // Get first vtTS once
            if (vtStr == 0)
            {
                vtStr = ts;
            }

            // Get record vtEnt
            vtEnt = (byte)ent;

            // Itterate record number
            vtRec++;

            // Save old vals
            vtTS[vtEnt, 0] = vtTS[vtEnt, 1];
            vtRad[vtEnt, 0] = vtRad[vtEnt, 1];

            // Convert to ms and update vtTS
            vtTS[vtEnt, 1] = (int)Math.Round((double)((ts - vtStr) / 1000));

            // Rescale y as VT data is compressed in y axis
            y = y * 1.0976;

            // Normalize 
            x = (x - X_CENT) / RADIUS;
            y = (y - Y_CENT) / RADIUS;

            // Flip y 
            y = y * -1;

            // Compute radians
            double rad = Math.Atan2(y, x);

            // Convert radians to range between [0, 2*pi]
            if (rad < 0)
            {
                rad = rad + 2 * Math.PI;
            }

            // Convert back to pixels with lower left = 0
            x = Math.Round(x * RADIUS) + RADIUS;
            y = Math.Round(y * RADIUS) + RADIUS;

            // Convert cart to cm
            x = x * (140 / (RADIUS * 2));
            y = y * (140 / (RADIUS * 2));

            // Compute velocity (cm/sec)
            double dt = (vtTS[vtEnt, 1] - vtTS[vtEnt, 0]) / 1000;
            double v = Math.Abs(rad - vtRad[vtEnt, 0]) *
                ((140 * Math.PI) / (2 * Math.PI)) /
                dt;

            // Convert rad to cm
            double radFlip = Math.Abs(rad - (2 * Math.PI)); // flip
            double cm = radFlip * ((140 * Math.PI) / (2 * Math.PI)); // convert

            // Update vars
            vtX = (float)x;
            vtY = (float)y;
            vtRad[vtEnt, 1] = (float)rad;
            vtVel = (float)v;
            vtCM = (float)cm;

        }

        #endregion

        #region ---------CALLBACKS AND WORKERS---------

        public static void NetComCallbackVT(object sender, MNetCom.MVideoRec records, int numRecords, string objectName)
        {
            // Compute position
            CompPos(records.swid, records.qwTimeStamp, records.dnextracted_x, records.dnextracted_y);
            // Send data
            SendData(c2r_id[5]);

            Thread.Sleep(10);
            // Send sim data for each VT2 rec
            if (doSim)
            {
                UpdateSimVT();
            }

        }

        private static void DataReceived_Xbee(object sender, SerialDataReceivedEventArgs e)
        {
            UnionHack u = new UnionHack(0, 0, 0, '0', 0);
            bool use_msg = false;
            byte[] id_rcvd = new byte[1];
            byte[] pack_rcvd = new byte[2];
            char id = ' ';
            short pack;

            // Check message is intended for CS
            while (!use_msg && sp_Xbee.BytesToRead > 0)
            {
                sp_Xbee.Read(id_rcvd, 0, 1);
                // Get id
                u.b1 = id_rcvd[0];
                u.b2 = 0;
                id = u.c1;
                for (int i = 0; i < r2c_id.Length; i++)
                {
                    if (id == r2c_id[i])
                    {
                        use_msg = true;
                    }
                }
            }
            if (use_msg && sp_Xbee.BytesToRead == 2)
            {
                // Get packet number
                sp_Xbee.Read(pack_rcvd, 0, 2);
                u.b1 = pack_rcvd[0];
                u.b2 = pack_rcvd[1];
                pack = u.s1;

                // Print data recieved
                string str = String.Format("   Rsvd: [id:{0} pack:{1}]", id, pack);
                PrintState(str);

                // Update list
                r2c_idHist.Add(id);
                r2c_packHist.Add(pack);
                // Update last pack
                c2r_packLast[CharInd(id, r2c_id)] = pack;
            }
            // Dump
            else sp_Xbee.DiscardInBuffer();
        }

        [STAThread]
        private static void DoWork_RunGUI(object sender, DoWorkEventArgs e)
        {
            PrintState("[DoWork_RunGUI] RUN RunGUI Worker");
            object result = null;
            if (!doExit && !doQuit)
            {
                com_Matlab.Execute(@"addpath(genpath('C:\Users\lester\MeDocuments\AppData\MATLAB\Code'));");
                com_Matlab.Execute(@"addpath(genpath('C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\MATLAB\ICR_ARENA'));");
                com_Matlab.Execute(@"cd C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\MATLAB\ICR_ARENA");
                PrintState("[DoWork_RunGUI] BEGIN ICR_GUI");
                com_Matlab.Feval("ICR_GUI", 1, out result);
                PrintState("[DoWork_RunGUI] FINISHED ICR_GUI");
            }
            e.Result = " ";
        }

        private static void RunWorkerCompleted_RunGUI(object sender, RunWorkerCompletedEventArgs e)
        {
            PrintState("[RunWorkerCompleted_RunGUI] DONE RunGUI Worker");
            doExit = true;
        }

        [STAThread]
        private static void DoWork_MatCOM(object sender, DoWorkEventArgs e)
        {
            PrintState("[DoWork_MatCOM] RUN MatCOM Worker");
            BackgroundWorker worker = (BackgroundWorker)sender;

            Tuple<char, double, double> bw_args = (Tuple<char, double, double>)e.Argument;
            //Put the arguments into nicely named variables:
            char bw_id = bw_args.Item1;
            double bw_d1 = bw_args.Item2;
            double bw_d2 = bw_args.Item3;
            char id_last;

            // Initalize Matlab global vars
            com_Matlab.PutWorkspaceData("cmdMatOut", "global", m2c_id[5]);
            com_Matlab.PutWorkspaceData("datMatOut_1", "global", 9999.00);
            com_Matlab.PutWorkspaceData("datMatOut_2", "global", 9999.00);

            while (!doQuit)
            {
                id_last = bw_id;

                // Get new message
                var cmd = com_Matlab.GetVariable("cmdMatOut", "global");

                // Wait till var is not empty
                while (IsEmptyMat(cmd)) cmd = com_Matlab.GetVariable("cmdMatOut", "global");
                bw_id = System.Convert.ToChar(cmd);

                // Check for new command
                if (bw_id != id_last)
                {
                    var tmp_bw_d1 = com_Matlab.GetVariable("datMatOut_1", "global");
                    bw_d2 = Double.NaN;
                    if (!IsEmptyMat(tmp_bw_d1))
                    {
                        if (tmp_bw_d1 != 9999)
                        {
                            bw_d1 = (double)tmp_bw_d1;
                        }
                    }
                    var tmp_bw_d2 = com_Matlab.GetVariable("datMatOut_2", "global");
                    bw_d2 = Double.NaN;
                    if (!IsEmptyMat(tmp_bw_d2))
                    {
                        if (tmp_bw_d2 != 9999)
                        {
                            bw_d2 = (double)tmp_bw_d2;
                        }
                    }

                    string str = String.Format("   mCOM: [id:{0} dat1:{1:0.00} dat2:{2:0.00}]", bw_id, bw_d1, bw_d2);
                    PrintState(str);

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

            bool do_send = false;
            bool do_check_done = false;

            // Relay all matlab data to arduino except quit
            if (matIn_id == m2c_id[1]) // quit
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
                    if (matIn_id == m2c_id[2]) // move to
                    {
                        // calculate move to pos
                        matIn_dat1 = CalcMove(matIn_dat1);
                        do_check_done = true;
                    }

                    RepeatSend(matIn_id, 0, do_check_done);
                }
            }
        }

        private static void RunWorkerCompleted_MatCOM(object sender, RunWorkerCompletedEventArgs e)
        {
            PrintState("[RunWorkerCompleted_MatCOM] DONE MatCOM Worker");
            doQuit = true;
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

        public static double CalcMove(double pos_rad)
        {
            // Convert from rad to CM
            double flip_rad = Math.Abs(pos_rad - (2 * Math.PI));
            double cm = (flip_rad * ((140 * Math.PI) / (2 * Math.PI)));
            return cm;
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

        private static void PrintState(string str)
        {
            float t = (float)(stopWatch.ElapsedMilliseconds) / 1000.0f;
            Console.Write("\n" + str + " ({0:0.00} sec)\n", t);
        }

        public static void GetSimCSV()
        {
            // Symulate Matlab move to command
            string cmdStr = String.Format("CS_Send(D, {0}, {1});", m2c_id[2], simMovePos);
            com_Matlab.Execute(cmdStr);
            Console.WriteLine(cmdStr);

            // Create parser object
            parser.TextFieldType = FieldType.Delimited;
            parser.SetDelimiters(",");

            // Get header which gives n samples
            string[] header = parser.ReadFields();
            nSimSamp = (int)float.Parse(header[0]);

            // initialize ts vars
            double tsStr = 0;
            double ts;

            for (int i = 0; i < nSimSamp; i++)
            {
                // get start ts for subtraction
                string[] fields = parser.ReadFields();
                if (i == 0)
                {
                    tsStr = double.Parse(fields[0]);
                    ts = tsStr;
                }
                else
                {
                    ts = double.Parse(fields[0]);
                }
                // Add csv eliments
                simArrTS[i] = (ulong)(ts - tsStr);
                simArrX[i] = float.Parse(fields[1]);
                simArrY[i] = float.Parse(fields[2]);
                //string str;
                //str = String.Format("{0} {1} {2} \n", simArrTS[i], simArrX[i], simArrY[i]);
                //Console.Write(str);

            }

        }

        public static void UpdateSimVT()
        {
            // Check if ts start needs to be updated
            if (simStrTS == 0) simStrTS = vtStr;
            // Check if need to restart TS simulation data
            if (simIndTS == nSimSamp - 1)
            {
                // Restart pos data sampling
                simIndTS = 0;
                // Update start time
                simStrTS = simStrTS + (ulong)vtTS[0, 1] * 1000 + (ulong)(vtTS[0, 1] - vtTS[0, 0]) * 1000;

            }
            // Check if need to restart Pos simulation data
            if (simIndPos == nSimSamp - 1)
                simIndPos = 0;

            // Run CompPos with new values
            CompPos(0, simArrTS[simIndTS] + simStrTS, simArrX[simIndPos], simArrY[simIndPos]);
            // Send data
            SendData(c2r_id[5]);

            simIndTS++;
            if (simSampCnt > 30 * 5)
                simIndPos++;
            simSampCnt++;
        }

        #endregion

    }
}