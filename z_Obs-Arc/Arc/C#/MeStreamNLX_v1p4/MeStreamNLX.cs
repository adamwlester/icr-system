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

        // To quit exit program smoothly
        private static bool doQuit = false;
        private static bool doExit = false;

        // Create stop watch object
        private static Stopwatch stopWatch = new Stopwatch();

        // Initialize callback object
        private static MNetCom.MNC_VTCallback mNetcomVTCallback;

        // Initialize serial port object to Xbee
        private static System.IO.Ports.SerialPort serialPortXbee = new System.IO.Ports.SerialPort();

        // Initialize MATLAB COM object
        private static MLApp.MLApp comPortMat = new MLApp.MLApp();

        // Matlab communication
        private static char[] mat2cs_id = new char[] { // prefix giving masage id
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // dispense reward
            'H', // halt movement
            'N', // matlab not loaded
            'L', // matlab loaded
             };
        private static char cmdMatNow = mat2cs_id[5]; // matlab now
        private static char cmdMatLast = cmdMatNow; // matlab last
        private static double datMatOut_1; // matlab data
        private static double datMatOut_2; // matlab data

        // Arduino serial communication
        private static char[] cs2rob_id = new char[] {
            'S', // start session
            'Q', // quit session
            'M', // move to position
            'R', // dispense reward
            'H', // halt movement
            'P', // position data
             };
        private static char[] rob2cs_id = new char[] {
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
        private static short short_packNum = 0; // packet number
        private static byte[] cs2rob_head = new byte[2];
        private static byte[] cs2rob_foot = new byte[1];
        private static byte[] cs2rob_packNum = new byte[2];
        private static List<char> listRcvd_id = new List<char>();
        private static List<short> listRcvd_pack = new List<short>();


        // Position variables
        private static float movePos;
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


        // Entry point for script
        static void Main()
        {
            PrintState("BEGIN RUN.........");
            // Hide matlab app window
            comPortMat.Visible = 0;

            // Start stopwatch
            stopWatch.Start();

            // Create header and footer byte array
            UnionHack u = new UnionHack(0, 0, 0, '0', 0);
            // get header chars
            u.c1 = char_head[0];
            u.c2 = char_head[1];
            // use only first byte 
            // becasue arduino uses uni-8
            cs2rob_head[0] = u.b1;
            cs2rob_head[1] = u.b3;
            // Get footer byte 
            u.c1 = char_foot;
            cs2rob_foot[0] = u.b1;

            //// Set serialPortXbee parameters
            //serialPortXbee.ReadTimeout = 1000;
            //serialPortXbee.WriteTimeout = 1000;
            //serialPortXbee.BaudRate = 111111;
            //serialPortXbee.PortName = "COM16";
            //serialPortXbee.Open();
            //// Create event handeler for incoming data
            //serialPortXbee.DataReceived += event_xbee_data_received;

            //char id = cs2rob_id[3];
            //vtRec = (short)1;
            //vtEnt = 1;
            //vtTS[vtEnt, 1] = 10000;
            //vtCM = 420.0f;

            //for (int i = 0; i < 998; i++)
            //{
            //    // Load list

            //    SendData(id);
            //    Thread.Sleep(10);
            //    vtRec++;
            //    vtTS[vtEnt, 1] += 33;
            //}
            //SendData(cs2rob_id[1]);
            //return;

            // Initialize ICR_GUI background worker
            var bw_rungui = new BackgroundWorker();
            bw_rungui.DoWork += do_work_rungui;
            bw_rungui.RunWorkerCompleted += run_worker_completed_rungui;
            bw_rungui.RunWorkerAsync(); // start worker

            // Start Cheetah if it is not already running
            while (!IsProcessOpen("Cheetah"))
            {
                OpenCheetah("Cheetah.cfg");
            }
            PrintState("CHEETAH OPENED");

            // Initialize MATLAB com background worker
            var bw_matcom = new BackgroundWorker();
            bw_matcom.DoWork += do_work_matcom;
            bw_matcom.RunWorkerCompleted += run_worker_completed_matcom;

            // Initilize deligate for VT callback
            MNetComClient mNetComClient = new MNetComClient();
            mNetcomVTCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            mNetComClient.SetCallbackFunctionVT(mNetcomVTCallback, new MeStreamNLX());

            // Set serialPortXbee parameters
            serialPortXbee.ReadTimeout = 100;
            serialPortXbee.BaudRate = 111111;
            serialPortXbee.PortName = "COM16";
            // Create event handeler for incoming data
            serialPortXbee.DataReceived += event_xbee_data_received;
            // Open serial port connection
            serialPortXbee.Open();
            PrintState("XBEE PORT OPENED");

            // Set NetCom parameters
            var NETCOM_APP_ID = "MeStreamNLX"; // string displayed in Cheetah when connected
            var NETCOM_ACQ_ENT_1 = "VT1"; // aquisition entity to stream
            var NETCOM_ACQ_ENT_2 = "VT2"; // aquisition entity to stream
            var NETCOM_IP = "127.0.0.1"; // host computer IP

            // Initalize Matlab vars
            comPortMat.Execute(@"global cmdMatOut;");

            // Initalize cmdMatOut to not loaded
            string cmdStr = String.Format("cmdMatOut = {0};", mat2cs_id[5]);
            comPortMat.Execute(cmdStr);

            // Wait for ICR_GUI to load
            while (cmdMatNow != mat2cs_id[6])
            {
                var cmd = comPortMat.GetVariable("cmdMatOut", "global");
                if (cmd is string)
                {
                    cmdMatNow = System.Convert.ToChar(cmd);
                }
            }

            // Start backround matcom worker
            bw_matcom.RunWorkerAsync();
            PrintState("GUI LOADED");

            // Load CSV data for simulation
            if (doSim)
            {
                GetSimCSV();
            }

            // Setup connection
            if (!(mNetComClient.AreWeConnected()))
            {
                if (mNetComClient.ConnectToServer(NETCOM_IP))
                {

                    // Begin stream and set app name
                    mNetComClient.SetApplicationName(NETCOM_APP_ID);
                    if (!doSim)
                    {
                        mNetComClient.OpenStream(NETCOM_ACQ_ENT_1);
                    }
                    mNetComClient.OpenStream(NETCOM_ACQ_ENT_2);
                    PrintState("NLX STREAMING");

                    // Wait for streaming established cmd
                    char first_cmd = ' ';
                    while (first_cmd != rob2cs_id[6] && !(doQuit || doExit))
                    {
                        first_cmd = (listRcvd_id.Count != 0) ? listRcvd_id[listRcvd_id.Count - 1] : ' ';
                    }
                    // Check if never revieved first command
                    if (first_cmd != rob2cs_id[6])
                    {
                        listRcvd_id.Add('N');
                        listRcvd_pack.Add(0);
                    }
                    PrintState("ROB STREAMING");

                    // Wait for initial move to command from Matlab
                    while (listRcvd_id[listRcvd_id.Count - 1] != rob2cs_id[2] && !(doQuit || doExit)) ;
                    PrintState("ROB RECIEVED MOVED TO SQ");

                    // Wait for start (rat in) command from Matlab
                    while (listRcvd_id[listRcvd_id.Count - 1] != rob2cs_id[0] && !(doQuit || doExit)) ;
                    PrintState("ROB SETUP");

                    // While connected
                    while (mNetComClient.AreWeConnected() && !(doQuit || doExit)) ;

                    // Close down everything
                    PrintState("Exiting.........");

                    // Move back to start pos
                    CalcMove(3.9124);
                    RepeatSend(rob2cs_id[2]);
                    PrintState("ROB RECIEVED MOVED TO DEFAULT");

                    // Wait for movement to finish
                    bool pass = false;
                    while (!pass && !doExit)
                    {
                        // Check for done command
                        if (listRcvd_id[listRcvd_id.Count - 1] == rob2cs_id[5])
                        {
                            short done_pack = listRcvd_pack[listRcvd_id.Count - 1];
                            // Search for matching move command packet
                            for (int i = 0; i < listRcvd_id.Count; i++)
                            {
                                if (listRcvd_id[i] == rob2cs_id[2] && listRcvd_pack[i] == done_pack)
                                {
                                    pass = true;
                                }
                            }
                        }
                    }
                    PrintState("ROB MOVED TO DEFAULT");

                    //// Stop recording aquisition
                    //string reply = " ";
                    //mNetComClient.SendCommand("-StopRecording", ref reply);
                    //mNetComClient.SendCommand("-StopAcquisition", ref reply);

                    // Send command for ICR_GUI to exit
                    comPortMat.Execute("cmdMatIn = 'E';");

                    // Close NetCom sreams
                    mNetComClient.CloseStream(NETCOM_ACQ_ENT_1);
                    mNetComClient.CloseStream(NETCOM_ACQ_ENT_2);

                    // Disconect from NetCom
                    while (mNetComClient.AreWeConnected())
                    {
                        mNetComClient.DisconnectFromServer();
                    }
                    PrintState("NETCOM CLOSED");

                    // Close port
                    serialPortXbee.Close();
                    PrintState("XBEE PORT CLOSED");

                    // Dispose of workers
                    bw_rungui.Dispose();
                    bw_matcom.Dispose();

                    // Quit matlab 
                    comPortMat.Quit();
                    PrintState("MATCOM PORT CLOSED");
                    PrintState("FINISHED RUN.........");
                    //Console.ReadKey();
                    return;

                }
            }
        }

        // PRINT STATUS
        private static void PrintState(string str)
        {
            float t = (float)(stopWatch.ElapsedMilliseconds) / 1000.0f;
            Console.Write("\n" + str + " ({0:0.00} sec)\n", t);
        }

        private static void RepeatSend(char cmd)
        {
            short pack_num = SendData(cmd);
            bool pass = false;
            int cnt = 0;
            while (!pass)
            {
                // Search for matching move command packet
                for (int i = listRcvd_id.Count - 1; i >= 0; i--)
                {
                    if (listRcvd_pack[i] == pack_num)
                    {
                        pass = true;
                        return;
                    }
                }
                cnt++;
                // Terminate program after 20 itterations
                if (cnt >= 20)
                {
                    //doExit = true;
                    return;
                }
                // Resend
                Thread.Sleep(250);
                SendData(cmd, pack_num);
            }
        }

        public static short SendData(char id, short optPackNum = 0)
        {

            UnionHack u = new UnionHack(0, 0, 0, '0', 0);
            short pack_num;

            // Get new packet number
            if (optPackNum == 0)
            {
                short_packNum++;
                pack_num = short_packNum;
            }
            // use old packet number
            else pack_num = optPackNum;

            // Store pack number
            u.s1 = pack_num;
            cs2rob_packNum[0] = u.b1;
            cs2rob_packNum[1] = u.b2;

            // Store ID
            byte[] msg_id = new byte[1];
            u.c1 = id;
            msg_id[0] = u.b1;

            // Will store message data
            byte[] msg_data = null;
            int nDataBytes = 0;

            // Send setup data
            if (id == cs2rob_id[0])
            {
                nDataBytes = 2;
                msg_data = new byte[nDataBytes];
                // Add session cond
                msg_data[0] = (byte)datMatOut_1;
                // Add tone cond
                msg_data[1] = (byte)datMatOut_2;
            }

            // Send MoveTo data
            else if (id == cs2rob_id[2])
            {
                nDataBytes = 4;
                msg_data = new byte[nDataBytes];
                // Add move pos
                u.f = movePos;
                msg_data[0] = u.b1;
                msg_data[1] = u.b2;
                msg_data[2] = u.b3;
                msg_data[3] = u.b4;
            }

            // Send pos data
            else if (id == cs2rob_id[5])
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
                cs2rob_head.Length +    // header
                cs2rob_packNum.Length + // packet num
                msg_id.Length +          // id
                nDataBytes +             // data
                cs2rob_foot.Length      // footer
                ];
            // add header
            cs2rob_head.CopyTo(msgByteArr, 0);
            // add packet number
            cs2rob_packNum.CopyTo(msgByteArr, cs2rob_head.Length);
            // add id
            msg_id.CopyTo(msgByteArr, cs2rob_head.Length + cs2rob_packNum.Length);
            // add data
            if (nDataBytes > 0)
            {
                msg_data.CopyTo(msgByteArr, cs2rob_head.Length + cs2rob_packNum.Length + msg_id.Length);
            }
            // add footer
            cs2rob_foot.CopyTo(msgByteArr, cs2rob_head.Length + cs2rob_packNum.Length + msg_id.Length + nDataBytes);

            // Track round-trip mesage time
            if (id != cs2rob_id[5])
            {
                float t = (float)(stopWatch.ElapsedMilliseconds / 1000.0f);
                Console.WriteLine("   Sent: [id:{0} pack:{1} ({2:0.00} sec)]", id, pack_num, t);
            }

            // Send to arduino
            serialPortXbee.Write(msgByteArr, 0, msgByteArr.Length);

            // Return packet number 
            return pack_num;

            //string str;
            //str = String.Format("Bytes: {0}  Pack: {1}  ID: {2}  Rec: {3}  Ent: {4}  TS: {5}  CM: {6}\n", msgByteArr.Length, cs2rob_packNum[0], id, vtRec, vtEnt, vtTS[vtEnt, 1], vtCM);
            //Console.Write(str);

        }

        public static void CalcMove(double pos_rad)
        {
            // Convert from rad to CM
            double flip_rad = Math.Abs(pos_rad - (2 * Math.PI));
            movePos = (float)(flip_rad * ((140 * Math.PI) / (2 * Math.PI)));
        }

        public static void NetComCallbackVT(object sender, MNetCom.MVideoRec records, int numRecords, string objectName)
        {
            // Compute position
            CompPos(records.swid, records.qwTimeStamp, records.dnextracted_x, records.dnextracted_y);
            // Send data
            SendData(cs2rob_id[5]);

            Thread.Sleep(10);
            // Send sim data for each VT2 rec
            if (doSim)
            {
                UpdateSimVT();
            }

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

        private static void event_xbee_data_received(object sender, SerialDataReceivedEventArgs e)
        {
            UnionHack u = new UnionHack(0, 0, 0, '0', 0);
            bool use_msg = false;
            byte[] id_rcvd = new byte[1];
            byte[] pack_rcvd = new byte[2];
            char id = ' ';
            short pack;

            // Check message is intended for CS
            while (!use_msg && serialPortXbee.BytesToRead > 0)
            {
                serialPortXbee.Read(id_rcvd, 0, 1);
                // Get id
                u.b1 = id_rcvd[0];
                u.b2 = 0;
                id = u.c1;
                for (int i = 0; i < rob2cs_id.Length; i++)
                {
                    if (id == rob2cs_id[i])
                    {
                        use_msg = true;
                    }
                }
            }
            if (use_msg && serialPortXbee.BytesToRead == 2)
            {
                // Get packet number
                serialPortXbee.Read(pack_rcvd, 0, 2);
                u.b1 = pack_rcvd[0];
                u.b2 = pack_rcvd[1];
                pack = u.s1;

                // Update list
                listRcvd_id.Add(id);
                listRcvd_pack.Add(pack);
                float t = (float)(stopWatch.ElapsedMilliseconds / 1000.0f);
                Console.WriteLine("   Rsvd: [id:{0} pack:{1} ({2:0.00} sec)]", id, pack, t);
            }
            // Dump
            else serialPortXbee.DiscardInBuffer();
        }

        [STAThread]
        private static void do_work_rungui(object sender, DoWorkEventArgs e)
        {
            object result = null;
            if (!doExit && !doQuit)
            {
                comPortMat.Execute(@"addpath(genpath('C:\Users\lester\MeDocuments\AppData\MATLAB\Code'));");
                comPortMat.Execute(@"addpath(genpath('C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\MATLAB\ICR_ARENA'));");
                comPortMat.Execute(@"cd C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\MATLAB\ICR_ARENA");
                PrintState("ICR_GUI RUNNING");
                comPortMat.Feval("ICR_GUI", 1, out result);
                PrintState("ICR_GUI FINISHED");
            }
            e.Result = " ";
        }

        private static void run_worker_completed_rungui(object sender, RunWorkerCompletedEventArgs e)
        {
            PrintState("WORKER RUNGUI FINISHED");
            doExit = true;
        }

        [STAThread]
        private static void do_work_matcom(object sender, DoWorkEventArgs e)
        {

            while (!doExit)
            {
                cmdMatLast = cmdMatNow;

                // Get new message
                var cmd = comPortMat.GetVariable("cmdMatOut", "global");
                while (cmd.GetType() != typeof(string))
                {
                    cmd = comPortMat.GetVariable("cmdMatOut", "global");
                }
                cmdMatNow = System.Convert.ToChar(cmd);

                // Check for new command
                if (cmdMatNow != cmdMatLast)
                {
                    var dat1 = (double)comPortMat.GetVariable("datMatOut_1", "global");
                    if (dat1 != 9999)
                    {
                        datMatOut_1 = (double)dat1;
                    }
                    var dat2 = (double)comPortMat.GetVariable("datMatOut_2", "global");
                    if (dat1 != 9999)
                    {
                        datMatOut_2 = (double)dat2;
                    }

                    // Relay all matlab data to arduino except quit
                    if (cmdMatNow == mat2cs_id[1]) // quit
                    {
                        doQuit = true;
                    }
                    else
                    {
                        if (cmdMatNow == mat2cs_id[2]) // move to
                        {
                            // calculate move to pos
                            CalcMove(datMatOut_1);
                        }

                        // Send to arduino and wait for recieved confirmation
                        RepeatSend(cmdMatNow);
                    }
                }
            }
            // end polling
            e.Result = " ";
        }

        private static void run_worker_completed_matcom(object sender, RunWorkerCompletedEventArgs e)
        {
            PrintState("WORKER MATCOM FINISHED");
            doExit = true;
        }

        public static void GetSimCSV()
        {
            // Symulate Matlab move to command
            string cmdStr = String.Format("CS_Send(D, {0}, {1});", mat2cs_id[2], simMovePos);
            comPortMat.Execute(cmdStr);
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
            SendData(cs2rob_id[5]);

            simIndTS++;
            if (simSampCnt > 30 * 5)
                simIndPos++;
            simSampCnt++;
        }

    }
}