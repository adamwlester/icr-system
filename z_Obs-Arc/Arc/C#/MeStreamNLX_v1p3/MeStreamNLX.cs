using System;
using MNetCom;
using System.ComponentModel;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using Microsoft.VisualBasic.FileIO;
using System.Collections;
using MLApp;

namespace MeStreamNLX
{
    class MeStreamNLX
    {

        // To exit program smoothly
        private static bool shouldExit = false;

        // Initialize callback object
        private static MNetCom.MNC_VTCallback mNetcomVTCallback;

        // Initialize serial port object to Xbee
        private static System.IO.Ports.SerialPort serialPortXbee = new System.IO.Ports.SerialPort();

        // Initialize MATLAB COM object
        private static bool runGUI = false;
        private static MLApp.MLApp comPortMat = new MLApp.MLApp();

        // Serial communication
        private static string msgMatNow; // matlab now
        private static string msgMatLast; // matlab last
        private static char[] msg2ard_id = new char[] { // prefix giving masage id
            'S', // start session
            'P', // position data
            'R', // dispense reward
            'M', // move to position
            'H', // halt movement
            'Q', // quit session
             };
        private static char[] char_head = new char[] { '<', '_' }; // 2 byte header
        private static char char_foot = '>'; // 1 byte header
        private static byte[] msg2ard_head = new byte[2];
        private static byte[] msg2ard_foot = new byte[1];
        private static byte[] msg2ard_packNum = new byte[] {0};


        // Position variables
        private static double X_CENT = 359.5553;
        private static double Y_CENT = 260.2418;
        private static double RADIUS = 179.4922;
        private static byte vt_ent = 0;
        private static short vt_rec = (short)0;
        private static float vt_x = 0;
        private static float vt_y = 0;
        private static float[,] vt_rad = new float[2, 2];
        private static float vt_cm;
        private static ulong vt_tsStr = 0;
        private static int[,] vt_ts = new int[2, 2];
        private static float vt_vel;

        // PID tuning simulation
        private static bool doSim = true;
        private static String csvFi = "simRatPos.csv"; // simConstVel_40.csv simRatPos.csv
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
            // Hide matlab app window
            comPortMat.Visible = 0;

            // Create header and footer byte array
            UnionHack u = new UnionHack(0, 0, 0, '0', 0);
            // get header chars
            u.c1 = char_head[0];
            u.c2 = char_head[1];
            // use only first byte 
            // becasue arduino uses uni-8
            msg2ard_head[0] = u.b1;
            msg2ard_head[1] = u.b3;
            // Get footer byte 
            u.c1 = char_foot;
            msg2ard_foot[0] = u.b1;

            //// Set serialPortXbee parameters
            //serialPortXbee.ReadTimeout = 1000;
            //serialPortXbee.BaudRate = 57600;
            //serialPortXbee.PortName = "COM22";
            //serialPortXbee.Open();

            //char id = msg2ard_id[1];
            //vt_rec = (short)1;
            //vt_ent = 1;
            //vt_ts[vt_ent, 1] = 10000;
            //vt_cm = 300f;

            //sendData(msg2ard_id[0]);
            //for (int i = 0; i < 998; i++)
            //{
            //    // Load list

            //    sendData(id);
            //    Thread.Sleep(33);
            //    vt_rec++;
            //    vt_ts[vt_ent, 1] += 33;
            //}
            //sendData(msg2ard_id[5]);
            //return;

            // Start Cheetah if it is not already running
            while (!IsProcessOpen("Cheetah"))
            {
                OpenCheetah("Cheetah.cfg");
            }

            // Initialize ICR_GUI background worker
            var bw_rungui = new BackgroundWorker();
            bw_rungui.DoWork += do_work_rungui;
            bw_rungui.RunWorkerCompleted += run_worker_completed_rungui;
            if (runGUI)
            {
                bw_rungui.RunWorkerAsync(); // start worker
            }

            // Initialize MATLAB com background worker
            var bw_matcom = new BackgroundWorker();
            bw_matcom.DoWork += do_work_matcom;
            bw_matcom.RunWorkerCompleted += run_worker_completed_matcom;

            // Initilize deligate for VT callback
            MNetComClient mNetComClient = new MNetComClient();
            mNetcomVTCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            mNetComClient.SetCallbackFunctionVT(mNetcomVTCallback, new MeStreamNLX());

            // Set serialPortXbee parameters
            serialPortXbee.ReadTimeout = 1000;
            serialPortXbee.BaudRate = 57600;
            serialPortXbee.PortName = "COM22";
            // Open serial port connection
            serialPortXbee.Open();

            // Set NetCom parameters
            var NETCOM_APP_ID = "MeStreamNLX"; // string displayed in Cheetah when connected
            var NETCOM_ACQ_ENT_1 = "VT1"; // aquisition entity to stream
            var NETCOM_ACQ_ENT_2 = "VT2"; // aquisition entity to stream
            var NETCOM_IP = "127.0.0.1"; // host computer IP

            // Wait for ICR_GUI to connect to NLX fist
            if (runGUI)
            {
                comPortMat.Execute(@"global cMesage;");
                comPortMat.Execute(@"cMesage = 'None';");
                while ((string)comPortMat.GetVariable("cMesage", "global") != "matlabConnected") ;

                // Start backround matcom worker
                bw_matcom.RunWorkerAsync();
            }

            // Load CSV data
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
                    if (!runGUI)
                    {
                        string nlxCommand = "-StartAcquisition";
                        string reply = " ";
                        mNetComClient.SendCommand(nlxCommand, ref reply);
                    }

                    // Start arduino exicution
                    sendData(msg2ard_id[0]);

                    // While connected
                    while (!shouldExit && mNetComClient.AreWeConnected()) ;

                    // Close down everything
                    Console.WriteLine("Exiting.........");

                    // Close NetCom sreams
                    mNetComClient.CloseStream(NETCOM_ACQ_ENT_1);
                    mNetComClient.CloseStream(NETCOM_ACQ_ENT_2);

                    // Disconect from NetCom
                    while (mNetComClient.AreWeConnected())
                    {
                        mNetComClient.DisconnectFromServer();
                    }

                    // Stop arduino exicution
                    sendData(msg2ard_id[5]);

                    // Close port
                    serialPortXbee.Close();

                    // Dispose of workers
                    bw_rungui.Dispose();
                    bw_matcom.Dispose();

                    // Quit matlab 
                    comPortMat.Quit();
                    return;

                }
            }
        }

        public static void sendData(char id)
        {

            UnionHack u = new UnionHack(0, 0, 0, '0', 0);

            msg2ard_packNum[0]++;
            if (msg2ard_packNum[0] > 254) msg2ard_packNum[0] = 1;

            // Save ID
            byte[] msg_id = new byte[1];
            u.c1 = id;
            msg_id[0] = u.b1;

            // Will store message data
            byte[] msg_data = null;
            int nDataBytes = 0;

            // Send pos data
            if (id == msg2ard_id[1])
            {
                nDataBytes = 11;
                msg_data = new byte[nDataBytes];
                // Add vt_rec
                u.s1 = vt_rec;
                msg_data[0] = u.b1;
                msg_data[1] = u.b2;
                // Add vt_ent byte
                msg_data[2] = vt_ent;
                // Add vt_ts int 
                u.i = vt_ts[vt_ent, 1];
                msg_data[3] = u.b1;
                msg_data[4] = u.b2;
                msg_data[5] = u.b3;
                msg_data[6] = u.b4;
                // Add vt_cm float 
                u.f = vt_cm;
                msg_data[7] = u.b1;
                msg_data[8] = u.b2;
                msg_data[9] = u.b3;
                msg_data[10] = u.b4;

            }

            // Concatinate header and footer
            byte[] msgByteArr = new byte[
                msg2ard_head.Length +    // header
                msg2ard_packNum.Length + // packet num
                msg_id.Length +          // id
                nDataBytes +             // data
                msg2ard_foot.Length      // footer
                ];
            // add header
            msg2ard_head.CopyTo(msgByteArr, 0);
            // add packet number
            msg2ard_packNum.CopyTo(msgByteArr, msg2ard_head.Length);
            // add id
            msg_id.CopyTo(msgByteArr, msg2ard_head.Length + msg2ard_packNum.Length);
            // add data
            if (nDataBytes > 0)
            {
                msg_data.CopyTo(msgByteArr, msg2ard_head.Length + msg2ard_packNum.Length + msg_id.Length);
            }
            // add footer
            msg2ard_foot.CopyTo(msgByteArr, msg2ard_head.Length + msg2ard_packNum.Length + msg_id.Length + nDataBytes);

            // Send to arduino
            try
            {
                serialPortXbee.Write(msgByteArr, 0, msgByteArr.Length);
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine(ex.GetType().FullName);
                Console.WriteLine(ex.Message);
            }
            Console.WriteLine(msgByteArr.Length);

            string str;
            str = String.Format("{0} {1} {2} {3} {4} {5} {6}\n", msgByteArr.Length, msg2ard_packNum[0], id, vt_rec, vt_ent, vt_ts[vt_ent, 1], vt_cm);
            Console.Write(str);

        }

        public static void NetComCallbackVT(object sender, MNetCom.MVideoRec records, int numRecords, string objectName)
        {
            // Compute position
            CompPos(records.swid, records.qwTimeStamp, records.dnextracted_x, records.dnextracted_y);
            // Send data
            sendData(msg2ard_id[1]);

            // Send sim data for each VT2 rec
            if (doSim)
            {
                UpdateSimVT();
            }

        }

        public static void CompPos(ushort ent, ulong ts, double x, double y)
        {

            // Get first vt_ts once
            if (vt_tsStr == 0)
            {
                vt_tsStr = ts;
            }

            // Get record vt_ent
            vt_ent = (byte)ent;

            // Itterate record number
            vt_rec++;

            // Save old vals
            vt_ts[vt_ent, 0] = vt_ts[vt_ent, 1];
            vt_rad[vt_ent, 0] = vt_rad[vt_ent, 1];

            // Convert to ms and update vt_ts
            vt_ts[vt_ent, 1] = (int)Math.Round((double)((ts - vt_tsStr) / 1000));

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
            double dt = (vt_ts[vt_ent, 1] - vt_ts[vt_ent, 0]) / 1000;
            double v = Math.Abs(rad - vt_rad[vt_ent, 0]) *
                ((140 * Math.PI) / (2 * Math.PI)) /
                dt;

            // Convert rad to cm
            double radFlip = Math.Abs(rad - (2 * Math.PI)); // flip
            double cm = radFlip * ((140 * Math.PI) / (2 * Math.PI)); // convert

            // Update vars
            vt_x = (float)x;
            vt_y = (float)y;
            vt_rad[vt_ent, 1] = (float)rad;
            vt_vel = (float)v;
            vt_cm = (float)cm;

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

        public static void GetSimCSV()
        {
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
            if (simStrTS == 0) simStrTS = vt_tsStr;
            // Check if need to restart TS simulation data
            if (simIndTS == nSimSamp - 1)
            {
                // Restart pos data sampling
                simIndTS = 0;
                // Update start time
                simStrTS = simStrTS + (ulong)vt_ts[0, 1] * 1000 + (ulong)(vt_ts[0, 1] - vt_ts[0, 0]) * 1000;

            }
            // Check if need to restart Pos simulation data
            if (simIndPos == nSimSamp - 1)
                simIndPos = 0;

            // Run CompPos with new values
            CompPos(0, simArrTS[simIndTS] + simStrTS, simArrX[simIndPos], simArrY[simIndPos]);
            // Send data
            sendData(msg2ard_id[1]);

            simIndTS++;
            if (simSampCnt > 30 * 5)
                simIndPos++;
            simSampCnt++;
        }

        private static void run_worker_completed_rungui(object sender, RunWorkerCompletedEventArgs e)
        {
            shouldExit = true;
        }

        [STAThread]
        private static void do_work_rungui(object sender, DoWorkEventArgs e)
        {
            //System.Array mat = new double[2];
            //mat.SetValue(11, 0);
            //mat.SetValue(12, 1);
            //comPortMat.PutFullMatrix("a", "base", mat, mat);
            object result = null;
            //comPortMat.Execute("a = 2;");
            if (!shouldExit)
            {
                comPortMat.Execute(@"addpath(genpath('C:\Users\lester\MeDocuments\AppData\MATLAB\Code'));");
                comPortMat.Execute(@"addpath(genpath('C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\MATLAB\ICR_ARENA'));");
                comPortMat.Execute(@"cd C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\MATLAB\ICR_ARENA");
                comPortMat.Feval("ICR_GUI", 1, out result);
                //shouldExit = (bool)result;
            }
            shouldExit = true;
            e.Result = " ";
        }

        private static void run_worker_completed_matcom(object sender, RunWorkerCompletedEventArgs e)
        {
            shouldExit = true;
        }

        [STAThread]
        private static void do_work_matcom(object sender, DoWorkEventArgs e)
        {

            while (!shouldExit)
            {
                msgMatLast = msgMatNow;
                //Console.ReadKey();
                msgMatNow = (string)comPortMat.GetVariable("cMesage", "global");
                if (msgMatNow == "quit")
                {
                    e.Result = " ";
                }
                else if (msgMatNow != msgMatLast)
                {
                    //Console.WriteLine(msgMatNow);
                }
            }
        }

    }
}