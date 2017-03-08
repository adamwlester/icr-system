using System;
using MNetCom;
using System.ComponentModel;
using System.IO.Ports;
using System.Threading;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Collections;

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
        private static MLApp.MLApp comPortMat = new MLApp.MLApp();

        // Message strings
        private static string msgMatNow; // matlab now
        private static string msgMatLast; // matlab last
        private static char[] msg2ard_id = new char[] { // prefix giving masage id
            'P', // position
            'S', // start movement
            'H', // halt movement
            'R', // dispense reward
             };
        private static char[] msg2ard_head = new char[] { '<', '_' }; // 2 byte header
        private static char msg2ard_foot = '>'; // 1 byte header


        // Position variables
        private static double X_CENT = 359.5553;
        private static double Y_CENT = 260.2418;
        private static double RADIUS = 179.4922;
        private static byte vt_ent = 0;
        private static short vt_rec = (short)0;
        private static float vt_x = 0;
        private static float vt_y = 0;
        private static float[,] vt_rad = new float[2, 2];
        private static ulong vt_tsStr = 0;
        private static int[,] vt_ts = new int[2, 2];
        private static float vt_vel;

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
            //// Set serialPortXbee parameters
            //serialPortXbee.ReadTimeout = 1000;
            //serialPortXbee.BaudRate = 57600;
            //serialPortXbee.PortName = "COM22";
            //serialPortXbee.Open();

            //char id = msg2ard_id[0];
            //vt_rec = (short)1;
            //vt_ent = 1;
            //vt_ts[vt_ent, 1] = 1000;
            //vt_rad[vt_ent, 1] = 3.26f;

            //while (true)
            //{
            //    // Load list

            //    sendData(id);
            //    Thread.Sleep(33);
            //    vt_rec++;
            //    vt_ts[vt_ent, 1] += 33;
            //}
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
            bw_rungui.RunWorkerAsync(); // start worker

            // Initialize MATLAB com background worker
            var bw_matcom = new BackgroundWorker();
            bw_matcom.DoWork += do_work_matcom;
            bw_matcom.RunWorkerCompleted += run_worker_completed_matcom;
            bw_matcom.RunWorkerAsync(); // start worker

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
            var NETCOM_APP_ID = "MeStreamNLX"; // vt_ent string displayed in Cheetah when connected
            var NETCOM_ACQ_ENT_1 = "VT1"; // vt_ent for aquisition entity to stream
            var NETCOM_ACQ_ENT_2 = "VT2"; // vt_ent for aquisition entity to stream
            var NETCOM_IP = "127.0.0.1"; // host computer IP

            // Pause to let ICR_GUI connect to NLX fist
            comPortMat.Execute(@"global cMesage;");
            comPortMat.Execute(@"cMesage = 'None';");
            while ((string)comPortMat.GetVariable("cMesage", "global") != "matlabConnected") ;

            // Setup connection
            if (!(mNetComClient.AreWeConnected()))
            {
                if (mNetComClient.ConnectToServer(NETCOM_IP))
                {
                    // Begin stream and set app name
                    mNetComClient.SetApplicationName(NETCOM_APP_ID);
                    mNetComClient.OpenStream(NETCOM_ACQ_ENT_1);
                    mNetComClient.OpenStream(NETCOM_ACQ_ENT_2);

                    // While connected
                    while (mNetComClient.AreWeConnected())
                    {
                        if ((string)comPortMat.GetVariable("cMesage", "global") == "quit")
                        {
                            shouldExit = true;
                        }

                        if (!shouldExit)
                        {
                            msgMatLast = msgMatNow;
                            //Console.ReadKey();
                            msgMatNow = (string)comPortMat.GetVariable("cMesage", "global");
                            if (msgMatNow != msgMatLast)
                            {
                                //Console.WriteLine(msgMatNow);
                            }
                        }

                        // Close down everything
                        else
                        {

                            Console.WriteLine("Exiting.........");
                            // Close NetCom sreams
                            mNetComClient.CloseStream(NETCOM_ACQ_ENT_1);
                            mNetComClient.CloseStream(NETCOM_ACQ_ENT_2);
                            // Disconect from NetCom
                            while (mNetComClient.AreWeConnected())
                            {
                                mNetComClient.DisconnectFromServer();
                            }
                            // Dispose of workers
                            bw_rungui.Dispose();
                            bw_matcom.Dispose();
                            // Quit matlab 
                            comPortMat.Quit();
                            break;
                        }

                    }
                }
            }
        }

        public static void NetComCallbackVT(object sender, MNetCom.MVideoRec records, int numRecords, string objectName)
        {
            // Compute position
            CompPos(records.swid, records.qwTimeStamp, records.dnextracted_x, records.dnextracted_y);

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

            // Update vars
            vt_x = (float)x;
            vt_y = (float)y;
            vt_rad[vt_ent, 1] = (float)rad;
            vt_vel = (float)v;

        }

        public static void sendData(char id)
        {
      
            UnionHack u = new UnionHack(0, 0, 0, '0', 0);

            if (id == msg2ard_id[0])
            {
                byte[] msgByteArr = new byte[15];
                // Add char data first

                // Add header chars
                u.c1 = msg2ard_head[0];
                u.c2 = msg2ard_head[1];
                // send only first byte 
                // becasue arduino uses uni-8
                msgByteArr[0] = u.b1;
                msgByteArr[1] = u.b3;
                // Add id and footer
                u.c1 = id;
                u.c2 = msg2ard_foot;
                msgByteArr[2] = u.b1;
                msgByteArr[msgByteArr.Length-1] = u.b3;
                // Add vt_rec
                u.s1 = vt_rec;
                msgByteArr[3] = u.b1;
                msgByteArr[4] = u.b2;
                Console.WriteLine(u.s1);
                // Add vt_ent byte
                msgByteArr[5] = vt_ent;
                // Add vt_ts int 
                u.i = vt_ts[vt_ent, 1];
                msgByteArr[6] = u.b1;
                msgByteArr[7] = u.b2;
                msgByteArr[8] = u.b3;
                msgByteArr[9] = u.b4;
                // Add vt_rad float 
                u.f = vt_rad[vt_ent, 1];
                msgByteArr[10] = u.b1;
                msgByteArr[11] = u.b2;
                msgByteArr[12] = u.b3;
                msgByteArr[13] = u.b4;

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

            }

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
            Thread.Sleep(1000000);
        }

    }
}