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
        private static ArrayList msgList = new ArrayList();
        private static string msgMatNow; // matlab now
        private static string msgMatLast; // matlab last
        private static string msg2ard; // VT pos
        private static string[] msg2ard_key = new string[] { // prefix giving masage type
            "PS", // position
            "SM", // start movement
            "EM", // halt movement
            "RW", // dispense reward
             };


        // Position variables
        private static double X_CENT = 359.5553;
        private static double Y_CENT = 260.2418;
        private static double RADIUS = 179.4922;
        private static byte vt_id = 0;
        private static short[] vt_rec = new short[2];
        private static float vt_x = 0;
        private static float vt_y = 0;
        private static float[,] vt_rad = new float[2, 2];
        private static ulong vt_tsStr = 0;
        private static ulong[,] vt_ts = new ulong[2, 2];
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
            public char c1; // (float) 4 byte
            [FieldOffset(2)]
            public char c2; // (char) 1 byte
            [FieldOffset(0)]
            public byte b1; // (byte) 1 byte
            [FieldOffset(1)]
            public byte b2;
            [FieldOffset(2)]
            public byte b3;
            [FieldOffset(3)]
            public byte b4;

            // Constructor:
            public UnionHack(int i, float f, char c, byte b)
            {
                this.i = i;
                this.f = f;
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

            Marshal.SizeOf(typeof(UnionHack)); // 4

            // Set serialPortXbee parameters
            serialPortXbee.ReadTimeout = 1000;
            serialPortXbee.BaudRate = 57600;
            serialPortXbee.PortName = "COM26";
            // Open serial port connection
            serialPortXbee.Open();

            // Pack into string
            //msg2ard = String.Format("{0}:{1:0.##},{2:0.##},{3:0.##}", msg2ard_key[0], vt_id, vt_rad[vt_id, 1], vt_ts[vt_id, 1]);
            double msgDbl = (double)125.26589654;
            int msgInt = (int)msgDbl;
            char msgChar = (char)msgInt;
            String msgStr = String.Format("{0:0.##}", msgChar);
            float msgFloat = (float)msgDbl;
            //byte[] msgByte = BitConverter.GetBytes(msgFloat);

            UnionHack u = new UnionHack(0, 0, '0', 0);
            u.c1 = 'A';
            u.c2 = 'B';
            //u.f = msgFloat;
            byte[] msgByte = new byte[4] { u.b1, u.b2, u.b3, u.b4 };

            while (true)
            {
                Thread.Sleep(33);
                //serialPortXbee.Write(msgStr);
                serialPortXbee.Write(msgByte, 0, 4);
            }
            Console.WriteLine(msgFloat);
            Console.WriteLine(msgInt);
            Console.WriteLine((int)u.f);
            Console.WriteLine(u.i);
            Console.ReadKey();

            return;

            // Start Cheetah if it is not already running
            while (!IsProcessOpen("Cheetah"))
            {
                OpenCheetah("Cheetah.cfg");
            }

            // Initilize deligate for VT callback
            MNetComClient mNetComClient = new MNetComClient();
            mNetcomVTCallback = new MNetCom.MNC_VTCallback(NetComCallbackVT);
            mNetComClient.SetCallbackFunctionVT(mNetcomVTCallback, new MeStreamNLX());

            // Initialize ICR_GUI background worker
            var bw_rungui = new BackgroundWorker();
            bw_rungui.DoWork += do_work_rungui;
            bw_rungui.RunWorkerCompleted += run_worker_completed_rungui;
            bw_rungui.RunWorkerAsync(); // start worker

            //comPortMat.Execute("D.UI.trckH(1).LineWidth = " + Convert.ToString(10) + ";");
            //comPortMat.PutWorkspaceData("D.UI.trckH(1).LineWidth", "base", 10);

            // Set serialPortXbee parameters
            serialPortXbee.ReadTimeout = 1000;
            serialPortXbee.BaudRate = 57600;
            serialPortXbee.PortName = "COM26";

            // Set NetCom parameters
            var NETCOM_APP_ID = "MyNetComStream"; // vt_id string displayed in Cheetah when connected
            var NETCOM_ACQ_ENT_1 = "VT1"; // vt_id for aquisition entity to stream
            var NETCOM_ACQ_ENT_2 = "VT2"; // vt_id for aquisition entity to stream
            var NETCOM_IP = "127.0.0.1"; // host computer IP

            // Open serial port connection
            serialPortXbee.Open();

            // Pause to let ICR_GUI connect to NLX fist
            comPortMat.Execute(@"global cMesage;");
            comPortMat.Execute(@"cMesage = 'None';");
            while ((String)comPortMat.GetVariable("cMesage", "global") != "matlabConnected") ;

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
                        if ((String)comPortMat.GetVariable("cMesage", "global") == "quit")
                        {
                            shouldExit = true;
                        }

                        if (!shouldExit)
                        {
                            msgMatLast = msgMatNow;
                            //Console.ReadKey();
                            msgMatNow = (String)comPortMat.GetVariable("cMesage", "global");
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

            // Pack pos data for serial
            msg2ard =
                String.Format("{0}:{1:0.##},{2:0.##},{3:0.##}",
                msg2ard_key[0], vt_id, vt_rad[vt_id, 1], vt_ts[vt_id, 1]);

            // Print in console
            Console.WriteLine(msg2ard);

            // Send to arduino
            try
            {
                serialPortXbee.WriteLine(msg2ard);
            }
            catch (InvalidOperationException ex)
            {
                Console.WriteLine(ex.GetType().FullName);
                Console.WriteLine(ex.Message);
            }
        }

        public static void CompPos(ushort id, ulong ts, double x, double y)
        {

            // Get first vt_ts once
            if (vt_tsStr == 0)
            {
                vt_tsStr = ts;
            }

            // Get record vt_id
            vt_id = (byte)id;

            // Itterate event number
            vt_rec[vt_id]++;

            // Save old vals
            vt_rad[vt_id, 0] = vt_rad[vt_id, 1];
            vt_ts[vt_id, 0] = vt_ts[vt_id, 1];

            // Convert to ms and update vt_ts
            vt_ts[vt_id, 1] = (ulong)Math.Round((double)(ts - vt_tsStr) / 1000);

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
            double dt = (vt_ts[vt_id, 1] - vt_ts[vt_id, 0]) / 1000;
            double v = Math.Abs(rad - vt_rad[vt_id, 0]) *
                ((140 * Math.PI) / (2 * Math.PI)) /
                dt;

            // Update vars
            vt_x = (float)x;
            vt_y = (float)y;
            vt_rad[vt_id, 1] = (float)rad;
            vt_vel = (float)v;

            // Load list
            msgList.Add(msg2ard_key);
            msgList.Add(vt_id);
            msgList.Add(vt_ts[vt_id, 1]);
            msgList.Add(vt_rad[vt_id, 1]);

        }

        public static void sendData()
        {

            string key = (string)msgList[0];
            char[] chr = new char[2];
            chr = key.ToCharArray();
            UnionHack u = new UnionHack(0, 0, '0', 0);

            if (key == msg2ard_key[0])
            {
                byte[] msgByte = new byte[11];
                // Add key chars
                u.c1 = chr[1];
                u.c2 = chr[2];
                msgByte[0] = u.b1;
                msgByte[1] = u.b2;
                // Add vt_id byte
                msgByte[2] = (byte)msgList[1];
                // Add vt_ts int 
                u.i = (int)msgList[2];
                msgByte[3] = u.b1;
                msgByte[4] = u.b2;
                msgByte[5] = u.b3;
                msgByte[6] = u.b4;
                // Add vt_rad float 
                u.f = (float)msgList[3];
                msgByte[7] = u.b1;
                msgByte[8] = u.b2;
                msgByte[9] = u.b3;
                msgByte[10] = u.b4;

                // Send to arduino
                try
                {
                    serialPortXbee.Write(msgByte, 0, msgByte.Length);
                }
                catch (InvalidOperationException ex)
                {
                    Console.WriteLine(ex.GetType().FullName);
                    Console.WriteLine(ex.Message);
                }

            }

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

    }
}