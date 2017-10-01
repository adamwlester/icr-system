function[status] = ICR_GUI(sysTest, doDebug, isMatSolo)
% INPUT:
%	sysTest = [0,1,2,3]
%    	0: No test
%     	1: PID calibration
%   	2: Halt error test
%    	3: Simulated rat test
%   doDebug = [0,1]
%     	0: Dont break on errors
%       1: Break on errors
%   isMatSolo = [true,false]
%     	true: Matlab running alone
%       true: Matlab running with other programs






%% ========================= TOP LEVEL RUN ==============================

% ----------------------------- SETUP GLOBALS -----------------------------

% Matlab globals
global ME; % error handeling
global doExit; % exit flag
global FigH; % ui figure handle
global D; % main data struct
global tcpIP; % tcp object
global timer_c2m; % timer to check c2m

% Matlab to CS communication
global m2c; % local data struct
global m2c_pack; % message out to CS
global m2c_dir; % current cheetah directory

% CS to Matlab communication
global c2m; % data struct

% Get start time
startTime = now;

% ------------------------- SETUP ERROR HANDELING -------------------------

% Preset status abort flag
status = 'failed'; %#ok<NASGU>
doExit = false;

% Handle input args
if nargin < 3
    isMatSolo = true;
end
if nargin < 2
    doDebug = true;
end
if nargin < 1
    sysTest = 0;
end

% Setup error handeling
if doDebug
    % Stop on error
    dbstop if error;
else
    % Catch and print error in console
    dbclear if caught error;
end

%---------------------SET DEBUG/TEST PARAMETERS----------------------------

% Session conditions
D.DB.ratLab = 'r9999';
D.DB.Session_Condition = 'Behavior_Training'; % ['Manual_Training' 'Behavior_Training' 'Rotation']
D.DB.Session_Task = 'Forage'; % ['Track' 'Forage']
D.DB.Reward_Delay = '1.0';
D.DB.Cue_Condition = 'Half';
D.DB.Sound_Conditions = [1,1];
D.DB.Rotation_Direction = 'CW'; % ['CCW' 'CW']
D.DB.Start_Quadrant = 'NE'; % [NE,SE,SW,NW];
D.DB.Rotation_Positions = [180,180,180,90,180,270,90,180,270]; % [90,180,270];

% Simulated rat test
D.DB.ratVelStart = 100;
D.DB.ratMaxAcc = 200; % (cm/sec/sec)
D.DB.ratMaxDec = 200; % (cm/sec/sec)

% Halt Error test
D.DB.velSteps = 10:10:80; % (cm/sec)
D.DB.stepSamp = 4;

%---------------------Important variable formats---------------------------
%...........................D.UI.snd....................................
%   val 1 = White Noise [true, false]
%   val 2 = Reward Tone [true, false]
%...........................D.AC.data......................................
%   val 1 = conection [0, 1], [no, yes]
%   val 2 = display image [0, 1, 2, 3], [Close all, 0-deg, -40-deg, 40-deg]
%   val 3 = sound state [0, 1], [no sound, sound]

if doDebug
    
    % ------------------------- RUN IN DEBUG MODE -------------------------
    
    % RUN MAIN SETUP
    if ~doExit
        Setup();
    end
    
    % RUN MAIN RUN
    if ~doExit
        Run();
    end
    
    % RUN MAIN EXIT
    Exit();
    
    % SET STATUS
    status = 'succeeded';
    
else
    
    % ---------------------- RUN IN CATCH ERROR MODE ----------------------
    % RUN MAIN SETUP
    try
        if ~doExit
            Setup();
        end
    catch ME
        doExit = true;
    end
    
    % RUN MAIN RUN
    try
        if ~doExit
            Run();
        end
    catch ME
        doExit = true;
    end
    
    % RUN MAIN EXIT
    try
        Exit();
    catch ME
        doExit = true;
    end
    
    % HANDLE ERRRORS
    if ~isempty(ME)
        if exist('D', 'var')
            if(isfield(D,'DB'))
                D.DB.isErrExit = true;
            end
        end
        % Log/print error
        err = sprintf([ ...
            'Time: %0.2f\r\n', ...
            'ID: %s\r\n', ...
            'Msg: \"%s\"\r\n', ...
            'Function: %s\r\n', ...
            'Lines: '], ...
            Elapsed_Seconds(now), ...
            ME.identifier, ...
            ME.message, ...
            ME.stack(1).name);
        for z_line = 1:length(ME.stack)
            err = [err, sprintf('%d|', ME.stack(z_line).line)]; %#ok<AGROW>
        end
        err = err(1:end-1);
        err_print = [sprintf('!!!!!!!!!!!!!ERROR!!!!!!!!!!!!!\r\n'), err, ...
            sprintf('\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n')];
        Console_Write(err_print);
        
        % Store status
        err_status = regexprep(err, '\r\n', ' ');
        err_status = regexprep(err_status, '\s*', ' ');
        status = err_status;
        
        % SET STATUS
    else
        status = 'succeeded';
    end
    
end

% Clear all variables
clearvars -global;
close all;

% For added measure
Vars=whos;
Vars={Vars.name};
Vars(ismember(Vars,'status')) = [];
clear(Vars{:});

% PRINT RUN ENDED
fprintf('END OF RUN\n');






%% ========================= MAIN FUNCTIONS ===============================

% -----------------------------MAIN SETUP----------------------------------
    function[] = Setup()
        
        % ---------------------------- SET PARAMETERS ---------------------
        
        % MAIN RUN PARAMETERS
        
        % Poll fs (sec)
        D.PAR.polRate = 1/30;
        % Min time in start quad (sec)
        D.PAR.strQdDel = 0.5;
        % Min time in rf target (sec)
        D.PAR.rfRewDel = 0.5;
        % Warning battery voltage level
        D.PAR.batVoltWarning = 11.6;
        % Replace battery voltage level
        D.PAR.batVoltReplace = 11.8;
        % Robot guard dist
        D.PAR.guardDist = 4.5 * ((2 * pi)/(140 * pi));
        % PID setPoint
        D.PAR.setPoint = 50 * ((2 * pi)/(140 * pi));
        % Robot butt dist
        D.PAR.buttDist = 18 * ((2 * pi)/(140 * pi));
        % Feeder dist from rob tracker
        D.PAR.feedDist = 66 * ((2 * pi)/(140 * pi));
        
        % DIRECTORIES
        
        % Top directory
        D.DIR.top = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running';
        
        % IO dirs
        D.DIR.ioTop = fullfile(D.DIR.top,'IOfiles');
        D.DIR.ioTestOut = fullfile(D.DIR.top,'Testing','Output');
        D.DIR.ioWallImage = fullfile(D.DIR.ioTop,'Images','plot_images','wall_top_down.png');
        D.DIR.ioSS_In_All = fullfile(D.DIR.ioTop, 'SessionData', 'SS_In_All.mat');
        D.DIR.ioSS_Out_ICR = fullfile(D.DIR.ioTop, 'SessionData', 'SS_Out_ICR.mat');
        D.DIR.ioTrkBnds = fullfile(D.DIR.ioTop, 'Operational', 'track_bounds(new_cam).mat');
        D.DIR.ioRFPath = fullfile(D.DIR.ioTop, 'Operational', 'forrage_path.mat');
        
        % Cheetah dirs
        D.DIR.nlxTempTop = 'C:\CheetahData\Temp';
        D.DIR.nlxSaveTop = 'G:\BehaviorPilot';
        D.DIR.recFi = '0000-00-00_00-00-00';
        
        % Log dirs
        D.DIR.logFi = 'ICR_GUI_Log.csv';
        D.DIR.logTemp = fullfile(D.DIR.nlxTempTop,'0000-00-00_00-00-00', D.DIR.logFi);
        
        % ------------------------- SETUP TOP LEVEL VARS ------------------
        
        % m2c struct
        id_list = [ ...
            'i', ... % handshake request [NA]
            'p', ... % simulation data [ts, x, y]
            'G', ... % matlab gui loaded [NA]
            'A', ... % connected to AC computer [NA]
            'N', ... % netcom setup [NA]
            'F', ... % data saved [NA]
            'X', ... % confirm quit
            'C', ... % confirm close
            'T', ... % system test command [(uint)test]
            'S', ... % setup session [(uint)ses_cond, (byte)sound_cond]
            'M', ... % move to position [(float)targ_pos]
            'R', ... % run reward [(float)rew_pos, (uint)rew_cond (uint)zone_ind/rew_delay]
            'H', ... % halt movement [(uint)halt_state]
            'B', ... % bulldoze rat [(uint)bull_delay, (float)bull_speed]
            'I' ... % rat in/out [(uint)in/out]
            'O' ... % confirm rat out [NA]
            ];
        for z_id = 1:length(id_list)
            m2c.(id_list(z_id)) = cell2struct( ...
                {id_list(z_id), 0, 0, 0, 0, 0, 0}, ...
                {'id', 'dat1', 'dat2', 'dat3', 'pack', 'packLast', 't_sent'}, 2);
        end
        m2c.cnt_pack = 0;
        
        % m2c_pack array [id, dat1, dat2, dat3, pack, flag]
        m2c_pack(1:6) = 0;
        
        % c2m struct
        id_list = [ ...
            'g', ... % request m2c data
            'h', ... % setup handshake
            'J', ... % battery voltage
            'Z', ... % reward zone
            'V', ... % robot streaming exit
            'K', ... % robot in place
            'Y', ... % enable save
            'E', ... % exit
            'C' ... % confirm close
            ];
        for z_id = 1:length(id_list)
            c2m.(id_list(z_id)) = cell2struct( ...
                {id_list(z_id), 0, 0, 0, 0}, ...
                {'id', 'dat1', 'pack', 'packLast', 't_rcvd'}, 2);
        end
        
        % Set top level vars
        D.DB.consoleStr = [repmat(' ',1000,150),repmat('\r',1000,1)];
        D.DB.logStr = cell(1000,1);
        D.DB.logCount = 0;
        D.DB.isTestRun = false;
        D.DB.doPidCalibrationTest = false;
        D.DB.doHaltErrorTest = false;
        D.DB.doSimRatTest = false;
        D.DB.isErrExit = false;
        D.DB.isForceClose = false;
        
        % Get testing condition
        switch sysTest
            case 1
                D.DB.isTestRun = true;
                D.DB.doPidCalibrationTest = true;
                
            case 2
                D.DB.isTestRun = true;
                D.DB.doHaltErrorTest = true;
                
            case 3
                D.DB.isTestRun = true;
                D.DB.doSimRatTest = true;
                
            otherwise
                D.DB.isTestRun = false;
                D.DB.doPidCalibrationTest = false;
                D.DB.doHaltErrorTest = false;
                D.DB.doSimRatTest = false;
        end
        
        % Bypass some things if running solo
        if isMatSolo
            c2m.('V').dat1 = 1;
            c2m.('K').dat1 = 1;
            c2m.('Y').dat1 = 1;
            c2m.('C').dat1 = 1;
        end
        
        % Log print run conditions
        Console_Write(sprintf('[Setup] RUNNING ICR_GUI.m: sysTest=%d doDebug=%d isMatSolo=%d', ...
            sysTest, doDebug, isMatSolo));
        
        % Start c2m com check timer
        timer_c2m = timer;
        timer_c2m.ExecutionMode = 'fixedRate';
        timer_c2m.Period = 0.1;
        timer_c2m.TimerFcn = @CheckC2M;
        start(timer_c2m);
        
        % ------------------------ SETUP AC CONNECTION --------------------
        Console_Write('[Setup] RUNNING: Connect to AC Computer...');
        
        % Set flag
        D.AC.connected = false;
        
        % Setup communication with ARENACONTROLLER
        D.AC.IP = '172.17.0.3';
        
        % Initialize parameters
        % signal to connect to ARENACONTROLLER
        D.AC.data(1) = 1;
        % signal for display image
        D.AC.data(2) = 0;
        % signal for sound condition
        D.AC.data(3) = 0;
        
        % Convert to 8 bit signed int
        D.AC.data = int8(D.AC.data);
        
        % Stop timer
        stop(timer_c2m);
        
        % Create tcpip object
        tcpIP = tcpip('0.0.0.0',55000, ...
            'OutputBufferSize', length(D.AC.data), ...
            'NetworkRole','Server', ...
            'Timeout', 1);
        
        % Establish connection
        % will loop here until connection established
        if ~isMatSolo
            fopen(tcpIP);
            D.AC.connected = true;
        else
            D.AC.connected = false;
        end
        
        % Restart timer
        start(timer_c2m);
        
        % Print that AC computer is connected
        if (D.AC.connected)
            Console_Write(sprintf('[Setup] FINISHED: Connect to AC Computer IP=%s', ...
                D.AC.IP));
        else
            Console_Write(sprintf('**WARNING** [Setup] ABORTED: Connect to AC Computer IP=%s', ...
                D.AC.IP));
        end
        
        % Send to CS
        SendM2C('A');
        
        % -------------------------- SETUP HANDSHAKE ----------------------
        
        % Wait for sync time
        Console_Write('[Setup] RUNNING: Wait for Handshake...');
        if ~isMatSolo
            
            % Tell CS ready for handshake
            SendM2C('i');
            
            % Wait for setup handshake
            while ...
                    c2m.('h').dat1 == 0 && ...
                    ~doExit
                pause(0.01);
            end
            
            % Set sync time
            if (c2m.('h').dat1 ~= 0)
                Console_Write('[Setup] FINISHED: Wait for Handshake');
                Console_Write(sprintf('SET SYNC TIME: %ddays',startTime));
            else
                Console_Write('!!ERROR!! ABORTED: Wait for Handshake');
                return
            end
            
        end
        
        % Open figure
        FigH = figure('Visible', 'Off', ...
            'DeleteFcn', {@ForceClose});
        
    end

% ------------------------------MAIN RUN-----------------------------------
    function[] = Run()
        
        %% =========================== MAIN LOOP ==========================
        
        MainLoop();
        
        function[] = MainLoop()
            
            
            % ---------------------------SETUP & RUN----------------------------------
            
            % Run variable setup code
            Console_Write('[MainLoop] RUNNING: "Var_Setup()"...');
            Var_Setup();
            Console_Write('[MainLoop] FINISHED: "Var_Setup()"');
            
            % Run UI setup code
            Console_Write('[MainLoop] RUNNING: "UI_Setup()"...');
            UI_Setup();
            Console_Write('[MainLoop] FINISHED: "UI_Setup()"');
            SendM2C('G');
            
            % Run NLX setup code
            Console_Write('[MainLoop] RUNNING: "NLX_Setup()"...');
            NLX_Setup();
            Console_Write('[MainLoop] FINISHED: "NLX_Setup()"');
            SendM2C('N');
            
            % Run testing setup
            Console_Write('[MainLoop] RUNNING: "Run_Test_Setup()"...');
            Run_Test_Setup();
            Console_Write('[MainLoop] FINISHED: "Run_Test_Setup()"');
            
            while ~doExit
                
                % TRACK LOOP TIME
                if (D.T.loop > 0)
                    D.DB.loop(1) = (Elapsed_Seconds(now) - D.T.loop) * 1000;
                    D.DB.loop(2) = min(D.DB.loop(1), D.DB.loop(2));
                    D.DB.loop(3) = max(D.DB.loop(1), D.DB.loop(3));
                    D.DB.loop(4) = D.DB.loop(1) + D.DB.loop(4);
                    D.DB.loop(5) = D.DB.loop(5) + 1;
                end
                D.T.loop = Elapsed_Seconds(now);
                
                % PLOT POSITION
                update_ui = Pos_Plot();
                
                % PRINT SES INFO
                Inf_Print();
                
                % UPDATE GUI
                if update_ui
                    Update_UI(0);
                else
                    Update_UI(100);
                end
                
                % CHECK FOR EXIT FLAG
                if doExit
                    continue;
                end
                
                % --------------------CHECK WHAT TO DO---------------------
                D.F.do_last_main = D.F.do_what_main;
                
                if ~D.F.data_loaded
                    D.F.do_what_main = 'WAIT FOR UI SETUP';
                    
                elseif ~D.F.setup_done
                    D.F.do_what_main = 'FINISH SETUP';
                    
                elseif D.F.poll_nlx && ...
                        (Elapsed_Seconds(now) - D.T.poll_last) >= D.PAR.polRate
                    D.F.do_what_main = 'POLL NLX';
                    
                else
                    continue;
                    
                end
                
                % PRINT CHANGE IN FLOW
                if ~strcmp(D.F.do_what_main, D.F.do_last_main)
                    Console_Write(sprintf('[MainLoop] SWITCH FROM ""%s"" TO ""%s""', D.F.do_last_main, D.F.do_what_main));
                end
                
                % HANDLE CASE
                switch D.F.do_what_main
                    
                    
                    case 'WAIT FOR UI SETUP'
                        %% ---------------WAIT FOR UI SETUP----------------
                        continue;
                        
                        
                    case 'FINISH SETUP'
                        %% -----------------FINISH SETUP-------------------
                        
                        % Dump initial vt recs
                        Console_Write('[MainLoop] RUNNING: Dump VT Recs...');
                        while true
                            pause(0.01);
                            VT_Get('Rat');
                            VT_Get('Rob');
                            Evt_Get();
                            
                            % Check if no new recs
                            if ...
                                    D.P.Rat.vtNRecs == 0 && ...
                                    D.P.Rat.vtNRecs == 0 && ...
                                    D.E.evtNRecs == 0
                                break;
                            end
                        end
                        Console_Write('[MainLoop] FINISHED: Dump VT Recs...');
                        
                        % Wait for robot streaming to start
                        Console_Write('[MainLoop] RUNNING: Wait for Robot Streaming...');
                        while ...
                                c2m.('V').dat1 == 0 && ...
                                ~doExit
                            pause(0.01);
                            Update_UI(50);
                        end
                        if ~doExit
                            Console_Write('[MainLoop] FINISHED: Wait for Robot Streaming');
                        else
                            Console_Write('**WARNING** [MainLoop] ABORTED: Wait for Robot Streaming');
                            return
                        end
                        
                        % Run Finish setup code
                        Console_Write('[MainLoop] RUNNING: "Finish_Setup()"...');
                        Finish_Setup();
                        Console_Write('[MainLoop] FINISHED: "Finish_Setup()"');
                        
                        % Run 'Track' task setup code
                        if D.PAR.sesTask == 'Track'
                            Console_Write('[MainLoop] RUNNING: "Track_Task_Setup()"');
                            Track_Task_Setup();
                            Console_Write('[MainLoop] FINISHED: "Track_Task_Setup()"');
                        end
                        
                        % Run 'Forage' task setup code
                        if D.PAR.sesTask == 'Forage'
                            Console_Write('[MainLoop] RUNNING: "Forage_Task_Setup()"');
                            Forage_Task_Setup();
                            Console_Write('[MainLoop] FINISHED: "Forage_Task_Setup()"');
                        end
                        
                        % Refresh UI
                        Update_UI(0);
                        
                        % Set to start polling NLX
                        D.F.poll_nlx = true;
                        
                        % Flag setup done
                        D.F.setup_done = true;
                        
                        % Pause for Setup to be received
                        pause(0.1);
                        
                        % Begin main loop
                        Console_Write('[MainLoop] READY TO ROCK!');
                        
                        
                    case 'POLL NLX'
                        %% ------------------POLL NETCOM-------------------
                        
                        % STORE POLL TIME
                        D.T.poll_last = Elapsed_Seconds(now);
                        
                        % PROCESS NLX EVENTS
                        Evt_Proc();
                        
                        % PROCESS NLX VT
                        VT_Get('Rob');
                        VT_Proc('Rob');
                        if ~D.DB.doSimRatTest
                            VT_Get('Rat');
                            VT_Proc('Rat');
                        end
                        
                        % RUN TEST/DEBUG CODE
                        Run_Test();
                        
                        % ---------------CHECK WHAT TO DO------------------
                        D.F.do_last_poll = D.F.do_what_poll;
                        
                        if ~D.F.is_first_move_sent
                            D.F.do_what_poll = 'CHECK IF MOVE READY';
                            
                        elseif c2m.('K').dat1 == 1 && ...
                                ~D.F.rat_in && ...
                                D.F.rec && ...
                                ~D.F.rec_done && ...
                                ~D.F.quit
                            D.F.do_what_poll = 'CHECK IF RAT IN';
                            
                        elseif D.F.rat_in
                            D.F.do_what_poll = 'RUN MAIN';
                            
                        elseif (D.F.rec_done || D.F.quit) && ~D.F.rat_out
                            D.F.do_what_poll = 'SEND RECORDING DONE CONFIRMATION';
                            
                        elseif D.F.rec_done && D.F.rat_out
                            D.F.do_what_poll = 'WAIT FOR SAVE';
                            
                        else
                            continue;
                            
                        end
                        
                        % PRINT CHANGE IN FLOW
                        if ~strcmp(D.F.do_what_poll, D.F.do_last_poll)
                            Console_Write(sprintf('[MainLoop] SWITCH FROM ""%s"" TO ""%s""', D.F.do_last_poll, D.F.do_what_poll));
                        end
                        
                        % HANDLE CASE
                        switch D.F.do_what_poll
                            
                            case 'CHECK IF MOVE READY'
                                % ----------CHECK IF MOVE READY------------
                                
                                % Wait till at least 2000 VT samples collected
                                if D.P.Rob.cnt_vtRec > 2000
                                    continue;
                                end
                                
                                % Send C# command to move robot to start quad or reward loc
                                if D.PAR.sesCond ~= 'Manual_Training' %#ok<*STCMP>
                                    SendM2C('M', D.PAR.strQuadBnds(1));
                                else
                                    SendM2C('M', D.UI.rewZoneRad(1));
                                end
                                
                                % Set flag
                                D.F.is_first_move_sent = true;
                                Console_Write('[MainLoop] SENT STARTING MOVE COMMAND');
                                
                            case 'CHECK IF RAT IN'
                                % ------------CHECK IF RAT IN--------------
                                
                                Rat_In_Check();
                                
                            case 'RUN MAIN'
                                % ---------------RUN MAIN------------------
                                
                                % ROTATION TRIGGER CHECK
                                Rotation_Trig_Check();
                                
                                % TRACK REWARD RESET CHECK
                                Track_Reward_Send_Check();
                                
                                % TRACK REWARD CHECK
                                Track_Reward_Zone_Check();
                                
                                % FORAGE REWARD CHECK
                                Forage_Reward_Zone_Check();
                                
                                % LAP CHECK
                                Lap_Check();
                                
                                
                            case 'SEND RECORDING DONE CONFIRMATION'
                                % --------SEND RECORDING DONE CONFIRMATION--------
                                
                                % Tell CS recording is finished
                                SendM2C('O');
                                D.F.rat_out = true;
                                Console_Write('[MainLoop] SENT RECORDING DONE CONFIRMATION');
                                
                                
                            case 'WAIT FOR SAVE'
                                % -------------WAIT FOR SAVE---------------
                                
                                % Check if CS has enabled save
                                if c2m.('Y').dat1 == 1
                                    
                                    % Enable save button
                                    set(D.UI.btnSave, ...
                                        'Enable', 'on', ...
                                        'ForegroundColor' , D.UI.enabledBtnFrgCol, ...
                                        'BackgroundColor', D.UI.activeCol);
                                    
                                    % Reset flag
                                    c2m.('Y').dat1 = 0;
                                    Console_Write('[MainLoop] SAVE BUTTON ENABLED');
                                end
                                
                                % Save sesion data
                                if D.F.do_save
                                    Console_Write('[MainLoop] SAVE INITIATED');
                                    Save_Ses_Data();
                                    D.F.do_save = false;
                                end
                                
                            otherwise
                                continue;
                                
                        end
                        
                    otherwise
                        continue;
                end
                
            end
            
        end
        
        
        
        
        
        
        %% ========================= SETUP FUNCTIONS ======================
        
        % ------------------------------VAR SETUP--------------------------
        
        function [] = Var_Setup()
            
            %% TOP LEVEL PARAMETERS
            
            % LOAD TRACK BOUNDS
            
            % bounding box for video tracker (in pixels)
            % track plot bounds (width, hight)
            S = load(D.DIR.ioTrkBnds);
            % track bound radius
            D.PAR.R = S.R;
            % track bound center X
            D.PAR.XC = S.XC;
            % track bound center Y
            D.PAR.YC = S.YC;
            clear S;
            
            % POSITION VARS
            
            % Plot bounds
            D.UI.vtRes = round(D.PAR.R*2);
            % track plot bounds (left, bottom)
            D.UI.lowLeft = round([D.PAR.XC-D.PAR.R,D.PAR.YC-D.PAR.R]);
            
            % Calculate pixel to cm conversion factors
            D.UI.cm2pxl = D.UI.vtRes/140;
            
            % Pos lims
            % arena radius (cm)
            D.UI.arnRad = 70;
            % track width (cm)
            D.UI.trkWdt = 10;
            % random forrage radius (cm)
            D.UI.rfRad = 60;
            % rew zone width (cm)
            D.UI.rfTargWdt = 15;
            
            % FORAGE TASK VARS
            
            % pos occ bins
            D.PAR.rfBins = (D.UI.arnRad*2)/2 + 1;
            % occ bin edges
            D.PAR.rfBinEdgeX = linspace(D.UI.lowLeft(1), D.UI.lowLeft(1)+D.UI.vtRes, D.PAR.rfBins+1);
            D.PAR.rfBinEdgeY = linspace(D.UI.lowLeft(2), D.UI.lowLeft(2)+D.UI.vtRes, D.PAR.rfBins+1);
            % distance between paths in degrees
            D.PAR.pathDegDist = 5;
            % target width
            D.PAR.pathTargWdt = 30;
            % target array
            D.PAR.pathTargArr = 0:D.PAR.pathDegDist:360-D.PAR.pathDegDist;
            % number of posible paths from each targ
            D.PAR.nPaths = 45/D.PAR.pathDegDist*2 + 1;
            % path lenths
            D.PAR.pathLengthArr = zeros(1,D.PAR.nPaths);
            
            % TRACK TASK VARS
            
            % min/max reward duration
            D.PAR.rewDurLim = [500, 2000];
            % reward zone positions
            D.PAR.zoneLocs = 20:-5:-20;
            % reward zone reward durations
            D.PAR.zoneRewDur = ...
                [500, 910, 1420, 1840, 2000, 1840, 1420, 910, 500];
            % current reward duration
            D.PAR.rewDur = max(D.PAR.rewDurLim);
            
            %% FLOW CONTROL VARIABLES
            
            % Load data tables
            T = load(D.DIR.ioSS_In_All);
            D.SS_In_All = T.SS_In_All;
            T = load(D.DIR.ioSS_Out_ICR);
            D.SS_Out_ICR = T.SS_Out_ICR;
            clear T;
            
            % CATEGORICAL VARS
            
            % age group
            D.PAR.catAgeGrp = categories(D.SS_In_All.Age_Group); % [Young,Old];
            % session condition
            D.PAR.catSesCond = categories(D.SS_In_All.Session_Condition); % [M,B,I,R];
            % feeder condition
            D.PAR.catFeedCnd = categories(D.SS_In_All.Feeder_Condition); % [C1,C2];
            % cue condition
            D.PAR.catCueCond = categories(D.SS_In_All.Cue_Condition); % [All, Half, None]
            % start qauadrant
            D.PAR.catStrQuad = categories(D.SS_In_All.Start_Quadrant{1}); % [NE,SE,SW,NW];
            % rotation direction
            D.PAR.catRotDrc = categories(D.SS_In_All.Rotation_Direction{1}); % [CCW,CW];
            % rotation position
            D.PAR.catRotPos = categories(D.SS_In_All.Rotation_Positions{1}); % [90,180,270];
            
            % COUNTERS
            
            % track icr events
            D.C.rot_cnt = 0;
            % lap by rotation
            D.C.lap_cnt = num2cell(zeros(1,3));
            % rew by rotation
            D.C.rew_cnt = num2cell(zeros(1,3));
            % reward crossings
            D.C.rew_cross_cnt = 0;
            % missed rewards [consecutive, total]
            D.C.missed_rew_cnt = [0, 0];
            % bulldozing event count
            D.C.bull_cnt = 0;
            % counter for each reward zone
            D.C.zone = zeros(2,length(D.PAR.zoneLocs));
            
            % FLAGS
            
            % switch case main loop
            D.F.do_what_main = 'NULL';
            D.F.do_last_main = 'NULL';
            % switch case poll
            D.F.do_what_poll = 'NULL';
            D.F.do_last_poll = 'NULL';
            % rat data loaded
            D.F.data_loaded = false;
            % setup finished
            D.F.setup_done = false;
            % polling nlx
            D.F.poll_nlx = false;
            % polling nlx
            D.F.is_first_move_sent = false;
            % acquiring nlx
            D.F.acq = false;
            % recording nlx
            D.F.rec = false;
            % recording done
            D.F.rec_done = false;
            % track if rat is in arena
            D.F.rat_in = false;
            % track if rat is out of arena
            D.F.rat_out = false;
            % flag to do save
            D.F.do_save = false;
            % flag quit
            D.F.quit = false;
            % flag gui forced exit
            D.DB.isForceClose = false;
            % flag gui closed
            D.F.close = false;
            % rotation has occured
            D.F.rotated = false;
            % track if reward in progress
            D.F.is_rewarding = false;
            % block any cueing
            D.F.do_block_cue = false;
            % cue every lap
            D.F.do_all_cue = false;
            % track if next lap should be cued for half cue cond
            D.F.is_cued_rew = false;
            % flag if halted
            D.F.is_halted = false;
            % track reward reset
            D.F.flag_rew_confirmed = false;
            % track reset crossing
            D.F.flag_rew_send_crossed = false;
            % track reward crossing
            D.F.flag_rew_zone_crossed = false;
            % check for reward zone confirmaton
            D.F.check_rew_confirm = false;
            % track lap bounds
            D.F.check_inbound_lap = false(4,1);
            % track if pos should be plotted
            D.F.Rat.plot_pos = false;
            D.F.Rob.plot_pos = false;
            % track if velocity should be plotted
            D.F.Rat.plot_vel = false;
            D.F.Rob.plot_vel = false;
            % plot HD
            D.F.plot_hd = false;
            
            % TIMERS
            
            % loop time
            D.T.loop = 0;
            % last poll time
            D.T.poll_last = Elapsed_Seconds(now);
            % last ui redraw
            D.T.ui_update = Elapsed_Seconds(now);
            % last text info update
            D.T.info_txt_update = Elapsed_Seconds(now);
            % time session starts
            D.T.ses_str_tim = Elapsed_Seconds(now);
            % total acq time
            D.T.acq_tot_tim = 0;
            % acq restart time
            D.T.acq_tim = 0;
            % total rec time
            D.T.rec_tot_tim = 0;
            % rec restart time
            D.T.rec_tim = 0;
            % run start time
            D.T.run_str = 0;
            % run time
            D.T.run_tim = 0;
            % track lap times
            D.T.lap_tim = 0;
            % track lap times total
            D.T.lap_tim_sum = 0;
            % track manual reward sent time
            D.T.manual_rew_sent = 0;
            % track last reward time
            D.T.rew_last = Elapsed_Seconds(now);
            % track reward start
            D.T.rew_start = 0;
            % track reward end
            D.T.rew_end = 0;
            % track reward ts
            D.T.rew_nlx_ts = [0,0];
            % start quad tim
            D.T.strqd_inbnd_t1 = 0;
            D.T.strqd_inbnd_t2 = 0;
            % track last pos update
            D.T.Rat.last_pos_update = Elapsed_Seconds(now);
            D.T.Rob.last_pos_update = Elapsed_Seconds(now);
            % rf reward target tim
            D.T.rf_rew_inbnd_t1 = 0;
            D.T.rf_rew_inbnd_t2 = 0;
            
            % INDEXING
            
            % current wall image index
            D.I.rot = 1;
            % feeder index
            D.I.img_ind = [1, NaN];
            % current lap quadrant for lap track
            D.I.lap_hunt_ind = 1;
            % current reward zone
            D.I.zone = ceil(length(D.PAR.zoneLocs)/2); % default max
            % distrebution of reward zones
            D.I.zoneArr = NaN(1,100);
            % store reward zone for each reward event
            D.I.zoneHist = NaN(1,100);
            % current targ ind
            D.I.targInd = 1;
            
            %% DATA STORAGE VARIABLES
            
            % VCC VARS
            
            D.PAR.vcc_now = 0;
            D.PAR.vcc_last = 0;
            
            % EVENT DATA
            D.E.evtTS = NaN;
            D.E.evtStr = '';
            D.E.evtNRecs = 0;
            D.E.cnt_evtRec = 0;
            
            % POS DATA
            
            % nlx inputs
            D.P.Rat.vtTS = NaN;
            D.P.Rat.vtPos = NaN;
            D.P.Rat.vtHD = NaN;
            D.P.Rat.vtNRecs = 0;
            D.P.Rat.cnt_vtRec = 0;
            D.P.Rob.vtTS = NaN;
            D.P.Rob.vtPos = NaN;
            D.P.Rob.vtHD = NaN;
            D.P.Rob.vtNRecs = 0;
            D.P.Rob.cnt_vtRec = 0;
            % cardinal pos now
            D.P.Rat.x = NaN;
            D.P.Rat.y = NaN;
            D.P.Rob.x = NaN;
            D.P.Rob.y = NaN;
            % pos rad
            D.P.Rat.rad = NaN;
            D.P.Rob.rad = NaN;
            % pos roh
            D.P.Rat.roh = NaN;
            D.P.Rob.roh = NaN;
            % last used rad
            D.P.Rat.radLast = NaN;
            D.P.Rob.radLast = NaN;
            % vel radian
            D.P.Rat.velRad = NaN;
            D.P.Rob.velRad = NaN;
            % velocity data sample length
            D.P.vel_nsmp = 10;
            % store velocity data
            D.P.Rat.rad_samples = NaN(D.P.vel_nsmp,1);
            D.P.Rob.rad_samples = NaN(D.P.vel_nsmp,1);
            D.P.Rat.ts_samples = NaN(D.P.vel_nsmp,1);
            D.P.Rob.ts_samples = NaN(D.P.vel_nsmp,1);
            D.P.Rat.vel = 0;
            D.P.Rob.vel = 0;
            D.P.Rat.vel_max_all = 0;
            D.P.Rob.vel_max_all = 0;
            D.P.Rat.vel_max_lap = 0;
            D.P.Rob.vel_max_lap = 0;
            D.P.Rat.vel_pol_arr = NaN(3,2);
            D.P.Rob.vel_pol_arr = NaN(3,2);
            % store hostory of pos x by y
            D.P.Rat.pos_hist = NaN(120*60*33,2);
            % store history of vel by roh
            D.P.Rat.vel_lap = NaN(60*60*33,2);
            D.P.Rob.vel_lap = NaN(60*60*33,2);
            % store 1 lap of vel by roh
            D.P.Rat.vel_hist = NaN(500,101,2);
            D.P.Rob.vel_hist = NaN(500,101,2);
            % track position cutoff
            D.P.trackRohBnd(1) = 1 - ((D.UI.trkWdt+5)/D.UI.arnRad);
            D.P.trackRohBnd(2) = 1 + (5/D.UI.arnRad);
            % velocity cutoff
            D.P.velRohMax = D.P.trackRohBnd(1);
            D.P.velRohMin = D.P.trackRohBnd(1) - 0.25;
            D.P.velMin = 0;
            D.P.velMax = 100;
            % random forage
            D.P.rfRohBnd(1) = (D.UI.rfRad/D.UI.arnRad) - ((D.UI.rfTargWdt)/D.UI.arnRad);
            D.P.rfRohBnd(2) = D.UI.rfRad/D.UI.arnRad;
            D.P.occMat = zeros(D.PAR.rfBins,D.PAR.rfBins);
            D.P.occMatRaw = D.P.occMat;
            D.P.pathMat = zeros(D.PAR.rfBins,D.PAR.rfBins,D.PAR.nPaths,length(D.PAR.pathTargArr));
            D.P.pathNowMat = D.P.occMat;
            
            
            % DEBUG VARS
            
            % track reward duration [now, min, max, sum, count]
            D.DB.rew_duration = [0, inf, 0, 0, 0];
            % track reward round trip [now, min, max, sum, count]
            D.DB.rew_round_trip = [0, inf, 0, 0, 0];
            % track loop dt [now, min, max, sum, count]
            D.DB.loop = [0, inf, 0, 0, 0];
            % draw now dt [now, min, max, sum, count]
            D.DB.draw = [0, inf, 0, 0, 0];
            % halt error [now, min, max, sum, count]
            D.DB.halt_error = [0, inf, 0, 0, 0];
            % simulated rat forage data
            D.DB.simRatPathMat = D.P.pathMat;
            
            %% UI HANDLES
            
            % AXIS HANDLES
            D.UI.axH = gobjects(4,1);
            
            % TRACK TASK HANDLES
            
            % track bound line handles
            D.UI.linTrckH = gobjects(2,1);
            % vel plot line handles
            D.UI.nVelRings = ceil(D.P.velMax/10)+1;
            D.UI.linVelH = gobjects(D.UI.nVelRings,1);
            % start quad patch
            D.UI.ptchStQ = [];
            % start quad text
            D.UI.txtStQ = [];
            % reward zone patch
            D.UI.ptchFdZineH = gobjects(length(D.PAR.zoneLocs),2);
            % reward duration text
            D.UI.txtFdDurH = gobjects(length(D.PAR.zoneLocs),2);
            % lap bounds patch
            D.UI.ptchLapBnds = gobjects(1,4);
            % reward reset patch
            D.UI.ptchFdRstH = gobjects(1,2);
            % current feeder marker/patch/line
            D.UI.mixFdNow = gobjects(3,2);
            
            % FORAGE TASK HANDLES
            
            % rf bound line handles
            D.UI.linRFH = gobjects(2,1);
            % rf bound mask
            D.UI.imgMaskRFH = [];
            % rf targ patch
            D.UI.ptchRFTarg = gobjects(360/D.PAR.pathDegDist,1);
            % rf occ mat
            D.UI.imgRFOCC = [];
            
            % POS DATA
            
            % vt plot handle array
            D.UI.ratPltH = gobjects(1,60*120/D.PAR.polRate);
            D.UI.cnt_ratPltH = 0;
            % vt plot handles of velocity
            D.UI.ratPltHvel = gobjects(1,60*120/D.PAR.polRate); % rat
            D.UI.robPltHvel = gobjects(1,60*120/D.PAR.polRate); % rob
            D.UI.cnt_ratPltHvel = 0;
            D.UI.cnt_robPltHvel = 0;
            % handles for average vel plots
            D.UI.Rat.pltHvelAvg = gobjects(1,1); % rat
            D.UI.Rob.pltHvelAvg = gobjects(1,1); % rob
            % handles for history data
            D.UI.Rat.pltHposAll = gobjects(1,1); % rat
            D.UI.Rat.pltHvelAll = gobjects(1,1); % rat
            D.UI.Rob.pltHvelAll = gobjects(1,1); % rob
            % reward zone
            D.UI.zoneAllH = gobjects(length(D.PAR.zoneLocs),2);
            D.UI.zoneNowH = gobjects(1,1);
            D.UI.zoneAvgH = gobjects(1,1);
            D.UI.txtFdDurH = gobjects(1,1);
            
            %% UI APPERANCE
            
            % UI COLOR
            
            % Lines and Markers
            
            % color for rotation conditions
            D.UI.rotCol = [0, 0, 1;0, 0.5, 0];
            % color of marker for rat pos
            D.UI.ratPosAllCol = [0, 0, 0];
            D.UI.ratPosHistCol = [0.75, 0.75, 0.75];
            % color of marker for current rat pos
            D.UI.ratNowCol = [0.5, 0, 0.5];
            D.UI.ratAvgCol = [0.4180, 0.3984, 0.6953];
            D.UI.ratHistCol = [0.8711, 0.7461, 0.8711];
            % color of marker for current rob pos
            D.UI.robNowCol = [1, 0.5, 0];
            D.UI.robAvgCol = [0.9961, 0.6953, 0.3984];
            D.UI.robHistCol = [0.9961, 0.8711, 0.7461];
            % color of line for rob guard pos
            D.UI.guardPosCol = D.UI.robNowCol;
            % width of line for rob guard pos
            D.UI.guardPosLineWidth = 5;
            % color of line for feeder pos
            D.UI.feedPosCol = D.UI.rotCol(1,:);
            % marker size for feeder pos
            D.UI.feedPosMarkSize = 10;
            % color of line for setpoint
            D.UI.setPosCol = D.UI.robNowCol;
            % width of line for setpoint
            D.UI.setPosLineWidth = 2;
            
            % UI objects
            D.UI.figBckCol = [1, 1, 1];
            D.UI.enabledCol = [0.25, 0.25, 0.25];
            D.UI.enabledBtnFrgCol = [0.95, 0.95, 0.95];
            D.UI.enabledPrintFrgCol = [0.1, 0.1, 0.1];
            D.UI.disabledBckCol = [0.75 0.75 0.75];
            D.UI.activeCol = [0.75, 0, 0];
            D.UI.warningCol = [1 0 0];
            
            % UI FONTS
            
            D.UI.btnFont = 'MS Sans Serif';
            D.UI.popFont = 'MS Sans Serif';
            D.UI.txtFont = 'Courier New';
            
            % UI OBJECT FEATURES
            
            % btnFillFd
            % color
            D.UI.btnFlcol = [0.75 0 0; 0 0.75 0];
            % string
            D.UI.btnFlstr = [{'Stop'}; {'Fill'}];
            
            % btnCleanFd
            % color
            D.UI.btnFdcol = [0.75 0 0; 0 0.75 0];
            % string
            D.UI.btnFdstr = [{'Stop'}; {'Clean'}];
            
            % popRat
            % make rat list
            ind = D.SS_In_All.Include_Run & ...
                ~isundefined(D.SS_In_All.Session_Condition);
            D.UI.ratList = ...
                [D.SS_In_All.Properties.RowNames(ind), ...
                cellstr(char(D.SS_In_All.Yoke_Mate(ind)))];
            % remove 'r'
            D.UI.ratList = regexprep(D.UI.ratList, 'r', '');
            % add space
            D.UI.ratList = [{blanks(16)}; ...
                cellfun(@(x,y) sprintf('%s (%s)', x, y), ...
                D.UI.ratList(:,1), D.UI.ratList(:,2), 'Uni', false)];
            
            % popTask
            D.UI.taskList = {''; 'Track'; 'Forage'};
            D.PAR.sesTask = categorical({'<undefined>'}, {'Track', 'Forage'});
            
            % popCond
            D.UI.condList = {''; 'Manual_Training'; 'Behavior_Training'; 'Implant_Training'; 'Rotation'};
            D.PAR.sesCond = categorical({'<undefined>'}, D.PAR.catSesCond);
            
            % popRewDel
            D.UI.delList = {''; '0.0 '; '1.0 '; '2.0'; '3.0'};
            
            % toggCue
            D.PAR.cueFeed = categorical({'<undefined>'}, D.PAR.catCueCond);
            
            % toggSnd
            % labels
            D.UI.sndLabs = [...
                {'White Noise'}, ...
                {'Reward Tone'}];
            % state strings
            D.UI.sndState = [{'Off'},{'on'}];
            % for storing sound settings
            D.UI.snd = false(1,2);
            
            % bulldoze
            D.UI.bullList = {'0 sec'; '5 sec'; '10 sec'; '30 sec'; '60 sec'; '120 sec'};
            D.PAR.bullDel = 30;
            D.PAR.bullSpeed = 5;
            D.UI.bullLastVal = 0;
            
            % btnAcq and btnRec
            % run Btn_Col function
            [D.UI.radBtnCmap] = Btn_Col();
            
            % btnICR
            % string
            D.UI.btnICRstr = cell(2,1);
            
            % btnSave
            % session saved
            D.UI.save_done = false;
            
            % popLapTim
            % list lap times
            D.UI.lapTimList = {[]};
            
            % popRewInfo
            % info about each reward
            D.UI.rewInfoList = {[]};
            
            % Log/print
            Console_Write('[Var_Setup] FINISHED: Variable Setup');
            
        end
        
        % -------------------------------UI SETUP--------------------------
        
        function[] = UI_Setup()
            
            %% GENERATE FIGURE AND AXES
            
            % Get screen dimensions
            sc = get(0,'MonitorPositions');
            D.UI.sc1 = sc(1,:);
            D.UI.sc2 = sc(2,:);
            
            % Set figure
            set(FigH,...
                'Name', 'ICR Behavior Pilot', ...
                'MenuBar', 'none', ...
                'Color', [1, 1, 1], ...
                'Visible', 'off');
            
            % Ongoing data
            % Note: used for pos/vel history
            D.UI.axH(1) = axes(...
                'Units', 'Normalized', ...
                'Visible', 'off');
            hold on;
            
            % Real time data
            % Note: used for path and other dynamic features
            D.UI.axH(2) = axes(...
                'Units', 'Normalized', ...
                'Visible', 'off');
            hold on;
            
            % Random forage occ data
            % Note: used for occ imagesc
            D.UI.axH(3) = axes(...
                'Units', 'Normalized', ...
                'Visible', 'off');
            hold on;
            
            % RF Mask
            % Note: mask for imagesc occ plot
            D.UI.axH(4) = axes(...
                'Units', 'Normalized', ...
                'Visible', 'off');
            hold on;
            
            % Wall image
            % Note: used for wall images
            D.UI.axH(5) = axes(...
                'Units', 'Normalized', ...
                'Visible', 'off');
            hold on;
            
            % Specify stack order
            uistack(D.UI.axH(1),'bottom');
            uistack(D.UI.axH(3),'bottom');
            uistack(D.UI.axH(4),'top');
            uistack(D.UI.axH(5),'top');
            uistack(D.UI.axH(2),'top');
            
            % Specigy figure width/hight
            fg_wh = [1240 800];
            
            % Set figure pos
            D.UI.fg_pos = [ ...
                D.UI.sc1(3) + (D.UI.sc2(3)-fg_wh(1))/2, ...
                (D.UI.sc2(4) - fg_wh(2))/2, ...
                fg_wh(1), ...
                fg_wh(2)];
            set(FigH,'Position',D.UI.fg_pos);
            
            % Set axis limits
            axis(D.UI.axH, [ ...
                D.UI.lowLeft(1), ...
                D.UI.lowLeft(1)+D.UI.vtRes, ...
                D.UI.lowLeft(2), ...
                D.UI.lowLeft(2)+D.UI.vtRes]);
            
            % Set axis pos
            D.UI.ax_pos = [ ...
                (1 - (0.9/(fg_wh(1)/fg_wh(2)))) / 2, ...
                0.05, ...
                0.9/(fg_wh(1)/fg_wh(2)), ...
                0.9];
            set(D.UI.axH, 'Position', D.UI.ax_pos);
            
            % ADD CARDINAL COORDINATES
            
            % mid x/y pos
            xy_mid = round(D.UI.vtRes./2) + D.UI.lowLeft;
            % min x/y pos
            xy_min = D.UI.lowLeft;
            % max x/y pos
            xy_max = D.UI.lowLeft + D.UI.vtRes;
            % text offset
            offset = 7;
            
            % Add cardinal coordinates text
            t_h(1) = text(xy_max(1)+offset, xy_mid(2), 'E', 'Parent', D.UI.axH(2));
            t_h(2) = text(xy_mid(1), xy_min(2)+2-offset, 'S', 'Parent', D.UI.axH(2));
            t_h(3) = text(xy_min(1)-offset, xy_mid(2), 'W', 'Parent', D.UI.axH(2));
            t_h(4) = text(xy_mid(1), xy_max(2)+offset, 'N', 'Parent', D.UI.axH(2));
            set(t_h, ...
                'FontName','Monospaced', ...
                'FontSize', 20, ...
                'FontWeight', 'bold', ...
                'FontSmoothing', 'on', ...
                'Color', [1,1,1], ...
                'HorizontalAlignment','center', ...
                'VerticalAlignment','middle')
            t2_h = copyobj(t_h, D.UI.axH(2));
            set(t2_h, ...
                'Color', D.UI.activeCol, ...
                'FontWeight', 'light', ...
                'FontSmoothing', 'on', ...
                'FontSize', 20);
            
            % Get coordinates for circle
            circ = [0:.01:2*pi,0];
            
            % Clear vars
            clear xy_mid xy_min xy_max offset t2_h;
            
            % CREATE TRACK OUTLINE
            
            % Plot outer track
            [out_X,out_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.arnRad));
            xout = out_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
            yout = out_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
            D.UI.linTrckH(1) = plot(xout, yout, ...
                'color', [0.5 0.5 0.5], ...
                'LineWidth', 2, ...
                'Parent', D.UI.axH(2), ...
                'Visible', 'off');
            
            % Plot inner track
            [in_X,in_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.arnRad-D.UI.trkWdt));
            xin = in_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
            yin = in_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
            D.UI.linTrckH(2) = plot(xin, yin, ...
                'color', [0.5 0.5 0.5], ...
                'LineWidth', 2, ...
                'Parent', D.UI.axH(2), ...
                'Visible', 'off');
            
            % Clear vars
            clear out_X out_Y xout yout in_X in_Y xin yin;
            
            % CREATE VEL PLOT OUTLINE
            roh_inc = linspace(D.P.velRohMin, D.P.velRohMax, D.UI.nVelRings);
            
            for z_lin = 1:D.UI.nVelRings
                [x,y] = pol2cart(circ, ones(1,length(circ))*D.UI.arnRad*roh_inc(z_lin));
                x = x*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
                y = y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
                % set line width
                if z_lin==1 || z_lin == length(roh_inc)
                    lin_wdth = 1;
                    lin_style = '-';
                    col = [0.5, 0.5, 0.5];
                else
                    lin_wdth = 0.5;
                    lin_style = '-';
                    col = [0.75, 0.75, 0.75];
                end
                D.UI.linVelH(z_lin) = plot(x, y, ...
                    'color', col, ...
                    'LineWidth', lin_wdth, ...
                    'LineStyle', lin_style, ...
                    'Parent', D.UI.axH(2), ...
                    'Visible', 'off');
            end
            
            % Clear vars
            clear roh_inc x y lin_wdth lin_style col;
            
            % CREATE RF BND LINES
            
            % Plot outer bounds
            [out_X,out_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.rfRad));
            xout = out_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
            yout = out_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
            D.UI.linRFH(1) = plot(xout, yout, ...
                'color', [0.5 0.5 0.5], ...
                'LineWidth', 2, ...
                'Parent', D.UI.axH(2), ...
                'Visible', 'off');
            
            % Plot inner bounds
            [in_X,in_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.rfRad-D.UI.rfTargWdt));
            xin = in_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
            yin = in_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
            D.UI.linRFH(2) = plot(xin, yin, ...
                'color', [0.5 0.5 0.5], ...
                'LineWidth', 2, ...
                'Parent', D.UI.axH(2), ...
                'Visible', 'off');
            
            % Clear vars
            clear out_X out_Y xout yout in_X in_Y xin yin;
            
            % GUI OBJECT POSITIONS
            
            % Questions dialogue pos
            D.UI.qstDlfPos = [D.UI.sc1(3) + D.UI.sc2(3)/2, D.UI.sc2(4)/2];
            
            % Bounds of plot space
            D.UI.main_ax_bounds = [ ...
                D.UI.ax_pos(1) - 0.05, ...
                D.UI.ax_pos(2) - 0.05, ...
                D.UI.ax_pos(3) + 0.1, ...
                D.UI.ax_pos(4) + 0.1];
            
            % Panel positions
            % setup
            D.UI.stup_pan_pos = ...
                [0,  ...
                1 - 0.46, ...
                D.UI.main_ax_bounds(1), ...
                0.46];
            % recording
            D.UI.rec_pan_pos = ...
                [0, ...
                D.UI.stup_pan_pos(2) - 0.01 - 0.215, ...
                D.UI.main_ax_bounds(1), ...
                0.215];
            % console
            cnsl_pan_ht = 1 - ...
                D.UI.stup_pan_pos(4) - 0.01 - ...
                D.UI.rec_pan_pos(4) - 0.01 - ...
                0.05 - 0.01;
            D.UI.cnsl_pan_pos = ...
                [0, ...
                0.05 + 0.01, ...
                D.UI.main_ax_bounds(1)+0.05, ...
                cnsl_pan_ht];
            
            % Pixel to normalzied unit conversion factor
            D.UI.pxl2norm_x = 1/fg_wh(1);
            D.UI.pxl2norm_y = 1/fg_wh(2);
            
            % Get scaled up pixel vals
            rad_pxl = ceil(D.UI.rfRad*D.UI.cm2pxl)*4;
            lim_pxl = D.UI.vtRes*4;
            pad_pxl = round((lim_pxl-(rad_pxl*2))/2);
            
            % Make mask
            [colNums, rowNums] = meshgrid(1:rad_pxl*2, 1:rad_pxl*2);
            D.PAR.pathMask = ...
                (rowNums - rad_pxl).^2 + (colNums - rad_pxl).^2 <= rad_pxl.^2;
            D.PAR.pathMask = ~padarray(D.PAR.pathMask, [pad_pxl, pad_pxl]);
            
            % Set lims
            set(D.UI.axH(4), ...
                'XLim', [1,size(D.PAR.pathMask,1)],...
                'YLim', [1,size(D.PAR.pathMask,2)]);
            
            % Show mask
            D.UI.imgMaskRFH = ...
                imshow(ones(size(D.PAR.pathMask,1),size(D.PAR.pathMask,2)), ...
                'Parent',D.UI.axH(4));
            set(D.UI.imgMaskRFH, ...
                'AlphaData', D.PAR.pathMask, ...
                'Visible', 'off');
            
            % Store path mask
            D.PAR.pathMask = imresize(~D.PAR.pathMask, [D.PAR.rfBins,D.PAR.rfBins]);
            
            % Clear vars
            clear rad_pxl lim_pxl pad_pxl mask;
            
            % MAKE WALL IMAGES
            img_ax_pos = [D.UI.ax_pos(1) - D.UI.ax_pos(3)*0.035, ...
                D.UI.ax_pos(2) - D.UI.ax_pos(4)*0.035, ...
                D.UI.ax_pos(3) * 1.07, ...
                D.UI.ax_pos(4) * 1.07];
            
            % Wall image axis
            set(D.UI.axH(5), 'Position', img_ax_pos);
            
            % Read in image
            [img{1}, ~, alph] = imread(D.DIR.ioWallImage);
            set(D.UI.axH(5), 'XLim', [0,size(img{1},2)], 'YLim', [0,size(img{1},1)]);
            img{1} = flip(img{1}, 1);
            mask = true(size(img{1}));
            % -40 deg
            img{2} = imrotate(img{1}, -40, 'crop');
            maskR = ~imrotate(mask, -40, 'crop');
            img{2}(maskR) = 255;
            % 40 deg
            img{3} = imrotate(img{1}, 40, 'crop');
            maskR = ~imrotate(mask, 40, 'crop');
            img{3}(maskR) = 255;
            
            % Store for later
            D.UI.wallImgH(1) = image(img{1}, 'Parent', D.UI.axH(5), 'Visible', 'on');
            D.UI.wallImgH(2) = image(img{2}, 'Parent', D.UI.axH(5), 'Visible', 'off');
            D.UI.wallImgH(3) = image(img{3}, 'Parent', D.UI.axH(5), 'Visible', 'off');
            % set alpha
            set(D.UI.wallImgH, 'AlphaData', alph);
            
            % Clear vars
            clear img alph;
            
            %% ========================= ADD UI OBJECTS ===============================
            
            %% ---------------------------SETUP PANEL----------------------------------
            
            % Position settings
            obj_gap_ht = 0.025;
            pos_ht_dflt = (1 - 2*obj_gap_ht) / 8;
            pos_lft_dflt = 0.05;
            pos_wd_dflt = 1-pos_lft_dflt*2;
            
            % Font settings
            head_font_sz = ...
                [10, 13/(D.UI.fg_pos(4)*D.UI.stup_pan_pos(4))*1.15] ;
            text1_font_sz = ....
                [9, 12/(D.UI.fg_pos(4)*D.UI.stup_pan_pos(4))] ;
            text2_font_sz = ...
                [9, 12/(D.UI.fg_pos(4)*D.UI.stup_pan_pos(4))] ;
            
            % SETUP PANEL
            D.UI.panStup = uipanel(...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'BorderType','line', ...
                'BorderWidth',4, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.disabledBckCol, ...
                'HighlightColor',D.UI.disabledBckCol, ...
                'FontSize',15, ...
                'FontWeight','Bold', ...
                'Title','Setup', ...
                'TitlePosition','centertop', ...
                'Clipping','on', ...
                'Position', D.UI.stup_pan_pos);
            
            % RAT SELECTION
            % text
            botm = 1 - obj_gap_ht - head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
            D.UI.txtRat = uicontrol('Style','text', ...
                'Parent',D.UI.panStup, ...
                'String','Rat Number', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'HorizontalAlignment', 'Center', ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontWeight','Bold', ...
                'FontName','MS Sans Serif', ...
                'FontSize', head_font_sz(1));
            % popupmenu
            botm = botm - pos_ht_dflt + head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)*1.9];
            D.UI.popRat = uicontrol('Style','popupmenu', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback', {@PopRat}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',text1_font_sz(1), ...
                'FontWeight','Bold', ...
                'String',D.UI.ratList, ...
                'Value',1);
            
            % ICR CONDITION
            % text
            botm = botm - head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
            D.UI.txtCond = uicontrol('Style','text', ...
                'Parent',D.UI.panStup, ...
                'String','Condition', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'HorizontalAlignment', 'Center', ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontWeight','Bold', ...
                'FontName','MS Sans Serif', ...
                'FontSize', head_font_sz(1));
            % popupmenu
            botm = botm - pos_ht_dflt + head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)*1.9];
            D.UI.popCond = uicontrol('Style','popupmenu', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback', {@PopCond}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',text1_font_sz(1), ...
                'FontWeight','Bold', ...
                'String',D.UI.condList, ...
                'Value',1);
            
            % ICR TASK
            % text
            botm = botm - head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
            D.UI.txtTask = uicontrol('Style','text', ...
                'Parent',D.UI.panStup, ...
                'String','Task', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'HorizontalAlignment', 'Center', ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontWeight','Bold', ...
                'FontName','MS Sans Serif', ...
                'FontSize', head_font_sz(1));
            % popupmenu
            botm = botm - pos_ht_dflt + head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)*1.9];
            D.UI.popTask = uicontrol('Style','popupmenu', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback', {@PopTask}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',text1_font_sz(1), ...
                'FontWeight','Bold', ...
                'String',D.UI.taskList, ...
                'Value',1);
            
            % REWARD DELAY
            % text
            botm = botm - head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
            D.UI.txtRwDl = uicontrol('Style','text', ...
                'Parent',D.UI.panStup, ...
                'String','Reward Delay', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'HorizontalAlignment', 'Center', ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontWeight','Bold', ...
                'FontName','MS Sans Serif', ...
                'FontSize', head_font_sz(1));
            % popupmenu
            botm = botm - pos_ht_dflt + head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)*1.9];
            D.UI.popRewDel = uicontrol('Style','popupmenu', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback', {@PopRewDel}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',text1_font_sz(1), ...
                'FontWeight','Bold', ...
                'String',D.UI.delList, ...
                'Value',1);
            
            % CUE BUTTON PANEL
            offset = 0.05;
            botm = botm - pos_ht_dflt - obj_gap_ht;
            pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
            D.UI.spanCue = uibuttongroup(...
                'Parent',D.UI.panStup, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'HighlightColor',D.UI.enabledCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'Title','Cue Condition', ...
                'TitlePosition','centertop', ...
                'FontWeight','Bold', ...
                'FontSize',head_font_sz(1), ...
                'Clipping','off');
            % CUE CONDITION
            % all
            botm = botm + text2_font_sz(2)*0.5;
            wdth = (pos_wd_dflt-(offset*2)) / 3;
            pos = [pos_lft_dflt + offset, botm, wdth, text2_font_sz(2)*1.5];
            D.UI.toggCue(1) = uicontrol('Style','togglebutton', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback',{@ToggCue}, ...
                'String','All', ...
                'UserData', 1, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', text2_font_sz(1), ...
                'Value',0);
            % half
            pos = [pos_lft_dflt + offset + wdth, botm, wdth, pos(4)];
            D.UI.toggCue(2) = uicontrol('Style','togglebutton', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback',{@ToggCue}, ...
                'String','Half', ...
                'UserData', 2, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', text2_font_sz(1), ...
                'Value',0);
            % none
            pos = [pos_lft_dflt + offset + wdth*2, botm, wdth, pos(4)];
            D.UI.toggCue(3) = uicontrol('Style','togglebutton', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback',{@ToggCue}, ...
                'String','None', ...
                'UserData', 3, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', text2_font_sz(1), ...
                'Value',0);
            
            % SOUND BUTTON PANEL
            offset = 0.05;
            botm = botm - pos_ht_dflt - obj_gap_ht;
            pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
            % panel
            D.UI.spanSnd = uibuttongroup(...
                'Parent',D.UI.panStup, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'HighlightColor', D.UI.enabledCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'Title','Sound', ...
                'TitlePosition','centertop', ...
                'FontWeight','Bold', ...
                'FontSize',head_font_sz(1), ...
                'Clipping','off');
            % SOUND CONDITION
            % white
            botm = botm + text2_font_sz(2)*0.5;
            wdth = (pos_wd_dflt-(offset*3)) / 2;
            pos = [pos_lft_dflt + offset, botm, wdth, text2_font_sz(2)*1.5];
            D.UI.toggSnd(1) = uicontrol('Style','togglebutton', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback',{@ToggSnd}, ...
                'UserData', 1, ...
                'String','White', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', text2_font_sz(1), ...
                'Value',0);
            % reward
            pos = [pos_lft_dflt + offset*2 + wdth, botm, wdth, pos(4)];
            D.UI.toggSnd(2) = uicontrol('Style','togglebutton', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback',{@ToggSnd}, ...
                'UserData', 2, ...
                'String','Reward', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', text2_font_sz(1), ...
                'Value',0);
            
            % SETUP DONE
            ht = 0.05 * 1/D.UI.stup_pan_pos(4);
            botm = botm/2 - ht/2;
            pos = [0.25, botm, 0.5, ht];
            D.UI.btnSetupDone = uicontrol('Style','togglebutton', ...
                'Parent', D.UI.panStup, ...
                'Enable', 'off', ...
                'String','DONE', ...
                'Callback', {@BtnSetupDone}, ...
                'UserData', 0, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize',12, ...
                'Value',0);
            
            %% --------------------------RECORD PANEL----------------------------------
            
            % Position settings
            obj_gap_ht = 0.025;
            pos_ht_dflt = (1 - 2*obj_gap_ht) / 3;
            pos_lft_dflt = 0.1;
            pos_wd_dflt = 1-pos_lft_dflt*2;
            
            % Font settings
            head_font_sz = ...
                [12, 16/(D.UI.fg_pos(4)* D.UI.rec_pan_pos(4))*1.15];
            text1_font_sz = ...
                [12, 16/(D.UI.fg_pos(4)* D.UI.rec_pan_pos(4))];
            btn_rot_px_ht = 40/(D.UI.fg_pos(4)* D.UI.rec_pan_pos(4));
            
            % RECORD PANEL
            D.UI.panRec = uipanel(...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'BorderType','line', ...
                'BorderWidth',4, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.disabledBckCol, ...
                'HighlightColor', D.UI.disabledBckCol, ...
                'FontSize',15, ...
                'FontWeight','Bold', ...
                'Title','Record', ...
                'TitlePosition','centertop', ...
                'Clipping','on', ...
                'Position',  D.UI.rec_pan_pos);
            
            % NEURALYNX BUTTON PANEL
            botm = 1 - obj_gap_ht - pos_ht_dflt;
            pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
            D.UI.spanNLX = uibuttongroup(...
                'Parent',D.UI.panRec, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'HighlightColor', D.UI.enabledCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'Title','Cheetah', ...
                'TitlePosition','centertop', ...
                'FontWeight','Bold', ...
                'FontSize',head_font_sz(1), ...
                'Clipping','off');
            
            % ACQ BUTTON
            offst = 0.1;
            ht = pos_ht_dflt*0.5;
            botm = botm + 0.025;
            wdth = pos_wd_dflt*0.45;
            pos = [pos_lft_dflt + offst*0.3, botm, wdth, ht];
            D.UI.btnAcq = uicontrol('Style','radiobutton', ...
                'Parent',D.UI.panRec, ...
                'UserData', 0, ...
                'Enable', 'on', ...
                'Visible', 'on', ...
                'Callback', {@BtnAcq}, ...
                'UserData', 0, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'String','ACQ', ...
                'CData',D.UI.radBtnCmap{1}, ...
                'FontWeight','Bold', ...
                'FontWeight','Bold', ...
                'FontSize', text1_font_sz(1), ...
                'Value',0);
            % REC BUTTON
            pos = [pos_lft_dflt + wdth + offst*0.4, botm, wdth, ht];
            D.UI.btnRec = uicontrol('Style','radiobutton', ...
                'Parent',D.UI.panRec, ...
                'Enable', 'on', ...
                'Visible', 'on', ...
                'Callback', {@BtnRec}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'String','REC', ...
                'CData',D.UI.radBtnCmap{1}, ...
                'FontWeight','Bold', ...
                'FontWeight','Bold', ...
                'FontSize', text1_font_sz(1), ...
                'Value',0);
            
            % ICR BUTTON
            botm = botm - obj_gap_ht - btn_rot_px_ht;
            pos = [pos_lft_dflt, botm, pos_wd_dflt, btn_rot_px_ht];
            D.UI.btnICR = uicontrol('Style','push', ...
                'Parent',D.UI.panRec, ...
                'Enable', 'off', ...
                'Callback', {@BtnICR}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize',12, ...
                'UserData', false);
            
            % RECORDING DONE
            ht = 0.05 * 1/D.UI.rec_pan_pos(4);
            botm = botm/2 - ht/2;
            pos = [0.25, botm, 0.5, ht];
            D.UI.btnRecDone = uicontrol('Style','togglebutton', ...
                'Parent', D.UI.panRec, ...
                'Enable', 'off', ...
                'String','DONE', ...
                'Callback', {@BtnRecDone}, ...
                'UserData', 0, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize',12);
            
            %% --------------------------CONSOLE PANEL--------------------------------
            
            % CONSOLE PANEL
            D.UI.panConsole = uipanel(...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'BorderType','line', ...
                'BorderWidth',4, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.disabledBckCol, ...
                'HighlightColor', D.UI.disabledBckCol, ...
                'FontSize',15, ...
                'FontWeight','Bold', ...
                'Title','Console', ...
                'TitlePosition','centertop', ...
                'Clipping','on', ...
                'Position', D.UI.cnsl_pan_pos);
            
            % Editable text list
            D.UI.listConsole = uicontrol(...
                'Style','listbox',...
                'Parent',D.UI.panConsole,...
                'Units','normalized',...
                'Position',[0,0,1,1],...
                'HorizontalAlignment', 'Left', ...
                'FontSize', 7, ...
                'FontName','Monospaced', ...
                'Max', 1000, ...
                'Enable','inactive',...
                'String',D.DB.consoleStr);
            
            %% --------------------------OTHER BUTTONS---------------------------------
            
            % SAVE SESSION DATA
            pos = [0, D.UI.main_ax_bounds(2), D.UI.main_ax_bounds(1)/2, 0.05];
            D.UI.btnSave = uicontrol('Style','push', ...
                'Enable', 'off', ...
                'String','SAVE', ...
                'Callback', {@BtnSave}, ...
                'Unit', 'Normalized', ...
                'Position', pos, ...
                'FontName', D.UI.btnFont, ...
                'FontSize',14, ...
                'Value',0);
            
            % QUIT ALL
            pos = [D.UI.main_ax_bounds(1)/2, D.UI.main_ax_bounds(2), D.UI.main_ax_bounds(1)/2, 0.05];
            D.UI.btnQuit = uicontrol('Style','push', ...
                'Enable', 'off', ...
                'String','QUIT', ...
                'Callback', {@BtnQuit}, ...
                'Unit', 'Normalized', ...
                'Position', pos, ...
                'FontName', D.UI.btnFont, ...
                'FontSize',14);
            
            % HALT ROBOT
            wdth = 0.1;
            pos = [D.UI.main_ax_bounds(1)+0.01, D.UI.main_ax_bounds(4)-0.01-0.06, 0.1, 0.06];
            D.UI.btnHaltRob = uicontrol('Style','togglebutton', ...
                'Parent',FigH, ...
                'String','Halt Robot', ...
                'Callback', {@BtnHaltRob}, ...
                'Enable', 'off', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', 12, ...
                'Value', 0);
            
            % BULLDOZE RAT
            % popup
            pos = [pos(1), pos(2)-0.03-0.01, wdth, 0.03];
            D.UI.popBulldoze = uicontrol('Style','popupmenu', ...
                'Parent',FigH, ...
                'Callback', {@Bulldoze}, ...
                'Units','Normalized', ...
                'Enable', 'off', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',10, ...
                'FontWeight','Bold', ...
                'String',D.UI.bullList);
            % toggle
            pos = [pos(1), pos(2), wdth-wdth*0.125, 0.03];
            D.UI.btnBulldoze = uicontrol('Style','togglebutton', ...
                'Parent',FigH, ...
                'String','Bulldoze', ...
                'Callback', {@Bulldoze}, ...
                'Enable', 'off', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', 10, ...
                'Value', 0);
            % secret speed field
            D.UI.editBulldoze = uicontrol(...
                'Parent',FigH, ...
                'Units','Normalized',...
                'Position',[pos(1)+wdth, pos(2), wdth*0.5, pos(4)],...
                'Style','edit',...
                'HorizontalAlignment', 'Left', ...
                'FontSize', 12, ...
                'FontName','Monospaced', ...
                'Max', 1000, ...
                'Enable','inactive',...
                'Visible', 'off',...
                'String',num2str(D.PAR.bullSpeed));
            
            % REWARD BUTTON
            % popup
            pos = [pos(1), pos(2)-0.03-0.01, wdth, 0.03];
            D.UI.popReward = uicontrol('Style','popupmenu', ...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'Enable', 'off', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',10, ...
                'FontWeight','Bold', ...
                'String',cellstr(num2str(D.PAR.zoneRewDur')), ...
                'Value', D.I.zone);
            % toggle
            pos = [pos(1), pos(2), wdth-wdth*0.125, 0.03];
            D.UI.btnReward = uicontrol('Style','push', ...
                'Parent',FigH, ...
                'String','Reward', ...
                'Callback', {@BtnReward}, ...
                'Enable', 'off', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', 10, ...
                'Value',0);
            
            % BLOCK CUE
            pos = [pos(1), pos(2)-0.03-0.01, wdth*0.7, 0.03];
            D.UI.btnBlockCue = uicontrol('Style','togglebutton', ...
                'Parent',FigH, ...
                'String','Block Cue', ...
                'Callback', {@BlockCue}, ...
                'Enable', 'off', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', 10, ...
                'UserData', 1, ...
                'Value', 0);
            
            % ALL CUE
            pos = [pos(1), pos(2)-0.03-0.01, wdth*0.7, 0.03];
            D.UI.btnAllCue = uicontrol('Style','togglebutton', ...
                'Parent',FigH, ...
                'String','All Cue', ...
                'Callback', {@AllCue}, ...
                'Enable', 'off', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', 10, ...
                'UserData', 1, ...
                'Value', 0);
            
            % CLEAR VT BUTTON
            pos = [pos(1), 0.01, 0.1, 0.03];
            D.UI.btnClrVT = uicontrol('Style','push', ...
                'Parent',FigH, ...
                'Enable', 'off', ...
                'Callback', {@BtnClrVT}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'String','Clear VT Data', ...
                'BackgroundColor', D.UI.disabledBckCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', 10, ...
                'Value',0);
            
            %% --------------------------PRINTED INFO----------------------------------
            
            % Position settings
            pos_lft_dflt = (D.UI.main_ax_bounds(1)+D.UI.main_ax_bounds(3)) + 0.005;
            pos_wd_dflt = 1 - (D.UI.main_ax_bounds(1)+D.UI.main_ax_bounds(3)) - 2*0.005;
            pan_lft = (D.UI.main_ax_bounds(1)+D.UI.main_ax_bounds(3));
            pan_wd = 1 - (D.UI.main_ax_bounds(1)+D.UI.main_ax_bounds(3));
            
            % Font settings
            head_font_sz = ...
                [12, 16*D.UI.pxl2norm_y*1.25];
            text_font_sz = ...
                [10, 13*D.UI.pxl2norm_y*1.25];
            dd_font_sz = ...
                [10, 22*D.UI.pxl2norm_y];
            
            % RAT INFO
            
            % Number of text lines not including header
            nlines(1) = 12;
            
            % pannel
            D.UI.sesInfPanHt = ...
                (nlines(1)-2)*text_font_sz(2) + ...
                2*dd_font_sz(2) + ...
                head_font_sz(2) + ...
                4*D.UI.pxl2norm_y*2;
            D.UI.sesInfPanBtm = ...
                1 - D.UI.sesInfPanHt;
            D.UI.sesInfPanPos = [...
                pan_lft, ...
                D.UI.sesInfPanBtm, ...
                pan_wd, ...
                D.UI.sesInfPanHt];
            D.UI.panSesInf = uipanel(...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'BorderType','line', ...
                'BorderWidth',4, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'HighlightColor', D.UI.disabledBckCol, ...
                'Position',  D.UI.sesInfPanPos);
            % heading
            botm = D.UI.sesInfPanBtm + ...
                D.UI.sesInfPanHt - ...
                head_font_sz(2) - ...
                4*D.UI.pxl2norm_y;
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
            D.UI.txtSesInfHed = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','Session Info', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', head_font_sz(1));
            
            % ICR ses info
            botm = botm - 7*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*6];
            D.UI.txtSesInf(1) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontWeight','Light', ...
                'FontSize', text_font_sz(1));
            
            % Rotation ses info
            botm = botm - 3*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*2];
            D.UI.txtSesInf(2) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontWeight','Light', ...
                'FontSize', text_font_sz(1));
            % laps per dropdown
            botm = botm - dd_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
            D.UI.popLapsPerRot = uicontrol('Style','popupmenu', ...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontSize',dd_font_sz(1), ...
                'FontWeight','Light', ...
                'Visible','off', ...
                'Value',1);
            % rotation position dropdown
            botm = botm - dd_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
            D.UI.popRotPos = uicontrol('Style','popupmenu', ...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontSize',dd_font_sz(1), ...
                'FontWeight','Light', ...
                'Visible','off', ...
                'Value',1);
            
            % PERFORMANCE INFO
            
            % Number of text lines not including header
            nlines(2) = 19;
            
            % pannel
            D.UI.perfInfPanHt = ...
                (nlines(2)-1)*text_font_sz(2) + ...
                2*dd_font_sz(2) + ...
                head_font_sz(2) + ...
                4*D.UI.pxl2norm_y*2;
            D.UI.perfInfPanBtm = ...
                D.UI.sesInfPanBtm - ...
                D.UI.perfInfPanHt - ...
                2*D.UI.pxl2norm_y;
            D.UI.perfInfPanPos = [...
                pan_lft, ...
                D.UI.perfInfPanBtm, ...
                pan_wd, ...
                D.UI.perfInfPanHt];
            D.UI.panPerfInf = uipanel(...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'BorderType','line', ...
                'BorderWidth',4, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'HighlightColor', D.UI.disabledBckCol, ...
                'Position',  D.UI.perfInfPanPos);
            % heading
            botm = D.UI.perfInfPanBtm + ...
                D.UI.perfInfPanHt - ...
                head_font_sz(2) - ...
                4*D.UI.pxl2norm_y;
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
            D.UI.txtPerfInfHed = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','Performance Info', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', head_font_sz(1));
            % total
            botm = botm - 6*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*5];
            D.UI.txtPerfInf(4) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            % lap time dropdown
            botm = botm - dd_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
            D.UI.popLapTim = uicontrol('Style','popupmenu', ...
                'Parent',FigH, ...
                'String','Lap Times', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontSize',dd_font_sz(1), ...
                'FontWeight','Light', ...
                'Visible','off', ...
                'Value',1);
            % reward info dropdown
            botm = botm - dd_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
            D.UI.popRewInfo = uicontrol('Style','popupmenu', ...
                'Parent',FigH, ...
                'String','Reward Info', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontSize',dd_font_sz(1), ...
                'FontWeight','Light', ...
                'Visible','off', ...
                'Value',1);
            % standard
            botm = botm - 3*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*2];
            D.UI.txtPerfInf(1) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', [0.5,0.5,0.5], ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            % 40 deg
            botm = botm - 2*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*2];
            D.UI.txtPerfInf(2) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.rotCol(2,:), ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            % 0 deg
            botm = botm - 2*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*2];
            D.UI.txtPerfInf(3) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.rotCol(1,:), ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            
            % rat velocity
            botm = botm - 2*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)];
            D.UI.txtPerfInf(5) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.ratNowCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            % robot velocity
            botm = botm - text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)];
            D.UI.txtPerfInf(6) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.robNowCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            
            % battery voltage
            botm = botm - 2*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)];
            D.UI.txtPerfInf(7) = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            
            % TIMER INFO
            
            % Number of text lines not including header
            nlines(3) = 10;
            
            % pannel
            D.UI.timInfPanHt = ...
                D.UI.sesInfPanBtm - D.UI.perfInfPanHt - 4*D.UI.pxl2norm_y;
            D.UI.timInfPanBtm = 0;
            D.UI.timInfPanPos = [...
                pan_lft, ...
                D.UI.timInfPanBtm, ...
                pan_wd, ...
                D.UI.timInfPanHt];
            D.UI.panTimInf = uipanel(...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'BorderType','line', ...
                'BorderWidth',4, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'HighlightColor', D.UI.disabledBckCol, ...
                'Position',  D.UI.timInfPanPos);
            % heading
            botm = D.UI.timInfPanHt - 4*D.UI.pxl2norm_y - head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
            D.UI.txtTimInfHed = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','Timer Info', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', head_font_sz(1));
            % time elapsed info
            botm = botm - text_font_sz(2)*5;
            pos = [pos_lft_dflt, botm, ...
                pos_wd_dflt, text_font_sz(2)*4];
            D.UI.txtTimElspInf = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            
            % time start info
            botm = botm - text_font_sz(2);
            pos = [pos_lft_dflt, botm, ...
                pos_wd_dflt, text_font_sz(2)*1];
            D.UI.txtTimStrInf = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontWeight','Light', ...
                'FontSize', text_font_sz(1));
            pos = [pos(1)+pos_wd_dflt/2, pos(2), (pos_wd_dflt/2)*0.75, pos(4)];
            D.UI.editTimStrInf = uicontrol(...
                'Parent',FigH,...
                'Units','normalized',...
                'Position',pos,...
                'Style','edit',...
                'HorizontalAlignment', 'Center', ...
                'FontSize', text_font_sz(1), ...
                'FontName','Monospaced', ...
                'Max', 1000, ...
                'Enable','on', ...
                'Visible', 'off');
            % time stop info
            botm = botm - text_font_sz(2);
            pos = [pos_lft_dflt, botm, ...
                pos_wd_dflt, text_font_sz(2)*1];
            D.UI.txtTimEndInf = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontWeight','Light', ...
                'FontSize', text_font_sz(1));
            pos = [pos(1)+pos_wd_dflt/2, pos(2), (pos_wd_dflt/2)*0.75, pos(4)];
            D.UI.editTimEndInf = uicontrol(...
                'Parent',FigH,...
                'Units','normalized',...
                'Position',pos,...
                'Style','edit',...
                'HorizontalAlignment', 'Center', ...
                'FontSize', text_font_sz(1), ...
                'FontName','Monospaced', ...
                'Max', 1000, ...
                'Enable','on', ...
                'Visible', 'off');
            
            % Debug info
            botm = botm - text_font_sz(2)*3;
            pos = [pos_lft_dflt, botm, ...
                pos_wd_dflt, text_font_sz(2)*2.35];
            D.UI.txtTimDebug = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledPrintFrgCol, ...
                'FontName','Courier New', ...
                'FontWeight','Light', ...
                'FontSize', 6);
            
            %% ========================================================================
            
            %% MAKE FIGURE VISIBLE
            
            % Set position
            movegui(FigH, 'center')
            % Bring UI to top
            uistack(FigH, 'top')
            % Make visible
            set(FigH, 'Visible', 'on');
            
            % Log/print
            Console_Write('[UI_Setup] FINISHED: UI Setup');
            
        end
        
        % ------------------------------NLX SETUP--------------------------
        function[] = NLX_Setup()
            
            %% DEFINE IO PARAMETERS
            
            % Note: Opto issolator is powered of of Port-0 Bit-3
            % Each event has a unique bit associated with it
            
            % DygitalLynx server IP
            D.NLX.IP = '192.168.3.100'; %the server we are going to connect with to get streaming data
            
            % Aquisition ent names
            D.NLX.vt_rat_ent = 'VT1';
            D.NLX.vt_rob_ent = 'VT2';
            D.NLX.event_ent = 'Events';
            
            % TTL board/port parameters
            D.NLX.DevName = 'AcqSystem1_0'; % TTL board name
            
            % Port labels
            D.NLX.port_0 = '0';
            D.NLX.port_1 = '1';
            
            % INPUT TTL PIN/BIT
            % vars = (port, pin)
            
            % Audio channels
            D.NLX.snd_rt_wn_bit = [{'1'},{'5'}];
            D.NLX.snd_lft_rt_bit = [{'1'},{'4'}];
            
            % IR time sync LED
            D.NLX.ir_ts_bit = [{'1'},{'7'}];
            
            % Rew
            D.NLX.rew_on_bit = [{'1'},{'0'}];
            D.NLX.rew_off_bit = [{'1'},{'1'}];
            
            % PID mode
            D.NLX.pid_run_bit = [{'0'},{'0'}];
            D.NLX.pid_stop_bit = [{'0'},{'1'}];
            
            % Bulldozer state
            D.NLX.bull_run_bit = [{'0'},{'2'}];
            D.NLX.bull_stop_bit = [{'0'},{'3'}];
            
            % Photo Transducers
            D.NLX.north_bit = [{'0'},{'7'}];
            D.NLX.west_bit = [{'0'},{'6'}];
            D.NLX.south_bit = [{'0'},{'5'}];
            D.NLX.east_bit = [{'0'},{'4'}];
            
            % TTL STRINGS
            
            % Audio channels
            D.NLX.snd_rt_wn_str = 'TTL_Sound_Right_On';
            D.NLX.snd_lft_rt_str = 'TTL_Sound_Left_On';
            
            % Rew
            D.NLX.rew_on_str = 'TTL_Rew_On';
            D.NLX.rew_off_str = 'TTL_Rew_Off';
            
            % PID mode
            D.NLX.pid_run_str = 'TTL_PID_Run';
            D.NLX.pid_stop_str = 'TTL_PID_Stop';
            
            % Bulldozer state
            D.NLX.bull_run_str = 'TTL_Bulldozer_Run';
            D.NLX.bull_stop_str = 'TTL_Bulldozer_Stop';
            
            % Photo Transducers
            D.NLX.north_str = 'TTL_North_On';
            D.NLX.west_str = 'TTL_West_On';
            D.NLX.south_str = 'TTL_South_On';
            D.NLX.east_str = 'TTL_East_On';
            
            % IR time sync LED
            D.NLX.ir_ts_str = 'IR_Time_Sync';
            
            % EVENT STRINGS
            
            % Rat in event command strings
            D.NLX.rat_in_evt = '-PostEvent Post_Rat_In 201 0';
            D.NLX.rat_out_evt = '-PostEvent Post_Rat_Out 202 0';
            
            % Feeder cue event command strings
            D.NLX.cue_on_evt = '-PostEvent Post_Feeder_Cue_On 203 0';
            D.NLX.cue_off_evt = '-PostEvent Post_Feeder_Cue_Off 204 0';
            
            % ICR event command strings
            D.NLX.rot_evt{1} = '-PostEvent Post_Rotation_0_Deg 205 0';
            D.NLX.rot_evt{2} = '-PostEvent Post_Rotation_40_Deg 206 0';
            
            % Reward Zoneet and Duration
            D.NLX.rew_evt = '-PostEvent Post_Reward_Zone:%d_Duration:%d 211 0';
            
            % Session end command string
            D.NLX.ses_end_evt = '-PostEvent Post_Session_End 211 0';
            
            %% CONNECT TO NETCOM
            
            % Set flag
            D.NLX.running = false;
            D.NLX.connected = false;
            
            % Wait for Cheetah to open
            Console_Write('[NLX_Setup] RUNNING: Confirm Cheetah.exe Running...');
            while ~D.NLX.running && ...
                    ~isMatSolo && ...
                    ~doExit
                % Check EXE status
                [~,result] = system('tasklist /FI "imagename eq cheetah.exe" /fo table /nh');
                D.NLX.running = any(strfind(result, 'Cheetah.exe'));
            end
            
            % Log/print
            if D.NLX.running
                Console_Write('[NLX_Setup] FINISHED: Confirm Cheetah.exe Running');
                % Pause before connecting
                if ~isMatSolo
                    Console_Write('[NLX_Setup] RUNNING: Wait for Cheetah.exe to Load...');
                    tic;
                    while (toc < 5 && ~doExit)
                    end
                    if ~doExit
                        Console_Write('[NLX_Setup] FINISHED: Wait for Cheetah.exe to Load');
                    else
                        Console_Write('**WARNING** [NLX_Setup] ABORTED: Wait for Cheetah.exe to Load');
                    end
                end
            else
                Console_Write('**WARNING** [NLX_Setup] ABORTED: Confirm Cheetah.exe Running');
            end
            
            % Load NetCom into Matlab, and connect to the NetCom server
            Console_Write(sprintf('[NLX_Setup] RUNNING: Connect to NLX... IP=%s', ...
                D.NLX.IP));
            
            % Run if not connected already
            if NlxAreWeConnected() ~= 1
                
                % Keep attempting till success
                while ~D.NLX.connected && ...
                        ~isMatSolo && ...
                        ~doExit
                    
                    % Attempt connection
                    D.NLX.connected = NlxConnectToServer(D.NLX.IP) == 1;
                    
                    % Identify id to server
                    if D.NLX.connected
                        NlxSetApplicationName('ICR_GUI');
                    end
                    
                end
                
                % Log/print status
                if D.NLX.connected
                    Console_Write('[NLX_Setup] FINISHED: Connect to NLX');
                else
                    Console_Write('**WARNING** [NLX_Setup] ABORTED: Connect to NLX');
                end
            else
                % Log/print already connected
                Console_Write('[NLX_Setup] CONFIRM: Already Connected to NLX');
            end
            
            % Bail if exit initiated
            if doExit
                return
            end
            
            %% CONFIGURE DIGITAL IO
            
            % Log/print
            Console_Write(sprintf('[NLX_Setup] RUNNING: Configure NLX...'));
            
            % SETUP CUBE
            
            % Pair Cube headstage (Should be commented out unless router has been unpluged)
            %NlxSendCommand('-SendLynxSXCommand AcqSystem1 -InitWHSPairing 30')
            
            % Turn on Cube LEDs
            %NlxSendCommand('-SendLynxSXCommand AcqSystem1 -WHSSetTrackingLED 1 1');
            
            % SETUP PORTS
            
            % Set port directions
            NlxSendCommand(['-SetDigitalIOportDirection ', D.NLX.DevName, ' ', D.NLX.port_0, ' Input']);
            NlxSendCommand(['-SetDigitalIOportDirection ', D.NLX.DevName, ' ', D.NLX.port_1, ' Input']);
            
            % Enable digital io events
            NlxSendCommand(['-SetDigitalIOEventsEnabled ', D.NLX.DevName, ' ', D.NLX.port_0, ' True']);
            NlxSendCommand(['-SetDigitalIOEventsEnabled ', D.NLX.DevName, ' ', D.NLX.port_1, ' True']);
            
            % CONFIGURE TTL EVENTS
            
            % Audio channels
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.snd_rt_wn_bit{1}, ' ', D.NLX.snd_rt_wn_bit{2}, ' ', D.NLX.snd_rt_wn_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.snd_lft_rt_bit{1}, ' ', D.NLX.snd_lft_rt_bit{2}, ' ', D.NLX.snd_lft_rt_str]);
            
            % Reward
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.rew_on_bit{1}, ' ', D.NLX.rew_on_bit{2}, ' ', D.NLX.rew_on_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.rew_off_bit{1}, ' ', D.NLX.rew_off_bit{2}, ' ', D.NLX.rew_off_str]);
            
            % Pid state
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.pid_run_bit{1}, ' ', D.NLX.pid_run_bit{2}, ' ', D.NLX.pid_run_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.pid_stop_bit{1}, ' ', D.NLX.pid_stop_bit{2}, ' ', D.NLX.pid_stop_str]);
            
            % Bulldozer state
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.bull_run_bit{1}, ' ', D.NLX.bull_run_bit{2}, ' ', D.NLX.bull_run_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.bull_stop_bit{1}, ' ', D.NLX.bull_stop_bit{2}, ' ', D.NLX.bull_stop_str]);
            
            % Photo transdicers
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.north_bit{1}, ' ', D.NLX.north_bit{2}, ' ', D.NLX.north_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.west_bit{1}, ' ', D.NLX.west_bit{2}, ' ', D.NLX.west_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.south_bit{1}, ' ', D.NLX.south_bit{2}, ' ', D.NLX.south_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.east_bit{1}, ' ', D.NLX.east_bit{2}, ' ', D.NLX.east_str]);
            
            % IR time sync LED
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.ir_ts_bit{1}, ' ', D.NLX.ir_ts_bit{2}, ' ', D.NLX.ir_ts_str]);
            
            % Bail if exit initiated
            if doExit
                return
            else
                Console_Write('[NLX_Setup] FINISHED: Configure NLX');
            end
            
            %% START STREAMING
            
            % Log/print
            Console_Write('[NLX_Setup] RUNNING: Open NLX Stream...');
            
            % Open the data stream for the VT acquisition entity.  This tells Cheetah to begin
            % streaming data for the VT acq ent.
            s1 = NlxOpenStream(D.NLX.vt_rat_ent);
            s2 = NlxOpenStream(D.NLX.vt_rob_ent);
            s3 = NlxOpenStream(D.NLX.event_ent);
            
            % Run BtnAcq
            set(D.UI.btnAcq, 'Value', 1)
            BtnAcq(D.UI.btnAcq);
            
            % Bail if exit initiated
            if doExit
                return
            else
                Console_Write(sprintf('[NLX_Setup] RUNNING: Open NLX Stream: Status vt1=%d vt2=%d evt=%d', s1, s2, s3));
                clear s1 s2 s3;
            end
            
            %% ENABLE SETUP PANEL ONCE CONNECTED
            
            % Log/print
            Console_Write('[NLX_Setup] RUNNING: Enable Setup Panel...');
            
            % Enable all setup buttons
            set(D.UI.panStup, 'ForegroundColor', D.UI.enabledCol, ...
                'HighlightColor',D.UI.enabledCol)
            set(D.UI.popRat, 'Enable', 'on')
            set(D.UI.popCond, 'Enable', 'on')
            set(D.UI.popTask, 'Enable', 'on')
            set(D.UI.popRewDel, 'Enable', 'on')
            set(D.UI.toggCue, 'Enable', 'on', ...
                'ForegroundColor', D.UI.enabledBtnFrgCol, ...
                'BackgroundColor', D.UI.enabledCol)
            set(D.UI.toggSnd, 'Enable', 'on', ...
                'ForegroundColor', D.UI.enabledBtnFrgCol, ...
                'BackgroundColor', D.UI.enabledCol)
            set(D.UI.btnSetupDone, 'Enable', 'on', ...
                'ForegroundColor', D.UI.enabledBtnFrgCol, ...
                'BackgroundColor', D.UI.enabledCol)
            
            % Enable Quit
            set( D.UI.btnQuit, 'Enable', 'on');
            
            % Log/print
            Console_Write('[NLX_Setup] FINISHED: Enable Setup Panel')
            
        end
        
        % ----------------------------FINISH SETUP-------------------------
        function [] = Finish_Setup()
            
            %% Update session specific vars
            
            % Get session number
            var_ind = ...
                ismember(D.SS_In_All.Properties.VariableNames, ['Session_',char(D.PAR.sesCond)]);
            col_ind = ismember([{'Track'},{'Forage'}], D.PAR.sesTask);
            D.PAR.sesNum = ...
                D.SS_In_All{D.PAR.ratInd, var_ind}(col_ind) + 1;
            
            % Save new session number back to SS_In_All
            D.SS_In_All{D.PAR.ratInd, var_ind}(col_ind) = D.PAR.sesNum;
            
            % Get session total
            D.PAR.sesNumAll = D.SS_In_All{D.PAR.ratInd, 'Session_Manual_Training'}(col_ind) + ...;
                D.SS_In_All{D.PAR.ratInd, 'Session_Behavior_Training'}(col_ind) + ...
                D.SS_In_All{D.PAR.ratInd, 'Session_Implant_Training'}(col_ind) + ...
                D.SS_In_All{D.PAR.ratInd, 'Session_Rotation'};
            
            % start quadrant
            D.PAR.ratStrQuad = categorical({'<undefined>'}, D.PAR.catStrQuad);
            if D.PAR.sesCond ~= 'Manual_Training'
                D.PAR.ratStrQuad = ... % [NE,SE,SW,NW]
                    D.SS_In_All.Start_Quadrant{D.PAR.ratInd}(D.PAR.sesNumAll);
            else
                % set start quad to feeder pos
                if D.PAR.ratFeedCnd == 'C1'
                    D.PAR.ratStrQuad(:) = 'NW';
                else
                    D.PAR.ratStrQuad(:) = 'SE';
                end
            end
            
            % Direction of rotation string
            D.PAR.ratRotDrc = ... % [CCW,CW]
                D.SS_In_All.Rotation_Direction{D.PAR.ratInd}(D.PAR.sesNum);
            
            % Rotations per session
            D.PAR.rotPerSes = ... % [2,4,6]
                str2double(char(D.SS_In_All.Rotations_Per_Session{D.PAR.ratInd}(D.PAR.sesNum,:)));
            
            % Rotations position
            D.PAR.rotPos = ... % [90,180,270]
                D.SS_In_All.Rotation_Positions{D.PAR.ratInd}(D.PAR.sesNum,:)';
            
            % Laps per session
            D.PAR.lapsPerRot = ... % [5:8,6:9,7:10]
                D.SS_In_All.Laps_Per_Rotation{D.PAR.ratInd}(D.PAR.sesNum,:)';
            
            % Days till rotation
            D.PAR.daysTilRot = ... % [5:8,6:9,7:10]
                D.SS_In_All.Days_Till_Rotation{D.PAR.ratInd}(D.PAR.sesNum);
            
            % Seed random number generator based on session number
            rng(D.PAR.sesNumAll)
            
            % Make rew per ses list
            txt = [{'Laps Per Rotation'}; ...
                cellfun(@(x,y) sprintf('%d:    %s', x, y), ...
                num2cell(1:length(D.PAR.rotPos))', ...
                cellstr(char(D.PAR.lapsPerRot)), 'Uni', false)];
            set(D.UI.popLapsPerRot, 'String', txt);
            
            % Make rot pos list
            txt = [{'Rotation Positions'}; ...
                cellfun(@(x,y) sprintf('%d:    %s', x, y), ...
                num2cell(1:length(D.PAR.rotPos))', ...
                cellstr(char(D.PAR.rotPos)), 'Uni', false)];
            set(D.UI.popRotPos, 'String', txt);
            
            % Genertate rotation button string
            D.UI.btnICRstr = ...
                [{sprintf('ROTATE 40%c %s', char(176), char(D.PAR.catRotDrc(D.PAR.catRotDrc ~= D.PAR.ratRotDrc)))}; ...
                {sprintf('ROTATE 40%c %s', char(176), char(D.PAR.ratRotDrc))}];
            
            % Modify vars based on rot dir
            if D.PAR.ratRotDrc == 'CCW'
                D.I.img_ind(2) = 2;
            elseif D.PAR.ratRotDrc == 'CW'
                D.I.img_ind(2) = 3;
            end
            
            %% Update and send AC.data values
            
            % Display image
            D.AC.data(2) = 1;
            
            % Sound stimuli (start without sound)
            D.AC.data(3) = single(D.UI.snd(1));
            
            % Post to AC computer
            SendM2AC();
            
            %% Update UI objects
            
            % Disable all setup buttons
            set(D.UI.panStup, 'ForegroundColor', D.UI.disabledBckCol, ...
                'HighlightColor',D.UI.disabledBckCol)
            set(D.UI.popRat, 'Enable', 'off')
            set(D.UI.popCond, 'Enable', 'off')
            set(D.UI.popTask, 'Enable', 'off')
            set(D.UI.popRewDel, 'Enable', 'off')
            set(D.UI.toggCue, 'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol)
            set(D.UI.toggSnd, 'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol)
            
            % Enable other buttons
            
            if D.PAR.sesCond ~= 'Manual_Training'
                
                % Halt Robot
                set(D.UI.btnHaltRob, ...
                    'Enable', 'on', ...
                    'BackgroundColor', D.UI.enabledCol, ...
                    'ForegroundColor' , D.UI.enabledBtnFrgCol);
                
                % Bulldoze
                set(D.UI.popBulldoze, ...
                    'Enable', 'on')
                set(D.UI.btnBulldoze, ...
                    'Enable', 'on', ...
                    'BackgroundColor', D.UI.enabledCol, ...
                    'ForegroundColor' , D.UI.enabledBtnFrgCol, ...
                    'Value', 1);
                % bulldoze delay default
                val = find(ismember(D.UI.bullList, [num2str(D.PAR.bullDel), ' sec']));
                set(D.UI.popBulldoze, ...
                    'Value', val);
                Bulldoze();
                
                if D.PAR.cueFeed == 'Half' || D.PAR.cueFeed == 'None'
                    % Block Cue
                    set(D.UI.btnBlockCue, ...
                        'Enable', 'on', ...
                        'BackgroundColor', D.UI.enabledCol, ...
                        'ForegroundColor' , D.UI.enabledBtnFrgCol);
                    % All Cue
                    set(D.UI.btnAllCue, ...
                        'Enable', 'on', ...
                        'BackgroundColor', D.UI.enabledCol, ...
                        'ForegroundColor' , D.UI.enabledBtnFrgCol);
                end
                
            end
            
            % Reward button
            set(D.UI.popReward, ...
                'Enable', 'on', ...
                'BackgroundColor', D.UI.enabledCol);
            set(D.UI.btnReward, ...
                'Enable', 'on', ...
                'BackgroundColor', D.UI.enabledCol);
            
            % Clear VT
            set(D.UI.btnClrVT, ...
                'Enable', 'on', ...
                'BackgroundColor', D.UI.enabledCol);
            
            % Enable acq record buttons
            set(D.UI.panRec, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'HighlightColor', D.UI.enabledCol)
            set(D.UI.btnRec, ...
                'Enable', 'on', ...
                'Visible', 'on')
            
            % Enable inf panels
            set(D.UI.panSesInf, 'HighlightColor', D.UI.enabledCol)
            set(D.UI.panPerfInf, 'HighlightColor', D.UI.enabledCol)
            set(D.UI.panTimInf, 'HighlightColor', D.UI.enabledCol)
            
            % Enable other popup menues
            set(D.UI.popLapsPerRot, 'Visible', 'on');
            set(D.UI.popRotPos, 'Visible', 'on');
            set(D.UI.popLapTim, 'Visible', 'on');
            set(D.UI.popRewInfo, 'Visible', 'on');
            
            %% Print session and performance and time info
            
            % Print session info
            dobNum = datenum(D.PAR.ratDOB, 'yyyy/mm/dd');
            agemnth = num2str(floor((now - dobNum)/365*12));
            ses = num2str(D.PAR.sesNum);
            ses_tot = num2str(D.PAR.sesNumAll);
            infstr = sprintf([...
                'Session:%s%s|%s\n', ...
                'Age_Months:%s%s\n', ...
                'Age_Group:%s%s\n', ...
                'Feeder:%s%s\n', ...
                'Direction:%s%s\n', ...
                'Start_Quad:%s%s\n'], ...
                repmat('_',1,7), ses, ses_tot, ...
                repmat('_',1,4), agemnth, ...
                repmat('_',1,5), char(D.PAR.ratAgeGrp), ...
                repmat('_',1,8), char(D.PAR.ratFeedCnd), ...
                repmat('_',1,5), char(D.PAR.ratRotDrc), ...
                repmat('_',1,4), char(D.PAR.ratStrQuad));
            set(D.UI.txtSesInf(1),'String', infstr)
            
            % Rot cond specific info
            if D.PAR.sesCond == 'Rotation'
                rots = num2str(D.PAR.rotPerSes);
            else
                rots = 'NA';
                set(D.UI.popLapsPerRot, 'Enable', 'off');
                set(D.UI.popRotPos, 'Enable', 'off');
            end
            infstr = sprintf([...
                'Rotation Info\n', ...
                'Rotations:%s%s\n'], ...
                repmat('_',1,5), rots);
            set(D.UI.txtSesInf(2),'String', infstr)
            
            
            % Print performance info
            % total
            infstr = sprintf([...
                'Laps______Total:%s%d\n', ...
                'Rewards___Total:%s%d\n', ...
                'Rotations_Total:%s%d\n', ...
                'Missed_Rewards_:%s%d|%d\n',...
                'Bulldozings____:%s%d'], ...
                repmat('_',1,1), 0, ...
                repmat('_',1,1), 0, ...
                repmat('_',1,1), 0, ...
                repmat('_',1,1), 0,0, ...
                repmat('_',1,1), 0);
            set(D.UI.txtPerfInf(4), 'String', infstr)
            % standard
            infstr = sprintf([...
                'Laps______Stand:%s%d\n', ...
                'Rewards___Stand:%s%d'], ...
                repmat('_',1,2), 0, ...
                repmat('_',1,2), 0);
            set(D.UI.txtPerfInf(1), 'String', infstr)
            % 40 deg
            infstr = sprintf([...
                'Laps______40%c:%s%d|%d\n', ...
                'Rewards___40%c:%s%d|%d'], ...
                char(176), repmat('_',1,4), 0,0, ...
                char(176), repmat('_',1,4), 0,0);
            set(D.UI.txtPerfInf(2), 'String', infstr)
            % 0 deg
            infstr = sprintf([...
                'Laps______0%c:%s%d|%d\n', ...
                'Rewards___0%c:%s%d|%d'], ...
                char(176), repmat('_',1,5), 0,0, ...
                char(176), repmat('_',1,5), 0,0);
            set(D.UI.txtPerfInf(3), 'String', infstr)
            
            % rat vel
            infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', 0, 0, 0);
            set(D.UI.txtPerfInf(5), 'String', infstr)
            % rob vel
            infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', 0, 0, 0);
            set(D.UI.txtPerfInf(6), 'String', infstr)
            
            % bat volt
            infstr = sprintf('Battery:_%0.1fV', 0);
            set(D.UI.txtPerfInf(7), 'String', infstr)
            
            % Print time info
            % elapsed
            infstr = sprintf([ ...
                '(SES):%s%s\n', ...
                '(REC):%s%s\n', ...
                '(LAP):%s%s\n', ...
                '(RUN):%s%s\n'], ...
                repmat('_',1,6), datestr(0, 'HH:MM:SS'), ...
                repmat('_',1,6), datestr(0, 'HH:MM:SS'), ...
                repmat('_',1,6), datestr(0, 'HH:MM:SS'), ...
                repmat('_',1,6), datestr(0, 'HH:MM:SS'));
            set(D.UI.txtTimElspInf, 'String', infstr)
            % start
            infstr = sprintf( ...
                'Start:%s', repmat('_',1,6));
            set(D.UI.txtTimStrInf, 'String', infstr)
            infstr = datestr(startTime, 'HH:MM:SS');
            set(D.UI.editTimStrInf, 'String', infstr, ...
                'Visible', 'on')
            % end
            infstr = sprintf( ...
                'Stop_:%s\n', repmat('_',1,6));
            set(D.UI.txtTimEndInf, 'String', infstr)
            infstr = datestr(0, 'HH:MM:SS');
            set(D.UI.editTimEndInf, 'String', infstr, ...
                'Visible', 'on')
            % loop
            infstr = sprintf( ...
                [...
                'RD: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                'RT: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                'DN: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                'Lp: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                ], ...
                0, 0, 0, 0, ...
                0, 0, 0, 0, ...
                0, 0, 0, 0, ...
                0, 0, 0, 0 ...
                );
            set(D.UI.txtTimDebug, 'String', infstr)
            
            %% Get bounds for various opperations
            
            % Specify reward feeder locations
            % NOTE: Feeder index is based on the position of the feeder with
            % respect to the 0 deg point (East quadrant)in the arena
            rewFeeds = [11, 15, 7; 29, 33, 25];
            
            % Feeder paramiters
            % boundary before and after rew feeder (deg)
            D.PAR.trigDist = rad2deg(D.PAR.feedDist - D.PAR.setPoint);
            D.PAR.feedSet = [...
                D.PAR.trigDist - 2.5, ...
                D.PAR.trigDist + 2.5];
            
            % Get reward and unrewarded feeder based on rotation direction
            % corrected index
            D.UI.rewFeed = rewFeeds(D.PAR.ratFeedCnd_Num, D.I.img_ind);
            D.UI.oppFeed = rewFeeds([1, 2] ~=  D.PAR.ratFeedCnd_Num, D.I.img_ind);
            
            % Calculate feeder locations
            
            % Calculate all feeder locations
            fdLocs = circshift((0:10:350)+5,[0,0]);
            
            % Calculate all feed locs
            [fd_X,fd_Y] = pol2cart(deg2rad(fdLocs), ones(1,length(fdLocs)) * D.UI.arnRad);
            % all feeders x
            D.UI.fd_x = fd_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
            % all feeders y
            D.UI.fd_y = fd_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
            
            % Save reward feeder rad pos
            D.UI.rewZoneRad(1) = deg2rad(fdLocs(D.UI.rewFeed(1)));
            D.UI.rewZoneRad(2) = deg2rad(fdLocs(D.UI.rewFeed(2)));
            % with setpoint correction
            D.UI.rewRatHead(1:2) = ...
                D.UI.rewZoneRad + deg2rad(D.PAR.trigDist);
            
            % TRACK REWARD ZONE BOUNDS
            D.PAR.rewZoneBnds = NaN(length(D.PAR.zoneLocs),2,2);
            for z_zone = 1:length(D.PAR.zoneLocs)
                D.PAR.rewZoneBnds(z_zone,:,1) = [...
                    D.UI.rewZoneRad(1) + deg2rad(D.PAR.feedSet(1) + D.PAR.zoneLocs(z_zone)), ...
                    D.UI.rewZoneRad(1) + deg2rad(D.PAR.feedSet(2) + D.PAR.zoneLocs(z_zone))];
                D.PAR.rewZoneBnds(z_zone,:,2) = [...
                    D.UI.rewZoneRad(2) + deg2rad(D.PAR.feedSet(1) + D.PAR.zoneLocs(z_zone)), ...
                    D.UI.rewZoneRad(2) + deg2rad(D.PAR.feedSet(2) + D.PAR.zoneLocs(z_zone))];
            end
            % [zone,min_max,rot_cond]
            D.PAR.rewZoneBnds = wrapTo2Pi(D.PAR.rewZoneBnds);
            
            % REWARD RESET BOUNDS
            D.UI.rewRstBnds(1,1:2) = D.PAR.rewZoneBnds(1,end,1) + [deg2rad(10),deg2rad(40)];
            D.UI.rewRstBnds(2,1:2) = D.PAR.rewZoneBnds(1,end,2) + [deg2rad(10),deg2rad(40)];
            D.UI.rewRstBnds = wrapTo2Pi(D.UI.rewRstBnds);
            
            % REWARD PASS BOUNDS
            D.UI.rewPassBnds(1,1:2) = D.PAR.rewZoneBnds(end,end,1) - [deg2rad(35), deg2rad(5)];
            D.UI.rewPassBnds(2,1:2) = D.PAR.rewZoneBnds(end,end,2) - [deg2rad(35), deg2rad(5)];
            D.UI.rewPassBnds = wrapTo2Pi(D.UI.rewPassBnds);
            
            % ROTATION BOUNDS
            
            % Calculate crossing points for 90 180 and 270 deg from rew feeders
            D.UI.rotLocs = [...
                (fdLocs(D.UI.rewFeed(1))-90:-90:fdLocs(D.UI.rewFeed(1))-270)', ...
                (fdLocs(D.UI.rewFeed(2))-90:-90:fdLocs(D.UI.rewFeed(2))-270)'];
            
            % Calculate 30 deg wide bounds for each rotation pos
            D.UI.rotBnds = arrayfun(@(x,y) cat(3, [x, x-30], [y, y-30]), ...
                D.UI.rotLocs(:,1), D.UI.rotLocs(:,2), 'Uni', false);
            D.UI.rotBnds = cell2mat(D.UI.rotBnds);
            % set range to [0, 360]
            D.UI.rotBnds = wrapTo360(D.UI.rotBnds);
            % convert to radians
            D.UI.rotBnds = deg2rad(D.UI.rotBnds);
            % Flip direction
            D.UI.rotBnds = flip(D.UI.rotBnds, 1);
            D.UI.rotBnds = flip(D.UI.rotBnds, 2);
            D.UI.rotBnds = wrapTo2Pi(D.UI.rotBnds);
            
            % LAP BOUNDS
            
            % Calculate lap count crossing points for every 90 deg
            % first ind is NE going ccw
            quadBndLocs = 45:90:360;
            % shift so that start quadrant is last entry
            strQuadInd = find(D.PAR.catStrQuad == D.PAR.ratStrQuad);
            lapBndLocs = circshift(quadBndLocs,[0, -1*(strQuadInd-1)])';
            % flip direction
            lapBndLocs = flip(lapBndLocs);
            
            % Calculate 90 deg wide bounds
            D.PAR.lapBnds = arrayfun(@(x) [x-45, x+45], lapBndLocs, 'Uni', false);
            D.PAR.lapBnds = cell2mat(D.PAR.lapBnds);
            % set range to [0, 360]
            D.PAR.lapBnds = wrapTo360(D.PAR.lapBnds);
            % convert to radians
            D.PAR.lapBnds = deg2rad(D.PAR.lapBnds);
            D.PAR.lapBnds = wrapTo2Pi(D.PAR.lapBnds);
            
            % START QUADRANT BOUNDS
            
            % Set start quadrant bound to 60 deg
            D.PAR.strQuadBnds = [lapBndLocs(end) - 30, ...
                lapBndLocs(end) + 30];
            % convert to radians
            D.PAR.strQuadBnds = deg2rad(D.PAR.strQuadBnds);
            D.PAR.strQuadBnds = wrapTo2Pi(D.PAR.strQuadBnds);
            
            % FORAGE REWARD TARGET BOUNDS
            D.PAR.rewTargBnds = NaN(length(D.PAR.pathTargArr),2);
            for z_targ = 1:length(D.PAR.pathTargArr)
                D.PAR.rewTargBnds(z_targ,:) = [...
                    Rad_Diff(deg2rad(D.PAR.pathTargArr(z_targ)), deg2rad(D.PAR.pathTargWdt/2)), ...
                    Rad_Sum(deg2rad(D.PAR.pathTargArr(z_targ)), deg2rad(D.PAR.pathTargWdt/2))];
            end
            % [zone,min_max,rot_cond]
            D.PAR.rewTargBnds = wrapTo2Pi(D.PAR.rewTargBnds);
            
            %% Plot UI features
            
            % Plot start quadrant
            [xbnd, ybnd] =  Get_Rad_Bnds(D.PAR.strQuadBnds);
            D.UI.ptchStQ = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                [0.5,0.5,0.5], ...
                'FaceAlpha',0.5, ...
                'Visible', 'off', ...
                'Parent',D.UI.axH(2));
            % add start text
            D.UI.txtStQ = ...
                text(mean(xbnd(:,round(size(xbnd,2)/2))), ...
                mean(ybnd(:,round(size(ybnd,2)/2))), ...
                'Start', ...
                'FontSize', 12, ...
                'FontWeight', 'Bold', ...
                'Color', D.UI.enabledCol, ...
                'HorizontalAlignment', 'Center', ...
                'Visible', 'off', ...
                'Parent',D.UI.axH(2));
            
            % Plot all feeders
            D.UI.fdAllH = plot(D.UI.fd_x, D.UI.fd_y, 'o', ...
                'MarkerFaceColor', [0.5 0.5 0.5], ...
                'MarkerEdgeColor', [0.1,0.1,0.1], ...
                'MarkerSize', 20, ...
                'Visible', 'on', ...
                'Parent', D.UI.axH(2));
            
            % Plot feeder pos in rad and cm
            fd_cm = (140*pi)-(140*pi)/36/2:-(140*pi)/36:(140*pi)/36/2;
            fd_rad = (2*pi)/36/2:(2*pi)/36:(2*pi)-(2*pi)/36/2;
            fdcmH = gobjects(1,36);
            for z_fd = 1:36
                fdcmH(z_fd) = text(...
                    D.UI.fd_x(z_fd), D.UI.fd_y(z_fd), ...
                    sprintf('%0.1f\n%0.0f', fd_rad(z_fd), fd_cm(z_fd)), ...
                    'Color', [1, 1, 1], ...
                    'HorizontalAlignment', 'center', ...
                    'FontSize', 6, ...
                    'FontWeight', 'bold', ...
                    'Visible', 'on', ...
                    'Parent', D.UI.axH(2));
            end
            
            %% Start recording a
            
            % Run BtnRec
            set(D.UI.btnRec,'Value', 1);
            BtnRec(D.UI.btnRec);
            
            %% Send setup command to robot
            % ses cond
            if D.PAR.sesCond == 'Manual_Training'
                d1 = 0;
            else
                d1 = 1;
            end
            
            % sound cond
            if all(~D.UI.snd)
                d2 = 0;
            elseif ~D.UI.snd(2)
                d2 = 1;
            else
                d2 = 2;
            end
            SendM2C('S', d1, d2);
            
            % Log/print
            Console_Write('[Finish_Setup] FINISHED: Session Setup');
            
        end
        
        % --------------------------TRACK TASK SETUP-----------------------
        
        function[] = Track_Task_Setup()
            
            %% REWARD ZONE SETUP
            
            % Initialize long distrebution
            sub_samp = 100;
            x_long = 1:11*sub_samp;
            zone_short = linspace(-25,25,11);
            
            % Setup axes
            wdth = 0.23;
            ht = 0.2;
            lft = (D.UI.main_ax_bounds(1)+(D.UI.main_ax_bounds(3)/2)) - wdth/2;
            botm = 0.5 - ht/2;
            zone_ax_pos = [...
                lft, ...
                botm, ...
                wdth, ...
                ht ...
                ];
            D.UI.axZoneH(1) = axes( ...
                'Color', 'none', ...
                'Position',zone_ax_pos, ...
                'XLim',[min(x_long)+sub_samp/2-1,max(x_long)-sub_samp/2], ...
                'Visible','off');
            hold on;
            D.UI.axZoneH(2) = axes( ...
                'Color','none', ...
                'Position',zone_ax_pos, ...
                'XLim',[min(zone_short)+2.5, max(zone_short)-2.5], ...
                'XTick',zone_short, ...
                'Visible','off');
            box on;
            hold on
            set(D.UI.axZoneH, ...
                'FontWeight', 'bold', ...
                'FontSize', 7)
            
            % Get long dist
            % front (i.e., < 0)
            han = sub_samp*11;
            dist_long = hanning(han);
            dist_long = (dist_long/max(dist_long)) * (max(dist_long)/5)*4 + max(dist_long)/5;
            dist_long = [min(dist_long)*ones((length(x_long)-han)/2, 1); dist_long];
            % set first and last values to zero
            dist_long([1:sub_samp,end-sub_samp+1:end]) = 0;
            % normalize
            dist_long = (dist_long/sum(dist_long))';
            % scale y axis
            set(D.UI.axZoneH(1), 'YLim', [0,max(dist_long)]);
            
            % Plot example distrebution
            y = dist_long;
            y(y == 0) = NaN;
            plot(x_long, y,'k', ...
                'LineWidth', 2, ...
                'Parent',D.UI.axZoneH(1));
            % plot center line
            x = repmat(round(max(x_long)/2),1,2);
            y = get(D.UI.axZoneH(1),'YLim');
            plot(x, y, 'k', ...
                'LineWidth', 2, ...
                'Color', [0, 0, 0], ...
                'Parent',D.UI.axZoneH(1));
            
            % Subsample long distrebution
            short_ind = floor(linspace(1, length(x_long), length(zone_short)));
            dist_short = dist_long(short_ind);
            
            % Plot point for reward size at each zone
            x =  short_ind(2:end-1);
            y = D.PAR.zoneRewDur/max(D.PAR.zoneRewDur)*max(dist_long);
            plot(x, y, 'or', ...
                'MarkerFaceColor', [0.5, 0.5, 0.5], ...
                'MarkerEdgeColor', [0.1, 0.1, 0.1], ...
                'MarkerSize', 10, ...
                'Parent',D.UI.axZoneH(1));
            
            % Rescale dist and set main axis
            dist_short = dist_short * (1/sum(dist_short));
            % set axis
            set(D.UI.axZoneH(2), ...
                'YLim' , [0, 1], ...
                'YTick',0:500/D.PAR.rewDurLim(2):1, ...
                'YTickLabel',0:500:D.PAR.rewDurLim(2), ...
                'XTickLabel' , [], ...
                'XGrid', 'on', ...
                'YGrid', 'on', ...
                'YMinorGrid', 'on');
            % make lables
            x_tic_labs = arrayfun(@(x,y,z) (sprintf('%d%c \n%d_{ms} \n(%0.0f%%)', x, char(176), y, z)), ...
                zone_short, [0,D.PAR.zoneRewDur,0], dist_short*100, 'uni', false);
            for z_tick = 2:length(x_tic_labs)-1
                text(D.UI.axZoneH(2).XTick(z_tick), -0.15, x_tic_labs{z_tick}, ...
                    'FontSize', 7, ...
                    'FontWeight', 'bold', ...
                    'HorizontalAlignment', 'center');
            end
            
            % Remove unused vals
            dist_cut = find(dist_short ~= 0, 1, 'last');
            zone_interp = zone_short(1:dist_cut);
            dist_interp = dist_short(1:dist_cut);
            
            % Compute cumsum and interpolate from random data
            cum = cumsum(dist_interp);
            cum = cum/max(cum);
            
            % get random values based on distrebution
            bins = linspace(min(zone_interp)-5,max(zone_interp),length(zone_interp)+1);
            % run initial rand
            rand_dist = round(interp1(cum, zone_interp, sort(rand(1,100))));
            samp_hist = histc(rand_dist, bins);
            samp_hist = samp_hist(1:end-1);
            
            % Get final zone dist
            zone_arr = cell2mat(arrayfun(@(x) (repmat(zone_interp(x),samp_hist(x),1)), ...
                (1:dist_cut)', 'Uni', false));
            % shuffle
            zone_arr = zone_arr(randperm(length(zone_arr)));
            
            % remove unused values
            zone_arr(zone_arr < -20 | zone_arr > 20) = 0;
            
            % Plot saved sample dist
            bins = ...
                linspace(min(-1*D.PAR.zoneLocs)-2.5, max(-1*D.PAR.zoneLocs)+2.5, length(D.PAR.zoneLocs)+1);
            samp_hist = histc(zone_arr, bins);
            samp_hist(end) = [];
            % rescale for plotting
            samp_dist = samp_hist'/max(samp_hist);
            
            % Reverse sign and store values
            D.I.zoneArr = zone_arr*-1;
            
            % plot sample dist
            if D.PAR.cueFeed ~= 'None'
                D.UI.zoneArrH = createPatches(...
                    -1*D.PAR.zoneLocs, samp_dist, 2, ...
                    [0, 0, 0], ...
                    0.25, ...
                    D.UI.axZoneH(2));
            end
            
            % Compute reward vlue by pos
            rew_by_pos = dist_short(2:end-1);
            rew_by_pos = (rew_by_pos-min(rew_by_pos)) / ...
                (max(rew_by_pos)-min(rew_by_pos));
            rew_by_pos = ...
                rew_by_pos * ...
                diff(D.PAR.rewDurLim) + D.PAR.rewDurLim(1);
            % must be convertable to byte
            rew_by_pos = floor(rew_by_pos/10)*10; %#ok<NASGU>
            % Store values
            %D.PAR.zoneRewDur = rew_by_pos;
            
            % Keep count shown on top of ax 1
            set(D.UI.axZoneH(1), ...
                'XAxisLocation','top', ...
                'YTickLabel', [], ...
                'XTick', short_ind(2:end-1), ...
                'XTickLabel', D.C.zone(D.I.rot,:));
            
            % Move axes to top of stack
            uistack(D.UI.axZoneH, 'top');
            uistack(D.UI.axZoneH(1), 'top');
            
            %             % Print vals to console
            %             str = [];
            %             for z_s = 1:7:length(D.I.zoneArr)
            %                 if z_s+7 < length(D.I.zoneArr)
            %                     str = [str, sprintf('%s\r', num2str(D.I.zoneArr(z_s:z_s+7)'))];
            %                 else
            %                     str = [str, sprintf('%s\r', num2str(D.I.zoneArr(z_s:end)'))];
            %                 end
            %             end
            %
            %             Console_Write(sprintf('[Track_Task_Setup] Computed Track Task: \r%s', ...
            %                 str,));
            
            %% SETUP UI OBJECTS
            
            % Plot reward bounds
            for z_fd = 1:2
                for z_zone = 1:length(D.PAR.zoneLocs)
                    
                    % reward bounds
                    [xbnd, ybnd] =  ...
                        Get_Rad_Bnds(D.PAR.rewZoneBnds(z_zone,:,z_fd));
                    D.UI.ptchFdZineH(z_zone,z_fd) = ...
                        patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                        [ybnd(1,:),fliplr(ybnd(2,:))], ...
                        D.UI.rotCol(z_fd,:), ...
                        'EdgeColor', [0, 0, 0], ...
                        'EdgeAlpha', 0.025, ...
                        'FaceAlpha', 0.05, ...
                        'LineWidth', 1, ...
                        'Visible', 'off', ...
                        'Parent', D.UI.axH(2));
                    
                    % Add reward duration text
                    str = ...
                        sprintf('%d%c\n%d ms', -1*D.PAR.zoneLocs(z_zone), char(176), D.PAR.zoneRewDur(z_zone));
                    D.UI.txtFdDurH(z_zone,z_fd) = text(...
                        mean(mean(xbnd)), mean(mean(ybnd)), ...
                        str, ...
                        'Color', [1, 1, 1], ...
                        'HorizontalAlignment', 'center', ...
                        'FontSize', 8, ...
                        'FontWeight', 'bold', ...
                        'Visible', 'off', ...
                        'Parent', D.UI.axH(2));
                end
            end
            % bring text to top of stack
            uistack(reshape(D.UI.txtFdDurH,1,[]),'top');
            % make 0 bounds visible
            set(D.UI.ptchFdZineH(:,1), 'EdgeAlpha', 0.05, 'FaceAlpha', 0.15);
            
            % Plot reward reset bounds
            for z_fd = 1:2
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds(D.UI.rewRstBnds(z_fd,:));
                D.UI.ptchFdRstH(z_fd) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    D.UI.rotCol(z_fd,:), ...
                    'FaceAlpha', 0.5, ...
                    'EdgeAlpha', 0.5, ...
                    'Visible', 'off', ...
                    'Parent', D.UI.axH(2));
            end
            
            % Plot lap bounds
            % set alpha to 0 (transparent)
            for z_quad = 1:4
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds(D.PAR.lapBnds(z_quad,:));
                D.UI.ptchLapBnds(z_quad) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    [0.5,0.5,0.5], ...
                    'FaceAlpha', 0.1, ...
                    'EdgeAlpha', 0, ...
                    'Visible', 'off', ...
                    'Parent', D.UI.axH(2));
            end
            
            % Feeder reward feeder marker
            for z_fd = 1:2
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds([D.UI.rewZoneRad(z_fd), ...
                    D.UI.rewZoneRad(z_fd) + deg2rad(D.PAR.trigDist)]);
                % setpoint zone line
                D.UI.mixFdNow(1,z_fd) = ...
                    plot(xbnd(:,end), ybnd(:,end), ...
                    'Color', D.UI.rotCol(z_fd,:), ...
                    'LineWidth', 2, ...
                    'Visible', 'off', ...
                    'Parent',D.UI.axH(2));
                % distance line
                D.UI.mixFdNow(2,z_fd) = ...
                    plot(xbnd(2,:), ybnd(2,:), ...
                    'Color', D.UI.rotCol(z_fd,:), ...
                    'LineWidth', 3, ...
                    'Visible', 'off', ...
                    'Parent',D.UI.axH(2));
                % marker
                D.UI.mixFdNow(3,z_fd) = ...
                    plot(D.UI.fd_x(D.UI.rewFeed(z_fd)), ...
                    D.UI.fd_y(D.UI.rewFeed(z_fd)), 'o', ...
                    'MarkerFaceColor', D.UI.rotCol(z_fd,:), ...
                    'MarkerEdgeColor', [0.1,0.1,0.1], ...
                    'MarkerSize', 20, ...
                    'Visible', 'off', ...
                    'Parent',D.UI.axH(2));
            end
            
            % Plot opposite unrewarded feeders darker
            plot(D.UI.fd_x(D.UI.oppFeed), D.UI.fd_y(D.UI.oppFeed), 'o', ...
                'MarkerFaceColor', [0.25 0.25 0.25], ...
                'MarkerEdgeColor', [0.1,0.1,0.1], ...
                'MarkerSize', 20, ...
                'Parent',D.UI.axH(2));
            
            %% MAKE UI OBJECTS VISIBLE
            
            % Set other plot features to visible
            set(D.UI.linTrckH, 'Visible', 'on');
            set(D.UI.linVelH, 'Visible', 'on');
            set(D.UI.ptchFdZineH, 'Visible', 'on');
            set(D.UI.ptchStQ, 'Visible', 'on');
            set(D.UI.txtStQ, 'Visible', 'on');
            
            % Enlarge 0 deg marker
            set(D.UI.mixFdNow(3,1), 'MarkerSize', 25);
            set(D.UI.mixFdNow, 'Visible', 'on');
            uistack(reshape(D.UI.mixFdNow(1:2,:),1,[]),'bottom',1);
            
            % Make sure reward reset is active
            if D.PAR.sesCond ~= 'Manual_Training' %#ok<*STCMP>
                % Set reset patch to visible
                set(D.UI.ptchFdRstH(D.I.rot), 'Visible', 'on');
            end
            
            % Set reward zone axes
            set(D.UI.axZoneH, 'Visible', 'on');
            
        end
        
        % --------------------------FORAGE TASK SETUP----------------------
        
        function[] = Forage_Task_Setup()
            
            % Set axis lims
            set(D.UI.axH(3), ...
                'YDir', 'reverse', ...
                'XLim', [0,D.PAR.rfBins], ...
                'YLim', [0,D.PAR.rfBins]);
            
            % Load file if exists
            if exist(D.DIR.ioRFPath, 'file')
                load(D.DIR.ioRFPath);
                
                % Store in struct
                D.P.pathMat = double(path_mat); %#ok<NODEF>
                D.DB.simRatPathMat = double(sim_rat_path_mat); %#ok<NODEF>
                D.PAR.pathLengthArr = path_length_arr;  %#ok<NODEF>
                clear path_mat sim_rat_path_mat path_length_arr;
            else
                
                % Deg bin var
                n_targs = 360/D.PAR.pathDegDist;
                
                % Number of bins in rf area
                path_bins = round(D.PAR.rfBins*(D.UI.rfRad / D.UI.arnRad));
                
                % Width of path
                path_width = ((2*D.UI.rfRad*pi) * (D.PAR.pathTargWdt/360)) * ...
                    (D.PAR.rfBins / (D.UI.arnRad*2));
                % Make sure width is odd
                if (mod(floor(path_width),2) == 1)
                    path_width = floor(path_width);
                else
                    path_width = ceil(path_width);
                end
                
                % Setup path mat
                D.P.pathMat = zeros(D.PAR.rfBins,D.PAR.rfBins,D.PAR.nPaths,n_targs);
                
                % Setup temp path mat
                mat_eye = eye(path_bins*2 + path_width);
                mat_p = zeros(size(mat_eye));
                ind = floor(path_width/2)*-1 : floor(path_width/2);
                for i = 1:path_width
                    if ind(i) < 0
                        mat_p = mat_p + padarray(mat_eye(abs(ind(i))+1:end,:),[abs(ind(i)),0],'post');
                    elseif ind(i) > 0
                        mat_p = mat_p + padarray(mat_eye(1:end-abs(ind(i)),:),[abs(ind(i)),0],'pre');
                    else
                        mat_p = mat_p + mat_eye;
                    end
                end
                
                % Setup temp sim rat mat
                mat_r = mat_p + padarray(mat_eye(1+1:end,:),[1,0],'post');
                
                % Compute mat for each pos condition
                path_ang_arr = linspace(0,90,90/D.PAR.pathDegDist+1);
                for c = 1:2
                    if c==1
                        mat_now = mat_p;
                    else
                        mat_now = mat_r;
                    end
                    
                    for i = 1:D.PAR.nPaths
                        
                        deg = path_ang_arr(i);
                        % Rotate
                        mat_rot = imrotate(mat_now,deg,'bilinear','crop');
                        % Cut and pad
                        rind = floor(size(mat_eye,1)/2)-floor(path_bins/2):floor(size(mat_eye,1)/2)+floor(path_bins/2);
                        cind = floor(size(mat_eye,2)/2):floor(size(mat_eye,1)/2)+path_bins-1;
                        mat_rot = mat_rot(rind,cind);
                        % Flip
                        mat_rot = flip(mat_rot,2);
                        mat_rot = flip(mat_rot,1);
                        
                        % Pad to full size
                        pad_lng = round((D.PAR.rfBins - path_bins)/2);
                        mat_rot = padarray(mat_rot, [pad_lng,pad_lng]);
                        
                        % Store
                        for j = [1:n_targs]-1
                            if c==1
                                D.P.pathMat(:,:,i,j+1) = imrotate(mat_rot,j*D.PAR.pathDegDist,'bilinear','crop');
                            else
                                D.DB.simRatPathMat(:,:,i,j+1) = imrotate(mat_rot,j*D.PAR.pathDegDist,'bilinear','crop');
                            end
                        end
                    end
                end
                
                % Mask values outside circle
                mask = repmat(D.PAR.pathMask,[1,1,D.PAR.nPaths,n_targs]);
                D.P.pathMat = D.P.pathMat.*mask;
                D.DB.simRatPathMat = D.DB.simRatPathMat.*mask;
                
                % Nomalize
                D.P.pathMat(D.P.pathMat>0) = 1;
                %D.P.pathMat = D.P.pathMat ./ repmat(sum(sum(D.P.pathMat,1),2),[D.PAR.rfBins,D.PAR.rfBins,1,1]);
                D.DB.simRatPathMat(D.DB.simRatPathMat>0) = 1;
                
                % Plot path averages
                ih = imagesc(sum(D.P.pathMat(:,:,:,1),3), 'Parent', D.UI.axH(3));
                pause(1);
                delete(ih);
                
                % Plot accross pos and paths
                ih = imagesc(sum(D.P.pathMat(:,:,:,1),3), 'Parent', D.UI.axH(3));
                pause(1);
                delete(ih);
                for i = 1:D.PAR.nPaths
                    ih = imagesc(D.P.pathMat(:,:,i,1), 'Parent', D.UI.axH(3));
                    pause(0.1);
                    delete(ih);
                end
                
                % Compute path lengths
                ang_arr = linspace(-45,45,D.PAR.nPaths);
                [x_path,y_path] = pol2cart(deg2rad(ang_arr), ones(1,D.PAR.nPaths));
                [x_str, y_str] = pol2cart(deg2rad(180), 1);
                path_length_arr = sqrt((abs(x_path-x_str)).^2 + abs((y_path-y_str)).^2);
                path_length_arr = path_length_arr * (D.UI.rfRad/max(path_length_arr));
                D.PAR.pathLengthArr = path_length_arr;
                
                % Save path info
                path_mat = single(D.P.pathMat); %#ok<NASGU>
                sim_rat_path_mat = single(D.DB.simRatPathMat); %#ok<NASGU>
                save(D.DIR.ioRFPath, 'path_mat', 'sim_rat_path_mat', 'path_length_arr');
                clear path_mat sim_rat_path_mat path_length_arr;
                
            end
            
            % Create target patches
            for z_targ = 1:length(D.PAR.pathTargArr)
                
                % reward bounds
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds(D.PAR.rewTargBnds(z_targ,:));
                D.UI.ptchRFTarg(z_targ) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    D.UI.activeCol, ...
                    'EdgeColor', [0, 0, 0], ...
                    'EdgeAlpha', 0.025, ...
                    'FaceAlpha', 0.5, ...
                    'LineWidth', 1, ...
                    'Visible', 'off', ...
                    'Parent', D.UI.axH(2));
            end
            
            % Set color lims
            set(D.UI.axH(3), 'CLim', [0,1]);
            
            % Plot occ
            D.UI.imgRFOCC = imagesc(D.P.occMat+D.P.pathNowMat, ...
                'Parent', D.UI.axH(3));
            
            % Set other plot features to visible
            set(D.UI.linRFH, 'Visible', 'on');
            set(D.UI.imgMaskRFH, 'Visible', 'on');
            set(D.UI.ptchStQ, 'Visible', 'on');
            set(D.UI.txtStQ, 'Visible', 'on');
            
            % Get first target set to 180 from start quad
            D.I.targInd = ...
                find(rad2deg(Rad_Sum(mean(D.PAR.strQuadBnds), pi)) == D.PAR.pathTargArr);
            set(D.UI.ptchRFTarg(D.I.targInd), 'Visible', 'on');
              
        end
        
        % ----------------------------RAT IN CHECK-------------------------
        
        function [] = Rat_In_Check()
            
            % Signal ready for rat
            if strcmp(get(D.UI.txtStQ,'String'), 'Start')
                set(D.UI.txtStQ, ...
                    'String','READY', ...
                    'Color',  D.UI.activeCol);
            end
            
            % Bail if no new data
            if all(isnan(D.P.Rat.rad))
                return
            end
            
            % Keep checking if rat is in the arena
            check_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.PAR.strQuadBnds);
            if ~any(check_inbound)
                % Reinitialize
                D.T.strqd_inbnd_t1 = 0;
                return
            end
            
            % Get inbound ts
            if D.T.strqd_inbnd_t1 == 0
                D.T.strqd_inbnd_t1 = D.P.Rat.ts(find(check_inbound, 1, 'first'));
            else
                D.T.strqd_inbnd_t2 = D.P.Rat.ts(find(check_inbound, 1, 'last'));
            end
            
            % Compute time in seconds
            inbndTim = (D.T.strqd_inbnd_t2 - D.T.strqd_inbnd_t1) / 10^6;
            
            % Check if rat has been in for the min delay period
            if inbndTim < D.PAR.strQdDel
                return
            end
            
            % Tell CS rat is in
            SendM2C('I', 1);
            
            % Post NLX event: rat in
            NlxSendCommand(D.NLX.rat_in_evt);
            
            % Change bool so code will only be run once
            D.F.rat_in = true;
            
            % Save time
            D.T.run_str = Elapsed_Seconds(now);
            
            % Start tracking lap times
            D.T.lap_tim = Elapsed_Seconds(now);
            
            % Set patch to nonvisible
            set(D.UI.ptchStQ, ...
                'FaceAlpha', 0, ...
                'EdgeAlpha', 0);
            
            % Reset start string
            set(D.UI.txtStQ, ...
                'String','Start', ...
                'Color',  D.UI.enabledCol);
            
            % Clear VT data
            BtnClrVT();
            
            % Reinitialize
            D.T.strqd_inbnd_t1 = 0;
            
            % Set lap patches to visible
            if D.PAR.sesTask == 'Track'
                set(D.UI.ptchLapBnds, 'Visible', 'on');
            end
            
            % Log/print
            Console_Write(sprintf('[Rat_In_Check] FINISHED: Rat In Check: OCC=%0.2fsec', ...
                inbndTim));
            
        end
        
        % -----------------------------TEST SETUP--------------------------
        
        function[] = Run_Test_Setup()
            
            %% BAIL IF NOT TEST RUN
            if ~D.DB.isTestRun
                return
            end
            
            %% SET LAST SAVED ENTRIES TO WHAT WE WANT
            
            % Load rat 0000
            
            % Change data table entries which will be loaded later
            ratInd = ...
                find(ismember(D.SS_In_All.Properties.RowNames, D.DB.ratLab));
            
            % Set condition
            D.SS_In_All.Session_Condition(ratInd) = D.DB.Session_Condition;
            % Set task
            D.SS_In_All.Session_Task(ratInd) = D.DB.Session_Task;
            % Set reward delay
            D.SS_In_All.Reward_Delay(ratInd)= D.DB.Reward_Delay;
            % Set cue condition
            D.SS_In_All.Cue_Condition(ratInd) = D.DB.Cue_Condition;
            % Set sound conditions
            D.SS_In_All.Sound_Conditions(ratInd,1:2) = D.DB.Sound_Conditions;
            
            % Get session number
            var_ind = ...
                ismember(D.SS_In_All.Properties.VariableNames, ...
                ['Session_',char(D.DB.Session_Condition)]);
            if strcmp(D.DB.Session_Condition, 'Rotation')
                col_ind = 1;
            else
                col_ind = ismember([{'Track'},{'Forage'}], D.DB.Session_Task);
            end
            ses_next = ...
                D.SS_In_All{ratInd, var_ind}(col_ind) + 1;
            % Get session total
            ses_all_next = D.SS_In_All{ratInd, 'Session_Manual_Training'}(col_ind) + ...;
                D.SS_In_All{ratInd, 'Session_Behavior_Training'}(col_ind) + ...
                D.SS_In_All{ratInd, 'Session_Implant_Training'}(col_ind) + ...
                D.SS_In_All{ratInd, 'Session_Rotation'} + 1;
            
            % Set start quad
            D.SS_In_All.Start_Quadrant{ratInd}(ses_all_next) = D.DB.Start_Quadrant;
            % Set rot dir
            D.SS_In_All.Rotation_Direction{ratInd}(ses_next) = D.DB.Rotation_Direction;
            % Set rot pos
            for i = 1:length(D.SS_In_All.Rotation_Positions{ratInd}(ses_next,:))
                D.SS_In_All.Rotation_Positions{ratInd}(ses_next,i) = num2str(D.DB.Rotation_Positions(i));
            end
            
            % Run PopRat
            val = find(cell2mat(cellfun(@(x) strcmp(x(1:4), D.DB.ratLab(2:end)), D.UI.ratList, 'uni', false)));
            set(D.UI.popRat, 'Value', val);
            PopRat();
            
            %% SETUP SPECIFIC TEST
            
            % PID CALIBRATION
            if D.DB.doPidCalibrationTest
                % Set flag to start pid
                D.F.pidStarted = false;
            end
            
            % HALT ERROR TEST
            if D.DB.doHaltErrorTest
                D.DB.haltCnt = D.DB.stepSamp;
                D.DB.haltDur = 5; % (sec)
                D.DB.halt_error_str = '';
                D.DB.t_halt = Elapsed_Seconds(now);
                D.DB.nowStep = 0;
                D.DB.nowVel = 0;
                D.DB.isHalted = true;
                D.DB.sendPos = 0;
                
                % Enable editing in console
                set(D.UI.listConsole, 'Enable','on');
            end
            
            % SIMULATED RAT TEST
            if D.DB.doSimRatTest
                D.DB.simXY = [NaN,NaN];
                D.DB.simRadLast = NaN;
                D.DB.simRohLast = NaN;
                D.DB.simRunRad = NaN;
                D.DB.simTargAng = NaN;
                D.DB.simVelLast = NaN;
                D.DB.simTSStart = NaN;
                D.DB.simTSLast = NaN;
                D.F.initVals = true;
                
                pos = [D.UI.sesInfPanPos(1)-0.18, 0.01, 0.175,0.02];
                D.UI.sldSimVel = uicontrol('Style', 'slider',...
                    'Parent',FigH, ...
                    'Units', 'Normalized', ...
                    'Min',0,'Max',100,'Value',D.DB.ratVelStart,...
                    'SliderStep', [0.01,0.1], ...
                    'Visible', 'off',...
                    'Enable', 'off',...
                    'Position', pos);
                % Vel text
                pos = [pos(1)-0.03, pos(2), 0.03, pos(4)];
                D.UI.txtSimVel = uicontrol('Style', 'text',...
                    'Parent',FigH, ...
                    'Units', 'Normalized', ...
                    'BackgroundColor', D.UI.figBckCol, ...
                    'ForegroundColor', D.UI.enabledCol, ...
                    'FontSize', 12, ...
                    'FontWeight', 'Bold', ...
                    'String','100',...
                    'Visible', 'off',...
                    'Position', pos);
            end
            
            % TRIGGER SETUP DONE
            set(D.UI.btnSetupDone, 'Value', 1);
            BtnSetupDone();
            
            % Log/print
            Console_Write('[Run_Test_Setup] FINISHED: Test Run Setup');
            
        end
        
        
        
        
        
        
        %% ======================== ONGOING FUNCTIONS =====================
        
        % ---------------------------UPDATE UI-----------------------------
        
        function [] = Update_UI(dt_min)
            
            % Default to 50 ms min delay
            if nargin < 1
                dt_min = 100;
            end
            
            % Convert to seconds
            dt_min = dt_min/1000;
            
            % Check if UI should be updated
            t_now = Elapsed_Seconds(now);
            if t_now-D.T.ui_update >= dt_min
                drawnow;
                D.T.ui_update = t_now;
                
                % Track redraw time
                D.DB.draw(1) = (Elapsed_Seconds(now) - t_now) * 1000;
                D.DB.draw(2) = min(D.DB.draw(1), D.DB.draw(2));
                D.DB.draw(3) = max(D.DB.draw(1), D.DB.draw(3));
                D.DB.draw(4) = D.DB.draw(1) + D.DB.draw(4);
                D.DB.draw(5) = D.DB.draw(5) + 1;
            end
            
        end
        
        
        % ---------------------------GET NLX VT----------------------------
        
        function [] = VT_Get(fld)
            
            % Bail if not connected
            if ~D.NLX.connected
                return
            end
            
            % Get NXL vt data and reformat data with samples in column vectors
            if strcmp(fld, 'Rat')
                % Get rat vt data
                [~, D.P.(fld).vtTS, D.P.(fld).vtPos, D.P.(fld).vtHD, D.P.(fld).vtNRecs, ~] = ...
                    NlxGetNewVTData(D.NLX.vt_rat_ent);
                D.P.(fld).hd_deg = D.P.(fld).vtHD';
            else
                % Get robot vt data
                [~, D.P.(fld).vtTS, D.P.(fld).vtPos, ~, D.P.(fld).vtNRecs, ~] = ...
                    NlxGetNewVTData(D.NLX.vt_rob_ent);
            end
            
            % Add to count
            D.P.(fld).cnt_vtRec = D.P.(fld).cnt_vtRec + D.P.(fld).vtNRecs;
            
        end
        
        % -------------------------PROCESS NLX VT--------------------------
        
        function [] = VT_Proc(fld)
            
            if D.P.(fld).vtNRecs > 0
                
                %% PROCESS NEW DATA
                
                % convdert to [r = samp, c = dim]
                xy_pos = reshape(double(D.P.(fld).vtPos),2,[])';
                ts = double(D.P.(fld).vtTS)';
                recs = D.P.(fld).vtNRecs;
                
                % Convert to normalized polar vals
                [rad, roh] = VT_2_Rad(xy_pos);
                
                % Exclude outlyer values > || < track bounds plus 5 cm
                if ~(D.PAR.sesTask == 'Forage' && strcmp(fld, 'Rat'))
                    exc_1 = roh > D.P.trackRohBnd(2) | roh < D.P.trackRohBnd(1);
                else
                    exc_1 = roh > D.P.rfRohBnd(2) | roh < 0;
                end
                
                % Exclude values based on current sample diff
                rad_diff = abs(diff([D.P.(fld).radLast; rad]));
                exc_2 = ...
                    rad_diff > (1/8)*0.9*(2 * pi) &...
                    rad_diff < 0.9*(2 * pi);
                
                % Exclude values based last used sample diff
                rad_diff_last = abs(rad - D.P.(fld).radLast);
                exc_3 = ...
                    rad_diff_last > (1/8)*0.9*(2 * pi) &...
                    rad_diff_last < 0.9*(2 * pi);
                
                % Do not use exc_3 if "Forage" run or more than reward duration of unused data
                if  Elapsed_Seconds(now) - D.T.(fld).last_pos_update > (D.PAR.rewDur+100)/1000
                    exc_3 = zeros(size(exc_3,1), 1);
                end
                
                % Do not use exc_2/3 if "Forage" run
                if  (D.PAR.sesTask == 'Forage' && strcmp(fld, 'Rat'))
                    exc_2 = zeros(size(exc_2,1), 1);
                    exc_3 = zeros(size(exc_3,1), 1);
                end
                
                % Check that reward flag has not been set for too long
                if D.T.rew_start > 0
                    if Elapsed_Seconds(now) - D.T.rew_start > 5 && ...
                            D.F.is_rewarding == true
                        % reset flag
                        D.F.is_rewarding = false;
                    end
                end
                
                % Combine exclusion criteria
                exc_ind = exc_1 | exc_2 | exc_3;
                
                % Set bad recs to empty
                rad(exc_ind) = [];
                roh(exc_ind) = [];
                ts(exc_ind) = [];
                
                % Recalculate cartisian values
                [x, y] = pol2cart(rad,roh);
                x =  x.*D.PAR.R + D.PAR.XC;
                y =  y.*D.PAR.R + D.PAR.YC;
                
                % Check if any data kept
                if sum(exc_ind) == recs
                    
                    % Do not plot any data this loop
                    D.F.(fld).plot_pos = false;
                    D.F.(fld).plot_vel = false;
                    
                    % Set vars to NaN
                    D.P.(fld).x = NaN;
                    D.P.(fld).y = NaN;
                    D.P.(fld).rad = NaN;
                    D.P.(fld).roh = NaN;
                    D.P.(fld).ts = NaN;
                    D.P.(fld).recs = NaN;
                    D.P.(fld).vel_pol_arr(end,:) = NaN;
                    
                    % Exit function
                    return
                    
                else
                    % Plot pos data this loop
                    D.F.(fld).plot_pos = true;
                    
                    % Keep track of updates
                    D.T.(fld).last_pos_update = Elapsed_Seconds(now);
                    
                    % Save last usable rad value
                    D.P.(fld).radLast = rad(end);
                end
                
                % Store vars for later use
                D.P.(fld).x = x;
                D.P.(fld).y = y;
                D.P.(fld).rad = rad;
                D.P.(fld).roh = roh;
                D.P.(fld).ts = ts;
                D.P.(fld).recs = recs;
                
                % Save history for rat pos
                if strcmp(fld, 'Rat')
                    ind(1) = find(isnan(D.P.Rat.pos_hist(:,1)), 1, 'first');
                    ind(2) = ind(1)+length(x)-1;
                    D.P.Rat.pos_hist(ind(1):ind(2), 1) = x;
                    D.P.Rat.pos_hist(ind(1):ind(2), 2) = y;
                end
                
                %% HANDLE RAT FORAGE DATA
                
                % Bail if processing rat forrage data
                if D.PAR.sesTask == 'Forage' && strcmp(fld, 'Rat')
                    D.PAR.rfBinEdgeX = linspace(D.UI.lowLeft(1), D.UI.lowLeft(1)+D.UI.vtRes, D.PAR.rfBins+1);
                    D.PAR.rfBinEdgeY = linspace(D.UI.lowLeft(2), D.UI.lowLeft(2)+D.UI.vtRes, D.PAR.rfBins+1);
                    
                    N = histcounts2(D.P.(fld).y,D.P.(fld).x,D.PAR.rfBinEdgeY,D.PAR.rfBinEdgeX);
                    D.P.occMatRaw = D.P.occMatRaw+flip(N,1);
                    non_zer_occ = D.P.occMatRaw(D.P.occMatRaw(:) > 0);
                    scale = prctile(non_zer_occ,99);
                    if scale == 0
                        scale = 1;
                    end
                    D.P.occMat = D.P.occMatRaw/scale;
                    
                    return
                end
                
                %% GET SETPOINT, FEEDER POS AND GUARD POS
                
                if strcmp(fld, 'Rob')
                    
                    % Get setpoint in radians
                    D.P.Rob.setRad = D.P.(fld).radLast - D.PAR.setPoint;
                    if D.P.Rob.setRad < 0
                        D.P.Rob.setRad = 2*pi + D.P.Rob.setRad;
                    end
                    
                    % Get feeder pos
                    D.P.Rob.feedRad = D.P.(fld).radLast - D.PAR.feedDist;
                    if D.P.Rob.feedRad < 0
                        D.P.Rob.feedRad = 2*pi + D.P.Rob.feedRad;
                    end
                    
                    % Get guard pos
                    D.P.Rob.guardRad = D.P.(fld).radLast - D.PAR.guardDist;
                    if D.P.Rob.guardRad < 0
                        D.P.Rob.guardRad = 2*pi + D.P.Rob.guardRad;
                    end
                    
                    % Get butt pos
                    D.P.Rob.buttRad = D.P.(fld).radLast + D.PAR.buttDist;
                    if D.P.Rob.buttRad > 2*pi
                        D.P.Rob.buttRad = D.P.Rob.buttRad - 2*pi;
                    end
                    
                end
                
                %% COMPUTE VELOCITY
                
                % Get radian value for vel plot
                set_vel_nan = false;
                
                % Check what rob vel plot should be aligned to
                if strcmp(fld, 'Rob') && ~D.F.rat_in
                    % Use setpoint pos
                    D.P.(fld).velRad = D.P.Rob.setRad;
                else
                    % Use rat pos and check for rad 'jumps'
                    if abs(angdiff(D.P.(fld).velRad,D.P.Rat.radLast)) > deg2rad(20)
                        set_vel_nan = true;
                    end
                    D.P.(fld).velRad = D.P.Rat.radLast;
                end
                
                % Update samples
                
                % Total usable samples
                nDat = sum(~exc_ind);
                
                % Keep only max samples
                if nDat >= D.P.vel_nsmp
                    nDat = D.P.vel_nsmp;
                    D.P.(fld).rad_samples = rad(1:nDat);
                    D.P.(fld).ts_samples = ts(1:nDat);
                    % Combine new and old vals
                else
                    % combine rad
                    old_dat = D.P.(fld).rad_samples;
                    D.P.(fld).rad_samples(end-nDat+1:end) = rad;
                    D.P.(fld).rad_samples(1:end-nDat) = ...
                        old_dat(nDat+1:end);
                    % combine ts
                    old_dat = D.P.(fld).ts_samples;
                    D.P.(fld).ts_samples(end-nDat+1:end) = ts;
                    D.P.(fld).ts_samples(1:end-nDat) = ...
                        old_dat(nDat+1:end);
                end
                
                % Pull out non-nan values
                rad_arr = D.P.(fld).rad_samples(~isnan(D.P.(fld).rad_samples));
                ts_arr = D.P.(fld).ts_samples(~isnan(D.P.(fld).ts_samples));
                
                % Compute sample velocities
                if length(rad_arr) < 1
                    vel = [];
                else
                    rad_diff = min(2*pi - abs(diff(rad_arr)), ...
                        abs(diff(rad_arr)));
                    dt = double(diff(ts_arr)) / 10^6;
                    cm = rad_diff * ((140 * pi)/(2 * pi));
                    vel = cm./dt;
                end
                
                % Check if we have usable data
                if isempty(vel) || isempty(D.P.(fld).velRad)
                    
                    % Set not to plot this vel
                    D.F.(fld).plot_vel = false;
                    
                    % Avoid jumps in plot
                    D.P.(fld).vel_pol_arr(end,:) = NaN;
                    
                else
                    
                    % Set to plot this vel
                    D.F.(fld).plot_vel = true;
                    
                    % Save average
                    D.P.(fld).vel = nanmean(vel);
                    
                    % Store vel max
                    if D.P.(fld).vel < 150
                        D.P.(fld).vel_max_lap = max(D.P.(fld).vel_max_lap, D.P.(fld).vel);
                        D.P.(fld).vel_max_all = max(D.P.(fld).vel_max_lap, D.P.(fld).vel_max_all);
                    end
                    
                    % Shift stored averages
                    D.P.(fld).vel_pol_arr = circshift(D.P.(fld).vel_pol_arr, -1, 1);
                    
                    % Cap to vel min,max
                    velRoh = D.P.(fld).vel;
                    % Set >max to max
                    if velRoh>D.P.velMax
                        velRoh = D.P.velMax;
                        % Set <min to min
                    elseif velRoh<D.P.velMin
                        velRoh = D.P.velMin;
                    end
                    % Adjust range
                    velRoh = velRoh/D.P.velMax;
                    velRoh = velRoh*(D.P.velRohMax-D.P.velRohMin) + D.P.velRohMin;
                    
                    % Cap to roh max
                    if velRoh > D.P.velRohMax
                        velRoh = D.P.velRohMax;
                    end
                    
                    % Store plot values
                    if ~set_vel_nan
                        % Convert to cart
                        [D.P.(fld).vel_pol_arr(end, 1), D.P.(fld).vel_pol_arr(end, 2)] = ...
                            pol2cart(D.P.(fld).velRad, velRoh);
                        D.P.(fld).vel_pol_arr(end, 1) =  D.P.(fld).vel_pol_arr(end, 1).*D.PAR.R + D.PAR.XC;
                        D.P.(fld).vel_pol_arr(end, 2) =  D.P.(fld).vel_pol_arr(end, 2).*D.PAR.R + D.PAR.YC;
                    else
                        % Set to NaN because rad diff to large
                        D.P.(fld).vel_pol_arr(end, :) = NaN;
                        D.P.(fld).velRad = NaN;
                        velRoh = NaN;
                    end
                    
                    % Save history
                    ind = find(isnan(D.P.(fld).vel_lap(:,1)), 1, 'first');
                    D.P.(fld).vel_lap(ind, 1) = D.P.(fld).velRad;
                    D.P.(fld).vel_lap(ind, 2) = velRoh;
                    
                end
                
                %% TRANSFORM HD
                
                % Run only for rat data
                if  D.F.plot_hd && strcmp(fld, 'Rat')
                    
                    % Exclude values
                    D.P.Rat.hd_deg(exc_ind) = NaN;
                    
                    % Flip HD angle
                    D.P.Rat.hd_deg = abs(single(D.P.Rat.hd_deg)-360);
                    
                    % Shift HD by 90 deg to align with track/matlab 0 deg
                    D.P.Rat.hd_deg = D.P.Rat.hd_deg + 90;
                    if any(D.P.Rat.hd_deg > 360)
                        D.P.Rat.hd_deg(D.P.Rat.hd_deg > 360)  = D.P.Rat.hd_deg(D.P.Rat.hd_deg > 360) - 360;
                    end
                    
                    % Convert HD to radians
                    D.P.Rat.hd_rad = deg2rad(D.P.Rat.hd_deg);
                    
                end
                
            end
            
        end
        
        % -------------------------GET NLX EVENTS--------------------------
        
        function [] = Evt_Get()
            
            % Bail if not connected
            if ~D.NLX.connected
                return
            end
            
            % Read in event data
            %[evtPass, D.E.evtTS, evtID, evtTTL, D.E.evtStr, D.E.evtNRecs, evtDropped]
            [~, D.E.evtTS, ~, ~ , D.E.evtStr, D.E.evtNRecs, ~] = ...
                NlxGetNewEventData('Events');
            
            % Add to count
            D.P.(fld).cnt_evtRec = D.P.(fld).cnt_evtRec + D.E.evtNRecs;
        end
        
        % ------------------------PROCESS NLX EVENTS-----------------------
        
        function [] = Evt_Proc()
            
            %% CHECK FOR NEW DATA
            Evt_Get();
            
            % Bail if no new data
            if D.E.evtNRecs == 0
                return
            end
            
            %% CHECK FOR REWARD
            
            % Reward Started
            if any(ismember(D.E.evtStr, D.NLX.rew_on_str))
                
                % Save reward start time
                D.T.rew_start = Elapsed_Seconds(now);
                
                % Get time stamp
                D.T.rew_nlx_ts(1) = D.E.evtTS(ismember(D.E.evtStr, D.NLX.rew_on_str));
                
                % Store round trip time
                if datenum(D.T.manual_rew_sent) > 0 && ...
                        datenum(D.T.manual_rew_sent) < datenum(D.T.rew_start)
                    % Save reward mesage round trip time
                    D.DB.rew_round_trip(1) = (D.T.rew_start - D.T.manual_rew_sent) * 1000;
                    D.DB.rew_round_trip(2) = min(D.DB.rew_round_trip(1), D.DB.rew_round_trip(2));
                    D.DB.rew_round_trip(3) = max(D.DB.rew_round_trip(1), D.DB.rew_round_trip(3));
                    D.DB.rew_round_trip(4) = D.DB.rew_round_trip(1) + D.DB.rew_round_trip(4);
                    D.DB.rew_round_trip(5) = D.DB.rew_round_trip(5) + 1;
                    % Reset time
                    D.T.manual_rew_sent = 0;
                end
                
                % Change feeder dish plot color and marker size
                D.UI.feedPosCol = D.UI.activeCol;
                D.UI.feedPosMarkSize = 30;
                if isfield(D.UI, 'feedPltNow')
                    set(D.UI.feedPltNow, ...
                        'MarkerFaceColor', D.UI.feedPosCol, ...
                        'MarkerSize', D.UI.feedPosMarkSize);
                end
                
                % Turn on reward button backround color
                set(D.UI.btnReward, ...
                    'BackgroundColor', D.UI.activeCol);
                
                % Add to reward count for manual session
                if D.PAR.sesCond == 'Manual_Training'
                    D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
                end
                
                % Set flags
                D.F.is_rewarding = true;
                
            end
            
            % Reward Ended
            if any(ismember(D.E.evtStr, D.NLX.rew_off_str))
                
                % Save reward end time
                D.T.rew_end = Elapsed_Seconds(now);
                
                % Get time stamp
                D.T.rew_nlx_ts(2) = D.E.evtTS(ismember(D.E.evtStr, D.NLX.rew_off_str));
                
                % Store reward duration
                dt = [0,0];
                % matlab time
                if datenum(D.T.rew_start) > 0 && ...
                        datenum(D.T.rew_start) < datenum(D.T.rew_end)
                    dt(1) = (D.T.rew_end - D.T.rew_start) * 1000;
                end
                % nlx ts time
                if D.T.rew_nlx_ts(1) < D.T.rew_nlx_ts(2) && ...
                        D.T.rew_nlx_ts(1) > 0
                    dt(2) = (D.T.rew_nlx_ts(2) - D.T.rew_nlx_ts(1))/1000;
                end
                if dt(2) > 0
                    % Save reward duration
                    D.DB.rew_duration(1) = dt(2);
                    D.DB.rew_duration(2) = min(D.DB.rew_duration(1), D.DB.rew_duration(2));
                    D.DB.rew_duration(3) = max(D.DB.rew_duration(1), D.DB.rew_duration(3));
                    D.DB.rew_duration(4) = D.DB.rew_duration(1) + D.DB.rew_duration(4);
                    D.DB.rew_duration(5) = D.DB.rew_duration(5) + 1;
                end
                
                % Save halt error
                if ~isempty(D.I.zone)
                    halt_err = D.PAR.rewZoneBnds(D.I.zone,2,D.I.rot) - (D.P.Rob.radLast-D.PAR.setPoint);
                    % Save reward duration
                    D.DB.halt_error(1) = halt_err * ((140*pi)/(2*pi));
                    D.DB.halt_error(2) = min(D.DB.halt_error(1), D.DB.halt_error(2));
                    D.DB.halt_error(3) = max(D.DB.halt_error(1), D.DB.halt_error(3));
                    D.DB.halt_error(4) = D.DB.halt_error(1) + D.DB.halt_error(4);
                    D.DB.halt_error(5) = D.DB.halt_error(5) + 1;
                end
                
                % Set flag
                D.F.is_rewarding = false;
                
                % Change feeder dish plot color and marker size
                D.UI.feedPosCol = D.UI.rotCol(D.I.rot,:);
                D.UI.feedPosMarkSize = 10;
                if isfield(D.UI, 'feedPltNow')
                    set(D.UI.feedPltNow, ...
                        'MarkerFaceColor', D.UI.feedPosCol, ...
                        'MarkerSize', D.UI.feedPosMarkSize);
                end
                
                % Turn off reward button backround color
                set(D.UI.btnReward, ...
                    'BackgroundColor', D.UI.enabledCol);
                
            end
            
            %% CHECK FOR PID
            
            % Running
            if any(ismember(D.E.evtStr, D.NLX.pid_run_str))
                % Change setpoint plot color and width
                D.UI.setPosCol = D.UI.activeCol;
                D.UI.setPosLineWidth = 4;
            end
            % Stopped
            if any(ismember(D.E.evtStr, D.NLX.pid_stop_str))
                % Change setpoint plot color
                D.UI.setPosCol = D.UI.robNowCol;
                D.UI.setPosLineWidth = 2;
            end
            
            %% CHECK FOR BULLDOZE
            
            % Running
            if any(ismember(D.E.evtStr, D.NLX.bull_run_str))
                % Change guard plot color and width
                D.UI.guardPosCol = D.UI.activeCol;
                D.UI.guardPosLineWidth = 10;
                
                % Track count
                D.C.bull_cnt = D.C.bull_cnt + 1;
            end
            % Stopped
            if any(ismember(D.E.evtStr, D.NLX.bull_stop_str))
                % Change guard plot color
                D.UI.guardPosCol = D.UI.robNowCol;
                D.UI.guardPosLineWidth = 5;
            end
            
        end
        
        % ----------------------ROTATION TRIGGER CHECK---------------------
        
        function [] = Rotation_Trig_Check()
            
            % Bail if not a 'Track' session
            if D.PAR.sesTask == 'Forage'
                return
            end
            
            % Bail if button not active
            if ~get(D.UI.btnICR, 'UserData')
                return
            end
            
            % Check if rat in rotation bounds
            check_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.UI.rotBndNext);
            if ~any(check_inbound)
                return
            end
            
            % Set flag
            D.F.rotated = true;
            
            % Save current image
            rotLast = D.I.rot;
            
            % Get new image
            D.I.rot = find([1, 2] ~=  D.I.rot);
            
            % Update D.AC.data(2) and send command to rotate image
            D.AC.data(2) = D.I.img_ind(D.I.rot);
            SendM2AC();
            
            % Post NLX event: rotaion *deg
            NlxSendCommand(D.NLX.rot_evt{D.I.rot});
            
            % Change plot marker size
            % active feeder
            set(D.UI.mixFdNow(3, D.I.rot), ...
                'MarkerSize', 25);
            % inactive feeder
            set(D.UI.mixFdNow(3, [1, 2] ~=  D.I.rot), ...
                'MarkerSize', 20)
            
            % Delete old patches
            if isfield(D.UI, 'ptchRtTrgH')
                delete(D.UI.ptchRtTrgH)
            end
            
            % Plot rotation pos mark
            r = D.P.Rat.rad(check_inbound);
            [x,y] = Get_Rad_Bnds(r(1));
            plot(x,y, ...
                'Color', D.UI.rotCol(D.I.rot,:), ...
                'LineWidth', 3, ...
                'Parent',D.UI.axH(2));
            
            % Reset reward bounds patch feeder
            set(D.UI.ptchFdZineH, ...
                'EdgeColor', [0, 0, 0], ...
                'FaceAlpha', 0.05, ...
                'EdgeAlpha',0.025)
            % active feeder
            set(D.UI.ptchFdZineH(:, D.I.rot), ...
                'FaceAlpha', 0.15, ...
                'EdgeAlpha',0.025)
            
            % Change button features
            set(D.UI.btnICR,'string', D.UI.btnICRstr{rotLast}, ...
                'BackgroundColor', D.UI.rotCol(rotLast,:), ...
                'UserData', false);
            
            % Change wall image
            set(D.UI.wallImgH(D.I.img_ind(rotLast)), 'Visible', 'off');
            set(D.UI.wallImgH(D.I.img_ind(D.I.rot)), 'Visible', 'on');
            
            % Change reward reset
            % hide reward reset patch
            set(D.UI.ptchFdRstH([1, 2] ~=  D.I.rot), ...
                'FaceAlpha', 0.5, ...
                'Visible', 'off');
            % show new patch
            set(D.UI.ptchFdRstH(D.I.rot), ...
                'FaceAlpha', 0.5, ...
                'Visible', 'on');
            
            % Change session info font weight
            % active feeder
            set(D.UI.txtPerfInf(D.I.rot), 'FontWeight', 'Bold')
            
            % inactive feeder
            set(D.UI.txtPerfInf([1, 2] ~=  D.I.rot), 'FontWeight', 'Light')
            
            % Add to roation counter
            D.C.rot_cnt = D.C.rot_cnt + 1;
            
            % Add new lap counter after all rot conds run once
            if  D.C.rot_cnt > 2
                D.C.lap_cnt{D.I.rot} = [D.C.lap_cnt{D.I.rot}, 0];
                D.C.rew_cnt{D.I.rot} = [D.C.rew_cnt{D.I.rot}, 0];
            end
            
        end
        
        % ---------------------TRACK REWARD SEND CHECK---------------------
        
        function [] = Track_Reward_Send_Check()
            
            % BAIL IF MANUAL OR FORAGE TRAINING OR BOUNDS PASSED
            if D.PAR.sesCond == 'Manual_Training' || ...
                    D.PAR.sesTask == 'Forage' || ...
                    D.F.flag_rew_send_crossed || ...
                    all(isnan(D.P.Rat.rad))
                return
            end
            
            % Check if rat is in quad
            check_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.UI.rewRstBnds(D.I.rot,:));
            if ~any(check_inbound)
                return
            end
            
            % Print reset bounds crossed
            Console_Write('[Track_Reward_Send_Check] Crossed Reset Bounds');
            
            % Check if next reward is cued
            D.F.is_cued_rew = ...
                D.PAR.cueFeed == 'All' || ...
                (D.PAR.cueFeed == 'Half' && D.C.rew_cross_cnt == 0) || ...
                (D.PAR.cueFeed == 'Half' && ~D.F.is_cued_rew);
            
            % Check if cue should be forced
            if D.F.do_all_cue && D.F.is_cued_rew == false
                D.F.is_cued_rew = true;
            elseif D.F.do_block_cue && D.F.is_cued_rew == true
                D.F.is_cued_rew = false;
            end
            
            % Disable cue buttons
            Set_Cue_Buttons('Disable');
            
            % Get new reward data if rat triggered reward on last lap
            if D.F.is_cued_rew && ...
                    (D.F.flag_rew_confirmed || D.C.rew_cross_cnt == 0)
                
                % Get new reward zone
                D.I.zone = find(D.PAR.zoneLocs == ...
                    D.I.zoneArr(sum([D.C.rew_cnt{:}])+1));
                
                % Get new reward duration
                D.PAR.rewDur = D.PAR.zoneRewDur(D.I.zone);
                
                % Post NLX event: cue on
                NlxSendCommand(D.NLX.cue_on_evt);
            end
            
            % Reset patches
            set(D.UI.ptchFdZineH(:, D.I.rot), ...
                'EdgeColor', [0, 0, 0], ...
                'FaceAlpha', 0.15, ...
                'EdgeAlpha', 0.05);
            % Clear last duration
            set(D.UI.txtFdDurH(:, :), ...
                'Visible', 'off');
            
            % Check if this is cued reward
            if D.F.is_cued_rew
                
                % Will send CS command with pos and zone
                r_pos = D.UI.rewRatHead(D.I.rot);
                z_ind = D.I.zone;
                r_cond = 2;
                
                % Send free reward command
                SendM2C('R', r_pos, r_cond, z_ind);
                
                % Show new reward taget patch
                set(D.UI.ptchFdZineH(D.I.zone, D.I.rot), ...
                    'EdgeColor', D.UI.activeCol, ...
                    'FaceAlpha', 0.75, ...
                    'EdgeAlpha', 1);
                
                % Print new duration
                set(D.UI.txtFdDurH(D.I.zone, D.I.rot), ...
                    'Visible', 'on');
            else
                
                % Send reward center and no zone ind
                r_pos = D.UI.rewRatHead(D.I.rot);
                r_del = D.PAR.rewDel;
                r_cond = 3;
                
                % Send free reward command
                SendM2C('R', r_pos, r_cond, r_del);
                
                % Darken all zone patches
                set(D.UI.ptchFdZineH(:, D.I.rot), ...
                    'FaceAlpha', 0.75)
            end
            
            % Set flags
            D.F.flag_rew_send_crossed = true;
            D.F.check_rew_confirm = true;
            D.F.flag_rew_zone_crossed = false;
            D.F.flag_rew_confirmed = false;
            
            % Hide reset patch
            set(D.UI.ptchFdRstH(D.I.rot), ...
                'FaceAlpha', 0.05);
            
            % Stop bulldozer if active
            D.UI.bullLastVal = get(D.UI.btnBulldoze, 'Value');
            if D.UI.bullLastVal == 1
                set(D.UI.btnBulldoze, 'Value', 0);
                Bulldoze();
            end
            
        end
        
        % ----------------------TRACK REWARD ZONE CHECK--------------------
        
        function [] = Track_Reward_Zone_Check()
            
            %% BAIL IF MANUAL OR FORRAGE TRAINING OR REWARD BOUNDS PASSED OR REWARD
            
            if D.PAR.sesCond == 'Manual_Training' || ...
                    D.PAR.sesTask == 'Forage' || ...
                    D.F.flag_rew_zone_crossed || ...
                    ~D.F.flag_rew_send_crossed
                
                return
            end
            
            %% CHECK FOR REWARD CONFIRMATION
            
            % Check if reward has been reset
            if  D.F.check_rew_confirm && ...
                    c2m.('Z').dat1 ~= 0 && ...
                    ~D.F.flag_rew_confirmed
                
                % Get rewarded zone ind
                D.I.zone = c2m.('Z').dat1;
                
                % Post NLX event: cue off
                if D.F.is_cued_rew
                    NlxSendCommand(D.NLX.cue_off_evt);
                end
                
                % Post NLX event: reward info
                NlxSendCommand(...
                    sprintf(D.NLX.rew_evt, D.PAR.zoneLocs(D.I.zone), D.PAR.zoneRewDur(D.I.zone)));
                
                % Add to total reward count
                if D.F.rotated
                    D.C.rew_cnt{D.I.rot}(end) = D.C.rew_cnt{D.I.rot}(end) + 1;
                else
                    D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
                end
                
                % Store reward zone with range [-20,20]
                D.I.zoneHist(sum([D.C.rew_cnt{:}])) = -1*D.PAR.zoneLocs(D.I.zone);
                
                % Reset reward zone patches
                set(D.UI.ptchFdZineH(:, D.I.rot), ...
                    'EdgeColor', [0, 0, 0], ...
                    'FaceAlpha', 0.15, ...
                    'EdgeAlpha', 0.05);
                % Lighten rewarded zone
                set(D.UI.ptchFdZineH(D.I.zone, D.I.rot), ...
                    'FaceAlpha', 0.5, ...
                    'EdgeAlpha', 0.5);
                % Print new duration
                set(D.UI.txtFdDurH(D.I.zone, D.I.rot), ...
                    'Visible', 'on');
                
                % Update zone dist plot
                D.C.zone(D.I.rot,D.I.zone) = D.C.zone(D.I.rot,D.I.zone)+1;
                x = -1*D.PAR.zoneLocs;
                y = D.C.zone(D.I.rot,:) / sum(D.C.zone(D.I.rot,:));
                y = y/max(y);
                delete(D.UI.zoneAllH(:,D.I.rot));
                D.UI.zoneAllH(:,D.I.rot) = createPatches(...
                    x, y, 2, ...
                    D.UI.rotCol(D.I.rot,:), ...
                    0.25, ...
                    D.UI.axZoneH(2));
                delete(D.UI.zoneNowH);
                D.UI.zoneNowH = createPatches(...
                    x(D.I.zone), y(D.I.zone), 2, ...
                    D.UI.rotCol(D.I.rot,:), ...
                    0.75, ...
                    D.UI.axZoneH(2));
                
                % Display count
                set(D.UI.axZoneH(1), 'XTickLabel', D.C.zone(D.I.rot,:))
                
                % Reset missed rewards
                D.C.missed_rew_cnt(2) = sum(D.C.missed_rew_cnt);
                D.C.missed_rew_cnt(1) = 0;
                
                % Plot average zone pos
                delete(D.UI.zoneAvgH);
                avg_trig = D.PAR.zoneLocs*D.C.zone(D.I.rot,:)' / sum(D.C.zone(D.I.rot,:));
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds(D.UI.rewZoneRad(D.I.rot) + deg2rad(avg_trig + D.PAR.trigDist));
                D.UI.zoneAvgH = ...
                    plot(xbnd, ybnd, ...
                    'Color', D.UI.rotCol(D.I.rot,:), ...
                    'LineStyle', '-', ...
                    'LineWidth', 1, ...
                    'Parent',D.UI.axH(2));
                uistack(D.UI.zoneAvgH, 'bottom');
                
                % Set/reset flags
                c2m.('Z').dat1 = 0;
                D.F.flag_rew_confirmed = true;
                D.F.check_rew_confirm = false;
                
                Console_Write(sprintf('[Track_Reward_Zone_Check] Rewarded: Zone=%d Vel=%0.2fcm/sec', ...
                    D.I.zoneHist(sum([D.C.rew_cnt{:}])), D.P.Rat.vel));
                
            end
            
            %% CHECK IF ALL ZONES PASSED OR CONFIRMATION RECEIVED
            
            % Track reward crossing
            if ~D.F.flag_rew_zone_crossed
                
                % Check if rat has passed all zones or reward confirmed
                check_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.UI.rewPassBnds(D.I.rot,:));
                
                if ~(any(check_inbound) || D.F.flag_rew_confirmed)
                    return
                end
                
                % Set flags
                D.F.flag_rew_zone_crossed = true;
                D.F.flag_rew_send_crossed = false;
                D.F.check_rew_confirm = false;
                
                % Itterate count
                D.C.rew_cross_cnt = D.C.rew_cross_cnt+1;
                
                % Enable cue buttons
                Set_Cue_Buttons('Enable');
                
                % Restart bulldozer if was active
                if D.UI.bullLastVal == 1
                    set(D.UI.btnBulldoze, 'Value', 1);
                    Bulldoze();
                end
                
                % Check for missed rewards
                if ~D.F.flag_rew_confirmed
                    
                    % Add to missed reward count
                    D.C.missed_rew_cnt(1) = D.C.missed_rew_cnt(1)+1;
                    
                    % Cue next lap
                    if D.PAR.cueFeed == 'Half'
                        D.F.is_cued_rew = true;
                    end
                    
                    % Reset reward zone patches
                    set(D.UI.ptchFdZineH(:, D.I.rot), ...
                        'EdgeColor', [0, 0, 0], ...
                        'FaceAlpha', 0.15, ...
                        'EdgeAlpha', 0.05);
                    set(D.UI.txtFdDurH(:, D.I.rot), ...
                        'Visible', 'off');
                    
                    % Print missed reward
                    Console_Write(sprintf('[Track_Reward_Zone_Check] Detected Missed Reward: cross_cnt=%d miss_cnt=%d|%d', ...
                        D.C.rew_cross_cnt, D.C.missed_rew_cnt(1), D.C.missed_rew_cnt(2)));
                    
                end
                
                % Set reset patch to visible
                set(D.UI.ptchFdRstH(D.I.rot), ...
                    'FaceAlpha', 0.5);
                
                % Update reward info list
                rew_ellapsed = Elapsed_Seconds(now) - D.T.rew_last;
                D.T.rew_last = Elapsed_Seconds(now);
                D.UI.rewInfoList = [...
                    D.UI.rewInfoList; ...
                    {sprintf('%d: T:%0.2f Z:%d M:%d', ...
                    sum([D.C.rew_cnt{:}])+1, ...
                    rew_ellapsed, ...
                    D.PAR.zoneLocs(D.I.zone)*-1, ...
                    sum(D.C.missed_rew_cnt)) ...
                    }];
                rew_percent = ...
                    round(100-(sum(D.C.missed_rew_cnt)/D.C.rew_cross_cnt)*100);
                infstr = [...
                    sprintf('Reward Info (%d%%)', rew_percent); ...
                    D.UI.rewInfoList];
                set(D.UI.popRewInfo, 'String', infstr);
                
                % Print reset bounds crossed
                Console_Write('[Track_Reward_Zone_Check] Crossed Reward Bounds');
            end
            
        end
        
        % ----------------------FORAGE REWARD ZONE CHECK-------------------
        
        function [] = Forage_Reward_Zone_Check()
            
            % BAIL IF MANUAL OR TRACK TRAINING
            
            if D.PAR.sesCond == 'Manual_Training' || ...
                    D.PAR.sesTask == 'Track'
                return
            end
            
            % Bail if no new data
            if all(isnan(D.P.Rat.rad))
                return
            end
            
            % Keep checking if rat is in the arena
            check_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.PAR.rewTargBnds(D.I.targInd,:));
            if ~any(check_inbound)
                % Reinitialize
                D.T.rf_rew_inbnd_t1 = 0;
                return
            end
            
            % Get inbound ts
            if D.T.rf_rew_inbnd_t1 == 0
                D.T.rf_rew_inbnd_t1 = D.P.Rat.ts(find(check_inbound, 1, 'first'));
            else
                D.T.rf_rew_inbnd_t2 = D.P.Rat.ts(find(check_inbound, 1, 'last'));
            end
            
            % Compute time in seconds
            inbndTim = (D.T.rf_rew_inbnd_t2 - D.T.rf_rew_inbnd_t1) / 10^6;
            
            % Check if rat has been in for the min delay period
            if inbndTim < D.PAR.rfRewDel
                return
            end
            
            % Get path deg array
            path_arr = linspace(-45,45,45/D.PAR.pathDegDist*2 + 1);
            
            % Get optimal path
            % invert occ values
            inv_occ = abs(D.P.occMat-max(D.P.occMat(:)));
            % get sum of product of occ and possible paths
            occ_prod = ...
                squeeze(sum(sum(D.P.pathMat(:,:,:,D.I.targInd) .* ...
                repmat(inv_occ,[1,1,size(D.P.pathMat,3)]),1),2));
            % normalize
            occ_prod = occ_prod/sum(occ_prod);
            % weight by path length normalized to occ prod range
            occ_scale = abs(D.PAR.pathLengthArr - max(D.PAR.pathLengthArr));
            occ_scale = occ_scale/sum(occ_scale);
            %occ_scale = 1 + occ_scale'*((max(occ_prod)-min(occ_prod))/max(occ_scale));
            occ_scale = 1 + occ_scale';
            occ_prod = occ_prod.*occ_scale;
            % get optimal path
            path_ind = find(occ_prod == max(occ_prod));
            if length(path_ind) > 1
                path_ind = path_ind(ceil(rand(1,1)*length(path_ind)));
            end
            path_rad = wrapTo2Pi(deg2rad(path_arr(path_ind)*2));
            
            % Get new targ
            rad_last_targ = deg2rad(D.PAR.pathTargArr(D.I.targInd));
            rad_new_targ = Rad_Sum(Rad_Sum(rad_last_targ,pi), path_rad);
            targ_ind_last = D.I.targInd;
            targ_ind_new = ...
                find(round(rad2deg(rad_new_targ)) == D.PAR.pathTargArr);
            
            % Plot path
            D.P.pathNowMat = D.P.pathMat(:,:,path_ind,D.I.targInd);
            D.P.pathNowMat(D.P.pathNowMat>0) = 0.5;
            D.UI.imgRFOCC = imagesc(D.P.pathNowMat, ...
                'Parent', D.UI.axH(3));
            
            % Update patches
            set(D.UI.ptchRFTarg(targ_ind_last), 'Visible', 'off');
            set(D.UI.ptchRFTarg(targ_ind_new), 'Visible', 'on');
            
            % Reinitialize
            D.T.rf_rew_inbnd_t1 = 0;
            
            % Store new targ
            D.I.targInd = targ_ind_new;
            
            % Log/print
            Console_Write(sprintf('[Forage_Reward_Zone_Check] Rewarded: Targ_Last=%ddeg Targ_New=%ddeg', ...
                D.PAR.pathTargArr(targ_ind_last), D.PAR.pathTargArr(D.I.targInd)));
            
        end
        
        % ----------------------------LAP CHECK----------------------------
        
        function [] = Lap_Check()
            
            %% BAIL FOR NON-TRACK RUN
            
            % Bail if not a 'Track' session
            if D.PAR.sesTask == 'Forage'
                return
            end
            
            % CHECK BOUNDS AND/OR BAIL
            track_quad = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.PAR.lapBnds(D.I.lap_hunt_ind, :));
            
            % Bail if not in bounds
            if ~any(track_quad)
                return
            end
            
            %% CHECK FOR BOUNDS CROSSING
            
            % Set specific bound bool to true
            D.F.check_inbound_lap(D.I.lap_hunt_ind) = true;
            
            % Set later quads to false to prevent backwards runs
            D.F.check_inbound_lap(1:4 > D.I.lap_hunt_ind) = false;
            
            % Set patches
            set(D.UI.ptchLapBnds(D.F.check_inbound_lap), 'Visible', 'off');
            set(D.UI.ptchLapBnds(~D.F.check_inbound_lap), 'Visible', 'on');
            
            % Update current quadrant
            if any(D.F.check_inbound_lap)
                D.I.lap_hunt_ind = find(~D.F.check_inbound_lap, 1, 'first');
            else
                D.I.lap_hunt_ind = 1;
            end
            
            % Bail if all bounds not crossed
            if ~all(D.F.check_inbound_lap)
                return
            end
            
            % Update lap count
            if D.F.rotated
                D.C.lap_cnt{D.I.rot}(end) = D.C.lap_cnt{D.I.rot}(end) + 1;
            else
                D.C.lap_cnt{3}(end) = D.C.lap_cnt{3}(end) + 1;
            end
            
            % Reset lap track bool
            D.F.check_inbound_lap = false(1,4);
            
            % Reset lap quad index
            D.I.lap_hunt_ind = 1;
            
            % Set all back to dark
            set(D.UI.ptchLapBnds, 'Visible', 'on');
            
            %% PLOT CUMULATIVE VT DATA
            
            % Delete all tracker data from this lap
            delete(D.UI.ratPltH(1:D.UI.cnt_ratPltH))
            delete(D.UI.ratPltHvel(1:D.UI.cnt_ratPltHvel))
            delete(D.UI.robPltHvel(1:D.UI.cnt_robPltHvel))
            D.UI.cnt_ratPltH = 0;
            D.UI.cnt_ratPltHvel = 0;
            D.UI.cnt_robPltHvel = 0;
            
            % Rat pos
            delete(D.UI.Rat.pltHposAll);
            x = D.P.Rat.pos_hist(:,1);
            y = D.P.Rat.pos_hist(:,2);
            exc = find(diff(x)/D.UI.cm2pxl > 10 | diff(y)/D.UI.cm2pxl > 10 == 1) + 1;
            x(exc) = NaN;
            y(exc) = NaN;
            D.UI.Rat.pltHposAll = ...
                plot(x, y, '-', ...
                'Color', D.UI.ratPosHistCol, ...
                'LineWidth', 1, ...
                'Parent', D.UI.axH(1));
            
            % Plot vel all
            cols = [D.UI.ratHistCol; D.UI.robHistCol];
            flds = [{'Rat'},{'Rob'}];
            for i = [2,1]
                fld = flds{i};
                
                % Get lap data
                ind = ~isnan(D.P.(fld).vel_lap(:, 1));
                vel_rad = D.P.(fld).vel_lap(ind, 1);
                vel_roh = D.P.(fld).vel_lap(ind, 2);
                
                % Sort by rad
                [vel_rad, s_ind] = sort(vel_rad);
                vel_roh = vel_roh(s_ind);
                
                % Interpolate missing values
                rad_bins = linspace(0, 2*pi, 101);
                if length(vel_rad)<10 || ...
                        max(diff(rad2deg(vel_rad))) > 45 || ...
                        max(diff(rad2deg(vel_rad))) == 0
                    
                    % Set to nan if insuficient samples or big jumps
                    roh_interp = nan(1,101);
                    
                else
                    
                    % Histogram
                    [~,h_inds]= histc(vel_rad, rad_bins);
                    roh_interp = cell2mat(arrayfun(@(x) nanmean(vel_roh(h_inds==x)), ...
                        1:101, 'Uni', false));
                    
                    % Interpolate missing vals
                    ind = ~isnan(roh_interp);
                    roh_interp = interp1(rad_bins(ind), roh_interp(ind), rad_bins);
                    roh_interp(end) = roh_interp(1);
                    
                    % Keep in bounds
                    roh_interp(roh_interp>D.P.velRohMax) = D.P.velRohMax;
                    roh_interp(roh_interp<D.P.velRohMin) = D.P.velRohMin;
                end
                
                % Add to history
                D.P.(fld).vel_hist(sum(cell2mat(D.C.lap_cnt)),1:101,1) = rad_bins;
                D.P.(fld).vel_hist(sum(cell2mat(D.C.lap_cnt)),1:101,2) = roh_interp;
                
                % Delete old handle and lap data
                delete(D.UI.(fld).pltHvelAll);
                D.P.(fld).vel_lap = NaN(60*60*33,2);
                
                % Get history as 1D array
                vel_rad = reshape(D.P.(fld).vel_hist(:,:,1)',1,[]);
                vel_roh = reshape(D.P.(fld).vel_hist(:,:,2)',1,[]);
                ind = ~isnan(vel_rad);
                
                % Convert to cart
                [x, y] = pol2cart(vel_rad(ind), vel_roh(ind));
                x =  x.*D.PAR.R + D.PAR.XC;
                y =  y.*D.PAR.R + D.PAR.YC;
                % Plot
                D.UI.(fld).pltHvelAll = ...
                    plot(x, y, '-', ...
                    'Color', cols(i,:), ...
                    'LineWidth', 1, ...
                    'Parent', D.UI.axH(1));
            end
            
            % Plot vel avg
            cols = [D.UI.ratAvgCol; D.UI.robAvgCol];
            flds = [{'Rat'},{'Rob'}];
            for i = [2,1]
                fld = flds{i};
                
                % Get accross lap average
                vel_rad = D.P.(fld).vel_hist(1,:,1);
                roh_avg = nanmean(D.P.(fld).vel_hist(:,:,2),1);
                
                % Convert to cart
                [x, y] = pol2cart(vel_rad, roh_avg);
                x =  x.*D.PAR.R + D.PAR.XC;
                y =  y.*D.PAR.R + D.PAR.YC;
                
                % Plot
                delete(D.UI.(fld).pltHvelAvg)
                D.UI.(fld).pltHvelAvg = ...
                    plot(x, y, '-', ...
                    'Color', cols(i,:), ...
                    'LineWidth', 4, ...
                    'Parent', D.UI.axH(1));
            end
            
            % Reset max velocity
            D.P.Rat.vel_max_lap = 0;
            D.P.Rob.vel_max_lap = 0;
            
            %% PRINT LAP TIME INFO
            
            % Save time
            lap_tim_ellapsed = Elapsed_Seconds(now) - D.T.lap_tim;
            
            % Get average
            D.T.lap_tim_sum = D.T.lap_tim_sum + lap_tim_ellapsed;
            lap_tim_average = D.T.lap_tim_sum / sum([D.C.lap_cnt{:}]);
            
            % Update lap time dropdown
            D.UI.lapTimList = [...
                D.UI.lapTimList; ...
                {sprintf('%d: T:%1.1f M:%d', ...
                sum([D.C.lap_cnt{:}]), ...
                lap_tim_ellapsed, ...
                sum(D.C.missed_rew_cnt))}];
            infstr = [...
                sprintf('Lap Times (%1.1fs)', lap_tim_average); ...
                D.UI.lapTimList];
            set(D.UI.popLapTim, 'String', infstr);
            
            % reset timer
            D.T.lap_tim = Elapsed_Seconds(now);
            
        end
        
        % --------------------------PLOT POSITION--------------------------
        
        function [update_ui] = Pos_Plot()
            
            % BAIL IF SETUP NOT FINISHED
            update_ui = false;
            if ~D.F.setup_done
                return
            end
            
            %% ROBOT POS
            
            if D.F.Rob.plot_pos
                
                % Plot rob patch
                if isfield(D.UI, 'ptchRobPos')
                    delete(D.UI.ptchRobPos);
                end
                [xbnd, ybnd] =  Get_Rad_Bnds( [D.P.Rob.guardRad, D.P.Rob.buttRad]);
                D.UI.ptchRobPos = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    D.UI.robNowCol, ...
                    'EdgeColor', D.UI.robNowCol, ...
                    'LineWidth', 2, ...
                    'FaceAlpha',0.75, ...
                    'Parent',D.UI.axH(2));
                
                % Plot current rob tracker pos
                if isfield(D.UI, 'vtRobPltNow')
                    delete(D.UI.vtRobPltNow);
                end
                D.UI.vtRobPltNow = ...
                    plot(D.P.Rob.x(end), D.P.Rob.y(end), 'o', ...
                    'MarkerFaceColor', D.UI.robNowCol, ...
                    'MarkerEdgeColor', D.UI.robNowCol, ...
                    'MarkerSize', 10, ...
                    'Parent', D.UI.axH(2));
                
                % Plot rob arm
                if isfield(D.UI, 'armPltNow')
                    delete(D.UI.armPltNow);
                end
                [xbnd, ybnd] =  Get_Rad_Bnds([D.P.Rob.feedRad, D.P.Rob.buttRad]);
                D.UI.armPltNow = ...
                    plot(xbnd(1,:), ybnd(1,:), ...
                    'Color', D.UI.robNowCol, ...
                    'LineWidth', 5, ...
                    'Parent',D.UI.axH(2));
                
                % Plot guard pos
                if isfield(D.UI, 'quardPltNow')
                    delete(D.UI.quardPltNow);
                end
                [xbnd, ybnd] =  Get_Rad_Bnds(D.P.Rob.guardRad);
                D.UI.quardPltNow = ...
                    plot(xbnd, ybnd, ...
                    'Color', D.UI.guardPosCol, ...
                    'LineWidth', D.UI.guardPosLineWidth, ...
                    'Parent',D.UI.axH(2));
                
                % Plot set pos
                if isfield(D.UI, 'setPltNow')
                    delete(D.UI.setPltNow);
                end
                [xbnd, ybnd] =  Get_Rad_Bnds(D.P.Rob.setRad);
                D.UI.setPltNow = ...
                    plot(xbnd, ybnd, ...
                    'Color', D.UI.setPosCol, ...
                    'LineWidth', D.UI.setPosLineWidth, ...
                    'Parent',D.UI.axH(2));
                
                % Plot feeder pos feed in cond color
                if isfield(D.UI, 'feedPltNow')
                    delete(D.UI.feedPltNow);
                end
                [xbnd, ybnd] =  Get_Rad_Bnds(D.P.Rob.feedRad);
                D.UI.feedPltNow = ...
                    plot(xbnd(1), ybnd(1), 'o', ...
                    'MarkerFaceColor', D.UI.feedPosCol, ...
                    'MarkerEdgeColor', D.UI.robNowCol, ...
                    'LineWidth', 2, ...
                    'MarkerSize', D.UI.feedPosMarkSize, ...
                    'Parent', D.UI.axH(2));
                
            end
            
            %% RAT POS
            
            if D.F.Rat.plot_pos
                
                % Plot all new VT data
                if D.PAR.sesTask == 'Track'
                D.UI.cnt_ratPltH = D.UI.cnt_ratPltH+1;
                D.UI.ratPltH(D.UI.cnt_ratPltH) = ...
                    plot(D.P.Rat.x, D.P.Rat.y, '.', ...
                    'MarkerFaceColor', D.UI.ratPosAllCol, ...
                    'MarkerEdgeColor', D.UI.ratPosAllCol, ...
                    'MarkerSize', 2, ... // TEMP
                    'Parent', D.UI.axH(1));
                end
                
                % Plot occ
                if D.PAR.sesTask == 'Forage'
                    delete(D.UI.imgRFOCC);
                    D.UI.imgRFOCC = imagesc(D.P.occMat+D.P.pathNowMat, ...
                        'Parent', D.UI.axH(3));
                end
                
                % Plot current rat position with larger marker
                if isfield(D.UI, 'vtRatPltNow')
                    delete(D.UI.vtRatPltNow);
                end
                D.UI.vtRatPltNow = ...
                    plot(D.P.Rat.x(end), D.P.Rat.y(end), 'o', ...
                    'MarkerFaceColor', D.UI.ratNowCol, ...
                    'MarkerEdgeColor', [0, 0, 0], ...
                    'MarkerSize', 10, ...
                    'Parent', D.UI.axH(2));
                
            end
            
            %% VELOCITY
            
            % ROB
            if D.F.Rob.plot_vel
                % Store handle and plot
                D.UI.cnt_robPltHvel = D.UI.cnt_robPltHvel+1;
                D.UI.robPltHvel(D.UI.cnt_robPltHvel) = ...
                    plot(D.P.Rob.vel_pol_arr(:,1), D.P.Rob.vel_pol_arr(:,2), '-', ...
                    'Color', D.UI.robNowCol, ...
                    'LineWidth', 2, ...
                    'Parent', D.UI.axH(1));
            end
            
            % RAT
            if D.F.Rat.plot_vel
                % Store handle and plot
                D.UI.cnt_ratPltHvel = D.UI.cnt_ratPltHvel+1;
                D.UI.ratPltHvel(D.UI.cnt_ratPltHvel) = ...
                    plot(D.P.Rat.vel_pol_arr(:,1), D.P.Rat.vel_pol_arr(:,2), '-', ...
                    'Color', D.UI.ratNowCol, ...
                    'LineWidth', 2, ...
                    'Parent', D.UI.axH(1));
            end
            
            %% HEADING
            
            % Plot arrow
            if D.F.plot_hd && D.F.Rat.plot_pos
                
                % Delete plot object
                if isfield(D.UI, 'vtPltHD')
                    delete(D.UI.ratPltHD(:));
                end
                
                % Get current heading
                hd_deg = rad2deg(D.P.Rat.hd_rad(end));
                
                % Calculate coordinates for line
                % x start/end
                xs = D.P.Rat.x(1,end);
                xe = cos(deg2rad(hd_deg))*10 + D.P.Rat.x(1,end);
                x = [xs, xe, ...
                    cos(deg2rad(hd_deg-90))*2 + xe, ...
                    cos(deg2rad(hd_deg))*4 + xe, ...
                    cos(deg2rad(hd_deg+90))*2 + xe, ...
                    xe];
                % y start/end
                ys = D.P.Rat.y(end);
                ye = sin(deg2rad(hd_deg))*10 + D.P.Rat.y(end);
                y = [ys, ye, ...
                    sin(deg2rad(hd_deg-90))*2 + ye, ...
                    sin(deg2rad(hd_deg))*4 + ye, ...
                    sin(deg2rad(hd_deg+90))*2 + ye, ...
                    ye];
                
                % Plot thick backround line
                D.UI.ratPltHD(1) = ...
                    plot(x, y, '-', ...
                    'Color', [0.5, 0.5, 0.5], ...
                    'LineWidth', 3, ...
                    'Parent', D.UI.axH(2));
                
                % Plot thin foreground line
                D.UI.ratPltHD(2) = ...
                    plot(D.UI.axH(2), x, y, '-', ...
                    'Color', D.UI.ratPosAllCol, ....
                    'LineWidth', 1, ...
                    'Parent', D.UI.axH(2));
            end
            
            % Check if ui updated
            if ...
                    D.F.Rob.plot_pos ||...
                    D.F.Rat.plot_pos ||...
                    D.F.Rob.plot_vel ||...
                    D.F.Rat.plot_vel
                update_ui = true;
            end
            
            % Reset flags
            D.F.Rob.plot_pos = false;
            D.F.Rat.plot_pos = false;
            D.F.Rob.plot_vel = false;
            D.F.Rat.plot_vel = false;
            
        end
        
        % -------------------------PRINT SES INFO--------------------------
        
        function [] = Inf_Print()
            
            %% BAIL IF SETUP NOT FINISHED OR DT NOT REACEHD
            if ...
                    ~D.F.setup_done || ...
                    Elapsed_Seconds(now) - D.T.info_txt_update < 0.1
                return
            end
            
            %% PRINT PERFORMANCE INFO
            
            % total
            infstr = sprintf([...
                'Laps______Total:%s%d\n', ...
                'Rewards___Total:%s%d\n', ...
                'Rotations_Total:%s%d\n', ...
                'Missed_Rewards_:%s%d|%d\n',...
                'Bulldozings____:%s%d'], ...
                repmat('_',1,1), sum([D.C.lap_cnt{:}]), ...
                repmat('_',1,1), sum([D.C.rew_cnt{:}]), ...
                repmat('_',1,1), D.C.rot_cnt, ...
                repmat('_',1,1), D.C.missed_rew_cnt(1),sum(D.C.missed_rew_cnt), ...
                repmat('_',1,1), D.C.bull_cnt);
            set(D.UI.txtPerfInf(4), 'String', infstr)
            
            % standard
            infstr = sprintf([...
                'Laps______Stand:%s%d\n', ...
                'Rewards___Stand:%s%d'], ...
                repmat('_',1,2), D.C.lap_cnt{3}(end), ...
                repmat('_',1,2), D.C.rew_cnt{3}(end));
            set(D.UI.txtPerfInf(1), 'String', infstr)
            
            % 40 deg
            infstr = sprintf([...
                'Laps______40%c:%s%d|%d\n', ...
                'Rewards___40%c:%s%d|%d'], ...
                char(176), repmat('_',1,4), D.C.lap_cnt{2}(end), sum(D.C.lap_cnt{2}), ...
                char(176), repmat('_',1,4), D.C.rew_cnt{2}(end), sum(D.C.rew_cnt{2}));
            set(D.UI.txtPerfInf(2), 'String', infstr)
            
            % 0 deg
            infstr = sprintf([...
                'Laps______0%c:%s%d|%d\n', ...
                'Rewards___0%c:%s%d|%d'], ...
                char(176), repmat('_',1,5), D.C.lap_cnt{1}(end), sum(D.C.lap_cnt{1}), ...
                char(176), repmat('_',1,5), D.C.rew_cnt{1}(end), sum(D.C.rew_cnt{1}));
            set(D.UI.txtPerfInf(3), 'String', infstr)
            
            % Rat vel
            infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', ...
                D.P.Rat.vel, D.P.Rat.vel_max_lap, D.P.Rat.vel_max_all);
            set(D.UI.txtPerfInf(5), 'String', infstr)
            
            % Robot vel
            infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', ...
                D.P.Rob.vel, D.P.Rob.vel_max_lap, D.P.Rob.vel_max_all);
            set(D.UI.txtPerfInf(6), 'String', infstr)
            
            % Bat volt
            if c2m.('J').dat1 ~= 0
                D.PAR.vcc_last = D.PAR.vcc_now;
                D.PAR.vcc_now = c2m.('J').dat1;
                
                % Check if voltage low
                if D.PAR.vcc_now <= D.PAR.batVoltWarning && D.PAR.vcc_now > 0
                    set(D.UI.txtPerfInf(7), 'ForegroundColor', D.UI.warningCol);
                    
                    % Turn red and flicker if bellow 12 V
                    if strcmp(get(D.UI.txtPerfInf(7), 'Visible'), 'on')
                        set(D.UI.txtPerfInf(7), 'Visible', 'off')
                    else
                        set(D.UI.txtPerfInf(7), 'Visible', 'on')
                    end
                else
                    
                    % Check if printing a new value is new
                    if Elapsed_Seconds(now) - c2m.('J').t_rcvd < 5
                        set(D.UI.txtPerfInf(7), 'ForegroundColor', D.UI.activeCol);
                    else
                        set(D.UI.txtPerfInf(7), 'ForegroundColor', D.UI.enabledPrintFrgCol);
                    end
                end
                infstr = sprintf('Battery:_%0.2fV', D.PAR.vcc_now);
                set(D.UI.txtPerfInf(7), 'String', infstr)
            end
            
            %% PRINT TIME INFO
            
            % Get session time
            nowTim(1) =  Elapsed_Seconds(now) - D.T.ses_str_tim;
            
            % Get recording elapsed time plus saved time
            if  D.F.rec
                nowTim(2) = (Elapsed_Seconds(now) - D.T.rec_tim)  + D.T.rec_tot_tim;
            else nowTim(2) = D.T.rec_tot_tim; % keep showing save time
            end
            
            % Get lap time
            if D.F.rat_in
                nowTim(3) = Elapsed_Seconds(now) - D.T.lap_tim;
            else nowTim(3) = 0;
            end
            
            % Run time
            if D.F.rat_in && ~D.F.rec_done
                D.T.run_tim = Elapsed_Seconds(now) - D.T.run_str;
            end
            nowTim(4) = D.T.run_tim;
            
            % Make string
            infstr = sprintf([ ...
                '(SES):%s%s\n', ...
                '(REC):%s%s\n', ...
                '(LAP):%s%s\n'...
                '(RUN):%s%s\n'], ...
                repmat('_',1,6), datestr(nowTim(1)/(24*60*60), 'HH:MM:SS'), ...
                repmat('_',1,6), datestr(nowTim(2)/(24*60*60), 'HH:MM:SS'), ...
                repmat('_',1,6), datestr(nowTim(3)/(24*60*60), 'HH:MM:SS'), ...
                repmat('_',1,6), datestr(nowTim(4)/(24*60*60), 'HH:MM:SS'));
            
            % Print time
            set(D.UI.txtTimElspInf, 'String', infstr)
            
            % Save current time to UserData
            set(D.UI.txtTimElspInf, 'UserData', nowTim)
            
            % Loop and reward times
            infstr = sprintf( ...
                [...
                'RD: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                'RT: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                'DN: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                'Lp: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                ], ...
                D.DB.rew_duration(1), D.DB.rew_duration(2), D.DB.rew_duration(3), D.DB.rew_duration(4)/D.DB.rew_duration(5), ...
                D.DB.rew_round_trip(1), D.DB.rew_round_trip(2), D.DB.rew_round_trip(3), D.DB.rew_round_trip(4)/D.DB.rew_round_trip(5), ...
                D.DB.draw(1), D.DB.draw(2), D.DB.draw(3), D.DB.draw(4)/D.DB.draw(5), ...
                D.DB.loop(1), D.DB.loop(2), D.DB.loop(3), D.DB.loop(4)/D.DB.loop(5) ...
                );
            set(D.UI.txtTimDebug, 'String', infstr)
            
        end
        
        % ------------------------------RUN TEST CODE----------------------
        
        function [] = Run_Test()
            
            %% BAIL IF NOT TEST RUN
            if ~D.DB.isTestRun
                return
            end
            
            %% PID CALIBRATION
            if D.DB.doPidCalibrationTest && ...
                    ~D.F.pidStarted && ...
                    c2m.('K').dat1 == 1
                % Start pid test
                SendM2C('T', sysTest, 0);
                % Set flag to start pid
                D.F.pidStarted = true;
            end
            
            %% HALT ERROR TEST
            if D.DB.doHaltErrorTest
                
                % Check if robot should be restarted
                if D.DB.isHalted && ...
                        ~D.F.is_halted && ...
                        Elapsed_Seconds(now) - D.DB.t_halt > D.DB.haltDur
                    
                    % Save and print halt error
                    % Store halt error
                    halt_err = Rad_Diff(D.P.Rob.radLast, D.DB.sendPos) * ((140*pi)/(2*pi));
                    D.DB.halt_error(1) = halt_err;
                    D.DB.halt_error(2) = min(D.DB.halt_error(1), D.DB.halt_error(2));
                    D.DB.halt_error(3) = max(D.DB.halt_error(1), D.DB.halt_error(3));
                    D.DB.halt_error(4) = D.DB.halt_error(1) + D.DB.halt_error(4);
                    D.DB.halt_error(5) = D.DB.halt_error(5) + 1;
                    % Print halt error
                    D.DB.halt_error_str = [D.DB.halt_error_str, ...
                        sprintf('\r%4.0f, %4.0f, %4.0f, %0.4f, %4.0f\r', ...
                        D.DB.halt_error(1), D.DB.halt_error(2), D.DB.halt_error(3), D.DB.halt_error(4)/D.DB.halt_error(5), D.DB.nowVel)];
                    % Log/print
                    Console_Write(D.DB.halt_error_str);
                    
                    % Check if vel should be stepped
                    if D.DB.haltCnt == D.DB.stepSamp
                        
                        % Reset counter
                        D.DB.haltCnt = 0;
                        
                        % Incriment step
                        D.DB.nowStep = D.DB.nowStep+1;
                        
                        % Incriment vel
                        if D.DB.nowStep <= length(D.DB.velSteps);
                            D.DB.nowVel = D.DB.velSteps(D.DB.nowStep);
                        else
                            % End test
                            D.DB.nowVel = 0;
                            D.DB.doHaltErrorTest = false;
                            
                            % Save data to csv file
                            fi_out = fullfile(D.DIR.ioTestOut,'halt_error.csv');
                            file_id = fopen(fi_out,'w');
                            fprintf(file_id,'Err, Min, Max, Avg, Vel\r');
                            fprintf(file_id,D.DB.halt_error_str(3:end));
                            fclose(file_id);
                            % Log/print
                            Console_Write(sprintf('[Run_Test] Saved Halt Error Test: %s', ...
                                fi_out));
                        end
                        
                        % Log/print
                        Console_Write(sprintf('[Run_Test] Halt Error Test: New Vel=%dcm/sec', ...
                            D.DB.nowVel));
                        
                    end
                    
                    % Tell CS to resume run
                    SendM2C('T', sysTest, D.DB.nowVel);
                    
                    % Set flag
                    D.DB.isHalted = false;
                    
                    % Check if robot has stopped
                elseif ~D.DB.isHalted && ...
                        D.P.Rob.vel < 5
                    
                    % Tell CS to resume run
                    SendM2C('T', sysTest, D.DB.nowVel);
                    
                end
                
                % Check if robot has passed 0 deg
                if ~D.DB.isHalted && ...
                        Elapsed_Seconds(now) - D.DB.t_halt > D.DB.haltDur+1 && ...
                        any(Check_Pol_Bnds(D.P.Rob.rad, D.P.Rob.roh, [deg2rad(355), deg2rad(360)]))
                    
                    % Incriment counter
                    D.DB.haltCnt = D.DB.haltCnt+1;
                    
                    % Tell CS to Halt Robot
                    SendM2C('T', sysTest, 0);
                    
                    % Store robots current pos and time
                    D.DB.sendPos = D.P.Rob.radLast;
                    D.DB.t_halt = Elapsed_Seconds(now);
                    
                    % Set flag
                    D.DB.isHalted = true;
                    
                    % Log/print
                    Console_Write('[Run_Test] Halt Error Test: Halting Robot');
                end
                
            end
            
            %% SIMULATED RAT TEST
            if D.DB.doSimRatTest
                
                % Wait till recording
                if D.F.rec
                    
                    % Wait for 5 sec after setup
                    if Elapsed_Seconds(now) - D.T.rec_tim > 5
                        
                        % Local vars
                        cm = 0;
                        vel_now = 0;
                        
                        % Start rat in start quad
                        if D.F.initVals
                            
                            % Setup vars
                            if D.PAR.sesTask == 'Forage'
                                
                                % Store current target
                                D.DB.simTargAng = D.PAR.pathTargArr(D.I.targInd);
                                
                                % Compute angle between start pos and targ
                                ang_str = rad2deg(mean(D.PAR.strQuadBnds));
                                ang_targ = D.DB.simTargAng;
                                [x1,y1] = pol2cart(deg2rad(ang_str), 1);
                                [x2,y2] = pol2cart(deg2rad(ang_targ), 1);
                                x_diff = x2-x1;
                                y_diff = y2-y1;
                                ang_new = rad2deg(atan(y_diff/x_diff));
                                if (x_diff < 0)
                                    ang_new = ang_new - 180;
                                    if ang_new < 0
                                        ang_new = ang_new + 360;
                                    end
                                end
                                D.DB.simRunRad = deg2rad(ang_new);
                                
                                % Store rf roh
                                roh = mean(D.P.rfRohBnd);
                            else
                                % Store track roh
                                roh = mean(D.P.trackRohBnd);
                            end
                            D.DB.simRadLast = mean(D.PAR.strQuadBnds);
                            D.DB.simRohLast = 0;
                            D.DB.simVelLast = 0;
                            D.DB.simTSStart = Elapsed_Seconds(now);
                            D.DB.simTSLast = 0;
                            
                            % Get inital x/y
                            xy_pos = Rad_2_VT(wrapTo2Pi(D.DB.simRadLast), roh);
                            D.DB.simXY = reshape(xy_pos', 1, []);
                            
                            % Make UI stuff visible
                            set(D.UI.sldSimVel, ...
                                'Visible', 'on',...
                                'Enable', 'on');
                            set(D.UI.txtSimVel, ...
                                'Visible', 'on');
                            
                            % Send test info to robot once
                            SendM2C('T', sysTest, 0);
                            
                            % Unset flag
                            D.F.initVals = false;
                        end
                        
                        % Compute ts(us) from dt(s)
                        ts_now = ceil((Elapsed_Seconds(now) - D.DB.simTSStart)*10^6);
                        dt_sec = (ts_now - D.DB.simTSLast) / 10^6;
                        D.DB.simTSLast = ts_now;
                        
                        % Get slider val
                        sld_vel = round(get(D.UI.sldSimVel, 'Value'));
                        set(D.UI.txtSimVel, 'String', num2str(sld_vel));
                        
                        % Update vel if not halted or holding for 2 sec for setup
                        if ...
                                D.F.rat_in && ...
                                ~D.F.is_halted && ...
                                Elapsed_Seconds(now) - D.T.run_str > 2
                            
                            % Check vel
                            if D.DB.simVelLast == sld_vel
                                % Hold velocity
                                vel_now = D.DB.simVelLast;
                            elseif D.DB.simVelLast < sld_vel
                                % Accelerate
                                vel_now = D.DB.simVelLast + (D.DB.ratMaxAcc*dt_sec);
                            elseif D.DB.simVelLast > sld_vel
                                % Deccelerate
                                vel_now = D.DB.simVelLast - (D.DB.ratMaxDec*dt_sec);
                            end
                            
                            % Keep in bounds
                            if vel_now > D.UI.sldSimVel.Max
                                vel_now = D.UI.sldSimVel.Max;
                            elseif vel_now < D.UI.sldSimVel.Min
                                vel_now = D.UI.sldSimVel.Min;
                            end
                        else
                            % Keep robot halted
                            vel_now = 0;
                        end
                        
                        % Compute new pos
                        if D.F.rat_in && ...
                                ~isnan(vel_now) && ~isnan(cm) && ~isnan(dt_sec)
                            
                            % Get delta pos cm
                            cm = vel_now * dt_sec;
                            
                            % Get delta pos
                            if D.PAR.sesTask == 'Track'
                                
                                % Compute rad change
                                rad_diff = cm / ((140 * pi)/(2 * pi));
                                rad_now = Rad_Diff(D.DB.simRadLast, rad_diff);
                                D.DB.simRadLast = rad_now;
                                
                                % Convert rad back to cart
                                xy_pos = Rad_2_VT(wrapTo2Pi(rad_now), mean(D.P.trackRohBnd));
                                D.DB.simXY = reshape(xy_pos', 1, []);
                                
                            else
                                
                                % Check for changed targ
                                if D.DB.simTargAng ~= D.PAR.pathTargArr(D.I.targInd)
                                    
                                    % Get current rad pos
                                    [rad_start, ~] = VT_2_Rad(D.DB.simXY);
                                    
                                    % Add some noise
                                    ang_end = D.PAR.pathTargArr(D.I.targInd);
                                    noise = (rand(1) - 0.5) * ...
                                        (((2*D.UI.rfRad*pi) * (D.PAR.pathTargWdt/360)) * 0.5);
                                    ang_end = ang_end+noise;
                                    
                                    % Compute angle between current pos and new targ
                                    ang_str = rad2deg(rad_start);
                                    ang_targ = ang_end;
                                    [x1,y1] = pol2cart(deg2rad(ang_str), 1);
                                    [x2,y2] = pol2cart(deg2rad(ang_targ), 1);
                                    x_diff = x2-x1;
                                    y_diff = y2-y1;
                                    ang_new = rad2deg(atan(y_diff/x_diff));
                                    if (x_diff < 0)
                                        ang_new = ang_new - 180;
                                        if ang_new < 0
                                            ang_new = ang_new + 360;
                                        end
                                    end
                                    D.DB.simRunRad = deg2rad(ang_new);
                                    
                                    % Store new target
                                    D.DB.simTargAng = D.PAR.pathTargArr(D.I.targInd);
                                    
                                    % Reset roh
                                    D.DB.simRohLast = 0;
                                end
                                
                                % Move along roh
                                roh_diff = cm / D.UI.arnRad;
                                roh_now = D.DB.simRohLast + roh_diff;
                                D.DB.simRohLast = roh_now;
                                
                                % Convert to cart
                                rad = wrapTo2Pi(D.DB.simRunRad);
                                rad = abs(rad - 2*pi);
                                rad = wrapToPi(rad);
                                [x_diff,y_diff] = pol2cart(rad, roh_now);
                                
                                % Check if out of bounds
                                [~, roh] = VT_2_Rad([D.DB.simXY(1)+x_diff, D.DB.simXY(2)+y_diff]);
                                if roh < D.P.rfRohBnd(2)
                                    D.DB.simXY = [D.DB.simXY(1)+x_diff, D.DB.simXY(2)+y_diff];
                                end
                                
                                % Store current targ ang
                                D.DB.simTargAng = D.PAR.pathTargArr(D.I.targInd);
                            end
                            
                        end
                        
                        % Send Pos data to CS
                        SendM2C('p', ts_now, D.DB.simXY(1), D.DB.simXY(2));
                        
                        % Update simulated rat data
                        D.DB.simVelLast = vel_now;
                        D.P.Rat.vtTS = single(ts_now);
                        D.P.Rat.vtPos = single(D.DB.simXY);
                        D.P.Rat.vtNRecs = single(1);
                        
                        % Run VT_Proc('Rat');
                        VT_Proc('Rat');
                        
                    end
                    
                end
                
            end
            
        end
        
        
        
        
        
        
        %% ======================== CALLBACK FUNCTIONS ====================
        
        % RAT SELECTION
        function [] = PopRat(~, ~, ~)
            
            if get(D.UI.popRat,'Value') ~= 1
                
                % Change to active color
                set(D.UI.popRat, ...
                    'BackgroundColor', D.UI.activeCol, ...
                    'ForegroundColor', D.UI.enabledBtnFrgCol);
                
                % Get Rat label
                D.PAR.ratLab = ... % ('r####')
                    ['r',D.UI.ratList{get(D.UI.popRat,'Value')}(1:4)];
                
                % Get rat number
                D.PAR.ratNum = ... % (####)
                    str2double(D.PAR.ratLab(2:end));
                
                % Get rat index in D.SS_In_All
                D.PAR.ratInd = ...
                    find(ismember(D.SS_In_All.Properties.RowNames, D.PAR.ratLab));
                
                % Get rat age
                D.PAR.ratAgeGrp = ... % [Young,Old]
                    D.SS_In_All.Age_Group(D.PAR.ratInd);
                
                % Get rat dob
                D.PAR.ratDOB = ...
                    D.SS_In_All.DOB(D.PAR.ratInd);
                
                % Get feeder condition
                D.PAR.ratFeedCnd = ... % [C1,C2]
                    D.SS_In_All.Feeder_Condition(D.PAR.ratInd);
                
                % Get feeder condition number
                D.PAR.ratFeedCnd_Num = ... % [1,2]
                    find(D.PAR.catFeedCnd == D.PAR.ratFeedCnd);
                
                % SET UI TO LAST SESSION
                
                % Set Session Condition
                D.PAR.sesCond = ...
                    D.SS_In_All.Session_Condition(D.PAR.ratInd);
                set(D.UI.popCond, 'Value', ...
                    find(ismember(D.UI.condList, D.PAR.sesCond)));
                % run callback
                PopCond();
                
                % Set Session Task
                D.PAR.sesTask = ...
                    D.SS_In_All.Session_Task(D.PAR.ratInd);
                set(D.UI.popTask, 'Value', ...
                    find(ismember(D.UI.taskList, D.PAR.sesTask)));
                % run callback
                PopTask();
                
                % Set cue condition
                D.PAR.cueFeed = ...
                    D.SS_In_All.Cue_Condition(D.PAR.ratInd);
                set(D.UI.toggCue( ...
                    ismember(D.PAR.catCueCond, D.PAR.cueFeed)), 'Value', 1);
                % Set all others to off
                set(D.UI.toggCue( ...
                    ~ismember(D.PAR.catCueCond, D.PAR.cueFeed)), 'Value', 0);
                ToggCue();
                
                % Set reward delay
                D.PAR.rewDel = ...
                    D.SS_In_All.Reward_Delay(D.PAR.ratInd);
                set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.delList, D.PAR.rewDel)));
                % run callback
                PopRewDel();
                
                % Set sound conditions
                D.UI.snd(1:2) = logical(...
                    D.SS_In_All.Sound_Conditions(D.PAR.ratInd,1:2));
                set(D.UI.toggSnd(D.UI.snd(1:2)), 'Value', 1);
                ToggSnd();
                
                % Log/print
                Console_Write(sprintf('[%s] Set to \"%s\"', 'PopRat', D.PAR.ratLab));
                Update_UI(10);
            end
            
        end
        
        % SESSION CONDITION
        function [] = PopCond(~, ~, ~)
            
            % Change to active color
            set(D.UI.popCond, ...
                'BackgroundColor', D.UI.activeCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol);
            
            % Store value
            D.PAR.sesCond(:) = D.UI.condList(get(D.UI.popCond, 'Value'));
            
            % Change other buttons for manual training
            if D.PAR.sesCond == 'Manual_Training'
                
                % Set reward delay
                set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.delList, D.UI.delList(2))));
                % run callback
                PopRewDel();
                
                % Set cue condition
                set(D.UI.toggCue( ...
                    ismember(D.PAR.catCueCond, 'None')), 'Value', 1);
                % Set all others to off
                set(D.UI.toggCue( ...
                    ~ismember(D.PAR.catCueCond, 'None')), 'Value', 0);
                ToggCue();
                
                % Change other buttons for rot cond
            elseif D.PAR.sesCond == 'Rotation'
                
                % Set Session Task to Track
                set(D.UI.popTask, 'Value', ...
                    find(ismember(D.UI.taskList, 'Track')));
                % run callback
                PopTask();
                
                % Set reward delay
                set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.delList, D.UI.delList(end))));
                % run callback
                PopRewDel();
                
                % Set cue condition
                set(D.UI.toggCue( ...
                    ismember(D.PAR.catCueCond, 'None')), 'Value', 1);
                % Set all others to off
                set(D.UI.toggCue( ...
                    ~ismember(D.PAR.catCueCond, 'None')), 'Value', 0);
                ToggCue();
                
                % Set SOUND condition
                set(D.UI.toggSnd(D.UI.snd(1:2)), 'Value', 1);
                ToggSnd();
                
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%s\"', 'PopCond', char(D.PAR.sesCond)));
            Update_UI(10);
            
            % Check settings
            CheckSetupDefaults();
        end
        
        % SESSION TASK
        function [] = PopTask(~, ~, ~)
            
            % Change to active color
            set(D.UI.popTask, ...
                'BackgroundColor', D.UI.activeCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol);
            
            % Store value
            D.PAR.sesTask(:) = D.UI.taskList(get(D.UI.popTask, 'Value'));
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%s\"', 'PopTask', char(D.PAR.sesTask)));
            Update_UI(10);
            
            % Check settings
            CheckSetupDefaults();
        end
        
        % REWARD DELAY
        function [] = PopRewDel(~, ~, ~)
            
            % Change to active color
            set(D.UI.popRewDel, ...
                'BackgroundColor', D.UI.activeCol, ...
                'ForegroundColor', D.UI.enabledBtnFrgCol);
            
            if get(D.UI.popRewDel,'Value') ~= 1
                % Get selected minimum time in bounds for reward
                D.PAR.rewDel = str2double(D.UI.delList{get(D.UI.popRewDel,'Value'),:});
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%s\"', 'PopRewDel', num2str(D.PAR.rewDel)));
            Update_UI(10);
            
            % Check settings
            CheckSetupDefaults();
        end
        
        % CUE CONDITION
        function [] = ToggCue(~, ~, ~)
            
            % Get trigger button
            cue_cond = find(cell2mat(get(D.UI.toggCue, 'Value')) == 1);
            
            % Switching buttons
            if length(cue_cond)>1
                cue_cond = get(gcbo, 'UserData');
            end
            
            % Change to active
            if get(D.UI.toggCue(cue_cond), 'Value') == 1
                
                % Store string
                cue_str = get(D.UI.toggCue(cue_cond), 'String');
                
                % Activate current button
                set(D.UI.toggCue(cue_cond), ...
                    'BackgroundColor', D.UI.activeCol);
                
                % Inactivate other buttons
                set(D.UI.toggCue(([1,2,3] ~= cue_cond)), ...
                    'BackgroundColor', D.UI.enabledCol, ...
                    'Value',0);
                
                % Save data
                D.PAR.cueFeed(:) = cue_str;
                
                % Change to inactive
            else
                set(D.UI.toggCue(cue_cond), ...
                    'BackgroundColor', D.UI.enabledCol, ...
                    'Value',0);
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%s\"', 'ToggCue', char(D.PAR.cueFeed)));
            Update_UI(10);
            
            % Check settings
            CheckSetupDefaults();
        end
        
        % SOUND CONDITION
        function [] = ToggSnd(~, ~, ~)
            
            % Note: D.UI.snd format
            %         {'White'}
            %         {'Reward'}
            
            for z_sa = 1:2
                % Get button ID and state value
                D.UI.sndVal = get(D.UI.toggSnd(z_sa), 'Value');
                D.UI.sndID = get(D.UI.toggSnd(z_sa), 'UserData');
                if D.UI.sndVal == 1
                    set(D.UI.toggSnd(z_sa), ...
                        'BackgroundColor', D.UI.activeCol);
                else
                    set(D.UI.toggSnd(z_sa), ...
                        'BackgroundColor', D.UI.enabledCol);
                end
                if D.UI.sndVal == 1
                    D.UI.snd(z_sa) = true;
                else
                    D.UI.snd(z_sa) = false;
                end
            end
            
            % Turn off all sound if white noise is off
            if D.UI.snd(1) == false
                D.UI.snd(2) = false;
                set(D.UI.toggSnd(2), 'Value', 0);
                set(D.UI.toggSnd(2), ...
                    'BackgroundColor', D.UI.enabledCol);
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\" \"%d\"', 'ToggSnd', ...
                get(D.UI.toggSnd(1), 'Value'), get(D.UI.toggSnd(2), 'Value')));
            Update_UI(10);
            
            % Check settings
            CheckSetupDefaults();
        end
        
        % SETUP DONE
        function [] = BtnSetupDone(~, ~, ~)
            
            % Disable button so only pressed once
            set(D.UI.btnSetupDone, 'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            
            % Confirm UI entries
            if ...
                    get(D.UI.popRat,'Value') == 1 || ...
                    get(D.UI.popCond,'Value') == 1 || ...
                    get(D.UI.popTask,'Value') == 1 || ...
                    get(D.UI.popRewDel,'Value') == 1 || ...
                    all(cell2mat(get(D.UI.toggCue,'Value')) == 0)
                
                choice = dlgAWL(...
                    '!!WARNING: MISSING ENTRY!!', ...
                    'MISSING SETUP ENTRIES', ...
                    'OK', [], [], 'OK', ...
                    D.UI.qstDlfPos, ...
                    'Warn');
                
                % Bail out of function
                switch choice
                    case 'OK'
                        return
                end
                
            end
            
            %                     % Confirm if all fields entered correctly with pop-up window
            %                     qststr = sprintf(['IS THIS CORRECT:\n\n'...
            %                         'Rat_Number:_______%d\n'...
            %                         'ICR_Condition:____%s\n', ...
            %                         'Feeder_Delay:_____%1.1f\n', ...
            %                         'Cue_Conditon:_____%s\n'...
            %                         'Sound_White:______%s\n'...
            %                         'Sound_Reward:_____%s\n'], ...
            %                         D.PAR.ratNum, ...
            %                         D.UI.condList{get(D.UI.popCond, 'Value')}, ...
            %                         D.PAR.rewDel, ...
            %                         char(D.PAR.cueFeed), ...
            %                         D.UI.sndState{get(D.UI.toggSnd(1),'Value')+1}, ...
            %                         D.UI.sndState{get(D.UI.toggSnd(2),'Value')+1});
            %                     choice = dlgAWL(qststr, ...
            %                         'CHECK SETTINGS', ...
            %                         'Yes', 'No', [], 'No', ...
            %                         D.UI.qstDlfPos, ...
            %                         'Ques');
            %
            %                     % Handle response
            %                     switch choice
            %                         case 'Yes'
            %                         case 'No'
            %                             return
            %                     end
            
            % Signal to continue setup
            D.F.data_loaded = true;
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnSetupDone', get(D.UI.btnSetupDone,'Value')));
            Update_UI(10);
        end
        
        % ACQ BUTTON
        function [] = BtnAcq(~, ~, ~)
            
            % Run only if not recording
            if get(D.UI.btnRec,'Value') ~= 1
                
                % Start aquisition
                if get(D.UI.btnAcq,'Value') == 1
                    NlxSendCommand('-StartAcquisition');
                    set(D.UI.btnAcq,'CData',D.UI.radBtnCmap{2})
                else NlxSendCommand('-StopAcquisition');
                    set(D.UI.btnAcq,'CData',D.UI.radBtnCmap{1})
                end
                
                % Set time tracking variables
                if D.F.acq
                    % save out time before stopping
                    D.T.acq_tot_tim = (Elapsed_Seconds(now) - D.T.acq_tim) + D.T.acq_tot_tim;
                end
                
                % Change aquiring status
                D.F.acq = ~D.F.acq;
                
                % Reset temp now
                D.T.acq_tim = Elapsed_Seconds(now);
                
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnAcq', get(D.UI.btnAcq,'Value')));
            Update_UI(10);
        end
        
        % REC BUTTON
        function [] = BtnRec(~, ~, ~)
            
            % Start aquisition
            if get(D.UI.btnRec,'Value') == 1
                NlxSendCommand('-StartRecording');
                set(D.UI.btnRec,'CData',D.UI.radBtnCmap{3});
            else
                NlxSendCommand('-StopRecording');
                set(D.UI.btnRec,'CData',D.UI.radBtnCmap{1});
            end
            
            % Set time tracking variables
            if  D.F.rec
                % save out time before stopping
                D.T.rec_tot_tim = (Elapsed_Seconds(now) - D.T.rec_tim) + D.T.rec_tot_tim;
            end
            D.F.rec = ~ D.F.rec;
            D.T.rec_tim = Elapsed_Seconds(now);
            
            % Enable icr button
            if strcmp(get(D.UI.btnICR, 'Enable'), 'off')
                if D.PAR.sesCond == 'Rotation'
                    set(D.UI.btnICR, ...
                        'Enable', 'on', ...
                        'String', D.UI.btnICRstr{2}, ...
                        'BackgroundColor', D.UI.rotCol(2,:));
                end
            end
            
            % Enable recording done button
            if strcmp(get(D.UI.btnRecDone, 'Enable'), 'off') && ...
                    ~D.F.rec_done
                set(D.UI.btnRecDone, ...
                    'Enable', 'on', ...
                    'ForegroundColor', D.UI.enabledBtnFrgCol, ...
                    'BackgroundColor', D.UI.enabledCol)
            end
            
            % Start acq if not running
            if get(D.UI.btnRec,'Value') == 1 && ...
                    get(D.UI.btnAcq,'Value') == 0
                
                % Set aquire button value to 1
                set(D.UI.btnAcq,'Value', 1)
                % run BtnAcq
                BtnAcq(D.UI.btnAcq);
                
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnRec', get(D.UI.btnRec,'Value')));
            Update_UI(10);
        end
        
        % ROTATION BUTTON
        function [] = BtnICR(~, ~, ~)
            
            % get image and color for post rotation feeder
            rotNext = [1, 2] ~=  D.I.rot;
            
            % Determine rotation trigger bounds for this event
            D.UI.rotBndNext = D.UI.rotBnds(...
                D.PAR.catRotPos == D.PAR.rotPos(D.C.rot_cnt+1), ...
                :, rotNext);
            
            % Plot selected rotation trigger bounds
            [xbnd, ybnd] =  Get_Rad_Bnds(D.UI.rotBndNext);
            D.UI.ptchRtTrgH = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                D.UI.rotCol(rotNext,:), ...
                'EdgeColor', D.UI.activeCol, ...
                'LineWidth', 2, ...
                'FaceAlpha',0.75, ...
                'Parent', D.UI.axH(2));
            
            % Make button red and set user data to true
            set(D.UI.btnICR,'string', 'Rotation Ready', ...
                'BackgroundColor', D.UI.activeCol, ...
                'UserData', true);
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnICR', get(D.UI.btnICR,'Value')));
            Update_UI(10);
        end
        
        % RECORDING DONE
        function [] = BtnRecDone(~, ~, ~)
            
            % Check if session is done
            choice = dlgAWL(...
                'End Session?', ...
                'END SESSION', ...
                'Yes', 'No', [], 'No', ...
                D.UI.qstDlfPos, ...
                'Ques');
            
            % Handle response
            switch choice
                case 'Yes'
                    % Post NLX event: session end
                    NlxSendCommand(D.NLX.ses_end_evt);
                case 'No'
                    return
            end
            
            % Disable button so only pressed once
            set(D.UI.btnRecDone, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            
            % Set flag
            D.F.rec_done = true;
            
            % Save end time
            D.T.ses_end_tim = Elapsed_Seconds(now);
            
            % Print end time
            infstr = datestr(now, 'HH:MM:SS');
            set(D.UI.editTimEndInf, 'String', infstr)
            
            % Halt robot if not already halted
            if strcmp(get(D.UI.btnHaltRob, 'Enable'), 'on')
                if (~D.F.is_halted)
                    set(D.UI.btnHaltRob, 'Value', 1);
                    BtnHaltRob();
                end
            end
            
            % Check if rat is out of room
            dlgAWL(...
                'Take out rat', ...
                'RAT OUT', ...
                'OK', [], [], 'OK', ...
                D.UI.qstDlfPos, ...
                'Dflt');
            
            % Post NLX event: rat out
            NlxSendCommand(D.NLX.rat_out_evt);
            
            % Stop halt if still active
            if (D.F.is_halted)
                set(D.UI.btnHaltRob, 'Value', 0);
                BtnHaltRob();
            end
            
            % Stop bulldoze if still active
            if (get(D.UI.btnBulldoze, 'Value') == 1)
                set(D.UI.btnBulldoze, 'Value', 0);
                Bulldoze();
            end
            
            % Tell CS rat is out
            SendM2C('I', 0)
            D.F.rat_in = false;
            
            % Stop recording
            if D.F.rec
                set(D.UI.btnRec,'Value', 0)
                BtnRec(D.UI.btnRec);
            end
            
            % Disable all recording buttons
            set(D.UI.panRec, ...
                'ForegroundColor', D.UI.disabledBckCol, ...
                'HighlightColor', D.UI.disabledBckCol)
            set(D.UI.btnAcq, ...
                'Enable', 'off', ...
                'Visible', 'off')
            set(D.UI.btnRec, ...
                'Enable', 'off', ...
                'Visible', 'off')
            set(D.UI.btnICR, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            % Disable extra buttons
            set(D.UI.btnHaltRob, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            set(D.UI.popBulldoze, ...
                'Enable', 'off')
            set(D.UI.btnBulldoze, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            set(D.UI.btnBlockCue, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            set(D.UI.btnAllCue, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            set(D.UI.btnReward, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            set(D.UI.popReward, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            set(D.UI.btnClrVT, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            % Disable toggSnd buttons
            set(D.UI.toggSnd, 'Enable', 'off')
            
            % Disable inf panels
            set(D.UI.panSesInf, 'HighlightColor', D.UI.disabledBckCol)
            set(D.UI.panPerfInf, 'HighlightColor', D.UI.disabledBckCol)
            set(D.UI.panTimInf, 'HighlightColor', D.UI.disabledBckCol)
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnRecDone', get(D.UI.btnRecDone,'Value')));
            Update_UI(10);
        end
        
        % SAVE SESSION DATA
        function [] = BtnSave(~, ~, ~)
            
            % Disable save button so only pressed once
            set(D.UI.btnSave, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            
            % Set flag to save at end of main loop
            D.F.do_save = true;
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnSave', get(D.UI.btnSave,'Value')));
            Update_UI(10);
        end
        
        % QUIT ALL
        function [] = BtnQuit(~, ~, ~)
            
            % Check for save unless quit before setup done
            if ~D.UI.save_done && D.F.data_loaded
                
                % Construct a questdlg with two options
                choice = dlgAWL('!!WARNING: QUIT WITHOUT SAVING?!!', ...
                    'ABBORT RUN', ...
                    'Yes', 'No', [], 'No', ...
                    D.UI.qstDlfPos, ...
                    'Warn');
                
                % Handle response
                switch choice
                    case 'Yes'
                    case 'No'
                        return
                end
                
                % Make sure rat is out
                if D.F.rat_in
                    dlgAWL(...
                        '!!WARNING: TAKE OUT RAT BEFORE PRECEDING!!', ...
                        'RAT OUT', ...
                        'OK', [], [], 'OK', ...
                        D.UI.qstDlfPos, ...
                        'Warn');
                end
                
            end
            
            % Print session aborting
            if ~D.UI.save_done
                Console_Write('**WARNING** [BtnQuit] ABORTING SESSION...');
            end
            
            % Disable
            % quit button
            set(D.UI.btnQuit, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            
            % Stop halt if still active
            if (D.F.is_halted)
                set(D.UI.btnHaltRob, 'Value', 0);
                BtnHaltRob();
            end
            
            % Stop bulldoze if still active
            if (get(D.UI.btnBulldoze, 'Value') == 1)
                set(D.UI.btnBulldoze, 'Value', 0);
                Bulldoze();
            end
            
            % Tell CS rat is out
            if D.F.rat_in
                SendM2C('I', 0)
                D.F.rat_in = false;
            end
            
            % Stop recording
            if D.F.rec
                set(D.UI.btnRec,'Value', 0)
                BtnRec(D.UI.btnRec);
            end
            
            % Check if battery should be replaced
            if ...
                    c2m.('J').dat1 > 0 && ...
                    c2m.('J').dat1 <= D.PAR.batVoltReplace
                
                % Confirm that Cheetah is closed
                dlgAWL('!!WARNING!! REPLACE BATTERY BEFORE NEXT RUN', ...
                    'BATTERY LOW', ...
                    'OK', [], [], 'OK', ...
                    D.UI.qstDlfPos, ...
                    'Warn');
            end
            
            % Tell C# to begin quit
            SendM2C('X');
            
            % Set flag
            D.F.quit = true;
            
            % Shut down if matlab run alone
            if isMatSolo
                doExit = true;
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnQuit', get(D.UI.btnQuit,'Value')));
            Update_UI(10);
            
        end
        
        % HALT ROBOT
        function [] = BtnHaltRob(~, ~, ~)
            if (get(D.UI.btnHaltRob, 'Value') == 1)
                % Change backround color and text
                set(D.UI.btnHaltRob, ...
                    'String', 'Halted...', ...
                    'BackgroundColor', D.UI.activeCol);
                % Tell CS to Halt Robot
                SendM2C('H', 1);
                % Set flag
                D.F.is_halted = true;
            else
                % Change backround color and text
                set(D.UI.btnHaltRob, ...
                    'String', 'Halt Robot', ...
                    'BackgroundColor', D.UI.enabledCol);
                % Tell CS to stop halting
                SendM2C('H', 0);
                % Set flag
                D.F.is_halted = false;
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnHaltRob', get(D.UI.btnHaltRob,'Value')));
            Update_UI(10);
        end
        
        % BULLDOZE RAT
        function [] = Bulldoze(~, ~, ~)
            
            if (get(D.UI.btnBulldoze, 'Value') == 1)
                
                % Get bulldoze delay
                delStr = D.UI.bullList(get(D.UI.popBulldoze, 'Value'));
                delStr = regexp(delStr, '\d*', 'match');
                D.PAR.bullDel = str2double(delStr{:}{:});
                D.PAR.bullSpeed = str2double(get(D.UI.editBulldoze, 'String'));
                
                % Change backround color and text
                set(D.UI.btnBulldoze, ...
                    'String', sprintf('Bulldoze (%ds)', D.PAR.bullDel), ...
                    'BackgroundColor', D.UI.activeCol);
                
                % Tell CS to bulldoze
                SendM2C('B', D.PAR.bullDel, D.PAR.bullSpeed);
                
            else
                
                % Change backround color and text
                set(D.UI.btnBulldoze, ...
                    'String', 'Ceasefire', ...
                    'BackgroundColor', D.UI.enabledCol);
                
                % Tell CS to stop bulldozing
                SendM2C('B', 0, 0);
                
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\" \"%d\"', 'PopBulldoze', D.PAR.bullDel, D.PAR.bullSpeed));
            Update_UI(10);
        end
        
        % REWARD
        function [] = BtnReward(~, ~, ~)
            
            % Set reward pos and cond
            r_pos = 0;
            r_cond = 1;
            
            % Get reward zone/duration from dropdown handle
            z_ind = get(D.UI.popReward, 'Value');
            
            % Tell CS to trigger reward
            SendM2C('R', r_pos, r_cond, z_ind);
            
            %             % TEMP
            %             set(D.UI.btnBulldoze, ...
            %                 'Value', ~get(D.UI.btnBulldoze, 'Value'));
            %             Bulldoze();
            
            % Track round trip time
            D.T.manual_rew_sent = Elapsed_Seconds(now);
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnReward', get(D.UI.btnReward,'Value')));
            Update_UI(10);
        end
        
        % BLOCK CUE
        function [] = BlockCue(~, ~, ~)
            
            % Dont run if reward sent
            if get(D.UI.btnBlockCue, 'UserData') == 1
                
                if (get(D.UI.btnBlockCue, 'Value') == 1)
                    
                    % Change backround color and text
                    set(D.UI.btnBlockCue, ...
                        'BackgroundColor', D.UI.activeCol);
                    
                    % Set main flag
                    D.F.do_block_cue = true;
                    
                    % Set cue maker to not visible
                    set(reshape(D.UI.ptchFdZineH,1,[]), ...
                        'EdgeColor', [0, 0, 0]);
                    
                    % Set flag to false
                    D.F.is_cued_rew = false;
                    
                    % Make sure other button is inactivated
                    if  D.F.do_all_cue
                        set(D.UI.btnAllCue, 'Value',0);
                        AllCue();
                    end
                else
                    % Change backround color and text
                    set(D.UI.btnBlockCue, ...
                        'BackgroundColor', D.UI.enabledCol);
                    
                    % Unset main flag
                    D.F.do_block_cue = false;
                end
                
            else
                % Set back to what previous state
                set(D.UI.btnBlockCue, 'Value', ~get(D.UI.btnBlockCue, 'Value'))
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnBlockCue', get(D.UI.btnBlockCue,'Value')));
            Update_UI(10);
        end
        
        % ALL CUE
        function [] = AllCue(~, ~, ~)
            
            % Dont run if reward sent
            if get(D.UI.btnAllCue, 'UserData') == 1
                
                if (get(D.UI.btnAllCue, 'Value') == 1)
                    % Change backround color and text
                    set(D.UI.btnAllCue, ...
                        'BackgroundColor', D.UI.activeCol);
                    
                    % Set main flag
                    D.F.do_all_cue = true;
                    
                    % Set flag to true
                    D.F.is_cued_rew = true;
                    
                    % Make sure other button is inactivated
                    if D.F.do_block_cue
                        set(D.UI.btnBlockCue, 'Value',0);
                        BlockCue();
                    end
                    
                else
                    % Change backround color and text
                    set(D.UI.btnAllCue, ...
                        'BackgroundColor', D.UI.enabledCol);
                    
                    % Unset flag
                    D.F.do_all_cue = false;
                end
                
            else
                % Set back to what previous state
                set(D.UI.btnAllCue, 'Value', ~get(D.UI.btnAllCue, 'Value'))
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnAllCue', get(D.UI.btnAllCue,'Value')));
            Update_UI(10);
        end
        
        % CLEAR VT BUTTON
        function [] = BtnClrVT(~, ~, ~)
            
            % Delete all handle objects
            delete(D.UI.ratPltH(isgraphics(D.UI.ratPltH)))
            delete(D.UI.ratPltHvel(isgraphics(D.UI.ratPltHvel)))
            delete(D.UI.robPltHvel(isgraphics(D.UI.robPltHvel)))
            D.UI.cnt_ratPltH = 0;
            D.UI.cnt_ratPltHvel = 0;
            D.UI.cnt_robPltHvel = 0;
            delete(D.UI.Rat.pltHposAll);
            delete(D.UI.Rat.pltHvelAll);
            delete(D.UI.Rob.pltHvelAll);
            delete(D.UI.Rat.pltHvelAvg);
            delete(D.UI.Rob.pltHvelAvg);
            % Reset to nan
            D.P.Rat.pos_hist = NaN(120*60*33,2);
            D.P.Rat.vel_hist = NaN(500,101,2);
            D.P.Rob.vel_hist = NaN(500,101,2);
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnClrVT', get(D.UI.btnClrVT,'Value')));
            Update_UI(10);
        end
        
        % CHECK DEFAULTS
        function [] = CheckSetupDefaults()
            
            % Local vars
            rew_del = D.UI.delList{get(D.UI.popRewDel,'Value'),:};
            cue_cond = D.PAR.catCueCond(logical(cell2mat(get(D.UI.toggCue, 'Value'))));
            task_cond = D.UI.taskList{get(D.UI.popTask, 'Value')};
            is_del_changed = false;
            is_cue_changed = false;
            is_task_changed = false;
            is_sound_changed = false;
            
            % Bail on recursive call
            stack = dbstack;
            if contains(stack(3).name, 'CheckSetupDefaults')
                return
            end
            
            % Set empty values
            if  isempty(cue_cond)
                cue_cond = 'Null';
            end
            
            % CHANGE VALUES BASED ON CUE COND
            if (D.PAR.cueFeed == 'Half' || ...
                    D.PAR.cueFeed == 'None') && ...
                    D.PAR.sesCond ~= 'Manual_Training' && ...
                    strcmp(rew_del, D.UI.delList{2})
                
                % Set to minimum delay
                set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.delList, D.UI.delList(3))));
                PopRewDel();
                
                % Set flag
                is_del_changed = true;
                
            elseif D.PAR.cueFeed == 'All' && ...
                    ~strcmp(rew_del, D.UI.delList{2})
                
                % Set to no delay
                set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.delList, D.UI.delList(2))));
                PopRewDel();
                
                % Set flag
                is_del_changed = true;
            end
            
            % CHANGE VALUES BASED ON SES COND
            if D.PAR.sesCond == 'Manual_Training'
                
                % Set to no delay
                if ~strcmp(rew_del, D.UI.delList{2})
                    
                    % Change button
                    set(D.UI.popRewDel, 'Value', ...
                        find(ismember(D.UI.delList, D.UI.delList(2))));
                    PopRewDel();
                    
                    % Set flag
                    is_del_changed = true;
                end
                
                % Set cue condition to 'None'
                if ~strcmp(cue_cond, 'None')
                    
                    % Change buttons
                    set(D.UI.toggCue( ...
                        ismember(D.PAR.catCueCond, 'None')), 'Value', 1);
                    set(D.UI.toggCue( ...
                        ~ismember(D.PAR.catCueCond, 'None')), 'Value', 0);
                    ToggCue();
                    
                    % Set flag
                    is_cue_changed = true;
                end
                
                % Change other buttons for rot cond
            elseif D.PAR.sesCond == 'Rotation'
                
                % Set session task to 'Track'
                if ~strcmp(task_cond, 'Track')
                    
                    % Change button
                    set(D.UI.popTask, 'Value', ...
                        find(ismember(D.UI.taskList, 'Track')));
                    PopTask();
                    
                    % Set flag
                    is_task_changed = true;
                end
                
                % Set reward delay to max
                if ~strcmp(rew_del, D.UI.delList{end})
                    
                    % Change button
                    set(D.UI.popRewDel, 'Value', ...
                        find(ismember(D.UI.delList, D.UI.delList(end))));
                    PopRewDel();
                    
                    % Set flag
                    is_del_changed = true;
                end
                
                % Set cue condition to 'None'
                if ~strcmp(cue_cond, 'None')
                    
                    % Change buttons
                    set(D.UI.toggCue( ...
                        ismember(D.PAR.catCueCond, 'None')), 'Value', 1);
                    set(D.UI.toggCue( ...
                        ~ismember(D.PAR.catCueCond, 'None')), 'Value', 0);
                    ToggCue();
                    
                    % Set flag
                    is_cue_changed = true;
                end
                
                % Set sound condition
                if ~all(D.UI.snd(1:2))
                    
                    % Change buttons
                    set(D.UI.toggSnd(), 'Value', 1);
                    ToggSnd();
                    
                    % Set flag
                    is_sound_changed = true;
                end
                
            end
            
            % Log/print
            if (is_del_changed)
                Console_Write('[CheckSetupDefaults] Changed Reward Delay');
                Update_UI(10);
            end
            if (is_cue_changed)
                Console_Write('[CheckSetupDefaults] Changed Cue Condition');
                Update_UI(10);
            end
            if (is_task_changed)
                Console_Write('[CheckSetupDefaults] Changed Task Condition');
                Update_UI(10);
            end
            if (is_sound_changed)
                Console_Write('[CheckSetupDefaults] Changed Sound Condition');
                Update_UI(10);
            end
            
        end
        
        
        
        
        
        
        %% ========================== MINOR FUNCTIONS =====================
        
        % -----------------------CONVERT VT POS TO RAD POS-----------------
        function [rad, roh] = VT_2_Rad(xy_pos)
            
            % Save x/y pos samples in seperate vars
            x = xy_pos(:,1);
            y = xy_pos(:,2);
            
            % Rescale y as VT data is compressed in y axis
            y = y*11/10;
            
            % Get normalized pos data
            x_norm = (x-D.PAR.XC)./D.PAR.R;
            y_norm = (y-D.PAR.YC)./D.PAR.R;
            
            % Get position in radians
            [rad,roh] = cart2pol(x_norm, y_norm);
            
            % Convert radians between [0, 2*pi]
            rad = wrapTo2Pi(rad);
            
            % Flip radian values to acount for inverted y values from Cheetah
            rad = abs(rad - 2*pi);
        end
        
        % -----------------------CONVERT RAD POS TO VT POS-----------------
        function [xy_pos] = Rad_2_VT(rad, roh)
            
            % Flip values for inverted vt y
            rad = wrapTo2Pi(rad);
            rad = abs(rad - 2*pi);
            rad = wrapToPi(rad);
            
            % Convert to cart
            [x_norm,y_norm] = pol2cart(rad, roh);
            
            % Translate and scale to pixel space
            x = (x_norm.*D.PAR.R) + D.PAR.XC;
            y = (y_norm.*D.PAR.R) + D.PAR.YC;
            
            % Rescale y value
            y = y*(10/11);
            
            % Store values
            xy_pos(:,1) = x;
            xy_pos(:,2) = y;
        end
        
        % ---------------------------COMPUTE RAD DIFF----------------------
        function [rad_diff] = Rad_Diff(rad1, rad2)
            rad_diff = rad1-rad2;
            if rad_diff < 0
                rad_diff = rad_diff + 2*pi;
            end
        end
        
        % ---------------------------COMPUTE RAD SUM-----------------------
        function [rad_sum] = Rad_Sum(rad1, rad2)
            rad_sum = rad1+rad2;
            if rad_sum > 2*pi
                rad_sum = rad_sum - 2*pi;
            end
        end
        
        % ---------------------------GET TRACK BOUNDS----------------------
        function [xbnd, ybnd] = Get_Rad_Bnds(polbnds)
            
            % Set roh lims
            if D.PAR.sesTask == 'Track'
                inRohLim = D.UI.arnRad-D.UI.trkWdt;
                outRohLim = D.UI.arnRad;
            else
                inRohLim = D.UI.rfRad-D.UI.rfTargWdt;
                outRohLim = D.UI.rfRad;
            end
            
            % Convert to rad array
            if (length(polbnds) == 2)
                if polbnds(1) > polbnds(2)
                    polbnds = wrapToPi(polbnds);
                end
                radDist = min(2*pi - abs(polbnds(2)-polbnds(1)), abs(polbnds(2)-polbnds(1)));
                nPoints =  round(360 * (radDist/(2*pi))); % 360 pnts per 2*pi
                polbnds = linspace(polbnds(1), polbnds(2), nPoints);
            end
            
            % Compute inner bounds 0 deg
            [x(1,:),y(1,:)] = pol2cart(polbnds, ones(1,length(polbnds)) * inRohLim);
            xbnd(1,:) = x(1,:)*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
            ybnd(1,:) = y(1,:)*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
            
            % Compute outer bounds 0 deg
            [x(2,:),y(2,:)] = pol2cart(polbnds, ones(1,length(polbnds)) * outRohLim);
            xbnd(2,:) = x(2,:)*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
            ybnd(2,:) = y(2,:)*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
            
        end
        
        % ---------------------------CHECK POS BOUNDS----------------------
        function [bool_arr] = Check_Pol_Bnds(rad_arr, roh_arr, rad_bnds, roh_bnds)
            
            % Handle defaults
            if nargin<4
                if D.PAR.sesTask == 'Track'
                    roh_bnds =  D.P.trackRohBnd;
                else
                    roh_bnds =  D.P.rfRohBnd;
                end
            end
            
            % Get logical array of inbound value inds
            if all(isnan(rad_arr))
                bool_arr = false(size(rad_arr));
            else
                if rad_bnds(1) > rad_bnds(2)
                    rad_bnds = wrapToPi(rad_bnds);
                    rad_arr = wrapToPi(rad_arr);
                end
                bool_arr = ...
                    (rad_arr > rad_bnds(1) & rad_arr < rad_bnds(2)) & ...
                    (roh_arr > roh_bnds(1) & roh_arr < roh_bnds(2));
            end
            
        end
        
        % ---------------------------GET BUTTON COLLOR---------------------
        function [C] = Btn_Col()
            
            [X,Y] = meshgrid(linspace(-(2*pi),(2*pi),25));
            R = sqrt(X.^2 + Y.^2);
            Z = sin(R)./R;
            mask = R > (2*pi)*0.75;
            %Z = abs(R-max(max(R)));
            
            
            % Hi = Z + abs(min(min(Z)));
            % Hi = Hi/max(max(Hi));
            % Hi(1,:) = 0.1;
            % Hi(end,:) = 0.1;
            % Hi(:,1) = 0.1;
            % Hi(:,end) = 0.1;
            % Lo = abs(Hi - 1);
            
            % Scale so edges are equal to backround color
            Hi = Z + abs(min(min(Z)));
            Hi = Hi/(max(max(Hi))*1);
            Hi(Hi>1) = 1;
            Lo = abs(Hi -1);
            D.UI.scl = (1 - D.UI.figBckCol(1)) / (1 - Hi(1,1));
            Hi = 1 - D.UI.scl*(1 - Hi);
            Hi(mask) = D.UI.figBckCol(1);
            Lo = Lo * (D.UI.figBckCol(1)/Lo(1,1));
            D.UI.scl = (1 - D.UI.figBckCol(1)) / (1 - Lo(1,1));
            Lo = 1 - D.UI.scl*(1 - Lo);
            Lo(mask) = D.UI.figBckCol(1);
            % remove NaN and set max to 1
            Hi(isnan(Hi) | Hi>1) = 1;
            Lo(isnan(Lo)) = 0;
            Lo(Lo>1) = 1;
            Hi(Hi<0) = 0;
            Lo(Lo<0) = 0;
            
            % Gray
            C{1}(:,:,1) = Hi;
            C{1}(:,:,2) = Hi;
            C{1}(:,:,3) = Hi;
            
            % Green
            C{2}(:,:,1) = Lo;
            C{2}(:,:,2) = Hi;
            C{2}(:,:,3) = Lo;
            
            % Red
            C{3}(:,:,1) = Hi;
            C{3}(:,:,2) = Lo;
            C{3}(:,:,3) = Lo;
            
            C = C';
        end
        
        % --------------------ENABLE DISABLE CUE BUTTONS-------------------
        function [] = Set_Cue_Buttons(setting)
            
            if D.PAR.cueFeed == 'Half' || D.PAR.cueFeed == 'None'
                
                % Set to enabled color and change user data
                if strcmp(setting, 'Enable')
                    
                    % Set user data
                    set(D.UI.btnBlockCue, 'UserData', 1);
                    set(D.UI.btnAllCue,  'UserData', 1);
                    
                    % Set button color
                    if get(D.UI.btnBlockCue, 'Value') == 1
                        set(D.UI.btnBlockCue,  'BackgroundColor', D.UI.activeCol);
                    else
                        set(D.UI.btnBlockCue, 'BackgroundColor', D.UI.enabledCol);
                    end
                    if get(D.UI.btnAllCue, 'Value') == 1
                        set(D.UI.btnAllCue,  'BackgroundColor', D.UI.activeCol);
                    else
                        set(D.UI.btnAllCue, 'BackgroundColor', D.UI.enabledCol);
                    end
                    
                    % Set to disabled color
                elseif strcmp(setting, 'Disable')
                    
                    % Set user data
                    set(D.UI.btnBlockCue, 'UserData', 0);
                    set(D.UI.btnAllCue,  'UserData', 0);
                    
                    % Set button color
                    set(D.UI.btnBlockCue, 'BackgroundColor',  D.UI.disabledBckCol);
                    set(D.UI.btnAllCue, 'BackgroundColor',  D.UI.disabledBckCol);
                    
                end
                
                % Log/print
                Console_Write(sprintf('[Cue_Buttons] Set Cue Buttons to \"%s\"', setting));
            end
        end
        
        
        
        
        
        
        %% ======================= SAVE SESSION DATA ======================
        function [] =  Save_Ses_Data()
            
            % Log/print
            Console_Write('[Save_Ses_Data] RUNNING: Save Session Data...');
            
            %% Disconnect from NetCom
            
            Disconnect_NLX()
            
            % Confirm that Cheetah is closed
            dlgAWL('Close Cheetah', ...
                'CLOSE CHEETAH', ...
                'OK', [], [], 'OK', ...
                D.UI.qstDlfPos, ...
                'Dflt');
            
            %% Change NLX recording directory
            
            % Save directory var
            % format: datestr(now, 'yyyy-mm-dd_HH-MM-SS', 'local');
            D.DIR.nlxSaveRat = fullfile(D.DIR.nlxSaveTop, D.PAR.ratLab(2:end));
            
            % Get current Cheetah recording dir
            dirs = dir(D.DIR.nlxTempTop);
            dirs = dirs(3:end);
            fi_dat_num = ...
                cell2mat(cellfun(@(x) datenum(x, 'yyyy-mm-dd_HH-MM-SS'), {dirs.name}, 'uni', false));
            D.DIR.recFi = dirs(fi_dat_num == max(fi_dat_num)).name;
            
            % Save to global for CS
            m2c_dir = fullfile(D.DIR.nlxSaveRat, D.DIR.recFi);
            
            % Make directory if none exists
            if exist(D.DIR.nlxSaveRat, 'dir') == 0
                mkdir(D.DIR.nlxSaveRat);
            end
            
            % Save GUI window image
            export_fig(FigH, fullfile(D.DIR.nlxTempTop, D.DIR.recFi, 'GUI.jpg'));
            
            %% Save Table Data
            
            % Get row ind
            % determine if this is first entry
            if strcmp(D.SS_Out_ICR.(D.PAR.ratLab).Date{1}, '');
                % is first entry
                rowInd = 1;
            else
                rowInd = size(D.SS_Out_ICR.(D.PAR.ratLab), 1) + 1;
                % add new row at end of table
                D.SS_Out_ICR.(D.PAR.ratLab) = ...
                    [D.SS_Out_ICR.(D.PAR.ratLab); D.SS_Out_ICR.(D.PAR.ratLab)(end,:)];
            end
            
            % Add entries
            D.SS_Out_ICR.(D.PAR.ratLab).Include_Analysis(rowInd) = true;
            D.SS_Out_ICR.(D.PAR.ratLab).Date{rowInd} = D.DIR.recFi;
            D.SS_Out_ICR.(D.PAR.ratLab).Start_Time{rowInd} = datestr(startTime, 'HH:MM:SS');
            D.SS_Out_ICR.(D.PAR.ratLab).Total_Time(rowInd) = (D.T.ses_end_tim - D.T.ses_str_tim) / 60;
            D.SS_Out_ICR.(D.PAR.ratLab).Session_Condition(rowInd) = char(D.PAR.sesCond);
            D.SS_Out_ICR.(D.PAR.ratLab).Session_Task(rowInd) = char(D.PAR.sesTask);
            % save session number of this condition
            var_ind = ...
                ismember(D.SS_Out_ICR.(D.PAR.ratLab).Properties.VariableNames, ['Session_',char(D.PAR.sesCond)]);
            col_ind = ismember([{'Track'},{'Forage'}], D.PAR.sesTask);
            D.SS_Out_ICR.(D.PAR.ratLab){rowInd, var_ind}(col_ind) = D.PAR.sesNum;
            % save other entries
            D.SS_Out_ICR.(D.PAR.ratLab).Feeder_Condition(rowInd) = D.PAR.ratFeedCnd;
            D.SS_Out_ICR.(D.PAR.ratLab).Rotation_Direction(rowInd) = D.PAR.ratRotDrc;
            D.SS_Out_ICR.(D.PAR.ratLab).Reward_Delay(rowInd) = sprintf('%1.1f', D.PAR.rewDel);
            D.SS_Out_ICR.(D.PAR.ratLab).Cue_Condition(rowInd) = char(D.PAR.cueFeed);
            D.SS_Out_ICR.(D.PAR.ratLab).Pulse_Duration(rowInd) = D.PAR.rewDur;
            D.SS_Out_ICR.(D.PAR.ratLab).Sound_Conditions(rowInd,:) = D.UI.snd;
            D.SS_Out_ICR.(D.PAR.ratLab).Start_Quadrant(rowInd) = D.PAR.ratStrQuad;
            D.SS_Out_ICR.(D.PAR.ratLab).Bulldozings(rowInd) = D.C.bull_cnt;
            D.SS_Out_ICR.(D.PAR.ratLab).Zones_Rewarded{rowInd} = ...
                D.I.zoneHist(1:find(~isnan(D.I.zoneHist),1,'last'));
            D.SS_Out_ICR.(D.PAR.ratLab).Rewards_Standard{rowInd} = sum(D.C.rew_cnt{3});
            D.SS_Out_ICR.(D.PAR.ratLab).Laps_Standard{rowInd} = sum(D.C.lap_cnt{3});
            D.SS_Out_ICR.(D.PAR.ratLab).Notes{rowInd} = '';
            
            % Rotation session specific vars
            if D.PAR.sesCond == 'Rotation'
                D.SS_Out_ICR.(D.PAR.ratLab).Rotations_Per_Session(rowInd) = D.C.rot_cnt;
                D.SS_Out_ICR.(D.PAR.ratLab).Rotation_Positions{rowInd} = ...
                    cell2mat(cellfun(@(x) str2double(x), cellstr(D.PAR.rotPos(1:D.C.rot_cnt)), 'uni', false))';
                if length(D.C.lap_cnt{1}) < length(D.C.lap_cnt{2});
                    D.C.lap_cnt{1} = [D.C.lap_cnt{1},NaN];
                elseif length(D.C.lap_cnt{1}) > length(D.C.lap_cnt{2});
                    D.C.lap_cnt{2} = [D.C.lap_cnt{2},NaN];
                end
                D.SS_Out_ICR.(D.PAR.ratLab).Laps_Per_Rotation{rowInd} = ...
                    [D.C.lap_cnt{3}, reshape([[D.C.lap_cnt{2}]; [D.C.lap_cnt{1}]], 1, [])];
                D.SS_Out_ICR.(D.PAR.ratLab).Days_Till_Rotation(rowInd) = D.PAR.daysTilRot;
                D.SS_Out_ICR.(D.PAR.ratLab).Rewards_40_Deg{rowInd} = D.C.rew_cnt{2};
                D.SS_Out_ICR.(D.PAR.ratLab).Laps_40_Deg{rowInd} = D.C.lap_cnt{2};
                D.SS_Out_ICR.(D.PAR.ratLab).Rewards_0_Deg{rowInd} = D.C.rew_cnt{1};
                D.SS_Out_ICR.(D.PAR.ratLab).Laps_0_Deg{rowInd} = D.C.lap_cnt{1};
            else
                D.SS_Out_ICR.(D.PAR.ratLab).Rotations_Per_Session(rowInd) = NaN;
                D.SS_Out_ICR.(D.PAR.ratLab).Rotation_Positions{rowInd} = {[]};
                D.SS_Out_ICR.(D.PAR.ratLab).Laps_Per_Rotation(rowInd) = {[]};
                D.SS_Out_ICR.(D.PAR.ratLab).Days_Till_Rotation(rowInd) = '<undefined>';
                D.SS_Out_ICR.(D.PAR.ratLab).Rewards_40_Deg{rowInd} = 0;
                D.SS_Out_ICR.(D.PAR.ratLab).Rewards_0_Deg{rowInd} = 0;
                D.SS_Out_ICR.(D.PAR.ratLab).Laps_40_Deg{rowInd} = 0;
                D.SS_Out_ICR.(D.PAR.ratLab).Laps_0_Deg{rowInd} = 0;
            end
            
            % Update SS_In_All
            D.SS_In_All.Session_Condition(D.PAR.ratInd) = char(D.PAR.sesCond);
            D.SS_In_All.Reward_Delay(D.PAR.ratInd) = sprintf('%1.1f', D.PAR.rewDel);
            D.SS_In_All.Cue_Condition(D.PAR.ratInd) = char(D.PAR.cueFeed);
            D.SS_In_All.Pulse_Duration(D.PAR.ratInd) = D.PAR.rewDur;
            D.SS_In_All.Sound_Conditions(D.PAR.ratInd,:) = D.UI.snd;
            
            % Save out data
            SS_Out_ICR = D.SS_Out_ICR; %#ok<NASGU>
            save(D.DIR.ioSS_Out_ICR, 'SS_Out_ICR');
            SS_In_All = D.SS_In_All; %#ok<NASGU>
            save(D.DIR.ioSS_In_All, 'SS_In_All');
            
            
            % Print saved table data
            Console_Write('[Save_Ses_Data] FINISHED: Save Data Tables');
            
            %% Copy Cheetah file to rat directory
            
            % Get file size
            fiGigs = dir(fullfile(D.DIR.nlxTempTop, D.DIR.recFi));
            fiGigs = sum([fiGigs.bytes])/10^9;
            
            % Wait for Cheetah to fully close
            pause(3);
            
            % Log/print start
            Console_Write(sprintf('[Save_Ses_Data] RUNNING: Copy Cheetah File...: File=%s Size=%0.2fGB', ...
                D.DIR.recFi, fiGigs));
            
            copyfile(fullfile(D.DIR.nlxTempTop, D.DIR.recFi),fullfile(D.DIR.nlxSaveRat, D.DIR.recFi))
            
            % Log/print end
            Console_Write(sprintf('[Save_Ses_Data] FINISHED: Copy Cheetah File: File=%s Size=%0.2fGB', ...
                D.DIR.recFi, fiGigs));
            
            % Set flags
            D.UI.save_done = true;
            
            % Tell CS Matlab session saved
            SendM2C('F');
            
            % Highlight Quit button
            set(D.UI.btnQuit, ...
                'ForegroundColor' , D.UI.enabledBtnFrgCol, ...
                'BackgroundColor', D.UI.activeCol);
            
            % Print save completed
            Console_Write('[Save_Ses_Data] FINISHED: Save Cheetah Data');
            
        end
        
        
        
        
        
        
    end

% -----------------------------MAIN EXIT-----------------------------------
    function[] = Exit()
        % Start exiting
        Console_Write('[ICR_GUI] RUNNING: Exit ICR_GUI...');
        
        % Check if GUI was forced close
        
        if ~D.DB.isForceClose
            
            % Log/print
            Console_Write('[ICR_GUI] RUNNING: Normal Exit Procedure...');
            
            % Pause then shut it all down
            pause(1);
            % Disconnect AC computer
            Disconnect_AC();
            % Disconnect from NetCom
            Disconnect_NLX();
            
        end
        
        % Save log
        Console_Write('[ICR_GUI] RUNNING: Save ICR_GUI Log...');
        if size(who('global'),1) > 0 && ...
                exist(D.DIR.logTemp, 'dir')
            fi_path = D.DIR.logTemp;
            fid = fopen(fi_path,'wt');
            for z_l = 1:D.DB.logCount
                fprintf(fid, D.DB.logStr{z_l});
            end
            fclose(fid);
            Console_Write(sprintf('[ICR_GUI] FINISHED: Save ICR_GUI Log to \"%s\"', D.DIR.logTemp));
        else
            Console_Write('!!ERROR!! [ICR_GUI] FAILED: Save ICR_GUI Log');
        end
        
        % Close figure
        if ~D.DB.isForceClose
            D.F.close = true;
            close(FigH)
            delete(FigH)
        end
        
        % Confirm GUI closed
        SendM2C('C');
        
        % Wait for recieved confirmation
        Console_Write('[ICR_GUI] RUNNING: Wait for GUI Closed Confirm...');
        while exist('c2m', 'var') ~= 0
            if c2m.('C').dat1 == 1
                break;
            end
            pause(0.01);
        end
        Console_Write('[ICR_GUI] FINISHED: Wait for GUI Closed Confirm');
        
        % Stop and delete timer
        stop(timer_c2m);
        delete(timer_c2m);
    end






%% =================== TOP LEVEL SUPPORT FUNCTIONS ========================

% --------------------------SEND DATA TO CS--------------------------------
    function[] = SendM2C(id, dat1, dat2, dat3, pack)
        
        % Bail if isMatSolo
        if isMatSolo
            return
        end
        
        % Set mesage data
        if nargin == 3
            dat3 = -1;
        end
        if nargin == 2
            dat2 = -1;
            dat3 = -1;
        end
        if nargin == 1
            dat1 = -1;
            dat2 = -1;
            dat3 = -1;
        end
        
        % Wait a max of 1 sec for last message to clear
        t_start = Elapsed_Seconds(now);
        while m2c_pack(6) ~= 0 && ...
                Elapsed_Seconds(now) < t_start + 1
            pause(0.001);
        end
        if m2c_pack(6) ~= 0
            m2c_pack(6) = 0;
            Console_Write(sprintf('**WARNING** [SendM2C] CS Failed to Reset m2c Packet: id=''%s'' dat1=%2.2f dat2=%2.2f dat3=%2.2f dt=%dms', ...
                id, dat1, dat2, dat3, round((Elapsed_Seconds(now) - t_start)*1000)));
        end
        
        % Set mesage ID and dat
        m2c_pack(1) = unicode2native(id,'UTF-8');
        m2c_pack(2) = dat1;
        m2c_pack(3) = dat2;
        m2c_pack(4) = dat3;
        
        % Get new packet number
        if nargin < 5
            m2c.cnt_pack = m2c.cnt_pack+1;
            m2c.(id).packLast = m2c.(id).pack;
            m2c.(id).pack = m2c.cnt_pack;
            m2c_pack(5) = m2c.cnt_pack;
        else
            % Use packet input
            m2c_pack(5) = pack;
        end
        
        % Save dat to struct
        m2c.(id).dat1 = dat1;
        m2c.(id).dat2 = dat2;
        m2c.(id).dat3 = dat3;
        m2c.(id).t_send = Elapsed_Seconds(now);
        
        % Flag new data
        m2c_pack(6) = 1;
        
        % Log/print
        Console_Write(sprintf('   [SENT] m2c: id=''%s'' dat1=%2.2f dat2=%2.2f dat3=%2.2f pack=%d', ...
            id, dat1 ,dat2, dat3, m2c_pack(5)));
        
    end

% ---------------------------CHECK FOR NEW C2M-----------------------------
    function [] = CheckC2M(~,~)
        
        % Have to explicitely catch errors
        try
            
            % Bail if D deleted
            if ~exist('D', 'var')
                return
            end
            
            % Check for new packet
            c2m_mat = reshape(cell2mat(struct2cell(c2m)),1,[]);
            new_ind = [c2m_mat.pack] ~= [c2m_mat.packLast];
            
            % Bail if no new packets
            if ~any(new_ind)
                return
            end
            
            % Store new id
            id = c2m_mat(new_ind).id;
            
            % Check for handshake flag
            if ...
                    strcmp(id, 'h') && ...
                    c2m.(id).dat1 == 1
                
                % Set start time
                startTime = now;
                
                % Log/print handshake received
                Console_Write(sprintf('   [RCVD] CS HANDSHAKE COMMAND: id=''%s'' dat1=%d', ...
                    id, c2m.(id).dat1), now);
            end
            
            % Check for exit flag
            if ...
                    strcmp(id, 'E') && ...
                    c2m.(id).dat1 == 1
                
                % Set exit flag
                doExit = true;
                
                % Log/print exit received
                Console_Write(sprintf('   [RCVD] CS EXIT COMMAND: id=''%s'' dat1=%d', ...
                    id, c2m.(id).dat1), now);
            end
            
            %Print new data
            if ~ischar(c2m.(id).dat1)
                str = '   [RCVD] c2m: id=''%s'' dat1=%d pack=%d';
            else
                str = '   [RCVD] c2m: id=''%s'' dat1=''%s'' pack=%d';
            end
            Console_Write(sprintf(str, ...
                c2m.(id).id, c2m.(id).dat1, c2m.(id).pack), now);
            
            % Update packet info
            c2m.(id).packLast = c2m.(id).pack;
            
            % Update recieve time
            c2m.(id).t_rcvd = Elapsed_Seconds(now);
            
            % Check for resend request
            if ~strcmp('g', c2m.(id).id)
                return
            end
            
            % Get id to resend
            id_send = c2m.(id).dat1;
            Console_Write(sprintf('   [RCVD] Send/Resend Requested: id=%s dat1=%s', ...
                id, id_send), now);
            
            % Send again if any history of this id
            if m2c.(id_send).pack > 0
                SendM2C( ...
                    m2c.(id_send).id, ...
                    m2c.(id_send).dat1, ...
                    m2c.(id_send).dat2, ...
                    m2c.(id_send).dat3, ...
                    m2c.(id_send).pack);
            end
            
        catch ME
            Console_Write('!!ERROR!! [CheckC2M] FAILED', now);
            doExit = true;
        end
        
    end

% --------------------------SEND DATA TO AC--------------------------------
    function[] = SendM2AC()
        
        % Bail if not connected
        if ~D.AC.connected
            return
        end
        
        % Send data
        fwrite(tcpIP,D.AC.data,'int8');
        
        % Log/print
        Console_Write(sprintf('   [SENT] m2ac: dat=|%d|%d|%d|', ...
            D.AC.data(1),D.AC.data(2),D.AC.data(3)));
    end

% ---------------------------PRINT TO CONSOLE------------------------------
    function [] = Console_Write(str, t_now)
        
        % Bail if following conditions not met
        if ~exist('D','var'); return; end
        if ~isfield(D, 'DB'); return; end
        if ~isfield(D.DB, 'consoleStr'); return; end
        
        % Get time
        if nargin < 2
            t_now = now;
        end
        
        % Itterate log count
        D.DB.logCount = D.DB.logCount+1;
        
        % Get time
        t_s = Elapsed_Seconds(t_now);
        t_m = round(t_s*1000);
        
        % Add to strings
        p_msg = sprintf('\r%0.2f %s', t_s, str);
        l_msg = sprintf('[%d],%d,%s\r', D.DB.logCount, t_m, str);
        
        % Insert new print string into existing string
        if (D.DB.logCount>1)
            D.DB.consoleStr(2:D.DB.logCount+1,:) = D.DB.consoleStr(1:D.DB.logCount,:);
        end
        D.DB.consoleStr(1,1:150) = ' ';
        D.DB.consoleStr(1,1:length(p_msg)) = p_msg;
        
        % Store log str
        D.DB.logStr{D.DB.logCount} = l_msg;
        
        % Write to Matlab console
        disp(p_msg);
        
        % Update console window
        if ~isfield(D, 'UI'); return; end
        if ~isfield(D.UI, 'listConsole'); return; end
        if ~isvalid(D.UI.listConsole); return; end
        % Print normal
        set(D.UI.listConsole, ...
            'String', D.DB.consoleStr);
        
        % Set to red bold for error
        if D.DB.isErrExit
            set(D.UI.listConsole, ...
                'ForegroundColor', [1 0 0], ...
                'FontWeight','Bold');
        end
        
        % Update GUI during setup
        if ~D.F.data_loaded
            drawnow;
        end
        
    end

% ---------------------------GET TIME NOW-------------------------------
    function [t_sec] = Elapsed_Seconds(t_now)
        
        % Convert from days to seconds
        t_sec = (t_now-startTime)*24*60*60;
        
    end

% ------------------------Disconnect From NetCom---------------------------
    function [] = Disconnect_NLX()
        
        % End NLX polling
        D.F.poll_nlx = false;
        
        if isfield(D, 'NLX')
            if ~isempty(D.NLX)
                if NlxAreWeConnected() == 1
                    
                    % Log/print
                    Console_Write('[Disconnect_NLX] RUNNING: Disconnect from NLX...');
                    
                    % Stop recording and aquisition
                    NlxSendCommand('-StopAcquisition');
                    NlxSendCommand('-StopRecording');
                    pause(0.1);
                    
                    % Close VT stream
                    if isfield(D.NLX, 'vt_rat_ent')
                        NlxCloseStream(D.NLX.vt_rat_ent);
                        NlxCloseStream(D.NLX.vt_rob_ent);
                    end
                    
                    % Close event stream
                    if isfield(D.NLX, 'event_ent')
                        NlxCloseStream(D.NLX.event_ent);
                    end
                    
                    % Disconnect from the NLX server
                    NlxDisconnectFromServer();
                    while NlxAreWeConnected() == 1 && ~doExit
                        NlxDisconnectFromServer();
                    end
                    % Show status disconnected
                    if NlxAreWeConnected() ~= 1
                        Console_Write(sprintf('[Disconnect_NLX] FINISHED: Disconnect from NLX IP=%s', ...
                            D.NLX.IP));
                    else
                        Console_Write(sprintf('!!ERROR!! [Disconnect_NLX] ABORTED: Disconnect from NLX IP=%s', ...
                            D.NLX.IP));
                    end
                    
                end
            end
            
        end
    end

% -----------------------------Disconnect AC-------------------------------
    function [] = Disconnect_AC()
        
        if exist('D','var')
            
            % Log/print
            Console_Write('[Disconnect_AC] RUNNING: Disconnect from AC Computer...');
            
            % Disconnect from AC computer
            if isfield(D, 'AC')
                if ~isempty(D.AC)
                    if exist('tcpIP', 'var')
                        if isa(tcpIP, 'tcpip')
                            if isvalid(tcpIP)
                                
                                % Pause to allow image to close
                                D.AC.data = zeros(1, length(D.AC.data));
                                D.AC.data(1) = 1;
                                SendM2AC();
                                pause(0.1);
                                
                                % Send command to terminate run
                                D.AC.data = zeros(1, length(D.AC.data));
                                SendM2AC();
                                
                                % Close AC computer connection
                                fclose(tcpIP);
                                
                                % Show status disconnected
                                Console_Write(sprintf('[Disconnect_AC] FINISHED: Disconnect from AC Computer IP=%s', ...
                                    D.AC.IP));
                                
                            else
                                Console_Write('**WARNING** [Disconnect_AC] \"tcpIP\" Does Not Exist');
                            end
                        else
                            Console_Write('**WARNING** [Disconnect_AC] \"tcpIP\" is Not a tcpIP Object');
                        end
                    else
                        Console_Write('**WARNING** [Disconnect_AC] \"tcpIP\" Does Not Exist');
                    end
                else
                    Console_Write('**WARNING** [Disconnect_AC] \"D.AC\" is Empty');
                end
            else
                Console_Write('**WARNING** [Disconnect_AC] \"AC\" Not a Field of \"D\"');
            end
            try fclose(tcpIP);
            catch; end
            try delete(tcpIP);
            catch; end
            try clear tcpIP;
            catch; end
            
        end
    end

% ------------------------------FORCE QUIT---------------------------------
    function [] = ForceClose(~, ~, ~)
        
        % Dont run if global vars already deleted
        if size(who('global'),1) == 0
            return
        end
        
        % Dont run if gui closed in correct sequence
        if D.F.close
            return
        end
        
        % Log/print
        Console_Write('**WARNING** [ForceClose] RUNNING: ForceClose Exit Procedure...');
        
        % Disconnect AC computer
        Disconnect_AC();
        % Disconnect from NetCom
        Disconnect_NLX()
        
        % Set flags
        doExit = true;
        D.DB.isForceClose = true;
        
        return
    end






end