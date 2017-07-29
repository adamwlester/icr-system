function[] = ICR_GUI(sysTest, doDebug, isMatSolo)
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
global FigH; % UI figure handle
global D; % Main data struct
global tcpIP; % TCP object

% Matlab to CS communication
global m2c_id; % message out to CS
global m2c_dat1; % data out to CS
global m2c_dat2; % data out to CS
global m2c_dat3; % data out to CS
global m2c_pack; % packet number out to CS
global m2c_hist; % message history
global m2c_dir; % current cheetah directory
global m2c_posSim; % array for sim data

% CS to Matlab communication
global request_m2c; % request m2c data
global timer_c2m; % timer to check c2m
global c2m_W; % sync time
global c2m_J; % battery voltage
global c2m_Z; % reward zone
global c2m_V; % robot streaming
global c2m_K; % robot in place
global c2m_Y; % enable save button
global c2m_E; % exit
global c2m_C; % confirm close

% m2c send history
m2c_hist = {...
    '+', 0, 0, 0, 0; ... % matlab ready [NA]
    'T', 0, 0, 0, 0; ... % system test command [(byte)test]
    'G', 0, 0, 0, 0; ... % matlab gui loaded [NA]
    'N', 0, 0, 0, 0; ... % netcom setup [NA]
    'A', 0, 0, 0, 0; ... % connected to AC computer [NA]
    'S', 0, 0, 0, 0; ... % setup session [(byte)ses_cond, (byte)sound_cond]
    'M', 0, 0, 0, 0; ... % move to position [(float)targ_pos]
    'R', 0, 0, 0, 0; ... % run reward [(float)rew_pos, (byte)zone_ind, (byte)rew_delay]
    'H', 0, 0, 0, 0; ... % halt movement [(byte)halt_state]
    'B', 0, 0, 0, 0; ... % bulldoze rat [(byte)bull_delay, (byte)bull_speed]
    'I', 0, 0, 0, 0; ... % rat in/out [(byte)in/out]
    'F', 0, 0, 0, 0; ... % data saved [NA]
    'X', 0, 0, 0, 0; ... % confirm quit
    'C', 0, 0, 0, 0; ... % confirm close
    };

% Set top level vars
D.DB.startTime = now;
D.DB.consoleStr = [repmat(' ',1000,150),repmat('\r',1000,1)];
D.DB.logStr = cell(1000,1);
D.DB.logCount = 0;
D.DB.isTestRun = false;
D.DB.doPidCalibrationTest = false;
D.DB.doHaultErrorTest = false;
D.DB.doSimRatTest = false;
D.DB.isErrExit = false;
D.DB.isForceClose = false;

% Handle input args
if nargin < 3
    isMatSolo = true;
end
if nargin < 2
    doDebug = false;
end
if nargin < 1
    sysTest = 0;
end

% Get testing condition
switch sysTest
    case 1
        D.DB.isTestRun = true;
        D.DB.doPidCalibrationTest = true;
    case 2
        D.DB.isTestRun = true;
        D.DB.doHaultErrorTest = true;
    case 3
        D.DB.isTestRun = true;
        D.DB.doSimRatTest = true;
end


% Bypass some things if running solo
if isMatSolo
    c2m_V = 1;
    c2m_K = 1;
    c2m_Y = 1;
    c2m_C = 1;
end

%---------------------Important variable formats---------------------------
%...........................D.UI.snd....................................
%   val 1 = White Noise [true, false]
%   val 2 = Reward Tone [true, false]
%...........................D.AC.data......................................
%   val 1 = conection [0, 1], [no, yes]
%   val 2 = display image [0, 1, 2, 3], [Close all, 0-deg, -40-deg, 40-deg]
%   val 3 = sound state [0, 1], [no sound, sound]
%...........................m2c_id...................................
%    '+', // matlab ready [NA]
%    'T', // system test command [(byte)test]
%    'G', // matlab gui loaded [NA]
%    'N', // netcom setup [NA]
%    'A', // connected to AC computer [NA]
%    'S', // setup session [(byte)ses_cond, (byte)sound_cond]
%    'M', // move to position [(float)targ_pos]
%    'R', // run reward [(float)rew_pos, (byte)zone_ind, (byte)rew_delay]
%    'H', // halt movement [(byte)halt_state]
%    'B', // bulldoze rat [(byte)bull_delay, (byte)bull_speed]
%    'I', // rat in/out [(byte)in/out]
%    'F', // data saved [NA]
%    'X', // confirm quit
%    'C', // confirm close
%...........................c2m_id...................................
%    'J', // battery voltage
%    'Z', // reward zone
%    'V', // robot streaming exit
%    'K', // robot in place
%    'Y', // enable save
%    'E', // exit
%    'C', // confirm close
%--------------------------------------------------------------------------

% ---------------------------- SET PARAMETERS -----------------------------

% MAIN RUN PARAMETERS

% Poll fs (sec)
D.PAR.polRate = 1/30;
% Min time in start quad (sec)
D.PAR.strQdDel = 0.5;
% Warning battery voltage level
D.PAR.batVoltWarning = 11.6;
% Replace battery voltage level
D.PAR.batVoltReplace = 11.8;
% Robot guard dist
D.PAR.guardDist = 4.5 * ((2 * pi)/(140 * pi));
% PID setPoint
D.PAR.setPoint = 42 * ((2 * pi)/(140 * pi));
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

% Cheetah dirs
D.DIR.nlxTempTop = 'C:\CheetahData\Temp';
D.DIR.nlxSaveTop = 'E:\BehaviorPilot';
D.DIR.recFi = '0000-00-00_00-00-00';

% Log dirs
D.DIR.logFi = 'ICR_GUI_Log.csv';
D.DIR.logTemp = fullfile(D.DIR.nlxTempTop,'0000-00-00_00-00-00', D.DIR.logFi);

% DEBUG/TESTING

% Session conditions
D.DB.ratLab = 'r0000';
D.DB.Session_Condition = 'Rotation'; % ['Manual_Training' 'Behavior_Training' 'Rotation']
D.DB.Session_Task = 'Track'; % ['Track' 'Forrage']
D.DB.Reward_Delay = '1.0';
D.DB.Cue_Condition = 'None';
D.DB.Sound_Conditions = [1,1];
D.DB.Rotation_Direction = 'CW'; % ['CCW' 'CW']
D.DB.Start_Quadrant = 'NE'; % [NE,SE,SW,NW];
D.DB.Rotation_Positions = [180,180,180,90,180,270,90,180,270]; % [90,180,270];

% Simulated rat test
D.DB.ratVelStart = 20;
D.DB.ratMaxAcc = 60; % (cm/sec/sec)
D.DB.ratMaxDec = 100; % (cm/sec/sec)
m2c_posSim(:) = 0;

% Halt Error test
D.DB.velSteps = 10:10:80; % (cm/sec)
D.DB.stepSamp = 4;

% Setup c2m globals
timer_c2m = timer;
timer_c2m.ExecutionMode = 'fixedRate';
timer_c2m.Period = 0.1;
timer_c2m.TimerFcn = @CheckC2M;
start(timer_c2m);

% Wait for sync time
if ~isMatSolo
    % Tell CS Matlab ready
    SendM2C('+');
    % Wait for handshake signal
    while c2m_W == 0 && c2m_E == 0; pause(0.1); end
end

% Set sync time
if (c2m_W ~= 0)
    D.DB.startTime = now;
    Console_Write(sprintf('SET SYNC TIME: %ddays',D.DB.startTime));
end

% Open figure
if (c2m_E == 0)
    FigH = figure('Visible', 'Off', ...
        'DeleteFcn', {@ForceClose});
end

% ------------------------- RUN MAIN FUNCTION -----------------------------

% RUN MAIN FUNCTION
if (c2m_E == 0)
    Console_Write('[ICR_GUI] RUNNING: ICR_GUI.m');
    
    % Run in debug mode
    if doDebug
        
        % Stop on error
        dbstop if error;
        Console_Write('[ICR_GUI] RUNNING: In Debug Mode');
        
        % Run main script
        RunScript();
        
    end
    
    % Run in catch error mode
    if ~doDebug
        
        % Catch and print error in console
        dbclear if caught error;
        Console_Write(sprintf('[ICR_GUI] RUNNING: In Release Mode'));
        
        % Run main script
        try RunScript();
        catch ME
            D.DB.isErrExit = true;
            % Log/print error
            err = sprintf(['!!!!!!!!!!!!!ERROR!!!!!!!!!!!!!\r\r', ...
                'Time: %s\r\r', ...
                'ID: %s\r\r', ...
                'Msg: %s\r\r'], ...
                datestr(now, 'HH:MM:SS'), ...
                ME.identifier, ...
                ME.message);
            for z_line = 1:length(ME.stack)
                err = [err, sprintf('Fun: %s\rLine: %d\r', ME.stack(z_line).name, ME.stack(z_line).line)]; %#ok<AGROW>
            end
            err = [err, sprintf('\r!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')];
            Console_Write(err);
        end
    end
    
    % SKIP RUN AND ABORT
else
    Console_Write('!!ERROR!! [ICR_GUI] ABORTING: ICR_GUI.m');
end

% ---------------------------- EXIT PROGRAM -------------------------------

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
if size(who('global'),1) > 0
    % Make new file
    if ~exist(D.DIR.logTemp, 'file');
        mkdir(D.DIR.logTemp);
    end
    fi_path = D.DIR.logTemp;
    fid = fopen(fi_path,'wt');
    for z_l = 1:D.DB.logCount
        fprintf(fid, D.DB.logStr{z_l});
    end
    fclose(fid);
    Console_Write(sprintf('[ICR_GUI] FINISHED: Save ICR_GUI Log to \"%s\"', D.DIR.logTemp));
else
    Console_Write('!!ERROR!! [ICR_GUI] ABBORTED: Save ICR_GUI Log');
end

% Close figure
if ~D.DB.isForceClose
    D.B.close = true;
    close(FigH)
    delete(FigH)
end

% Confirm GUI closed
SendM2C('C');

% Wait for recieved confirmation
Console_Write('[ICR_GUI] RUNNING: Wait for GUI Closed Confirm...');
while c2m_C == 0; drawnow; end;
Console_Write('[ICR_GUI] FINISHED: Wait for GUI Closed Confirm');

% Clear all variables
stop(timer_c2m);
delete(timer_c2m);
clearvars -global;
clearvars;
close all;

% For added measure
Vars=whos;
Vars={Vars.name};
clear(Vars{:});






%% =========================MAIN FUNCTIONS=================================

    function[] = RunScript()
        
        MainLoop();
        
        %% =========================== MAIN LOOP ================================
        
        function[] = MainLoop()
            
            
            % ---------------------------SETUP & RUN----------------------------------
            
            % Run variable setup code
            Var_Setup();
            
            % Run UI setup code
            UI_Setup();
            SendM2C('G');
            
            % Run AC setup code
            AC_Setup();
            SendM2C('A');
            
            % Run NLX setup code
            NLX_Setup();
            SendM2C('N');
            
            % Run testing setup
            Run_Test_Setup();
            
            while c2m_E == 0
                
                % -----------------------CHECK FOR UI SETUP---------------------------
                
                if ~D.B.setup
                    drawnow;
                    
                elseif ~D.B.poll_nlx && ~D.B.rec_done
                    
                    % Run Zone setup code
                    Zone_Dist_Setup();
                    drawnow
                    
                    % Dump initial 1 sec of vt data
                    rat_vt_recs = 1; rob_vt_recs = 1; evt_recs = 1;
                    while Elapsed_Seconds() - D.T.acq_tim < 1 || ...
                            rat_vt_recs > 0 || ...
                            rob_vt_recs > 0 || ...
                            evt_recs > 0
                        [~, ~, ~, ~, rat_vt_recs, ~] = NlxGetNewVTData(D.NLX.vt_rat_ent);
                        [~, ~, ~, ~, rob_vt_recs, ~] = NlxGetNewVTData(D.NLX.vt_rob_ent);
                        [~, ~, ~, ~ , ~, evt_recs, ~] = NlxGetNewEventData('Events');
                    end
                    % Set initial poll time
                    D.T.last_poll_tim = Elapsed_Seconds();
                    
                    % Wait for robot streaming to start
                    Console_Write('[MainLoop] RUNNING: Wait for Robot Streaming...');
                    while c2m_V==0 && c2m_E==0; drawnow; end;
                    if c2m_E==0
                        Console_Write('[MainLoop] FINISHED: Wait for Robot Streaming');
                    else
                        Console_Write('!!ERROR!! [MainLoop] ABORTED: Wait for Robot Streaming');
                        return
                    end
                    
                    % Run Finish setup code
                    Finish_Setup();
                    drawnow;
                    
                    % Begin main loop
                    Console_Write('[MainLoop] READY TO ROCK!');
                    
                    % ---------------------------POLL NETCOM------------------------------
                    
                elseif D.B.poll_nlx
                    
                    % GET ELLAPSED TIME
                    D.T.loop = Elapsed_Seconds() - D.T.last_poll_tim;
                    
                    % PROCESS NLX EVENTS
                    Evt_Proc();
                    
                    % HOLD FOR NETCOM BUFFERS
                    if (D.T.loop >= D.PAR.polRate)
                        D.T.last_poll_tim = Elapsed_Seconds();
                        
                        % PROCESS NLX VT
                        
                        % Run VT processing code
                        % Robot
                        VT_Get('Rob');
                        VT_Proc('Rob');
                        % Rat
                        if ~D.DB.doSimRatTest
                            VT_Get('Rat');
                            VT_Proc('Rat');
                        end
                        
                        % RUN TEST/DEBUG CODE
                        Run_Test();
                        
                        % CHECK IF RAT IN
                        
                        % Run rat in check code
                        if ...
                                c2m_K == 1 && ...
                                ~D.B.rat_in && ...
                                D.B.rec && ...
                                ~D.B.rec_done && ...
                                ~D.B.quit
                            
                            Rat_In_Check();
                        end
                        
                        % -----------------------ONCE RAT IN-----------------------------------
                        
                        if D.B.rat_in
                            
                            % ROTATION TRIGGER CHECK
                            
                            % Run rotation check code
                            Rotation_Trig_Check();
                            
                            % REWARD RESET CHECK
                            
                            % Run reward reset check code
                            Reward_Send_Check();
                            
                            % REWARD FEEDER CHECK
                            
                            % Run reward check code
                            Reward_Check();
                            
                            % LAP CHECK
                            
                            % Run lap check code
                            Lap_Check();
                            
                        end
                        
                        % PLOT POSITION
                        
                        % Run plot position code
                        Pos_Plot();
                        
                        % PRINT SES INFO
                        
                        % Run info print code
                        Inf_Print();
                        
                    end
                    
                    % Check if CS has enabled save
                    if c2m_Y == 1 && D.B.rec_done
                        % Enable save button
                        set(D.UI.btnSave, ...
                            'Enable', 'on', ...
                            'ForegroundColor' , D.UI.enabledBtnFrgCol, ...
                            'BackgroundColor', D.UI.activeCol);
                        c2m_Y = 0;
                    end
                    
                    % Save sesion data
                    if D.B.do_save
                        Save_Ses_Data();
                        D.B.do_save = false;
                    end
                    
                end
                
                % Update GUI
                drawnow;
                
            end
            
        end
        
        
        
        
        
        
        %% ========================= SETUP FUNCTIONS ==============================
        
        % ------------------------------VAR SETUP----------------------------------
        
        function [] = Var_Setup()
            
            % SESSION VARIABLES
            
            % Load data tables
            T = load(D.DIR.ioSS_In_All);
            D.SS_In_All = T.SS_In_All;
            T = load(D.DIR.ioSS_Out_ICR);
            D.SS_Out_ICR = T.SS_Out_ICR;
            clear T;
            
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
            
            % BOOLEANS
            
            % setup
            D.B.setup = false;
            % polling nlx
            D.B.poll_nlx = false;
            % acquiring nlx
            D.B.acq = false;
            % recording nlx
            D.B.rec = false;
            % recording done
            D.B.rec_done = false;
            % track if rat is in arena
            D.B.rat_in = false;
            % flag to do save
            D.B.do_save = false;
            % flag quit
            D.B.quit = false;
            % flag gui forced exit
            D.DB.isForceClose = false;
            % flag gui closed
            D.B.close = false;
            % rotation has occured
            D.B.rotated = false;
            % track if reward in progress
            D.B.is_rewarding = false;
            % block any cueing
            D.B.do_block_cue = false;
            % cue every lap
            D.B.do_all_cue = false;
            % track if next lap should be cued for half cue cond
            D.B.is_cued_rew = false;
            % flag if haulted
            D.B.is_haulted = false;
            % track reward reset
            D.B.flag_rew_confirmed = false;
            % track reset crossing
            D.B.flag_rew_send_crossed = false;
            % track reward crossing
            D.B.flag_rew_zone_crossed = false;
            % check for reward zone confirmaton
            D.B.check_rew_confirm = false;
            % track lap bounds
            D.B.check_inbound_lap = false(4,1);
            % track if reward point should be plotted
            D.B.plot_rew = false;
            % track if pos should be plotted
            D.B.Rat.plot_pos = false;
            D.B.Rob.plot_pos = false;
            % track if velocity should be plotted
            D.B.Rat.plot_vel = false;
            D.B.Rob.plot_vel = false;
            % plot HD
            D.B.plot_hd = false;
            
            % TIMERS
            
            % last poll time
            D.T.last_poll_tim = Elapsed_Seconds();
            % time session starts
            D.T.ses_str_tim = Elapsed_Seconds();
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
            % loop time
            D.T.loop = 0;
            D.T.loop_min = 1;
            D.T.loop_max = NaN;
            % track manual reward sent time
            D.T.manual_rew_sent = 0;
            % track last reward time
            D.T.rew_last = Elapsed_Seconds();
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
            D.T.Rat.last_pos_update = Elapsed_Seconds();
            D.T.Rob.last_pos_update = Elapsed_Seconds();
            
            % DEBUG VARS
            % track reward duration [now, min, max, sum, count]
            D.DB.rew_duration = [0, inf, 0, 0, 0];
            % track reward round trip [now, min, max, sum, count]
            D.DB.rew_round_trip = [0, inf, 0, 0, 0];
            % halt error [now, min, max, sum, count]
            D.DB.halt_error = [0, inf, 0, 0, 0];
            
            % INDEXING
            
            % current wall image index
            D.I.rot = 1;
            % feeder index
            D.I.img_ind = [1, NaN];
            % current lap quadrant for lap track
            D.I.lap_hunt_ind = 1;
            
            % REWARD VARS
            % min/max reward duration
            D.PAR.rewDurLim = [500, 2000];
            % reward zone positions
            D.PAR.zoneLocs = 20:-5:-20;
            % reward zone reward durations
            D.PAR.zoneRewDur = ...
                [500, 910, 1420, 1840, 2000, 1840, 1420, 910, 500];
            % current reward duration
            D.PAR.rewDur = max(D.PAR.rewDurLim);
            % current reward zone
            D.I.zone = ceil(length(D.PAR.zoneLocs)/2); % default max
            % distrebution of reward zones
            D.I.zoneArr = NaN(1,100);
            % store reward zone for each reward event
            D.I.zoneHist = NaN(1,100);
            % counter for each reward zone
            D.C.zone = zeros(2,length(D.PAR.zoneLocs));
            % handles
            D.UI.zoneAllH = gobjects(length(D.PAR.zoneLocs),2);
            D.UI.zoneNowH = gobjects(1,1);
            D.UI.zoneAvgH = gobjects(1,1);
            D.UI.durNowTxtH = gobjects(1,1);
            
            % POSITION VARS
            
            % Arena dimensions
            % radius (cm)
            D.UI.arnRad = 70;
            % track width (cm)
            D.UI.trkWdt = 10;
            
            % Load track bound data
            S = load(D.DIR.ioTrkBnds);
            % track bound radius
            D.PAR.R = S.R;
            % track bound center X
            D.PAR.XC = S.XC;
            % track bound center Y
            D.PAR.YC = S.YC;
            clear S;
            
            % nlx input
            D.P.Rat.vtTS = NaN;
            D.P.Rat.vtPos = NaN;
            D.P.Rat.vtHD = NaN;
            D.P.Rat.vtNRecs = NaN;
            D.P.Rob.vtTS = NaN;
            D.P.Rob.vtPos = NaN;
            D.P.Rob.vtHD = NaN;
            D.P.Rob.vtNRecs = NaN;
            % cardinal pos now
            D.P.Rat.x = NaN;
            D.P.Rat.y = NaN;
            D.P.Rob.x = NaN;
            D.P.Rob.y = NaN;
            % pos rad
            D.P.Rat.rad = NaN;
            D.P.Rob.rad = NaN;
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
            % position cutoff
            D.P.posRohMax = 1 + (5/D.UI.arnRad);
            D.P.posRohMin = (1 - (D.UI.trkWdt+5)/D.UI.arnRad);
            % velocity cutoff
            D.P.velRohMax = D.P.posRohMin;
            D.P.velRohMin = D.P.posRohMin - 0.25;
            D.P.velMin = 0;
            D.P.velMax = 100;
            
            % UI POS PLOT HANDLES AND CUMULATIVE DATA ARRAYS
            
            % vt plot handle array
            D.UI.ratPltH = gobjects(1,60*120/D.PAR.polRate);
            % vt plot handles of velocity
            D.UI.ratPltHvel = gobjects(1,60*120/D.PAR.polRate); % rat
            D.UI.robPltHvel = gobjects(1,60*120/D.PAR.polRate); % rob
            % handles for average vel plots
            D.UI.Rat.pltHvelAvg = gobjects(1,1); % rat
            D.UI.Rob.pltHvelAvg = gobjects(1,1); % rob
            % handles for history data
            D.UI.Rat.pltHposAll = gobjects(1,1); % rat
            D.UI.Rat.pltHvelAll = gobjects(1,1); % rat
            D.UI.Rob.pltHvelAll = gobjects(1,1); % rob
            
            
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
            D.UI.enabledCol = [0.5, 0.5, 0.5];
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
            D.UI.taskList = {''; 'Track'; 'Forrage'};
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
        
        % -------------------------------UI SETUP-----------------------------------
        
        function[] = UI_Setup()
            
            %% GENERATE FIGURE AND AXES
            
            % Bounding box for video tracker (in pixels)
            % track plot bounds (width, hight)
            
            % Plot bounds
            D.UI.vtRes = round([D.PAR.R*2,D.PAR.R*2]);
            % track plot bounds (left, bottom)
            D.UI.lowLeft = round([D.PAR.XC-D.PAR.R,D.PAR.YC-D.PAR.R]);
            
            % Calculate pixel to cm conversion factors
            D.UI.xCMcnv = D.UI.vtRes(1)/140;
            D.UI.yCMcnv = D.UI.vtRes(2)/140;
            
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
            
            % Generate backround axis
            % backround axis
            D.UI.axH(1) = axes(...
                'Units', 'Normalized', ...
                'Visible', 'off');
            hold on;
            
            % Generate path axis
            % Note: used for path and other dynamic features
            D.UI.axH(2) = axes(...
                'Units', 'Normalized', ...
                'Visible', 'off');
            hold on;
            
            % Wall image axis
            D.UI.axH(3) = axes(...
                'Units', 'Normalized', ...
                'Visible', 'off');
            hold on;
            
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
                D.UI.lowLeft(1)+D.UI.vtRes(1), ...
                D.UI.lowLeft(2), ...
                D.UI.lowLeft(2)+D.UI.vtRes(2)]);
            
            % Set axis pos
            ax_pos = [ ...
                (1 - (0.9/(fg_wh(1)/fg_wh(2)))) / 2, ...
                0.05, ...
                0.9/(fg_wh(1)/fg_wh(2)), ...
                0.9];
            set(D.UI.axH, 'Position', ax_pos);
            
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
            % CREATE TRACK OUTLINE
            t2_h = copyobj(t_h, D.UI.axH(2));
            set(t2_h, ...
                'Color', D.UI.activeCol, ...
                'FontWeight', 'light', ...
                'FontSmoothing', 'on', ...
                'FontSize', 20);
            
            % Get coordinates for circle
            circ = [0:.01:2*pi,0];
            
            % Plot outer track
            [out_X,out_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.arnRad));
            xout = out_X*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
            yout = out_Y*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
            D.UI.trckH(1) = plot(xout, yout, ...
                'color', [0.5 0.5 0.5], ...
                'LineWidth', 2, ...
                'Parent', D.UI.axH(2));
            
            % Plot inner track
            [in_X,in_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.arnRad-D.UI.trkWdt));
            xin = in_X*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
            yin = in_Y*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
            D.UI.trckH(2) = plot(xin, yin, ...
                'color', [0.5 0.5 0.5], ...
                'LineWidth', 2, ...
                'Parent', D.UI.axH(2));
            
            % CREATE VEL PLOT OUTLINE
            n_rings = ceil(D.P.velMax/10)+1;
            roh_inc = linspace(D.P.velRohMin, D.P.velRohMax, n_rings);
            
            for z_lin = 1:n_rings
                [x,y] = pol2cart(circ, ones(1,length(circ))*D.UI.arnRad*roh_inc(z_lin));
                x = x*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
                y = y*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
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
                D.UI.velLimH(z_lin) = plot(x, y, ...
                    'color', col, ...
                    'LineWidth', lin_wdth, ...
                    'LineStyle', lin_style, ...
                    'Parent', D.UI.axH(2));
            end
            
            % GUI OBJECT POSITIONS
            
            % Questions dialogue pos
            D.UI.qstDlfPos = [D.UI.sc1(3) + D.UI.sc2(3)/2, D.UI.sc2(4)/2];
            
            % Bounds of plot space
            D.UI.main_ax_bounds = [ ...
                ax_pos(1) - 0.05, ...
                ax_pos(2) - 0.05, ...
                ax_pos(3) + 0.1, ...
                ax_pos(4) + 0.1];
            
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
            
            % MAKE WALL IMAGES
            
            % Wall image axis
            img_ax_pos = [ax_pos(1) - ax_pos(3)*0.035, ...
                ax_pos(2) - ax_pos(4)*0.035, ...
                ax_pos(3) * 1.07, ...
                ax_pos(4) * 1.07];
            set(D.UI.axH(3), 'Position', img_ax_pos);
            uistack(D.UI.axH(3), 'bottom');
            
            % Read in image
            img = imread(D.DIR.ioWallImage, 'BackgroundColor', D.UI.figBckCol);
            set(D.UI.axH(3), 'XLim', [0,size(img,2)], 'YLim', [0,size(img,1)]);
            img = flip(img, 1);
            % 0 deg
            img0 = img;
            % -40 deg
            mask = true(size(img));
            img = imrotate(img, -40, 'crop');
            maskR = ~imrotate(mask, -40, 'crop');
            img(maskR) = 255;
            imgCCW = img;
            % 40 deg
            mask = true(size(img));
            img = imrotate(img, 40, 'crop');
            maskR = ~imrotate(mask, 40, 'crop');
            img(maskR) = 255;
            imgCW = img;
            
            % Store for later
            D.UI.wallImgH(1) = image(img0, 'Parent', D.UI.axH(3), 'Visible', 'on');
            D.UI.wallImgH(2) = image(imgCCW, 'Parent', D.UI.axH(3), 'Visible', 'off');
            D.UI.wallImgH(3) = image(imgCW, 'Parent', D.UI.axH(3), 'Visible', 'off');
            
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
                'FontSize',text1_font_sz(1), ...
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
                'Units','normalized',...
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
            pos = [pos(1), pos(2)-0.03-0.01, wdth, 0.03];
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
            movegui(FigH,'center')
            % Bring UI to top
            uistack(FigH, 'top')
            % Make visible
            set(FigH, 'Visible', 'on');
            drawnow;
            
            % Log/print
            Console_Write('[UI_Setup] FINISHED: UI Setup');
            
        end
        
        % -------------------------------AC SETUP----------------------------------
        
        function [] = AC_Setup()
            
            % Setup communication with ARENACONTROLLER
            D.AC.IP = '172.17.0.3';
            
            % Initialize parameters
            % signal to connect to ARENACONTROLLER
            data(1) = 1;
            % signal for display image
            data(2) = 0;
            % signal for sound condition
            data(3) = 0;
            
            % Convert to 8 bit signed int
            data = int8(data);
            
            % Get byte size
            s = whos('data');
            
            % Move to D struct
            D.AC.data = data;
            clear data
            
            % Create tcpip object
            tcpIP = tcpip('0.0.0.0',55000,'NetworkRole','Server');
            
            % Set bitesize
            set(tcpIP,'OutputBufferSize',s.bytes);
            
            % Print that AC computer is connected
            Console_Write(sprintf('[AC_Setup] RUNNING: Connect to AC Computer... IP=%s', ...
                D.AC.IP));
            
            % Establish connection
            % will loop here until connection established
            fopen(tcpIP);
            
            % Print that AC computer is connected
            Console_Write('[AC_Setup] FINISHED: Connect to AC Computer');
            
        end
        
        % ------------------------------NLX SETUP----------------------------------
        
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
            
            % Log/print
            Console_Write(sprintf('[NLX_Setup] RUNNING: Connect to NLX... IP=%s', ...
                D.NLX.IP));
            
            % Wait for Cheetah to open
            is_running = false;
            while ~is_running && c2m_E == 0
                [~,result] = system('tasklist /FI "imagename eq cheetah.exe" /fo table /nh');
                is_running = any(strfind(result, 'Cheetah.exe'));
            end
            % Log/print
            Console_Write('[NLX_Setup] CONFIRM: Cheetah.exe Running');
            
            % Load NetCom into Matlab, and connect to the NetCom server if we aren’t connected
            if NlxAreWeConnected() ~= 1
                while NlxAreWeConnected() ~= 1 && c2m_E == 0
                    succeeded = NlxConnectToServer(D.NLX.IP);
                    if succeeded == 1
                        % Log/print
                        Console_Write('[NLX_Setup] CONFIRM: Connect to NLX');
                        %Identify this program to the server.
                        NlxSetApplicationName('ICR_GUI');
                    end
                end
            else
                % Log/print
                Console_Write('[NLX_Setup] CONFIRM: Already Connected to NLX');
            end
            
            %% CONFIGURE DIGITAL IO
            
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
            
            %% START STREAMING
            
            % Open the data stream for the VT acquisition entity.  This tells Cheetah to begin
            % streaming data for the VT acq ent.
            NlxOpenStream(D.NLX.vt_rat_ent);
            NlxOpenStream(D.NLX.vt_rob_ent);
            NlxOpenStream(D.NLX.event_ent);
            
            % Run BtnAcq
            set(D.UI.btnAcq, 'Value', 1)
            BtnAcq(D.UI.btnAcq);
            
            %% ENABLE BUTTONS ONCE CONNECTED
            
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
            Console_Write('[NLX_Setup] FINISHED: NLX Setup');
            
        end
        
        % -------------------------REWARD ZONE SETUP------------------------------
        
        function[] = Zone_Dist_Setup()
            
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
                'Visible','on');
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
                'Visible', 'on', ...
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
            %             Console_Write(sprintf('[Zone_Dist_Setup] Computed Zone Dist: \r%s', ...
            %                 str,));
            
            % Log/print
            Console_Write('[Zone_Dist_Setup] FINISHED: Zone Dist Setup');
            
        end
        
        % ----------------------------FINISH SETUP---------------------------------
        
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
            if D.PAR.ratRotDrc == 'CCW';
                D.I.img_ind(2) = 2;
            elseif D.PAR.ratRotDrc == 'CW';
                D.I.img_ind(2) = 3;
            end
            
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
            
            %% Update and send AC.data values
            
            % Display image
            D.AC.data(2) = 1;
            
            % Sound stimuli (start without sound)
            D.AC.data(3) = single(D.UI.snd(1));
            
            % Post to AC computer
            fwrite(tcpIP,D.AC.data,'int8');
            
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
            infstr = datestr(D.T.ses_str_tim, 'HH:MM:SS');
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
                'HE: %4.0f  mn:%4.0f  mx:%4.0f av:%2.2f\n', ...
                'Lp: %4.0f  mn:%4.0f  mx:%4.0f\n', ...
                ], ...
                0, 0, 0, 0, ...
                0, 0, 0, 0, ...
                0, 0, 0, 0, ...
                0, 0, 0 ...
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
            D.UI.fd_x = fd_X*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
            % all feeders y
            D.UI.fd_y = fd_Y*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
            
            % Save reward feeder rad pos
            D.UI.rewFeedRad(1) = deg2rad(fdLocs(D.UI.rewFeed(1)));
            D.UI.rewFeedRad(2) = deg2rad(fdLocs(D.UI.rewFeed(2)));
            % with setpoint correction
            D.UI.rewRatHead(1:2) = ...
                D.UI.rewFeedRad + deg2rad(D.PAR.trigDist);
            
            % REWARD FEEDER BOUNDS
            for z_zone = 1:length(D.PAR.zoneLocs)
                D.UI.rewBnds(z_zone,:,1) = [...
                    D.UI.rewFeedRad(1) + deg2rad(D.PAR.feedSet(1) + D.PAR.zoneLocs(z_zone)), ...
                    D.UI.rewFeedRad(1) + deg2rad(D.PAR.feedSet(2) + D.PAR.zoneLocs(z_zone))];
                D.UI.rewBnds(z_zone,:,2) = [...
                    D.UI.rewFeedRad(2) + deg2rad(D.PAR.feedSet(1) + D.PAR.zoneLocs(z_zone)), ...
                    D.UI.rewFeedRad(2) + deg2rad(D.PAR.feedSet(2) + D.PAR.zoneLocs(z_zone))];
            end
            % [zone,min_max,rot_cond]
            D.UI.rewBnds = wrapTo2Pi(D.UI.rewBnds);
            
            % REWARD RESET BOUNDS
            D.UI.rewRstBnds(1,1:2) = D.UI.rewBnds(1,end,1) + [deg2rad(10),deg2rad(40)];
            D.UI.rewRstBnds(2,1:2) = D.UI.rewBnds(1,end,2) + [deg2rad(10),deg2rad(40)];
            D.UI.rewRstBnds = wrapTo2Pi(D.UI.rewRstBnds);
            
            % REWARD PASS BOUNDS
            D.UI.rewPassBnds(1,1:2) = D.UI.rewBnds(end,end,1) - [deg2rad(35), deg2rad(5)];
            D.UI.rewPassBnds(2,1:2) = D.UI.rewBnds(end,end,2) - [deg2rad(35), deg2rad(5)];
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
            D.UI.lapBnds = arrayfun(@(x) [x-45, x+45], lapBndLocs, 'Uni', false);
            D.UI.lapBnds = cell2mat(D.UI.lapBnds);
            % set range to [0, 360]
            D.UI.lapBnds = wrapTo360(D.UI.lapBnds);
            % convert to radians
            D.UI.lapBnds = deg2rad(D.UI.lapBnds);
            D.UI.lapBnds = wrapTo2Pi(D.UI.lapBnds);
            
            % START QUADRANT BOUNDS
            
            % Set start quadrant bound to 60 deg
            D.UI.strQuadBnds = [lapBndLocs(end) - 30, ...
                lapBndLocs(end) + 30];
            % convert to radians
            D.UI.strQuadBnds = deg2rad(D.UI.strQuadBnds);
            D.UI.strQuadBnds = wrapTo2Pi(D.UI.strQuadBnds);
            
            %% Plot bounds data
            
            % Plot start quadrant
            [xbnd, ybnd] =  Get_Rad_Bnds(D.UI.strQuadBnds);
            D.UI.ptchStQ = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                [0.5,0.5,0.5], ...
                'FaceAlpha',0.5, ...
                'Parent',D.UI.axH(2));
            % add star text
            D.UI.txtStQ = ...
                text(mean(xbnd(:,round(size(xbnd,2)/2))), ...
                mean(ybnd(:,round(size(ybnd,2)/2))), ...
                'Start', ...
                'FontSize', 12, ...
                'FontWeight', 'Bold', ...
                'Color', [0.3,0.3,0.3], ...
                'HorizontalAlignment', 'Center', ...
                'Parent',D.UI.axH(2));
            
            % Plot reward bounds
            D.UI.ptchFdH = gobjects(length(D.PAR.zoneLocs),2);
            D.UI.durNowTxtH = gobjects(length(D.PAR.zoneLocs),2);
            for z_fd = 1:2
                for z_zone = 1:length(D.PAR.zoneLocs)
                    
                    % reward bounds
                    [xbnd, ybnd] =  ...
                        Get_Rad_Bnds(D.UI.rewBnds(z_zone,:,z_fd));
                    D.UI.ptchFdH(z_zone,z_fd) = ...
                        patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                        [ybnd(1,:),fliplr(ybnd(2,:))], ...
                        D.UI.rotCol(z_fd,:), ...
                        'EdgeColor', [0, 0, 0], ...
                        'EdgeAlpha', 0.025, ...
                        'FaceAlpha', 0.05, ...
                        'LineWidth', 1, ...
                        'Parent', D.UI.axH(2));
                    
                    % Add reward duration text
                    str = ...
                        sprintf('%d%c\n%d ms', -1*D.PAR.zoneLocs(z_zone), char(176), D.PAR.zoneRewDur(z_zone));
                    D.UI.durNowTxtH(z_zone,z_fd) = text(...
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
            uistack(reshape(D.UI.durNowTxtH,1,[]),'top');
            % make 0 bounds visible
            set(D.UI.ptchFdH(:,1), 'EdgeAlpha', 0.05, 'FaceAlpha', 0.15);
            
            % Plot reward reset bounds
            D.UI.ptchFdRstH = gobjects(1,2);
            for z_fd = 1:2
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds(D.UI.rewRstBnds(z_fd,:));
                D.UI.ptchFdRstH(z_fd) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    D.UI.rotCol(z_fd,:), ...
                    'FaceAlpha', 0.5, ...
                    'EdgeAlpha', 0.5, ...
                    'Parent', D.UI.axH(2), ...
                    'Visible', 'off');
            end
            
            % Plot lap bounds
            % set alpha to 0 (transparent)
            D.UI.ptchLapBnds = gobjects(1,4);
            for z_quad = 1:4
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds(D.UI.lapBnds(z_quad,:));
                D.UI.ptchLapBnds(z_quad) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    [0.5,0.5,0.5], ...
                    'FaceAlpha', 0.1, ...
                    'EdgeAlpha', 0, ...
                    'Parent', D.UI.axH(2), ...
                    'Visible', 'off');
            end
            
            %% Plot remaining features
            
            % Plot all feeders
            D.UI.fdAllH = plot(D.UI.fd_x, D.UI.fd_y, 'o', ...
                'MarkerFaceColor', [0.5 0.5 0.5], ...
                'MarkerEdgeColor', [0.1,0.1,0.1], ...
                'MarkerSize', 20, ...
                'Parent', D.UI.axH(2));
            
            % Feeder reward feeder marker
            D.UI.fdH = gobjects(4,2);
            for z_fd = 1:2
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds([D.UI.rewFeedRad(z_fd), ...
                    D.UI.rewFeedRad(z_fd) + deg2rad(D.PAR.trigDist)]);
                % setpoint zone line
                D.UI.fdH(1,z_fd) = ...
                    plot(xbnd(:,end), ybnd(:,end), ...
                    'Color', D.UI.rotCol(z_fd,:), ...
                    'LineWidth', 2, ...
                    'Parent',D.UI.axH(2));
                % distance line
                D.UI.fdH(2,z_fd) = ...
                    plot(xbnd(2,:), ybnd(2,:), ...
                    'Color', D.UI.rotCol(z_fd,:), ...
                    'LineWidth', 3, ...
                    'Parent',D.UI.axH(2),...
                    'Visible', 'on');
                % marker
                D.UI.fdH(3,z_fd) = ...
                    plot(D.UI.fd_x(D.UI.rewFeed(z_fd)), ...
                    D.UI.fd_y(D.UI.rewFeed(z_fd)), 'o', ...
                    'MarkerFaceColor', D.UI.rotCol(z_fd,:), ...
                    'MarkerEdgeColor', [0.1,0.1,0.1], ...
                    'MarkerSize', 20, ...
                    'Parent',D.UI.axH(2));
            end
            % Enlarge 0 deg marker
            set(D.UI.fdH(3,1), 'MarkerSize', 25)
            uistack(reshape(D.UI.fdH(1:2,:),1,[]),'down',1);
            
            % Plot opposite unrewarded feeders darker
            plot(D.UI.fd_x(D.UI.oppFeed), D.UI.fd_y(D.UI.oppFeed), 'o', ...
                'MarkerFaceColor', [0.25 0.25 0.25], ...
                'MarkerEdgeColor', [0.1,0.1,0.1], ...
                'MarkerSize', 20, ...
                'Parent',D.UI.axH(2));
            
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
            
            %% Start recording and set remaining vars
            
            % Make sure reward reset is active
            if D.PAR.sesCond ~= 'Manual_Training' %#ok<*STCMP>
                % Set reset patch to visible
                set(D.UI.ptchFdRstH(D.I.rot), 'Visible', 'on');
            end
            
            % Set to start polling NLX
            D.B.poll_nlx = true;
            
            % Run BtnRec
            set(D.UI.btnRec,'Value', 1);
            BtnRec(D.UI.btnRec);
            
            % Send C# command to move robot to start quad or reward loc
            if D.PAR.sesCond ~= 'Manual_Training' %#ok<*STCMP>
                SendM2C('M', D.UI.strQuadBnds(1));
            else
                SendM2C('M', D.UI.rewFeedRad(1));
            end
            
            % Log/print
            Console_Write('[Finish_Setup] FINISHED: Session Setup');
            
        end
        
        % ----------------------------RAT IN CHECK---------------------------------
        
        function [] = Rat_In_Check()
            
            % Bail if no new data
            if all(isnan(D.P.Rat.rad))
                return
            end
            
            % Keep checking if rat is in the arena
            check_inbound = Check_Rad_Bnds(D.P.Rat.rad, D.UI.strQuadBnds);
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
            D.B.rat_in = true;
            
            % Save time
            D.T.run_str = Elapsed_Seconds();
            
            % Start tracking lap times
            D.T.lap_tim = Elapsed_Seconds();
            
            % Set patch to nonvisible
            set(D.UI.ptchStQ, ...
                'FaceAlpha', 0, ...
                'EdgeAlpha', 0);
            
            % Set lap patches to visible
            set(D.UI.ptchLapBnds, 'Visible', 'on');
            
            % Clear VT data
            BtnClrVT();
            
            % Reinitialize
            D.T.strqd_inbnd_t1 = 0;
            
            % Log/print
            Console_Write(sprintf('[Rat_In_Check] FINISHED: Rat In Check: OCC=%0.2fsec', ...
                inbndTim));
            
        end
        
        % -----------------------------TEST SETUP---------------------------------
        
        function[] = Run_Test_Setup()
           
            if D.DB.isTestRun
                
                % Load rat 0000
                
                % Change data table entries which will be loaded later
                ratInd = ...
                    find(ismember(D.SS_In_All.Properties.RowNames, D.DB.ratLab));
                
                % SET LAST SAVED ENTRIES TO WHAT WE WANT
                
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
                col_ind = ismember([{'Track'},{'Forage'}], D.DB.Session_Task);
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
                
                % PID CALIBRATIONcross_cnt
                if D.DB.doPidCalibrationTest
                    % Set flag to start pid
                    D.B.pidStarted = false;
                end
                
                % HALT ERROR TEST
                if D.DB.doHaultErrorTest
                    D.DB.haltCnt = D.DB.stepSamp;
                    D.DB.haltDur = 5; % (sec)
                    D.DB.halt_error_str = '';
                    D.DB.t_halt = Elapsed_Seconds();
                    D.DB.nowStep = 0;
                    D.DB.nowVel = 0;
                    D.DB.isHalted = true;
                    D.DB.sendPos = 0;
                    
                    % Enable editing in console
                    set(D.UI.listConsole, 'Enable','on');
                end
                
                % SIMULATED RAT TEST
                if D.DB.doSimRatTest
                    D.B.simRadLast = NaN;
                    D.B.simVelLast = NaN;
                    D.B.simTSStart = NaN;
                    D.B.simTSLast = NaN;
                    D.B.initVals = true;
                    
                    pos = [0.5-0.175/2, 0.63, 0.175,0.02];
                    D.B.UI.sld = uicontrol('Style', 'slider',...
                        'Parent',FigH, ...
                        'Units', 'Normalized', ...
                        'Min',0,'Max',100,'Value',D.DB.ratVelStart,...
                        'SliderStep', [0.01,0.1], ...
                        'Position', pos);
                    % Vel text
                    pos = [pos(1)+pos(3), pos(2), 0.03, pos(4)];
                    D.B.UI.txt = uicontrol('Style', 'text',...
                        'Parent',FigH, ...
                        'Units', 'Normalized', ...
                        'BackgroundColor', D.UI.figBckCol, ...
                        'ForegroundColor', D.UI.enabledCol, ...
                        'FontSize', 12, ...
                        'FontWeight', 'Bold', ...
                        'String','100',...
                        'Position', pos);
                end
                
                % Run BtnSetupDone
                set(D.UI.btnSetupDone, 'Value', 1);
                BtnSetupDone();
                
                % Log/print
                Console_Write('[Run_Test_Setup] FINISHED: Test Run Setup');
            end
        end
        
        
        
        
        
        
        %% ======================== ONGOING FUNCTIONS =============================
        
        % ------------------------------GET NLX VT---------------------------------
        
        function [] = VT_Get(fld)
            
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
            
        end
        
        % ------------------------------PROCESS NLX VT---------------------------------
        
        function [] = VT_Proc(fld)
            
            if D.P.(fld).vtNRecs > 0
                
                %% PROCESS NEW DATA
                
                % convdert to [r = samp, c = dim]
                xy_pos = reshape(double(D.P.(fld).vtPos),2,[])';
                ts = double(D.P.(fld).vtTS)';
                recs = D.P.(fld).vtNRecs;
                
                % Save x/y pos samples in seperate vars
                x = xy_pos(:,1);
                y = xy_pos(:,2);
                
                % Rescale y as VT data is compressed in y axis
                y = y*11/10;
                
                % Get nnormalized pos data
                x_norm = (x-D.PAR.XC)./D.PAR.R;
                y_norm = (y-D.PAR.YC)./D.PAR.R;
                
                % Get position in radians
                [rad,roh] = cart2pol(x_norm, y_norm);
                
                % Convert radians between [0, 2*pi]
                rad = wrapTo2Pi(rad);
                
                % Flip radian values to acount for inverted y values from Cheetah
                rad = abs(rad - 2*pi);
                
                % Exclude outlyer values > || < track bounds plus 5 cm
                exc_1 = roh > D.P.posRohMax | roh < D.P.posRohMin;
                
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
                
                % Do not use exc_3/4 if more than reward duration of unused data
                if Elapsed_Seconds() - D.T.(fld).last_pos_update > (D.PAR.rewDur+100)/1000
                    exc_3 = zeros(size(exc_3,1), 1);
                end
                
                % Check that reward flag has not been set for too long
                if D.T.rew_start > 0
                    if Elapsed_Seconds() - D.T.rew_start > 5 && ...
                            D.B.is_rewarding == true
                        % reset flag
                        D.B.is_rewarding = false;
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
                    D.B.(fld).plot_pos = false;
                    D.B.(fld).plot_vel = false;
                    
                    % Set vars to NaN
                    D.P.(fld).x = NaN;
                    D.P.(fld).y = NaN;
                    D.P.(fld).rad = NaN;
                    D.P.(fld).ts = NaN;
                    D.P.(fld).recs = NaN;
                    D.P.(fld).vel_pol_arr(end,:) = NaN;
                    
                    % Exit function
                    return;
                    
                else
                    % Plot pos data this loop
                    D.B.(fld).plot_pos = true;
                    
                    % Keep track of updates
                    D.T.(fld).last_pos_update = Elapsed_Seconds();
                    
                    % Save last usable rad value
                    D.P.(fld).radLast = rad(end);
                    
                    %                     if strcmp(fld, 'Rob')
                    %                         fprintf('\rRob rad: %0.2f', D.P.(fld).radLast)
                    %                     else
                    %                         fprintf('   Rat rad: %0.2f\r', D.P.(fld).radLast)
                    %                     end
                end
                
                % Store vars for later use
                D.P.(fld).x = x;
                D.P.(fld).y = y;
                D.P.(fld).rad = rad;
                D.P.(fld).ts = ts;
                D.P.(fld).recs = recs;
                
                % Save history for rat pos
                if strcmp(fld, 'Rat')
                    ind(1) = find(isnan(D.P.Rat.pos_hist(:,1)), 1, 'first');
                    ind(2) = ind(1)+length(x)-1;
                    D.P.Rat.pos_hist(ind(1):ind(2), 1) = x;
                    D.P.Rat.pos_hist(ind(1):ind(2), 2) = y;
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
                if strcmp(fld, 'Rob') && ~D.B.rat_in
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
                    D.B.(fld).plot_vel = false;
                    
                    % Avoid jumps in plot
                    D.P.(fld).vel_pol_arr(end,:) = NaN;
                    
                else
                    
                    % Set to plot this vel
                    D.B.(fld).plot_vel = true;
                    
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
                    if velRoh>D.P.velMax; 
                        velRoh = D.P.velMax;
                      % Set <min to min  
                    elseif velRoh<D.P.velMin; 
                        velRoh = D.P.velMin;
                    end
                    % Adjust range
                    velRoh = velRoh/D.P.velMax;
                    velRoh = velRoh*(D.P.velRohMax-D.P.velRohMin) + D.P.velRohMin;
                    
                    % Cap to roh max
                    if velRoh > D.P.velRohMax;
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
                if  D.B.plot_hd && strcmp(fld, 'Rat')
                    
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
        
        % --------------------------PROCESS NLX EVENTS-----------------------------
        
        function [] = Evt_Proc()
            
            % Read in new event data
            %[evtPass, evtTS, evtID, evtTTL, evtStr, evtNRecs, evtDropped]
            [~, evtTS, ~, ~ , evtStr, evtNRecs, ~] = ...
                NlxGetNewEventData('Events');
            
            if evtNRecs > 0
                
                %% CHECK FOR REWARD
                
                % Reward Started
                if any(ismember(evtStr, D.NLX.rew_on_str))
                    
                    % Save reward start time
                    D.T.rew_start = Elapsed_Seconds();
                    
                    % Get time stamp
                    D.T.rew_nlx_ts(1) = evtTS(ismember(evtStr, D.NLX.rew_on_str));
                    
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
                    D.B.is_rewarding = true;
                    
                end
                
                % Reward Ended
                if any(ismember(evtStr, D.NLX.rew_off_str))
                    
                    % Save reward end time
                    D.T.rew_end = Elapsed_Seconds();
                    
                    % Get time stamp
                    D.T.rew_nlx_ts(2) = evtTS(ismember(evtStr, D.NLX.rew_off_str));
                    
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
                        halt_err = D.UI.rewBnds(D.I.zone,2,D.I.rot) - (D.P.Rob.radLast-D.PAR.setPoint);
                        % Save reward duration
                        D.DB.halt_error(1) = halt_err * ((140*pi)/(2*pi));
                        D.DB.halt_error(2) = min(D.DB.halt_error(1), D.DB.halt_error(2));
                        D.DB.halt_error(3) = max(D.DB.halt_error(1), D.DB.halt_error(3));
                        D.DB.halt_error(4) = D.DB.halt_error(1) + D.DB.halt_error(4);
                        D.DB.halt_error(5) = D.DB.halt_error(5) + 1;
                    end
                    
                    % Set flag
                    D.B.is_rewarding = false;
                    
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
                if any(ismember(evtStr, D.NLX.pid_run_str))
                    % Change setpoint plot color and width
                    D.UI.setPosCol = D.UI.activeCol;
                    D.UI.setPosLineWidth = 4;
                end
                % Stopped
                if any(ismember(evtStr, D.NLX.pid_stop_str))
                    % Change setpoint plot color
                    D.UI.setPosCol = D.UI.robNowCol;
                    D.UI.setPosLineWidth = 2;
                end
                
                %% CHECK FOR BULLDOZE
                
                % Running
                if any(ismember(evtStr, D.NLX.bull_run_str))
                    % Change guard plot color and width
                    D.UI.guardPosCol = D.UI.activeCol;
                    D.UI.guardPosLineWidth = 10;
                    
                    % Track count
                    D.C.bull_cnt = D.C.bull_cnt + 1;
                end
                % Stopped
                if any(ismember(evtStr, D.NLX.bull_stop_str))
                    % Change guard plot color
                    D.UI.guardPosCol = D.UI.robNowCol;
                    D.UI.guardPosLineWidth = 5;
                end
                
            end
            
        end
        
        % -----------------------ROTATION TRIGGER CHECK----------------------------
        
        function [] = Rotation_Trig_Check()
            
            % Check if rotation trigger button has been pressed
            if ~get(D.UI.btnICR, 'UserData')
                return
            end
            
            % Check if rat in rotation bounds
            check_inbound = Check_Rad_Bnds(D.P.Rat.rad, D.UI.rotBndNext);
            if ~any(check_inbound)
                return
            end
            
            % Set flag
            D.B.rotated = true;
            
            % Save current image
            rotLast = D.I.rot;
            
            % Get new image
            D.I.rot = find([1, 2] ~=  D.I.rot);
            
            % Update D.AC.data(2) and send command to rotate image
            D.AC.data(2) = D.I.img_ind(D.I.rot);
            fwrite(tcpIP,D.AC.data,'int8');
            
            % Post NLX event: rotaion *deg
            NlxSendCommand(D.NLX.rot_evt{D.I.rot});
            
            % Change plot marker size
            % active feeder
            set(D.UI.fdH(3, D.I.rot), ...
                'MarkerSize', 25);
            % inactive feeder
            set(D.UI.fdH(3, [1, 2] ~=  D.I.rot), ...
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
            set(D.UI.ptchFdH, ...
                'EdgeColor', [0, 0, 0], ...
                'FaceAlpha', 0.05, ...
                'EdgeAlpha',0.025)
            % active feeder
            set(D.UI.ptchFdH(:, D.I.rot), ...
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
        
        % ------------------------REWARD SEND CHECK-------------------------------
        
        function [] = Reward_Send_Check()
            
            % Bail for manual training, already reset or no new data
            if D.PAR.sesCond == 'Manual_Training' || ...
                    D.B.flag_rew_send_crossed || ...
                    all(isnan(D.P.Rat.rad))
                return
            end
            
            % Check if rat is in quad
            check_inbound = Check_Rad_Bnds(D.P.Rat.rad, D.UI.rewRstBnds(D.I.rot,:));
            if ~any(check_inbound)
                return
            end
            
            % Print reset bounds crossed
            Console_Write('[Reward_Send_Check] Crossed Reset Bounds');
            
            % Check if next reward is cued
            D.B.is_cued_rew = ...
                D.PAR.cueFeed == 'All' || ...
                (D.PAR.cueFeed == 'Half' && D.C.rew_cross_cnt == 0) || ...
                (D.PAR.cueFeed == 'Half' && ~D.B.is_cued_rew);
            
            % Check if cue should be forced
            if D.B.do_all_cue && D.B.is_cued_rew == false
                D.B.is_cued_rew = true;
            elseif D.B.do_block_cue && D.B.is_cued_rew == true
                D.B.is_cued_rew = false;
            end
            
            % Disable cue buttons
            Set_Cue_Buttons('Disable');
            
            % Get new reward data if rat triggered reward on last lap
            if D.B.is_cued_rew && ...
                    (D.B.flag_rew_confirmed || D.C.rew_cross_cnt == 0)
                
                % Get new reward zone
                D.I.zone = find(D.PAR.zoneLocs == ...
                    D.I.zoneArr(sum([D.C.rew_cnt{:}])+1));
                
                % Get new reward duration
                D.PAR.rewDur = D.PAR.zoneRewDur(D.I.zone);
                
                % Post NLX event: cue on
                NlxSendCommand(D.NLX.cue_on_evt);
            end
            
            % Reset patches
            set(D.UI.ptchFdH(:, D.I.rot), ...
                'EdgeColor', [0, 0, 0], ...
                'FaceAlpha', 0.15, ...
                'EdgeAlpha', 0.05);
            % Clear last duration
            set(D.UI.durNowTxtH(:, :), ...
                'Visible', 'off');
            
            % Check if this is cued reward
            if D.B.is_cued_rew
                
                % Will send CS command with pos and zone
                reward_pos_send = D.UI.rewRatHead(D.I.rot);
                zone_ind_send = D.I.zone;
                rew_delay_send = 0;
                
                % Show new reward taget patch
                set(D.UI.ptchFdH(D.I.zone, D.I.rot), ...
                    'EdgeColor', D.UI.activeCol, ...
                    'FaceAlpha', 0.75, ...
                    'EdgeAlpha', 1);
                
                % Print new duration
                set(D.UI.durNowTxtH(D.I.zone, D.I.rot), ...
                    'Visible', 'on');
            else
                
                % Send reward center and no zone ind
                reward_pos_send = D.UI.rewRatHead(D.I.rot);
                zone_ind_send = 0;
                rew_delay_send = D.PAR.rewDel;
                
                % Darken all zone patches
                set(D.UI.ptchFdH(:, D.I.rot), ...
                    'FaceAlpha', 0.75)
            end
            
            % Send reward command
            SendM2C('R', reward_pos_send, zone_ind_send, rew_delay_send);
            
            % Set flags
            D.B.flag_rew_send_crossed = true;
            D.B.check_rew_confirm = true;
            D.B.flag_rew_zone_crossed = false;
            D.B.flag_rew_confirmed = false;
            
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
        
        % -----------------------------REWARD CHECK-----------------------------
        
        function [] = Reward_Check()
            
            %% CHECK FOR REWARD CONFIRMATION
            
            % Bail for manual training
            if D.PAR.sesCond == 'Manual_Training'
                return
            end
            
            % Check if reward has been reset
            if  D.B.check_rew_confirm && ...
                    c2m_Z ~= 0 && ...
                    ~D.B.flag_rew_confirmed
                
                % Get rewarded zone ind
                D.I.zone = c2m_Z;
                
                % Post NLX event: cue off
                if D.B.is_cued_rew
                    NlxSendCommand(D.NLX.cue_off_evt);
                end
                
                % Post NLX event: reward info
                NlxSendCommand(...
                    sprintf(D.NLX.rew_evt, D.PAR.zoneLocs(D.I.zone), D.PAR.zoneRewDur(D.I.zone)));
                
                % Add to total reward count
                if D.B.rotated
                    D.C.rew_cnt{D.I.rot}(end) = D.C.rew_cnt{D.I.rot}(end) + 1;
                else
                    D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
                end
                
                % Store reward zone with range [-20,20]
                D.I.zoneHist(sum([D.C.rew_cnt{:}])) = -1*D.PAR.zoneLocs(D.I.zone);
                
                % Reset reward zone patches
                set(D.UI.ptchFdH(:, D.I.rot), ...
                    'EdgeColor', [0, 0, 0], ...
                    'FaceAlpha', 0.15, ...
                    'EdgeAlpha', 0.05);
                % Lighten rewarded zone
                set(D.UI.ptchFdH(D.I.zone, D.I.rot), ...
                    'FaceAlpha', 0.5, ...
                    'EdgeAlpha', 0.5);
                % Print new duration
                set(D.UI.durNowTxtH(D.I.zone, D.I.rot), ...
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
                    Get_Rad_Bnds(D.UI.rewFeedRad(D.I.rot) + deg2rad(avg_trig + D.PAR.trigDist));
                D.UI.zoneAvgH = ...
                    plot(xbnd, ybnd, ...
                    'Color', D.UI.rotCol(D.I.rot,:), ...
                    'LineStyle', '-', ...
                    'LineWidth', 1, ...
                    'Parent',D.UI.axH(2));
                uistack(D.UI.zoneAvgH, 'bottom');
                
                % Set flags
                D.B.flag_rew_confirmed = true;
                D.B.check_rew_confirm = false;
                c2m_Z = 0;
                D.B.plot_rew = true;
                
                Console_Write(sprintf('[Reward_Check] Rewarded: Zone=%d Vel=%0.2fcm/sec', ...
                    D.I.zoneHist(sum([D.C.rew_cnt{:}])), D.P.Rat.vel));
                
            end
            
            %% CHECK IF ALL ZONES PASSED OR CONFIRMATION RECEIVED
            
            % Track reward crossing
            if ~D.B.flag_rew_zone_crossed
                
                % Check if rat has passed all zones or reward confirmed
                check_inbound = Check_Rad_Bnds(D.P.Rat.rad, D.UI.rewPassBnds(D.I.rot,:));
                
                if ~(any(check_inbound) || D.B.flag_rew_confirmed)
                    return
                end
                
                % Set flags
                D.B.flag_rew_zone_crossed = true;
                D.B.flag_rew_send_crossed = false;
                D.B.check_rew_confirm = false;
                
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
                if ~D.B.flag_rew_confirmed
                    
                    % Add to missed reward count
                    D.C.missed_rew_cnt(1) = D.C.missed_rew_cnt(1)+1;
                    
                    % Cue next lap
                    if D.PAR.cueFeed == 'Half'
                        D.B.is_cued_rew = true;
                    end
                    
                    % Reset reward zone patches
                    set(D.UI.ptchFdH(:, D.I.rot), ...
                        'EdgeColor', [0, 0, 0], ...
                        'FaceAlpha', 0.15, ...
                        'EdgeAlpha', 0.05);
                    set(D.UI.durNowTxtH(:, D.I.rot), ...
                        'Visible', 'off');
                    
                    % Print missed reward
                    Console_Write(sprintf('[Reward_Check] Detected Missed Reward: cross_cnt=%d miss_cnt=%d|%d', ...
                        D.C.rew_cross_cnt, D.C.missed_rew_cnt(1), D.C.missed_rew_cnt(2)));
                    
                end
                
                % Set reset patch to visible
                set(D.UI.ptchFdRstH(D.I.rot), ...
                    'FaceAlpha', 0.5);
                
                % Update reward info list
                rew_ellapsed = Elapsed_Seconds() - D.T.rew_last;
                D.T.rew_last = Elapsed_Seconds();
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
                Console_Write('[Reward_Check] Crossed Reward Bounds');
            end
            
        end
        
        % ----------------------------LAP CHECK------------------------------------
        
        function [] = Lap_Check()
            
            %% CHECK FOR BOUNDS CROSSING
           
            % Check if rat in lap quad bound
            track_quad = Check_Rad_Bnds(D.P.Rat.rad, D.UI.lapBnds(D.I.lap_hunt_ind, :));
            
            % If in quad bounds
            if ~any(track_quad)
                return
            end
            
            % Set specific bound bool to true
            D.B.check_inbound_lap(D.I.lap_hunt_ind) = true;
            
            % Set later quads to false to prevent backwards runs
            D.B.check_inbound_lap(1:4 > D.I.lap_hunt_ind) = false;
            
            % Set patches
            set(D.UI.ptchLapBnds(D.B.check_inbound_lap), 'Visible', 'off');
            set(D.UI.ptchLapBnds(~D.B.check_inbound_lap), 'Visible', 'on');
            
            % Update current quadrant
            if any(D.B.check_inbound_lap)
                D.I.lap_hunt_ind = find(~D.B.check_inbound_lap, 1, 'first');
            else
                D.I.lap_hunt_ind = 1;
            end
            
            % Bail if all bounds not crossed
            if ~all(D.B.check_inbound_lap)
                return
            end
            
            % Update lap count
            if D.B.rotated
                D.C.lap_cnt{D.I.rot}(end) = D.C.lap_cnt{D.I.rot}(end) + 1;
            else
                D.C.lap_cnt{3}(end) = D.C.lap_cnt{3}(end) + 1;
            end
            
            % Reset lap track bool
            D.B.check_inbound_lap = false(1,4);
            
            % Reset lap quad index
            D.I.lap_hunt_ind = 1;
            
            % Set all back to dark
            set(D.UI.ptchLapBnds, 'Visible', 'on');
            
            %% PLOT CUMULATIVE VT DATA
            
            % Delete all tracker data from this lap
            delete(D.UI.ratPltH(isgraphics(D.UI.ratPltH)))
            delete(D.UI.ratPltHvel(isgraphics(D.UI.ratPltHvel)))
            delete(D.UI.robPltHvel(isgraphics(D.UI.robPltHvel)))
            
            % Rat pos
            delete(D.UI.Rat.pltHposAll);
            x = D.P.Rat.pos_hist(:,1);
            y = D.P.Rat.pos_hist(:,2);
            exc = find(diff(x)/D.UI.xCMcnv > 10 | diff(y)/D.UI.yCMcnv > 10 == 1) + 1;
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
            lap_tim_ellapsed = Elapsed_Seconds() - D.T.lap_tim;
            
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
            D.T.lap_tim = Elapsed_Seconds();
            
        end
        
        % --------------------------PLOT POSITION----------------------------------
        
        function [] = Pos_Plot()
            
            %% ROBOT POS
            
            if D.B.Rob.plot_pos
                
                % Plot rob patch
                if isfield(D.UI, 'ptchRobPos');
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
                if isfield(D.UI, 'vtRobPltNow');
                    delete(D.UI.vtRobPltNow);
                end
                D.UI.vtRobPltNow = ...
                    plot(D.P.Rob.x(end), D.P.Rob.y(end), 'o', ...
                    'MarkerFaceColor', D.UI.robNowCol, ...
                    'MarkerEdgeColor', D.UI.robNowCol, ...
                    'MarkerSize', 10, ...
                    'Parent', D.UI.axH(2));
                
                % Plot rob arm
                if isfield(D.UI, 'armPltNow');
                    delete(D.UI.armPltNow);
                end
                [xbnd, ybnd] =  Get_Rad_Bnds([D.P.Rob.feedRad, D.P.Rob.buttRad]);
                D.UI.armPltNow = ...
                    plot(xbnd(1,:), ybnd(1,:), ...
                    'Color', D.UI.robNowCol, ...
                    'LineWidth', 5, ...
                    'Parent',D.UI.axH(2));
                
                % Plot guard pos
                if isfield(D.UI, 'quardPltNow');
                    delete(D.UI.quardPltNow);
                end
                [xbnd, ybnd] =  Get_Rad_Bnds(D.P.Rob.guardRad);
                D.UI.quardPltNow = ...
                    plot(xbnd, ybnd, ...
                    'Color', D.UI.guardPosCol, ...
                    'LineWidth', D.UI.guardPosLineWidth, ...
                    'Parent',D.UI.axH(2));
                
                % Plot set pos
                if isfield(D.UI, 'setPltNow');
                    delete(D.UI.setPltNow);
                end
                [xbnd, ybnd] =  Get_Rad_Bnds(D.P.Rob.setRad);
                D.UI.setPltNow = ...
                    plot(xbnd, ybnd, ...
                    'Color', D.UI.setPosCol, ...
                    'LineWidth', D.UI.setPosLineWidth, ...
                    'Parent',D.UI.axH(2));
                
                % Plot feeder pos feed in cond color
                if isfield(D.UI, 'feedPltNow');
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
            
            if D.B.Rat.plot_pos
                
                % Plot all new VT data
                Hind = find(~isgraphics(D.UI.ratPltH), 1, 'first');
                D.UI.ratPltH(Hind) = ...
                    plot(D.P.Rat.x, D.P.Rat.y, '.', ...
                    'MarkerFaceColor', D.UI.ratPosAllCol, ...
                    'MarkerEdgeColor', D.UI.ratPosAllCol, ...
                    'MarkerSize', 6, ...
                    'Parent', D.UI.axH(1));
                
                % Plot current rat position with larger marker
                if isfield(D.UI, 'vtRatPltNow');
                    delete(D.UI.vtRatPltNow);
                end
                D.UI.vtRatPltNow = ...
                    plot(D.P.Rat.x(end), D.P.Rat.y(end), 'o', ...
                    'MarkerFaceColor', D.UI.ratNowCol, ...
                    'MarkerEdgeColor', [0, 0, 0], ...
                    'MarkerSize', 10, ...
                    'Parent', D.UI.axH(2));
                
                % Plot last reward point
                if any(D.B.plot_rew)
                    plot(D.P.Rat.x(end), D.P.Rat.y(end), 'o', ...
                        'MarkerFaceColor', D.UI.rotCol(D.I.rot,:), ...
                        'MarkerEdgeColor', [0, 0, 0], ...
                        'MarkerSize', 8, ...
                        'Parent', D.UI.axH(1));
                    
                    % Reset flag
                    D.B.plot_rew = false;
                    
                end
                
            end
            
            %% VELOCITY
            
            % ROB
            Hind = find(~isgraphics(D.UI.robPltHvel), 1, 'first');
            if D.B.Rob.plot_vel
                D.UI.robPltHvel(Hind) = ...
                    plot(D.P.Rob.vel_pol_arr(:,1), D.P.Rob.vel_pol_arr(:,2), '-', ...
                    'Color', D.UI.robNowCol, ...
                    'LineWidth', 2, ...
                    'Parent', D.UI.axH(1));
            end
            
            % RAT
            Hind = find(~isgraphics(D.UI.ratPltHvel), 1, 'first');
            if D.B.Rat.plot_vel
                D.UI.ratPltHvel(Hind) = ...
                    plot(D.P.Rat.vel_pol_arr(:,1), D.P.Rat.vel_pol_arr(:,2), '-', ...
                    'Color', D.UI.ratNowCol, ...
                    'LineWidth', 2, ...
                    'Parent', D.UI.axH(1));
            end
            
            %% HEADING
            
            % Plot arrow
            if D.B.plot_hd && D.B.Rat.plot_pos
                if any(~isnan(D.P.Rat.hd_rad))
                    
                    % Delete plot object
                    if isfield(D.UI, 'vtPltHD');
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
            end
            
        end
        
        % -------------------------PRINT SES INFO----------------------------------
        
        function [] = Inf_Print()
            
            %% Print performance info
            
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
            % Turn red and flicker if bellow 12 V
            if c2m_J ~= 0
                volt_now = c2m_J/10;
                if volt_now <= D.PAR.batVoltWarning && volt_now > 0
                    set(D.UI.txtPerfInf(7), 'ForegroundColor', D.UI.warningCol);
                    if strcmp(get(D.UI.txtPerfInf(7), 'Visible'), 'on')
                        set(D.UI.txtPerfInf(7), 'Visible', 'off')
                    else
                        set(D.UI.txtPerfInf(7), 'Visible', 'on')
                    end
                else
                    set(D.UI.txtPerfInf(7), 'ForegroundColor', D.UI.enabledPrintFrgCol);
                end
                infstr = sprintf('Battery:_%0.1fV', volt_now);
                set(D.UI.txtPerfInf(7), 'String', infstr)
            end
            
            %% Print time info
            
            % Get session time
            nowTim(1) =  Elapsed_Seconds() - D.T.ses_str_tim;
            
            % Get recording elapsed time plus saved time
            if  D.B.rec
                nowTim(2) = (Elapsed_Seconds() - D.T.rec_tim)  + D.T.rec_tot_tim;
            else nowTim(2) = D.T.rec_tot_tim; % keep showing save time
            end
            
            % Get lap time
            if D.B.rat_in
                nowTim(3) = Elapsed_Seconds() - D.T.lap_tim;
            else nowTim(3) = 0;
            end
            
            % Run time
            if D.B.rat_in && ~D.B.rec_done
                D.T.run_tim = Elapsed_Seconds() - D.T.run_str;
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
            if D.T.loop > 0;
                D.T.loop_min = min(D.T.loop, D.T.loop_min);
            end
            D.T.loop_max = max(D.T.loop, D.T.loop_max);
            infstr = sprintf( ...
                [...
                'RD: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                'RT: %4.0f  mn:%4.0f  mx:%4.0f av:%4.0f\n', ...
                'HE: %4.0f  mn:%4.0f  mx:%4.0f av:%2.2f\n', ...
                'Lp: %4.0f  mn:%4.0f  mx:%4.0f\n', ...
                ], ...
                D.DB.rew_duration(1), D.DB.rew_duration(2), D.DB.rew_duration(3), D.DB.rew_duration(4)/D.DB.rew_duration(5), ...
                D.DB.rew_round_trip(1), D.DB.rew_round_trip(2), D.DB.rew_round_trip(3), D.DB.rew_round_trip(4)/D.DB.rew_round_trip(5), ...
                D.DB.halt_error(1), D.DB.halt_error(2), D.DB.halt_error(3), D.DB.halt_error(4)/D.DB.halt_error(5), ...
                D.T.loop*1000, D.T.loop_min*1000, D.T.loop_max*1000 ...
                );
            set(D.UI.txtTimDebug, 'String', infstr)
            
        end
        
        % ------------------------------RUN TEST CODE---------------------------------
        
        function [] = Run_Test()
            if D.DB.isTestRun
                
                %% PID CALIBRATION
                if D.DB.doPidCalibrationTest && ...
                        ~D.B.pidStarted && ...
                        c2m_K ==1
                    % Start pid test
                    SendM2C('T', sysTest, 0);
                    % Set flag to start pid
                    D.B.pidStarted = true;
                end
                
                %% HALT ERROR TEST
                if D.DB.doHaultErrorTest
                    
                    % Check if robot should be restarted
                    if D.DB.isHalted && ...
                            ~D.B.is_haulted && ...
                            Elapsed_Seconds() - D.DB.t_halt > D.DB.haltDur
                        
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
                                D.DB.doHaultErrorTest = false;
                                
                                % Save data to csv file
                                fi_out = fullfile(D.DIR.ioTestOut,'halt_error.csv');
                                file_id = fopen(fi_out,'w');
                                fprintf(file_id,'Err, Min, Max, Avg, Vel\r');
                                fprintf(file_id,D.DB.halt_error_str(3:end));
                                fclose(file_id);
                                % Log/print
                                Console_Write(sprintf('[Run_Test] Saved Hault Error Test: %s', ...
                                    fi_out));
                            end
                            
                            % Log/print
                            Console_Write(sprintf('[Run_Test] Hault Error Test: New Vel=%dcm/sec', ...
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
                            Elapsed_Seconds() - D.DB.t_halt > D.DB.haltDur+1 && ...
                            any(Check_Rad_Bnds(D.P.Rob.rad, [deg2rad(355), deg2rad(360)]));
                        
                        % Incriment counter
                        D.DB.haltCnt = D.DB.haltCnt+1;
                        
                        % Tell CS to Halt Robot
                        SendM2C('T', sysTest, 0);
                        
                        % Store robots current pos and time
                        D.DB.sendPos = D.P.Rob.radLast;
                        D.DB.t_halt = Elapsed_Seconds();
                        
                        % Set flag
                        D.DB.isHalted = true;
                        
                        % Log/print
                        Console_Write('[Run_Test] Hault Error Test: Halting Robot');
                    end
                    
                end
                
                %% SIMULATED RAT TEST
                if D.DB.doSimRatTest
                    
                    % Wait till recording
                    if D.B.rec
                        
                        % Wait for 5 sec after setup
                        if Elapsed_Seconds() - D.T.rec_tim > 5
                            
                            % Local vars
                            cm = 0;
                            vel_now = 0;
                            
                            % Start rat in start quad
                            if D.B.initVals
                                D.B.simRadLast = mean(D.UI.strQuadBnds);
                                D.B.simVelLast = 0;
                                D.B.simTSStart = Elapsed_Seconds();
                                D.B.simTSLast = 0;
                                D.B.initVals = false;
                            end
                            
                            % Compute ts(us) from dt(s)
                            ts_now = (Elapsed_Seconds() - D.B.simTSStart)*10^6;
                            dt_sec = (ts_now - D.B.simTSLast) / 10^6;
                            D.B.simTSLast = ts_now;
                            
                            % Get slider val
                            sld_vel = round(get(D.B.UI.sld, 'Value'));
                            set(D.B.UI.txt, 'String', num2str(sld_vel));
                            
                            % Update vel if not halted or holding for 2 sec for setup
                            if ...
                                    D.B.rat_in && ...
                                    ~D.B.is_haulted && ...
                                    Elapsed_Seconds() - D.T.run_str > 2
                             
                                    % Check vel
                                    if D.B.simVelLast == sld_vel
                                        % Hold velocity
                                        vel_now = D.B.simVelLast;
                                    elseif D.B.simVelLast < sld_vel
                                        % Accelerate
                                        vel_now = D.B.simVelLast + (D.DB.ratMaxAcc*dt_sec);
                                    elseif D.B.simVelLast > sld_vel
                                        % Deccelerate
                                        vel_now = D.B.simVelLast - (D.DB.ratMaxDec*dt_sec);
                                    end
                                    
                                    % Keep in bounds
                                    if vel_now > D.B.UI.sld.Max
                                        vel_now = D.B.UI.sld.Max;
                                    elseif vel_now < D.B.UI.sld.Min
                                        vel_now = D.B.UI.sld.Min;
                                    end
                            else
                                % Keep robot haulted
                                vel_now = 0;
                            end
                            
                            % Compute new pos
                            if D.B.rat_in && ...
                                    ~isnan(vel_now) && ~isnan(cm) && ~isnan(dt_sec)
                                % Get delta pos
                                cm = vel_now * dt_sec;
                                rad_diff = cm / ((140 * pi)/(2 * pi));
                            else
                                % Use old pos
                                rad_diff = 0;
                            end
                            rad_now = D.B.simRadLast - rad_diff;
                            
                            % Convert rad back to cart
                            rad = wrapTo2Pi(rad_now);
                            rad = abs(rad - 2*pi);
                            rad = wrapToPi(rad);
                            roh = ones(length(rad), 1) - (D.UI.trkWdt/2/D.UI.arnRad);
                            [x_norm,y_norm] = pol2cart(rad, roh);
                            x = (x_norm.*D.PAR.R) + D.PAR.XC;
                            y = (y_norm.*D.PAR.R) + D.PAR.YC;
                            y = y*(10/11);
                            xy_pos(:,1) = x;
                            xy_pos(:,2) = y;
                            xy_pos = reshape(xy_pos', 1, []);
                            
                            % Update globals
                            m2c_posSim(1) = ts_now;
                            m2c_posSim(2) = x;
                            m2c_posSim(3) = y;
                            
                            % Update simulated rat data
                            D.B.simRadLast = rad_now;
                            D.B.simVelLast = vel_now;
                            D.P.Rat.vtTS = single(ts_now);
                            D.P.Rat.vtPos = single(xy_pos);
                            D.P.Rat.vtNRecs = single(1);
                            
                            % Run VT_Proc('Rat');
                            VT_Proc('Rat');
                            
                        end
                        
                    end
                    
                end
                
            end
        end
        
        
        
        
        
        
        %% ======================== CALLBACK FUNCTIONS ============================
        
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
            Console_Write(sprintf('[%s] Set to \"%s\"', 'PopCond', char(D.PAR.catCueCond)));
            
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
            
        end
        
        % CUE CONDITION
        function [] = ToggCue(~, ~, ~)
            
            % Get trigger button
            cue_cond = get(gcbo, 'UserData');
            
            % Check if callback was run without button press
            if isempty(cue_cond)
                cue_cond = find(cell2mat(get(D.UI.toggCue, 'Value')) == 1);
            end
            
            % Change button
            if get(D.UI.toggCue(cue_cond), 'Value') == 1
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
            end
            
            % Change Reward delay
            rew_del = str2double(D.UI.delList{get(D.UI.popRewDel,'Value'),:});
            if (D.PAR.cueFeed == 'Half' || ...
                    D.PAR.cueFeed == 'None') && ...
                    D.PAR.sesCond ~= 'Manual_Training' && ...
                    rew_del == 0
                
                % Set reward delay
                set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.delList, D.UI.delList(3))));
                % run callback
                PopRewDel();
                
            elseif D.PAR.cueFeed == 'All' && ...
                    rew_del ~= 0
                
                % 'All' should have 0 delay
                % Set reward delay
                set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.delList, D.UI.delList(2))));
                % run callback
                PopRewDel();
                
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%s\"', 'ToggCue', char(D.PAR.cueFeed)));
            
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
            D.B.setup = true;
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnSetupDone', get(D.UI.btnSetupDone,'Value')));
            
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
                if D.B.acq
                    % save out time before stopping
                    D.T.acq_tot_tim = (Elapsed_Seconds() - D.T.acq_tim) + D.T.acq_tot_tim;
                end
                
                % Change aquiring status
                D.B.acq = ~D.B.acq;
                
                % Reset temp now
                D.T.acq_tim = Elapsed_Seconds();
                
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnAcq', get(D.UI.btnAcq,'Value')));
            
        end
        
        % REC BUTTON
        function [] = BtnRec(~, ~, ~)
            
            % Start aquisition
            if get(D.UI.btnRec,'Value') == 1
                NlxSendCommand('-StartRecording');
                set(D.UI.btnRec,'CData',D.UI.radBtnCmap{3})
            else NlxSendCommand('-StopRecording');
                set(D.UI.btnRec,'CData',D.UI.radBtnCmap{1})
            end
            
            % Set time tracking variables
            if  D.B.rec
                % save out time before stopping
                D.T.rec_tot_tim = (Elapsed_Seconds() - D.T.rec_tim) + D.T.rec_tot_tim;
            end
            D.B.rec = ~ D.B.rec;
            D.T.rec_tim = Elapsed_Seconds();
            
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
                    ~D.B.rec_done
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
            
            % Set bool
            D.B.rec_done = true;
            
            % Save end time
            D.T.ses_end_tim = Elapsed_Seconds();
            
            % Print end time
            infstr = datestr(D.T.ses_end_tim, 'HH:MM:SS');
            set(D.UI.editTimEndInf, 'String', infstr)
            
            % Halt robot if not already halted
            if strcmp(get(D.UI.btnHaltRob, 'Enable'), 'on')
                if (~D.B.is_haulted)
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
            
            % Tell CS rat is out
            SendM2C('I', 0)
            
            % Stop recording
            if D.B.rec
                set(D.UI.btnRec,'Value', 0)
                BtnRec(D.UI.btnRec);
            end
            
            % Make sure main loop ends
            D.B.rat_in = false;
            
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
            set(D.UI.btnClrVT, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            % Disable toggSnd buttons
            set(D.UI.toggSnd, 'Enable', 'off')
            
            % Disable inf panels
            set(D.UI.panSesInf, 'HighlightColor', D.UI.disabledBckCol)
            set(D.UI.panPerfInf, 'HighlightColor', D.UI.disabledBckCol)
            set(D.UI.panTimInf, 'HighlightColor', D.UI.disabledBckCol)
            
            % Stop halt if still active
            if (D.B.is_haulted)
                set(D.UI.btnHaltRob, 'Value', 0);
                BtnHaltRob();
            end
            
            % Stop bulldoze if still active
            if (get(D.UI.btnBulldoze, 'Value') == 1)
                set(D.UI.btnBulldoze, 'Value', 0);
                Bulldoze();
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnRecDone', get(D.UI.btnRecDone,'Value')));
            
        end
        
        % SAVE SESSION DATA
        function [] = BtnSave(~, ~, ~)
            
            % Disable save button so only pressed once
            set(D.UI.btnSave, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            
            % Set flag to save at end of main loop
            D.B.do_save = true;
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnSave', get(D.UI.btnSave,'Value')));
            
        end
        
        % QUIT ALL
        function [] = BtnQuit(~, ~, ~)
            
            % Check for save unless quit before setup done
            if ~D.UI.save_done && D.B.setup
                
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
            end
            
            % Print session aborting
            if ~D.UI.save_done
                Console_Write('!!ERROR!! [BtnQuit] ABORTING SESSION...');
            end
            
            % Disable
            % quit button
            set(D.UI.btnQuit, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledBckCol);
            
            % Stop recording
            if D.B.rec
                set(D.UI.btnRec,'Value', 0)
                BtnRec(D.UI.btnRec);
            end
            
            % Make sure main loop ends
            if D.B.rat_in
                D.B.rat_in = false;
                % Tell CS rat is out
                SendM2C('I', 0)
            end
            
            % Stop halt if still active
            if (D.B.is_haulted)
                set(D.UI.btnHaltRob, 'Value', 0);
                BtnHaltRob();
            end
            
            % Stop bulldoze if still active
            if (get(D.UI.btnBulldoze, 'Value') == 1)
                set(D.UI.btnBulldoze, 'Value', 0);
                Bulldoze();
            end
            
            % Check if battery should be replaced
            if (c2m_J > 0 && c2m_J <= D.PAR.batVoltReplace)
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
            D.B.quit = true;
            
            % Shut down if matlab run alone
            if isMatSolo
                c2m_E = 1;
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnQuit', get(D.UI.btnQuit,'Value')));
            
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
                D.B.is_haulted = true;
            else
                % Change backround color and text
                set(D.UI.btnHaltRob, ...
                    'String', 'Halt Robot', ...
                    'BackgroundColor', D.UI.enabledCol);
                % Tell CS to stop halting
                SendM2C('H', 0);
                % Set flag
                D.B.is_haulted = false;
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnHaltRob', get(D.UI.btnHaltRob,'Value')));
            
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
            
        end
        
        % REWARD
        function [] = BtnReward(~, ~, ~)
            
            % Tell CS to trigger reward
            reward_pos_send = 0;
            zone_ind_send = find(D.PAR.zoneRewDur == max(D.PAR.zoneRewDur));
            reward_delay_send = 0;
            SendM2C('R', reward_pos_send, zone_ind_send, reward_delay_send);
            
            % Track round trip time
            D.T.manual_rew_sent = Elapsed_Seconds();
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnReward', get(D.UI.btnReward,'Value')));
            
        end
        
        % BLOCK CUE
        function [] = BlockCue(~, ~, ~)
            
            % Dont run if reward sent
            if ~D.B.flag_rew_send_crossed
                
                if (get(D.UI.btnBlockCue, 'Value') == 1)
                    
                    % Change backround color and text
                    set(D.UI.btnBlockCue, ...
                        'BackgroundColor', D.UI.activeCol);
                    
                    % Set main flag
                    D.B.do_block_cue = true;
                    
                    % Set cue maker to not visible
                    set(reshape(D.UI.ptchFdH,1,[]), ...
                        'EdgeColor', [0, 0, 0]);
                    
                    % Set flag to false
                    D.B.is_cued_rew = false;
                    
                    % Make sure other button is inactivated
                    if  D.B.do_all_cue
                        set(D.UI.btnAllCue, 'Value',0);
                        AllCue();
                    end
                else
                    % Change backround color and text
                    set(D.UI.btnBlockCue, ...
                        'BackgroundColor', D.UI.enabledCol);
                    
                    % Unset main flag
                    D.B.do_block_cue = false;
                end
                
            else
                % Set back to what previous state
                set(D.UI.btnBlockCue, 'Value', ~get(D.UI.btnBlockCue, 'Value'))
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnBlockCue', get(D.UI.btnBlockCue,'Value')));
            
        end
        
        % ALL CUE
        function [] = AllCue(~, ~, ~)
            
            % Dont run if reward sent
            if ~D.B.flag_rew_send_crossed
                
                if (get(D.UI.btnAllCue, 'Value') == 1)
                    % Change backround color and text
                    set(D.UI.btnAllCue, ...
                        'BackgroundColor', D.UI.activeCol);
                    
                    % Set main flag
                    D.B.do_all_cue = true;
                    
                    % Set flag to true
                    D.B.is_cued_rew = true;
                    
                    % Make sure other button is inactivated
                    if D.B.do_block_cue
                        set(D.UI.btnBlockCue, 'Value',0);
                        BlockCue();
                    end
                    
                else
                    % Change backround color and text
                    set(D.UI.btnAllCue, ...
                        'BackgroundColor', D.UI.enabledCol);
                    
                    % Unset flag
                    D.B.do_all_cue = false;
                end
                
            else
                % Set back to what previous state
                set(D.UI.btnAllCue, 'Value', ~get(D.UI.btnAllCue, 'Value'))
            end
            
            % Log/print
            Console_Write(sprintf('[%s] Set to \"%d\"', 'BtnAllCue', get(D.UI.btnAllCue,'Value')));
            
        end
        
        % CLEAR VT BUTTON
        function [] = BtnClrVT(~, ~, ~)
            
            % Delete all handle objects
            delete(D.UI.ratPltH(isgraphics(D.UI.ratPltH)))
            delete(D.UI.ratPltHvel(isgraphics(D.UI.ratPltHvel)))
            delete(D.UI.robPltHvel(isgraphics(D.UI.robPltHvel)))
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
            
        end
        
        
        
        
        
        
        %% ========================== MINOR FUNCTIONS =============================
    
        % ---------------------------COMPUTE RAD DIFF------------------------------
        function [rad_diff] = Rad_Diff(rad1, rad2)
            rad_diff = min(2*pi - abs(rad1-rad2), abs(rad1-rad2));
        end
        
        % ---------------------------GET TRACK BOUNDS------------------------------
        function [xbnd, ybnd] = Get_Rad_Bnds(polbnds)
            
            if (length(polbnds) == 2)
                if polbnds(1) > polbnds(2)
                    polbnds = wrapToPi(polbnds);
                end
                radDist = min(2*pi - abs(polbnds(2)-polbnds(1)), abs(polbnds(2)-polbnds(1)));
                nPoints =  round(360 * (radDist/(2*pi))); % 360 pnts per 2*pi
                polbnds = linspace(polbnds(1), polbnds(2), nPoints);
            end
            % inner bounds 0 deg
            [x(1,:),y(1,:)] = pol2cart(polbnds, ones(1,length(polbnds)) * (D.UI.arnRad-D.UI.trkWdt));
            xbnd(1,:) = x(1,:)*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
            ybnd(1,:) = y(1,:)*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
            % outer bounds 0 deg
            [x(2,:),y(2,:)] = pol2cart(polbnds, ones(1,length(polbnds)) * D.UI.arnRad);
            xbnd(2,:) = x(2,:)*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
            ybnd(2,:) = y(2,:)*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
            
        end
        
        % --------------------------CHECK TRACK BOUNDS-----------------------------
        function [bool_arr] = Check_Rad_Bnds(rad_arr, polbnds)
            
            if all(isnan(rad_arr))
                bool_arr = false(size(rad_arr));
            else
                if polbnds(1) > polbnds(2)
                    polbnds = wrapToPi(polbnds);
                    rad_arr = wrapToPi(rad_arr);
                end
                bool_arr = rad_arr > polbnds(1) & rad_arr < polbnds(2);
            end
            
        end
        
        % ---------------------------GET BUTTON COLLOR-----------------------------
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
      
        % --------------------ENABLE DISABLE CUE BUTTONS---------------------------
        function [] = Set_Cue_Buttons(setting)
            
            if D.PAR.cueFeed == 'Half' || D.PAR.cueFeed == 'None'
                if strcmp(setting, 'Enable')
                    set(D.UI.btnBlockCue, ...
                        'BackgroundColor', D.UI.enabledCol, ...
                        'ForegroundColor' , D.UI.enabledBtnFrgCol);
                    set(D.UI.btnAllCue, ...
                        'BackgroundColor', D.UI.enabledCol, ...
                        'ForegroundColor' , D.UI.enabledBtnFrgCol);
                elseif strcmp(setting, 'Disable')
                    set(D.UI.btnBlockCue, 'BackgroundColor', D.UI.disabledBckCol);
                    set(D.UI.btnAllCue, 'BackgroundColor', D.UI.disabledBckCol);
                end
                
                % Log/print
                Console_Write(sprintf('[Cue_Buttons] Set Cue Buttons to \"%s\"', setting));
            end
        end
        
        
        
        
        
        
        %% ======================= SAVE SESSION DATA ==============================
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
            D.SS_Out_ICR.(D.PAR.ratLab).Start_Time{rowInd} = datestr(D.T.ses_str_tim, 'HH:MM:SS');
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






%% =======================TOP LEVEL FUNCTIONS==============================

% --------------------------SEND CS COMMAND--------------------------------

    function[] = SendM2C(id, dat1, dat2, dat3, pack)
        
        % Local var
        persistent pack_last;
        if isempty(pack_last); pack_last = 0; end
        
        % Check if unset
        if isempty(m2c_pack)
            m2c_pack = 0; 
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
        
        % Make sure last message recieved
        t_start = Elapsed_Seconds();
        while m2c_pack ~= 0 && ...
                ~isMatSolo && ...
                c2m_C==0 && ....
                Elapsed_Seconds() < t_start + 1
            drawnow;
            pause(0.1);
        end;
        if m2c_pack ~= 0
            m2c_pack = 0; 
            Console_Write(sprintf('WARNING [SendM2C] Reset m2c_pack: id=%s dat1=%2.2f dat2=%2.2f dat3=%2.2f dt=%dms', ...
            id, dat1, dat2, dat3, round((Elapsed_Seconds() - t_start)*1000)));
        end 
        
        % Set mesage ID and dat
        m2c_id = id;
        m2c_dat1 = dat1;
        m2c_dat2 = dat2;
        m2c_dat3 = dat3;
        
        % Get new packet number
        if nargin < 5
        m2c_pack = pack_last+1;
        pack_last = pack_last+1;
        else
            m2c_pack = pack;
        end
        
        % Store in history
        m2c_hist(ismember(m2c_hist(:,1), id), 2:end) = {dat1, dat2, dat3, m2c_pack};
        
        % Log/print
        Console_Write(sprintf('   [SENT] m2c: id=%s dat1=%2.2f dat2=%2.2f dat3=%2.2f pack=%d', ...
            id, dat1 ,dat2, dat3, m2c_pack));
        
    end

% ---------------------------CHECK FOR NEW C2M-----------------------------
    function [] = CheckC2M(~,~)
        
        % Local vars
        persistent id_list;
        persistent id_last;
        
        % Bail if D deleted
        if ~exist('D', 'var')
            return;
        end
        
        % Get list of c2m vars
        if isempty(id_list) || isempty(id_last)
            var_list = who;
            c2m_ind = ...
                cell2mat(cellfun(@(x) ~isempty(strfind(x, 'c2m_')), var_list, 'uni', false));
            c2m_ind = find(c2m_ind == 1);
            % Track c2m vals
            id_last = zeros(1,length(c2m_ind));
            id_list = cell(1,length(c2m_ind));
            for z_v = 1:length(c2m_ind)
                id_list{z_v} =  var_list{c2m_ind(z_v)};
                % Set values if mat running alone
                if isMatSolo
                    eval(sprintf('%s = 0;', var_list{c2m_ind(z_v)}));
                end
            end
            % Bail on first run
            return;
        end
        
        % Loop through and check each var
        now_val = [];
        for i = 1:length(id_list)
            eval(sprintf('now_val = %s;', id_list{i}));
            if (now_val ~= id_last(i)) %#ok<BDSCI>
                Console_Write(sprintf('   [RCVD] c2m: id=%s dat=%d', id_list{i}, now_val));
                id_last(i) = now_val;
            end
        end
        
        % Check for resend request
        if ~ischar(request_m2c)
            return
        end
        
        if ~strcmp(request_m2c, ' ')
            Console_Write(sprintf('   [RCVD] Send Request: id=%s', request_m2c));
            id_ind = ismember(m2c_hist(:,1), request_m2c);
            
            % Send again if any history of this id
            if m2c_hist{id_ind,5} > 0
                SendM2C(m2c_hist{id_ind,1}, m2c_hist{id_ind,2}, m2c_hist{id_ind,3}, m2c_hist{id_ind,4}, m2c_hist{id_ind,5});
            end
            
            % Reset flag
            request_m2c = ' ';
        end
        
    end

% ---------------------------PRINT TO CONSOLE------------------------------
    function [] = Console_Write(str)

        % Itterate log count
        if ~exist('D','var'); return; end;
        D.DB.logCount = D.DB.logCount+1;
        
        % Get time
        t_s = Elapsed_Seconds();
        t_m = round(t_s*1000);

        % Add to strings
        p_msg = sprintf('\r%0.2fs %s', t_s, str);
        l_msg = sprintf('[%d],%d,%s\r', D.DB.logCount, t_m, str);
  
        % Store string
        if ~isfield(D, 'DB'); return; end;
        if ~isfield(D.DB, 'consoleStr'); return; end;
        
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
        if ~isfield(D, 'UI'); return; end;
        if ~isfield(D.UI, 'listConsole'); return; end;
        if ~isvalid(D.UI.listConsole); return; end;
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
        if ~D.B.setup
            drawnow;
        end
        
    end

% ---------------------------GET TIME NOW-------------------------------
    function [t_sec] = Elapsed_Seconds()
        
        % Convert from days to seconds
        t_sec = (now-D.DB.startTime)*24*60*60;
        
    end

% ------------------------Disconnect From NetCom---------------------------
    function [] = Disconnect_NLX()
        
        % End NLX polling
        D.B.poll_nlx = false;
        
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
                    while NlxAreWeConnected() == 1 && c2m_E == 0
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
                                fwrite(tcpIP,D.AC.data,'int8');
                                pause(0.01);
                                
                                % Send command to terminate run
                                D.AC.data = zeros(1, length(D.AC.data));
                                fwrite(tcpIP,D.AC.data,'int8'); 
                                
                                % Close AC computer connection
                                fclose(tcpIP); 
                                
                                % Show status disconnected
                                Console_Write(sprintf('[Disconnect_AC] FINISHED: Disconnect from AC Computer IP=%s', ...
                                    D.AC.IP));
                                
                            else Console_Write('**WARNING** [Disconnect_AC] \"tcpIP\" Does Not Exist');
                            end
                        else Console_Write('**WARNING** [Disconnect_AC] \"tcpIP\" is Not a tcpIP Object');
                        end
                    else Console_Write('**WARNING** [Disconnect_AC] \"tcpIP\" Does Not Exist');
                    end
                else Console_Write('**WARNING** [Disconnect_AC] \"D.AC\" is Empty');
                end
            else Console_Write('**WARNING** [Disconnect_AC] \"AC\" Not a Field of \"D\"');
            end
            %try fclose(tcpIP); catch; end;
            %try delete(tcpIP); catch; end;
            %try clear tcpIP; catch; end;
            
        end
    end

% ------------------------------FORCE QUIT---------------------------------
    function [] = ForceClose(~, ~, ~)
        
        % Dont run if global vars already deleted
        if size(who('global'),1) == 0
            return;
        end
        
        % Dont run if gui closed in correct sequence
        if D.B.close
            return;
        end
        
        % Log/print
        Console_Write('WARNING [ForceClose] RUNNING: ForceClose Exit Procedure...');
        
        % Disconnect AC computer
        Disconnect_AC();
        % Disconnect from NetCom
        Disconnect_NLX()
        
        % Set flags
        c2m_E = 1;
        D.DB.isForceClose = true;
        
        return;
    end






end