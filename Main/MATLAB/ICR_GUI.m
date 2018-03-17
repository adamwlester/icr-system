function[status] = ICR_GUI(SYSTEST, BREAKDEBUG, DOAUTOLOAD, DOPROFILE, ISMATSOLO)
% INPUT:
%
%	SYSTEST = [0:8, 0:8]
%    	0: No test
%    	1: Simulated rat test
%     	2: PID calibration
%   	3: VT calibration
%       4: Halt error test
%   	5: Wall image IR sync timing
%   	6: IR sync timing
%   	7: Robot hardware test
%   	8: Cube battery test
%
%   BREAKDEBUG = [0:n_lines]
%       [0]: Dont break on errors
%     	[1]: Break on errors
%     	[>1]: Break on line
%
%   DOAUTOLOAD = [true,false]
%     	true: Load rat data based on ICR_GUI hardcoded values
%       false: Start normally
%
%   ISMATSOLO = [true,false]
%     	true: Matlab running alone
%       false: Matlab running with other programs
%
%   DOPROFILE = [true,false]
%     	true: Run with profiler
%       false: Run without profiler






%% ============================== TOP LEVEL RUN ===========================

% ----------------------------- SETUP GLOBALS -----------------------------

% Matlab globals
global ME; % error handeling
global DOEXIT; % exit flag
global FORCECLOSE; % forced close flag
global ISCRASH; % crash flag
global FIGH; % ui figure group
global D; % main data struct
global TCPIP; % tcp object
global STATUS; % store status
global UPDATENOW; % update ui imediately
global TIMSTRLOCAL; % local time
global DTHANDSHAKE; % handshake time

% Matlab to CS communication
global m2c; % global data struct
global m2c_pack; % message out to CS
global m2c_dir; % current cheetah directory

% CS to Matlab communication
global c2m; % local data struct
global c2m_com; % message in from CS

% Initialize globals
TIMSTRLOCAL = now;
DTHANDSHAKE = 0;
ME = [];
DOEXIT = false;
FORCECLOSE = false;
ISCRASH = false;
UPDATENOW = true;
STATUS = 'failed';

% ------------------------ HANDLE INPUT ARGUMENTS -------------------------

% Set inputs
if nargin < 5
    ISMATSOLO = false;
end
if nargin < 4
    DOPROFILE = false;
end
if nargin < 3
    DOAUTOLOAD = false;
end
if nargin < 2
    BREAKDEBUG = 1;
end
if nargin < 1
    SYSTEST = 0;
end

%---------------------SET DEBUG/TEST PARAMETERS----------------------------

% AUTOLOAD PARAMETERS

% Rat
D.DB.ratLab = 'r0495'; %'r9999';

% Implant status
D.DB.Implanted = true;

% Session Type, Condition and Task
D.DB.Session_Type = 'TT_Turn' ; % ['ICR_Session' 'TT_Turn' 'Table_Update']
D.DB.Session_Condition = 'Implant_Training'; % ['Manual_Training' 'Behavior_Training' 'Implant_Training' 'Rotation']
D.DB.Session_Task = 'Track'; % ['Track' 'Forage']

% Other
D.DB.Feeder_Condition = 'C1'; % ['C1' 'C2']
D.DB.Reward_Delay = '3.0'; % ['0.0 ' '1.0 ' '2.0' '3.0']
D.DB.Cue_Condition = 'None'; % ['All' 'Half' 'None']
D.DB.Sound_Conditions = [1,1]; % [0 1]
D.DB.Rotation_Direction = 'CW'; % ['CCW' 'CW']
D.DB.Start_Quadrant = 'SE'; % ['NE' 'SE' 'SW' 'NW'];
D.DB.Rotation_Positions = [180,180,180,90,180,270,90,180,270]; % [90 180 270];

% HARDCODED FLAGS
D.DB.doForagePathCompute = false;
D.DB.doAutoSetTT = false;
D.DB.autoSetTTturns = 4;

% SYSTEM SET FLAGS
D.DB.isTestRun = true;
D.DB.t1_doSimRatTest = false;
D.DB.t2_doPidCalibrationTest = false;
D.DB.t3_doVTCalibrationTest = false;
D.DB.t4_doHaltErrorTest = false;
D.DB.t5_doWallIRTimingTest = false;
D.DB.t6_doSyncIRTimingTest = false;
D.DB.t7_doRobotHardwareTest = false;
D.DB.t8_doCubeBatteryTest = false;

% SIMULATED RAT TEST SETTINGS

% Starting velocity
D.DB.SIM.VelStart = 20; % (cm/sec)
% Max acc
D.DB.SIM.MaxAcc = 100; % (cm/sec/sec)
% Max dec
D.DB.SIM.MaxDec = 100; % (cm/sec/sec)
% Pause for reward dt
D.DB.SIM.dtRewPause = 5; % (sec)

% VT CALIBRATION TEST SETTINGS

% Run time
D.DB.CALVT.RunDur = 2; % (min)
% Robot vel
D.DB.CALVT.RobVel = 50; % (cm/sec)

% HALT ERROR TEST SETTINGS

% Velocities to step through
D.DB.HALT.VelSteps = 10:10:80; % (cm/sec)
% Number of trials for each step
D.DB.HALT.StepTrials = 4;

% WALL IMAGE IR TIMING TEST

% Number of image trials
D.DB.WALIR.imgTrials = 10; % 10
% Number of pulse trials
D.DB.WALIR.pulseTrials = 20; % 20
% Period between image change
D.DB.WALIR.DTImg = 1; % (sec) % 1
% Period between ir pulses
D.DB.WALIR.DTPulse = 100; % (ms) % 100
% Period between LED pulses
D.DB.WALIR.DTSensor = 10; % (sec) % 10

% SYNC IR TIMING TEST

% Number of pulse trials
D.DB.SYNCIR.pulseTrials = 100; % 20
% Period between ir pulses
D.DB.SYNCIR.DTPulse = 100; % (ms) % 100

% CUBE BATTERY TEST
% Notification percent steps
D.DB.CVCC.prcSteps = 95:-5:5; %95:-5:5;

%----------------SETUP DEBUGGING AND ERROR HANDELING-----------------------

% PROFILING CODE
if exist('DOPROFILE', 'var')
    if DOPROFILE
        profile on
    end
end

% SETUP ERROR HANDELING

% Reset last error
lasterror('reset'); %#ok<LERR>

% Setup error handeling
if BREAKDEBUG == 0
    % Catch and print error in console
    dbclear if caught error;
elseif BREAKDEBUG == 1
    % Stop on error
    dbstop if error;
else
    % Stop on error
    dbstop if error;
    % Stop on line
    eval(sprintf('dbstop at %d in ICR_GUI',BREAKDEBUG));
end

% DEBUG VARS AND FLAGS

% Console and log vars
D.DB.consoleStr = [repmat(' ',1000,150),repmat('\r',1000,1)];
D.DB.logStr = cell(1000,1);
D.DB.consoleCount = 0;
D.DB.logCount = 0;

% Set test flags
if length(SYSTEST)==1
    SYSTEST(2) = 0;
end

% Simulated rat test
if any(SYSTEST == 1)
    D.DB.t1_doSimRatTest = true;
end

% PID calibration
if any(SYSTEST == 2)
    D.DB.t2_doPidCalibrationTest = true;
end

% VT calibration
if any(SYSTEST == 3)
    D.DB.t3_doVTCalibrationTest = true;
end

% Halt error test
if any(SYSTEST == 4)
    D.DB.t4_doHaltErrorTest = true;
end

% Wall image IR sync timing
if any(SYSTEST == 5)
    D.DB.t5_doWallIRTimingTest = true;
end

% IR sync timing
if any(SYSTEST == 6)
    D.DB.t6_doSyncIRTimingTest = true;
end

% Robot hardware test
if any(SYSTEST == 7)
    D.DB.t7_doRobotHardwareTest = true;
end

% Cube battery test
if any(SYSTEST == 8)
    D.DB.t8_doCubeBatteryTest = true;
end

% No test
if all(SYSTEST == 0)
    D.DB.isTestRun = DOAUTOLOAD;
end

% Log print debug conditions
if ...
        any(SYSTEST == 0) > 0 || ...
        BREAKDEBUG > 0 || ...
        DOAUTOLOAD || ...
        DOPROFILE || ...
        ISMATSOLO
    Console_Write('RUN MODE = DEBUG');
else
    Console_Write('RUN MODE = DEBUG');
end

% Log print input arguments
Console_Write(sprintf('INPUT ARGUMENTS (%d): SYSTEST=|%d|%d| BREAKDEBUG=%d DOAUTOLOAD=%d DOPROFILE=%d ISMATSOLO=%d', ...
    nargin, SYSTEST, BREAKDEBUG, DOAUTOLOAD, DOPROFILE, ISMATSOLO));

%---------------------Important variable formats---------------------------
%...........................D.F.sound....................................
%   val 1 = White Noise [true, false]
%   val 2 = Reward Tone [true, false]
%...........................D.AC.data......................................
%   val 1 = conection [0, 1], [no, yes]
%   val 2 = display image [0, 1, 2, 3], [Close all, 0-deg, -40-deg, 40-deg]
%   val 3 = sound state [0, 1, 2], [no sound, white only, all sound]

if BREAKDEBUG > 0
    
    % ------------------------- RUN IN DEBUG MODE -------------------------
    
    % RUN MAIN SETUP
    if ~DOEXIT
        Setup();
    else
        Console_Write('**WARNING** [ICR_GUI] SKIPPED: ICR/Setup');
    end
    
    % RUN MAIN RUN
    if ~DOEXIT
        Run();
    else
        Console_Write('**WARNING** [ICR_GUI] SKIPPED: ICR/Run');
    end
    
    % RUN MAIN EXIT
    if ~ISCRASH
        Exit();
    else
        try
            Exit();
        catch ME
        end
    end
    
    % SET STATUS
    if ~ISCRASH
        STATUS = 'finished';
    else
        STATUS = 'crashed';
    end
    
else
    
    % ---------------------- RUN IN CATCH ERROR MODE ----------------------
    
    % RUN MAIN SETUP
    try
        if ~DOEXIT
            Setup();
        else
            Console_Write('**WARNING** [ICR_GUI] SKIPPED: ICR/Setup');
        end
    catch ME
        SetExit()
    end
    
    % RUN MAIN RUN
    try
        if ~DOEXIT && isempty(ME)
            Run();
        else
            Console_Write('**WARNING** [ICR_GUI] SKIPPED: ICR/Run');
        end
    catch ME
        SetExit()
    end
    
    % RUN MAIN EXIT
    try
        if isempty(ME)
            Exit();
        end
        
        % SET STATUS
        if ~ISCRASH && isempty(ME)
            STATUS = 'finished';
        else
            STATUS = 'crashed';
        end
        
    catch ME
        SetExit()
    end
    
end

% HANDLE ERRRORS
if ~isempty(ME)
    
    % Log/print error
    err = sprintf([ ...
        'Time: %0.2f\r\n', ...
        'ID: %s\r\n', ...
        'Msg: \"%s\"\r\n', ...
        'Stack: '], ...
        Sec_DT(now), ...
        ME.identifier, ...
        ME.message);
    for z_line = 1:length(ME.stack)
        err = [err, ...
            sprintf('\n   %s [%d]', ...
            ME.stack(z_line).name, ...
            ME.stack(z_line).line)]; %#ok<AGROW>
    end
    err = err(1:end);
    err_print = [sprintf('!!!!!!!!!!!!!ERROR!!!!!!!!!!!!!\r\n'), err, ...
        sprintf('\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n')];
    Console_Write(err_print);
    
    % Store status
    err_status = regexprep(err, '\r\n', ' ');
    err_status = regexprep(err_status, '\s*', ' ');
    STATUS = err_status;
    
    % SET STATUS
elseif strcmp(STATUS, 'finished')
    STATUS = 'succeeded';
end

% STOP PROFILER
if exist('DOPROFILE', 'var')
    if DOPROFILE
        profile viewer
    end
end

% RUN CLEAR AND CLOSE ALL
ClearCloseAll(true);

% COPY AND CLEAR STATUS
status = STATUS;
clear STATUS;

% PRINT FINAL STATUS
fprintf('\nRETURN STATUS: \n%s\n', status);

% PRINT RUN ENDED
fprintf('\n################# REACHED END OF RUN #################\n');






%% =========================== TOP LEVEL FUNCTION =========================

% -----------------------------MAIN SETUP----------------------------------
    function[] = Setup()
        
        % Start of setup
        Console_Write('[Setup] BEGIN: Setup()');
        
        % ---------------------------- SET PARAMETERS ---------------------
        
        % MAIN RUN PARAMETERS
        
        % Poll rate
        D.PAR.pollRate = 0.03; % (sec)
        % Interval between polling entities
        D.PAR.dtPoll = 0.01; % (sec)
        % Min dt loop
        D.PAR.dtMinLoop = 10; % (ms)
        % Min time in start quad
        D.PAR.strQdDel = 0.5; % (sec)
        % PID setPoint
        D.PAR.setPointBackpack = 60;
        D.PAR.setPointImplant = 66;
        D.PAR.setPointCM = D.PAR.setPointBackpack;
        D.PAR.setPointRad = D.PAR.setPointCM * ((2 * pi)/(140 * pi));
        % Robot guard dist
        D.PAR.guardDistRad = 4.5 * ((2 * pi)/(140 * pi));
        % Robot butt dist
        D.PAR.buttDistRad = 18 * ((2 * pi)/(140 * pi));
        % Feeder dist from rob tracker
        D.PAR.feedDistRad = 70 * ((2 * pi)/(140 * pi));
        % Sleep 1/2 duration
        D.PAR.sleepDur = [15, 15]*60; % (min)
        % Max tetrodes
        D.PAR.maxTT = 18;
        % Max clusters
        D.PAR.maxClust = 10;
        % Warning battery voltage level
        D.PAR.robVccWarning = 11.6; % (V)
        % Replace battery voltage level
        D.PAR.robVccReplace = 11.8; % (V)
        % Warning battery voltage level
        D.PAR.cubeVccWarning = 30; % (%)
        % Cube battery type ["C": Small, "A": Medium]
        D.PAR.cubeBatteryType = 'A';
        
        % DIRECTORIES
        
        % Top directory
        D.DIR.top = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running';
        
        % Test output
        D.DIR.ioTestOut = fullfile(D.DIR.top,'Testing','Output');
        
        % IO top
        D.DIR.ioTop = fullfile(D.DIR.top,'IOfiles');
        
        % IO images
        D.DIR.ioImg = fullfile(D.DIR.ioTop,'Images');
        % IO wall images
        D.DIR.wallImage = fullfile(D.DIR.ioImg,'plot_images','wall_top_down.png');
        % IO paxinos images
        D.DIR.paxDat = fullfile(D.DIR.ioImg, 'Paxinos', 'procPax.mat');
        D.DIR.paxImg{1} = fullfile(D.DIR.ioImg, 'Paxinos', 'sag_lab');
        D.DIR.paxImg{2} = fullfile(D.DIR.ioImg, 'Paxinos', 'cor_lab');
        D.DIR.paxImg{3} = fullfile(D.DIR.ioImg, 'Paxinos', 'hor_lab');
        
        % IO tables
        D.DIR.SS_IO_1 = fullfile(D.DIR.ioTop, 'SessionData', 'SS_IO_1.mat');
        D.DIR.SS_IO_2 = fullfile(D.DIR.ioTop, 'SessionData', 'SS_IO_2.mat');
        D.DIR.SS_IO_3 = fullfile(D.DIR.ioTop, 'SessionData', 'SS_IO_3.mat');
        D.DIR.TT_IO_1 = fullfile(D.DIR.ioTop, 'SessionData', 'TT_IO_1.mat');
        D.DIR.TT_IO_2 = fullfile(D.DIR.ioTop, 'SessionData', 'TT_IO_2.mat');
        
        % IO datasets
        D.DIR.trkBnds = fullfile(D.DIR.ioTop, 'Operational', 'track_bounds.mat');
        D.DIR.frgPath = fullfile(D.DIR.ioTop, 'Operational', 'forage_path.mat');
        
        % Cheetah dirs
        D.DIR.nlxCheetaEXE = 'C:\Program Files\Neuralynx\Cheetah';
        D.DIR.nlxSS3DEXE = 'C:\Program Files (x86)\Neuralynx\SpikeSort3D';
        D.DIR.nlxCfg = 'C:\Users\Public\Documents\Cheetah\Configuration';
        D.DIR.nlxTempTop = 'C:\CheetahData\Temp';
        D.DIR.nlxSaveTop = 'E:\BehaviorPilot';
        D.DIR.recFi = '0000-00-00_00-00-00';
        
        % Log dirs
        D.DIR.logFi = 'ICR_GUI_Log.csv';
        D.DIR.logTempDir = fullfile(D.DIR.nlxTempTop,'0000-00-00_00-00-00');
        m2c_dir = D.DIR.logTempDir;
        
        % ------------------------- SETUP TOP LEVEL VARS ------------------
        
        % COM STRUCTS
        
        % m2c struct
        id_list = [ ...
            'i', ... % handshake request [NA]
            'p', ... % simulation data [ts, x, y]
            'G', ... % session type [1:3]
            'A', ... % connected to AC computer [NA]
            'N', ... % netcom setup [NA]
            'F', ... % data saved [NA]
            'X', ... % confirm quit abort [abort]
            'C', ... % confirm close
            'T', ... % system test command [test]
            'S', ... % setup session [ses_cond, task_cond, sound_cond]
            'M', ... % move to position [targ_pos]
            'R', ... % run reward [rew_pos, rew_cond, zone_ind/rew_delay]
            'H', ... % halt movement [halt_state]
            'B', ... % bulldoze rat [bull_delay, bull_speed]
            'I' ... % rat in/out [in/out]
            'O' ... % confirm task done
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
            'h', ... % setup handshake
            'J', ... % battery voltage
            'Z', ... % reward zone
            'K', ... % robot status
            'Y', ... % task done
            'E', ... % exit
            'C' ... % confirm close
            ];
        for z_id = 1:length(id_list)
            c2m.(id_list(z_id)) = cell2struct( ...
                {id_list(z_id), 0, 0, 0, 0, 0, 0}, ...
                {'id', 'dat1', 'dat2', 'dat3', 'pack', 'packLast', 't_rcvd'}, 2);
        end
        
        % c2m struct shared with cs
        c2m_com = c2m;
        
        % Bypass some things if running solo
        if ISMATSOLO
            c2m.('K').dat1 = 3;
            c2m.('Y').dat1 = 1;
            c2m.('C').dat1 = 1;
        end
        
        % Setup c2m com check timer
        D.timer_c2m = timer;
        D.timer_c2m.ExecutionMode = 'fixedRate';
        D.timer_c2m.Period = 0.1;
        D.timer_c2m.TimerFcn = @TimerFcn_CheckCSCom;
        
        % Dynamic graphics timer
        D.timer_graphics = timer;
        D.timer_graphics.ExecutionMode = 'fixedRate';
        D.timer_graphics.Period = 0.25;
        D.timer_graphics.TimerFcn = {@TimerFcn_Graphics, 'Other'};
        
        % Setup save timer
        D.timer_save = timer;
        D.timer_save.ExecutionMode = 'fixedRate';
        D.timer_save.Period = 0.5;
        D.timer_save.TimerFcn = {@TimerFcn_Graphics, 'Save'};
        D.timer_save.StopFcn = {@TimerStop_BtnStatus, 'Save'};
        
        % Setup quit timer
        D.timer_quit = timer;
        D.timer_quit.ExecutionMode = 'fixedRate';
        D.timer_quit.Period = 0.5;
        D.timer_quit.TimerFcn = {@TimerFcn_Graphics, 'Quit'};
        D.timer_quit.StopFcn = {@TimerStop_BtnStatus, 'Quit'};
        
        % ------------------------- SETUP VARS AND UI ---------------------
        
        % Generate figure tab group
        FIGH = figure('Visible', 'Off', ...
            'DeleteFcn', {@DeleteFcn_ForceClose});
        
        % Start c2m timer
        Console_Write('[Setup] RUNNING: Start "timer_c2m"');
        start(D.timer_c2m);
        
        % Create log dir if none exists
        if ~exist(D.DIR.logTempDir, 'dir')
            Console_Write(sprintf('[Setup] RUNNING: Make Temp Log Directory: "%s"', D.DIR.logTempDir));
            mkdir(D.DIR.logTempDir);
        end
        
        % Run variable setup code
        Console_Write('[Setup] RUNNING: "Var_Setup()"...');
        Var_Setup();
        Console_Write('[Setup] FINISHED: "Var_Setup()"');
        
        % Run UI setup code
        Console_Write('[Setup] RUNNING: "UI_Setup()"...');
        UI_Setup();
        Console_Write('[Setup] FINISHED: "UI_Setup()"');
        
        % Enable setup panel and load popup
        Object_Group_State('Setup_Objects', 'Enable')
        
        % Run testing setup
        Console_Write('[Setup] RUNNING: "Test_Setup()"...');
        was_ran = Test_Setup();
        if was_ran
            Console_Write('[Setup] FINISHED: "Test_Setup()"');
        else
            Console_Write('[Setup] SKIPPED: "Test_Setup()"');
        end
        
        % Wait for ses type selection
        Console_Write('[Setup] RUNNING: Wait for Session Type Selection...');
        while true
            [abort, pass] = ...
                Check_Flag(DOEXIT, D.F.ses_type_confirmed);
            if abort || pass; break; end
        end
        
        % Check status
        if abort
            Console_Write('**WARNING** [Setup] ABORTED: Wait for Session Type Selection');
            return
        else
            Console_Write('[Setup] FINISHED: Wait for Session Type Selection');
        end
        
        % Send sesion type info to CS
        Send_CS_Com('G', ...
            D.PAR.sesType == 'ICR_Session', ...
            D.PAR.sesType == 'TT_Turn', ...
            D.PAR.sesType == 'Table_Update');
        
        % ------------------------ SETUP AC CONNECTION --------------------
        Console_Write('[Setup] RUNNING: Connect to AC Computer...');
        
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
        
        % Initialize flag
        D.F.ac_connected = false;
        
        % Skip if updating table
        if D.PAR.sesType ~= 'Table_Update'
            
            % Stop c2m timer
            if strcmp(D.timer_c2m.Running, 'on')
                stop(D.timer_c2m);
            end
            
            % Create tcpip object
            TCPIP = tcpip('0.0.0.0',55000, ...
                'OutputBufferSize', length(D.AC.data), ...
                'NetworkRole','Server', ...
                'Timeout', 1);
            
            % Establish connection
            if strcmp('ICRCHEETAH', getenv('computername'))
                fopen(TCPIP);
                D.F.ac_connected = true;
            end
            
            % Restart timer
            start(D.timer_c2m);
            
            % Print that AC computer is connected
            if (D.F.ac_connected)
                Console_Write(sprintf('[Setup] FINISHED: Connect to AC Computer IP=%s', ...
                    D.AC.IP));
            else
                Console_Write(sprintf('**WARNING** [Setup] ABORTED: Connect to AC Computer IP=%s', ...
                    D.AC.IP));
            end
            
        else
            Console_Write(sprintf('[Setup] SKIPPED: Connect to AC Computer IP=%s', ...
                D.AC.IP));
        end
        
        % Enable setup objects
        Object_Group_State('Setup_Objects', 'Enable')
        
        % Send to CS
        Send_CS_Com('A');
        
        % -------------------------- SETUP HANDSHAKE ----------------------
        
        % Tell CS ready for handshake
        Send_CS_Com('i');
        
        % Wait for sync time
        Console_Write('[Setup] RUNNING: Wait for Handshake...');
        if ~ISMATSOLO
            while true
                [abort, pass] = ...
                    Check_Flag(DOEXIT, c2m.('h').dat1 ~= 0);
                if abort || pass; break; end
            end
            
            % Check status
            if abort
                Console_Write('**WARNING** [Setup] ABORTED: Wait for Handshake');
                return
            elseif pass
                Console_Write('[Setup] FINISHED: Wait for Handshake');
                
                % Log/print sync time
                Console_Write(sprintf('SET SYNC TIME: %dms', round(DTHANDSHAKE*1000)));
            end
            
        end
        
        % End of setup
        Console_Write('[Setup] END: Setup()');
        
    end

% ------------------------------MAIN RUN-----------------------------------
    function[] = Run()
        
        % Start main run
        Console_Write('[Run] BEGIN: Run()');
        
        % ---------------------------SETUP & RUN----------------------------------
        
        while ~DOEXIT
            
            % THROTTLE LOOP
            while true
                dt_loop =  (Sec_DT(now) - D.T.loop) * 1000;
                [abort, pass] = ...
                    Check_Flag(DOEXIT, dt_loop>=D.PAR.dtMinLoop);
                if abort || pass; break; end
            end
            
            % TRACK LOOP TIME
            if (D.T.loop > 0)
                D.DB.loop = Update_DB_DT(D.DB.loop, Sec_DT(now));
            end
            D.T.loop = Sec_DT(now);
            
            % UPDATE GUI
            if ~D.F.ui_updated
                Update_UI(0, 'limitrate');
            end
            D.F.ui_updated = false;
            
            % CHECK FOR EXIT
            if DOEXIT
                continue
            end
            
            % PLOT POSITION
            VT_Plot_New();
            
            % PLOT TT DATA
            TT_Plot();
            
            % CHECK HARDWARE
            Hardware_Check();
            
            % PRINT SES INFO
            Inf_Print();
            
            % RUN TESTS
            Test_Run();
            
            % --------------------CHECK WHAT TO DO---------------------
            
            % CHECK FOR EXIT
            if DOEXIT
                continue
            end
            
            % Store last case
            D.F.main_case_last = D.F.main_case_now;
            
            % Handle Flow
            if ~D.F.ses_data_loaded
                D.F.main_case_now = 'WAIT FOR UI SETUP';
                
                % Finish Setup
            elseif ~D.F.do_quit && ~D.F.ses_setup_done
                D.F.main_case_now = 'FINISH SESSION SETUP';
                
                % Run Task
            elseif  (~D.F.task_done_confirmed || ~all(D.F.sleep_done)) && ...
                    ~(D.PAR.sesType == 'TT_Turn' || D.PAR.sesType == 'Table_Update')
                D.F.main_case_now = 'RUN ICR TASK';
                
                % Wait for save
            elseif ~D.F.do_quit && ~D.F.ses_save_done
                D.F.main_case_now = 'WAIT FOR SAVE';
                
            elseif ~D.F.do_quit
                D.F.main_case_now = 'WAIT FOR QUIT';
                
            else
                D.F.main_case_now = 'WAIT FOR EXIT';
                
            end
            
            % PRINT CHANGE IN FLOW
            if ~strcmp(D.F.main_case_now, D.F.main_case_last)
                Console_Write(sprintf('[Run:MainLoop] SWITCH FROM "%s" TO "%s"', D.F.main_case_last, D.F.main_case_now));
            end
            
            % HANDLE CASE
            switch D.F.main_case_now
                
                
                case 'WAIT FOR UI SETUP'
                    %% ---------------WAIT FOR UI SETUP----------------
                    continue
                    
                case 'FINISH SESSION SETUP'
                    %% -----------------FINISH SESSION SETUP-------------------
                    
                    % Flag setup done flag
                    D.F.ses_setup_done = true;
                    
                    % Setup NXL
                    Console_Write('[Run:MainLoop] RUNNING: "NLX_Start()"...');
                    NLX_Start();
                    Console_Write('[Run:MainLoop] FINISHED: "NLX_Start()"');
                    
                    % Run Table Setup code
                    Console_Write('[Run:MainLoop] RUNNING: "Table_Setup()"...');
                    Table_Setup();
                    D.F.table_tab_setup = true;
                    Console_Write('[Run:MainLoop] FINISHED: "Table_Setup()"');
                    
                    % Run TT Track Setup code
                    Console_Write('[Run:MainLoop] RUNNING: "TT_Track_Setup()"...');
                    was_ran = TT_Track_Setup();
                    if was_ran
                        Console_Write('[Run:MainLoop] FINISHED: "TT_Track_Setup()"');
                        D.F.tt_tab_setup = true;
                    else
                        Console_Write('[Run:MainLoop] SKIPPED: "TT_Track_Setup()"');
                    end
                    
                    % Connect to NLX
                    Console_Write('[Run:MainLoop] RUNNING: "NLX_Connect()"...');
                    was_ran = NLX_Connect();
                    if was_ran
                        Console_Write('[Run:MainLoop] FINISHED: "NLX_Connect()"');
                    else
                        Console_Write('[Run:MainLoop] SKIPPED: "NLX_Connect()"');
                    end
                    
                    % Send NLX connected confirmation
                    Send_CS_Com('N', D.PAR.R, D.PAR.XC, D.PAR.YC);
                    
                    % Run NLX Setup
                    Console_Write('[Run:MainLoop] RUNNING: "NLX_Setup()"...');
                    was_ran = NLX_Setup();
                    if was_ran
                        Console_Write('[Run:MainLoop] FINISHED: "NLX_Setup()"');
                    else
                        Console_Write('[Run:MainLoop] SKIPPED: "NLX_Setup()"');
                    end
                    
                    % Enable TT_Track Objects
                    if D.F.implant_session
                        Object_Group_State('TT_Track_Objects', 'Enable')
                    end
                    
                    % Run Finish AC setup code
                    Console_Write('[Run:MainLoop] RUNNING: "Finish_AC_Setup()"...');
                    Finish_AC_Setup();
                    Console_Write('[Run:MainLoop] FINISHED: "Finish_AC_Setup()"');
                    
                    % Start aquisition
                    if D.F.cheetah_running
                        Safe_Set(D.UI.toggAcq, 'Value', 1)
                        Togg_Acq(D.UI.toggAcq);
                    end
                    
                    % Start recording
                    if D.F.cheetah_running && D.PAR.sesType ~= 'TT_Turn'
                        Safe_Set(D.UI.toggRec,'Value', 1);
                        Togg_Rec(D.UI.toggRec);
                    end
                    
                    % Force update ui
                    Update_UI(0, 'force');
                    
                    % Bail here if running TT Track or Table solo
                    if D.PAR.sesType == 'TT_Turn' || D.PAR.sesType == 'Table_Update'
                        
                        % Switch to 'TT TRACK' tab
                        if D.PAR.sesType == 'TT_Turn'
                            Tab_GrpChange(D.UI.tabTT);
                        end
                        
                        % Switch to 'TABLE' tab
                        if D.PAR.sesType == 'Table_Update'
                            Tab_GrpChange(D.UI.tabTBL);
                        end
                        
                        % Set rat out flag
                        D.F.rat_out = true;
                        
                        % Bail
                        continue
                    end
                    
                    % Run ICR Session setup code
                    Console_Write('[Run:MainLoop] RUNNING: "ICR_Session_Setup()"...');
                    ICR_Session_Setup();
                    Console_Write('[Run:MainLoop] FINISHED: "ICR_Session_Setup()"');
                    
                    % Send first setup info packet
                    Send_CS_Com('S', 1, D.PAR.sesMsg(1,1), D.PAR.sesMsg(1,2));
                    
                    % Send second setup info packet
                    Send_CS_Com('S', 2, D.PAR.sesMsg(2,1), D.PAR.sesMsg(2,2));
                    
                    % Enable Run panel stuff
                    if D.F.implant_session
                        Object_Group_State('Sleep1_Objects', 'Enable')
                    else
                        Object_Group_State('Task_Objects', 'Enable')
                    end
                    
                    % Enable text info
                    Object_Group_State('Text_Objects', 'Enable')
                    
                    % Show all objects on 'ICR ARENA' tab
                    Tab_GrpChange(D.UI.tabICR);
                    
                    % Force update ui
                    Update_UI(0, 'force');
                    
                case 'RUN ICR TASK'
                    %% ------------------RUN ICR TASK-------------------
                    
                    % RESET POLL STATUS FLAG
                    D.F.poll_new_vt = false;
                    
                    % POLL NLX
                    if D.F.poll_nlx
                        
                        % GET POLL DT
                        dt_evt = Sec_DT(now) - D.T.poll_last_evt;
                        dt_vt = Sec_DT(now) - D.T.poll_last_vt;
                        dt_tt = Sec_DT(now) - D.T.poll_last_tt;
                        dt_poll = min([dt_evt, dt_vt, dt_tt]);
                        
                        % POLL NEXT
                        if dt_poll >= D.PAR.dtPoll && ...
                                any([dt_evt, dt_vt, dt_tt] > D.PAR.pollRate)
                            
                            if all(dt_evt > max([dt_vt, dt_tt]))
                                
                                % GET/PROCESS NLX EVENTS
                                Evt_Get();
                                Evt_Proc();
                                D.T.poll_last_evt = Sec_DT(now);
                                
                            elseif all(dt_vt > max([dt_evt, dt_tt]))
                                
                                % GET/PROCESS NLX VT
                                VT_Get('Rob');
                                VT_Proc('Rob');
                                if ~D.DB.t1_doSimRatTest
                                    VT_Get('Rat');
                                    VT_Proc('Rat');
                                end
                                D.T.poll_last_vt = Sec_DT(now);
                                D.F.poll_new_vt = true;
                                
                            elseif all(dt_tt > max([dt_evt, dt_vt]))
                                
                                % GET/PROCESS NLX TT DATA
                                TT_Get();
                                TT_Proc();
                                D.T.poll_last_tt = Sec_DT(now);
                                
                            end
                            
                        end
                        
                    end
                    
                    % ---------------CHECK WHAT TO DO------------------
                    D.F.sub_case_last = D.F.sub_case_now;
                    
                    
                    if  ~D.F.sleep_done(1)
                        D.F.sub_case_now = 'RUN SLEEP 1';
                        
                    elseif ~D.F.task_setup
                        D.F.sub_case_now = 'SETUP TASK';
                        
                    elseif ~D.F.rob_setup
                        D.F.sub_case_now = 'WAIT FOR ROBOT SETUP';
                        
                    elseif ~D.F.matlab_streaming
                        D.F.sub_case_now = 'WAIT FOR MATLAB STREAMING';
                        
                    elseif ~D.F.rob_streaming
                        D.F.sub_case_now = 'WAIT FOR ROBOT STREAMING';
                        
                    elseif ~(D.F.task_done || D.F.do_quit)
                        
                        if ~D.F.first_move_sent && D.F.poll_new_vt
                            D.F.sub_case_now = 'CHECK IF MOVE READY';
                            
                        elseif  ~D.F.rat_in && D.F.poll_new_vt
                            D.F.sub_case_now = 'CHECK IF RAT IN';
                            
                        elseif D.F.rat_in && D.F.poll_new_vt
                            D.F.sub_case_now = 'DO TASK CHECKS';
                            
                        else
                            continue
                        end
                        
                    elseif ~D.F.rat_out
                        D.F.sub_case_now = 'SEND TASK DONE CONFIRMATION';
                        
                    elseif ~D.F.task_done_confirmed
                        D.F.sub_case_now = 'CHECK FOR TASK DONE CONFIRMATION';
                        
                    elseif ~D.F.sleep_done(2)
                        D.F.sub_case_now = 'RUN SLEEP 2';
                        
                    end
                    
                    % PRINT CHANGE IN FLOW
                    if ~strcmp(D.F.sub_case_now, D.F.sub_case_last)
                        Console_Write(sprintf('[Run:SubLoop] SWITCH FROM "%s" TO "%s"', D.F.sub_case_last, D.F.sub_case_now));
                    end
                    
                    % HANDLE CASE
                    switch D.F.sub_case_now
                        
                        case 'WAIT FOR ROBOT SETUP'
                            %% -------------WAIT FOR ROBOT SETUP-------------------
                            
                            % Bail if setup not confirmed
                            if c2m.('K').dat1 < 1
                                continue
                            end
                            
                            % Log/print
                            Console_Write('[Run:SubLoop] CONFIRMED ROBOT SETUP');
                            
                            % Set flag
                            D.F.rob_setup = true;
                            
                        case 'RUN SLEEP 1'
                            %% ------------------RUN SLEEP 1-------------------
                            
                            % Check sleep status
                            Sleep_Check(1);
                            
                        case 'SETUP TASK'
                            %% ------------------SETUP TASK--------------------
                            
                            % Enable Run objects
                            Object_Group_State('Task_Objects', 'Enable');
                            
                            % Run 'Track' task setup code
                            if D.PAR.sesTask == 'Track'
                                Console_Write('[Run:SubLoop] RUNNING: "Track_Task_Setup()"');
                                Track_Task_Setup();
                                Console_Write('[Run:SubLoop] FINISHED: "Track_Task_Setup()"');
                            end
                            
                            % Run 'Forage' task setup code
                            if D.PAR.sesTask == 'Forage'
                                Console_Write('[Run:SubLoop] RUNNING: "Forage_Task_Setup()"');
                                Forage_Task_Setup();
                                Console_Write('[Run:SubLoop] FINISHED: "Forage_Task_Setup()"');
                            end
                            
                            % Show start button
                            if strcmp(get(D.UI.btnStart, 'Enable'), 'off')
                                
                                % Enable button
                                set(D.UI.btnStart, 'Visible', 'on');
                                Button_State(D.UI.btnStart, 'Enable');
                                [xbnd, ybnd] =  Get_Cart_Bnds(mean(D.PAR.strQuadBnds));
                                
                                % Set position
                                D.UI.btnStart.Position(1:2) = ...
                                    [D.UI.axH(3).Position(1) + D.UI.axH(3).Position(3)*((mean(xbnd)-D.UI.axH(3).XLim(1))/diff(D.UI.axH(3).XLim) - D.UI.btnStart.Position(3)), ...
                                    D.UI.axH(3).Position(2) + D.UI.axH(3).Position(4)*((mean(ybnd)-D.UI.axH(3).YLim(1))/diff(D.UI.axH(3).YLim))- D.UI.btnStart.Position(4)/2];
                                
                            end
                            
                            % Set flag
                            D.F.task_setup = true;
                            
                            % Force  update UI
                            Update_UI(0, 'force');
                            
                        case 'WAIT FOR MATLAB STREAMING'
                            
                            %% ----------WAIT FOR MATLAB STREAMING-----------
                            
                            % Bail if not streaming data received
                            if ~D.F.vt_rat_streaming && ...
                                    ~D.F.vt_rat_streaming && ...
                                    ~D.F.vt_rat_streaming
                                continue
                            end
                            
                            % Log/print
                            Console_Write('[Run:SubLoop] CONFIRMED MATLAB STREAMING');
                            
                            % Set flag
                            D.F.matlab_streaming = true;
                            
                        case 'WAIT FOR ROBOT STREAMING'
                            
                            %% ----------WAIT FOR ROBOT STREAMING-----------
                            
                            % Bail if robot streaming not confirmed
                            if c2m.('K').dat1 < 2
                                continue
                            end
                            
                            % Log/print
                            Console_Write('[Run:SubLoop] CONFIRMED ROBOT STREAMING');
                            
                            % Dump initial vt recs
                            Console_Write('[Run:SubLoop] RUNNING: Dump First VT Recs...');
                            while true
                                
                                % Get recs
                                VT_Get('Rat');
                                VT_Get('Rob');
                                Evt_Get();
                                
                                % Run till no new recs
                                [abort, pass] = ...
                                    Check_Flag(DOEXIT, ...
                                    D.P.Rat.vtNRecs == 0 && ...
                                    D.P.Rat.vtNRecs == 0 && ...
                                    D.E.evtNRecs == 0);
                                if abort || pass; break; end
                                
                            end
                            
                            % Check status
                            if abort
                                Console_Write('**WARNING** [Run:SubLoop] ABORTED: Dump VT Recs');
                                return
                            elseif pass
                                Console_Write('[Run:SubLoop] FINISHED: Dump VT Recs');
                            end
                            
                            % Set flags
                            D.F.rob_streaming = true;
                            D.F.poll_nlx = true;
                            UPDATENOW = false;
                            
                            % Begin main loop
                            Console_Write('[Run:SubLoop] READY TO ROCK!');
                            
                        case 'CHECK IF MOVE READY'
                            %% ------------CHECK IF MOVE READY--------------
                            
                            % Wait till at least 1000 VT samples collected
                            if ~ISMATSOLO && ...
                                    D.P.Rob.cnt_vtRec < 100
                                continue
                            end
                            
                            % Wait for start button on forage task
                            if D.PAR.sesTask == 'Forage' && ...
                                    get(D.UI.btnStart, 'Value') == 0
                                continue
                            end
                            
                            % Itterate move count
                            D.C.move = D.C.move+1;
                            
                            % Send C# command to move robot to start quad or reward loc
                            if D.PAR.sesTask == 'Forage'
                                
                                % Move to forage reward targ
                                Send_CS_Com('M', D.C.move, deg2rad(D.PAR.frgTargDegArr(D.I.targ_now)));
                                
                                Console_Write('[Run:SubLoop] SENT STARTING MOVE TO FORAGE REWARD TARGET COMMAND');
                                
                            elseif D.PAR.sesCond == 'Manual_Training' %#ok<*STCMP>
                                
                                % Move to track reward zone
                                Send_CS_Com('M', D.C.move, D.UI.rewZoneRad(1));
                                
                                Console_Write('[Run:SubLoop] SENT STARTING MOVE TO TRACK REWARD ZONE COMMAND');
                            else
                                
                                % Move to start quad
                                Send_CS_Com('M', D.C.move, D.PAR.strQuadBnds(1));
                                Console_Write('[Run:SubLoop] SENT STARTING MOVE TO TRACK START QUADRANT COMMAND');
                                
                            end
                            
                            % Set flag
                            D.F.first_move_sent = true;
                            
                        case 'CHECK IF RAT IN'
                            %% ------------CHECK IF RAT IN------------------
                            
                            Rat_In_Check();
                            
                        case 'DO TASK CHECKS'
                            %% ----------DO TASK CHECKS----------------
                            
                            % ROTATION TRIGGER CHECK
                            Rotation_Trig_Check();
                            
                            % TRACK REWARD RESET CHECK
                            Track_Reward_Send_Check();
                            
                            % TRACK REWARD CHECK
                            Track_Reward_Zone_Check();
                            
                            % FORAGE REWARD CHECK
                            Forage_Reward_Targ_Check();
                            
                            % LAP CHECK
                            Lap_Check();
                            
                        case 'SEND TASK DONE CONFIRMATION'
                            %% --------SEND TASK DONE CONFIRMATION-----------
                            
                            % Tell CS task is finished
                            Send_CS_Com('O', 1);
                            
                            % Set flag
                            D.F.rat_out = true;
                            Console_Write('[Run:SubLoop] SENT TASK DONE CONFIRMATION');
                            
                        case 'CHECK FOR TASK DONE CONFIRMATION'
                            %% ------CHECK FOR TASK DONE CONFIRMATION-------
                            
                            % Check if CS has confirmed task done
                            if c2m.('Y').dat1 == 1
                                
                                % Set flags
                                D.F.task_done_confirmed = true;
                                UPDATENOW = true;
                                
                                % Enable Sleep 2 objects
                                if D.F.implant_session
                                    Object_Group_State('Sleep2_Objects', 'Enable');
                                end
                                
                                % Reset c2m flag
                                c2m.('Y').dat1 = 0;
                                Console_Write('[Run:SubLoop] RECEIVED TASK DONE CONFIRMATION');
                            end
                            
                        case 'RUN SLEEP 2'
                            %% ------------------RUN SLEEP 2-------------------
                            
                            % Check sleep status
                            Sleep_Check(2);
                            
                            
                        otherwise
                            continue
                            
                    end
                    
                case 'WAIT FOR SAVE'
                    %% -------------WAIT FOR SAVE---------------
                    
                    % Set Save button to active color
                    if ~D.F.do_save && strcmp(D.UI.toggSave.Enable, 'off')
                        Button_State(D.UI.toggSave, 'Enable', D.UI.attentionCol);
                    end
                    
                    % Stop graphics timer
                    if strcmp(D.timer_graphics.Running, 'on')
                        stop(D.timer_graphics);
                    end
                    
                    % Save sesion data
                    if D.F.do_save
                        Console_Write('[Run:MainLoop] SAVE INITIATED');
                        
                        % Stop recording and aquisition
                        if D.F.cheetah_running
                            if Safe_Get(D.UI.toggRec,'Value') == 1
                                Safe_Set(D.UI.toggRec,'Value', 0);
                                Togg_Rec(D.UI.toggRec);
                            end
                            if Safe_Get(D.UI.toggAcq,'Value') == 1
                                Safe_Set(D.UI.toggAcq, 'Value', 0)
                                Togg_Acq(D.UI.toggAcq);
                            end
                        end
                        
                        % Save health and general data
                        Console_Write('[Run:MainLoop] RUNNING: "Save_General_Data()"...');
                        Save_General_Data();
                        Console_Write('[Run:MainLoop] FINISHED: "Save_General_Data()"');
                        
                        % Save task data
                        Console_Write('[Run:MainLoop] RUNNING: "Save_Task_Data()"...');
                        was_ran = Save_Task_Data();
                        if was_ran
                            Console_Write('[Run:MainLoop] FINISHED: "Save_Task_Data()"');
                        else
                            Console_Write('[Run:MainLoop] SKIPPED: "Save_Task_Data()"');
                        end
                        
                        % Save Cheetah data
                        Console_Write('[Run:MainLoop] RUNNING: "Save_TT_Track_Data()"...');
                        was_ran = Save_TT_Track_Data();
                        if was_ran
                            Console_Write('[Run:MainLoop] FINISHED: "Save_TT_Track_Data()"');
                        else
                            Console_Write('[Run:MainLoop] SKIPPED: "Save_TT_Track_Data()"');
                        end
                        
                        % Save Cheetah data
                        Console_Write('[Run:MainLoop] RUNNING: "Save_Cheetah_Data()"...');
                        was_ran = Save_Cheetah_Data();
                        if was_ran
                            Console_Write('[Run:MainLoop] FINISHED: "Save_Cheetah_Data()"');
                        else
                            Console_Write('[Run:MainLoop] SKIPPED: "Save_Cheetah_Data()"');
                        end
                        
                        % Set flags
                        D.F.ses_save_done = true;
                        
                        % Tell CS Matlab session saved
                        Send_CS_Com('F');
                        
                        % Stop status timer
                        stop(D.timer_save);
                        
                        % Highlight Quit button
                        Button_State(D.UI.toggQuit, 'Enable', D.UI.attentionCol);
                        
                        % Reset flag
                        D.F.do_save = false;
                    end
                    
                case 'WAIT FOR QUIT'
                    %% -------------WAIT FOR QUIT---------------
                    
                    % Set Quit button to active color
                    if ~D.F.do_quit && ~all(D.UI.toggQuit.BackgroundColor == D.UI.attentionCol)
                        Button_State(D.UI.toggQuit, 'Enable', D.UI.attentionCol);
                    end
                    
                case 'WAIT FOR EXIT'
                    %% -------------WAIT FOR EXIT---------------
                    Update_UI(0, 'force');
                    pause(0.01);
                    continue
                    
                otherwise
                    continue
            end
            
        end
        
        % End of Run
        Console_Write('[Run] END: Run()');
    end

% -----------------------------MAIN EXIT-----------------------------------
    function[] = Exit()
        
        % Start of exit
        Console_Write('[Exit] BEGIN: Exit()');
        
        % Check if GUI was forced close
        if ~FORCECLOSE
            Console_Write('[ICR_GUI] RUNNING: Normal Exit Procedure...');
            
            % Pause then shut it all down
            pause(1);
            % Disconnect from NetCom
            Disconnect_NLX();
            
        end
        
        % Save log
        Console_Write('[ICR_GUI] RUNNING: Save ICR_GUI Log...');
        
        % Make sure dir exists
        if size(who('global'),1) > 0 && ...
                exist(D.DIR.logTempDir, 'dir')
            
            % Track logs
            cnt_stored = D.DB.logCount;
            cnt_saved = 0;
            
            % Open and write to file
            fi_path = fullfile(D.DIR.logTempDir, D.DIR.logFi);
            fid = fopen(fi_path,'wt');
            for z_l = 1:cnt_stored
                
                % Attempt to store log
                try
                    % Print log being stored
                    %fprintf('Writing Log %d/%d: \"%s\"', z_l, cnt_stored, D.DB.logStr{z_l})
                    
                    % Store log
                    fprintf(fid, D.DB.logStr{z_l});
                    cnt_saved = cnt_saved+1;
                catch ME
                    
                    % Create error string
                    err_msg = sprintf('!!ERROR!! [EXIT] Error Writing Log %d/%d to \"%s\"', ...
                        z_l, cnt_stored, D.DIR.logTempDir);
                    
                    % Print and log store failure
                    fprintf(fid, err_msg);
                    fprintf(err_msg);
                    
                end
                
            end
            
            % Close file and print status
            fclose(fid);
            Console_Write(sprintf('[ICR_GUI] FINISHED: Save ICR_GUI Log: %d/%d to \"%s\"', ...
                cnt_saved, cnt_stored, D.DIR.logTempDir));
        else
            % Print failure
            Console_Write(sprintf('**WARNING** [ICR_GUI] FAILED: Saving %d Logs to \"%s\"', ...
                D.DB.logCount, D.DIR.logTempDir));
        end
        
        % Close figure
        if ~FORCECLOSE
            D.F.close = true;
            close(FIGH)
            delete(FIGH)
        end
        
        % Confirm GUI closed
        Send_CS_Com('C');
        
        % Wait for recieved confirmation
        Console_Write('[ICR_GUI] RUNNING: Wait for GUI Closed Confirm...');
        while exist('c2m', 'var')
            [abort, pass] = ...
                Check_Flag(exist('c2m', 'var'), ...
                c2m.('C').dat1 == 1);
            if abort || pass; break; end
        end
        if abort
            Console_Write('**WARNING** [ICR_GUI] ABORTED: Wait for GUI Closed Confirm');
        else
            Console_Write('[ICR_GUI] FINISHED: Wait for GUI Closed Confirm');
        end
        
        % End of Exit
        Console_Write('[Exit] END: Exit()');
    end






%% ============================= SETUP FUNCTIONS ==========================

% ------------------------------VAR SETUP--------------------------
    function[] = Var_Setup()
        
        %% RUN PARAMETERS
        
        % LOAD DATA TABLES
        T = load(D.DIR.SS_IO_1);
        D.SS_IO_1 = T.SS_IO_1;
        T = load(D.DIR.SS_IO_2);
        D.SS_IO_2 = T.SS_IO_2;
        T = load(D.DIR.SS_IO_3);
        D.SS_IO_3 = T.SS_IO_3;
        T = load(D.DIR.TT_IO_1);
        D.TT_IO_1 = T.TT_IO_1;
        T = load(D.DIR.TT_IO_2);
        D.TT_IO_2 = T.TT_IO_2;
        clear T;
        
        % LOAD TRACK BOUNDS
        
        % bounding box for video tracker (in pixels)
        % track plot bounds (width, hight)
        S = load(D.DIR.trkBnds);
        % track bound radius
        D.PAR.R = S.R;
        % track bound center X
        D.PAR.XC = S.XC;
        % track bound center Y
        D.PAR.YC = S.YC;
        clear S;
        
        % CATEGORICAL VARS
        
        % age group
        D.PAR.listAgeGrp = categories(D.SS_IO_1.Age_Group); % [Young,Old];
        % human
        D.PAR.listHuman = categories(D.SS_IO_1.Human);
        % session type
        D.PAR.listSesType = categories(D.SS_IO_1.Session_Type);
        % session condition
        D.PAR.listSesCond = categories(D.SS_IO_1.Session_Condition);
        % task condition
        D.PAR.listSesTask = {'Track'; 'Forage'};
        % feeder condition
        D.PAR.listFeedCnd = categories(D.SS_IO_1.Feeder_Condition); % [C1,C2];
        % delay cond
        D.PAR.listDel = {'0.0 '; '1.0 '; '2.0'; '3.0'};
        % cue condition
        D.PAR.listCueCond = categories(D.SS_IO_1.Cue_Condition); % [All, Half, None]
        % start qauadrant
        D.PAR.listStrQuad = categories(D.SS_IO_1.Start_Quadrant{1}); % [NE,SE,SW,NW];
        % rotation direction
        D.PAR.listRotDrc = categories(D.SS_IO_1.Rotation_Direction{1}); % [CCW,CW];
        % rotation position
        D.PAR.listRotPos = categories(D.SS_IO_1.Rotation_Positions{1}); % [90,180,270];
        % bulldoze delay
        D.PAR.listBull = {'0 sec'; '5 sec'; '10 sec'; '30 sec'; '60 sec'; '120 sec'};
        % cap weights
        D.PAR.capWeightList = {'None'; 'Housing'; '1'; '2'; '3'; '4'; '5'; '6'; '7'; '8'};
        
        % RAT LIST
        
        % Pull out row names
        D.PAR.listRat = ...
            [D.SS_IO_1.Properties.RowNames(D.SS_IO_1.Include_Run), ...
            cellstr(char(D.SS_IO_1.Yoke_Mate(D.SS_IO_1.Include_Run)))];
        
        % Remove 'r'
        D.PAR.listRat = regexprep(D.PAR.listRat, 'r', '');
        
        % Add space
        D.PAR.listRat = cellfun(@(x,y) sprintf('%s (%s)', x, y), ...
            D.PAR.listRat(:,1), D.PAR.listRat(:,2), 'Uni', false);
        
        % POSITION VARS
        
        % Pixel width/height
        D.UI.vtRes = round(D.PAR.R*2);
        
        % Track plot bounds [left, bottom]
        D.UI.lowLeft = round([D.PAR.XC-D.PAR.R, D.PAR.YC-D.PAR.R]);
        
        % Calculate pixel to cm conversion factors
        D.UI.cm2pxl = D.UI.vtRes/140;
        
        % Pos lims
        % arena radius
        D.UI.arnRad = 70; % (cm)
        % track width
        D.UI.trkWdt = 10; % (cm)
        % random forrage radius
        D.UI.frgRad = D.UI.arnRad - D.UI.trkWdt; % (cm)
        % rew target depth
        D.UI.frgTargDpth = 30; % (cm)
        % reward target width
        D.PAR.frgTargWdt = 50; % (deg)
        % track roh limits
        D.P.trackRohBnd(1) = 1 - (D.UI.trkWdt/D.UI.arnRad);
        D.P.trackRohBnd(2) = 1;
        % track roh cutoff +- 5 cm
        D.P.trackRohCut(1) = D.P.trackRohBnd(1) - 10/D.UI.arnRad;
        D.P.trackRohCut(2) = D.P.trackRohBnd(2) + 10/D.UI.arnRad;
        % forage roh limits
        D.P.frgRohBnd(1) = (D.UI.frgRad/D.UI.arnRad) - (D.UI.frgTargDpth/D.UI.arnRad);
        D.P.frgRohBnd(2) = D.UI.frgRad/D.UI.arnRad;
        % vel exlcude
        D.P.velExc = 300;
        
        % FORAGE TASK VARS
        
        % reward block dt
        D.PAR.frgRewBlock = 15; % (sec)
        % pos occ bins
        D.PAR.frgBins = (D.UI.arnRad*2)/2 + 1;
        % occ bin edges
        D.PAR.frgBinEdgeX = linspace(D.UI.lowLeft(1), D.UI.lowLeft(1)+D.UI.vtRes, D.PAR.frgBins+1);
        D.PAR.frgBinEdgeY = linspace(D.UI.lowLeft(2), D.UI.lowLeft(2)+D.UI.vtRes, D.PAR.frgBins+1);
        % distance between paths
        D.PAR.frgPathSpace = 5; % (deg)
        % path width (deg)
        D.PAR.frgPathWdt = 30; % (deg)
        % target angle array
        D.PAR.frgTargDegArr = 0:D.PAR.frgPathSpace:360-D.PAR.frgPathSpace;
        % path angle array
        D.PAR.frgPathDegArr = linspace(-45,45,45/D.PAR.frgPathSpace*2 + 1);
        % target select subset
        D.PAR.frgTargSelectInd = [];
        % path select subset
        D.PAR.frgPathSelectInd = [];
        % number of posible paths from each targ
        D.PAR.nPaths = 45/D.PAR.frgPathSpace*2 + 1;
        % path lenths
        D.PAR.pathLengthArr = zeros(1,D.PAR.nPaths);
        
        % TRACK TASK VARS
        
        % min/max reward duration
        D.PAR.rewDurLim = [500, 2000];
        % reward zone positions
        D.PAR.zoneLocs = 20:-5:-20;
        % cued zone
        D.PAR.zoneCue = 0;
        % reward zone reward durations
        D.PAR.zoneRewDur = ...
            [500, 910, 1420, 1840, 2000, 1840, 1420, 910, 500];
        % current reward duration
        D.PAR.rewDur = max(D.PAR.rewDurLim);
        
        % TT PLOT VARS
        % tt 2D occ bins
        D.PAR.tt2dBins = D.PAR.frgBins;
        % tt 2D occ bin edges
        D.PAR.tt2dBinEdgeX = D.PAR.frgBinEdgeX;
        D.PAR.tt2dBinEdgeY = D.PAR.frgBinEdgeY;
        % tt 1D occ bins
        D.PAR.tt1dBins = 360/2.5;
        % tt 1D occ bin edges
        D.PAR.tt1dBinEdge = linspace(0, 2*pi, D.PAR.tt1dBins+1);
        
        % TT TRACK VARS
        
        %% FLOW CONTROL VARIABLES
        
        % COUNTERS
        
        % track icr events
        D.C.rot_cnt = 0;
        % lap by rotation
        D.C.lap_cnt = num2cell(zeros(1,3));
        % rew by rotation
        D.C.rew_cnt = num2cell(zeros(1,3));
        % reward send
        D.C.rew_send_cnt = 0;
        % reward crossings
        D.C.rew_cross_cnt = 0;
        % missed rewards [consecutive, total]
        D.C.missed_rew_cnt = [0, 0];
        % bulldozing event count
        D.C.bull_cnt = 0;
        % counter for each reward zone
        D.C.zone = zeros(2,length(D.PAR.zoneLocs));
        % track number of robot move cammands
        D.C.move = 0;
        
        % FLAGS
        
        % ac computer connected
        D.F.ac_connected = false;
        % ui updated
        D.F.ui_updated = false;
        % session type selected
        D.F.ses_type_confirmed = false;
        % implant session
        D.F.implant_session = false;
        % rat_implanted
        D.F.rat_implanted = false;
        % table tab setup
        D.F.table_tab_setup = false;
        % tt tab setup
        D.F.tt_tab_setup = false;
        % switch case main loop
        D.F.main_case_now = 'NULL';
        D.F.main_case_last = 'NULL';
        % switch case poll
        D.F.sub_case_now = 'NULL';
        D.F.sub_case_last = 'NULL';
        % status of Cheetah.exe
        D.F.cheetah_running = false;
        % status of SpikeSort3D.exe
        D.F.spikesort_running = false;
        % cube connection status
        D.F.cube_connected = false;
        % nlx streaming
        D.F.matlab_streaming = false;
        D.F.vt_rat_streaming = false;
        D.F.vt_rob_streaming  = false;
        D.F.evt_streaming = false;
        % robot setup
        D.F.rob_setup = false;
        % sleep done
        D.F.sleep_done = [false, false];
        % ses data loaded
        D.F.ses_data_loaded = false;
        % ui setup finished
        D.F.ses_setup_done = false;
        % setup finished
        D.F.task_setup = false;
        % robot streaming
        D.F.rob_streaming = false;
        % polling nlx
        D.F.poll_nlx = false;
        % new nlx vt data
        D.F.poll_new_vt = false;
        % first move sent to robot
        D.F.first_move_sent = false;
        % recording done
        D.F.task_done = false;
        % task done confirm enabled
        D.F.task_done_confirmed = false;
        % track if rat is in arena
        D.F.rat_in = false;
        % track if rat is out of arena
        D.F.rat_out = false;
        % flag to do save
        D.F.do_save = false;
        % flag to do save cheetah data
        D.F.do_nlx_save = false;
        % session save done
        D.F.ses_save_done = false;
        % flag quit
        D.F.do_quit = false;
        % flag gui closed
        D.F.close = false;
        % sound settings
        D.F.sound = false(1,2);
        % acquiring nlx
        D.F.acq = false;
        % recording nlx
        D.F.rec = false;
        % do rotation
        D.F.rotate = false;
        % rotation has occured
        D.F.rotated = false;
        % if reward in progress
        D.F.rewarding = false;
        % flag if halted
        D.F.halted = false;
        % reward reset
        D.F.rew_confirmed = false;
        % reward reset
        D.F.rew_reset = false;
        % reward sent
        D.F.rew_sent = false;
        % all reward zones crossed
        D.F.rew_zone_crossed = false;
        % flag to move to new forage targ
        D.F.move_to_targ = false;
        % track lap bounds
        D.F.check_inbound_lap = false(4,1);
        % do stream hd
        D.F.stream_hd = false;
        % new x,y data processed
        D.F.Rat.new_pos_data = false;
        D.F.Rob.new_pos_data = false;
        % new vel data processed
        D.F.Rat.new_vel_data = false;
        D.F.Rob.new_vel_data = false;
        % new hd data processed
        D.F.Rat.new_hd_data = false;
        % track clusters that are loaded
        D.F.clust_loaded = false(D.PAR.maxTT,D.PAR.maxClust);
        % track chans that are disabled
        D.F.tt_chan_disable = false(D.PAR.maxTT,4);
        % track tts that are actively streaming
        D.F.tt_streaming = false(D.PAR.maxTT,1);
        % plot clusters
        D.F.plot_clust = false(D.PAR.maxTT,D.PAR.maxClust);
        % flag for update status of each tt
        D.F.tt_updated = false(D.PAR.maxTT,1);
        
        % TIMERS
        
        % loop time
        D.T.loop = 0;
        % last vt poll time
        D.T.poll_last_evt = 0;
        % last vt poll time
        D.T.poll_last_vt = 0;
        % last tt poll time
        D.T.poll_last_tt = 0;
        % last ui redraw
        D.T.ui_update = 0;
        % time session starts
        D.T.ses_str = Sec_DT(now);
        % first nxl ts
        D.T.poll_str_nlx = 0;
        % time session ends
        D.T.ses_end = D.T.ses_str;
        % sleep 1/2 start
        D.T.sleep_str = [0,0];
        % sleep 1/2 end
        D.T.sleep_end = [0,0];
        % total acq time
        D.T.acq_tot_tim = 0;
        % acq restart time
        D.T.acq_tim = 0;
        % total rec time
        D.T.rec_tot_tim = 0;
        % rec restart time
        D.T.rec_tim = 0;
        % run start time local
        D.T.task_str = 0;
        % run start time cheetah
        D.T.task_str_nlx = 0;
        % track lap times
        D.T.lap_str = 0;
        % track lap times total
        D.T.lap_tot = 0;
        % last text info update
        D.T.info_txt_update = Sec_DT(now);
        % last reset of db info max reset
        D.T.info_db_reset = Sec_DT(now);
        % track manual reward sent time
        D.T.btn_rew_sent = 0;
        % track last reward time
        D.T.rew_last = Sec_DT(now);
        % track reward start
        D.T.rew_start = 0;
        % track reward end
        D.T.rew_end = 0;
        % track reward ts
        D.T.rew_nlx_ts = [0,0];
        % forage reward time
        D.T.frg_rew = 0;
        % start quad tim
        D.T.strqd_inbnd_t1 = 0;
        D.T.strqd_inbnd_t2 = 0;
        % track last pos update
        D.T.Rat.last_pos_update = Sec_DT(now);
        D.T.Rob.last_pos_update = Sec_DT(now);
        % forage reward in target tim
        D.T.frg_rew_inbnd_t1 = 0;
        D.T.frg_rew_inbnd_t2 = 0;
        % forage reward out target tim
        D.T.frg_rew_outbnd_t1 = 0;
        D.T.frg_rew_outbnd_t2 = 0;
        % cube status check
        D.T.cube_status_check = 0;
        % Last time cube vcc checked
        D.T.cube_vcc_check = 0;
        
        % INDEXING
        
        % current wall image index
        D.I.rot = 1;
        % feeder index
        D.I.img_ind = [1, NaN];
        % current lap quadrant for lap track
        D.I.lap_hunt_ind = 1;
        % current reward zone
        D.I.zone_now = ceil(length(D.PAR.zoneLocs)/2);
        % next reward zone
        D.I.zone_select = 0;
        % track last rewarded zone
        D.I.zone_active = false(1,length(D.PAR.zoneLocs));
        % current targ ind
        D.I.targ_now = 1;
        % last targ ind
        D.I.targ_last = 1;
        
        %% DATA STORAGE VARIABLES
        
        % SESSION PARAMETERS
        D.PAR.sesType = categorical({'<undefined>'}, D.PAR.listSesType);
        D.PAR.sesHuman = categorical({'<undefined>'}, D.PAR.listHuman);
        D.PAR.sesCond = categorical({'<undefined>'}, D.PAR.listSesCond);
        D.PAR.sesTask = categorical({'<undefined>'}, D.PAR.listSesTask);
        D.PAR.sesCue = categorical({'<undefined>'}, D.PAR.listCueCond);
        D.PAR.sesRewDel = categorical({'<undefined>'}, D.PAR.listDel);
        
        % RAT PARAMETERS
        D.PAR.ratFeedCnd = '<undefined>';
        D.PAR.ratRotDrc = '<undefined>';
        D.PAR.ratStrQuad = '<undefined>';
        
        % VCC VARS
        D.PAR.rob_vcc = 0;
        D.PAR.rob_vcc_last = 0;
        D.PAR.cube_vcc = 0;
        D.PAR.cube_vcc_last = 0;
        
        % EVENT DATA
        D.E.evtTS = NaN;
        D.E.evtStr = '';
        D.E.evtNRecs = 0;
        D.E.cnt_evtRec = 0;
        
        % POS DATA
        % nlx outputs
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
        % time stamps
        D.P.Rat.ts = NaN;
        D.P.Rob.ts = NaN;
        % cardinal pos now
        D.P.Rat.x = NaN;
        D.P.Rat.y = NaN;
        D.P.Rob.x = NaN;
        D.P.Rob.y = NaN;
        % last valide x y
        D.P.Rat.xLast = NaN;
        D.P.Rob.xLast = NaN;
        D.P.Rat.yLast = NaN;
        D.P.Rob.yLast = NaN;
        % pos rad
        D.P.Rat.rad = NaN;
        D.P.Rob.rad = NaN;
        % last valid rad and ts
        D.P.Rat.radLast = NaN;
        D.P.Rob.radLast = NaN;
        D.P.Rat.tsLast = NaN;
        D.P.Rob.tsLast = NaN;
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
        % store hostory of pos [x, y] [rad, roh] [ts]
        D.P.Rat.posAll = cell2struct( ...
            {[1,0], [1,0], single(NaN(120*60*33,2)), single(NaN(120*60*33,2)), NaN(120*60*33,1)}, ...
            {'indLap', 'indAll', 'Cart', 'Pol', 'TS'}, 2);
        D.P.Rob.posAll = D.P.Rat.posAll;
        % store history of vel [x, y] [rad, roh] [ts] [rad, roh]
        D.P.Rat.velAll = cell2struct( ...
            {[1,0], [1,0], [1,0], single(NaN(120*60*33, 2)), single(NaN(120*60*33,2)), NaN(120*60*33,1), NaN(500,101,2)}, ...
            {'indLap', 'indAll', 'indHist', 'Cart', 'Pol', 'TS', 'Hist'}, 2);
        D.P.Rob.velAll = D.P.Rat.velAll;
        % velocity cutoff
        D.P.velRohMax = D.P.trackRohBnd(1);
        D.P.velRohMin = D.P.trackRohBnd(1) - 0.25;
        D.P.velMin = 0;
        D.P.velMax = 100;
        % random forage
        D.P.frgOccMatRaw =  zeros(D.PAR.frgBins,D.PAR.frgBins);
        D.P.frgOccMatScale = D.P.frgOccMatRaw;
        D.P.frgOccMatBinary = D.P.frgOccMatRaw;
        D.P.pathMat = zeros(D.PAR.frgBins,D.PAR.frgBins,D.PAR.nPaths,length(D.PAR.frgTargDegArr));
        D.P.pathNowMat = D.P.frgOccMatScale;
        
        % TT DATA
        % nlx outputs
        D.TT.ttTS = cell(D.PAR.maxTT,1);
        D.TT.ttClust = cell(D.PAR.maxTT,1);
        D.TT.ttRecs = cell(D.PAR.maxTT,1);
        D.TT.nClust = zeros(D.PAR.maxTT,1);
        D.TT.posClust = cell(D.PAR.maxTT,D.PAR.maxClust);
        
        % DEBUG VARS
        % track reward duration [now, min, max, sum, count]
        D.DB.rew_duration = [0, inf, 0, 0, 0];
        % track reward round trip [now, min, max, sum, count]
        D.DB.rew_round_trip = [0, inf, 0, 0, 0];
        % track loop dt [now, min, max, sum, count]
        D.DB.loop = [0, inf, 0, 0, 0];
        % plot time [now, min, max, sum, count]
        D.DB.draw = [0, inf, 0, 0, 0];
        % draw time [now, min, max, sum, count]
        D.DB.plot = [0, inf, 0, 0, 0];
        % Info print time [now, min, max, sum, count]
        D.DB.inf = [0, inf, 0, 0, 0];
        % poll event dt time [now, min, max, sum, count]
        D.DB.pollevt = [0, inf, 0, 0, 0];
        % poll vt dt time [now, min, max, sum, count]
        D.DB.pollvt = [0, inf, 0, 0, 0];
        % poll tt dt time [now, min, max, sum, count]
        D.DB.polltt = [0, inf, 0, 0, 0];
        % halt error [now, min, max, sum, count]
        D.DB.HALT.error = [0, inf, 0, 0, 0];
        
        % OTHER
        % session message
        D.PAR.sesMsg = NaN(2,2);
        % bulldoze defaults
        D.PAR.bullDel = 30;
        D.PAR.bullSpeed = 10;
        D.PAR.bullLastVal = 0;
        % rotaion pos
        D.PAR.rotPos = [];
        % store reward zone for each reward event
        D.PAR.zone_hist = NaN(1,100);
        % cued rewards
        D.PAR.cued_rew = [];
        
        %% UI HANDLES
        
        % AXIS HANDLES
        D.UI.axH = gobjects(7,1);
        D.UI.axZoneH = gobjects(2,1);
        D.UI.axClstH = gobjects(D.PAR.maxClust,1);
        
        % TRACK TASK HANDLES
        
        % track bound line handles
        D.UI.linTrkH = gobjects(2,1);
        % vel plot line handles
        D.UI.nVelRings = ceil(D.P.velMax/10)+1;
        D.UI.linVelH = gobjects(D.UI.nVelRings,1);
        % start quad patch
        D.UI.ptchStQ = [];
        % start quad text
        D.UI.txtStQ = [];
        % reward zone patch
        D.UI.ptchRewZoneBndsH = gobjects(2, length(D.PAR.zoneLocs));
        % reward duration text
        D.UI.txtFdDurH = gobjects(2, length(D.PAR.zoneLocs));
        % reward zone plot
        D.UI.ptchRewZoneHistH = gobjects(2, length(D.PAR.zoneLocs));
        D.UI.linZoneAvgH = gobjects(1,1);
        % reward reset patch
        D.UI.ptchRewSendH = gobjects(1,2);
        % current feeder marker/patch/line
        D.UI.mixFdNow = gobjects(3,2);
        
        % FORAGE TASK HANDLES
        
        % forage bound line handles
        D.UI.linFrgH = gobjects(2,1);
        % forage targ patch
        D.UI.ptchRewTargBnds = gobjects(360/D.PAR.frgPathSpace,1);
        % forage occ mat
        D.UI.imgFrgOcc = gobjects(1,1);
        
        % POS DATA
        
        % vt rat pos
        D.UI.Rat.mrkPosNowH = gobjects(1,1);
        D.UI.Rat.mrkPosLapH = gobjects(1,1);
        % vt head direction (backround, foreground)
        D.UI.Rat.mrkHeadNowH = gobjects(1,2);
        % robot body features
        D.UI.Rob.ptchPosNowH = gobjects(1,1);
        D.UI.Rob.mrkPosNowH = gobjects(1,1);
        D.UI.Rob.linPosNowArmH = gobjects(1,1);
        D.UI.Rob.linPosNowGuardH = gobjects(1,1);
        D.UI.Rob.linPosNowSetH = gobjects(1,1);
        D.UI.Rob.mrkPosNowDishH = gobjects(1,1);
        % vt plot handles of velocity
        D.UI.Rat.linVelLapH = gobjects(1,1);
        D.UI.Rob.linVelLapH = gobjects(1,1);
        % handles for average vel plots
        D.UI.Rat.linVelAvgH = gobjects(1,1); % rat
        D.UI.Rob.linVelAvgH = gobjects(1,1); % rob
        % handles for history data
        D.UI.Rat.linPosAll = gobjects(1,1); % rat
        D.UI.Rat.linVelAll = gobjects(1,1); % rat
        D.UI.Rob.linVelAll = gobjects(1,1); % rob
        
        % TT PLOT
        
        % cell plot handles
        D.UI.mrkClustH = gobjects(D.PAR.maxTT,D.PAR.maxClust);
        D.UI.imgClustH = gobjects(D.PAR.maxTT,D.PAR.maxClust);
        D.UI.ptchClustH = gobjects(D.PAR.maxClust, D.PAR.tt1dBins);
        D.UI.linColBarH = gobjects(D.PAR.maxClust,25);
        D.UI.txtColBarH = gobjects(D.PAR.maxClust,1);
        
        % TT TRACK
        
        % TT position plot
        D.UI.ttTrkLin = gobjects(D.PAR.maxTT,1);
        D.UI.ttTrkMrk = gobjects(D.PAR.maxTT,50);
        % TT position legend plots
        D.UI.ttBndlLegMrk = gobjects(D.PAR.maxTT,1);
        D.UI.ttBndlLegTxt = gobjects(D.PAR.maxTT,1);
        D.UI.ttDriveLegMrk = gobjects(D.PAR.maxTT,1);
        D.UI.ttDriveLegTxt = gobjects(D.PAR.maxTT,1);
        % Clust current axis 'reference'
        D.UI.ttClustAxRef = gobjects(D.PAR.maxTT,D.PAR.maxClust);
        % Create handle arrays
        D.UI.toggSelectTT = gobjects(D.PAR.maxTT,1);
        % Clust sub toggles
        D.UI.toggSubPlotTT = gobjects(D.PAR.maxTT,D.PAR.maxClust);
        % Flag features sub toggles
        D.UI.toggSubFlagTT = gobjects(D.PAR.maxTT,10);
        % TT hear sub toggles
        D.UI.toggSubHearChTT = gobjects(D.PAR.maxTT,4,2);
        D.UI.toggSubHearSdTT = gobjects(D.PAR.maxTT,2);
        
        %% UI APPERANCE
        
        % UI POS AND SCALE
        
        % Specify default figure monitor
        D.UI.monDefault = 2;
        
        % Monitor positions
        sc = get(0,'MonitorPositions');
        [~, mon_ind] = sort(sc(:,1));
        D.UI.monPos = sc(mon_ind,:);
        
        % Possible mons
        D.UI.figMon = [1,1,1];
        if size(D.UI.monPos,1) > 1
            D.UI.figMon(2) = 2;
        end
        if size(D.UI.monPos,1) > 2
            D.UI.figMon(3) = 3;
        end
        
        % UI window positions
        fig_wh = [1240 1008];
        fig_btm = 42;
        for z_m = 1:3
            if z_m > size(D.UI.monPos,1)
                continue
            end
            fig_lft = D.UI.monPos(D.UI.figMon(z_m),1) + (D.UI.monPos(D.UI.figMon(z_m),3)-fig_wh(1))/2;
            D.UI.figGrpPos{z_m} = [fig_lft, fig_btm, fig_wh(1), fig_wh(2)];
        end
        
        % Store current pos as 4th entry
        D.UI.figGrpPos{4} = D.UI.figGrpPos{D.UI.monDefault};
        
        % Specify non reserved range
        D.UI.fig_rng = fig_wh - [0, 200];
        
        % Pixel to normalzied unit conversion factor
        D.UI.pxl2norm_x = 1/fig_wh(1);
        D.UI.pxl2norm_y = 1/fig_wh(2);
        
        % UI COLOR
        
        % UI objects
        D.UI.figBckCol = [1, 1, 1];
        D.UI.enabledCol = [0.35, 0.35, 0.35];
        D.UI.disabledCol = [0.75 0.75 0.75];
        D.UI.enabledBtnFrgCol = [0.95, 0.95, 0.95];
        D.UI.disabledBtnFrgCol = [0.5, 0.5, 0.5];
        D.UI.activeCol = [0, 0.475, 0.85];
        D.UI.attentionCol = [0.75, 0, 0];
        D.UI.warningCol = [1, 0, 0];
        
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
        D.UI.guardPosCol = D.UI.disabledCol;
        % width of line for rob guard pos
        D.UI.guardPosLineWidth = 5;
        % color of line for feeder pos
        D.UI.feedPosCol = D.UI.rotCol(1,:);
        % marker size for feeder pos
        D.UI.feedPosMarkSize = 10;
        % color of line for setpoint
        D.UI.setPosCol = D.UI.disabledCol;
        % width of line for setpoint
        D.UI.setPosLineWidth = 2;
        % tt color
        D.UI.ttCol = NaN(D.PAR.maxTT, 3);
        % clust colors
        D.UI.clustCol = NaN(D.PAR.maxTT, D.PAR.maxClust, 3);
        % clust color map
        D.UI.clustColMap = NaN(D.PAR.maxTT, D.PAR.maxClust, size(D.UI.linColBarH,2), 3);
        % chan hear colors
        D.UI.hearCol = hsv(5);
        D.UI.hearCol = D.UI.hearCol(2:end,:);
        
        % UI FONTS
        
        % Fonts
        D.UI.btnFont = 'MS Sans Serif';
        D.UI.headFont = 'MS Sans Serif';
        D.UI.popFont = 'Monospaced';
        D.UI.txtFont = 'Courier New';
        D.UI.titleFont = 'Monospaced';
        
        % Font sizes
        D.UI.fontSzTxtHuge = ...
            [14, 18*D.UI.pxl2norm_y*1.25];
        D.UI.fontSzTxtLrg = ...
            [12, 16*D.UI.pxl2norm_y*1.25];
        D.UI.fontSzTxtMed = ...
            [10, 13*D.UI.pxl2norm_y*1.25];
        D.UI.fontSzTxtSml = ...
            [8, 13*D.UI.pxl2norm_y*1.25];
        D.UI.fontSzPopLrg = ...
            [10, 0.025];
        D.UI.fontSzPopMed = ...
            [8, 0.025];
        D.UI.fontSzPopSml = ...
            [7, 0.0195];
        D.UI.fontSzBtnHuge = ...
            [14, 18*D.UI.pxl2norm_y*1.25];
        D.UI.fontSzBtnLrg = ...
            [10, 13*D.UI.pxl2norm_y*1.5];
        D.UI.fontSzBtnMed = ...
            [9, 12*D.UI.pxl2norm_y*1.5];
        D.UI.fontSzBtnSml = ...
            [8, 11*D.UI.pxl2norm_y*1.5];
        
        % UI OBJECT FEATURES
        
        % Rotation Button String
        D.UI.btnICRstr = cell(2,1);
        
        % Lap Time List
        D.UI.lapTimList = {[]};
        
        % Reward Info List
        D.UI.rewInfoList = {[]};
        
        % TT TRACK OBJECT FEATURES
        
        % Cannula spacing (mm)
        D.PAR.canSp = 0.3175;
        
        % TT diameter (mm)
        D.PAR.ttDiam = 0.0254;
        
        % Track lines offset (mm)
        D.UI.ttPlotLinOffset = 0.025;
        
        % Active tt apearance
        D.UI.ttBndlLegMrkWdth = [1,4];
        D.UI.ttFaceAlph = [0.05, 1];
        
    end

% -------------------------------UI SETUP--------------------------
    function[] = UI_Setup()
        
        %% ===================== SETUP FIGURE AND AXES ==========================
        
        % Create figure tab group
        D.UI.tabgp = ...
            uitabgroup(FIGH, ...
            'Units', 'Normalized', ...
            'SelectionChangedFcn', {@Tab_GrpChange}, ...
            'UserData', 'TT TRACK', ...
            'Position',[0,0,1,1]);
        
        % Set figure stuff
        set(FIGH,...
            'Name', 'ICR Behavior Pilot', ...
            'Position', D.UI.figGrpPos{4}, ...
            'MenuBar', 'none', ...
            'Color', [1, 1, 1]);
        
        % Add ICR ARENA tab
        D.UI.tabICR = uitab(D.UI.tabgp, ...
            'Title', 'ICR ARENA', ...
            'BackgroundColor', [1, 1, 1]);
        
        % Store pos
        Safe_Set(D.UI.tabICR, 'Units', 'Pixels');
        D.UI.tabICRPos = get(D.UI.tabICR, 'Position');
        
        % Wall Image
        % Note: used for wall images
        D.UI.axH(1)= axes(...
            'Units', 'Normalized', ...
            'Visible', 'off', ...
            'Parent', D.UI.tabICR);
        hold on;
        
        % Pos/Vel History Data
        % Note: used for pos/vel history
        D.UI.axH(2) = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        
        % Real Time Track Task Data
        % Note: used for path and other dynamic features
        D.UI.axH(3) = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        
        % Arena Features
        % Note: used for feeder pos
        D.UI.axH(4) = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        
        % Random Forage Task OCC Data
        % Note: used for occ imagesc
        D.UI.axH(5) = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        
        % Real Time TT Position/OCC Data
        D.UI.axH(6) = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        
        % Selectable Items
        D.UI.axH(7) = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        
        % Copy object for tt clust legend
        D.UI.axColLeg = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        
        % Copy For Zone Plot Axes
        D.UI.axZoneH(1) = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        D.UI.axZoneH(2) = copyobj(D.UI.axH(1), D.UI.tabICR);
        hold on;
        
        % Set axis limits
        axis(D.UI.axH, [ ...
            D.UI.lowLeft(1), ...
            D.UI.lowLeft(1)+D.UI.vtRes, ...
            D.UI.lowLeft(2), ...
            D.UI.lowLeft(2)+D.UI.vtRes]);
        
        % Set axis pos
        ax_btm = 1 - D.UI.fig_rng(2)/D.UI.figGrpPos{4}(4);
        D.UI.axPos = [ ...
            (1 - (0.9/(D.UI.fig_rng(1)/D.UI.fig_rng(2)))) / 2, ...
            0.05 + ax_btm, ...
            0.9/(D.UI.fig_rng(1)/D.UI.fig_rng(2)), ...
            0.9 - ax_btm];
        Safe_Set(D.UI.axH, 'Position', D.UI.axPos);
        
        % Make sure axes square
        axis(D.UI.axH, 'square');
        
        % Copy For Real Time TT OCC Data
        for z_c = 1:D.PAR.maxClust
            D.UI.axClstH(z_c) = ...
                copyobj(D.UI.axH(1), D.UI.tabICR);
            hold on;
        end
        
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
        t_h(1) = text(xy_max(1)+offset, xy_mid(2), 'E', 'Parent', D.UI.axH(4));
        t_h(2) = text(xy_mid(1), xy_min(2)+2-offset, 'S', 'Parent', D.UI.axH(4));
        t_h(3) = text(xy_min(1)-offset, xy_mid(2), 'W', 'Parent', D.UI.axH(4));
        t_h(4) = text(xy_mid(1), xy_max(2)+offset, 'N', 'Parent', D.UI.axH(4));
        set(t_h, ...
            'FontName','Monospaced', ...
            'FontSize', 20, ...
            'FontWeight', 'bold', ...
            'FontSmoothing', 'on', ...
            'Color', [1,1,1], ...
            'HorizontalAlignment','center', ...
            'VerticalAlignment','middle')
        t2_h = copyobj(t_h, D.UI.axH(4));
        set(t2_h, ...
            'Color', D.UI.attentionCol, ...
            'FontWeight', 'light', ...
            'FontSmoothing', 'on', ...
            'FontSize', 20);
        
        % Get coordinates for circle
        circ = [0:.01:2*pi,0];
        
        % Clear vars
        clear xy_mid xy_min xy_max offset t2_h;
        
        % CREATE TRACK OUTLINE
        
        % Get outer track
        x_mat = NaN(length(circ)+1,2);
        y_mat = NaN(length(circ)+1,2);
        [out_X,out_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.arnRad));
        x_mat(1:end-1,1) = out_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
        y_mat(1:end-1,1) = out_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
        
        % Get inner track
        [in_X,in_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.arnRad-D.UI.trkWdt));
        x_mat(1:end-1,2) = in_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
        y_mat(1:end-1,2) = in_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
        
        % Plot
        D.UI.linTrkH = line(x_mat, y_mat, ...
            'color', [0.5 0.5 0.5], ...
            'LineWidth', 2, ...
            'Parent', D.UI.axH(4), ...
            'Visible', 'off');
        
        % Clear vars
        clear out_X out_Y xout yout in_X in_Y xin yin;
        
        % CREATE VEL PLOT OUTLINE
        roh_inc = linspace(D.P.velRohMin, D.P.velRohMax, D.UI.nVelRings);
        x_mat = NaN(length(circ)+1,D.UI.nVelRings);
        y_mat = NaN(length(circ)+1,D.UI.nVelRings);
        for z_lin = 1:D.UI.nVelRings
            [x,y] = pol2cart(circ, ones(1,length(circ))*D.UI.arnRad*roh_inc(z_lin));
            x_mat(1:end-1,z_lin) = x*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
            y_mat(1:end-1,z_lin) = y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
        end
        D.UI.linVelH = line(x_mat, y_mat, ...
            'color', [0.5 0.5 0.5], ...
            'LineWidth', 0.5, ...
            'LineStyle', '-', ...
            'Parent', D.UI.axH(4), ...
            'Visible', 'off');
        
        % Clear vars
        clear roh_inc x y lin_wdth lin_style col;
        
        % CREATE RF BND LINES
        
        % Get outer bounds
        x_mat = NaN(length(circ)+1,2);
        y_mat = NaN(length(circ)+1,2);
        [out_X,out_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.frgRad));
        x_mat(1:end-1,1) = out_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
        y_mat(1:end-1,1) = out_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
        
        % Get inner bounds
        [in_X,in_Y] = pol2cart(circ, ones(1,length(circ)) * (D.UI.frgRad-D.UI.frgTargDpth));
        x_mat(1:end-1,2) = in_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
        y_mat(1:end-1,2) = in_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
        
        % Plot
        D.UI.linFrgH = line(x_mat, y_mat, ...
            'color', [0.5 0.5 0.5], ...
            'LineWidth', 2, ...
            'Parent', D.UI.axH(4), ...
            'Visible', 'off');
        
        % GUI OBJECT POSITIONS
        
        % Dialogue box positions
        for z_m = 1:3
            D.UI.dlgPos{z_m} = ...
                [D.UI.monPos(D.UI.figMon(z_m),1) + D.UI.monPos(D.UI.figMon(z_m),3)/2, D.UI.monPos(D.UI.figMon(z_m),4)/2];
        end
        D.UI.dlgPos{4} = D.UI.dlgPos{D.UI.monDefault};
        
        % Defaults
        obj_gap = 0.005;
        
        % Bounds of plot space
        D.UI.main_ax_bounds = [ ...
            D.UI.axPos(1) - 0.05, ...
            D.UI.axPos(2) - 0.05, ...
            D.UI.axPos(3) + 0.1, ...
            D.UI.axPos(4) + 0.1];
        
        % Monitor button pos
        D.UI.mon_btn_pos = [...
            0, ...
            0, ...
            D.UI.main_ax_bounds(1) / 3, ...
            0.02];
        
        % Save button pos
        D.UI.save_btn_pos = [...
            obj_gap, ...
            D.UI.mon_btn_pos(4) + obj_gap, ...
            (D.UI.main_ax_bounds(1) - 0.015) / 2, ...
            0.03];
        
        % Quit button pos
        D.UI.quit_btn_pos = [...
            D.UI.save_btn_pos(3) + obj_gap*2, ...
            D.UI.save_btn_pos(2), ...
            D.UI.save_btn_pos(3), ...
            D.UI.save_btn_pos(4)];
        
        % Panel setup
        pan_ht = 0.505 * D.UI.main_ax_bounds(4);
        D.UI.stup_pan_pos = ...
            [0,  ...
            1 - pan_ht, ...
            D.UI.main_ax_bounds(1), ...
            pan_ht];
        
        % Panel run
        pan_ht = D.UI.stup_pan_pos(2) - sum(D.UI.quit_btn_pos([2,4])) - 0.01;
        pan_botm = sum(D.UI.quit_btn_pos([2,4])) + obj_gap*2;
        D.UI.run_pan_pos = ...
            [0, ...
            pan_botm, ...
            D.UI.main_ax_bounds(1), ...
            pan_ht];
        
        % Panel ses info
        pan_lft = (D.UI.main_ax_bounds(1)+D.UI.main_ax_bounds(3));
        pan_wd = 1 - (D.UI.main_ax_bounds(1)+D.UI.main_ax_bounds(3));
        pan_ht = 0.25 * D.UI.main_ax_bounds(4);
        D.UI.ses_inf_pan_pos = [...
            pan_lft, ...
            1 - pan_ht, ...
            pan_wd, ...
            pan_ht];
        
        % Panel performance info
        pan_ht = 0.525 * D.UI.main_ax_bounds(4);
        D.UI.perf_inf_pan_pos = [...
            pan_lft, ...
            D.UI.ses_inf_pan_pos(2) - pan_ht - D.UI.pxl2norm_y, ...
            pan_wd, ...
            pan_ht];
        
        % Panel console
        pan_ht = 1 - D.UI.fig_rng(2)/D.UI.figGrpPos{4}(4);
        D.UI.cnsl_pan_pos = ...
            [D.UI.main_ax_bounds(1)+obj_gap, ...
            0, ...
            D.UI.main_ax_bounds(3) - pan_wd*0.75 - obj_gap*2, ...
            pan_ht];
        
        % Panel time info
        pan_ht = 0.15;
        D.UI.tim_inf_pan_pos = [...
            sum(D.UI.cnsl_pan_pos([1,3])) + obj_gap, ...
            0, ...
            pan_lft - sum(D.UI.cnsl_pan_pos([1,3])) - obj_gap*2, ...
            pan_ht];
        
        % Panel tt select
        pan_ht = D.UI.perf_inf_pan_pos(2)-0.04;
        D.UI.tab_1_tt_select_pan_pos(1,:) = [...
            pan_lft, ...
            0.04, ...
            pan_wd/2, ...
            pan_ht];
        D.UI.tab_1_tt_select_pan_pos(2,:) = [...
            pan_lft+pan_wd/2, ...
            0.04, ...
            pan_wd/2, ...
            pan_ht];
        
        % MAKE WALL IMAGES
        
        % Enlarge wall image axis
        Safe_Set(D.UI.axH(1), 'Units', 'Pixels');
        img_ax_pos = get(D.UI.axH(1), 'Position');
        img_ax_pos = [img_ax_pos(1) - 27, ...
            img_ax_pos(2) - 27, ...
            img_ax_pos(3) + 54, ...
            img_ax_pos(4) + 54];
        Safe_Set(D.UI.axH(1), 'Position', img_ax_pos);
        Safe_Set(D.UI.axH(1), 'Units', 'Normalized');
        
        % Read in image
        [img{1}, ~, D.UI.wall_mask] = imread(D.DIR.wallImage);
        Safe_Set(D.UI.axH(1), 'XLim', [0,size(img{1},2)], 'YLim', [0,size(img{1},1)]);
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
        D.UI.wallImgH(1) = image(img{1}, 'Parent', D.UI.axH(1), 'Visible', 'on');
        D.UI.wallImgH(2) = image(img{2}, 'Parent', D.UI.axH(1), 'Visible', 'off');
        D.UI.wallImgH(3) = image(img{3}, 'Parent', D.UI.axH(1), 'Visible', 'off');
        % set alpha
        Safe_Set(D.UI.wallImgH, 'AlphaData', D.UI.wall_mask);
        
        % Clear vars
        clear img;
        
        %% ========================= ADD UI OBJECTS ===============================
        
        %% ---------------------------SETUP PANEL----------------------------------
        
        % Position settings
        pos_lft_dflt = 0.005;
        pos_wd_dflt = D.UI.stup_pan_pos(3)-0.01;
        obj_gap = 0.01;
        
        % Bottom start
        btm = D.UI.stup_pan_pos(2) + D.UI.stup_pan_pos(4) - 2*obj_gap;
        
        % SETUP PANEL
        
        % Panel
        D.UI.panStup = uipanel(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'FontSize',12, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.titleFont, ...
            'FontName', D.UI.titleFont, ...
            'Title','Setup', ...
            'TitlePosition','centertop', ...
            'Clipping','on', ...
            'Position', D.UI.stup_pan_pos);
        
        % SESSION TYPE SELECTION
        
        % Header text
        wd = pos_wd_dflt;
        ht = D.UI.fontSzTxtMed(2);
        btm = btm - ht - obj_gap;
        pos = [pos_lft_dflt, btm, wd, ht];
        D.UI.txtType = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','Session Type', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.headFont, ...
            'FontSize', D.UI.fontSzTxtMed(1));
        
        % Popupmenu
        btm = btm - D.UI.fontSzPopLrg(2);
        pos = [pos_lft_dflt, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popType = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Togg_Type}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName',D.UI.popFont, ...
            'FontSize',D.UI.fontSzPopLrg(1), ...
            'FontWeight','Bold', ...
            'String',[{''}; D.PAR.listSesType], ...
            'Value',1);
        
        % Toggle
        ht = 2.5*obj_gap;
        wd = D.UI.stup_pan_pos(3)*0.4;
        btm = btm - 3*obj_gap;
        lft = (D.UI.stup_pan_pos(3) - wd)/2;
        pos = [lft, btm, wd, ht];
        D.UI.toggType = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Togg_Type}, ...
            'String','Confirm', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1));
        
        % RAT SELECTION
        
        % Header text
        wd = pos_wd_dflt*0.65;
        ht = D.UI.fontSzTxtMed(2);
        btm = btm - 2*obj_gap;
        pos = [pos_lft_dflt, btm, wd, ht];
        D.UI.txtRat = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','Rat', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.headFont, ...
            'FontSize', D.UI.fontSzTxtMed(1));
        
        % Popupmenu
        btm = btm - D.UI.fontSzPopLrg(2);
        pos = [pos_lft_dflt, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popRat = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Pop_Rat}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName',D.UI.popFont, ...
            'FontSize',D.UI.fontSzPopMed(1), ...
            'FontWeight','Bold', ...
            'String',{''}, ...
            'Value',1);
        
        % HUMAN SELECTION
        
        % Header text
        lft = pos_lft_dflt + wd;
        wd = pos_wd_dflt*0.35;
        btm = btm + D.UI.fontSzPopLrg(2);
        pos = [lft, btm, wd, ht];
        D.UI.txtHuman = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','Human', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.headFont, ...
            'FontSize', D.UI.fontSzTxtMed(1));
        
        % Popupmenu
        btm = btm - D.UI.fontSzPopLrg(2);
        pos = [lft, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popHuman = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Pop_Human}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName',D.UI.popFont, ...
            'FontSize',D.UI.fontSzPopMed(1), ...
            'FontWeight','Bold', ...
            'String',[{''}; D.PAR.listHuman], ...
            'Value',1);
        
        % ICR CONDITION
        
        % Header text
        wd = pos_wd_dflt*0.65;
        btm = btm - 2*obj_gap;
        pos = [pos_lft_dflt, btm, wd, D.UI.fontSzTxtMed(2)];
        D.UI.txtCond = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','Condition', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.headFont, ...
            'FontSize', D.UI.fontSzTxtMed(1));
        
        % Popupmenu
        btm = btm - D.UI.fontSzPopLrg(2);
        pos = [pos_lft_dflt, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popCond = uicontrol('Style','popupmenu', ...
            'Parent', D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Pop_Cond}, ...
            'Units', 'Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName' ,D.UI.popFont, ...
            'FontSize', D.UI.fontSzPopSml(1), ...
            'FontWeight', 'Bold', ...
            'String', [{''}; D.PAR.listSesCond], ...
            'Value',1);
        
        % ICR TASK
        
        % Header text
        lft = pos_lft_dflt + wd;
        wd = pos_wd_dflt*0.35;
        btm = btm + D.UI.fontSzPopLrg(2);
        pos = [lft, btm, wd, D.UI.fontSzTxtMed(2)];
        D.UI.txtTask = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','Task', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.headFont, ...
            'FontSize', D.UI.fontSzTxtMed(1));
        
        % Popupmenu
        btm = btm - D.UI.fontSzPopLrg(2);
        pos = [lft, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popTask = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Pop_Task}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName',D.UI.popFont, ...
            'FontSize',D.UI.fontSzPopSml(1), ...
            'FontWeight','Bold', ...
            'String',[{''}; D.PAR.listSesTask], ...
            'Value',1);
        
        % REWARD DELAY
        % Header text
        btm = btm - 2*obj_gap;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtMed(2)];
        D.UI.txtRwDl = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','Reward Delay', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.headFont, ...
            'FontSize', D.UI.fontSzTxtMed(1));
        
        % Popupmenu
        btm = btm - D.UI.fontSzPopLrg(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzPopLrg(2)];
        D.UI.popRewDel = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Pop_RewDel}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName',D.UI.popFont, ...
            'FontSize',D.UI.fontSzPopMed(1), ...
            'FontWeight','Bold', ...
            'String',[{''}; D.PAR.listDel], ...
            'Value',1);
        
        % CUE CONDITION
        
        % Buttongroup
        ht =  D.UI.fontSzBtnMed(2) + 2.5*obj_gap;
        btm = btm - ht - obj_gap;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, ht];
        D.UI.spanCue = uibuttongroup(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor',D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Cue Condition', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',D.UI.fontSzTxtMed(1), ...
            'Clipping','off');
        
        % All
        wd = (pos(3)-2*pos_lft_dflt)/3;
        lft = pos_lft_dflt*2;
        ht =  D.UI.fontSzBtnMed(2);
        btm = pos(2)+pos(4) - ht - 2*obj_gap;
        pos = [lft, btm, wd, D.UI.fontSzBtnMed(2)];
        D.UI.toggCue(1) = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback',{@Togg_Cue}, ...
            'String','All', ...
            'UserData', 1, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1));
        
        % Half
        lft = lft+wd;
        pos = [lft, btm, wd, D.UI.fontSzBtnMed(2)];
        D.UI.toggCue(2) = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback',{@Togg_Cue}, ...
            'String','Half', ...
            'UserData', 2, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1));
        
        % None
        lft = lft+wd;
        pos = [lft, btm, wd, D.UI.fontSzBtnMed(2)];
        D.UI.toggCue(3) = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback',{@Togg_Cue}, ...
            'String','None', ...
            'UserData', 3, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1));
        
        % SOUND CONDITION
        
        % Buttongroup
        ht =  D.UI.fontSzBtnMed(2) + 2.5*obj_gap;
        btm = btm - ht - obj_gap;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, ht];
        % panel
        D.UI.spanSnd = uibuttongroup(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Sound', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',D.UI.fontSzTxtMed(1), ...
            'Clipping','off');
        
        % White Noise
        wd = (pos(3)-2*pos_lft_dflt)/2;
        lft = pos_lft_dflt*2;
        ht =  D.UI.fontSzBtnMed(2);
        btm = pos(2)+pos(4) - ht - 2*obj_gap;
        pos = [lft, btm, wd, D.UI.fontSzBtnMed(2)];
        D.UI.toggSnd(1) = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback',{@Togg_Sound}, ...
            'UserData', 1, ...
            'String','White', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1));
        
        % Reward Tone
        lft = lft+wd;
        pos = [lft, btm, wd, D.UI.fontSzBtnMed(2)];
        D.UI.toggSnd(2) = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback',{@Togg_Sound}, ...
            'UserData', 2, ...
            'String','Reward', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1));
        
        % SETUP DONE
        ht = 0.035;
        wd = D.UI.stup_pan_pos(3)*0.4;
        btm = D.UI.stup_pan_pos(2) + 0.01;
        lft = (D.UI.stup_pan_pos(3) - wd)/2;
        pos = [lft, btm, wd, ht];
        D.UI.toggSetupDone = uicontrol('Style','togglebutton', ...
            'Parent', D.UI.tabICR, ...
            'Enable', 'off', ...
            'String','DONE', ...
            'Callback', {@Togg_SetupDone}, ...
            'UserData', 0, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize',D.UI.fontSzTxtLrg(1));
        
        %% ----------------------------RUN PANEL-----------------------------------
        
        % Position settings
        pos_lft_dflt = 0.005;
        pos_wd_dflt = D.UI.run_pan_pos(3)-0.01;
        obj_gap = 0.01;
        
        % Bottom start
        btm = D.UI.run_pan_pos(2) + D.UI.run_pan_pos(4) - 2*obj_gap;
        
        % RUN PANEL
        D.UI.panRun = uipanel(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'FontSize', D.UI.fontSzTxtLrg(1), ...
            'FontWeight','Bold', ...
            'FontName', D.UI.titleFont, ...
            'Title','Run', ...
            'TitlePosition','centertop', ...
            'Clipping','on', ...
            'Position',  D.UI.run_pan_pos);
        
        % NEURALYNX SUBPANEL
        
        % Bottongorup
        ht =  D.UI.fontSzBtnLrg(2) + D.UI.fontSzBtnLrg(2) + 3.5*obj_gap;
        btm = btm - ht - obj_gap;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, ht];
        D.UI.spanNLX = uibuttongroup(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Cheetah', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',D.UI.fontSzTxtMed(1), ...
            'Clipping','off');
        
        % Acquire
        wd = (pos_wd_dflt-2*pos_lft_dflt)/2;
        lft = pos_lft_dflt*2;
        ht =  D.UI.fontSzBtnLrg(2);
        btm = pos(2)+pos(4) - ht - 2*obj_gap;
        pos = [lft, btm, wd, ht];
        D.UI.toggAcq = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback',{@Togg_Acq}, ...
            'String','ACQ', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1));
        
        % Record
        lft = lft+wd;
        pos = [lft, btm, wd, D.UI.fontSzBtnLrg(2)];
        D.UI.toggRec = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback',{@Togg_Rec}, ...
            'String','REC', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1));
        
        % Cells cut toggle
        ht = D.UI.fontSzBtnLrg(2);
        btm = btm - ht - 0.5*obj_gap;
        wd = (pos_wd_dflt-2*pos_lft_dflt)/2;
        lft = pos_lft_dflt*2;
        pos = [lft, btm, wd, ht];
        D.UI.toggLoadClust = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','Load Clust', ...
            'Callback', {@Togg_LoadClust}, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1), ...
            'UserData', 0);
        
        % Cube LED
        lft = lft + wd;
        pos = [lft, btm, wd/2, ht];
        D.UI.toggCubeLED = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','QLED', ...
            'Callback', {@Togg_CubeLED}, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1), ...
            'UserData', 0);
        
        % Cube VCC Update
        lft = lft + wd/2;
        pos = [lft, btm, wd/2, ht];
        D.UI.toggCubeVcc = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','QVCC', ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1), ...
            'UserData', 0);
        f = @(x,y) (Button_State(x,y));
        set(D.UI.toggCubeVcc, 'Callback', @(x,y)f(D.UI.toggCubeVcc,'Update'));
        
        % Set to pan bottom
        btm = D.UI.spanNLX.Position(2);
        
        % TASK SUBPANEL
        
        % Buttongroup
        ht =  4*D.UI.fontSzTxtLrg(2) + 4*obj_gap;
        btm = btm - ht - obj_gap;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, ht];
        D.UI.spanTask = uibuttongroup(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Task', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',D.UI.fontSzTxtMed(1), ...
            'Clipping','off');
        
        % Sleep 1 button
        wd = (pos_wd_dflt - 2*pos_lft_dflt)/2;
        lft = 2*pos_lft_dflt;
        ht = D.UI.fontSzTxtLrg(2);
        btm = pos(2)+pos(4) - ht - 2*obj_gap;
        pos = [lft, btm, wd*0.9, ht];
        D.UI.toggSleep(1) = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'String', 'Sleep 1', ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzTxtLrg(1), ...
            'UserData', [1, 0]);
        
        % Sleep 1 edit
        str = ...
            sprintf('%s/%s', ...
            datestr(0, 'MM:SS'), ...
            datestr(D.PAR.sleepDur(1)/(24*60*60), 'MM:SS'));
        pos = [lft+pos(3), btm, wd*1.1, ht];
        D.UI.editSleep(1) = uicontrol(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized',...
            'Position',pos,...
            'Style','edit',...
            'HorizontalAlignment', 'Left', ...
            'FontSize', D.UI.fontSzBtnMed(1), ...
            'FontName','Monospaced', ...
            'Max', 1, ...
            'Enable','off',...
            'Visible', 'on', ...
            'String',str);
        
        % ICR buttons
        ht = 2*D.UI.fontSzTxtLrg(2);
        btm = btm - ht - 0.5*obj_gap;
        pos = [lft, btm, wd, ht];
        D.UI.toggICR(1) = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',  D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzTxtLrg(1));
        D.UI.toggICR(2) = copyobj(D.UI.toggICR(1), D.UI.tabICR);
        pos = [lft+wd, btm, wd, 2*D.UI.fontSzTxtLrg(2)];
        Safe_Set(D.UI.toggICR(2), 'Position', pos);
        Safe_Set(D.UI.toggICR, 'Callback', {@Togg_ICR})
        
        % Sleep 2 button
        ht = D.UI.fontSzTxtLrg(2);
        btm = btm - ht - 0.5*obj_gap;
        pos = [lft, btm, wd*0.9, ht];
        D.UI.toggSleep(2) = copyobj(D.UI.toggSleep(1), D.UI.tabICR);
        Safe_Set(D.UI.toggSleep(2), ...
            'String', 'Sleep 2', ...
            'UserData', [2, 0], ...
            'Position', pos);
        
        % Sleep 2 edit
        str = ...
            sprintf('%s/%s', ...
            datestr(0, 'MM:SS'), ...
            datestr(D.PAR.sleepDur(2)/(24*60*60), 'MM:SS'));
        pos = [lft+pos(3), btm, wd*1.1, ht];
        D.UI.editSleep(2) = copyobj(D.UI.editSleep(1), D.UI.tabICR);
        Safe_Set(D.UI.editSleep(2), ...
            'String', str, ...
            'Position', pos);
        
        % Set sleep button callback
        Safe_Set(D.UI.toggSleep, 'Callback', {@Togg_Sleep})
        
        % Set to pan bottom
        btm = D.UI.spanTask.Position(2);
        
        % ROBOT SUBPANEL
        
        % Buttongroup
        ht =  2*D.UI.fontSzBtnLrg(2) + D.UI.fontSzPopLrg(2) + 3*obj_gap;
        btm = btm - ht - obj_gap;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, ht];
        D.UI.spanRob = uibuttongroup(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Robot', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',D.UI.fontSzTxtMed(1), ...
            'Clipping','off');
        
        % Halt
        wd = pos(3)-2*pos_lft_dflt;
        lft = pos_lft_dflt*2;
        ht =  2*D.UI.fontSzBtnLrg(2);
        btm = pos(2)+pos(4) - ht - 2*obj_gap;
        pos = [lft, btm, wd, ht];
        D.UI.toggHaltRob = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','Halt Robot', ...
            'Callback', {@Togg_HaltRob}, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzTxtLrg(1));
        
        % Bulldoze popup
        ht = D.UI.fontSzPopLrg(2);
        btm = btm - ht - 0.5*obj_gap;
        pos = [lft, btm, wd, ht];
        D.UI.popBulldoze = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Callback', {@Pop_Bulldoze}, ...
            'Units','Normalized', ...
            'Enable', 'off', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName',D.UI.popFont, ...
            'FontSize',D.UI.fontSzPopLrg(1), ...
            'FontWeight','Bold', ...
            'String',D.PAR.listBull);
        
        % Bulldoze toggle
        pos = [lft, pos(2), wd-0.0425, pos(4)];
        D.UI.toggBulldoze = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','Bulldoze', ...
            'Callback', {@Pop_Bulldoze}, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1));
        
        % Bulldoze speed field
        lft = lft+pos(3);
        pos = [lft, pos(2), 0.03, pos(4)];
        D.UI.editBulldoze = uicontrol(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized',...
            'Position',pos,...
            'Style','edit',...
            'HorizontalAlignment', 'Left', ...
            'FontSize', D.UI.fontSzBtnLrg(1), ...
            'FontName','Monospaced', ...
            'Max', 1, ...
            'Enable','off',...
            'Visible', 'on', ...
            'String',num2str(D.PAR.bullSpeed));
        
        % Set to pan bottom
        btm = D.UI.spanRob.Position(2);
        
        % REWARD SUBPANEL
        
        % Buttongroup
        ht =  3*D.UI.fontSzBtnLrg(2) + D.UI.fontSzPopLrg(2) + 5*obj_gap;
        btm = btm - ht - obj_gap;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, ht];
        D.UI.spanRew = uibuttongroup(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Reward', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',D.UI.fontSzTxtMed(1), ...
            'Clipping','off');
        
        % Reward duration popup
        wd = pos(3)-2*pos_lft_dflt;
        lft = pos_lft_dflt*2;
        ht = D.UI.fontSzPopLrg(2);
        btm = pos(2)+pos(4) - ht - 2*obj_gap;
        pos = [lft, btm, wd, ht];
        D.UI.popReward = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Enable', 'off', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName',D.UI.popFont, ...
            'FontSize',D.UI.fontSzBtnLrg(1), ...
            'FontWeight','Bold', ...
            'String',cellstr(num2str(D.PAR.zoneRewDur')), ...
            'Value', D.I.zone_now);
        
        % Reward toggle
        pos = [pos(1), pos(2), wd-0.0125, pos(4)];
        D.UI.btnReward = uicontrol('Style','push', ...
            'Parent',D.UI.tabICR, ...
            'String','Reward', ...
            'Callback', {@Btn_Reward}, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1), ...
            'Value',0);
        
        % Cue reward
        ht = D.UI.fontSzBtnLrg(2);
        btm = btm - ht - 0.5*obj_gap;
        wd = (pos_wd_dflt-2*pos_lft_dflt);
        pos = [pos(1), btm, wd, ht];
        D.UI.toggDoCue = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','Cue Reward', ...
            'Callback', {@Togg_DoCue}, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1));
        
        % Block cue
        ht = D.UI.fontSzBtnLrg(2);
        btm = btm - ht - 0.5*obj_gap;
        wd = (pos_wd_dflt-2*pos_lft_dflt)/2;
        pos = [pos(1), btm, wd, ht];
        D.UI.toggBlockCue = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','Block Cue', ...
            'Callback', {@Togg_BlockCue}, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1));
        
        % Force cue
        lft = lft+wd;
        pos = [lft, pos(2), pos(3), pos(4)];
        D.UI.toggForceCue = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','Force Cue', ...
            'Callback', {@Togg_ForceCue}, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1));
        
        % Pick reward pos
        ht = D.UI.fontSzBtnLrg(2);
        btm = btm - ht - 0.5*obj_gap;
        wd = pos_wd_dflt-2*pos_lft_dflt;
        lft = pos_lft_dflt*2;
        pos = [lft, btm, wd, ht];
        D.UI.toggPickRewPos = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'String','Select Reward Zone', ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'Callback', {@Togg_PickRewPos}, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1));
        
        % TASK DONE
        ht = 0.035;
        wd = D.UI.run_pan_pos(3)*0.4;
        btm = D.UI.run_pan_pos(2) + 0.01;
        lft = (D.UI.run_pan_pos(3) - wd)/2;
        pos = [lft, btm, wd, ht];
        D.UI.toggTaskDone = uicontrol('Style','togglebutton', ...
            'Parent', D.UI.tabICR, ...
            'Enable', 'off', ...
            'String','DONE', ...
            'Callback', {@Togg_TaskDone}, ...
            'UserData', 0, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize',D.UI.fontSzTxtLrg(1));
        
        %% --------------------------CONSOLE PANEL--------------------------------
        
        % CONSOLE PANEL
        D.UI.panConsole = uipanel(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'FontSize',D.UI.fontSzTxtLrg(1), ...
            'FontWeight','Bold', ...
            'FontName', D.UI.titleFont, ...
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
            'FontSize', D.UI.fontSzTxtMed(1), ...
            'FontName','Monospaced', ...
            'Max', 1000, ...
            'Enable','on',...
            'String',D.DB.consoleStr);
        
        %% ---------------------------OTHER STUFF---------------------------------
        
        % MONITOR SELECT
        for z_m = 1:3
            val = z_m == D.UI.monDefault;
            D.UI.toggMon(z_m) = uicontrol('Style','togglebutton', ...
                'Parent', D.UI.tabICR, ...
                'Enable', 'off', ...
                'String', sprintf('M%d', z_m), ...
                'Callback', {@Togg_Mon}, ...
                'Units', 'Normalized', ...
                'Position', D.UI.mon_btn_pos, ...
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.disabledBtnFrgCol, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', D.UI.fontSzTxtLrg(1), ...
                'UserData', z_m, ...
                'Value', val);
            
            % Shift left
            D.UI.mon_btn_pos(1) = D.UI.mon_btn_pos(1)+D.UI.mon_btn_pos(3);
        end
        
        % SAVE & AND QUIT SESSION
        
        % Save button
        D.UI.toggSave = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'String','SAVE', ...
            'Callback', {@Togg_Save}, ...
            'Unit', 'Normalized', ...
            'Position', D.UI.save_btn_pos, ...
            'FontName', D.UI.btnFont, ...
            'FontSize',D.UI.fontSzBtnHuge(1), ...
            'UserData', 0);
        
        % Quit button
        D.UI.toggQuit = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'String','QUIT', ...
            'Callback', {@Togg_Quit}, ...
            'Unit', 'Normalized', ...
            'Position', D.UI.quit_btn_pos, ...
            'FontName', D.UI.btnFont, ...
            'FontSize',D.UI.fontSzBtnHuge(1), ...
            'UserData', 0);
        
        % CLEAR VT BUTTON
        ht = 0.02;
        lft = D.UI.tim_inf_pan_pos(1);
        btm = D.UI.cnsl_pan_pos(4) - ht*1.5;
        wd = D.UI.tim_inf_pan_pos(3);
        pos = [lft, btm, wd, ht];
        D.UI.btnClrVT = uicontrol('Style','push', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Btn_ClrVT}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'String','Clear VT Data', ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1), ...
            'Value',0);
        
        % CLEAR TT BUTTON
        pos = [pos(1), pos(2)-ht, pos(3), pos(4)];
        D.UI.btnClrTT = uicontrol('Style','push', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Callback', {@Btn_ClrTT}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'String','Clear TT Data', ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1), ...
            'Visible', 'off', ...
            'Value',0);
        
        % START BUTTON
        pos = [0, 0, 0.04, 0.02];
        D.UI.btnStart = uicontrol('Style','togglebutton', ...
            'Parent',D.UI.tabICR, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'String','START', ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzBtnMed(1), ...
            'Visible', 'off', ...
            'Value',0);
        
        % MOUSE ACTIONS
        set(FIGH, 'WindowButtonMotionFcn', @Mouse_Track)
        
        % CHANGE SIZE FUNCTION
        set(FIGH, 'SizeChangedFcn', @SizeChanged_GetPosUI)
        
        %% --------------------------PRINTED INFO----------------------------------
        
        % Position settings
        pos_lft_dflt = (D.UI.main_ax_bounds(1)+D.UI.main_ax_bounds(3)) + 0.005;
        pos_wd_dflt = 1 - (D.UI.main_ax_bounds(1)+D.UI.main_ax_bounds(3)) - 2*0.005;
        
        % RAT INFO
        
        % Pannel
        D.UI.panSesInf = uipanel(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'Position',  D.UI.ses_inf_pan_pos);
        
        % Heading
        btm = D.UI.ses_inf_pan_pos(2) + ...
            D.UI.ses_inf_pan_pos(4) - ...
            D.UI.fontSzTxtLrg(2) - ...
            5*D.UI.pxl2norm_y;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtLrg(2)];
        D.UI.txtSesInfHed = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','Session Info', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.titleFont, ...
            'FontSize', D.UI.fontSzTxtLrg(1));
        
        % ICR ses info
        btm = btm - 6.5*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)*6];
        D.UI.txtSesInf(1) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'Visible', 'off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Rotation ses info
        btm = btm - 2.5*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)*2];
        D.UI.txtSesInf(2) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'Visible', 'off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Laps per dropdown
        btm = btm - D.UI.fontSzPopLrg(2);
        wd = pos_wd_dflt/2;
        pos = [pos_lft_dflt, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popLapsPerRot = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',D.UI.fontSzPopSml(1), ...
            'FontWeight','Light', ...
            'Visible','off', ...
            'Value',1);
        
        % Rotation position dropdown
        wd = pos_wd_dflt/2;
        lft = pos_lft_dflt+pos_wd_dflt/2;
        pos = [lft, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popRotPos = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',D.UI.fontSzPopSml(1), ...
            'FontWeight','Light', ...
            'Visible','off', ...
            'Value',1);
        
        % PERFORMANCE INFO
        
        % Pannel
        D.UI.panPerfInf = uipanel(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'Position',  D.UI.perf_inf_pan_pos);
        
        % Heading
        btm = D.UI.perf_inf_pan_pos(2) + ...
            D.UI.perf_inf_pan_pos(4) - ...
            D.UI.fontSzTxtLrg(2) - ...
            5*D.UI.pxl2norm_y;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtLrg(2)];
        D.UI.txtPerfInfHed = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','Performance Info', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.titleFont, ...
            'FontSize', D.UI.fontSzTxtLrg(1));
        
        % Totals
        btm = btm - 5.5*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)*5];
        D.UI.txtPerfInf(4) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible', 'off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Lap time dropdown
        btm = btm - D.UI.fontSzPopLrg(2);
        wd = pos_wd_dflt/2;
        pos = [pos_lft_dflt, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popLapTim = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'String','Lap Times', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',D.UI.fontSzPopSml(1), ...
            'FontWeight','Light', ...
            'Visible','off', ...
            'Value',1);
        
        % Reward info dropdown
        wd = pos_wd_dflt/2;
        lft = pos_lft_dflt+pos_wd_dflt/2;
        pos = [lft, btm, wd, D.UI.fontSzPopLrg(2)];
        D.UI.popRewInfo = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.tabICR, ...
            'String','Reward Info', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',D.UI.fontSzPopSml(1), ...
            'FontWeight','Light', ...
            'Visible','off', ...
            'Value',1);
        
        % Standard laps/reward
        btm = btm - 2.5*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)*2];
        D.UI.txtPerfInf(1) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', [0.5,0.5,0.5], ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible','off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % 40 deg laps/reward
        btm = btm - 2*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)*2];
        D.UI.txtPerfInf(2) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.rotCol(2,:), ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible','off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % 0 deg laps/reward
        btm = btm - 2*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)*2];
        D.UI.txtPerfInf(3) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.rotCol(1,:), ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible','off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Rat velocity
        btm = btm - 1.5*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)];
        D.UI.txtPerfInf(5) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.ratNowCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible','off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Robot velocity
        btm = btm - D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)];
        D.UI.txtPerfInf(6) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.robNowCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible','off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Robot battery voltage
        btm = btm - 1.5*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)];
        D.UI.txtPerfInf(7) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible','off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Cube battery voltage
        btm = btm - D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, pos_wd_dflt, D.UI.fontSzTxtSml(2)];
        D.UI.txtPerfInf(8) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible','off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Debug info
        ht = btm - D.UI.perf_inf_pan_pos(2) - D.UI.fontSzTxtSml(2);
        btm = btm - ht;
        pos = [pos_lft_dflt, btm, pos_wd_dflt, ht];
        D.UI.txtPerfInf(9) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'Visible','off', ...
            'FontSize', 6);
        
        % TIMER INFO
        pos_lft_dflt = D.UI.tim_inf_pan_pos(1) + 0.005;
        pos_wd_dflt = D.UI.tim_inf_pan_pos(3) - 0.01;
        
        % Pannel
        D.UI.panTimInf = uipanel(...
            'Parent',D.UI.tabICR, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'FontSize',D.UI.fontSzTxtLrg(1), ...
            'FontWeight','Bold', ...
            'FontName', D.UI.titleFont, ...
            'Title','Timers', ...
            'TitlePosition','centertop', ...
            'Clipping','on', ...
            'Position',  D.UI.tim_inf_pan_pos);
        
        % Time elapsed info
        btm = D.UI.tim_inf_pan_pos(2) + ...
            D.UI.tim_inf_pan_pos(4) - ...
            5.5*D.UI.fontSzTxtSml(2) - ...
            0.01;
        pos = [pos_lft_dflt, btm, ...
            pos_wd_dflt, D.UI.fontSzTxtSml(2)*4];
        D.UI.txtTimeInf(1) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'Visible','off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Time start text
        btm = btm - 1.5*D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, ...
            pos_wd_dflt, D.UI.fontSzTxtSml(2)*1];
        D.UI.txtTimeInf(2) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'Visible', 'off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        
        % Time start edit
        pos = [pos(1)+ pos_wd_dflt*0.4, pos(2), ...
            pos_wd_dflt*0.55, pos(4)];
        D.UI.editTimeInf(1) = uicontrol(...
            'Parent',D.UI.tabICR,...
            'Units','normalized',...
            'Position',pos,...
            'Style','edit',...
            'HorizontalAlignment', 'Center', ...
            'FontSize', D.UI.fontSzTxtSml(1), ...
            'FontName','Monospaced', ...
            'Max', 1, ...
            'Visible', 'off', ...
            'Enable','on');
        
        % Time stop text
        btm = btm - D.UI.fontSzTxtSml(2);
        pos = [pos_lft_dflt, btm, ...
            pos_wd_dflt, D.UI.fontSzTxtSml(2)*1];
        D.UI.txtTimeInf(3) = uicontrol('Style','text', ...
            'Parent',D.UI.tabICR, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'Visible', 'off', ...
            'FontSize', D.UI.fontSzTxtSml(1));
        pos = [pos(1)+pos_wd_dflt*0.4, pos(2), ...
            pos_wd_dflt*0.55, pos(4)];
        
        % Time stop edit
        D.UI.editTimeInf(2) = uicontrol(...
            'Parent',D.UI.tabICR,...
            'Units','normalized',...
            'Position',pos,...
            'Style','edit',...
            'HorizontalAlignment', 'Center', ...
            'FontSize', D.UI.fontSzTxtSml(1), ...
            'FontName','Monospaced', ...
            'Max', 1, ...
            'Visible', 'off', ...
            'Enable','on');
        
        %% ========================================================================
        
        %% ====================== MAKE FIGURE VISIBLE =============================
        
        % Bring UI to top
        uistack(FIGH, 'top')
        
        % Make visible
        set(FIGH, 'Visible', 'on');
        
        % Enable monitor toggles
        Button_State(D.UI.toggMon, 'Enable')
        
    end

% ------------------------------NLX SETUP--------------------------
    function[] = NLX_Start()
        
        %% DEFINE IO PARAMETERS
        
        % Note: Opto issolator is powered of of Port-0 Bit-3
        % Each event has a unique bit associated with it
        
        % DygitalLynx server IP
        if strcmp('ICRCHEETAH', getenv('computername'))
            % Recording room computer
            D.NLX.ServerIP = '192.168.3.100';
        else
            % Analysis computer
            D.NLX.ServerIP = '127.0.0.1';
        end
        
        % Aquisition ent names
        D.NLX.vt_rat_ent = 'VT1';
        D.NLX.vt_rob_ent = 'VT2';
        D.NLX.event_ent = 'Events';
        
        % TTL board/port parameters
        D.NLX.DevTTL = 'AcqSystem1_0'; % TTL board name
        
        % Port labels
        D.NLX.port_0 = '0';
        D.NLX.port_1 = '1';
        
        % INPUT TTL PIN/BIT
        % vars = (port, pin)
        
        % Audio channels
        D.NLX.snd_rt_wn_bit = [{D.NLX.port_1},{'4'}];
        D.NLX.snd_lft_rt_bit = [{D.NLX.port_1},{'5'}];
        
        % IR time sync LED
        D.NLX.ir_sync_bit = [{D.NLX.port_1},{'7'}];
        
        % Rew
        D.NLX.rew_on_bit = [{D.NLX.port_1},{'0'}];
        D.NLX.rew_off_bit = [{D.NLX.port_1},{'1'}];
        
        % PID mode
        D.NLX.pid_run_bit = [{D.NLX.port_0},{'0'}];
        D.NLX.pid_stop_bit = [{D.NLX.port_0},{'1'}];
        
        % Bulldozer state
        D.NLX.bull_run_bit = [{D.NLX.port_0},{'2'}];
        D.NLX.bull_stop_bit = [{D.NLX.port_0},{'3'}];
        
        % Photo Transducers
        D.NLX.north_bit = [{D.NLX.port_0},{'7'}];
        D.NLX.west_bit = [{D.NLX.port_0},{'6'}];
        D.NLX.south_bit = [{D.NLX.port_0},{'5'}];
        D.NLX.east_bit = [{D.NLX.port_0},{'4'}];
        
        % TTL STRINGS
        
        % Audio channels
        D.NLX.snd_rt_wn_str = 'TTL_Sound_Right_White_Noise_On';
        D.NLX.snd_lft_rt_str = 'TTL_Sound_Left_Reward_Tone_On';
        
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
        
        % Sleep 1
        D.NLX.sleep_start_evt{1} = '-PostEvent Sleep_1_Start 207 0';
        D.NLX.sleep_end_evt{1} = '-PostEvent Sleep_1_End 208 0';
        
        % Sleep 2
        D.NLX.sleep_start_evt{2} = '-PostEvent Sleep_2_Start 209 0';
        D.NLX.sleep_end_evt{2} = '-PostEvent Sleep_2_End 210 0';
        
        % Reward Zoneet and Duration
        D.NLX.rew_evt = '-PostEvent Post_Reward_Zone:%d_Duration:%d 211 0';
        
        % Session end command string
        D.NLX.ses_end_evt = '-PostEvent Post_Session_End 212 0';
        
        %% MAKE SURE SPIKESORT CLOSED
        
        % Get EXE status
        D.F.spikesort_running = Check_EXE('SpikeSort3D.exe');
        
        % Prompt to close
        if D.F.spikesort_running
            
            % Show prompt
            dlg_h = dlgAWL( ...
                'Close SpikeSort3D', ...
                'CLOSE SPIKESORT3D', ...
                'OK', [], [], 'OK', ...
                D.UI.dlgPos{4}, ...
                'default');
            Dlg_Wait(dlg_h);
            
        end
        
        %% START CHEETAH
        
        % Check if Cheetah open
        D.F.cheetah_running = Check_EXE('Cheetah.exe');
        
        % Only run if not running Table_Update solo
        if ~D.F.cheetah_running && D.PAR.sesType ~= 'Table_Update'
            
            % Log/print
            Console_Write('[NLX_Start] RUNNING: Open Cheetah.exe...');
            
            % Check if Cheetah should be run for solo session
            if ISMATSOLO
                dlg_h = dlgAWL(...
                    'Do you want to run Cheetah.exe?', ...
                    'RUN CHEETAH?', ...
                    'Yes', 'No', [], 'No', ...
                    D.UI.dlgPos{4}, ...
                    'question');
                choice = Dlg_Wait(dlg_h);
                
            else
                choice = 'Yes';
            end
            
            % Open Cheetah
            if strcmp(choice, 'Yes') && ~DOEXIT
                
                % NLX setup config
                if D.F.implant_session
                    start_cfg_path = fullfile(D.DIR.nlxCfg, 'ICR_Cheetah_Load_Ephys.cfg');
                else
                    start_cfg_path = fullfile(D.DIR.nlxCfg, 'ICR_Cheetah_Load_Behavior.cfg');
                end
                
                % Store current directory
                curdir = pwd;
                
                % Run EXE with specified config
                cd(D.DIR.nlxCheetaEXE);
                system(sprintf('Cheetah.exe %s&', start_cfg_path));
                cd(curdir);
                
                % Log/print
                Console_Write(sprintf('[NLX_Start] FINISHED: Run Cheetah.exe "%s"', start_cfg_path));
                
                % Set flag
                D.F.cheetah_running = true;
                
            elseif DOEXIT
                
                % Bail
                Console_Write('**WARNING** [NLX_Start] ABORTED: Run Cheetah.exe');
                return
                
            else
                
                % Bail
                Console_Write('[NLX_Start] SKIPPED: Run Cheetah.exe');
                return
                
            end
            
        end
        
    end

% -----------------------------TABLE SETUP-------------------------
    function[] = Table_Setup()
        
        %% SETUP TABLE UI
        
        % Add TABLE tab
        D.UI.tabTBL = uitab(D.UI.tabgp, ...
            'Title', 'TABLE', ...
            'BackgroundColor', [1, 1, 1]);
        
        % GET POSITION INFO
        
        % Defaults
        obj_gap = 0.005;
        
        % Panel group pos
        grp_lft = 0;
        grp_ht = (D.UI.tabTBL.Position(4)*0.75) - D.UI.run_pan_pos(2);
        grp_botm = D.UI.run_pan_pos(2);
        grp_wdth = D.UI.tabTBL.Position(3);
        D.UI.tab_grp_pos = ...
            [grp_lft, ...
            grp_botm, ...
            grp_wdth, ...
            grp_ht];
        
        % Panel weight
        pan_lft = 0;
        pan_ht = 0.14;
        pan_botm = 1 - pan_ht - obj_gap;
        pan_wdth = 0.2;
        D.UI.weight_pan_pos = ...
            [pan_lft, ...
            pan_botm, ...
            pan_wdth, ...
            pan_ht];
        
        % Panel food
        pan_lft = 0;
        pan_ht = pan_botm - sum(D.UI.tab_grp_pos([2,4])) - 2*obj_gap;
        pan_botm = sum(D.UI.tab_grp_pos([2,4])) + obj_gap;
        pan_wdth = 0.2;
        D.UI.food_pan_pos = ...
            [pan_lft, ...
            pan_botm, ...
            pan_wdth, ...
            pan_ht];
        
        % Panel General notes
        pan_lft = sum(D.UI.weight_pan_pos([1,3])) + obj_gap;
        pan_ht = 1 - D.UI.food_pan_pos(2) - obj_gap;
        pan_botm = D.UI.food_pan_pos(2);
        pan_wdth = (1 - sum(D.UI.weight_pan_pos([1,3])) - 2*obj_gap) / 2;
        D.UI.gen_notes_pan_pos = ...
            [pan_lft, ...
            pan_botm, ...
            pan_wdth, ...
            pan_ht];
        
        % Panel TT notes
        pan_lft = sum(D.UI.gen_notes_pan_pos([1,3])) + obj_gap;
        pan_ht = D.UI.gen_notes_pan_pos(4);
        pan_botm = D.UI.gen_notes_pan_pos(2);
        pan_wdth = D.UI.gen_notes_pan_pos(3);
        D.UI.tt_notes_pan_pos = ...
            [pan_lft, ...
            pan_botm, ...
            pan_wdth, ...
            pan_ht];
        
        %% CREATE TEMPLATE OBJECTS
        Console_Write('[Table_Setup] RUNNING: Create Template Objects');
        
        % Create tab group
        D.UI.tblTabSubGrp = ...
            uitabgroup(D.UI.tabTBL, ...
            'Units', 'Normalized', ...
            'UserData', 'TT TRACK', ...
            'Position',D.UI.tab_grp_pos);
        
        % Table Template
        D.UI.tblTemplate = uitable(...
            D.UI.tabTBL, ...
            'Units', 'normalized', ...
            'Position', [0,0,1,1], ...
            'FontSize', 8, ...
            'FontName', 'MonoSpace', ...
            'Enable', 'on', ...
            'ColumnEditable', true, ...
            'Visible', 'off');
        
        % Panel Template
        D.UI.panTemplate = uipanel(...
            'Parent',D.UI.tabTBL, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'FontSize',12, ...
            'FontWeight','Bold', ...
            'FontName', D.UI.titleFont, ...
            'FontName', D.UI.titleFont, ...
            'TitlePosition','centertop', ...
            'Clipping','on', ...
            'Position', [0,0,0.1,0.1], ...
            'Visible', 'off');
        
        % Text Template
        D.UI.txtTemplate = uicontrol(...
            'Style','text', ...
            'Parent',D.UI.tabTBL, ...
            'Units','Normalized', ...
            'Position', [0,0,0.1,D.UI.fontSzTxtMed(2)], ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName',D.UI.txtFont, ...
            'FontSize', D.UI.fontSzTxtMed(1), ...
            'Visible', 'off');
        
        % Edit Template
        D.UI.editTemplate = uicontrol(...
            'Style','edit',...
            'Parent',D.UI.tabTBL, ...
            'Units','Normalized',...
            'Position',[0,0,0.1,D.UI.fontSzTxtLrg(2)],...
            'HorizontalAlignment', 'Left', ...
            'FontSize', D.UI.fontSzTxtLrg(1), ...
            'FontName','Monospaced', ...
            'Max', 1, ...
            'Enable','inactive',...
            'Visible', 'off');
        
        % Popupmenu Template
        D.UI.popTemplate = uicontrol(...
            'Style','popupmenu', ...
            'Parent',D.UI.tabTBL, ...
            'Units','Normalized', ...
            'Position', [0,0,0.1,D.UI.fontSzPopSml(2)], ...
            'Enable', 'off', ...
            'BackgroundColor',D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName',D.UI.popFont, ...
            'FontSize',D.UI.fontSzPopSml(1), ...
            'FontWeight','Bold', ...
            'String', ' ', ...
            'Value',1);
        
        % Togle Template
        D.UI.toggTemplate = uicontrol(...
            'Style','togglebutton', ...
            'Parent',D.UI.tabTBL, ...
            'Enable', 'off', ...
            'Units','Normalized', ...
            'Position', [0,0,0.1,D.UI.fontSzTxtLrg(2)], ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight','Bold', ...
            'FontSize', D.UI.fontSzTxtMed(1));
        
        %% CREATE WEIGHT PANEL OBJECTS
        Console_Write('[Table_Setup] RUNNING: Create Weight Panel Objects');
        
        % GET WEIGHT VALUES
        
        % Get baseline weight
        D.PAR.ratWeightBaseline = D.SS_IO_1.Weight_Baseline(D.PAR.ratIndSS);
        
        % Get last corrected weight
        D.PAR.ratWeightLast = D.SS_IO_3.(D.PAR.ratLab).Weight_Corrected(end);
        
        % Get drive weight and cap weights
        if D.F.rat_implanted
            D.PAR.driveWeight = D.TT_IO_1.Weight_Drive(D.PAR.ratIndTT);
            D.PAR.capWeightArr = D.TT_IO_1.Cap_Weights{D.PAR.ratIndTT};
        else
            D.PAR.driveWeight = 0;
            D.PAR.capWeight = 0;
        end
        
        % ADD WEIGHT OBJECTS
        
        % Weight Panel
        D.UI.panWeight = copyobj(D.UI.panTemplate, D.UI.tabTBL);
        Safe_Set(D.UI.panWeight, ...
            'Title','Weights', ...
            'Visible', 'on', ...
            'Position', D.UI.weight_pan_pos)
        
        % Starting pos
        botm = sum(D.UI.weight_pan_pos([2,4])) - D.UI.fontSzTxtMed(2)*3;
        lft = D.UI.weight_pan_pos(1) + obj_gap;
        wd =  (D.UI.weight_pan_pos(3) - obj_gap*2)/2 - obj_gap;
        
        % New weight text
        D.UI.txtNewWeight = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtNewWeight.Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtNewWeight, ...
            'String', 'New Weight:', ...
            'HorizontalAlignment', 'Left', ...
            'Visible', 'on');
        
        % New weight edit
        lft_new = lft + wd + obj_gap;
        D.UI.editNewWeight = copyobj(D.UI.editTemplate, D.UI.tabTBL);
        D.UI.editNewWeight.Position(1:3) = [lft_new, botm, wd];
        Safe_Set(D.UI.editNewWeight, ...
            'Enable','on',...
            'Visible', 'on');
        
        % Cap weight text
        botm = botm - D.UI.txtTemplate.Position(4) - obj_gap;
        D.UI.txtCapWeight = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtCapWeight.Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtCapWeight, ...
            'String', 'Cap Weight:', ...
            'HorizontalAlignment', 'Left', ...
            'Visible', 'on');
        
        % Cap weight pop
        lft_new = lft + wd + obj_gap;
        D.UI.popCapWeight = copyobj(D.UI.popTemplate, D.UI.tabTBL);
        D.UI.popCapWeight.Position(1:3) = [lft_new, botm, wd];
        Safe_Set(D.UI.popCapWeight, ...
            'String', D.PAR.capWeightList, ...
            'Visible', 'on');
        
        % Enable if implanted
        if D.F.rat_implanted
            Safe_Set(D.UI.popCapWeight, 'Enable', 'On');
        end
        
        % Update weight button
        wd_new = sum(D.UI.weight_pan_pos([1,3]))*0.3;
        lft_new = D.UI.weight_pan_pos(1) + sum(D.UI.weight_pan_pos([1,3]))*0.35;
        botm = botm - D.UI.txtTemplate.Position(4) - 2*obj_gap;
        D.UI.toggUpdateWeight = copyobj(D.UI.toggTemplate, D.UI.tabTBL);
        D.UI.toggUpdateWeight.Position(1:3) = [lft_new, botm, wd_new];
        Safe_Set(D.UI.toggUpdateWeight, ...
            'String', 'Update', ...
            'Callback', {@Togg_UpdateWeight}, ...
            'Visible', 'on');
        
        % Enable button
        Button_State(D.UI.toggUpdateWeight, 'Enable');
        
        % Baseline weight text
        wd =  (D.UI.weight_pan_pos(3) - obj_gap)/4 - obj_gap;
        botm = botm - D.UI.txtTemplate.Position(4) - obj_gap;
        D.UI.txtBaselineWeight(1) = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtBaselineWeight(1).Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtBaselineWeight(1), ...
            'String', 'Base', ...
            'Visible', 'on');
        D.UI.txtBaselineWeight(2) = copyobj(D.UI.txtBaselineWeight(1), D.UI.tabTBL);
        D.UI.txtBaselineWeight(2).Position(2) = ...
            D.UI.txtBaselineWeight(2).Position(2)-D.UI.txtBaselineWeight(2).Position(4);
        D.UI.txtBaselineWeight(2).String = 'xxxg';
        
        % Corrected weight text
        lft = lft + wd + obj_gap;
        D.UI.txtCorrectedWeight(1) = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtCorrectedWeight(1).Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtCorrectedWeight(1), ...
            'String', 'New', ...
            'Visible', 'on');
        D.UI.txtCorrectedWeight(2) = copyobj(D.UI.txtCorrectedWeight(1), D.UI.tabTBL);
        D.UI.txtCorrectedWeight(2).Position(2) = ...
            D.UI.txtCorrectedWeight(2).Position(2)-D.UI.txtCorrectedWeight(2).Position(4);
        D.UI.txtCorrectedWeight(2).String = 'xxxg';
        
        % Last weight text
        lft = lft + wd + obj_gap;
        D.UI.txtLastWeight(1) = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtLastWeight(1).Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtLastWeight(1), ...
            'String', 'Last', ...
            'Visible', 'on');
        D.UI.txtLastWeight(2) = copyobj(D.UI.txtLastWeight(1), D.UI.tabTBL);
        D.UI.txtLastWeight(2).Position(2) = ...
            D.UI.txtLastWeight(2).Position(2)-D.UI.txtLastWeight(2).Position(4);
        D.UI.txtLastWeight(2).String = 'xxxg';
        
        % Percentage weight text
        lft = lft + wd + obj_gap;
        D.UI.txtPercentageWeight(1) = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtPercentageWeight(1).Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtPercentageWeight(1), ...
            'String', '%Base', ...
            'Visible', 'on');
        D.UI.txtPercentageWeight(2) = copyobj(D.UI.txtPercentageWeight(1), D.UI.tabTBL);
        D.UI.txtPercentageWeight(2).Position(2) = ...
            D.UI.txtPercentageWeight(2).Position(2)-D.UI.txtPercentageWeight(2).Position(4);
        D.UI.txtPercentageWeight(2).String = 'XX%';
        
        %% CREATE FOOD PANEL OBJECTS
        Console_Write('[Table_Setup] RUNNING: Create Food Panel Objects');
        
        % Food Panel
        D.UI.panFeed = copyobj(D.UI.panTemplate, D.UI.tabTBL);
        Safe_Set(D.UI.panFeed, ...
            'Title','Feeding', ...
            'Visible', 'on', ...
            'Position', D.UI.food_pan_pos)
        
        % Start and default position info
        botm = sum(D.UI.food_pan_pos([2,4])) - D.UI.fontSzTxtMed(2)*3;
        lft = D.UI.food_pan_pos(1) + obj_gap;
        wd =  (D.UI.food_pan_pos(3) - obj_gap*2)/4 - obj_gap;
        
        % Pellets text
        D.UI.txtFeedPellets(1) = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtFeedPellets(1).Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtFeedPellets(1), ...
            'FontSize', 7, ...
            'String', 'Pellets', ...
            'Visible', 'on');
        
        % Pellets edit
        D.UI.editFeedPellets = copyobj(D.UI.editTemplate, D.UI.tabTBL);
        D.UI.editFeedPellets.Position(1:3) = [lft, botm - D.UI.editTemplate.Position(4)*0.8, wd];
        Safe_Set(D.UI.editFeedPellets, ...
            'Enable','on',...
            'String', '0', ...
            'Visible', 'on');
        
        % Mash text
        lft = lft + wd + obj_gap;
        D.UI.txtFeedMash(1) = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtFeedMash(1).Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtFeedMash(1), ...
            'FontSize', 7, ...
            'String', 'Mash(TB)', ...
            'Visible', 'on');
        
        % Mash edit
        D.UI.editFeedMash = copyobj(D.UI.editTemplate, D.UI.tabTBL);
        D.UI.editFeedMash.Position(1:3) = [lft, botm - D.UI.editTemplate.Position(4)*0.8, wd];
        Safe_Set(D.UI.editFeedMash, ...
            'Enable','on',...
            'String', '0', ...
            'Visible', 'on');
        
        % Ensure text
        lft = lft + wd + obj_gap;
        D.UI.txtFeedEnsure(1) = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtFeedEnsure(1).Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtFeedEnsure(1), ...
            'FontSize', 7, ...
            'String', 'Ensure(ml)', ...
            'Visible', 'on');
        
        % Ensure edit
        D.UI.editFeedEnsure = copyobj(D.UI.editTemplate, D.UI.tabTBL);
        D.UI.editFeedEnsure.Position(1:3) = [lft, botm - D.UI.editTemplate.Position(4)*0.8, wd];
        Safe_Set(D.UI.editFeedEnsure, ...
            'Enable','on',...
            'String', '0', ...
            'Visible', 'on');
        
        % STAT text
        lft = lft + wd + obj_gap;
        D.UI.txtFeedSTAT(1) = copyobj(D.UI.txtTemplate, D.UI.tabTBL);
        D.UI.txtFeedSTAT(1).Position(1:3) = [lft, botm, wd];
        Safe_Set(D.UI.txtFeedSTAT(1), ...
            'FontSize', 7, ...
            'String', 'STAT(ml)', ...
            'Visible', 'on');
        
        % STAT edit
        D.UI.editFeedSTAT = copyobj(D.UI.editTemplate, D.UI.tabTBL);
        D.UI.editFeedSTAT.Position(1:3) = [lft, botm - D.UI.editTemplate.Position(4)*0.8, wd];
        Safe_Set(D.UI.editFeedSTAT, ...
            'Enable','on',...
            'String', '0', ...
            'Visible', 'on');
        
        % Save feed button
        botm = D.UI.food_pan_pos(2) + 0.01;
        D.UI.toggUpdateFed = copyobj(D.UI.toggUpdateWeight, D.UI.tabTBL);
        D.UI.toggUpdateFed.Position(2) = botm;
        Safe_Set(D.UI.toggUpdateFed, ...
            'String', 'Update', ...
            'Callback', {@Togg_UpdateFed}, ...
            'Visible', 'on');
        
        % Enable button
        Button_State(D.UI.toggUpdateFed, 'Enable');
        
        %% CREATE TAB TABLE OBJECTS
        Console_Write('[Table_Setup] RUNNING: Create Tab Table Objects');
        
        % Add SS_IO_1 tab
        D.UI.tbleSSIO1tab = uitab(D.UI.tblTabSubGrp, ...
            'Title', 'All', ...
            'BackgroundColor', [1, 1, 1]);
        
        % Format SS_IO_1 table
        D.UI.tblSSIO1 = FormatTable(D.SS_IO_1, D.UI.tbleSSIO1tab);
        
        % Add SS_IO_3 tab
        D.UI.tbleSSIO3tab = uitab(D.UI.tblTabSubGrp, ...
            'Title', 'Health', ...
            'BackgroundColor', [1, 1, 1]);
        
        % Format SS_IO_3 table
        D.UI.tblSSIO3 = FormatTable(D.SS_IO_3.(D.PAR.ratLab), D.UI.tbleSSIO3tab);
        
        % Add TT_IO tab and table
        if D.F.implant_session
            
            % Add TT_IO tab
            D.UI.tbleTTIOtab = uitab(D.UI.tblTabSubGrp, ...
                'Title', 'TT', ...
                'BackgroundColor', [1, 1, 1]);
            
            % Format TT_IO table
            D.UI.tblTTIO = FormatTable(D.TT_IO_2.(D.PAR.ratLab), D.UI.tbleTTIOtab);
            
        end
        
        % Get list of included rats
        rat_list = D.SS_IO_1.Properties.RowNames(D.SS_IO_1.Include_Run);
        
        % Put current rat at start of list
        rat_list = [D.PAR.ratLab; rat_list(~ismember(rat_list, D.PAR.ratLab))];
        
        % Make a tab for current rat only unless unless doing 'Table_Update'
        if D.PAR.sesType ~= 'Table_Update'
            rat_list = rat_list(1);
        end
        
        % Initialize tab objects
        D.UI.tbleSSIO2tab = gobjects(length(rat_list),1);
        
        % Add SS_IO_2 tab for each rat
        for z_r = 1:length(rat_list)
            
            % Add SS_IO_2 tab
            D.UI.tbleSSIO2tab(z_r) = uitab(D.UI.tblTabSubGrp, ...
                'Title', rat_list{z_r}(2:end), ...
                'BackgroundColor', [1, 1, 1]);
            
            % Format SS_IO_2 tables
            D.UI.tblSSIO2 = FormatTable(D.SS_IO_2.(rat_list{z_r}), D.UI.tbleSSIO2tab(z_r));
            
        end
        
        % Add new row to top of SS_IO_3 table
        if ~isempty(D.UI.tblSSIO3.RowName)
            D.UI.tblSSIO3.Data = [D.UI.tblSSIO3.Data(1,:); D.UI.tblSSIO3.Data];
        end
        D.UI.tblSSIO3.RowName = [{datestr(TIMSTRLOCAL, 'yyyy-mm-dd')}; D.UI.tblSSIO3.RowName];
        
        % Clear non health feilds
        var_ind = ~contains(D.UI.tblSSIO3.ColumnName,'Health');
        D.UI.tblSSIO3.Data(1, var_ind) = {[]};
        
        % Set tab to health
        D.UI.tblTabSubGrp.SelectedTab = D.UI.tbleSSIO3tab;
        
        %% CREATE SESSION NOTES OBJECTS
        Console_Write('[Table_Setup] RUNNING: Create Session Notes Objects');
        
        % Create notes panel
        D.UI.panGenNotes = copyobj(D.UI.panTemplate, D.UI.tabTBL);
        Safe_Set(D.UI.panGenNotes, ...
            'Title','General Notes', ...
            'Visible', 'on', ...
            'Position', D.UI.gen_notes_pan_pos)
        
        % Start and default position info
        ht_edit = 0.4;
        ht_tbl = 1 - ht_edit;
        botm_tbl = 0;
        botm_edit = ht_tbl;
        lft = 0;
        wd =  1;
        
        % Old task notes table
        D.UI.tblGenNotes = copyobj(D.UI.tblSSIO3,D.UI.panGenNotes);
        Safe_Set(D.UI.tblGenNotes, ...
            'ColumnFormat', {'char'}, ...
            'Position', [lft, botm_tbl, wd, ht_tbl], ...
            'Visible', 'on')
        
        % New task notes edit
        D.UI.editGenNotes = copyobj(D.UI.editTemplate, D.UI.panGenNotes);
        Safe_Set(D.UI.editGenNotes, ...
            'Position', [lft, botm_edit, wd, ht_edit], ...
            'Units', 'Normalized', ...
            'FontSize', 10, ...
            'Max', 100, ...
            'Enable','on',...
            'Visible', 'on');
        
        % Get notes and exclude empty entries
        notes_ind = ismember(D.UI.tblSSIO3.ColumnName, 'Notes');
        notes = D.UI.tblSSIO3.Data(:,notes_ind);
        notes = notes(cell2mat(cellfun(@(x) ~isempty(x), notes, 'uni', false)));
        notes = notes(~ismember(notes, 'NA'));
        
        % Keep only notes and dates entry
        D.UI.tblGenNotes.Data = notes;
        D.UI.tblGenNotes.ColumnName = {[]};
        D.UI.tblGenNotes.ColumnWidth = D.UI.tblSSIO3.ColumnWidth(notes_ind);
        
        % Remove notes from main tables
        D.UI.tblSSIO3.ColumnName(notes_ind) = [];
        D.UI.tblSSIO3.Data(:,notes_ind) = [];
        notes_ind = ismember(D.UI.tblSSIO2.ColumnName, 'Notes');
        D.UI.tblSSIO2(1).Data(:,notes_ind) = [];
        D.UI.tblSSIO2(1).ColumnName(notes_ind) = [];
        
        %% CREATE TT NOTES OBJECTS
        Console_Write('[Table_Setup] RUNNING: Create TT Notes Objects');
        
        % TT Notes Panel
        D.UI.panTTNotes = copyobj(D.UI.panTemplate, D.UI.tabTBL);
        Safe_Set(D.UI.panTTNotes, ...
            'Title','TT Notes', ...
            'Visible', 'on', ...
            'Position', D.UI.tt_notes_pan_pos)
        
        % Set panels disabled
        if ~D.F.implant_session
            
            % Disable tt notes panel
            Panel_State(D.UI.panTTNotes, 'Disable');
            
        else
            
            % Old tt notes table
            D.UI.tblTTNotes = copyobj(D.UI.tblGenNotes, D.UI.panTTNotes);
            D.UI.tblTTNotes.Data = D.UI.tblTTIO.Data;
            D.UI.tblTTNotes.ColumnName = D.UI.tblTTIO.ColumnName;
            D.UI.tblTTNotes.RowName = D.UI.tblTTIO.RowName;
            
            % New tt notes edit
            D.UI.editTTNotes = copyobj(D.UI.editGenNotes, D.UI.panTTNotes);
            
            % Get notes and exclude empty entries
            notes_ind = ismember(D.UI.tblTTIO.ColumnName, 'Notes');
            notes = D.UI.tblTTIO.Data(:,notes_ind);
            notes = notes(cell2mat(cellfun(@(x) ~isempty(x), notes, 'uni', false)));
            notes = notes(~ismember(notes, 'NA'));
            
            % Keep only notes entry
            D.UI.tblTTNotes.Data = notes;
            D.UI.tblTTNotes.ColumnName = {[]};
            D.UI.tblTTNotes.ColumnWidth = D.UI.tblTTIO.ColumnWidth(notes_ind);
            
            % Remove notes from main table
            D.UI.tblTTIO.Data(:,notes_ind) = [];
            D.UI.tblTTIO.ColumnName(notes_ind) = [];
            
        end
        
        %% FORMAT TABLE FUNCTION
        function [ui_table] = FormatTable(io_table, parent)
            
            % Convert table data to cell
            c_io = table2cell(io_table);
            
            % Remove nexted cell or 2d data
            cell_ind = cell2mat(cellfun(@(x) isa(x, 'cell'), ...
                c_io(1,:), 'uni', false));
            nd_ind = any(cell2mat(cellfun(@(x) max(size(x))>1 && ~isa(x, 'char'), ...
                c_io, 'uni', false)), 1);
            
            % Remove values
            exc_ind = cell_ind | nd_ind;
            c_io = c_io(:, ~exc_ind);
            
            % Replace [] with 'NA'
            empty_ind = cell2mat(cellfun(@(x) isempty(x), ...
                c_io, 'uni', false));
            c_io(empty_ind) = {'NA'};
            
            % Get variable names
            var_list = io_table.Properties.VariableNames(~exc_ind);
            
            % Handle data info
            if any(ismember(io_table.Properties.VariableNames , 'Date'))
                
                % Add date for first day
                if isempty(io_table.Date{1})
                    io_table.Date{1} = datestr(TIMSTRLOCAL, 'yyyy-mm-dd_HH-MM-SS');
                end
                
                % Get dates for row names
                date_list = io_table.Date;
                date_list = regexp(date_list, '\S*(?=_)', 'match');
                date_list = [date_list{:}];
                row_names = date_list;
                
                % Remove dates entry
                date_ind = ismember(io_table.Properties.VariableNames , 'Date');
                c_io(:,date_ind) = [];
                var_list(date_ind) = [];
                
            else
                % Set row names to rat name
                row_names = io_table.Properties.RowNames;
            end
            
            % Get numeric data inds
            num_ind = any(cell2mat(cellfun(@(x) isa(x, 'double'), ...
                c_io, 'uni', false)),1);
            
            % Get integer data inds
            int_col = find(num_ind);
            int_col = int_col(all(cell2mat(cellfun(@(x) mod(x, 1) == 0, ...
                c_io(:,num_ind), 'uni', false)),1));
            int_ind = false(1, length(num_ind));
            int_ind(int_col) = true;
            
            % Get char data inds
            char_ind = any(cell2mat(cellfun(@(x) isa(x, 'char'), ...
                c_io, 'uni', false)),1);
            
            % Get logical data inds
            bool_ind = any(cell2mat(cellfun(@(x) isa(x, 'logical'), ...
                c_io, 'uni', false)),1);
            
            % Get categorical data inds to convert to pop menu
            cat_ind = cell2mat(cellfun(@(x) isa(x, 'categorical'), ...
                c_io(1,:), 'uni', false));
            
            % ADD DATA TO TABLE
            
            % Copy table
            ui_table = copyobj(D.UI.tblTemplate,D.UI.tabTBL);
            set(ui_table, ...
                'Parent', parent, ...
                'Visible', 'on');
            
            % Add data to table
            ui_table.Data = cell(size(c_io));
            ui_table.Data(:,~cat_ind) = c_io(:,~cat_ind);
            ui_table.ColumnName = var_list;
            ui_table.RowName = row_names;
            
            % Set column formats
            ui_table.ColumnFormat = cell(1,size(c_io,2));
            ui_table.ColumnFormat(num_ind & int_ind) = {'numeric'};
            ui_table.ColumnFormat(num_ind & ~int_ind) = {'bank'};
            ui_table.ColumnFormat(char_ind) = {'char'};
            ui_table.ColumnFormat(bool_ind) = {'logical'};
            
            % Set column/row default width
            ui_table.ColumnWidth = repmat({'auto'}, 1, size(c_io,2));
            
            % Convert cat data to popmenu
            cat_col = find(cat_ind);
            for z_c = 1:length(cat_col)
                
                % Format for popmenu
                ui_table.ColumnFormat(cat_col(z_c)) = ...
                    {categories(c_io{1,cat_col(z_c)})'};
                
                % Add data
                cell_str = cellfun(@(x) char(x), c_io(:,cat_col(z_c)), 'uni', false);
                ui_table.Data(:,cat_col(z_c)) = cell_str;
                
                % Resize column
                wdth = max([10*length(ui_table.ColumnName{cat_col(z_c)}); ...
                    10*cell2mat(cellfun(@(x) max(size(x)), ...
                    cell_str, 'uni', false))]);
                ui_table.ColumnWidth(cat_col(z_c)) = {wdth};
            end
            
            % Remove vars with no values assigned
            iif = @(varargin) varargin{2 * find([varargin{1:2:end}], 1, 'first')}();
            exc_na = @(x1, x2, x3, y) iif( ...
                x1, @() isnan(y{:}), ...
                x2,       @() strcmp(y{:}, 'NA'), ...
                x3,       @() strcmp(y{:}, '<undefined>'), ...
                true,     false);
            exc_mat = arrayfun(@(x1, x2, x3, y) exc_na(x1, x2, x3, y), ...
                repmat(num_ind, size(ui_table.Data, 1), 1), ...
                repmat(char_ind, size(ui_table.Data, 1), 1), ...
                repmat(cat_ind, size(ui_table.Data, 1), 1), ...
                ui_table.Data);
            exc_ind = all(exc_mat, 1);
            
            % Keep notes
            exc_ind(ismember(ui_table.ColumnName, 'Notes')) = false;
            
            % Remove values from table
            set(ui_table, 'Data', ui_table.Data(:,~exc_ind));
            set(ui_table, 'ColumnName', ui_table.ColumnName(~exc_ind));
            set(ui_table, 'ColumnFormat', ui_table.ColumnFormat(~exc_ind));
            
            % Flip table so newest entries at top
            ui_table.Data = flip(ui_table.Data, 1);
            ui_table.RowName = flip(ui_table.RowName);
            
            % Resize notes
            ind = ismember(ui_table.ColumnName, 'Notes');
            wdth = max([8*length('Notes'); ...
                6*cell2mat(cellfun(@(x) max(size(x)), ...
                ui_table.Data(:,ind), 'uni', false))]);
            ui_table.ColumnWidth(ind) = {wdth};
            
        end
        
    end

% --------------------------TT TRACKING SETUP----------------------
    function[was_ran] = TT_Track_Setup()
        
        %% BAIL IF NOT IMPLANT SESSION
        
        % Initialize output
        was_ran = false;
        
        % Bail
        if ~D.F.implant_session
            return
        end
        
        % Set flag
        was_ran = true;
        
        %% ===================== IMPORT LOG TABLE DATA =====================
        
        % Get implant coordinates
        D.TT.ttCoords = D.TT_IO_1.Implant_Coordinates(D.PAR.ratIndTT,:);
        
        % Get number of bundles
        D.TT.nBndl = ~any(isnan(D.TT.ttCoords{2}))+1;
        
        % Get tt configs
        D.TT.ttConfig = D.TT_IO_1.Implant_Configuration(D.PAR.ratIndTT,:);
        
        % Get bundle angle
        D.UI.bndAng = D.TT_IO_1.Implant_Angle(D.PAR.ratIndTT,:);
        D.UI.bndAng{1} = deg2rad(270 + D.UI.bndAng{1});
        D.UI.bndAng{2} = deg2rad(270 + D.UI.bndAng{2});
        
        % Get bundle labels
        D.TT.bndlLab = D.TT_IO_1.Bundle_Label{D.PAR.ratIndTT,:};
        
        % Get tt list
        D.TT.ttLab = D.TT_IO_1.TT_Label{D.PAR.ratIndTT,:};
        
        % Get tt map
        D.TT.ttMap = D.TT_IO_1.TT_Mapping(D.PAR.ratIndTT,:);
        
        % Remove unused entries
        for z_b = 1:2
            if D.TT.nBndl==1 && z_b == 2
                D.TT.ttMap{z_b} = D.TT.ttMap{1}(1,1);
            else
                D.TT.ttMap{z_b} = D.TT.ttMap{z_b}(1:D.TT.ttConfig{z_b}(1),1:D.TT.ttConfig{z_b}(2));
            end
        end
        
        % Get references
        D.TT.refMap = ...
            cellfun(@(x,y) [x, ' ', y], ...
            cellstr(char(D.TT_IO_1.Reference_Mapping{D.PAR.ratIndTT}(:,1))), ...
            cellstr(char(D.TT_IO_1.Reference_Mapping{D.PAR.ratIndTT}(:,2))), ...
            'uni', false);
        D.TT.refList = cellstr(char(D.TT_IO_1.Reference_Mapping{D.PAR.ratIndTT}(:,1)));
        
        % Get thread pitch
        D.UI.umPerTurn = D.TT_IO_1.Thread_Pitch(D.PAR.ratIndTT)*1000;
        
        % Store current log
        D.TT.ttLogNew = D.TT_IO_2.(D.PAR.ratLab)(end,:);
        
        % Get session number
        D.TT.Ses = D.TT_IO_2.(D.PAR.ratLab).Session(end)+1;
        
        % Get tt list sorted by bundle
        tt_lab_list = cell(2,1);
        for z_b = 1:D.TT.nBndl
            tt_lab_list{z_b} = cellstr(char(D.TT.ttMap{z_b}(~isundefined(D.TT.ttMap{z_b}))));
            tt_lab_list{z_b} = D.TT.ttLab(ismember(D.TT.ttLab, tt_lab_list{z_b}))';
        end
        
        % Update list
        D.TT.ttLab = [[tt_lab_list{1}], [tt_lab_list{2}]];
        
        % Create label excluding 'TT'
        D.TT.ttNum = regexprep(D.TT.ttLab, 'TT', '');
        
        % Get tt bundle ind
        D.I.ttBndl = NaN(D.PAR.maxTT,1);
        for z_b = 1:D.TT.nBndl
            D.I.ttBndl(ismember(D.TT.ttLab, tt_lab_list{z_b})) =  z_b;
        end
        
        % Clear notes entry
        for z_tt = 1:length(D.TT.ttLab)
            D.TT.ttLogNew.([D.TT.ttLab{z_tt},'_N']){:} = '';
        end
        
        %% ================== IMPORT/FORMAT PAXINOS IMAGES ================
        
        % Get A-P lims
        ap_lim = [ceil(max(D.TT.ttCoords{1}(1), D.TT.ttCoords{2}(1)) + ...
            (max(size(D.TT.ttMap{1},1), size(D.TT.ttMap{2},1))/2) * D.PAR.canSp), ...
            floor(min(D.TT.ttCoords{1}(1), D.TT.ttCoords{2}(1)) - ...
            (max(size(D.TT.ttMap{1},1), size(D.TT.ttMap{2},1))/2) * D.PAR.canSp)];
        
        % Add 1 mm on either side
        ap_lim = [ap_lim(1)+1, ap_lim(2)-1];
        
        % Set plot lims [A-P,M-L,D-V] (mm)
        D.UI.mmPlotLims = [...
            ap_lim(1), ap_lim(2); ...
            0, 8; ...
            0, -10];
        
        % Paxinos image dims [y_mm, x_mm]
        D.PAR.mmPaxSize{1} = [11,21];
        D.PAR.mmPaxSize{2} = [11,8];
        D.PAR.mmPaxSize{3} = [8,21];
        
        % Paxinos image lims [x_min, x_max; y_min, y_max]
        D.PAR.pxlPalLims{1} = [243, 3854; 420, 2311]; % sag
        D.PAR.pxlPalLims{2} = [243, 2136; 247, 2842]; % cor
        D.PAR.pxlPalLims{3} = [243, 3854; 250, 1631]; % hor
        
        % Strange sized pax cor images
        D.PAR.pxlPalLims{4} = [206, 2099; 210, 2812]; % cor dumb
        
        % Image axis limits [A-P,M-L,D-V] (mm)
        D.UI.mmPaxLims = [...
            6, -15; ...
            0, 8; ...
            0, -11];
        
        % Paxinos view colors
        D.UI.paxViewCol = [...
            255, 198, 198; ... % red
            198, 200, 255; ... % blue
            198, 255, 201] / 255; % green
        
        % Get image stuff
        for z_view = 1:3
            
            % Get image list
            D.PAR.paxFiLabs{z_view} = dir(D.DIR.paxImg{z_view});
            D.PAR.paxFiLabs{z_view} = {D.PAR.paxFiLabs{z_view}(:).name};
            D.PAR.paxFiLabs{z_view} = ...
                D.PAR.paxFiLabs{z_view}(cell2mat(cellfun(@(x) ~isempty(x), strfind(D.PAR.paxFiLabs{z_view}, 'img'), 'uni', false)));
            
            % Setup image matrix [m, n, 3]
            D.UI.paxMat{z_view} = cell(1, length(D.PAR.paxFiLabs{z_view}));
            
            % Setup image handles
            D.UI.h_paxImg{z_view} = gobjects(1,length(D.PAR.paxFiLabs{z_view}));
            
            % Setup image coordinates
            D.TT.imgCoor{z_view} = NaN(1,length(D.PAR.paxFiLabs{z_view}));
        end
        
        % Check if we already have processed image data
        if exist(D.DIR.paxDat, 'file')
            load(D.DIR.paxDat);
            D.UI.paxMat = IMG_MAT; %#ok<NODEF>
            D.TT.imgCoor = IMG_COORD; %#ok<NODEF>
            
        else
            % Loop through paxinos directories
            for z_view = 1:3
                
                % Loop through each image
                for z_pax = 1:length(D.PAR.paxFiLabs{z_view})
                    
                    % Store
                    D.UI.paxMat{z_view}{z_pax} = ...
                        imread(fullfile(D.DIR.paxImg{z_view}, D.PAR.paxFiLabs{z_view}{z_pax}));
                    
                    % Handle stupid fucking pax bullshit
                    if z_view == 2 && size(D.UI.paxMat{z_view}{z_pax},1) == 3300
                        
                        inc_x = D.PAR.pxlPalLims{4}(2,1):D.PAR.pxlPalLims{4}(2,2);
                        inc_y = D.PAR.pxlPalLims{4}(1,1):D.PAR.pxlPalLims{4}(1,2);
                        
                    else
                        inc_x = D.PAR.pxlPalLims{z_view}(2,1):D.PAR.pxlPalLims{z_view}(2,2);
                        inc_y = D.PAR.pxlPalLims{z_view}(1,1):D.PAR.pxlPalLims{z_view}(1,2);
                    end
                    
                    % Clip image
                    D.UI.paxMat{z_view}{z_pax} = D.UI.paxMat{z_view}{z_pax}(inc_x, inc_y, :);
                    
                    % Get colored pixels
                    color_ind = mean(diff(D.UI.paxMat{z_view}{z_pax},1,3),3) > 50;
                    mono_ind = mean(D.UI.paxMat{z_view}{z_pax},3) < 250 & ~color_ind;
                    
                    % Create ind for each rgb colored pixels
                    red_pxls = padarray(color_ind, [0,0,2], 'post');
                    green_pxls = padarray(color_ind, [0,0,1], 'both');
                    blue_pxls = padarray(color_ind, [0,0,2], 'pre');
                    
                    % Set monochrome cols to gray
                    D.UI.paxMat{z_view}{z_pax}(repmat(mono_ind,[1,1,3])) = 127;
                    
                    % Set outline color
                    D.UI.paxMat{z_view}{z_pax}(red_pxls) = D.UI.paxViewCol(z_view, 1)*255;
                    D.UI.paxMat{z_view}{z_pax}(green_pxls) = D.UI.paxViewCol(z_view, 2)*255;
                    D.UI.paxMat{z_view}{z_pax}(blue_pxls) = D.UI.paxViewCol(z_view, 3)*255;
                    
                    % Reseize image to 1pxl per 10 um
                    D.UI.paxMat{z_view}{z_pax} = ...
                        imresize(D.UI.paxMat{z_view}{z_pax}, D.PAR.mmPaxSize{z_view}*100);
                    
                    % Get coordinate info
                    num_str = regexp(D.PAR.paxFiLabs{z_view}{z_pax}, 'lab_(-?\d?\d.\d\d).png', 'tokens');
                    D.TT.imgCoor{z_view}(z_pax) = str2double(num_str{:});
                    
                    % Flip images
                    D.UI.paxMat{z_view}{z_pax} = flip(D.UI.paxMat{z_view}{z_pax},1);
                    D.UI.paxMat{z_view}{z_pax} = flip(D.UI.paxMat{z_view}{z_pax},2);
                    
                end
                
                % Sort
                [D.TT.imgCoor{z_view}, sort_ind] = sort(D.TT.imgCoor{z_view});
                D.UI.paxMat{z_view} = D.UI.paxMat{z_view}(sort_ind);
            end
            
            % Save coordinate and image matrix
            IMG_MAT = D.UI.paxMat; %#ok<NASGU>
            IMG_COORD = D.TT.imgCoor; %#ok<NASGU>
            save(D.DIR.paxDat,  'IMG_MAT', 'IMG_COORD');
            
        end
        
        %% ======================== SETUP TT TAB  =========================
        
        % Add TT TRACK tab
        D.UI.tabTT = uitab(D.UI.tabgp, ...
            'Title', 'TT TRACK', ...
            'BackgroundColor', [1, 1, 1]);
        
        % Figure position
        D.UI.tabTTNormPos = get(D.UI.tabTT, 'Position');
        Safe_Set(D.UI.tabTT,'Units','Pixels');
        D.UI.wdScale = (D.UI.tabTT.Position(4)/D.UI.tabTT.Position(3));
        Safe_Set(D.UI.tabTT,'Units','Normalized');
        
        % Border offset for all GUI features
        D.UI.bordOffHt = 0.02;
        D.UI.bordOffWd = D.UI.bordOffHt*D.UI.wdScale;
        
        % Main axis reserved space
        ax_lfg_lim = D.UI.main_ax_bounds(1)+0.005;
        
        % Main axis actual pos
        ax_lft = ax_lfg_lim + 2*D.UI.bordOffWd;
        ax_wd = D.UI.tabTTNormPos(3) - ax_lft;
        ax_ht = D.UI.tabTTNormPos(4) - 2*D.UI.bordOffHt;
        ax_btm = 2*D.UI.bordOffHt;
        D.UI.axe3dPos =  ...
            [ax_lft, ax_btm, ax_wd, ax_ht];
        
        % Label axes lims
        lm_str = {...
            '|P|', '|A|'; ...
            '|M|', '|L|'; ...
            '|D|', '|V|'};
        lm = NaN(3,2);
        tk = cell(3,1);
        tl = cell(3,1);
        
        % Get axis stuff
        for z_view = 1:3
            
            % Get axis limits
            lm(z_view,:) = [min(D.UI.mmPaxLims(z_view,:)), max(D.UI.mmPaxLims(z_view,:))]*100;
            
            % Get axis ticks
            tk{z_view} = linspace(lm(z_view,1),lm(z_view,2),sum(abs(D.UI.mmPaxLims(z_view,:)))+1)';
            
            % Make axis tick labels
            tl{z_view} = num2str(tk{z_view}/100);
            tl{z_view}  = [repmat(blanks(3-size(tl{z_view} ,2)), size(tl{z_view} ,1), 1), tl{z_view}];
            
            % Label axes lims
            tl{z_view}(tk{z_view}/100==min(D.UI.mmPlotLims(z_view,:)),:) = lm_str{z_view,1};
            tl{z_view}(tk{z_view}/100==max(D.UI.mmPlotLims(z_view,:)),:) = lm_str{z_view,2};
        end
        
        % Create axis
        D.UI.axe3dH = axes(...
            'Parent', D.UI.tabTT,...
            'Units','Normalized',...
            'Position',D.UI.axe3dPos,...
            'XDir', 'Reverse', ...
            'XLim', lm(1,:), ...
            'YLim', lm(2,:), ...
            'ZLim', lm(3,:), ...
            'XTick', tk{1}, ...
            'YTick', tk{2}, ...
            'ZTick', tk{3}, ...
            'XTickLabel', tl{1}, ...
            'YTickLabel', tl{2}, ...
            'ZTickLabel', tl{3}, ...
            'Color', 'None', ...
            'Visible', 'on');
        hold on
        box on
        
        % Specify rotation mat
        img_rot = [ ...
            deg2rad(90), 0, 0; ...
            deg2rad(90), deg2rad(90), 0;
            0, 0, 0
            ];
        
        % Specify direction of translation
        img_trans_x = [D.UI.mmPaxLims(1,2), 0, D.UI.mmPaxLims(1,2)] * 100;
        img_trans_y = [D.UI.mmPaxLims(3,2), D.UI.mmPaxLims(3,2), 0] * 100;
        img_trans_z = [-1, 1, 1];
        
        % Show each image on correct plane
        for z_view = 1:3
            for z_img = 1:length(D.PAR.paxFiLabs{z_view})
                
                % Get transform
                zt = D.TT.imgCoor{z_view}(z_img) * 100 * img_trans_z(z_view);
                p = hgtransform('Parent',D.UI.axe3dH);
                rt = makehgtform(...
                    'xrotate', img_rot(z_view,1), ...
                    'yrotate', img_rot(z_view,2), ...
                    'zrotate', img_rot(z_view,3), ...
                    'translate',[img_trans_x(z_view), img_trans_y(z_view), zt]);
                h = hgtransform('Matrix',rt);
                set(h,'Parent',p)
                
                % Show image
                D.UI.h_paxImg{z_view}(z_img) =  ...
                    image(D.UI.paxMat{z_view}{z_img}, ...
                    'Parent', h, ...
                    'Visible', 'off');
                
                % Set alpha
                D.UI.h_paxImg{z_view}(z_img).AlphaData = 0.75;
            end
        end
        
        % Set final plot lims
        axis equal
        Safe_Set(D.UI.axe3dH, ...
            'XLim',  100*[min(D.UI.mmPlotLims(1,:)), max(D.UI.mmPlotLims(1,:))], ...
            'YLim',  100*[min(D.UI.mmPlotLims(2,:)), max(D.UI.mmPlotLims(2,:))], ...
            'ZLim',  100*[min(D.UI.mmPlotLims(3,:)), max(D.UI.mmPlotLims(3,:))])
        
        % Remove unused images
        exc_sag = D.TT.imgCoor{1}<min(D.UI.mmPlotLims(2,:)) | ...
            D.TT.imgCoor{1}>max(D.UI.mmPlotLims(2,:));
        exc_cor = D.TT.imgCoor{2}<min(D.UI.mmPlotLims(1,:)) | ...
            D.TT.imgCoor{2}>max(D.UI.mmPlotLims(1,:));
        exc_hor = D.TT.imgCoor{3}<min(D.UI.mmPlotLims(3,:)) | ...
            D.TT.imgCoor{3}>max(D.UI.mmPlotLims(3,:));
        D.TT.imgCoor{1}(exc_sag) = [];
        D.TT.imgCoor{2}(exc_cor) = [];
        D.TT.imgCoor{3}(exc_hor) = [];
        D.UI.h_paxImg{1}(exc_sag) = [];
        D.UI.h_paxImg{2}(exc_cor) = [];
        D.UI.h_paxImg{3}(exc_hor) = [];
        
        light_pos = [max(D.UI.axe3dH.XLim), max(D.UI.axe3dH.YLim), max(D.UI.axe3dH.ZLim)];
        D.UI.h_light(1) = light(D.UI.axe3dH, ...
            'Style', 'local', ...
            'Position', light_pos);
        light_pos = [min(D.UI.axe3dH.XLim), min(D.UI.axe3dH.YLim), min(D.UI.axe3dH.ZLim)];
        D.UI.h_light(2) = light(D.UI.axe3dH, ...
            'Style', 'local', ...
            'Position', light_pos);
        
        % Legend pos
        ht = 0.125; % height of axis
        wd = ht*D.UI.wdScale;
        lft = sum(D.UI.axe3dPos([1,3])) - wd*3 - 4.5*D.UI.bordOffWd;
        btm = D.UI.axe3dPos(2)+D.UI.bordOffHt;
        D.UI.leg_pos{1} = [lft , btm, wd, ht];
        D.UI.leg_pos{2} = D.UI.leg_pos{1};
        D.UI.leg_pos{2}(1) = sum(D.UI.leg_pos{1}([1,3])) + D.UI.bordOffWd;
        D.UI.leg_pos{2} = D.UI.leg_pos{1};
        D.UI.leg_pos{2}(1) = sum(D.UI.leg_pos{1}([1,3])) + D.UI.bordOffWd;
        D.UI.leg_pos{3} = D.UI.leg_pos{1};
        D.UI.leg_pos{3}(1) = sum(D.UI.leg_pos{2}([1,3])) + D.UI.bordOffWd;
        
        % Create Bndle 1 legend axis
        D.UI.axBndlLeg(1) = axes(...
            'Parent', D.UI.tabTT,...
            'Units', 'Normalized',...
            'Position', D.UI.leg_pos{1},...
            'Color', [1 1 1],...
            'XTick', [], ...
            'YTick', [], ...
            'Visible', 'on');
        box on
        hold on
        
        % Create Bundle 2 legend axis
        D.UI.axBndlLeg(2) = copyobj(D.UI.axBndlLeg(1), D.UI.tabTT);
        box on
        hold on
        
        % Create drive map legend axis
        D.UI.axBndlLeg(3) = copyobj(D.UI.axBndlLeg(1), D.UI.tabTT);
        box on
        hold on
        
        % Set positions
        set(D.UI.axBndlLeg(2), 'Position', D.UI.leg_pos{2})
        set(D.UI.axBndlLeg(3), 'Position', D.UI.leg_pos{3})
        
        % Create a backround axis
        lft = D.UI.leg_pos{1}(1)-D.UI.bordOffWd;
        btm = D.UI.axe3dPos(2);
        wd = wd*3 + D.UI.bordOffWd*4;
        ht = ht + D.UI.bordOffHt*3;
        D.UI.axBndlLegBack = copyobj(D.UI.axBndlLeg(1),D.UI.tabTT);
        D.UI.axBndlLegBack.Position = [lft , btm, wd, ht];
        
        % Orientation strings
        D.PAR.orCardList = [...
            {'N'}, {'NNW'},{'NW'},{'WNW'}, ...
            {'W'},{'WSW'},{'SW'},{'SSW'}, ...
            {'S'},{'SSE'},{'SE'},{'ESE'}, ....
            {'E'},{'ENE'},{'NE'},{'NNE'}];
        D.PAR.orFracList = [...
            {'0/16'},{'1/16'},{'2/16'},{'3/16'}, ...
            {'4/16'},{'5/16'},{'6/16'},{'7/16'}, ...
            {'8/16'},{'9/16'},{'10/16'},{'11/16'}, ...
            {'12/16'},{'13/16'},{'14/16'},{'15/16'}];
        
        % Move legend to top
        uistack(D.UI.axBndlLeg,'top')
        
        %% ==================== SETUP TT TAB OBJECTS ======================
        
        % ------------------------ PAXINOS VIEW CONTROLS --------------------------
        
        % Add Yaw (z axis) slider along bottom
        wd = D.UI.axe3dPos(3) - D.UI.bordOffWd;
        ht = D.UI.bordOffHt;
        lft = ax_lfg_lim + 2*D.UI.bordOffWd;
        btm = 0;
        pos = [lft, btm, wd, ht];
        D.UI.sldYaw = uicontrol('Style', 'slider',...
            'Parent', D.UI.tabTT, ...
            'Units', 'Normalized', ...
            'Callback', {@Sld_Set3dView},...
            'UserData', 0, ...
            'Min',-90,'Max',90, ...
            'Value',0,...
            'SliderStep', [1/180,45/180], ...
            'Visible', 'on',...
            'Enable', 'on',...
            'Position',  pos);
        
        % Add Pitch (x axis) slider along left
        wd = D.UI.bordOffWd;
        ht = D.UI.axe3dPos(4) - D.UI.bordOffHt;
        btm = D.UI.bordOffHt*2;
        lft = ax_lfg_lim;
        pos = [lft, btm, wd, ht];
        D.UI.sldPitch = uicontrol('Style', 'slider',...
            'Parent', D.UI.tabTT, ...
            'Units', 'Normalized', ...
            'Callback', {@Sld_Set3dView},...
            'UserData', 0, ...
            'Min',-90,'Max',90, ...
            'Value',0,...
            'SliderStep', [1/180,45/180], ...
            'Visible', 'on',...
            'Enable', 'on',...
            'Position',  pos);
        
        % Add rotate option
        D.UI.mouseRotView = rotate3d(D.UI.axe3dH);
        set(D.UI.mouseRotView, ...
            'ActionPostCallback', {@Sld_Set3dView},...
            'Enable', 'off')
        
        % Add setview button
        D.UI.btnSetView(1) = uicontrol('style','togglebutton', ...
            'Parent', D.UI.tabTT, ...
            'Enable', 'On', ...
            'UserData', 1, ...
            'Units', 'Normalized', ...
            'FontName',D.UI.btnFont(1),...
            'ForegroundColor', [0.1,0.1,0.1], ...
            'FontWeight','Bold',...
            'FontSize',D.UI.fontSzBtnLrg(1));
        
        % Create additional set view buttons
        D.UI.btnSetView(2) = copyobj(D.UI.btnSetView(1),D.UI.tabTT);
        D.UI.btnSetView(3) = copyobj(D.UI.btnSetView(1),D.UI.tabTT);
        D.UI.btnSetView(4) = copyobj(D.UI.btnSetView(1),D.UI.tabTT);
        
        % Change string and user data
        Safe_Set(D.UI.btnSetView(1), 'String', 'S', 'UserData', 1, 'BackgroundColor', D.UI.paxViewCol(1,:));
        Safe_Set(D.UI.btnSetView(2), 'String', 'C', 'UserData', 2, 'BackgroundColor', D.UI.paxViewCol(2,:));
        Safe_Set(D.UI.btnSetView(3), 'String', 'H', 'UserData', 3, 'BackgroundColor', D.UI.paxViewCol(3,:));
        Safe_Set(D.UI.btnSetView(4), 'String', 'O', 'UserData', 4, 'BackgroundColor', [0.8,0.8,0.8]);
        
        % Change pos
        ht = D.UI.bordOffHt;
        wd = D.UI.bordOffWd;
        lft = ax_lfg_lim;
        btm = 0;
        pos = [lft, btm, wd, ht];
        Safe_Set(D.UI.btnSetView, 'Position', pos);
        D.UI.btnSetView(1).Position(1) = pos(1)+pos(3);
        D.UI.btnSetView(3).Position(2) = pos(2)+pos(4);
        D.UI.btnSetView(4).Position(1:2) = [D.UI.btnSetView(1).Position(1), D.UI.btnSetView(3).Position(2)];
        
        % Set callback
        Safe_Set(D.UI.btnSetView, 'Callback', {@Sld_Set3dView});
        
        % Set view
        Sld_Set3dView(D.UI.btnSetView(4));
        
        % ----------------------- PAXINOS IMAGE CONTROLS --------------------------
        
        % Switch image button
        swtch_txt_wdth = 0.0625;
        swtch_sld_wdth = 0.16;
        swtch_ht = 0.025;
        swtch_lft = ax_lfg_lim + D.UI.bordOffHt*2;
        swtch_btm = D.UI.axe3dPos(2)+D.UI.bordOffHt/2 + 0.025*2 + swtch_ht;
        
        % Switch image text
        pos = [swtch_lft , swtch_btm, swtch_txt_wdth, swtch_ht];
        D.UI.txtImgCorr(1) = uicontrol('Style','text', ...
            'Parent', D.UI.tabTT, ...
            'String', ' ', ...
            'Units', 'Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Left',...
            'ForegroundColor', [0.1,0.1,0.1], ...
            'FontName', D.UI.txtFont,...
            'FontWeight', 'Bold',...
            'FontSize', D.UI.fontSzTxtHuge(1));
        
        % Switch image slider
        pos = [pos(1)+pos(3), pos(2), swtch_sld_wdth, swtch_ht];
        D.UI.sldSwtchImg(1) = uicontrol('Style', 'slider',...
            'Parent', D.UI.tabTT, ...
            'Units', 'Normalized', ...
            'Callback', {@Sld_Set3dView},...
            'Min', 1, ...
            'Visible', 'on',...
            'Enable', 'on',...
            'Position',  pos);
        
        % Create slider and text copies
        D.UI.txtImgCorr(2) = copyobj(D.UI.txtImgCorr(1),D.UI.tabTT);
        D.UI.txtImgCorr(3) = copyobj(D.UI.txtImgCorr(1),D.UI.tabTT);
        D.UI.sldSwtchImg(2) = copyobj(D.UI.sldSwtchImg(1),D.UI.tabTT);
        D.UI.sldSwtchImg(3) = copyobj(D.UI.sldSwtchImg(1),D.UI.tabTT);
        
        % Change slider and text positions
        D.UI.txtImgCorr(2).Position(2) = swtch_btm - (swtch_ht + swtch_ht/2);
        D.UI.sldSwtchImg(2).Position(2) = swtch_btm - (swtch_ht + swtch_ht/2);
        D.UI.txtImgCorr(3).Position(2) = swtch_btm - (swtch_ht + swtch_ht/2)*2;
        D.UI.sldSwtchImg(3).Position(2) = swtch_btm - (swtch_ht + swtch_ht/2)*2;
        
        % Loop through each view
        for z_v = 1:3
            
            % Change text backround color
            Safe_Set(D.UI.txtImgCorr(z_v), 'BackgroundColor', D.UI.paxViewCol(z_v,:))
            
            % Set user data
            Safe_Set(D.UI.sldSwtchImg(z_v), 'UserData', z_v);
            
            % Set slider steps
            n_img = length(D.TT.imgCoor{z_v});
            Safe_Set(D.UI.sldSwtchImg(z_v), ...
                'Max', n_img, ...
                'SliderStep', [1/n_img, 5*(1/n_img)], ...
                'Value', floor(n_img/2));
        end
        
        % Set callback
        Safe_Set(D.UI.sldSwtchImg, 'Callback', {@Sld_SwtchImg});
        
        % Set to first image
        Sld_SwtchImg(D.UI.sldSwtchImg(1));
        Sld_SwtchImg(D.UI.sldSwtchImg(2));
        Sld_SwtchImg(D.UI.sldSwtchImg(3));
        
        % ------------------------ TT HIDE SHOW TOGGLE ----------------------------
        
        % Create button
        wd = D.UI.bordOffWd;
        ht = D.UI.bordOffHt;
        D.UI.toggHideTT(1) = uicontrol('style','togglebutton', ...
            'Parent', D.UI.tabTT, ...
            'Units', 'Normalized', ...
            'String', sprintf('%c', char(216)),...
            'FontName', D.UI.btnFont,...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontWeight', 'Bold',...
            'FontSize',  D.UI.fontSzBtnMed(1), ...
            'Enable', 'off', ...
            'Value', 0);
        
        % Copy button
        D.UI.toggHideTT(2) = copyobj(D.UI.toggHideTT(1), D.UI.tabTT);
        
        % Set values
        Safe_Set(D.UI.toggHideTT(1), ...
            'Position', [D.UI.leg_pos{1}(1), D.UI.leg_pos{1}(2)+ D.UI.leg_pos{1}(4), wd, ht], ...
            'UserData', 1);
        Safe_Set(D.UI.toggHideTT(2), ...
            'Position', [D.UI.leg_pos{2}(1), D.UI.leg_pos{2}(2)+D.UI.leg_pos{2}(4), wd, ht], ...
            'UserData', 2);
        
        % Set callback
        Safe_Set(D.UI.toggHideTT, 'Callback', {@Togg_HideShowBndl});
        
        % -------------------------- PANEL POSITIONS ------------------------------
        
        % Panel tt select
        pan_ht = 0.385 - D.UI.bordOffHt;
        D.UI.tab_2_tt_select_pan_pos(1,:) = ...
            [0,  ...
            1 - pan_ht - 0.01, ...
            D.UI.main_ax_bounds(1)/2, ...
            pan_ht];
        D.UI.tab_2_tt_select_pan_pos(2,:) = D.UI.tab_2_tt_select_pan_pos(1,:);
        D.UI.tab_2_tt_select_pan_pos(2,1) = D.UI.tab_2_tt_select_pan_pos(1,3);
        
        % Panel tt track
        pan_ht = D.UI.tab_2_tt_select_pan_pos(1,2) - sum(D.UI.quit_btn_pos([2,4])) - D.UI.bordOffHt*2 - 0.02;
        pan_botm = sum(D.UI.quit_btn_pos([2,4])) + 0.01;
        D.UI.tt_track_pan_pos = ...
            [0,  ...
            pan_botm, ...
            D.UI.main_ax_bounds(1), ...
            pan_ht];
        
        % ------------------------- TT SELECT CONTROLS ----------------------------
        
        % Create TT select pannel
        for z_bndl = 1:2
            
            % Add bundle pannel
            D.UI.panSelectTT(z_bndl) = uipanel(...
                'Parent', D.UI.tabTT,...
                'Units', 'Normalized',...
                'BorderType', 'line',...
                'BorderWidth', 4,...
                'FontSize', D.UI.fontSzTxtLrg(1),...
                'FontWeight','Bold', ...
                'FontName', D.UI.titleFont, ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.disabledCol, ...
                'HighlightColor', D.UI.disabledCol, ...
                'TitlePosition', 'centertop',...
                'UserData', [],...
                'Clipping', 'on',...
                'Position', D.UI.tab_2_tt_select_pan_pos(z_bndl,:));
        end
        
        % Add action buttons
        ht = D.UI.bordOffHt;
        lft = 0;
        btm = D.UI.tab_2_tt_select_pan_pos(1,2) - ht;
        wd = D.UI.main_ax_bounds(1)/2;
        pos = [lft, btm, wd, ht];
        D.UI.toggDoTrackTT = uicontrol('style','togglebutton', ...
            'Parent', D.UI.tabTT, ...
            'Units', 'Normalized', ...
            'Enable', 'off', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'FontName', D.UI.btnFont, ...
            'FontWeight', 'Bold', ...
            'FontSize', D.UI.fontSzBtnLrg(1));
        
        % Copy main button
        D.UI.toggDoPlotTT = copyobj(D.UI.toggDoTrackTT, D.UI.tabTT);
        
        % Set main position
        D.UI.toggDoPlotTT.Position(2) = D.UI.toggDoTrackTT.Position(2) - ht;
        
        % Set main stuff
        Safe_Set(D.UI.toggDoTrackTT, ...
            'UserData', 1, ...
            'String', 'Track TTs')
        Safe_Set(D.UI.toggDoPlotTT, ...
            'UserData', 2, ...
            'String', 'Plot Spikes')
        
        % Set callback
        Safe_Set(D.UI.toggDoTrackTT, 'Callback', {@Togg_MainActionTT})
        Safe_Set(D.UI.toggDoPlotTT, 'Callback', {@Togg_MainActionTT})
        
        % Copy for sub buttons
        D.UI.toggFlagTT = copyobj(D.UI.toggDoTrackTT, D.UI.tabTT);
        D.UI.toggHearSourceTT(1) = copyobj(D.UI.toggDoTrackTT, D.UI.tabTT);
        D.UI.toggHearSourceTT(2) = copyobj(D.UI.toggDoTrackTT, D.UI.tabTT);
        D.UI.toggPlotTypeTT(1) = copyobj(D.UI.toggDoPlotTT, D.UI.tabTT);
        D.UI.toggPlotTypeTT(2) = copyobj(D.UI.toggDoPlotTT, D.UI.tabTT);
        
        % Set user data
        D.UI.toggFlagTT.UserData = 1;
        D.UI.toggHearSourceTT(1).UserData = 1;
        D.UI.toggHearSourceTT(2).UserData = 2;
        D.UI.toggPlotTypeTT(1).UserData = 1;
        D.UI.toggPlotTypeTT(2).UserData = 2;
        
        % Set postions and strings for 'Track TTs' sub button
        D.UI.toggFlagTT.String = 'Flags';
        D.UI.toggFlagTT.Position(1) = sum(D.UI.toggDoTrackTT.Position([1,3]));
        D.UI.toggFlagTT.Position(3) = wd/2;
        D.UI.toggHearSourceTT(1).String = 'TT';
        D.UI.toggHearSourceTT(1).Position(1) = sum(D.UI.toggFlagTT.Position([1,3]));
        D.UI.toggHearSourceTT(1).Position(3) = wd/4;
        D.UI.toggHearSourceTT(2).String = 'CS';
        D.UI.toggHearSourceTT(2).Position(1) = sum(D.UI.toggHearSourceTT(1).Position([1,3]));
        D.UI.toggHearSourceTT(2).Position(3) = wd/4;
        
        % Set postions and strings for 'Plot Spikes' sub button
        D.UI.toggPlotTypeTT(1).String = 'Spike';
        D.UI.toggPlotTypeTT(1).Position(1) = sum(D.UI.toggDoPlotTT.Position([1,3]));
        D.UI.toggPlotTypeTT(1).Position(3) = wd/2;
        D.UI.toggPlotTypeTT(2).String = 'Rate';
        D.UI.toggPlotTypeTT(2).Position(1) = D.UI.toggPlotTypeTT(2).Position(1) + 1.5*wd;
        D.UI.toggPlotTypeTT(2).Position(3) = wd/2;
        
        % Set callback
        Safe_Set(D.UI.toggFlagTT, 'Callback', {@Togg_FlagTT})
        Safe_Set(D.UI.toggHearSourceTT, 'Callback', {@Togg_HearSourceTT})
        Safe_Set(D.UI.toggPlotTypeTT, 'Callback', {@Togg_PlotTypeTT});
        
        % Store position for each panel
        lft_shft = D.UI.tab_1_tt_select_pan_pos(1,1) - D.UI.tab_2_tt_select_pan_pos(1,1);
        btm_shft = D.UI.tab_1_tt_select_pan_pos(1,2) - D.UI.tab_2_tt_select_pan_pos(1,2);
        
        % Move action type buttons
        D.UI.tab_2_main_act_pos = [D.UI.toggDoTrackTT.Position; D.UI.toggDoPlotTT.Position];
        for z_p = 1:2
            D.UI.tab_1_main_act_pos(z_p,:) = [D.UI.tab_2_main_act_pos(z_p,1:2) + [lft_shft, btm_shft], ...
                D.UI.tab_2_main_act_pos(z_p,3:end)];
        end
        
        % Move action select buttons
        D.UI.tab_2_flag_tt_pos = D.UI.toggFlagTT.Position;
        D.UI.tab_1_flag_tt_pos = [D.UI.tab_2_flag_tt_pos(1:2) + [lft_shft, btm_shft], ...
            D.UI.tab_2_flag_tt_pos(3:end)];
        
        for z_p = 1:2
            % Hear
            D.UI.tab_2_hear_pos(z_p,:) = D.UI.toggHearSourceTT(z_p).Position;
            D.UI.tab_1_hear_pos(z_p,:) = [D.UI.tab_2_hear_pos(z_p,1:2) + [lft_shft, btm_shft], ...
                D.UI.tab_2_hear_pos(z_p,3:end)];
            
            % Plot
            D.UI.tab_2_plot_pos(z_p,:) = D.UI.toggPlotTypeTT(z_p).Position;
            D.UI.tab_1_plot_pos(z_p,:) = [D.UI.tab_2_plot_pos(z_p,1:2) + [lft_shft, btm_shft], ...
                D.UI.tab_2_plot_pos(z_p,3:end)];
        end
        
        % --------------------------- TT TURN CONTROLS ----------------------------
        
        % TT settings pannel
        D.UI.panTrackTT = uipanel(...
            'Parent', D.UI.tabTT,...
            'Units', 'Normalized',...
            'BorderType', 'line',...
            'BorderWidth', 4,...
            'FontSize', D.UI.fontSzTxtLrg(1),...
            'FontWeight','Bold', ...
            'FontName', D.UI.titleFont, ...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'Title','TTXX', ...
            'TitlePosition', 'centertop',...
            'Clipping', 'on',...
            'Position', D.UI.tt_track_pan_pos);
        
        % Plot default hight
        dflt_ht = ((1-0.001)/10)*0.8;
        % Plot defualt left from pan
        dflt_lf = [0.001, 0.45];
        % Plot defualt width
        dflt_wd = 1/2-0.002;
        % Plot default bottom pos vector
        dflt_btm = linspace(1-dflt_ht*0.25, dflt_ht*0.25, 12);
        
        % Create orientation text
        pos = [dflt_lf(1), dflt_btm(2), dflt_wd, dflt_ht];
        D.UI.txtPanTT(1) = uicontrol(...
            'Style', 'text', ...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName', D.UI.txtFont,...
            'FontSize', D.UI.fontSzTxtSml(1),...
            'FontWeight', 'Bold',...
            'HorizontalAlignment', 'left',...
            'Position', pos, ...
            'String', sprintf('New Screw\r\nOrientation'),...
            'Visible', 'off');
        
        % Create enter orientation popup-menue object
        pos = [dflt_lf(2), dflt_btm(2), dflt_wd+0.05, dflt_ht];
        D.UI.popOr = uicontrol(...
            'Style', 'popupmenu',...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'FontName', D.UI.popFont,...
            'BackgroundColor', [1 1 1],...
            'FontSize', D.UI.fontSzPopLrg(1),...
            'Position', pos,...
            'String', {' '},...
            'Enable', 'off', ...
            'Visible', 'off', ...
            'UserData', ones(D.PAR.maxTT,1), ...
            'Value', 1);
        
        % Create enter number of rotations text
        pos = [dflt_lf(1), dflt_btm(3), dflt_wd, dflt_ht];
        D.UI.txtPanTT(2) = copyobj(D.UI.txtPanTT(1), D.UI.panTrackTT);
        Safe_Set(D.UI.txtPanTT(2), ...
            'Position', pos,...
            'String', sprintf('Number of\r\nFull Turns'))
        % Create number of rotations popup-menue object
        pos = [dflt_lf(2), dflt_btm(3), dflt_wd+0.05, dflt_ht];
        D.UI.popTrn = uicontrol(...
            'Style', 'popupmenu',...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'FontName', D.UI.popFont,...
            'BackgroundColor', [1 1 1],...
            'FontSize', D.UI.fontSzPopLrg(1),...
            'Position', pos,...
            'String', num2cell((0:20)'),...
            'Enable', 'off', ...
            'Visible', 'off', ...
            'UserData', ones(D.PAR.maxTT,1), ...
            'Value', 1);
        
        % Create direction text
        pos = [dflt_lf(1), dflt_btm(4), dflt_wd, dflt_ht];
        D.UI.txtPanTT(3) = copyobj(D.UI.txtPanTT(1), D.UI.panTrackTT);
        Safe_Set(D.UI.txtPanTT(3), ...
            'Position', pos,...
            'String', sprintf('TT Lowering\r\nDirection'))
        % Create direction popup-menue object
        pos = [dflt_lf(2), dflt_btm(4), dflt_wd+0.05, dflt_ht];
        D.UI.popDir = uicontrol(...
            'Style', 'popupmenu',...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'FontName', D.UI.popFont,...
            'BackgroundColor', [1 1 1],...
            'FontSize', D.UI.fontSzPopLrg(1),...
            'Position', pos,...
            'String', {'Down'; 'Up'},...
            'Enable', 'off', ...
            'Visible', 'off', ...
            'UserData', ones(D.PAR.maxTT,1), ...
            'Value', 1);
        
        % Create ref text
        pos = [dflt_lf(1), dflt_btm(5), dflt_wd, dflt_ht];
        D.UI.txtPanTT(4) = copyobj(D.UI.txtPanTT(1), D.UI.panTrackTT);
        Safe_Set(D.UI.txtPanTT(4), ...
            'Position', pos,...
            'String', sprintf('TT\r\nReference'))
        % Create ref popup-menue object
        pos = [dflt_lf(2), dflt_btm(5), dflt_wd+0.05, dflt_ht];
        D.UI.popRefTT = uicontrol(...
            'Style', 'popupmenu',...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'FontName', D.UI.popFont,...
            'BackgroundColor', [1 1 1],...
            'FontSize', D.UI.fontSzPopLrg(1),...
            'Position', pos,...
            'String', {'Down'; 'Up'},...
            'Enable', 'off', ...
            'Visible', 'off', ...
            'Value', 1);
        
        % Create impedance main text
        pos = [dflt_lf(1), dflt_btm(6)+dflt_ht*0.5, dflt_wd*2, dflt_ht/2];
        D.UI.txtPanTT(5) = copyobj(D.UI.txtPanTT(1), D.UI.panTrackTT);
        Safe_Set(D.UI.txtPanTT(5), ...
            'Position', pos,...
            'HorizontalAlignment', 'center', ...
            'String', sprintf('Electrode Impedance (M%c)', char(911)))
        % Create channel text
        D.UI.txtPanTT(6) = copyobj(D.UI.txtPanTT(1), D.UI.panTrackTT);
        pos = [dflt_lf(1), dflt_btm(6)+dflt_ht*0.125, dflt_wd*2, dflt_ht/2];
        Safe_Set(D.UI.txtPanTT(6), ...
            'Position', pos,...
            'FontSize', 6, ...
            'String', '1|R1/5  2|R2/6 3|R3/7  4|R4/8   Gnd')
        % Create 5 impedance edit boxes
        pos = [dflt_lf(1), dflt_btm(6)-dflt_ht*0.1, dflt_wd*(2/5), dflt_ht/2];
        D.UI.editImpTT = gobjects(5,1);
        for z_t = 1:5
            % Edit box
            D.UI.editImpTT(z_t) = uicontrol(...
                'Style', 'edit',...
                'Max', 1, ...
                'Parent', D.UI.panTrackTT,...
                'BackgroundColor', [1 1 1],...
                'HorizontalAlignment', 'left',...
                'Units', 'Normalized',...
                'FontSize', D.UI.fontSzTxtSml(1),...
                'Visible', 'off', ...
                'Position', pos);
            pos(1) = pos(1)+pos(3);
        end
        
        % Create old notes table
        pos = [dflt_lf(1), dflt_btm(9)-dflt_ht*0.25, dflt_wd*2, dflt_ht*1.5];
        D.UI.tblNoteTT = uitable(...
            D.UI.panTrackTT, ...
            'Units', 'normalized', ...
            'Position', pos, ...
            'FontSize', 7, ...
            'ColumnName', {}, ...
            'RowName', {}, ...
            'Enable', 'inactive', ...
            'Visible', 'off');
        
        % Create new notes text box object
        pos = [dflt_lf(1), pos(2)+pos(4), dflt_wd*2, dflt_ht*1.25];
        D.UI.editNoteTT = uicontrol(...
            'Style', 'edit',...
            'Max', 100, ...
            'Parent', D.UI.panTrackTT,...
            'BackgroundColor', [1 1 1],...
            'HorizontalAlignment', 'left',...
            'Units', 'Normalized',...
            'FontSize', D.UI.fontSzTxtSml(1),...
            'Visible', 'off', ...
            'Position', pos);
        
        % Create notes heading
        pos = [dflt_lf(1), pos(2)+pos(4), dflt_wd, dflt_ht*0.4];
        D.UI.txtPanTT(7) = copyobj(D.UI.txtPanTT(1), D.UI.panTrackTT);
        Safe_Set(D.UI.txtPanTT(7), ...
            'Position', pos,...
            'String', 'Notes')
        
        % Create save TT data button object
        pos = [0.25, dflt_btm(10), 0.5, dflt_ht*0.75];
        D.UI.toggUpdateLogTT = uicontrol(...
            'Style','togglebutton', ...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'Callback',{@Togg_UpdateLogTT},...
            'UserData', 0,...
            'FontSize', D.UI.fontSzBtnLrg(1),...
            'FontWeight', 'Bold',...
            'Position', pos,...
            'BackgroundColor', D.UI.disabledCol, ...
            'ForegroundColor', D.UI.disabledBtnFrgCol, ...
            'Enable', 'off', ...
            'Visible', 'off', ...
            'String', 'Update XX');
        
        % Create bottom sub-pannel
        lft = 0.05;
        wd = dflt_wd*2 - lft*2;
        pos = [lft, dflt_btm(end), wd, dflt_ht*2];
        D.UI.spanTrackTT = uibuttongroup(...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'FontSize', D.UI.fontSzTxtSml(1),...
            'BackgroundColor', D.UI.figBckCol, ...
            'HighlightColor',D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight', 'Bold',...
            'TitlePosition', 'centertop', ...
            'Title', 'Orientation & Depth',...
            'Clipping', 'off',...
            'Position', pos,...
            'Visible', 'off');
        
        % Create TT depth text object
        lft = lft*2;
        wd = dflt_wd*2 - lft*2;
        btm = dflt_btm(end)+0.01;
        pos = [lft, btm, wd, dflt_ht*0.5];
        D.UI.txtPanTT(9) = uicontrol(...
            'Style', 'text', ...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName', D.UI.txtFont,...
            'FontSize', D.UI.fontSzTxtHuge(1),...
            'FontWeight', 'Bold',...
            'HorizontalAlignment', 'center',...
            'Position', pos,...
            'String', 'XX',...
            'Visible', 'off');
        
        % Create TT orientation text object
        pos = [pos(1), btm+dflt_ht*0.75, pos(3), dflt_ht*0.5];
        D.UI.txtPanTT(8) = uicontrol(...
            'Style', 'text', ...
            'Parent', D.UI.panTrackTT,...
            'Units', 'Normalized',...
            'BackgroundColor', D.UI.figBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName', D.UI.txtFont,...
            'FontSize', D.UI.fontSzTxtHuge(1),...
            'FontWeight', 'Bold',...
            'HorizontalAlignment', 'center',...
            'Position', pos,...
            'String', 'XX',...
            'Visible', 'off');
        
        % Set text colors
        Safe_Set(D.UI.txtPanTT, 'ForegroundColor', D.UI.disabledCol)
        
        %% ===================== SETUP RAT TT STUFF =======================
        
        % -------------------SETUP BUNDLE CONFIG LEGENDS-------------------
        
        % Specify tt colors
        D.UI.ttCol = hsv(D.PAR.maxTT/2);
        
        % Set third entry to yellow
        D.UI.ttCol(3,:) = [1,1,0];
        
        % Diplicate for second bundle
        D.UI.ttCol = [D.UI.ttCol; D.UI.ttCol];
        
        %         % GET CLUSTER COLOR TO MATCH TT
        %
        %         % Get tt and clust specific color map
        %         tt_col_map = NaN(D.PAR.maxTT, size(D.UI.linColBarH,2), 3);
        %         for z_tt = 1:length(D.TT.ttLab)
        %             hsv_col = rgb2hsv(D.UI.ttCol(z_tt, :));
        %             hsv_col = repmat(hsv_col, size(D.UI.linColBarH,2), 1);
        %             hsv_col(:,2) = linspace(0.1, 1, size(D.UI.linColBarH,2))';
        %             tt_col_map(z_tt,:,:) = hsv2rgb(hsv_col);
        %         end
        %
        %         % Store colors for each clust
        %         for z_tt = 1:length(D.TT.ttLab)
        %
        %             % Set the first cluster of each tt to the tt color
        %             col_arr = circshift(D.UI.ttCol, -1*(z_tt-1), 1);
        %
        %             % Specify cluster color
        %             col_arr = col_arr(1:D.PAR.maxClust,:);
        %             D.UI.clustCol(z_tt,:,:) =  permute(col_arr, [3,1,2]);
        %
        %             % Specify cluster color map
        %             col_mat = circshift(tt_col_map, -1*(z_tt-1), 1);
        %             col_mat = col_mat(1:D.PAR.maxClust,:,:);
        %             D.UI.clustColMap(z_tt,:,:,:) = col_mat;
        %         end
        
        % GET CLUSTER COLOR TO MATCH NLX
        
        % Get cluster color
        clust_col = [[0.25,0.25,0.25]; D.UI.ttCol];
        D.UI.clustCol = repmat(clust_col, [1,1,D.PAR.maxTT]);
        D.UI.clustCol = permute(D.UI.clustCol, [3,1,2]);
        
        % Get tt and clust specific color map
        clust_col_map = NaN(D.PAR.maxClust, size(D.UI.linColBarH,2), 3);
        for z_c = 1:D.PAR.maxClust
            
            % Handle first clust
            if z_c == 1
                col_mat = ...
                    repmat(linspace(0.9, min(clust_col(z_c, :)), size(D.UI.linColBarH,2))', 1,3);
                clust_col_map(z_c,:,:) = col_mat;
            end
            
            % Get graded color map by incrimenting saturaton of hsv space
            if z_c ~= 1
                hsv_col = rgb2hsv(clust_col(z_c, :));
                hsv_col = repmat(hsv_col, size(D.UI.linColBarH,2), 1);
                hsv_col(:,2) = linspace(0.1, 1, size(D.UI.linColBarH,2))';
                clust_col_map(z_c,:,:) = hsv2rgb(hsv_col);
            end
            
        end
        
        % Store cluster color map
        col_mat = repmat(clust_col_map, [1,1,1,D.PAR.maxTT]);
        D.UI.clustColMap = permute(col_mat, [4,1,2,3]);
        
        % Resize legend axis
        center = (max([size(D.TT.ttMap{2}),size(D.TT.ttMap{1})]) + 0.5)/2;
        
        % Store titles
        title_arr = {sprintf('    %s Bundle', D.TT.bndlLab{1}), ...
            sprintf('    %s Bundle', D.TT.bndlLab{2}), ...
            'Drive Top'};
        
        % Store axes labels
        xlab_arr = {sprintf('A-P %0.2fmm', D.TT.ttCoords{1}(1)), ...
            sprintf('A-P %0.2fmm', D.TT.ttCoords{2}(1)), ...
            'Medial'};
        ylab_arr = {sprintf('M-L %0.2fmm', D.TT.ttCoords{1}(2)), ...
            sprintf('M-L %0.2fmm', D.TT.ttCoords{2}(2)), ...
            'Anterior'};
        
        % Plot TT bundle loc legends
        for z_l = 1:3
            
            % Add bundle title
            Safe_Set(D.UI.axBndlLeg(z_l), 'Title', ...
                text(...
                'String', title_arr{z_l}, ...
                'FontSize',D.UI.fontSzTxtLrg(1), ...
                'FontWeight', 'bold', ...
                'Color', 'k'))
            
            % Show implant A-P coordinates on x axes
            Safe_Set(D.UI.axBndlLeg(z_l).XLabel, ...
                'Units', 'Normalized', ...
                'FontSize', D.UI.fontSzTxtSml(1), ...
                'String', xlab_arr{z_l});
            D.UI.axBndlLeg(z_l).XLabel.Position(2) = D.UI.axBndlLeg(z_l).XLabel.Position(2)+0.05;
            
            % Show implant M-L coordinates on y axes
            Safe_Set(D.UI.axBndlLeg(z_l).YLabel, ...
                'Units', 'Normalized', ...
                'FontSize', D.UI.fontSzTxtSml(1), ...
                'String', ylab_arr{z_l});
            D.UI.axBndlLeg(z_l).YLabel.Position(1) = D.UI.axBndlLeg(z_l).YLabel.Position(1)+0.05;
            
            % Bail for last loop
            if z_l == 3
                continue
            end
            
            % Set axis lims
            Safe_Set(D.UI.axBndlLeg(z_l), ...
                'XLim', [D.TT.ttConfig{z_l}(1)/2 - center+ 0.5, D.TT.ttConfig{z_l}(1)/2 + center + 0.5], ...
                'YLim', [D.TT.ttConfig{z_l}(2)/2 - center+ 0.5, D.TT.ttConfig{z_l}(2)/2 + center + 0.5])
            
            % Add panel title
            Safe_Set(D.UI.panSelectTT(z_l), 'Title', D.TT.bndlLab{z_l});
            
            % Create markers with text
            tt_num_list = D.TT.ttNum(D.I.ttBndl==z_l);
            tt_lab_list = D.TT.ttLab(D.I.ttBndl==z_l);
            for z_tt = 1:length(tt_lab_list)
                
                % Find position of next tt
                [x,y] = ...
                    find(ismember(D.TT.ttMap{z_l}, tt_lab_list{z_tt}) == 1);
                
                % Get actual tt ind
                tt_ind = ismember(D.TT.ttLab, tt_lab_list{z_tt});
                
                % Bail if not found
                if isempty(x) || isempty(y)
                    continue
                end
                
                % Plot tt marker
                D.UI.ttBndlLegMrk(tt_ind) = ...
                    line(x , y, ...
                    'LineStyle', 'none', ...
                    'Marker', 'o', ...
                    'MarkerEdgeColor', D.UI.ttCol(tt_ind,:), ...
                    'MarkerFaceColor', D.UI.ttCol(tt_ind,:)*0.5, ...
                    'LineWidth', D.UI.ttBndlLegMrkWdth(1), ...
                    'MarkerSize', 13, ...
                    'Parent', D.UI.axBndlLeg(z_l));
                
                % Plot tt text
                D.UI.ttBndlLegTxt(tt_ind) = ...
                    text(x , y, tt_num_list{z_tt}, ...
                    'Color', [1, 1, 1], ...
                    'HorizontalAlignment', 'Center', ...
                    'FontSize', D.UI.fontSzTxtSml(1), ...
                    'FontWeight', 'bold', ...
                    'Parent', D.UI.axBndlLeg(z_l));
            end
        end
        
        % Get TT loc
        tt_lab_shift = circshift(D.TT.ttLab, 1);
        rad_arr = linspace(0, 2*pi - 2*pi*(1/length(tt_lab_shift)), length(tt_lab_shift));
        rad_arr = circshift(rad_arr, length(rad_arr)/2, 2) - (2*pi*(1/length(tt_lab_shift))/2);
        rad_arr = wrapTo2Pi(rad_arr);
        tt_loc_x = 0.85 * sin(rad_arr);
        tt_loc_y = 0.85 * cos(rad_arr);
        
        % Set axis lims
        Safe_Set(D.UI.axBndlLeg(3), ...
            'XLim', [-1,1], ...
            'YLim', [-1,1])
        
        % Plot TT loc on drive
        for z_tt = 1:length(tt_lab_shift)
            
            % Get bundle inds
            tt_lab_list = tt_lab_shift{z_tt};
            tt_ind = find(ismember(D.TT.ttLab, tt_lab_list));
            
            % Copy objects
            D.UI.ttDriveLegMrk(tt_ind) = ...
                copyobj(D.UI.ttBndlLegMrk(tt_ind), D.UI.axBndlLeg(3));
            D.UI.ttDriveLegTxt(tt_ind) = ...
                copyobj(D.UI.ttBndlLegTxt(tt_ind), D.UI.axBndlLeg(3));
            
            % Change position
            D.UI.ttDriveLegMrk(tt_ind).XData = tt_loc_x(z_tt);
            D.UI.ttDriveLegMrk(tt_ind).YData = tt_loc_y(z_tt);
            D.UI.ttDriveLegTxt(tt_ind).Position(1) = tt_loc_x(z_tt);
            D.UI.ttDriveLegTxt(tt_ind).Position(2) = tt_loc_y(z_tt);
        end
        
        % ------------------SETUP TT SELECT PANEL OBJECTS------------------
        
        % Set pop ref list
        D.UI.popRefTT.String = D.TT.refMap;
        
        % Button position
        wdth_norm = 0.3;
        lft_norm = 0.005;
        ht_norm = 1/(length(D.TT.ttLab)/2) * 0.85;
        btm_norm = ...
            repmat(linspace(1-ht_norm*1.25, 0.005, (length(D.TT.ttLab)/2)),1,2);
        
        % Flag features sub toggles
        set_flag_labs = ...
            {'U', char(415), 'R', char(708), 'N',  ...
            char(216), '1', '2', '3', '4'};
        
        % Looop through each TT
        for z_tt = 1:length(D.TT.ttLab)
            
            % Get bundle ind
            bndl_ind = D.I.ttBndl(z_tt);
            
            % ADD MAIN TT SELECT BUTTONS
            btn_tt_pos = [lft_norm(1), btm_norm(z_tt), wdth_norm, ht_norm];
            D.UI.toggSelectTT(z_tt) = ...
                uicontrol('style','togglebutton', ...
                'Parent', D.UI.panSelectTT(bndl_ind), ...
                'Enable', 'on', ...
                'BackgroundColor', D.UI.enabledCol, ...
                'ForegroundColor', D.UI.ttCol(z_tt,:), ...
                'String', D.TT.ttNum{z_tt},...
                'Callback', {@Togg_SelectTT},...
                'UserData', z_tt, ...
                'Units', 'Normalized', ...
                'Value', 0, ...
                'Position', btn_tt_pos, ...
                'FontName', D.UI.btnFont,...
                'FontWeight', 'Bold',...
                'FontSize', D.UI.fontSzBtnLrg(1), ...
                'Visible', 'off');
            
            % Plot tt track
            Plot_TT_Path(z_tt);
            
            % ADD CLUSTER BUTTONS
            wd = (1 - btn_tt_pos(3) - btn_tt_pos(1)*3)/5;
            ht = btn_tt_pos(4)/2;
            btm_arr = ...
                [repmat(btn_tt_pos(2)+ht,5,1);repmat(btn_tt_pos(2),5,1)];
            lft_arr = ...
                repmat(linspace(btn_tt_pos(1)+btn_tt_pos(3), btn_tt_pos(1)+btn_tt_pos(3) + wd*4, 5), 1, 2);
            for z_c = 1:D.PAR.maxClust
                
                % Create cluster button
                btn_c_pos = [lft_arr(z_c), btm_arr(z_c), wd, ht];
                D.UI.toggSubPlotTT(z_tt,z_c) = ...
                    uicontrol('style','togglebutton', ...
                    'Parent', D.UI.panSelectTT(bndl_ind), ...
                    'Enable', 'Off', ...
                    'BackgroundColor', D.UI.enabledCol, ...
                    'ForegroundColor', D.UI.clustCol(z_tt,z_c,:), ...
                    'String',num2str(z_c-1),...
                    'Callback', {@Togg_SubPlotTT},...
                    'UserData', [z_tt, z_c], ...
                    'Units','Normalized', ...
                    'Value', 0, ...
                    'Position', btn_c_pos, ...
                    'FontName',D.UI.btnFont,...
                    'FontWeight','Bold',...
                    'Visible', 'off', ...
                    'FontSize', D.UI.fontSzBtnSml(1));
                
            end
            
            % ADD FLAG FEATURES AND TT SUB BUTTONS
            
            % Copy cluster buttons
            D.UI.toggSubFlagTT(z_tt,:) = copyobj(D.UI.toggSubPlotTT(z_tt,:), D.UI.panSelectTT(bndl_ind));
            
            % Modify sub flag buttons
            for z_f = 1:10
                
                % Get last value
                val = D.TT.ttLogNew.([D.TT.ttLab{z_tt},'_F'])(z_f);
                
                % Set string etc
                Safe_Set(D.UI.toggSubFlagTT(z_tt,z_f), ...
                    'String', set_flag_labs{z_f}, ...
                    'ForegroundColor', D.UI.enabledBtnFrgCol, ...
                    'UserData', [z_tt,z_f], ...
                    'Callback', {@Togg_FlagTT}, ...
                    'Value', val);
                
            end
            
            % ADD HEAR TT CHAN BUTTONS
            
            % Copy cluster buttons
            D.UI.toggSubHearSdTT(z_tt,1) = copyobj(D.UI.toggSubPlotTT(z_tt,1), D.UI.panSelectTT(bndl_ind));
            D.UI.toggSubHearSdTT(z_tt,2) = copyobj(D.UI.toggSubPlotTT(z_tt,6), D.UI.panSelectTT(bndl_ind));
            D.UI.toggSubHearChTT(z_tt,:,1) = copyobj(D.UI.toggSubPlotTT(z_tt,2:5), D.UI.panSelectTT(bndl_ind));
            D.UI.toggSubHearChTT(z_tt,:,2) = copyobj(D.UI.toggSubPlotTT(z_tt,7:end), D.UI.panSelectTT(bndl_ind));
            
            % Modify buttons
            sn_lab = {'L','R'};
            for z_sn = 1:2
                
                % Set sound side button
                Safe_Set(D.UI.toggSubHearSdTT(z_tt,z_sn), ...
                    'String', sn_lab{z_sn}, ...
                    'ForegroundColor', D.UI.enabledBtnFrgCol, ...
                    'UserData', [z_tt,0,z_sn], ...
                    'Callback', {@Togg_SubHearTT});
                
                for z_chan = 1:4
                    
                    % Set values
                    Safe_Set(D.UI.toggSubHearChTT(z_tt,z_chan,z_sn), ...
                        'String', num2str(z_chan), ...
                        'ForegroundColor', D.UI.hearCol(z_chan,:), ...
                        'UserData', [z_tt,z_chan,z_sn], ...
                        'Callback', {@Togg_SubHearTT});
                end
            end
            
            % Modify buttons for ref chanels
            if contains(D.TT.ttLab{z_tt}, 'R')
                
                % Delete chan flag buttons > 0
                delete(D.UI.toggSubFlagTT(z_tt, 8:end));
                
                % Delete chan hear buttons > 0
                delete(D.UI.toggSubHearChTT(z_tt, 2:4));
                delete(D.UI.toggSubHearChTT(z_tt, 6:end));
            end
            
        end
        
        % ---------------------SET PAX IMAGE POSITIONS---------------------
        
        % Set paxinos sag center of first bundle
        img_ind = knnsearch(D.TT.imgCoor{1}', D.TT.ttCoords{1}(2));
        Safe_Set(D.UI.sldSwtchImg(1), 'Value', img_ind);
        Sld_SwtchImg(D.UI.sldSwtchImg(1));
        
        % Set paxinos cor center of first bundle
        img_ind = knnsearch(D.TT.imgCoor{2}', D.TT.ttCoords{1}(1));
        Safe_Set(D.UI.sldSwtchImg(2), 'Value', img_ind);
        Sld_SwtchImg(D.UI.sldSwtchImg(2));
        
        % Set hor to deepest tt
        dpth_max = 0;
        for z_tt = 1:length(D.TT.ttLab)
            dpth_max = max(dpth_max, D.TT.ttLogNew.([D.TT.ttLab{z_tt},'_D']));
        end
        img_ind = knnsearch(D.TT.imgCoor{3}', -1*dpth_max/1000);
        Safe_Set(D.UI.sldSwtchImg(3), 'Value', img_ind);
        Sld_SwtchImg(D.UI.sldSwtchImg(3));
        
        %% ====================== AUTO SET TT DEPTH =======================
        if (D.DB.doAutoSetTT)
            
            % Switch tab
            Tab_GrpChange(D.UI.tabTT);
            
            % Enable buttons
            Object_Group_State('TT_Track_Objects', 'Enable');
            Safe_Set(D.UI.toggDoTrackTT, 'Value', 1);
            Togg_MainActionTT(D.UI.toggDoTrackTT);
            
            % Number of turns
            turns = D.DB.autoSetTTturns;
            
            % Get number of full turns
            turn_ind = ...
                find(ismember(cellstr(get(D.UI.popTrn, 'String')), num2str(turns)));
            if isempty(turn_ind)
                turn_ind = length(get(D.UI.popTrn,'String'));
            end
            
            % Looop through each TT
            for z_tt = 1:length(D.TT.ttLab)
                
                % Load next TT
                Safe_Set(D.UI.toggSelectTT(z_tt), 'Value', 1);
                Togg_SelectTT(D.UI.toggSelectTT(z_tt));
                
                % Pause
                pause(0.05);
                
                % Set turns to max
                set(D.UI.popTrn, 'Value', turn_ind);
                
                % Pause
                pause(0.05);
                
                % Save
                Safe_Set(D.UI.toggUpdateLogTT, 'Value', 1);
                Togg_UpdateLogTT();
            end
        end
        
    end

% ---------------------------CONNECT TO NLX------------------------
    function[was_ran] = NLX_Connect()
        
        %% BAIL IF CHEETAH NOT RUNNING
        
        % Initialize output
        was_ran = false;
        
        % Bail
        if ~D.F.cheetah_running
            return
        end
        
        % Set flag
        was_ran = true;
        
        %% CONNECT TO NETCOM
        
        % Wait for Cheetah to open
        Console_Write('[NLX_Connect] RUNNING: Confirm Cheetah.exe Running...');
        while true
            % Check EXE status
            D.F.cheetah_running = Check_EXE('Cheetah.exe');
            
            % Get status
            [abort, pass] = ...
                Check_Flag(DOEXIT, D.F.cheetah_running);
            if abort || pass; break; end
        end
        
        % Check status
        if abort
            Console_Write('**WARNING** [NLX_Connect] ABORTED: Wait for Cheetah.exe to Open');
            return
        elseif pass
            Console_Write('[NLX_Connect] FINISHED: Wait for Cheetah.exe to Open');
        end
        
        % Connect to NetCom
        Console_Write(sprintf('[NLX_Connect] RUNNING: Connect to NLX IP=%s...', ...
            D.NLX.ServerIP));
        
        % Keep attempting till success
        while true
            
            % Get status
            [abort, pass] = ...
                Check_Flag(DOEXIT, NlxAreWeConnected() == 1);
            if abort || pass; break; end
            
            % Attempt connection
            NlxConnectToServer(D.NLX.ServerIP);
            
        end
        
        % Check status
        if abort
            Console_Write('**WARNING** [NLX_Connect] ABORTED: Connect to NLX');
            return
        elseif pass
            Console_Write('[NLX_Connect] FINISHED: Connect to NLX');
            
            % Identify id to server
            NlxSetApplicationName('ICR_GUI');
            
        end
        
        % Bail if exit initiated
        if DOEXIT
            return
        end
        
        % Send test command to confirm connected and get first ts
        [pass, ts_string] = Send_NLX_Cmd('-GetTimestamp');
        if pass == 1
            D.T.poll_str_nlx = int64(str2double(ts_string));
        end
        
        % Check if successful
        if NlxAreWeConnected() == 1
            
            Console_Write('[NLX_Connect] FINISHED: Confirm NLX Connection');
        else
            
            % Set exit flag and bail
            Console_Write('!!ERROR!! [NLX_Connect] FAILED: Confirm NLX Connection');
            SetExit();
            return
        end
    end

% --------------------------FINISH EPHYS SETUP------------------------
    function[was_ran] = NLX_Setup()
        
        %% BAIL IF CHEETAH NOT RUNNING
        
        % Initialize output
        was_ran = false;
        
        % Bail
        if ~D.F.cheetah_running
            return
        end
        
        % Set flag
        was_ran = true;
        
        %% CONFIGURE DIGITAL IO
        
        % Log/print
        Console_Write(sprintf('[NLX_Setup] RUNNING: Configure NLX...'));
        
        % SETUP PORTS
        
        % Set port directions
        Send_NLX_Cmd(['-SetDigitalIOportDirection ', D.NLX.DevTTL, ' ', D.NLX.port_0, ' Input']);
        Send_NLX_Cmd(['-SetDigitalIOportDirection ', D.NLX.DevTTL, ' ', D.NLX.port_1, ' Input']);
        
        % Enable digital io events
        Send_NLX_Cmd(['-SetDigitalIOEventsEnabled ', D.NLX.DevTTL, ' ', D.NLX.port_0, ' True']);
        Send_NLX_Cmd(['-SetDigitalIOEventsEnabled ', D.NLX.DevTTL, ' ', D.NLX.port_1, ' True']);
        
        % CONFIGURE TTL EVENTS
        
        % Audio channels
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.snd_rt_wn_bit{1}, ' ', D.NLX.snd_rt_wn_bit{2}, ' ', D.NLX.snd_rt_wn_str]);
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.snd_lft_rt_bit{1}, ' ', D.NLX.snd_lft_rt_bit{2}, ' ', D.NLX.snd_lft_rt_str]);
        
        % Reward
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.rew_on_bit{1}, ' ', D.NLX.rew_on_bit{2}, ' ', D.NLX.rew_on_str]);
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.rew_off_bit{1}, ' ', D.NLX.rew_off_bit{2}, ' ', D.NLX.rew_off_str]);
        
        % Pid state
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.pid_run_bit{1}, ' ', D.NLX.pid_run_bit{2}, ' ', D.NLX.pid_run_str]);
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.pid_stop_bit{1}, ' ', D.NLX.pid_stop_bit{2}, ' ', D.NLX.pid_stop_str]);
        
        % Bulldozer state
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.bull_run_bit{1}, ' ', D.NLX.bull_run_bit{2}, ' ', D.NLX.bull_run_str]);
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.bull_stop_bit{1}, ' ', D.NLX.bull_stop_bit{2}, ' ', D.NLX.bull_stop_str]);
        
        % Photo transdicers
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.north_bit{1}, ' ', D.NLX.north_bit{2}, ' ', D.NLX.north_str]);
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.west_bit{1}, ' ', D.NLX.west_bit{2}, ' ', D.NLX.west_str]);
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.south_bit{1}, ' ', D.NLX.south_bit{2}, ' ', D.NLX.south_str]);
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.east_bit{1}, ' ', D.NLX.east_bit{2}, ' ', D.NLX.east_str]);
        
        % IR time sync LED
        Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' ', D.NLX.ir_sync_bit{1}, ' ', D.NLX.ir_sync_bit{2}, ' ', D.NLX.ir_ts_str]);
        
        % Bail if exit initiated
        if DOEXIT
            return
        else
            Console_Write('[NLX_Setup] FINISHED: Configure NLX');
        end
        
        %% LOAD SETUP CONFIGURATION SETTINGS
        
        % NLX setup config
        if D.F.implant_session
            setup_cfg_fi = 'ICR_Cheetah_Setup_Ephys.cfg';
        else
            setup_cfg_fi = 'ICR_Cheetah_Setup_Behavior.cfg';
        end
        
        % Log/print
        Console_Write(sprintf('[NLX_Setup] RUNNING: Load Cheetah Setup Config: "%s"...', setup_cfg_fi));
        
        % Load setup config
        succeeded = Send_NLX_Cmd(sprintf('-ProcessConfigurationFile %s', setup_cfg_fi));
        
        % Log/print
        if succeeded == 1
            Console_Write(sprintf('[NLX_Setup] FINISHED: Load Cheetah Setup Config: "%s"', setup_cfg_fi));
        else
            Console_Write(sprintf('**WARNING** [NLX_Setup] FAILED: Load Cheetah Setup Config: "%s"', setup_cfg_fi));
        end
        
        %% DISPLAY PROMPT TO POWER ON CUBE
        
        % Start Cube
        if D.F.implant_session
            
            % Prompt to turn on cube
            dlg_h = dlgAWL( ...
                'Power on Cube', ...
                'START CUBE', ...
                'OK', [], [], 'OK', ...
                D.UI.dlgPos{4}, ...
                'default');
            Dlg_Wait(dlg_h);
            
        end
        
        %% LOAD PREVIOUS SETTINGS
        
        % Find current NLX recording directory
        dirs = dir(D.DIR.nlxTempTop);
        dirs = dirs(3:end);
        fi_dat_num = ...
            cell2mat(cellfun(@(x) datenum(x, 'yyyy-mm-dd_HH-MM-SS'), {dirs.name}, 'uni', false));
        D.DIR.recFi = dirs(fi_dat_num == max(fi_dat_num)).name;
        
        % Save directory var
        % format: datestr(now, 'yyyy-mm-dd_HH-MM-SS', 'local');
        D.DIR.nlxRecRat = fullfile(D.DIR.nlxSaveTop, D.PAR.ratLab(2:end));
        
        % Make directory if none exists
        if exist(D.DIR.nlxRecRat, 'dir') == 0 && ...
                exist(D.DIR.nlxSaveTop, 'dir') == 1
            Console_Write(sprintf('[NLX_Setup] RUNNING: Make Rat Rec Directory: "%s"', D.DIR.nlxRecRat));
            mkdir(D.DIR.nlxRecRat);
        end
        
        % Load previous settings
        if D.F.implant_session
            
            % Log/print
            Console_Write('[NLX_Setup] RUNNING: Load Previous Cheetah Settings...');
            
            % Initialize file
            nlx_cfg_last = [];
            
            % Find last rec dir
            if exist(D.DIR.nlxSaveTop, 'dir') > 0 && ...
                    size(dir(D.DIR.nlxRecRat), 1) > 2
                
                % Get config file path
                nlx_cfg_last = ...
                    fullfile(D.DIR.nlxRecRat, 'CheetahLastConfiguration.cfg');
                
                % Set to empty if file does not exist
                if ~exist(nlx_cfg_last, 'file')
                    nlx_cfg_last = [];
                end
                
            end
            
            % Initialize flag
            is_settings_loaded = false;
            
            % Parse last config file
            if ~isempty(nlx_cfg_last) && ~DOEXIT
                
                % List of commands to pars
                parse_list = { ...
                    '-SetInputRange', ...
                    '-SetSpikeThreshold', ...
                    };
                
                % Read in text file
                fi_imp = fileread(nlx_cfg_last);
                
                % Initialize message list
                msg_list = cell(length(parse_list),1);
                
                % Parse file
                for z_cmd = 1:length(parse_list)
                    
                    % Get matching commands
                    msg_list{z_cmd} = regexp(fi_imp, ['(', parse_list{z_cmd}, '[^\r\n\f]*)(?=\r|\n|\f)'], 'match');
            
                end
                
                % Concatinate message
                msg_list = [msg_list{:}];
                
                % Create dialogue msg
                dlg_msg = ...
                    [sprintf('LOAD PREVIOUS SETTINGS:\n'), ...
                    sprintf('\n\t%s', msg_list{:})];
                
                % Check if settings should be used
                dlg_h = dlgAWL(...
                    dlg_msg, ...
                    'LOAD CONFIG SETTINGS', ...
                    'Yes', 'No', [], 'No', ...
                    D.UI.dlgPos{4}, ...
                    'question', ...
                    true);
                set(dlg_h ,'WindowStyle','normal')
                choice = Dlg_Wait(dlg_h);
                
                % Load last settings
                if strcmp(choice, 'Yes') && ~DOEXIT
                    
                    for i_msg = 1:length(msg_list)
                        
                        % Send each command
                        Send_NLX_Cmd(msg_list{i_msg});
                        
                    end
                    
                    % Set flag
                    is_settings_loaded = true;
                end
            end
            
            % Log/print status
            if is_settings_loaded
                Console_Write('[NLX_Setup] FINISHED: Load Previous Cheetah Settings');
                
            elseif DOEXIT
                Console_Write('**WARNING** [NLX_Setup] ABORTED: Load Previous Cheetah Settings');
                
                % Bail
                return
                
            else
                Console_Write('[NLX_Setup] SKIPPED: Load Previous Cheetah Settings');
            end
        end
        
        %% RUN SPIKESORT EXE
        
        % Check if SpikeSort should be run
        if D.F.cheetah_running && D.F.implant_session
            
            dlg_h = dlgAWL(...
                'Do you want to run SpikeSort3D.exe?', ...
                'RUN SS3D?', ...
                'Yes', 'No', [], 'No', ...
                D.UI.dlgPos{4}, ...
                'question');
            choice = Dlg_Wait(dlg_h);
            
        else
            choice = 'No';
        end
        
        % Recheck EXE status
        D.F.spikesort_running = Check_EXE('SpikeSort3D.exe');
        
        % Run SpikeSort3D.exe
        if strcmp('Yes', choice) && ...
                ~D.F.spikesort_running && ...
                ~DOEXIT
            
            Console_Write('[NLX_Setup] RUNNING: Open SpikeSort3D.exe...');
            
            % Specify start config
            start_cfg_path = fullfile(D.DIR.nlxCfg, 'ICR_SpikeSort3D.cfg');
            
            % Store current directory
            curdir = pwd;
            
            % Run EXE with specified config
            cd(D.DIR.nlxSS3DEXE);
            system(sprintf('SpikeSort3D.exe %s&', start_cfg_path));
            cd(curdir);
            
            Console_Write('[NLX_Setup] FINISHED: Open SpikeSort3D.exe...');
        else
            Console_Write('[NLX_Setup] SKIPPED: Open SpikeSort3D.exe');
        end
        
        %% OPEN NETCOM STREAMS
        
        % Open VT and Event streams
        Console_Write('[NLX_Setup] RUNNING: Open VT and Event Streams...');
        
        % Check for exit
        if DOEXIT
            % Bail
            Console_Write('**WARNING** [NLX_Setup] ABORTED: Open VT and Event Streams');
            return
        end
        
        % Get all DAS objects
        Console_Write('[NLX_Setup] RUNNING: "NlxGetDASObjectsAndTypes()"...');
        [succeeded, D.NLX.das_objects, ~] = NlxGetDASObjectsAndTypes();
        if succeeded == 1
            Console_Write('[NLX_Setup] FINISHED: "NlxGetDASObjectsAndTypes()"...');
        else
            Console_Write('!!ERROR!! [NLX_Setup] FAILED: "NlxGetDASObjectsAndTypes()"...');
        end
        
        % Open rat vt stream
        if any(contains(D.NLX.das_objects, D.NLX.vt_rat_ent))
            D.F.vt_rat_streaming = NlxOpenStream(D.NLX.vt_rat_ent) == 1;
            Console_Write(sprintf('[NLX_Setup] FINISHED: Open NLX "%s" Stream: status=%d', ...
                D.NLX.vt_rat_ent, D.F.vt_rat_streaming ));
        else
            Console_Write(sprintf('!!ERROR!! [NLX_Setup] Missing DAS Object: "%s"', D.NLX.vt_rat_ent));
        end
        
        % Open robot vt stream
        if any(contains(D.NLX.das_objects, D.NLX.vt_rob_ent))
            D.F.vt_rob_streaming = NlxOpenStream(D.NLX.vt_rob_ent) == 1;
            Console_Write(sprintf('[NLX_Setup] FINISHED: Open NLX "%s" Stream: status=%d', ...
                D.NLX.vt_rob_ent, D.F.vt_rob_streaming ));
        else
            Console_Write(sprintf('!!ERROR!! [NLX_Setup] Missing DAS Object: "%s"', D.NLX.vt_rob_ent));
        end
        
        % Open event vt stream
        if any(contains(D.NLX.das_objects, D.NLX.event_ent))
            D.F.evt_streaming = NlxOpenStream(D.NLX.event_ent) == 1;
            Console_Write(sprintf('[NLX_Setup] FINISHED: Open NLX "%s" Stream: status=%d', ...
                D.NLX.event_ent, D.F.evt_streaming ));
        else
            Console_Write(sprintf('!!ERROR!! [NLX_Setup] Missing DAS Object: "%s"', D.NLX.event_ent));
        end
        
        D.F.evt_streaming = NlxOpenStream(D.NLX.event_ent) == 1;
        Console_Write(sprintf('[NLX_Setup] FINISHED: Open NLX "%s" Stream: status=%d', ...
            D.NLX.event_ent, D.F.evt_streaming));
        
        % Log/print stream status
        if D.F.vt_rat_streaming && ...
                D.F.vt_rob_streaming  && ...
                D.F.evt_streaming
            % All streaming
            Console_Write(sprintf('[NLX_Setup] FINISHED: Open VT and Event Streams: vt1=%d vt2=%d evt=%d', ...
                D.F.vt_rat_streaming , D.F.vt_rob_streaming , D.F.evt_streaming));
        else
            % Open stream failed
            Console_Write(sprintf('**WARING** [NLX_Setup] FAILED: Open VT and Event Streams: vt1=%d vt2=%d evt=%d', ...
                D.F.vt_rat_streaming , D.F.vt_rob_streaming , D.F.evt_streaming));
        end
        
        % Update/refresh window positions
        Togg_Mon(D.UI.toggMon(D.UI.monDefault));
        
        %% DISABLE PLOTTING OF FLAGGED TTS
        
        % Get flagged chanels
        for z_tt = 1:length(D.TT.ttLab)
            
            % Store flag
            D.F.tt_chan_disable(z_tt,:) = Safe_Get(D.UI.toggSubFlagTT(z_tt, 7:end), 'Value');
            
            % Run callback for exluded elecrodes
            if all(D.F.tt_chan_disable(z_tt,:))
                
                % Disable all elecrodes for this tt
                Togg_FlagTT(D.UI.toggSubFlagTT(z_tt, 6));
                
            elseif any(D.F.tt_chan_disable(z_tt,:))
                
                % Disable specific elecrodes
                for z_c = 1:4
                    if D.F.tt_chan_disable(z_tt,z_c)
                        Togg_FlagTT(D.UI.toggSubFlagTT(z_tt, 6+z_c));
                    end
                end
            end
            
        end
        
    end

% --------------------------FINISH AC SETUP------------------------
    function Finish_AC_Setup()
        
        % Display image
        D.AC.data(2) = 1;
        
        % No sound
        if ~D.F.sound(1) && ~D.F.sound(2)
            D.AC.data(3) = single(0);
        end
        
        % White noise only
        if D.F.sound(1) && ~D.F.sound(2)
            D.AC.data(3) = single(1);
        end
        
        % White noise and reward tone only
        if D.F.sound(1) && D.F.sound(2)
            D.AC.data(3) = single(2);
        end
        
        % Post to AC computer
        Send_AC_Com();
        
    end

% -----------------------------SETUP ICR SESSION---------------------------
    function ICR_Session_Setup()
        
        %% UPDATE SESSION SPECIFIC VARS
        
        % Handle inplant session
        if D.F.implant_session
            
            % Specify inplant setpoint
            D.PAR.setPointCM = D.PAR.setPointImplant;
            D.PAR.setPointRad = D.PAR.setPointCM * ((2 * pi)/(140 * pi));
            
            % Set stream hd flag
            D.F.stream_hd = true;
            
        else
            
            % Specify behavior setpoint
            D.PAR.setPointCM = D.PAR.setPointBackpack;
            D.PAR.setPointRad = D.PAR.setPointCM * ((2 * pi)/(140 * pi));
        end
        
        % Get session number
        var_ind = ...
            ismember(D.SS_IO_1.Properties.VariableNames, ['Session_',char(D.PAR.sesCond)]);
        col_ind = ismember([{'Track'},{'Forage'}], D.PAR.sesTask);
        D.PAR.sesNum = ...
            D.SS_IO_1{D.PAR.ratIndSS, var_ind}(col_ind) + 1;
        
        % Save new session number back to SS_IO_1
        D.SS_IO_1{D.PAR.ratIndSS, var_ind}(col_ind) = D.PAR.sesNum;
        
        % Get session total
        D.PAR.sesNumAll = D.SS_IO_1{D.PAR.ratIndSS, 'Session_Manual_Training'}(col_ind) + ...;
            D.SS_IO_1{D.PAR.ratIndSS, 'Session_Behavior_Training'}(col_ind) + ...
            D.SS_IO_1{D.PAR.ratIndSS, 'Session_Implant_Training'}(col_ind) + ...
            D.SS_IO_1{D.PAR.ratIndSS, 'Session_Rotation'};
        
        % Start quadrant
        D.PAR.ratStrQuad = categorical({'<undefined>'}, D.PAR.listStrQuad);
        
        % Get start quad for 'Track' task
        if D.PAR.sesTask == 'Track'
            
            % Manual track session set start quad to feeder pos
            if D.PAR.sesCond == 'Manual_Training'
                
                if D.PAR.ratFeedCnd == 'C1'
                    D.PAR.ratStrQuad(:) = 'NW';
                else
                    D.PAR.ratStrQuad(:) = 'SE';
                end
                
                % Set start quad to stored quad
            else
                
                D.PAR.ratStrQuad = ... % [NE,SE,SW,NW]
                    D.SS_IO_1.Start_Quadrant{D.PAR.ratIndSS}(D.PAR.sesNumAll);
            end
            
        end
        
        % For forage session always set start to 'NE'
        if D.PAR.sesTask == 'Forage'
            D.PAR.ratStrQuad(:) = 'NE';
        end
        
        % Direction of rotation string
        D.PAR.ratRotDrc = ... % [CCW,CW]
            D.SS_IO_1.Rotation_Direction{D.PAR.ratIndSS}(D.PAR.sesNum);
        
        % Rotations per session
        D.PAR.rotPerSes = ... % [2,4,6]
            str2double(char(D.SS_IO_1.Rotations_Per_Session{D.PAR.ratIndSS}(D.PAR.sesNum,:)));
        
        % Rotations position
        D.PAR.rotPosList = ... % [90,180,270]
            D.SS_IO_1.Rotation_Positions{D.PAR.ratIndSS}(D.PAR.sesNum,:)';
        
        % Laps per session
        D.PAR.lapsPerRot = ... % [5:8,6:9,7:10]
            D.SS_IO_1.Laps_Per_Rotation{D.PAR.ratIndSS}(D.PAR.sesNum,:)';
        
        % Days till rotation
        D.PAR.daysTilRot = ... % [5:8,6:9,7:10]
            D.SS_IO_1.Days_Till_Rotation{D.PAR.ratIndSS}(D.PAR.sesNum);
        
        % Seed random number generator based on session number
        rng(D.PAR.sesNumAll)
        
        % Make rew per ses list
        txt = [{'Laps Per Rot'}; ...
            cellfun(@(x,y) sprintf('%d:    %s', x, y), ...
            num2cell(1:length(D.PAR.rotPosList))', ...
            cellstr(char(D.PAR.lapsPerRot)), 'Uni', false)];
        Safe_Set(D.UI.popLapsPerRot, 'String', txt);
        
        % Make rot pos list
        txt = [{'Rotation Pos'}; ...
            cellfun(@(x,y) sprintf('%d:    %s', x, y), ...
            num2cell(1:length(D.PAR.rotPosList))', ...
            cellstr(char(D.PAR.rotPosList)), 'Uni', false)];
        Safe_Set(D.UI.popRotPos, 'String', txt);
        
        % Generate rotation button string
        D.UI.btnICRstr = ...
            [{sprintf('40%c %s', char(176), char(D.PAR.listRotDrc(D.PAR.listRotDrc ~= D.PAR.ratRotDrc)))}; ...
            {sprintf('40%c %s', char(176), char(D.PAR.ratRotDrc))}];
        
        % Modify vars based on rot dir
        if D.PAR.ratRotDrc == 'CCW'
            D.I.img_ind(2) = 2;
        elseif D.PAR.ratRotDrc == 'CW'
            D.I.img_ind(2) = 3;
        end
        
        %% COMPUTE BOUNDS
        
        % Specify reward feeder locations
        % NOTE: Feeder index is based on the position of the feeder with
        % respect to the 0 deg point (East quadrant)in the arena
        rewFeeds = [11, 15, 7; 29, 33, 25];
        
        % Feeder paramiters
        % boundary before and after rew feeder (deg)
        D.PAR.trigDist = rad2deg(D.PAR.feedDistRad - D.PAR.setPointRad);
        D.PAR.feedSet = [...
            D.PAR.trigDist - 2.5, ...
            D.PAR.trigDist + 2.5];
        
        % Get reward and unrewarded feeder based on rotation direction
        % corrected index
        D.UI.rewFeed = rewFeeds(D.PAR.ratFeedCnd_Num, D.I.img_ind);
        D.UI.oppFeed = rewFeeds([1, 2] ~=  D.PAR.ratFeedCnd_Num, D.I.img_ind);
        
        % Calculate feeder locations
        
        % Calculate all feeder/strut locations
        fdLocs = circshift((0:10:350)+5,[0,0]);
        
        % Calculate all feed locs
        [fd_X,fd_Y] = pol2cart(deg2rad(fdLocs), ones(1,length(fdLocs)) * D.UI.arnRad);
        % all feeders x
        D.UI.fd_x = fd_X*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
        % all feeders y
        D.UI.fd_y = fd_Y*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
        
        % Save reward feeder/pos rad pos
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
        D.UI.rewRstBnds(1,1:2) = D.PAR.rewZoneBnds(1,end,1) + [deg2rad(10),deg2rad(30)];
        D.UI.rewRstBnds(2,1:2) = D.PAR.rewZoneBnds(1,end,2) + [deg2rad(10),deg2rad(30)];
        D.UI.rewRstBnds = wrapTo2Pi(D.UI.rewRstBnds);
        
        % REWARD PASS BOUNDS
        D.UI.rewPassBnds(1,1:2) = D.PAR.rewZoneBnds(end,end,1) - [deg2rad(30), deg2rad(10)];
        D.UI.rewPassBnds(2,1:2) = D.PAR.rewZoneBnds(end,end,2) - [deg2rad(30), deg2rad(10)];
        D.UI.rewPassBnds = wrapTo2Pi(D.UI.rewPassBnds);
        
        % ROTATION BOUNDS
        
        % Calculate crossing points for 90 180 and 270 deg from rew
        % feeders/pos
        bnd_space = 20;
        bnd_wdth = 30;
        D.PAR.rotDistDeg = 110:bnd_space:290;
        D.UI.rotLocs = [...
            fdLocs(D.UI.rewFeed(1)) + D.PAR.trigDist + D.PAR.rotDistDeg', ...
            fdLocs(D.UI.rewFeed(2)) + D.PAR.trigDist + D.PAR.rotDistDeg'];
        
        % Calculate 30 deg wide bounds for each rotation pos
        rot_bnds = arrayfun(@(x,y) cat(3, [x-bnd_space, x], [y-bnd_space, y]), ...
            D.UI.rotLocs(:,1), D.UI.rotLocs(:,2), 'Uni', false);
        rot_bnds = cell2mat(rot_bnds);
        % set range to [0, 360]
        rot_bnds = wrapTo360(rot_bnds);
        % convert to radians
        rot_bnds = deg2rad(rot_bnds);
        rot_bnds = wrapTo2Pi(rot_bnds);
        D.UI.rotBnds = rot_bnds;
        D.PAR.rotBnds = rot_bnds;
        D.PAR.rotBnds(:,1,:) = wrapTo2Pi(rot_bnds(:,1,:)-deg2rad(bnd_wdth-bnd_space));
        D.UI.rotCatInd = knnsearch([90,180,270]', D.PAR.rotDistDeg');
        
        % LAP BOUNDS
        
        % Calculate lap count crossing points for every 90 deg
        % first ind is NE going ccw
        quadBndLocs = 45:90:360;
        % shift so that start quadrant is last entry
        strQuadInd = find(D.PAR.listStrQuad == D.PAR.ratStrQuad);
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
        
        % Set 60 deg start quadrant bound to last lap bound
        D.PAR.strQuadBnds = [lapBndLocs(end) - 30, ...
            lapBndLocs(end) + 30];
        
        % Convert to radians
        D.PAR.strQuadBnds = deg2rad(D.PAR.strQuadBnds);
        D.PAR.strQuadBnds = wrapTo2Pi(D.PAR.strQuadBnds);
        
        % FORAGE REWARD TARGET BOUNDS
        
        % [zone,min_max,rot_cond]
        D.PAR.rewTargBnds = NaN(length(D.PAR.frgTargDegArr),2);
        for z_targ = 1:length(D.PAR.frgTargDegArr)
            D.PAR.rewTargBnds(z_targ,:) = [...
                Rad_Diff(deg2rad(D.PAR.frgTargWdt/2), deg2rad(D.PAR.frgTargDegArr(z_targ)), 'wrap'), ...
                Rad_Sum(deg2rad(D.PAR.frgTargWdt/2), deg2rad(D.PAR.frgTargDegArr(z_targ)))];
        end
        D.PAR.rewTargBnds = wrapTo2Pi(D.PAR.rewTargBnds);
        
        %% SETUP ARENA FEATURES
        
        % Plot all feeders/struts
        D.UI.fdAllH = line(D.UI.fd_x, D.UI.fd_y, ...
            'LineStyle', 'none', ...
            'Marker', 'o', ...
            'MarkerFaceColor', [0.5 0.5 0.5], ...
            'MarkerEdgeColor', [0.1,0.1,0.1], ...
            'MarkerSize', 20, ...
            'Visible', 'on', ...
            'Parent', D.UI.axH(4));
        
        % Plot strut pos in rad and cm
        fd_cm = (140*pi)-(140*pi)/36/2:-(140*pi)/36:(140*pi)/36/2;
        fd_rad = (2*pi)/36/2:(2*pi)/36:(2*pi)-(2*pi)/36/2;
        D.UI.txtFdPosH = gobjects(1,36);
        for z_fd = 1:36
            D.UI.txtFdPosH(z_fd) = text(...
                D.UI.fd_x(z_fd), D.UI.fd_y(z_fd), ...
                sprintf('%0.1f\n%0.0f', fd_rad(z_fd), fd_cm(z_fd)), ...
                'Color', [1, 1, 1], ...
                'HorizontalAlignment', 'center', ...
                'FontSize', 6, ...
                'FontWeight', 'bold', ...
                'Visible', 'on', ...
                'Parent', D.UI.axH(4));
            
            % Plot start quadrant
            [xbnd, ybnd] =  Get_Cart_Bnds(D.PAR.strQuadBnds);
            D.UI.ptchStQ = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                [0.5,0.5,0.5], ...
                'FaceAlpha',0.5, ...
                'EdgeAlpha',0.5, ...
                'Visible', 'off', ...
                'HitTest', 'off', ...
                'Parent',D.UI.axH(3));
            % add start text
            D.UI.txtStQ = ...
                text(mean(xbnd(:,round(size(xbnd,2)/2))), ...
                mean(ybnd(:,round(size(ybnd,2)/2))), ...
                '', ...
                'FontSize', D.UI.fontSzTxtLrg(1), ...
                'FontWeight', 'Bold', ...
                'Color', D.UI.enabledCol, ...
                'HorizontalAlignment', 'Center', ...
                'Visible', 'off', ...
                'HitTest', 'off', ...
                'Parent',D.UI.axH(3));
        end
        
        %% PRINT TEXT INFO
        
        % SESSION INFO
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
        Safe_Set(D.UI.txtSesInf(1),'String', infstr)
        
        % ROTATION INFO
        if D.PAR.sesCond == 'Rotation'
            rots = num2str(D.PAR.rotPerSes);
        else
            rots = 'NA';
        end
        infstr = sprintf([...
            'Rotation Info\n', ...
            'Rotations:%s%s\n'], ...
            repmat('_',1,5), rots);
        Safe_Set(D.UI.txtSesInf(2),'String', infstr)
        
        % PERFORMANCE INFO
        
        % Totals
        infstr = sprintf([...
            'Laps______All:%s%d\n', ...
            'Rewards___All:%s%d\n', ...
            'Rotations_All:%s%d\n', ...
            'Missed_Rewards_:%s%d|%d\n',...
            'Bulldozings____:%s%d'], ...
            repmat('_',1,1), 0, ...
            repmat('_',1,1), 0, ...
            repmat('_',1,1), 0, ...
            repmat('_',1,1), 0,0, ...
            repmat('_',1,1), 0);
        Safe_Set(D.UI.txtPerfInf(4), 'String', infstr)
        
        % Standard laps
        infstr = sprintf([...
            'Laps______Stand:%s%d\n', ...
            'Rewards___Stand:%s%d'], ...
            repmat('_',1,1), 0, ...
            repmat('_',1,1), 0);
        Safe_Set(D.UI.txtPerfInf(1), 'String', infstr)
        
        % 40 deg laps
        infstr = sprintf([...
            'Laps______40%c:%s%d|%d\n', ...
            'Rewards___40%c:%s%d|%d'], ...
            char(176), repmat('_',1,3), 0,0, ...
            char(176), repmat('_',1,3), 0,0);
        Safe_Set(D.UI.txtPerfInf(2), 'String', infstr)
        
        % 0 deg laps
        infstr = sprintf([...
            'Laps______0%c:%s%d|%d\n', ...
            'Rewards___0%c:%s%d|%d'], ...
            char(176), repmat('_',1,4), 0,0, ...
            char(176), repmat('_',1,4), 0,0);
        Safe_Set(D.UI.txtPerfInf(3), 'String', infstr)
        
        % Rat vel
        infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', 0, 0, 0);
        Safe_Set(D.UI.txtPerfInf(5), 'String', infstr)
        
        % Rob vel
        infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', 0, 0, 0);
        Safe_Set(D.UI.txtPerfInf(6), 'String', infstr)
        
        % Robot bat volt
        infstr = sprintf('Rob_Battery_:_%0.1fV', 0);
        Safe_Set(D.UI.txtPerfInf(7), 'String', infstr)
        
        % Cube bat volt percent
        infstr = sprintf('Cube_Battery:_%d%%', 0);
        Safe_Set(D.UI.txtPerfInf(8), 'String', infstr)
        
        % TIMER INFO
        
        % Elapsed time
        infstr = sprintf([ ...
            'SES:%s%s\n', ...
            'REC:%s%s\n', ...
            'LAP:%s%s\n', ...
            'RUN:%s%s\n'], ...
            repmat('_',1,1), datestr(0, 'HH:MM:SS'), ...
            repmat('_',1,1), datestr(0, 'HH:MM:SS'), ...
            repmat('_',1,1), datestr(0, 'HH:MM:SS'), ...
            repmat('_',1,1), datestr(0, 'HH:MM:SS'));
        Safe_Set(D.UI.txtTimeInf(1), 'String', infstr)
        
        % Start time
        infstr = sprintf('Start: ');
        Safe_Set(D.UI.txtTimeInf(2), 'String', infstr)
        infstr = datestr(TIMSTRLOCAL, 'HH:MM:SS');
        Safe_Set(D.UI.editTimeInf(1), 'String', infstr)
        
        % End time
        infstr = sprintf('Stop_: ');
        Safe_Set(D.UI.txtTimeInf(3), 'String', infstr)
        infstr = datestr(0, 'HH:MM:SS');
        Safe_Set(D.UI.editTimeInf(2), 'String', infstr)
        
        % DB timers
        infstr = sprintf( ...
            [...
            'pEv: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'pVT: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'pTT: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'Inf: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'Plt: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'Drw: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'Lop: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            ], ...
            0, 0, 0, 0, ...
            0, 0, 0, 0, ...
            0, 0, 0, 0, ...
            0, 0, 0, 0, ...
            0, 0, 0, 0, ...
            0, 0, 0, 0, ...
            0, 0, 0, 0 ...
            );
        Safe_Set(D.UI.txtPerfInf(9), 'String', infstr)
        
        %% SPECIFY AXIS UI STACK ORDER
        
        % Random Forage Task OCC Data
        uistack(D.UI.axH(5),'top');
        
        % Pos/Vel History Data
        uistack(D.UI.axH(2),'top');
        
        % Real Time Track Task Data
        uistack(D.UI.axH(3),'top');
        
        % Zone Plot Axes
        uistack(D.UI.axZoneH(2), 'top');
        uistack(D.UI.axZoneH(1), 'top');
        
        % Wall Image
        uistack(D.UI.axH(1),'top');
        
        % Arena Features
        uistack(D.UI.axH(4),'top');
        
        % Real Time TT OCC Data
        uistack(D.UI.axClstH,'top');
        
        % TT Clust Corlor Legend
        uistack(D.UI.axColLeg,'top');
        
        % Real Time TT Position/OCC Data
        uistack(D.UI.axH(6),'top');
        
        % Selectable Items
        uistack(D.UI.axH(7),'top');
        
        %% SETUP EPHYS PLOTTING
        
        % Bail here if TT TRACK SESSION
        if D.F.implant_session
            
            % SETUP FIRING RATE AXES
            Console_Write('[NLX_Setup] RUNNING: Setup Cluster Rate Patches...');
            
            % Calculate patch bounds
            ptch_bounds = arrayfun(@(x) [x, x+((2*pi)/D.PAR.tt1dBins)], ...
                D.PAR.tt1dBinEdge', 'Uni', false);
            ptch_bounds = cell2mat(ptch_bounds);
            ptch_bounds = flip(ptch_bounds, 1);
            ptch_bounds = wrapTo2Pi(ptch_bounds);
            
            % Create rate patch objects
            if D.PAR.sesTask == 'Track'
                for z_c = 1:D.PAR.maxClust
                    
                    % Create/copy rate patch objects
                    if z_c == 1
                        
                        % Create patch object
                        for z_p = 1:D.PAR.tt1dBins
                            [xbnd, ybnd] =  ...
                                Get_Cart_Bnds(ptch_bounds(z_p,:));
                            D.UI.ptchClustH(z_c,z_p) = ...
                                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                                D.UI.disabledCol, ...
                                'EdgeAlpha', 0, ...
                                'FaceAlpha', 0, ...
                                'Visible', 'on', ...
                                'Parent', D.UI.axClstH(z_c));
                        end
                    else
                        
                        % Copy patch object
                        for z_p = 1:D.PAR.tt1dBins
                            D.UI.ptchClustH(z_c,z_p) = ...
                                copyobj(D.UI.ptchClustH(1,z_p), D.UI.axClstH(z_c));
                        end
                    end
                    
                end
                
            end
            
            % Rescale axis for hist data
            if D.PAR.sesTask == 'Forage'
                
                % Rescale axis for hist data
                Safe_Set(D.UI.axClstH, ...
                    'YDir', 'reverse', ...
                    'Position', D.UI.axH(5).Position, ...
                    'XLim', [0,D.PAR.tt2dBins], ...
                    'YLim', [0,D.PAR.tt2dBins]);
            end
            
            % Set axis user data properties
            Safe_Set(D.UI.axClstH, ...
                'CLim',[0,1], ...
                'UserData', 0);
            
            % SETUP PLOT COLOR MAP
            Console_Write('[NLX_Setup] RUNNING: Setup Cluster Colormaps...');
            
            % Colorbar legend pos
            ax_pos = [...
                D.UI.tim_inf_pan_pos(1), ...
                D.UI.cnsl_pan_pos(4), ...
                D.UI.tim_inf_pan_pos(3), ...
                0.115];
            
            % Setup legend axis
            set(D.UI.axColLeg, ...
                'Position', ax_pos,...
                'Color', [1 1 1],...
                'XLim', [0,1], ...
                'YLim', [0, D.PAR.maxClust], ...
                'XTick', [], ...
                'YTick', [])
            
            % Setup clust color map legend
            for z_c = 1:D.PAR.maxClust
                
                % Set axis to default colormap
                colormap(D.UI.axClstH(z_c), D.UI.disabledCol)
                
                % Add 'colorbar'
                y_max = size(D.UI.linColBarH,2);
                x = repmat(linspace(1/y_max/2,1-1/y_max/2,size(D.UI.linColBarH,2)),2,1);
                y = repmat([0;1], 1, size(D.UI.linColBarH,2)) + D.PAR.maxClust-z_c;
                
                % Add 'colorbar'
                D.UI.linColBarH(z_c,:) = ...
                    line(...
                    x, y, ...
                    'LineWidth', 4, ...
                    'Parent',D.UI.axColLeg, ...
                    'Visible', 'off');
                
                % Add text
                btm = sum(ax_pos([2,4])) - (ax_pos(4)/D.PAR.maxClust)*z_c;
                pos_txt = [ax_pos(1)-ax_pos(3)/3, btm, ax_pos(3)/3, ax_pos(4)/D.PAR.maxClust];
                D.UI.txtColBarH(z_c) = uicontrol('Style', 'text',...
                    'Parent', D.UI.tabICR, ...
                    'Units', 'Normalized', ...
                    'BackgroundColor', D.UI.figBckCol, ...
                    'ForegroundColor', D.UI.enabledCol, ...
                    'FontSize', 8, ...
                    'FontWeight', 'Bold', ...
                    'String',sprintf('TTxx-%d', z_c-1),...
                    'Visible', 'off',...
                    'Position', pos_txt);
                
            end
            
            % Create legend outline
            y_max = D.PAR.maxClust;
            x = [0, 0, NaN, 1, 1, NaN];
            y = [0, y_max, NaN, y_max, 0, NaN];
            y = [y, reshape([0:y_max; 0:y_max; NaN(1,y_max+1)], 1, [])];
            x = [x, repmat([0,1,NaN], 1, y_max+1)];
            
            % Plot outline
            line(...
                x, y, ...
                'Color', 'k', ...
                'LineWidth', 0.5, ...
                'Parent',D.UI.axColLeg);
            
            % Set legend to default col
            set(D.UI.linColBarH, ...
                'Color', D.UI.disabledCol, ...
                'Visible', 'on');
            
            % Log/print
            Console_Write('[NLX_Setup] FINISHED: Setup Cluster Objects');
            
        else
            
            Console_Write('[NLX_Setup] SKIPPED: Setup Cluster Objects');
            
        end
        
        %% SEND SETUP COMMAND TO ROBOT
        
        % Session condition
        if D.PAR.sesCond == 'Manual_Training'
            
            % Manual session
            D.PAR.sesMsg(1,1) = 1;
        elseif ~D.F.implant_session
            
            % Behavior session
            D.PAR.sesMsg(1,1) = 2;
        else
            
            % Implant session
            D.PAR.sesMsg(1,1) = 3;
        end
        
        % Task condition
        if D.PAR.sesTask == 'Track'
            
            % Track task
            D.PAR.sesMsg(1,2) = 1;
        elseif D.PAR.sesTask == 'Forage'
            
            % Forage task
            D.PAR.sesMsg(1,2) = 2;
        end
        
        % Sound condition
        if all(~D.F.sound)
            
            % No sound
            D.PAR.sesMsg(2,1) = 0;
        elseif ~D.F.sound(2)
            
            % White noise
            D.PAR.sesMsg(2,1) = 1;
        else
            
            % White and reward
            D.PAR.sesMsg(2,1) = 2;
        end
        
        % Pid setpoint
        if ~D.F.implant_session
            D.PAR.sesMsg(2,2) = D.PAR.setPointBackpack;
        else
            D.PAR.sesMsg(2,2) = D.PAR.setPointImplant;
        end
        
        % Start timer
        start(D.timer_graphics);
        
        % Log/print
        Console_Write('[ICR_Session_Setup] FINISHED: Session Setup');
        
    end

% --------------------------TRACK TASK SETUP-----------------------
    function[] = Track_Task_Setup()
        
        %% REWARD ZONE SETUP
        
        % Initialize long distrebution
        sub_samp = 100;
        x_long = 1:11*sub_samp;
        zone_short = linspace(-25,25,11);
        
        % Setup axes
        wd = 0.23;
        ht = 0.2*D.UI.main_ax_bounds(4);
        lft = (D.UI.main_ax_bounds(1)+(D.UI.main_ax_bounds(3)/2)) - wd/2;
        btm = D.UI.main_ax_bounds(2) + D.UI.main_ax_bounds(4)/2 - ht/2;
        zone_ax_pos = [...
            lft, ...
            btm, ...
            wd, ...
            ht ...
            ];
        Safe_Set(D.UI.axZoneH(1), ...
            'Color', 'none', ...
            'Position',zone_ax_pos, ...
            'XLim',[min(x_long)+sub_samp/2-1,max(x_long)-sub_samp/2], ...
            'Visible','off', ...
            'Parent', D.UI.tabICR);
        hold on;
        Safe_Set(D.UI.axZoneH(2), ...
            'Color','none', ...
            'Position',zone_ax_pos, ...
            'XLim',[min(zone_short)+2.5, max(zone_short)-2.5], ...
            'XTick',zone_short, ...
            'Visible','off', ...
            'Parent', D.UI.tabICR);
        box on;
        hold on
        Safe_Set(D.UI.axZoneH, ...
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
        Safe_Set(D.UI.axZoneH(1), 'YLim', [0,max(dist_long)]);
        
        % Plot example distrebution
        y = dist_long;
        y(y == 0) = NaN;
        line(x_long, y, ...
            'Color', [0,0,0], ...
            'LineWidth', 2, ...
            'Visible','off', ...
            'Parent',D.UI.axZoneH(1));
        
        % Plot center line
        x = repmat(round(max(x_long)/2),1,2);
        y = get(D.UI.axZoneH(1),'YLim');
        line(x, y, ...
            'Color', [0,0,0], ...
            'LineWidth', 2, ...
            'Color', [0, 0, 0], ...
            'Visible','off', ...
            'Parent',D.UI.axZoneH(1));
        
        % Subsample long distrebution
        short_ind = floor(linspace(1, length(x_long), length(zone_short)));
        dist_short = dist_long(short_ind);
        
        % Plot point for reward size at each zone
        x =  short_ind(2:end-1);
        y = D.PAR.zoneRewDur/max(D.PAR.zoneRewDur)*max(dist_long);
        line(x, y, ...
            'LineStyle', 'none', ...
            'Marker', 'o', ...
            'MarkerFaceColor', [0.5, 0.5, 0.5], ...
            'MarkerEdgeColor', [0.1, 0.1, 0.1], ...
            'MarkerSize', 10, ...
            'Visible','off', ...
            'Parent',D.UI.axZoneH(1));
        
        % Rescale dist and set main axis
        dist_short = dist_short * (1/sum(dist_short));
        % set axis
        Safe_Set(D.UI.axZoneH(2), ...
            'YLim' , [0, 1], ...
            'YTick',0:500/D.PAR.rewDurLim(2):1, ...
            'YTickLabel',0:500:D.PAR.rewDurLim(2), ...
            'XTickLabel' , [], ...
            'XGrid', 'on', ...
            'YGrid', 'on', ...
            'YMinorGrid', 'on');
        % make labels
        x_tic_labs = arrayfun(@(x,y,z) (sprintf('%d%c \n%d_{ms} \n(%0.0f%%)', x, char(176), y, z)), ...
            zone_short, [0,D.PAR.zoneRewDur,0], dist_short*100, 'uni', false);
        for z_tick = 2:length(x_tic_labs)-1
            text(D.UI.axZoneH(2).XTick(z_tick), -0.15, x_tic_labs{z_tick}, ...
                'FontSize', 7, ...
                'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', ...
                'Visible','off', ...
                'Parent', D.UI.axZoneH(2));
        end
        
        % Keep count shown on top of ax 1
        Safe_Set(D.UI.axZoneH(1), ...
            'XAxisLocation','top', ...
            'YTickLabel', [], ...
            'XTick', short_ind(2:end-1), ...
            'XTickLabel', D.C.zone(D.I.rot,:));
        
        %% CREATE ADITIONAL UI OBJECTS
        
        % Plot reward bounds
        for z_rot = 1:2
            for z_zone = 1:length(D.PAR.zoneLocs)
                
                % reward bounds
                [xbnd, ybnd] =  ...
                    Get_Cart_Bnds(D.PAR.rewZoneBnds(z_zone,:,z_rot));
                D.UI.ptchRewZoneBndsH(z_rot,z_zone) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    D.UI.rotCol(z_rot,:), ...
                    'ButtonDownFcn', {@Mouse_SelectRewZone}, ...
                    'EdgeColor', [0, 0, 0], ...
                    'EdgeAlpha', 0, ...
                    'FaceAlpha', 0.025, ...
                    'LineWidth', 1, ...
                    'Visible', 'off', ...
                    'UserData', [z_rot, z_zone], ...
                    'Parent', D.UI.axH(7));
                
                % Add reward duration text
                str = ...
                    sprintf('%d%c\n%d ms', -1*D.PAR.zoneLocs(z_zone), char(176), D.PAR.zoneRewDur(z_zone));
                D.UI.txtFdDurH(z_rot,z_zone) = text(...
                    mean(mean(xbnd)), mean(mean(ybnd)), ...
                    str, ...
                    'Color', [1, 1, 1], ...
                    'ButtonDownFcn', {@Mouse_SelectRewZone}, ...
                    'HorizontalAlignment', 'center', ...
                    'FontSize', D.UI.fontSzTxtSml(1), ...
                    'FontWeight', 'bold', ...
                    'Visible', 'off', ...
                    'UserData', [z_rot, z_zone], ...
                    'Parent', D.UI.axH(7));
            end
        end
        % bring text to top of stack
        uistack(D.UI.txtFdDurH(:),'top');
        
        % Plot reward reset bounds
        for z_rot = 1:2
            [xbnd, ybnd] =  ...
                Get_Cart_Bnds(D.UI.rewRstBnds(z_rot,:));
            D.UI.ptchRewSendH(z_rot) = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                D.UI.rotCol(z_rot,:), ...
                'FaceAlpha', 0.5, ...
                'EdgeAlpha', 0.5, ...
                'Visible', 'off', ...
                'HitTest', 'off', ...
                'Parent', D.UI.axH(3));
        end
        
        % Feeder reward feeder marker
        for z_rot = 1:2
            [xbnd, ybnd] =  ...
                Get_Cart_Bnds([D.UI.rewZoneRad(z_rot), ...
                D.UI.rewZoneRad(z_rot) + deg2rad(D.PAR.trigDist)]);
            
            % Setpoint zone line
            D.UI.mixFdNow(1,z_rot) = ...
                line(xbnd(:,end), ybnd(:,end), ...
                'Color', D.UI.rotCol(z_rot,:), ...
                'LineWidth', 2, ...
                'Visible', 'off', ...
                'HitTest', 'off', ...
                'Parent',D.UI.axH(3));
            
            % Distance line
            D.UI.mixFdNow(2,z_rot) = ...
                line(xbnd(2,:), ybnd(2,:), ...
                'Color', D.UI.rotCol(z_rot,:), ...
                'LineWidth', 5, ...
                'Visible', 'off', ...
                'HitTest', 'off', ...
                'Parent',D.UI.axH(3));
            
            % Marker
            D.UI.mixFdNow(3,z_rot) = ...
                line(D.UI.fd_x(D.UI.rewFeed(z_rot)), ...
                D.UI.fd_y(D.UI.rewFeed(z_rot)), ...
                'LineStyle', 'none', ...
                'Marker', 'o', ...
                'MarkerFaceColor', D.UI.rotCol(z_rot,:), ...
                'MarkerEdgeColor', [0.1,0.1,0.1], ...
                'MarkerSize', 20, ...
                'Visible', 'off', ...
                'HitTest', 'off', ...
                'Parent',D.UI.axH(4));
        end
        
        % Plot opposite unrewarded feeders darker
        line(D.UI.fd_x(D.UI.oppFeed), D.UI.fd_y(D.UI.oppFeed), ...
            'LineStyle', 'none', ...
            'Marker', 'o', ...
            'MarkerFaceColor', [0.25 0.25 0.25], ...
            'MarkerEdgeColor', [0.1,0.1,0.1], ...
            'MarkerSize', 20, ...
            'Parent',D.UI.axH(4));
        
        % Bring text back to top
        uistack(D.UI.txtFdPosH, 'top');
        
        % Plot all rot bounds
        for z_rot = 1:2
            for z_pos = 1:size(D.UI.rotBnds,1)
                [xbnd, ybnd] =  Get_Cart_Bnds(D.UI.rotBnds(z_pos,:,z_rot));
                
                % Patch
                D.UI.ptchRtBnds(z_rot, z_pos) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    D.UI.rotCol(z_rot,:), ...
                    'FaceAlpha', 0.1, ...
                    'EdgeAlpha', 0.5, ...
                    'LineWidth', 1, ...
                    'Visible', 'off', ...
                    'UserData', [z_rot, z_pos], ...
                    'ButtonDownFcn', {@Mouse_SelectRot}, ...
                    'HitTest', 'off', ...
                    'Parent', D.UI.axH(7));
                
                % Text
                str = ...
                    sprintf('%d%c', D.PAR.rotDistDeg(z_pos), char(176));
                D.UI.txtRtBnds(z_rot, z_pos) = text(...
                    mean(mean(xbnd)), mean(mean(ybnd)), ...
                    str, ...
                    'Color', [1, 1, 1], ...
                    'HorizontalAlignment', 'center', ...
                    'FontSize', D.UI.fontSzTxtSml(1), ...
                    'FontWeight', 'bold', ...
                    'Visible', 'off', ...
                    'UserData', [z_rot, z_pos], ...
                    'ButtonDownFcn', {@Mouse_SelectRot}, ...
                    'HitTest', 'off', ...
                    'Parent', D.UI.axH(7));
            end
        end
        
        %% SET UI OBJECTS
        
        % Set other plot features to visible
        Safe_Set(D.UI.linTrkH, 'Visible', 'on');
        Safe_Set(D.UI.linVelH, 'Visible', 'on');
        Patch_State(D.UI.ptchRewZoneBndsH(1,:), ...
            'ShowAll', D.UI.rotCol(1,:));
        Patch_State(D.UI.ptchRewZoneBndsH(2,:), ...
            'ShowPartial', D.UI.rotCol(2,:));
        Patch_State(D.UI.ptchStQ, 'Select');
        Safe_Set(D.UI.txtStQ, 'Visible', 'on');
        
        % Enlarge 0 deg marker
        Safe_Set(D.UI.mixFdNow(3,1), 'MarkerSize', 25);
        Safe_Set(D.UI.mixFdNow, 'Visible', 'on');
        uistack(reshape(D.UI.mixFdNow(1:2,:),1,[]),'bottom',1);
        
        % Make sure reward reset is active
        if D.PAR.sesCond ~= 'Manual_Training' %#ok<*STCMP>
            % Set reset patch to visible
            Patch_State(D.UI.ptchRewSendH(D.I.rot), ...
                'Select', D.UI.rotCol(D.I.rot,:));
        end
        
        % Show reward zone plot and children
        Safe_Set(D.UI.axZoneH, 'Visible', 'on');
        ch_h = Safe_Get(D.UI.axZoneH, 'Children')';
        for i = 1:length(ch_h)
            Safe_Set(ch_h{i}, 'Visible', 'on');
        end
        
        % Bulldoze default
        if D.PAR.sesCond ~= 'Manual_Training'
            val = find(ismember(D.UI.popBulldoze.String, [num2str(D.PAR.bullDel), ' sec']));
            Safe_Set(D.UI.popBulldoze, 'Value', val);
            Safe_Set(D.UI.toggBulldoze, 'Value', 1);
            Pop_Bulldoze();
        end
        
        % Cue setup
        if D.PAR.sesCond ~= 'Manual_Training' && ...
                (D.PAR.sesCue == 'Half' || D.PAR.sesCue == 'All')
            
            % Set cue button on
            Safe_Set(D.UI.toggDoCue, 'Value', 1);
            Togg_DoCue();
            
        end
        
        % Rotation buttons
        if D.PAR.sesCond == 'Rotation'
            
            % Set strings
            Safe_Set(D.UI.toggICR(1), ...
                'String', D.UI.btnICRstr{1});
            Safe_Set(D.UI.toggICR(2), ...
                'String', D.UI.btnICRstr{2});
            
            % Set rot bound patch HitTest
            Safe_Set(D.UI.ptchRtBnds, 'HitTest', 'on')
            
        end
        
    end

% --------------------------FORAGE TASK SETUP----------------------
    function[] = Forage_Task_Setup()
        
        % Set axis lims
        Safe_Set(D.UI.axH(5), ...
            'YDir', 'reverse', ...
            'XLim', [0,D.PAR.frgBins], ...
            'YLim', [0,D.PAR.frgBins]);
        
        % Load path file if exists and not flaged to recompute
        if ~D.DB.doForagePathCompute && ...
                exist(D.DIR.frgPath, 'file')
            load(D.DIR.frgPath);
            
            % Store in struct
            D.P.pathMat = double(path_mat); %#ok<NODEF>
            D.PAR.pathLengthArr = path_length_arr;  %#ok<NODEF>
            D.PAR.frgMask = forage_mask;  %#ok<NODEF>
            
        else
            
            % Deg bin var
            n_targs = 360/D.PAR.frgPathSpace;
            
            % Number of bins in forage area
            path_bins = round(D.PAR.frgBins*(D.UI.frgRad / D.UI.arnRad));
            
            % Width of path
            path_width = ((2*D.UI.frgRad*pi) * (D.PAR.frgPathWdt/360)) * ...
                (D.PAR.frgBins / (D.UI.arnRad*2));
            % Make sure width is odd
            if (mod(floor(path_width),2) == 1)
                path_width = floor(path_width);
            else
                path_width = ceil(path_width);
            end
            
            % Setup path mat
            D.P.pathMat = zeros(D.PAR.frgBins,D.PAR.frgBins,D.PAR.nPaths,n_targs);
            
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
            
            % Compute mat for each pos condition
            path_ang_arr = linspace(0,90,90/D.PAR.frgPathSpace+1);
            
            for i = 1:D.PAR.nPaths
                
                % Store angle
                deg = path_ang_arr(i);
                
                % Rotate
                mat_rot = imrotate(mat_p,deg,'bilinear','crop');
                
                % Cut and pad
                rind = floor(size(mat_eye,1)/2)-floor(path_bins/2):floor(size(mat_eye,1)/2)+floor(path_bins/2);
                cind = floor(size(mat_eye,2)/2):floor(size(mat_eye,1)/2)+path_bins-1;
                mat_rot = mat_rot(rind,cind);
                
                % Flip
                mat_rot = flip(mat_rot,2);
                mat_rot = flip(mat_rot,1);
                
                % Pad to full size
                pad_lng = round((D.PAR.frgBins - path_bins)/2);
                mat_rot = padarray(mat_rot, [pad_lng,pad_lng]);
                
                % Store in 4D mat
                for j = (1:n_targs)-1
                    D.P.pathMat(:,:,i,j+1) = imrotate(mat_rot,j*D.PAR.frgPathSpace,'bilinear','crop');
                end
                
            end
            
            % Get scaled up pixel vals
            rad_pxl = ceil(D.UI.frgRad*D.UI.cm2pxl)*4;
            lim_pxl = D.UI.vtRes*4;
            pad_pxl = round((lim_pxl-(rad_pxl*2))/2);
            
            % Make forage mask to remove values outside bounds
            [colNums, rowNums] = meshgrid(1:rad_pxl*2, 1:rad_pxl*2);
            D.PAR.frgMask = ...
                (rowNums - rad_pxl).^2 + (colNums - rad_pxl).^2 <= rad_pxl.^2;
            D.PAR.frgMask = ~padarray(D.PAR.frgMask, [pad_pxl, pad_pxl]);
            
            % Scale to occ mat
            D.PAR.frgMask = imresize(~D.PAR.frgMask, [D.PAR.frgBins,D.PAR.frgBins]);
            
            % Copy to local var
            forage_mask = D.PAR.frgMask; %#ok<NASGU>
            
            % Mask values outside circle
            mask = repmat(D.PAR.frgMask,[1,1,D.PAR.nPaths,n_targs]);
            D.P.pathMat = D.P.pathMat.*mask;
            
            % Nomalize
            D.P.pathMat(D.P.pathMat>0) = 1;
            
            % Plot path averages
            ih = imagesc(sum(D.P.pathMat(:,:,:,1),3), 'Parent', D.UI.axH(5));
            pause(1);
            delete(ih);
            
            % Plot accross pos and paths
            ih = imagesc(sum(D.P.pathMat(:,:,:,1),3), 'Parent', D.UI.axH(5));
            pause(1);
            delete(ih);
            for i = 1:D.PAR.nPaths
                ih = imagesc(D.P.pathMat(:,:,i,1), 'Parent', D.UI.axH(5));
                pause(0.1);
                delete(ih);
            end
            
            % Compute path lengths
            ang_arr = linspace(-45,45,D.PAR.nPaths);
            [x_path,y_path] = pol2cart(deg2rad(ang_arr), ones(1,D.PAR.nPaths));
            [x_str, y_str] = pol2cart(deg2rad(180), 1);
            path_length_arr = sqrt((abs(x_path-x_str)).^2 + abs((y_path-y_str)).^2);
            path_length_arr = path_length_arr * (D.UI.frgRad/max(path_length_arr));
            D.PAR.pathLengthArr = path_length_arr;
            
            % Store path info
            path_mat = single(D.P.pathMat); %#ok<NASGU>
            
            % Save path info
            fi_path = D.DIR.frgPath;
            save(fi_path, 'path_mat', 'path_length_arr', 'forage_mask');
            
            % Save second copy with date
            fi_name = sprintf('forage_path_%s.mat',datestr(now, 'yymmdd'));
            fi_path = regexprep(fi_path, 'forage_path.mat', fi_name);
            save(fi_path, 'path_mat', 'path_length_arr', 'forage_mask');
            
        end
        
        % Create target patches
        for z_targ = 1:length(D.PAR.frgTargDegArr)
            
            % random forage reward bounds
            [xbnd, ybnd] =  ...
                Get_Cart_Bnds(D.PAR.rewTargBnds(z_targ,:));
            D.UI.ptchRewTargBnds(z_targ) = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                [0,0,0], ...
                'ButtonDownFcn', {@Mouse_SelectRewTarg}, ...
                'EdgeColor', [0,0,0], ...
                'EdgeAlpha', 0.1, ...
                'FaceAlpha', 0.1, ...
                'LineWidth', 1, ...
                'UserData', z_targ, ...
                'Visible', 'off', ...
                'Parent', D.UI.axH(7));
        end
        
        % Set color lims
        Safe_Set(D.UI.axH(5), 'CLim', [0,1]);
        
        % Plot occ
        D.UI.imgFrgOcc = imagesc(D.P.frgOccMatScale+D.P.pathNowMat, ...
            'Parent', D.UI.axH(5));
        
        % Set other plot features to visible
        Safe_Set(D.UI.linFrgH, 'Visible', 'on');
        Patch_State(D.UI.ptchStQ, 'Select');
        Safe_Set(D.UI.txtStQ, 'Visible', 'on');
        
        % Get first target set to 180 deg from 'NE'
        D.I.targ_now = ...
            find(round(rad2deg(Rad_Sum(pi, mean(D.PAR.strQuadBnds)))) == D.PAR.frgTargDegArr);
        Patch_State(D.UI.ptchRewTargBnds(D.I.targ_now), ...
            'Active', D.UI.activeCol);
        
    end

% ----------------------------RAT IN CHECK-------------------------
    function Rat_In_Check()
        
        % Bypass if button pressed
        if get(D.UI.btnStart, 'Value') == 0
            
            % Have to press button for forage task
            if D.PAR.sesTask == 'Forage'
                return
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
            
        else
            % Set inbound time
            inbndTim = 0;
        end
        
        % Post NLX event: rat in
        Send_NLX_Cmd(D.NLX.rat_in_evt);
        
        % Save time local time
        D.T.task_str = Sec_DT(now);
        
        % Save Cheetah time
        [pass, ts_string] = Send_NLX_Cmd('-GetTimestamp');
        if pass == 1
            D.T.task_str_nlx = int64(str2double(ts_string)) - D.T.poll_str_nlx;
        else
            D.T.task_str_nlx = 0;
        end
        
        % Start tracking lap times
        D.T.lap_str = Sec_DT(now);
        
        % Tell CS rat is in
        if D.PAR.sesTask == 'Track'
            % Rat on track
            Send_CS_Com('I', 1);
        else
            % Rat on forage platform
            Send_CS_Com('I', 0);
        end
        
        % Set flag
        D.F.rat_in = true;
        
        % Set patch to nonvisible
        Patch_State(D.UI.ptchStQ, 'Hide');
        
        % Reset start string
        Safe_Set(D.UI.txtStQ, ...
            'String', 'START', ...
            'Color',  D.UI.enabledCol);
        
        % Clear VT data
        Btn_ClrVT();
        
        % Reinitialize
        D.T.strqd_inbnd_t1 = 0;
        
        % Disable button
        set(D.UI.btnStart, 'Visible', 'off')
        
        % Log/print
        if inbndTim > 0
            Console_Write(sprintf('[Rat_In_Check] FINISHED: Rat In Check: dt_occ=%0.2fsec', ...
                inbndTim));
        else
            Console_Write('[Rat_In_Check] FINISHED: Rat In Check: Started Manually');
        end
        
    end

% -----------------------------TEST SETUP--------------------------
    function[was_ran] = Test_Setup()
        
        %% BAIL IF NOT TEST RUN
        
        % Initialize output
        was_ran = false;
        
        % Bail
        if ~D.DB.isTestRun
            return
        end
        
        % Set flag
        was_ran = true;
        
        %% SETUP SPECIFIC TEST
        
        % SIMULATED RAT TEST
        if D.DB.t1_doSimRatTest
            
            % Initialize variables
            D.DB.SIM.XY = [NaN,NaN];
            D.DB.SIM.RadLast = NaN;
            D.DB.SIM.RohLast = NaN;
            D.DB.SIM.RunRad = NaN;
            D.DB.SIM.TargAng = NaN;
            D.DB.SIM.VelLast = NaN;
            D.DB.SIM.TSStart = NaN;
            D.DB.SIM.TSLast = NaN;
            D.DB.SIM.SwayDir = -1;
            D.DB.SIM.SldVelLast = NaN;
            D.DB.SIM.t_resumeRun = 0;
            D.DB.isTestStarted = false;
            
            % Set streaming flag
            D.F.vt_rat_streaming = true;
            
            % Create rat velocity slider object
            pos_sld = [D.UI.ses_inf_pan_pos(1)-0.185, 1-0.025, 0.175, 0.02];
            vel_rng = [-25,100];
            step_rng = [0.01,0.1]*(100/diff(vel_rng));
            D.UI.sldSimVel = uicontrol('Style', 'slider',...
                'Parent',D.UI.tabICR, ...
                'Units', 'Normalized', ...
                'Min',vel_rng(1),'Max',vel_rng(2), ...
                'Value',D.DB.SIM.VelStart,...
                'SliderStep', step_rng, ...
                'Visible', 'off',...
                'Enable', 'off',...
                'Position', pos_sld);
            
            % Vel text
            pos_txt = [pos_sld(1)-0.03, pos_sld(2), 0.03, pos_sld(4)];
            D.UI.txtSimVel = uicontrol('Style', 'text',...
                'Parent',D.UI.tabICR, ...
                'Units', 'Normalized', ...
                'BackgroundColor', D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontSize', D.UI.fontSzTxtLrg(1), ...
                'FontWeight', 'Bold', ...
                'String','100',...
                'Visible', 'off',...
                'Position', pos_txt);
            
            % Axes ticks
            pos_txt(1) = pos_sld(1)-0.005;
            pos_txt(2) = pos_txt(2)-pos_txt(4);
            pos_txt(3) = pos_txt(3)*0.75;
            D.UI.txtVelTick(1) = copyobj(D.UI.txtSimVel, D.UI.tabICR);
            set(D.UI.txtVelTick(1), ...
                'Position', pos_txt, ...
                'FontSize', 8, ...
                'String', num2str(vel_rng(1)))
            
            pos_txt(1) = pos_sld(1) + 0.035;
            D.UI.txtVelTick(2) = copyobj(D.UI.txtSimVel, D.UI.tabICR);
            set(D.UI.txtVelTick(2), ...
                'Position', pos_txt, ...
                'FontSize', 8, ...
                'String', num2str(0))
            
            pos_txt(1) = pos_sld(1) + (pos_sld(3)-pos_sld(3)*0.1);
            D.UI.txtVelTick(3) = copyobj(D.UI.txtSimVel, D.UI.tabICR);
            set(D.UI.txtVelTick(3), ...
                'Position', pos_txt, ...
                'FontSize', 8, ...
                'String', num2str(vel_rng(2)))
            
        end
        
        % PID CALIBRATION
        if D.DB.t2_doPidCalibrationTest
            
            % Set flag to start pid
            D.DB.isTestStarted = false;
            
            % Change implanted flag
            D.DB.Implanted = false;
            % Change task
            D.DB.Session_Task = 'Track';
            
        end
        
        % VT CALIBRATION
        if D.DB.t3_doVTCalibrationTest
            
            % Initialize variables
            D.DB.isTestStarted = false;
            D.DB.CALVT.RunEnd = NaN;
            D.DB.CALVT.VTHist = NaN(1, ceil(D.DB.CALVT.RunDur*60*30*2*1.1));
            D.DB.CALVT.PosHist = NaN(2, ceil(D.DB.CALVT.RunDur*60*30*1.1));
            D.DB.CALVT.RecCnt = 0;
            
            % Change implanted flag
            D.DB.Implanted = false;
            % Change condition flag
            D.DB.Session_Condition = 'Manual_Training';
            % Change task
            D.DB.Session_Task = 'Track';
            
        end
        
        % HALT ERROR TEST
        if D.DB.t4_doHaltErrorTest
            
            % Initialize variables
            D.DB.HALT.Cnt = D.DB.HALT.StepTrials;
            D.DB.HALT.Dur = 5; % (sec)
            D.DB.HALT.error_str = '';
            D.DB.HALT.t_halted = Sec_DT(now);
            D.DB.HALT.NowStep = 0;
            D.DB.HALT.NowVel = 0;
            D.DB.HALT.IsHalted = true;
            D.DB.HALT.SendPos = 0;
            
            % Change implanted flag
            D.DB.Implanted = false;
            % Change task
            D.DB.Session_Task = 'Track';
            
        end
        
        % WALL IMAGE IR TIMING TEST
        if D.DB.t5_doWallIRTimingTest
            
            % Notes:
            %   Connect LED to DigitalLynx port 2 pin 0
            %   Turn off CheetahDue IR blink (might have program do it...)
            
            % Initialize variables
            D.DB.isTestStarted = false;
            % LED on event string
            D.DB.WALIR.NLXevtStr = 'TTL_LED_On';
            % PT TTL strings
            D.DB.WALIR.PTttlStr = {D.NLX.north_str, D.NLX.west_str, D.NLX.south_str, D.NLX.east_str};
            % LED pulse command
            D.DB.WALIR.LEDttlStr = ['-DigitalIOTtlPulse ', D.NLX.DevTTL, ' 2 0 High'];
            % Next pulse/image change time
            D.DB.WALIR.t_next = 0;
            % Curent event number [sent, evt1 rcvd, evt2 rcvd]
            D.DB.WALIR.cntEvent = [0, 0, 0];
            % Curent sensor number
            D.DB.WALIR.cntSensor = 0;
            % Send time
            D.DB.WALIR.t_send = NaN(D.DB.WALIR.imgTrials,4);
            % Trigger time
            D.DB.WALIR.t_trig = NaN(D.DB.WALIR.imgTrials,4);
            % Flag image test done
            D.DB.WALIR.isImgTestDone = false;
            % Flag ir test done
            D.DB.WALIR.isIRTestDone = false;
            
            % Prevent start move
            D.F.first_move_sent = true;
            
            % Change implanted flag
            D.DB.Implanted = false;
            % Change condition and Task
            D.DB.Session_Condition = 'Behavior_Training';
            % Change task
            D.DB.Session_Task = 'Track';
            
        end
        
        % SYNC IR TIMING TEST
        if D.DB.t6_doSyncIRTimingTest
            
            % Notes:
            %   Connect robot Reward LED red and ground to DigitalLynx port 2 pin 0
            %   Turn off CheetahDue IR blink (might have program do it...)
            
            % Initialize variables
            D.DB.isTestStarted = false;
            % Robot on event string
            D.DB.SYNCIR.NLXevtStr = 'TTL_IR_Detected';
            % Curent event number [send, ttl ard, ttl rob]
            D.DB.SYNCIR.cntEvent = [0, 0, 0];
            % Next pulse time
            D.DB.SYNCIR.t_next = 0;
            % Pulse time
            D.DB.SYNCIR.t_pulse = NaN(D.DB.SYNCIR.pulseTrials,1);
            % TTL trigger time
            D.DB.SYNCIR.t_trig = NaN(D.DB.SYNCIR.pulseTrials,1);
            
            % Prevent start move
            D.F.first_move_sent = true;
            
            % Change implanted flag
            D.DB.Implanted = false;
            % Change condition and Task
            D.DB.Session_Condition = 'Behavior_Training';
            % Change task
            D.DB.Session_Task = 'Track';
            
        end
        
        % HARDWARE TEST
        if D.DB.t7_doRobotHardwareTest
            
            % Notes:
            %   Nothing to set here
            
        end
        
        % CUBE VCC TEST
        if D.DB.t8_doCubeBatteryTest
            
            % Initialize variables
            D.DB.isTestStarted = false;
            % Flag percentages printed
            D.DB.CVCC.flagPrc = false(1, length(D.DB.CVCC.prcSteps));
            % Check start time
            D.DB.CVCC.t_start = [];
            % Store time at change
            D.DB.CVCC.dt_step = NaN(1, length(D.DB.CVCC.prcSteps) + 1);
            % Vcc at start
            D.DB.CVCC.vcc_step = NaN(1, length(D.DB.CVCC.prcSteps) + 1);
            % Figure and axis
            D.DB.CVCC.fg = figure(...
                'Visible', 'off', ...
                'Position', FIGH.Position);
            D.DB.CVCC.ax = axes( ...
                'YLim', [0,100], ...
                'XTick', 0:60, ...
                'XGrid', 'on',...
                'YGrid', 'on', ...
                'XMinorGrid', 'on',...
                'YMinorGrid', 'on', ...
                'GridColor', [0.1,0.1,0.1], ...
                'MinorGridColor', [0.2,0.2,0.2]);
            hold on;
            D.DB.CVCC.ax.YLabel.String = 'Vcc (%)';
            D.DB.CVCC.ax.XLabel.String = 'DT Start (min)';
            
            % Change implanted flag
            D.DB.Implanted = true;
            % Change condition and Task
            D.DB.Session_Condition = 'Implant_Training';
        end
        
        % Log/print
        Console_Write('[Test_Setup] FINISHED: Test Run Setup');
        
        %% AUTOLOAD RAT DATA
        
        if DOAUTOLOAD
            
            % Set and run ses type pop
            Safe_Set(D.UI.popType, 'Value', ...
                find(ismember(D.UI.popType.String,  D.DB.Session_Type)));
            Safe_Set(D.UI.toggType, 'Value', 1);
            Togg_Type(D.UI.toggType);
            
            % Get rat table ind
            ratInd = ...
                find(ismember(D.SS_IO_1.Properties.RowNames, D.DB.ratLab));
            
            % Set Rat
            val = 1 + find(cell2mat(cellfun(@(x) strcmp(x(1:4), D.DB.ratLab(2:end)), D.UI.popRat.String(2:end), 'uni', false)));
            Safe_Set(D.UI.popRat, 'Value', val);
            
            % Set Human
            D.SS_IO_1.Human(ratInd) = {'AWL'};
            
            % Remaining vars
            if D.PAR.sesType == 'ICR_Session'
                
                % Set session condition
                D.SS_IO_1.Session_Condition(ratInd) = D.DB.Session_Condition;
                
                % Set implanted flag
                D.SS_IO_1.Implanted(ratInd) = D.DB.Implanted;
                
                % Set task
                D.SS_IO_1.Session_Task(ratInd) = D.DB.Session_Task;
                
                % Set feeder condition
                D.SS_IO_1.Feeder_Condition(ratInd) = D.DB.Feeder_Condition;
                
                % Set reward delay
                D.SS_IO_1.Reward_Delay(ratInd)= D.DB.Reward_Delay;
                
                % Set cue condition
                D.SS_IO_1.Cue_Condition(ratInd) = D.DB.Cue_Condition;
                
                % Set sound conditions
                D.SS_IO_1.Sound_Conditions(ratInd,1:2) = D.DB.Sound_Conditions;
                
                % Get session number
                var_ind = ...
                    ismember(D.SS_IO_1.Properties.VariableNames, ...
                    ['Session_',char(D.DB.Session_Condition)]);
                if strcmp(D.DB.Session_Condition, 'Rotation')
                    col_ind = 1;
                else
                    col_ind = ismember([{'Track'},{'Forage'}], D.DB.Session_Task);
                end
                ses_next = ...
                    D.SS_IO_1{ratInd, var_ind}(col_ind) + 1;
                % Get session total
                ses_all_next = D.SS_IO_1{ratInd, 'Session_Manual_Training'}(col_ind) + ...;
                    D.SS_IO_1{ratInd, 'Session_Behavior_Training'}(col_ind) + ...
                    D.SS_IO_1{ratInd, 'Session_Implant_Training'}(col_ind) + ...
                    D.SS_IO_1{ratInd, 'Session_Rotation'} + 1;
                
                % Set start quad
                D.SS_IO_1.Start_Quadrant{ratInd}(ses_all_next) = D.DB.Start_Quadrant;
                
                % Set rot dir
                D.SS_IO_1.Rotation_Direction{ratInd}(ses_next) = D.DB.Rotation_Direction;
                
                % Set rot pos
                for i = 1:length(D.SS_IO_1.Rotation_Positions{ratInd}(ses_next,:))
                    D.SS_IO_1.Rotation_Positions{ratInd}(ses_next,i) = num2str(D.DB.Rotation_Positions(i));
                end
                
            end
            
            % Run PopRat
            Pop_Rat();
            
            % TRIGGER SETUP DONE
            Safe_Set(D.UI.toggSetupDone, 'Value', 1);
            Togg_SetupDone();
            
        end
        
    end






%% ============================ ONGOING FUNCTIONS =========================

% ---------------------------GET NLX VT----------------------------
    function VT_Get(fld)
        
        % Bail if not connected
        if ~NlxAreWeConnected() == 1
            return
        end
        
        % Bail if not streaming
        if (strcmp(fld, 'Rat') && ~D.F.vt_rat_streaming ) || ...
                (strcmp(fld, 'Rob') && ~D.F.vt_rob_streaming )
            return
        end
        
        % Include head direction for implant sessions
        if strcmp(fld, 'Rat')
            
            % Get vt data including head direction
            [~, D.P.(fld).vtTS, D.P.(fld).vtPos, D.P.(fld).vtHD, D.P.(fld).vtNRecs, ~] = ...
                NlxGetNewVTData(D.NLX.vt_rat_ent);
            
        elseif strcmp(fld, 'Rob')
            
            % Dont include head direction
            [~, D.P.(fld).vtTS, D.P.(fld).vtPos, ~, D.P.(fld).vtNRecs, ~] = ...
                NlxGetNewVTData(D.NLX.vt_rob_ent);
            
        end
        
        % Center ts at poll start
        if ~isempty(D.P.(fld).vtTS)
            D.P.(fld).vtTS = D.P.(fld).vtTS - D.T.poll_str_nlx;
        end
        
        % Check if this is first record
        if D.P.(fld).vtNRecs > 0 && D.P.(fld).cnt_vtRec == 0
            % Log/print
            Console_Write(sprintf('[VT_Get] FIRST %s VT RECORD: x=%d y=%d ts=%d', ...
                upper(fld), D.P.(fld).vtTS(1), D.P.(fld).vtPos(1), D.P.(fld).vtPos(2)));
        end
        
        % Add to count
        D.P.(fld).cnt_vtRec = D.P.(fld).cnt_vtRec + D.P.(fld).vtNRecs;
        
    end

% -------------------------PROCESS NLX VT--------------------------
    function VT_Proc(fld)
        
        if D.P.(fld).vtNRecs > 0
            
            %% PROCESS NEW DATA
            
            % Convert to [r = samp, c = dim]
            xy_pos = reshape(double(D.P.(fld).vtPos),2,[])';
            recs = D.P.(fld).vtNRecs;
            
            % Convert ts to double
            ts = double(D.P.(fld).vtTS');
            
            % Convert to normalized polar vals
            [rad, roh] = VT_2_Pol(xy_pos);
            
            % Exclude outlyer values > || < track bounds plus 5 cm
            if ~(D.PAR.sesTask == 'Forage' && strcmp(fld, 'Rat'))
                exc_1 = roh > D.P.trackRohCut(2) | roh < D.P.trackRohCut(1);
            else
                exc_1 = roh > D.P.frgRohBnd(2) | roh < 0;
            end
            
            % Exclude velocity values based on each preceding record
            vel_1 = Rad_Vel([D.P.(fld).radLast; rad], [D.P.(fld).radLast; ts]);
            exc_2 = vel_1 > D.P.velMax;
            
            % Exclude values based on last used record
            vel_2 = Rad_Vel(...
                reshape([repmat(D.P.(fld).radLast, 1, length(rad)); rad'], [], 1), ...
                reshape([repmat(D.P.(fld).tsLast, 1, length(ts)); ts'], [], 1));
            exc_3 = vel_2(1:2:end) > D.P.velMax;
            
            % Do not use exc_2/3 if "Forage" run
            if  (D.PAR.sesTask == 'Forage' && strcmp(fld, 'Rat'))
                exc_2 = false(size(exc_2,1), 1);
                exc_3 = false(size(exc_3,1), 1);
            end
            
            % Do not use exc_3 if more than reward duration of unused data
            if  Sec_DT(now) - D.T.(fld).last_pos_update > (D.PAR.rewDur+100)/1000
                exc_3 = false(size(exc_3,1), 1);
            end
            
            % Combine exclusion criteria
            exc_ind = exc_1 | exc_2 | exc_3;
            
            % Set bad recs to empty
            rad(exc_ind) = [];
            roh(exc_ind) = [];
            ts(exc_ind) = [];
            
            % Recalculate cartisian values
            [x, y] = pol2cart(rad,roh);
            
            % Convert back to vt pixels
            x =  x.*D.PAR.R + D.PAR.XC;
            y =  y.*D.PAR.R + D.PAR.YC;
            
            % Check if any data kept
            if all(exc_ind)
                
                % Do not plot any data this loop
                D.F.(fld).new_pos_data = false;
                D.F.(fld).new_vel_data = false;
                
                % Set vars to NaN
                D.P.(fld).x = NaN;
                D.P.(fld).y = NaN;
                D.P.(fld).rad = NaN;
                D.P.(fld).ts = NaN;
                D.P.(fld).recs = NaN;
                
                % Exit function
                return
                
            else
                % Set new data flag
                D.F.(fld).new_pos_data = true;
                
                % Keep track of updates
                D.T.(fld).last_pos_update = Sec_DT(now);
                
                % Save last usable rad and ts value
                D.P.(fld).radLast = rad(end);
                D.P.(fld).tsLast = ts(end);
                
                % Save last
                D.P.(fld).xLast = x(end);
                D.P.(fld).yLast = y(end);
            end
            
            % Store vars for later use
            D.P.(fld).x = x;
            D.P.(fld).y = y;
            D.P.(fld).rad = rad;
            D.P.(fld).roh = roh;
            D.P.(fld).ts = ts;
            D.P.(fld).recs = recs;
            
            % Add to conplete data
            if strcmp(fld, 'Rat')
                
                % Get store pos indeces
                ind_set_str = D.P.Rat.posAll.indAll(2) + 1;
                ind_set_end = ind_set_str+length(x)-1;
                
                % Store indeces
                D.P.(fld).posAll.indAll(2) = ind_set_end;
                D.P.(fld).posAll.indLap(2) = ind_set_end;
                
                % Store cart pos data
                D.P.(fld).posAll.Cart(ind_set_str:ind_set_end, 1) = single(x);
                D.P.(fld).posAll.Cart(ind_set_str:ind_set_end, 2) = single(y);
                
                % Store rad data
                D.P.(fld).posAll.Pol(ind_set_str:ind_set_end, 1) = single(rad);
                D.P.(fld).posAll.Pol(ind_set_str:ind_set_end, 2) = single(roh);
                
                % Store ts
                D.P.(fld).posAll.TS(ind_set_str:ind_set_end) = ts;
            end
            
            %% TRANSFORM HD
            
            % Run only for rat data
            if  D.F.stream_hd && ...
                    strcmp(fld, 'Rat') && ...
                    D.F.Rat.new_pos_data
                
                % Copy over data
                D.P.(fld).hd_deg = D.P.(fld).vtHD';
                
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
                
                % Set flag
                D.F.Rat.new_hd_data = any(~isnan(D.P.Rat.hd_rad));
                
            end
            
            %% HANDLE RAT FORAGE DATA
            
            % Bail if processing rat forrage data
            if D.PAR.sesTask == 'Forage' && strcmp(fld, 'Rat')
                
                % Compute inbound occ bin counts
                occ_now = histcounts2(D.P.(fld).y,D.P.(fld).x,D.PAR.frgBinEdgeY,D.PAR.frgBinEdgeX);
                occ_now = flip(occ_now,1);
                
                % Store raw values
                D.P.frgOccMatRaw = D.P.frgOccMatRaw + occ_now;
                
                % Compute and store scaled occ
                non_zer_occ = D.P.frgOccMatRaw(D.P.frgOccMatRaw(:) > 0);
                scale = prctile(non_zer_occ,95);
                if scale == 0
                    scale = 1;
                end
                D.P.frgOccMatScale = D.P.frgOccMatRaw/scale;
                
                % Compute binary value
                D.P.frgOccMatBinary = D.P.frgOccMatBinary  + occ_now;
                D.P.frgOccMatBinary = ceil(D.P.frgOccMatBinary/max(D.P.frgOccMatBinary(:)));
                
                return
            end
            
            %% GET SETPOINT, FEEDER POS AND GUARD POS
            
            if strcmp(fld, 'Rob')
                
                % Get setpoint in radians
                D.P.Rob.setRad = D.P.(fld).radLast - D.PAR.setPointRad;
                if D.P.Rob.setRad < 0
                    D.P.Rob.setRad = 2*pi + D.P.Rob.setRad;
                end
                
                % Get feeder pos
                D.P.Rob.feedRad = D.P.(fld).radLast - D.PAR.feedDistRad;
                if D.P.Rob.feedRad < 0
                    D.P.Rob.feedRad = 2*pi + D.P.Rob.feedRad;
                end
                
                % Get guard pos
                D.P.Rob.guardRad = D.P.(fld).radLast - D.PAR.guardDistRad;
                if D.P.Rob.guardRad < 0
                    D.P.Rob.guardRad = 2*pi + D.P.Rob.guardRad;
                end
                
                % Get butt pos
                D.P.Rob.buttRad = D.P.(fld).radLast + D.PAR.buttDistRad;
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
                if Rad_Diff(D.P.(fld).velRad, D.P.Rat.radLast, 'min') > deg2rad(20)
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
                D.P.(fld).ts_samples = double(ts(1:nDat));
                % Combine new and old vals
            else
                % combine rad
                old_dat = D.P.(fld).rad_samples;
                D.P.(fld).rad_samples(end-nDat+1:end) = rad;
                D.P.(fld).rad_samples(1:end-nDat) = ...
                    old_dat(nDat+1:end);
                % combine ts
                old_dat = D.P.(fld).ts_samples;
                D.P.(fld).ts_samples(end-nDat+1:end) = double(ts);
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
                vel = Rad_Vel(rad_arr, ts_arr);
            end
            
            % Get next vel hisory ind
            ind_set_str = D.P.(fld).velAll.indAll(2) + 1;
            
            % Store new ind
            D.P.(fld).velAll.indAll(2) = ind_set_str;
            D.P.(fld).velAll.indLap(2) = ind_set_str;
            
            % Store TS
            D.P.(fld).velAll.TS(ind_set_str) = ts(end);
            
            % Check if we have usable data
            if isempty(vel) || isempty(D.P.(fld).velRad)
                
                % Set not to plot this vel
                D.F.(fld).new_vel_data = false;
                
                % Set to NaN to avoid jumps in plot
                D.P.(fld).velAll.Cart(ind_set_str,:) = single(NaN);
                D.P.(fld).velAll.Pol(ind_set_str,:) = single(NaN);
            else
                
                % Set to plot this vel if 'Track' session
                D.F.(fld).new_vel_data = D.PAR.sesTask == 'Track';
                
                % Save average
                D.P.(fld).vel = nanmean(vel);
                
                % Store vel max
                if D.P.(fld).vel < 150
                    D.P.(fld).vel_max_lap = max(D.P.(fld).vel_max_lap, D.P.(fld).vel);
                    D.P.(fld).vel_max_all = max(D.P.(fld).vel_max_lap, D.P.(fld).vel_max_all);
                end
                
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
                    [x, y] = pol2cart(D.P.(fld).velRad, velRoh);
                    x =  x.*D.PAR.R + D.PAR.XC;
                    y =  y.*D.PAR.R + D.PAR.YC;
                    
                    % Store values
                    D.P.(fld).velAll.Cart(ind_set_str, 1) = single(x);
                    D.P.(fld).velAll.Cart(ind_set_str, 1) = single(y);
                else
                    
                    % Set to NaN because rad diff to large
                    D.P.(fld).velAll.Cart(ind_set_str, :) = single(NaN);
                    D.P.(fld).velRad = NaN;
                    velRoh = NaN;
                    
                    % Set not to plot this vel
                    D.F.(fld).new_vel_data = false;
                end
                
                % Save pol history
                D.P.(fld).velAll.Pol(ind_set_str, 1) = single(D.P.(fld).velRad);
                D.P.(fld).velAll.Pol(ind_set_str, 2) = single(velRoh);
                
            end
            
        end
        
    end

% ----------------------PLOT CURRENT POSITION----------------------
    function VT_Plot_New()
        
        % BAIL IF SETUP NOT FINISHED
        if ~D.F.ses_setup_done
            return
        end
        
        % Store time
        t_now = Sec_DT(now);
        
        %% ROBOT POS
        
        if D.F.Rob.new_pos_data
            
            % Plot rob patch
            [xbnd, ybnd] =  Get_Cart_Bnds( [D.P.Rob.guardRad, D.P.Rob.buttRad], D.P.trackRohBnd);
            x = [xbnd(1,:),fliplr(xbnd(2,:))];
            y = [ybnd(1,:),fliplr(ybnd(2,:))];
            if ~isgraphics(D.UI.Rob.ptchPosNowH)
                D.UI.Rob.ptchPosNowH = ...
                    patch(x, y, ...
                    D.UI.robNowCol, ...
                    'EdgeColor', D.UI.robNowCol, ...
                    'LineWidth', 2, ...
                    'FaceAlpha',0.75, ...
                    'HitTest', 'off', ...
                    'Parent',D.UI.axH(4));
            else
                Safe_Set(D.UI.Rob.ptchPosNowH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Plot current rob tracker pos
            x = D.P.Rob.xLast;
            y = D.P.Rob.yLast;
            if ~isgraphics(D.UI.Rob.mrkPosNowH)
                D.UI.Rob.mrkPosNowH = ...
                    line(x, y, ...
                    'LineStyle', 'none', ...
                    'Marker', 'o', ...
                    'MarkerFaceColor', D.UI.robNowCol, ...
                    'MarkerEdgeColor', D.UI.robNowCol, ...
                    'MarkerSize', 10, ...
                    'HitTest', 'off', ...
                    'Parent', D.UI.axH(4));
            else
                Safe_Set(D.UI.Rob.mrkPosNowH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Plot rob arm
            [xbnd, ybnd] =  Get_Cart_Bnds([D.P.Rob.feedRad, D.P.Rob.buttRad], D.P.trackRohBnd);
            x = xbnd(1,:);
            y = ybnd(1,:);
            if ~isgraphics(D.UI.Rob.linPosNowArmH)
                D.UI.Rob.linPosNowArmH = ...
                    line(x, y, ...
                    'Color', D.UI.robNowCol, ...
                    'LineWidth', 5, ...
                    'HitTest', 'off', ...
                    'Parent',D.UI.axH(4));
                uistack(D.UI.Rob.linPosNowArmH, 'top');
            else
                Safe_Set(D.UI.Rob.linPosNowArmH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Plot guard pos
            [xbnd, ybnd] =  Get_Cart_Bnds(D.P.Rob.guardRad, D.P.trackRohBnd);
            x = xbnd;
            y = ybnd;
            if ~isgraphics(D.UI.Rob.linPosNowGuardH)
                D.UI.Rob.linPosNowGuardH = ...
                    line(x, y, ...
                    'Color', D.UI.guardPosCol, ...
                    'LineWidth', D.UI.guardPosLineWidth, ...
                    'HitTest', 'off', ...
                    'Parent',D.UI.axH(4));
            else
                Safe_Set(D.UI.Rob.linPosNowGuardH, ...
                    'Color', D.UI.guardPosCol, ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Plot set pos
            [xbnd, ybnd] =  Get_Cart_Bnds(D.P.Rob.setRad, D.P.trackRohBnd);
            x = xbnd;
            y = ybnd;
            if ~isgraphics(D.UI.Rob.linPosNowSetH)
                D.UI.Rob.linPosNowSetH = ...
                    line(x, y, ...
                    'Color', D.UI.setPosCol, ...
                    'LineWidth', D.UI.setPosLineWidth, ...
                    'HitTest', 'off', ...
                    'Parent',D.UI.axH(4));
            else
                Safe_Set(D.UI.Rob.linPosNowSetH, ...
                    'Color', D.UI.setPosCol, ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Plot feeder pos feed in cond color
            [xbnd, ybnd] =  Get_Cart_Bnds(D.P.Rob.feedRad, D.P.trackRohBnd);
            x = xbnd(1);
            y = ybnd(1);
            if ~isgraphics(D.UI.Rob.mrkPosNowDishH)
                D.UI.Rob.mrkPosNowDishH = ...
                    line(x, y, ...
                    'LineStyle', 'none', ...
                    'Marker', 'o', ...
                    'LineWidth', 2, ...
                    'MarkerSize', D.UI.feedPosMarkSize, ...
                    'MarkerFaceColor', D.UI.feedPosCol, ...
                    'MarkerEdgeColor', D.UI.robNowCol, ...
                    'HitTest', 'off', ...
                    'Parent', D.UI.axH(4));
                uistack(D.UI.Rob.mrkPosNowDishH, 'top');
            else
                Safe_Set(D.UI.Rob.mrkPosNowDishH, ...
                    'MarkerSize', D.UI.feedPosMarkSize, ...
                    'MarkerFaceColor', D.UI.feedPosCol, ...
                    'XData', x, ...
                    'YData', y);
            end
            
        end
        
        %% RAT POS
        
        if D.F.Rat.new_pos_data
            
            % Plot all VT data for this lap
            ind_get_str = D.P.Rat.posAll.indLap(1);
            ind_get_end = D.P.Rat.posAll.indLap(2);
            x = D.P.Rat.posAll.Cart( ind_get_str: ind_get_end,1);
            y = D.P.Rat.posAll.Cart( ind_get_str: ind_get_end,2);
            if ~isgraphics(D.UI.Rat.mrkPosLapH)
                D.UI.Rat.mrkPosLapH = ...
                    line(x, y, ...
                    'LineStyle', 'none', ...
                    'Marker', '.', ...
                    'MarkerFaceColor', D.UI.ratPosAllCol, ...
                    'MarkerEdgeColor', D.UI.ratPosAllCol, ...
                    'MarkerSize', 6, ...
                    'HitTest', 'off', ...
                    'Parent', D.UI.axH(3));
            else
                Safe_Set(D.UI.Rat.mrkPosLapH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Plot current rat position with larger marker
            x = D.P.Rat.xLast;
            y = D.P.Rat.yLast;
            if ~isgraphics(D.UI.Rat.mrkPosNowH)
                D.UI.Rat.mrkPosNowH = ...
                    line(x, y, ...
                    'LineStyle', 'none', ...
                    'Marker', 'o', ...
                    'MarkerFaceColor', D.UI.ratNowCol, ...
                    'MarkerEdgeColor', [0, 0, 0], ...
                    'MarkerSize', 10, ...
                    'HitTest', 'off', ...
                    'Parent', D.UI.axH(3));
            else
                Safe_Set(D.UI.Rat.mrkPosNowH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Update forage occ plot
            if D.PAR.sesTask == 'Forage'
                cdat = D.P.frgOccMatScale+D.P.pathNowMat;
                if ~isgraphics(D.UI.imgFrgOcc)
                    D.UI.imgFrgOcc = imagesc(cdat, ...
                        'Parent', D.UI.axH(5));
                else
                    Safe_Set(D.UI.imgFrgOcc, 'CData', cdat);
                end
            end
            
        end
        
        %% VELOCITY
        
        % ROB VELOCITY
        if D.F.Rob.new_vel_data
            
            % Get non-zero vals to plot
            ind_get_str = D.P.Rob.velAll.indLap(1);
            ind_get_end = D.P.Rob.velAll.indLap(2);
            x = D.P.Rob.velAll.Cart(ind_get_str:ind_get_end, 1);
            y = D.P.Rob.velAll.Cart(ind_get_str:ind_get_end, 2);
            % Plot
            if ~isgraphics(D.UI.Rob.linVelLapH)
                D.UI.Rob.linVelLapH = ...
                    line(x, y, ...
                    'LineStyle', '-', ...
                    'Color', D.UI.robNowCol, ...
                    'LineWidth', 2, ...
                    'Parent', D.UI.axH(3));
            else
                Safe_Set(D.UI.Rob.linVelLapH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
        end
        
        % RAT VELOCITY
        if D.F.Rat.new_vel_data
            
            % Get non-zero vals to plot
            ind_get_str = D.P.Rat.velAll.indLap(1);
            ind_get_end = D.P.Rat.velAll.indLap(2);
            x = D.P.Rat.velAll.Cart(ind_get_str:ind_get_end, 1);
            y = D.P.Rat.velAll.Cart(ind_get_str:ind_get_end, 2);
            % Plot
            if ~isgraphics(D.UI.Rat.linVelLapH)
                D.UI.Rat.linVelLapH = ...
                    line(x, y, ...
                    'LineStyle', '-', ...
                    'Color', D.UI.ratNowCol, ...
                    'LineWidth', 2, ...
                    'Parent', D.UI.axH(3));
            else
                Safe_Set(D.UI.Rat.linVelLapH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
        end
        
        %% HEADING
        
        % Plot arrow
        if D.F.Rat.new_hd_data && D.F.Rat.new_pos_data
            
            % Get current heading
            hd_deg = rad2deg(D.P.Rat.hd_rad(end));
            
            % Calculate coordinates for line
            % x start/end
            xs = D.P.Rat.xLast;
            xe = cos(deg2rad(hd_deg))*10 + xs;
            x = [xs, xe, ...
                cos(deg2rad(hd_deg-90))*2 + xe, ...
                cos(deg2rad(hd_deg))*4 + xe, ...
                cos(deg2rad(hd_deg+90))*2 + xe, ...
                xe];
            % y start/end
            ys = D.P.Rat.yLast;
            ye = sin(deg2rad(hd_deg))*10 + ys;
            y = [ys, ye, ...
                sin(deg2rad(hd_deg-90))*2 + ye, ...
                sin(deg2rad(hd_deg))*4 + ye, ...
                sin(deg2rad(hd_deg+90))*2 + ye, ...
                ye];
            
            % Plot thick backround line
            if ~isgraphics(D.UI.Rat.mrkHeadNowH(1))
                D.UI.Rat.mrkHeadNowH(1) = ...
                    line(x, y, ...
                    'LineStyle', '-', ...
                    'Color', [0.5, 0.5, 0.5], ...
                    'LineWidth', 3, ...
                    'Parent', D.UI.axH(4));
            else
                Safe_Set(D.UI.Rat.mrkHeadNowH(1), ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Plot thin foreground line
            if ~isgraphics(D.UI.Rat.mrkHeadNowH(2))
                D.UI.Rat.mrkHeadNowH(2) = ...
                    line(x, y, ...
                    'LineStyle', '-', ...
                    'Color', D.UI.ratPosAllCol, ....
                    'LineWidth', 1, ...
                    'Parent', D.UI.axH(3));
            else
                Safe_Set(D.UI.Rat.mrkHeadNowH(2), ...
                    'XData', x, ...
                    'YData', y);
            end
            
        end
        
        %% STORE TIME AND SET FLAGS
        
        % Update time
        D.DB.plot = Update_DB_DT(D.DB.plot, t_now);
        
        % Update UI
        if ...
                D.F.Rob.new_pos_data || ...
                D.F.Rob.new_pos_data || ...
                D.F.Rob.new_vel_data || ...
                D.F.Rob.new_vel_data || ...
                D.F.Rat.new_hd_data
            
            Update_UI(0, 'limitrate');
        end
        
        % Reset flags
        D.F.Rob.new_pos_data = false;
        D.F.Rat.new_pos_data = false;
        D.F.Rob.new_vel_data = false;
        D.F.Rat.new_vel_data = false;
        D.F.Rat.new_hd_data = false;
        
    end

% ----------------------PLOT POSITION HISTORY----------------------
    function VT_Plot_Hist()
        
        % Reset lap level indeces
        D.P.Rat.posAll.indLap(1) = D.P.Rat.posAll.indLap(2);
        D.P.Rob.posAll.indLap(1) = D.P.Rob.posAll.indLap(2);
        D.P.Rat.velAll.indLap(1) = D.P.Rat.velAll.indLap(2);
        D.P.Rob.velAll.indLap(1) = D.P.Rob.velAll.indLap(2);
        
        % Reset max vel
        D.P.Rat.vel_max_lap = 0;
        D.P.Rob.vel_max_lap = 0;
        
        % Plot rat pos all
        ind_get_str = D.P.Rat.posAll.indAll(1);
        ind_get_end = D.P.Rat.posAll.indAll(2);
        x = D.P.Rat.posAll.Cart(ind_get_str:ind_get_end,1);
        y = D.P.Rat.posAll.Cart(ind_get_str:ind_get_end,2);
        % set big jumps to NaN
        exc = find(diff(x)/D.UI.cm2pxl > 10 | diff(y)/D.UI.cm2pxl > 10 == 1) + 1;
        x(exc) = NaN;
        y(exc) = NaN;
        if ~isgraphics(D.UI.Rat.linPosAll)
            D.UI.Rat.linPosAll = ...
                line(x, y, ...
                'LineStyle', '-', ...
                'Color', D.UI.ratPosHistCol, ...
                'LineWidth', 1, ...
                'Parent', D.UI.axH(2));
        else
            Safe_Set(D.UI.Rat.linPosAll, ...
                'XData', x, ...
                'YData', y);
        end
        
        % Bail if not a 'Track' session
        if D.PAR.sesTask == 'Forage'
            return
        end
        
        % Plot vel all
        cols = [D.UI.ratHistCol; D.UI.robHistCol];
        flds = [{'Rat'},{'Rob'}];
        for i = [2,1]
            
            % Get field name
            fld = flds{i};
            
            % Bail if no data collected
            if D.P.(fld).velAll.indAll(2) == 0
                continue
            end
            
            % Get all vel pol data
            ind_get_str = D.P.(fld).velAll.indAll(1);
            ind_get_end = D.P.(fld).velAll.indAll(2);
            vel_rad = D.P.(fld).velAll.Pol(ind_get_str:ind_get_end, 1);
            vel_roh = D.P.(fld).velAll.Pol(ind_get_str:ind_get_end, 2);
            
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
            
            % Add to hist data
            ind_set_str = D.P.(fld).velAll.indHist(2) + 1;
            D.P.(fld).velAll.Hist(ind_set_str,1:101,1) = rad_bins;
            D.P.(fld).velAll.Hist(ind_set_str,1:101,2) = roh_interp;
            
            % Store ind
            D.P.(fld).velAll.indHist(2) = ind_set_str;
            
            % Get rad vel as 1D array
            ind_get_str = D.P.(fld).velAll.indHist(1);
            ind_get_end = ind_set_str;
            vel_rad = reshape(D.P.(fld).velAll.Hist(ind_get_str:ind_get_end,:,1)',1,[]);
            vel_roh = reshape(D.P.(fld).velAll.Hist(ind_get_str:ind_get_end,:,2)',1,[]);
            ind = ~isnan(vel_rad);
            
            % Plot all vel in cart space
            [x, y] = pol2cart(vel_rad(ind), vel_roh(ind));
            x =  x.*D.PAR.R + D.PAR.XC;
            y =  y.*D.PAR.R + D.PAR.YC;
            if ~isgraphics(D.UI.(fld).linVelAll)
                D.UI.(fld).linVelAll = ...
                    line(x, y, ...
                    'LineStyle', '-', ...
                    'Color', cols(i,:), ...
                    'LineWidth', 1, ...
                    'Parent', D.UI.axH(2));
            else
                Safe_Set(D.UI.(fld).linVelAll, ...
                    'XData', x, ...
                    'YData', y);
            end
            
        end
        
        % Plot vel avg
        cols = [D.UI.ratAvgCol; D.UI.robAvgCol];
        flds = [{'Rat'},{'Rob'}];
        for i = [2,1]
            
            % Get field name
            fld = flds{i};
            
            % Bail if no data collected
            if D.P.(fld).velAll.indHist(2) == 0
                continue
            end
            
            % Get accross lap average
            ind_get_str = D.P.(fld).velAll.indHist(1);
            ind_get_end = D.P.(fld).velAll.indHist(2);
            vel_rad = D.P.(fld).velAll.Hist(ind_get_str,:,1);
            roh_avg = nanmean(D.P.(fld).velAll.Hist(ind_get_str:ind_get_end,:,2),1);
            
            % Plot avg vel in cart space
            [x, y] = pol2cart(vel_rad, roh_avg);
            x =  x.*D.PAR.R + D.PAR.XC;
            y =  y.*D.PAR.R + D.PAR.YC;
            if ~isgraphics(D.UI.(fld).linVelAvgH)
                D.UI.(fld).linVelAvgH = ...
                    line(x, y, ...
                    'LineStyle', '-', ...
                    'Color', cols(i,:), ...
                    'LineWidth', 4, ...
                    'Parent', D.UI.axH(2));
            else
                Safe_Set(D.UI.(fld).linVelAvgH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
        end
        
    end

% -------------------------GET NLX EVENTS--------------------------
    function Evt_Get()
        
        % Bail if not connected
        if ~NlxAreWeConnected() == 1
            return
        end
        
        % Bail if not streaming
        if ~D.F.evt_streaming
            return
        end
        
        % Read in event data
        %[evtPass, D.E.evtTS, evtID, evtTTL, D.E.evtStr, D.E.evtNRecs, evtDropped]
        [~, D.E.evtTS, ~, ~ , D.E.evtStr, D.E.evtNRecs, ~] = ...
            NlxGetNewEventData('Events');
        
        % Center ts at poll start
        if ~isempty(D.E.evtTS)
            D.E.evtTS = D.E.evtTS - D.T.poll_str_nlx;
        end
        
        % Add to count
        D.E.cnt_evtRec = D.E.cnt_evtRec + D.E.evtNRecs;
    end

% ------------------------PROCESS NLX EVENTS-----------------------
    function Evt_Proc()
        
        %% CHECK FOR NEW DATA
        
        % Bail if no new data
        if D.E.evtNRecs == 0
            return
        end
        
        %% CHECK FOR REWARD
        
        % Reward Started
        if any(ismember(D.E.evtStr, D.NLX.rew_on_str))
            
            % Save reward start time
            D.T.rew_start = Sec_DT(now);
            
            % Get time stamp
            D.T.rew_nlx_ts(1) = D.E.evtTS(ismember(D.E.evtStr, D.NLX.rew_on_str));
            
            % Store round trip time
            if datenum(D.T.btn_rew_sent) > 0 && ...
                    datenum(D.T.btn_rew_sent) < datenum(D.T.rew_start)
                % Save reward mesage round trip time
                D.DB.rew_round_trip(1) = (D.T.rew_start - D.T.btn_rew_sent) * 1000;
                D.DB.rew_round_trip(2) = min(D.DB.rew_round_trip(1), D.DB.rew_round_trip(2));
                D.DB.rew_round_trip(3) = min(999, max(D.DB.rew_round_trip(1), D.DB.rew_round_trip(3)));
                D.DB.rew_round_trip(4) = D.DB.rew_round_trip(1) + D.DB.rew_round_trip(4);
                D.DB.rew_round_trip(5) = D.DB.rew_round_trip(5) + 1;
                % Reset time
                D.T.btn_rew_sent = 0;
            end
            
            % Change feeder dish plot color and marker size
            D.UI.feedPosCol = D.UI.attentionCol;
            D.UI.feedPosMarkSize = 30;
            if isgraphics(D.UI.Rob.mrkPosNowDishH)
                Safe_Set(D.UI.Rob.mrkPosNowDishH, ...
                    'MarkerFaceColor', D.UI.feedPosCol, ...
                    'MarkerSize', D.UI.feedPosMarkSize);
            end
            
            % Turn on reward button backround color
            Safe_Set(D.UI.btnReward, ...
                'BackgroundColor', D.UI.attentionCol);
            
            % Add to reward count for manual session
            if D.PAR.sesCond == 'Manual_Training'
                D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
            end
            
            % Set flags
            D.F.rewarding = true;
            
            % Log/print
            Console_Write(sprintf('[Evt_Proc] Recieved "%s" Event', D.NLX.rew_on_str));
            
        end
        
        % Reward Ended
        if any(ismember(D.E.evtStr, D.NLX.rew_off_str))
            
            % Save reward end time
            D.T.rew_end = Sec_DT(now);
            
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
                D.DB.rew_duration(3) = min(999, max(D.DB.rew_duration(1), D.DB.rew_duration(3)));
                D.DB.rew_duration(4) = D.DB.rew_duration(1) + D.DB.rew_duration(4);
                D.DB.rew_duration(5) = D.DB.rew_duration(5) + 1;
            end
            
            % Save halt error
            if D.I.zone_now ~= 0
                halt_err = D.PAR.rewZoneBnds(D.I.zone_now,2,D.I.rot) - (D.P.Rob.radLast-D.PAR.setPointRad);
                % Save reward duration
                D.DB.HALT.error(1) = halt_err * ((140*pi)/(2*pi));
                D.DB.HALT.error(2) = min(D.DB.HALT.error(1), D.DB.HALT.error(2));
                D.DB.HALT.error(3) = min(999, max(D.DB.HALT.error(1), D.DB.HALT.error(3)));
                D.DB.HALT.error(4) = D.DB.HALT.error(1) + D.DB.HALT.error(4);
                D.DB.HALT.error(5) = D.DB.HALT.error(5) + 1;
            end
            
            % Set flag
            D.F.rewarding = false;
            
            % Change feeder dish plot color and marker size
            D.UI.feedPosCol = D.UI.rotCol(D.I.rot,:);
            D.UI.feedPosMarkSize = 10;
            if isgraphics(D.UI.Rob.mrkPosNowDishH)
                Safe_Set(D.UI.Rob.mrkPosNowDishH, ...
                    'MarkerFaceColor', D.UI.feedPosCol, ...
                    'MarkerSize', D.UI.feedPosMarkSize);
            end
            
            % Turn off reward button backround color
            Safe_Set(D.UI.btnReward, ...
                'BackgroundColor', D.UI.enabledCol);
            
            % Log/print
            Console_Write(sprintf('[Evt_Proc] Recieved "%s" Event', D.NLX.rew_off_str));
            
        end
        
        %% CHECK FOR PID
        
        % Running
        if any(ismember(D.E.evtStr, D.NLX.pid_run_str))
            
            % Change setpoint plot color and width
            D.UI.setPosCol = D.UI.activeCol;
            D.UI.setPosLineWidth = 4;
            
            % Log/print
            Console_Write(sprintf('[Evt_Proc] Recieved "%s" Event', D.NLX.pid_run_str));
        end
        
        % Stopped
        if any(ismember(D.E.evtStr, D.NLX.pid_stop_str))
            
            % Change setpoint plot color
            D.UI.setPosCol = D.UI.disabledCol;
            D.UI.setPosLineWidth = 2;
            
            % Log/print
            Console_Write(sprintf('[Evt_Proc] Recieved "%s" Event', D.NLX.pid_stop_str));
        end
        
        %% CHECK FOR BULLDOZE
        
        % Running
        if any(ismember(D.E.evtStr, D.NLX.bull_run_str))
            
            % Change guard plot color and width
            D.UI.guardPosCol = D.UI.attentionCol;
            D.UI.guardPosLineWidth = 10;
            
            % Track count
            D.C.bull_cnt = D.C.bull_cnt + 1;
            
            % Log/print
            Console_Write(sprintf('[Evt_Proc] Recieved "%s" Event', D.NLX.bull_run_str));
        end
        % Stopped
        if any(ismember(D.E.evtStr, D.NLX.bull_stop_str))
            
            % Change guard plot color
            D.UI.guardPosCol = D.UI.disabledCol;
            D.UI.guardPosLineWidth = 5;
            
            % Log/print
            Console_Write(sprintf('[Evt_Proc] Recieved "%s" Event', D.NLX.bull_stop_str));
        end
        
    end

% ------------------------GET NLX CELL DATA------------------------
    function TT_Get()
        
        % Bail if not connected
        if ~NlxAreWeConnected() == 1 || ...
                ~D.F.cube_connected
            return
        end
        
        % Bail if cells not cut
        if get(D.UI.toggLoadClust, 'Value') == 0
            return
        end
        
        % Bail if nothing to stream
        if ~any(D.F.tt_streaming)
            return
        end
        
        % Get list of active tt
        tt_active_ind = find(D.F.tt_streaming);
        
        % Loop through and get new tt data
        for z_tt = 1:length(tt_active_ind)
            
            % Store tt label and ind
            tt_ind = tt_active_ind(z_tt);
            tt_lab = D.TT.ttLab{tt_ind};
            
            % Get new data
            [succeeded, ~, D.TT.ttTS{tt_ind}, ~, D.TT.ttClust{tt_ind}, ~, D.TT.ttRecs{tt_ind}, ~ ] = ...
                NlxGetNewTTData(tt_lab);
            
            % Center ts at poll start
            if ~isempty(D.TT.ttTS{tt_ind})
                D.TT.ttTS{tt_ind} = D.TT.ttTS{tt_ind} - D.T.poll_str_nlx;
            end
            
            % Check if streaming failed
            if succeeded ~= 1
                Console_Write(sprintf('**WARNING** [TT_Get] NlxGetNewTTData(%s) FAILED', tt_lab));
            end
            
        end
        
    end

% -----------------------PROCESS NLX CELL DATA---------------------
    function TT_Proc()
        
        % Bail if not connected
        if ~NlxAreWeConnected() == 1 || ...
                ~D.F.cube_connected
            return
        end
        
        % Bail if cells not cut
        if get(D.UI.toggLoadClust, 'Value') == 0
            return
        end
        
        % Bail if nothing to stream
        if ~any(D.F.tt_streaming)
            return
        end
        
        % Get list of active tt
        tt_active_ind = find(D.F.tt_streaming);
        
        % Loop through and check for new tt data
        for z_tt = 1:length(tt_active_ind)
            
            % Store tt ind
            tt_ind = tt_active_ind(z_tt);
            
            % Bail if no data
            if D.TT.ttRecs{tt_ind} < 1
                continue
            end
            
            % Store values
            ts_clust = double(D.TT.ttTS{tt_ind}');
            id_clust = D.TT.ttClust{tt_ind}';
            
            % Exclude times from before rat in
            exc_ind = ts_clust < D.T.task_str_nlx;
            ts_clust(exc_ind) = [];
            id_clust(exc_ind) = [];
            
            % Bail if no data left
            if sum(exc_ind) == D.TT.ttRecs{tt_ind}
                continue
            end
            
            % Bail if no pos data
            if D.P.Rat.posAll.indAll(2) == 0
                continue
            end
            
            % Set to last position value
            clust_cart_mat = ...
                repmat(D.P.Rat.posAll.Cart(D.P.Rat.posAll.indAll(2),:), D.TT.ttRecs{tt_ind}, 1);
            clust_pol_mat = ...
                repmat(D.P.Rat.posAll.Pol(D.P.Rat.posAll.indAll(2),:), D.TT.ttRecs{tt_ind}, 1);
            
            % Specify pos jitter
            cart_jit = [D.UI.vtRes, D.UI.vtRes] * 0.001;
            pol_jit = [2*pi, 1] * 0.001;
            
            % Add slight random offset to each sample
            if D.TT.ttRecs{tt_ind} > 1
                clust_cart_mat(:,1) = ...
                    clust_cart_mat(:,1) + randn(D.TT.ttRecs{tt_ind},1) * cart_jit(1);
                clust_cart_mat(:,2) = ...
                    clust_cart_mat(:,2) + randn(D.TT.ttRecs{tt_ind},1) * cart_jit(2);
                clust_pol_mat(:,1) = ...
                    clust_pol_mat(:,1) + randn(D.TT.ttRecs{tt_ind},1) * pol_jit(1);
                clust_pol_mat(:,2) = ...
                    clust_pol_mat(:,2) + randn(D.TT.ttRecs{tt_ind},1) * pol_jit(2);
            end
            
            % Get unique clusters
            clusts = unique(id_clust);
            
            % Store cluster pos info
            for z_c = 1:length(clusts)
                
                % Get cluster ind
                clust_ind = clusts(z_c)+1;
                
                % Get coluster position data ind
                inc_ind = id_clust == clusts(z_c);
                
                % Get number of samples
                n_samp = sum(inc_ind);
                
                % Get store start ind
                ind_set_str = D.TT.posClust{tt_ind, clust_ind}.indAll(2) + 1;
                
                % Check for overflow
                if isempty(ind_set_str)
                    
                    % Shift values back half way
                    shft = round(size(D.TT.posClust{tt_ind, clust_ind}.Cart, 1)/2);
                    D.TT.posClust{tt_ind, clust_ind}.Cart = ...
                        circshift(D.TT.posClust{tt_ind, clust_ind}.Cart, -1*shft, 1);
                    D.TT.posClust{tt_ind, clust_ind}.Pol = ...
                        circshift(D.TT.posClust{tt_ind, clust_ind}.Pol, -1*shft, 1);
                    D.TT.posClust{tt_ind, clust_ind}.TS = ...
                        circshift(D.TT.posClust{tt_ind, clust_ind}.TS, -1*shft, 1);
                    
                    % Set half of history to NaN
                    ind_set_str = shft;
                    D.TT.posClust{tt_ind, clust_ind}.Cart(ind_set_str:end,:) = NaN;
                end
                
                % Get store end ind
                ind_set_end = ind_set_str + n_samp-1;
                
                % Update store inds
                D.TT.posClust{tt_ind, clust_ind}.indAll(2) = ind_set_end;
                D.TT.posClust{tt_ind, clust_ind}.indLap(2) = ind_set_end;
                
                % Update pos values
                D.TT.posClust{tt_ind, clust_ind}.Cart(ind_set_str:ind_set_end, 1) = clust_cart_mat(inc_ind,1);
                D.TT.posClust{tt_ind, clust_ind}.Cart(ind_set_str:ind_set_end, 2) = clust_cart_mat(inc_ind,2);
                D.TT.posClust{tt_ind, clust_ind}.Pol(ind_set_str:ind_set_end, 1) = clust_pol_mat(inc_ind,1);
                D.TT.posClust{tt_ind, clust_ind}.Pol(ind_set_str:ind_set_end, 2) = clust_pol_mat(inc_ind,1);
                D.TT.posClust{tt_ind, clust_ind}.TS(ind_set_str:ind_set_end) = ts_clust(inc_ind);
                
                % Set plot flag
                D.F.plot_clust(tt_ind, clust_ind) = ...
                    Safe_Get(D.UI.toggSubPlotTT(tt_ind,clust_ind), 'Value') == 1;
                
            end
            
        end
        
        
        
        
    end

% ----------------------PLOT CURRENT POSITION----------------------
    function TT_Plot()
        
        % Bail if not connected
        if ~NlxAreWeConnected() == 1 || ...
                ~D.F.cube_connected
            return
        end
        
        % Bail if cells not cut
        if get(D.UI.toggLoadClust, 'Value') == 0
            return
        end
        
        % Bail if neither plot buttons active
        if get(D.UI.toggPlotTypeTT(1),'Value') == 0 && ...
                get(D.UI.toggPlotTypeTT(2),'Value') == 0
            return
        end
        
        % Get list of tts to plot
        tt_plot = find(any(D.F.plot_clust, 2));
        
        % Bail if nothing new to plot
        if isempty(tt_plot)
            return
        end
        
        % Plot cell
        for z_tt = 1:length(tt_plot)
            
            % Get tt label and ind
            tt_ind = tt_plot(z_tt);
            
            % Get clusters to plot
            clust_plot = find(D.F.plot_clust(tt_ind,:));
            
            % Check each clust button
            for z_c = 1:length(clust_plot)
                
                % Get clust ind
                clust_ind = clust_plot(z_c);
                
                % Bail if no data to plot
                if D.TT.posClust{tt_ind, clust_ind}.indLap(2) == 0
                    continue
                end
                
                % Get data to plot
                ind_get_str = D.TT.posClust{tt_ind, clust_ind}.indLap(1);
                ind_get_end = D.TT.posClust{tt_ind, clust_ind}.indLap(2);
                
                % Pull out values
                tt_x = D.TT.posClust{tt_ind, clust_ind}.Cart(ind_get_str:ind_get_end,1);
                tt_y = D.TT.posClust{tt_ind, clust_ind}.Cart(ind_get_str:ind_get_end,2);
                tt_rad = D.TT.posClust{tt_ind, clust_ind}.Pol(ind_get_str:ind_get_end,1);
                
                % Do spike position
                if get(D.UI.toggPlotTypeTT(1),'Value') == 1
                    
                    % Plot spike pos marker
                    if ~isgraphics(D.UI.mrkClustH(tt_ind, clust_ind))
                        D.UI.mrkClustH(tt_ind, clust_ind) = ...
                            line(tt_x, tt_y, ...
                            'LineStyle', 'none', ...
                            'Marker', 'o', ...
                            'LineWidth', 0.25, ...
                            'MarkerFaceColor', D.UI.clustCol(tt_ind,clust_ind,:), ...
                            'MarkerEdgeColor', [0,0,0], ... % 'MarkerEdgeColor', 'none', ...
                            'MarkerSize', 5, ...
                            'Parent', D.UI.axH(6));
                    else
                        Safe_Set(D.UI.mrkClustH(tt_ind, clust_ind), ...
                            'XData', tt_x, ...
                            'YData', tt_y);
                    end
                    
                end
                
                % Plot rate map
                if get(D.UI.toggPlotTypeTT(2),'Value') == 1
                    
                    % Compute 1D hist for spike and occ
                    if D.PAR.sesTask == 'Track'
                        
                        % Get occ hist from rad data
                        ind_get_str = D.P.Rat.posAll.indAll(1);
                        ind_get_end = D.P.Rat.posAll.indAll(2);
                        rat_rad = ...
                            D.P.Rat.posAll.Pol(ind_get_str:ind_get_end, 1);
                        occ_hist = histc(rat_rad, D.PAR.tt1dBinEdge);
                        occ_hist = occ_hist(2:end);
                        
                        % Get tt hist from pol data
                        tt_hist = histc(tt_rad, D.PAR.tt1dBinEdge);
                        tt_hist = tt_hist(2:end);
                        
                    end
                    
                    % Compute 2D hist for spike and occ
                    if D.PAR.sesTask == 'Forage'
                        
                        % Copy stored occ data
                        occ_hist = D.P.frgOccMatRaw;
                        
                        % Get tt hist from cart data
                        tt_hist = histcounts2(tt_y, tt_x, D.PAR.tt2dBinEdgeY, D.PAR.tt2dBinEdgeX);
                        tt_hist = flip(tt_hist,1);
                    end
                    
                    % Make sure occ_hist is column vector
                    if size(occ_hist, 1) < size(occ_hist, 2)
                        occ_hist = occ_hist';
                    end
                    
                    % Make sure tt_hist is column vector
                    if size(tt_hist, 1) < size(tt_hist, 2)
                        tt_hist = tt_hist';
                    end
                    
                    % Norm hist data
                    tt_hist = tt_hist/max(tt_hist(:));
                    occ_hist = occ_hist/max(occ_hist(:));
                    
                    % Convert to rate
                    rate_now = tt_hist./occ_hist;
                    
                    % Replace inf vals with raw tt_hist (i.e., no occ data for this bin)
                    rate_now(isinf(rate_now)) = tt_hist(isinf(rate_now));
                    
                    % Replace nan vals with 0 (i.e., no tt data for this bin)
                    rate_now(isnan(rate_now)) = 0;
                    
                    % Scale rate to 99th percentile
                    non_zer_rate = rate_now(rate_now(:) > 0);
                    scale = prctile(non_zer_rate,99);
                    if scale == 0
                        scale = 1;
                    end
                    
                    % Scale to percentile
                    rate_scale = rate_now/scale;
                    
                    % Renorm rate to range [0,1]
                    rate_norm = rate_scale/max(rate_scale);
                    
                    % Set 1D plot patch face alpha based on rate
                    if D.PAR.sesTask == 'Track'
                        
                        % Store alpha in cell array
                        alph = num2cell(rate_norm);
                        
                        % Get handle to current axis patches
                        patch_h = findobj(D.UI.ttClustAxRef(tt_ind, clust_ind),'Type','patch');
                        
                        % Set axis patch alpha based on rate
                        [patch_h.FaceAlpha] = alph{:};
                        
                    end
                    
                    % Plot 2D rate map
                    if D.PAR.sesTask == 'Forage'
                        
                        % Get alpha mask
                        alph = double(rate_scale>0) * 0.9;
                        
                        % Show 2D spike occ
                        cdat = rate_scale;
                        if ~isgraphics(D.UI.imgClustH(tt_ind, clust_ind))
                            D.UI.imgClustH(tt_ind, clust_ind) = ...
                                imagesc(cdat, ...
                                'AlphaData', alph, ...
                                'Parent', D.UI.ttClustAxRef(tt_ind, clust_ind));
                        else
                            Safe_Set(D.UI.imgClustH(tt_ind, clust_ind), ...
                                'CData', cdat, ...
                                'AlphaData', alph);
                            
                            % Check for axis change
                            if D.UI.imgClustH(tt_ind, clust_ind).Parent ~= ...
                                    D.UI.ttClustAxRef(tt_ind, clust_ind)
                                
                                Safe_Set(D.UI.imgClustH(tt_ind, clust_ind), ...
                                    'Parent', D.UI.ttClustAxRef(tt_ind, clust_ind))
                            end
                        end
                        
                    end
                    
                end
                
                % Reset flag
                D.F.plot_clust(tt_ind, clust_ind) = false;
                
            end
            
        end
        
        % Update UI
        Update_UI(0, 'limitrate');
        
    end

% ------------------CHECK CUBE AND ROBOT STATUS--------------------
    function Hardware_Check()
        
        % GET ROBOT VOLTAGE
        if c2m.('J').dat1 ~= 0
            
            % Update vcc
            D.PAR.rob_vcc_last = D.PAR.rob_vcc;
            D.PAR.rob_vcc = c2m.('J').dat1;
            
        end
        
        % CHECK CUBE STATUS
        
        % Bail if not implant session or Cheetah not running
        if ~D.F.implant_session || ~D.F.cheetah_running
            return
        end
        
        % See if cube ready for setup
        if ~D.F.cube_connected && Sec_DT(now) >= D.T.cube_status_check
            
            % Try to get cube vcc
            [pass, vcc] = Get_Cube_Vcc();
            
            % Bail if not succeeded or vcc = 0
            if ~pass || vcc == 0
                
                % Update check time
                D.T.cube_status_check = Sec_DT(now) + 1;
                
                % Bail
                return;
            end
            
            % Set flag
            D.F.cube_connected = true;
            
            % Enable cube buttons
            Button_State(D.UI.toggCubeLED, 'Enable');
            Button_State(D.UI.toggCubeVcc, 'Enable');
            
            % Set button to get first cube vcc
            Safe_Set(D.UI.toggCubeVcc, 'Value', 1);
            Button_State(D.UI.toggCubeVcc, 'Update');
            
            % Start with Cube LED off
            Safe_Set(D.UI.toggCubeLED, 'Value', 0)
            Togg_CubeLED(D.UI.toggCubeLED);
            
            % Set Cube battery type
            %   Note:
            %       "C": Small (green dot)
            %       "A": Medium (blue dot)
            Send_NLX_Cmd(sprintf('-SendLynxSXCommand AcqSystem1 -SetBatteryType "%s"', D.PAR.cubeBatteryType));
            
            % Create message
            msg = sprintf('CUBE CONNECTED: Battery Type is "%s"', D.PAR.cubeBatteryType);
            
            % Prompt cube connected
            dlgAWL( ...
                msg, ...
                'CUBE CONNECTED', ...
                'OK', [], [], 'OK', ...
                D.UI.dlgPos{4}, ...
                'default');
            
            % Print Cube status
            Console_Write(['[Hardware_Check] ', msg]);
            
        end
        
        % GET CUBE VOLTAGE
        
        % Bail if not connected or button not pressed
        if ~D.F.cube_connected || Safe_Get(D.UI.toggCubeVcc, 'Value') ~= 1
            return
        end
        
        % Unset button
        Safe_Set(D.UI.toggCubeVcc, 'Value', 0);
        Button_State(D.UI.toggCubeVcc, 'Update');
        
        % Run nested function
        [pass, vcc] = Get_Cube_Vcc();
        
        % Bail if failed
        if ~pass
            return
        end
        
        % Store last value
        D.PAR.cube_vcc_last = D.PAR.cube_vcc;
        
        % Store new value
        D.PAR.cube_vcc = vcc;
        
        % Update check time
        D.T.cube_vcc_check = Sec_DT(now);
        
        % GET CUBE VOLTAGE FUNCTION
        function[pass, vcc] = Get_Cube_Vcc()
            
            % Initialize output
            pass = false;
            vcc = 0;
            
            % Send NLX command
            [succeeded, nlx_str] = Send_NLX_Cmd('-SendLynxSXCommand AcqSystem1 -WHSGetStateOfCharge 1', false);
            %[succeeded, nlx_str] = Send_NLX_Cmd('-SendLynxSXCommand AcqSystem1 -GetBatteryRemaining AcqSystem1', false);
            
            % Bail if command failed
            if succeeded ~= 1
                return
            else
                pass = true;
            end
            
            % Parce nxl string
            nlx_str = regexp(nlx_str{:}, '\d*$', 'match');
            
            % Check if parsed
            if ~isempty(nlx_str)
                vcc = str2double(nlx_str{:});
            else
                pass = false;
            end
            
        end
        
    end

% ------------------------CHECK SLEEP STATUS-----------------------
    function Sleep_Check(sleep_phase)
        
        % Set flag and bail if not implant session
        if ~D.F.implant_session
            
            % Set flag
            D.F.sleep_done(sleep_phase) = true;
            return
        end
        
        % Bail if sleep not started or finished
        if D.T.sleep_str(sleep_phase) == 0 || ...
                D.F.sleep_done(sleep_phase)
            return
        end
        
        % Check time left
        sleep_dt = ...
            (Sec_DT(now) - D.T.sleep_str(sleep_phase));
        
        % Check for sleep done button flag
        is_done = D.UI.toggSleep(sleep_phase).UserData(2) == 1;
        
        % Wait till time ellapsed
        if sleep_dt < D.PAR.sleepDur(sleep_phase) && ...
                ~is_done && ...
                ~DOEXIT
            
            % Change button string
            str = ...
                sprintf('%s/%s', ...
                datestr(sleep_dt/(24*60*60), 'MM:SS'), ...
                datestr(D.PAR.sleepDur(sleep_phase)/(24*60*60), 'MM:SS'));
            D.UI.editSleep(sleep_phase).String = str;
            
            % Pause before next loop
            pause(0.1);
            
            % Exit function
            return
            
        end
        
        % Unset sleep button if time elapsed
        if sleep_dt >= D.PAR.sleepDur(sleep_phase)
            Safe_Set(D.UI.toggSleep(sleep_phase), 'Value', 0);
            Togg_Sleep(D.UI.toggSleep(sleep_phase));
        end
        
        % Set final button string
        if sleep_dt >= D.PAR.sleepDur(sleep_phase)
            str = ...
                sprintf('%s/%s', ...
                datestr(D.PAR.sleepDur(sleep_phase)/(24*60*60), 'MM:SS'), ...
                datestr(D.PAR.sleepDur(sleep_phase)/(24*60*60), 'MM:SS'));
            D.UI.editSleep(sleep_phase).String = str;
        end
        
        % Set flag
        D.F.sleep_done(sleep_phase) = true;
        
        % Handle Sleep 2
        if sleep_phase == 2
            
            % Stop recording
            if D.F.rec
                Safe_Set(D.UI.toggRec,'Value', 0)
                Togg_Rec(D.UI.toggRec);
            end
            
        end
        
    end

% ----------------------ROTATION TRIGGER CHECK---------------------
    function Rotation_Trig_Check()
        
        % Bail if not a 'Track' session
        if D.PAR.sesTask == 'Forage'
            return
        end
        
        % Bail if not flagged
        if ~D.F.rotate
            return
        end
        
        % Check if rat in rotation bounds
        check_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.UI.rotBndNext);
        if ~any(check_inbound)
            return
        end
        
        % Set flags
        D.F.rotated = true;
        D.F.rotate = false;
        
        % Save current image
        rotLast = D.I.rot;
        
        % Get new image
        D.I.rot = find([1, 2] ~=  D.I.rot);
        
        % Update D.AC.data(2) and send command to rotate image
        D.AC.data(2) = D.I.img_ind(D.I.rot);
        Send_AC_Com();
        
        % Post NLX event: rotaion *deg
        Send_NLX_Cmd(D.NLX.rot_evt{D.I.rot});
        
        % Change plot marker size
        % active feeder
        Safe_Set(D.UI.mixFdNow(3, D.I.rot), ...
            'MarkerSize', 25);
        % inactive feeder
        Safe_Set(D.UI.mixFdNow(3, [1, 2] ~=  D.I.rot), ...
            'MarkerSize', 20)
        
        % Reset rot bounds
        Safe_Set(D.UI.txtRtBnds(:), ...
            'Visible', 'off', ...
            'HitTest', 'off');
        Patch_State(D.UI.ptchRtBnds(1,:), ...
            'Hide', D.UI.rotCol(1,:));
        Patch_State(D.UI.ptchRtBnds(2,:), ...
            'Hide', D.UI.rotCol(2,:));
        Safe_Set(D.UI.ptchRtBnds, 'HitTest', 'off')
        
        % Plot rotation pos mark
        r = D.P.Rat.rad(check_inbound);
        [xbnd,ybnd] = Get_Cart_Bnds([r(1), r(1)+deg2rad(1)]);
        patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
            [ybnd(1,:),fliplr(ybnd(2,:))], ...
            D.UI.robNowCol, ...
            'EdgeColor', D.UI.rotCol(D.I.rot,:),...
            'FaceColor', D.UI.rotCol(D.I.rot,:),...
            'LineWidth', 4, ...
            'FaceAlpha',0.5, ...
            'EdgeAlpha',0.5, ...
            'HitTest', 'off', ...
            'Parent',D.UI.axH(3));
        
        % Reset reward bounds patch feeder
        Patch_State(D.UI.ptchRewZoneBndsH(rotLast,:), ...
            'ShowPartial', D.UI.rotCol(rotLast,:));
        Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot,:), ...
            'ShowAll', D.UI.rotCol(D.I.rot,:));
        
        % Change button settings
        Button_State(D.UI.toggICR(D.I.rot), 'Disable', D.UI.rotCol(D.I.rot,:));
        Button_State(D.UI.toggICR(rotLast), 'Enable', D.UI.rotCol(rotLast,:));
        
        % Change wall image
        Safe_Set(D.UI.wallImgH(D.I.img_ind(rotLast)), 'Visible', 'off');
        Safe_Set(D.UI.wallImgH(D.I.img_ind(D.I.rot)), 'Visible', 'on');
        
        % Hide reward reset patch
        Patch_State(D.UI.ptchRewSendH([1, 2] ~=  D.I.rot), ...
            'Hide', D.UI.rotCol([1, 2] ~=  D.I.rot,:));
        
        % Show new patch
        Patch_State(D.UI.ptchRewSendH(D.I.rot), ...
            'ShowAll', D.UI.rotCol(D.I.rot,:));
        
        % Change session info font weight
        % active feeder
        Safe_Set(D.UI.txtPerfInf(D.I.rot), 'FontWeight', 'Bold')
        
        % inactive feeder
        Safe_Set(D.UI.txtPerfInf([1, 2] ~=  D.I.rot), 'FontWeight', 'Light')
        
        % Add to roation counter
        D.C.rot_cnt = D.C.rot_cnt + 1;
        
        % Add new lap counter after all rot conds run once
        if  D.C.rot_cnt > 2
            D.C.lap_cnt{D.I.rot} = [D.C.lap_cnt{D.I.rot}, 0];
            D.C.rew_cnt{D.I.rot} = [D.C.rew_cnt{D.I.rot}, 0];
        end
        
    end

% ---------------------TRACK REWARD SEND CHECK---------------------
    function Track_Reward_Send_Check()
        
        % BAIL IF MANUAL OR FORAGE TRAINING OR BOUNDS PASSED
        if D.PAR.sesCond == 'Manual_Training' || ...
                D.PAR.sesTask == 'Forage' || ...
                D.F.rew_sent || ...
                all(isnan(D.P.Rat.rad))
            return
        end
        
        % Check if rat is in quad
        check_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.UI.rewRstBnds(D.I.rot,:));
        if ~any(check_inbound)
            return
        end
        
        % Itterate count
        D.C.rew_send_cnt = D.C.rew_send_cnt+1;
        
        % Disable cue buttons
        Cue_Button_State('Disable');
        
        % Reset reward zone patches
        Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot,:), ...
            'ShowAll', D.UI.rotCol(D.I.rot,:));
        
        % Clear last duration
        Safe_Set(D.UI.txtFdDurH(:), 'Visible', 'off');
        
        % Handle cued reward
        if get(D.UI.toggDoCue, 'Value') == 1
            
            % Use cued zone
            D.I.zone_now = find(D.PAR.zoneLocs == D.PAR.zoneCue);
            
            % Get new reward duration
            D.PAR.rewDur = D.PAR.zoneRewDur(D.I.zone_now);
            
            % Will send CS command with pos and zone
            r_pos = D.UI.rewRatHead(D.I.rot);
            z_ind = D.I.zone_now;
            r_cond = 2;
            
            % Send cued reward command
            Send_CS_Com('R', r_pos, r_cond, z_ind);
            
            % Post NLX event: cue on
            Send_NLX_Cmd(D.NLX.cue_on_evt);
            
            % Show new cued reward target patch
            Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot,D.I.zone_now), ...
                'Active', D.UI.rotCol(D.I.rot,:));
            
            % Print new duration
            Safe_Set(D.UI.txtFdDurH(D.I.rot,D.I.zone_now), ...
                'Visible', 'on');
            
        else
            
            % Send reward center and no zone ind
            r_pos = D.UI.rewRatHead(D.I.rot);
            r_del = D.PAR.rewDel;
            r_cond = 3;
            
            % Send free reward command
            Send_CS_Com('R', r_pos, r_cond, r_del);
            
            % Post NLX event: cue off
            Send_NLX_Cmd(D.NLX.cue_off_evt);
            
            % Darken all zone patches
            Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot,:), ...
                'Select', D.UI.rotCol(D.I.rot,:))
        end
        
        % Set reward sent flag
        D.F.rew_sent = true;
        
        % Reset reward check flags
        D.F.rew_zone_crossed = false;
        D.F.rew_confirmed = false;
        D.F.rew_reset = false;
        
        % Hide reset patch
        Patch_State(D.UI.ptchRewSendH(D.I.rot), ...
            'Hide', D.UI.rotCol(D.I.rot,:));
        
        % Stop bulldozer if active
        D.PAR.bullLastVal = get(D.UI.toggBulldoze, 'Value');
        if D.PAR.bullLastVal == 1
            Safe_Set(D.UI.toggBulldoze, 'Value', 0);
            Pop_Bulldoze();
        end
        
        % Print reset bounds crossed
        Console_Write(sprintf('[Track_Reward_Send_Check] Crossed Reward Send Bounds: cross_cnt=%d', D.C.rew_send_cnt));
        
    end

% ----------------------TRACK REWARD ZONE CHECK--------------------
    function Track_Reward_Zone_Check()
        
        %% BAIL IF MANUAL OR FORRAGE TRAINING
        
        if D.PAR.sesCond == 'Manual_Training' || ...
                D.PAR.sesTask == 'Forage'
            
            return
        end
        
        %% CHECK FOR REWARD CONFIRMATION
        
        % Check if reward has been reset
        if  ~D.F.rew_zone_crossed && ...
                D.F.rew_sent && ...
                c2m.('Z').dat1 ~= 0 && ...
                ~D.F.rew_confirmed
            
            % Store rewarded zone ind
            D.I.zone_now = c2m.('Z').dat1;
            D.I.zone_active(:) = false;
            D.I.zone_active(D.I.zone_now) = true;
            
            % Reset next zone
            D.I.zone_select = 0;
            
            % Post NLX event: reward info
            Send_NLX_Cmd(...
                sprintf(D.NLX.rew_evt, -1*D.PAR.zoneLocs(D.I.zone_now), D.PAR.zoneRewDur(D.I.zone_now)));
            
            % Update reward count
            if D.F.rotated
                % Add to rotation condition count
                D.C.rew_cnt{D.I.rot}(end) = D.C.rew_cnt{D.I.rot}(end) + 1;
            else
                % Add to standard count
                D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
            end
            
            % Update cued reward count
            if get(D.UI.toggDoCue, 'Value') == 1
                D.PAR.cued_rew = [D.PAR.cued_rew, sum([D.C.rew_cnt{:}])];
            end
            
            % Store reward zone with range [-20,20]
            D.PAR.zone_hist(sum([D.C.rew_cnt{:}])) = -1*D.PAR.zoneLocs(D.I.zone_now);
            
            % Get zone dist data
            D.C.zone(D.I.rot,D.I.zone_now) = D.C.zone(D.I.rot,D.I.zone_now)+1;
            x_zone = -1*D.PAR.zoneLocs;
            y_zone = D.C.zone(D.I.rot,:) / sum(D.C.zone(D.I.rot,:));
            y_zone = y_zone/max(y_zone);
            
            % Update zone all patch
            x = x_zone;
            y = y_zone;
            D.UI.ptchRewZoneHistH(D.I.rot,:) = ...
                Plot_Zone_Hist(...
                x, y, 2, ...
                D.UI.rotCol(D.I.rot,:), ...
                0.25, ...
                D.UI.axZoneH(2), ...
                D.UI.ptchRewZoneHistH(D.I.rot,:));
            
            % Display count
            Safe_Set(D.UI.axZoneH(1), 'XTickLabel', D.C.zone(D.I.rot,:))
            
            % Reset missed rewards
            D.C.missed_rew_cnt(2) = sum(D.C.missed_rew_cnt);
            D.C.missed_rew_cnt(1) = 0;
            
            % Plot average zone pos
            avg_trig = D.PAR.zoneLocs*D.C.zone(D.I.rot,:)' / sum(D.C.zone(D.I.rot,:));
            [xbnd, ybnd] =  ...
                Get_Cart_Bnds(D.UI.rewZoneRad(D.I.rot) + deg2rad(avg_trig + D.PAR.trigDist));
            x = xbnd;
            y = ybnd;
            if ~isgraphics(D.UI.linZoneAvgH)
                D.UI.linZoneAvgH = ...
                    line(x, y, ...
                    'Color', D.UI.rotCol(D.I.rot,:), ...
                    'LineStyle', '-', ...
                    'LineWidth', 1, ...
                    'Parent',D.UI.axH(3));
                uistack(D.UI.linZoneAvgH, 'bottom');
            else
                Safe_Set(D.UI.linZoneAvgH, ...
                    'XData', x, ...
                    'YData', y);
            end
            
            % Reset reward zone patches
            Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot,:), ...
                'ShowAll', D.UI.rotCol(D.I.rot,:));
            
            % Darken current zone
            Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot,D.I.zone_now), ...
                'Select', D.UI.rotCol(D.I.rot,:));
            
            % Print new duration
            Safe_Set(D.UI.txtFdDurH(D.I.rot,D.I.zone_now), ...
                'Visible', 'on');
            
            % Set reward confirmed flag
            D.F.rew_confirmed = true;
            
            % Reset 'Z' data
            c2m.('Z').dat1 = 0;
            
            % Reset flags
            D.F.check_rew_confirm = false;
            
            % Log/print reward details
            Console_Write(sprintf('[Track_Reward_Zone_Check] Rewarded: rew_cnt=%d zone=%d vel=%0.2fcm/sec', ...
                sum([D.C.rew_cnt{:}]), D.PAR.zone_hist(sum([D.C.rew_cnt{:}])), D.P.Rat.vel));
            
            % Update UI
            Update_UI(0, 'limitrate');
            
        end
        
        %% CHECK IF ALL ZONES PASSED OR CONFIRMATION RECEIVED
        
        % Track reward crossing
        if D.F.rew_sent && ~D.F.rew_zone_crossed
            
            % Check if rat passed all zones
            check_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.UI.rewPassBnds(D.I.rot,:));
            
            % Bail if not passed zones and reward not confirmed
            if ~any(check_inbound) && ~D.F.rew_confirmed
                return
            end
            
            % Check for zone crossing before resseting flags for reward
            % send to run
            if any(check_inbound)
                
                % Set reward zones corssed flag
                D.F.rew_zone_crossed = true;
                
                % Reset other flags
                D.F.rew_sent = false;
                
                % Itterate count
                D.C.rew_cross_cnt = D.C.rew_cross_cnt+1;
                
            end
            
            % Bail if reset already done
            if D.F.rew_reset
                return
            end
            
            % Set reward reset flag
            D.F.rew_reset = true;
            
            % Enable cue buttons
            Cue_Button_State('Enable');
            
            % Check if next rew is cued
            if ...
                    get(D.UI.toggForceCue, 'Value') == 0 && ...
                    get(D.UI.toggBlockCue, 'Value') == 0 && ...
                    (D.F.rew_confirmed || D.C.rew_send_cnt == 0)
                
                if ...
                        D.PAR.sesCue == 'All' || ...
                        (D.PAR.sesCue == 'Half' && get(D.UI.toggDoCue, 'Value') == 0)
                    
                    % Set cue button on
                    Safe_Set(D.UI.toggDoCue, 'Value', 1);
                    Togg_DoCue();
                    
                elseif D.PAR.sesCue == 'Half'
                    
                    % Set cue button off
                    Safe_Set(D.UI.toggDoCue, 'Value', 0);
                    Togg_DoCue();
                end
            end
            
            % Restart bulldozer if was active
            if D.PAR.bullLastVal == 1
                Safe_Set(D.UI.toggBulldoze, 'Value', 1);
                Pop_Bulldoze();
            end
            
            % Check for missed rewards
            if ~D.F.rew_confirmed
                
                % Add to missed reward count
                D.C.missed_rew_cnt(1) = D.C.missed_rew_cnt(1)+1;
                
                % Lighten all reward zone patches
                Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot,:), ...
                    'ShowAll', D.UI.rotCol(D.I.rot,:));
                
                % Hide last duration
                Safe_Set(D.UI.txtFdDurH(D.I.rot,:), ...
                    'Visible', 'off');
                
                % Print missed reward
                Console_Write(sprintf('[Track_Reward_Zone_Check] Detected Missed Reward: cross_cnt=%d miss_cnt=%d|%d', ...
                    D.C.rew_cross_cnt, D.C.missed_rew_cnt(1), D.C.missed_rew_cnt(2)));
                
            end
            
            % Set reward send patch to visible
            Patch_State(D.UI.ptchRewSendH(D.I.rot), ...
                'Select', D.UI.rotCol(D.I.rot,:));
            
            % Update reward info list
            rew_ellapsed = Sec_DT(now) - D.T.rew_last;
            D.T.rew_last = Sec_DT(now);
            D.UI.rewInfoList = [...
                D.UI.rewInfoList; ...
                {sprintf('%d: T:%0.2f Z:%d M:%d', ...
                sum([D.C.rew_cnt{:}])+1, ...
                rew_ellapsed, ...
                -1*D.PAR.zoneLocs(D.I.zone_now), ...
                sum(D.C.missed_rew_cnt)) ...
                }];
            rew_percent = ...
                round(100-(sum(D.C.missed_rew_cnt)/D.C.rew_cross_cnt)*100);
            infstr = [...
                sprintf('Reward Info (%d%%)', rew_percent); ...
                D.UI.rewInfoList];
            Safe_Set(D.UI.popRewInfo, 'String', infstr);
            
            % Log/print reset bounds crossed
            Console_Write(sprintf('[Track_Reward_Zone_Check] Crossed Reward Bounds: cross_cnt=%d', D.C.rew_cross_cnt));
            
            % Update UI
            Update_UI(0, 'limitrate');
            
        end
        
    end

% ----------------------FORAGE REWARD ZONE CHECK-------------------
    function Forage_Reward_Targ_Check()
        
        % BAIL IF MANUAL OR TRACK TRAINING
        if D.PAR.sesCond == 'Manual_Training' || ...
                D.PAR.sesTask == 'Track'
            return
        end
        
        % Bail if no new data
        if all(isnan(D.P.Rat.rad))
            return
        end
        
        % CHECK IF RAT SHOULD BE REWARDED
        
        % Check if rat in new target bounds
        check_current_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.PAR.rewTargBnds(D.I.targ_now,:));
        
        % Reward
        if any(check_current_inbound) && ...
                ~D.F.move_to_targ
            
            % Get inbound ts
            if D.T.frg_rew_inbnd_t1 == 0
                D.T.frg_rew_inbnd_t1 = D.P.Rat.ts(find(check_current_inbound, 1, 'first'));
            else
                D.T.frg_rew_inbnd_t2 = D.P.Rat.ts(find(check_current_inbound, 1, 'last'));
            end
            
            % Compute time in seconds
            inbndTim = (D.T.frg_rew_inbnd_t2 - D.T.frg_rew_inbnd_t1) / 10^6;
            
            % Check if rat has been in for the min delay period
            if inbndTim < D.PAR.rewDel
                return
            end
            
            % Reinitialize inbound time
            D.T.frg_rew_inbnd_t1 = 0;
            
            % Reset occ once 50% of area covered
            if sum(D.P.frgOccMatBinary(:))/sum(D.PAR.frgMask(:)) >= 0.5
                % Set all to zero
                D.P.frgOccMatBinary(:) = 0;
            end
            
            % Send reward now command
            r_pos = 0;
            r_cond = 1;
            z_ind = get(D.UI.popReward, 'Value');
            Send_CS_Com('R', r_pos, r_cond, z_ind);
            
            % Add to reward count
            D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
            
            % Store reward time
            D.T.frg_rew = Sec_DT(now);
            
            % Flag to do move after time ellapsed
            D.F.move_to_targ = true;
            
            % Store last targ
            D.I.targ_last = D.I.targ_now;
            
            % Lighten patch
            Patch_State(D.UI.ptchRewTargBnds(D.I.targ_now), ...
                'ShowAll');
            
            % Enable pick rew targ
            Button_State(D.UI.toggPickRewPos, 'Enable');
            
            % Update plot history
            VT_Plot_Hist()
            
            % Log/print
            Console_Write(sprintf('[Forage_Reward_Targ_Check] Rewarded: targ=%ddeg dt_occ=%0.2fsec', ...
                D.PAR.frgTargDegArr(D.I.targ_last), inbndTim));
            
            % Update UI
            Update_UI(0, 'limitrate');
            
            % Bail
            return
            
        end
        
        % Reinitialize time
        D.T.frg_rew_inbnd_t1 = 0;
        
        % GET NEXT REWARD TARGET
        
        % Check if rat still in last target bounds
        check_last_inbound = Check_Pol_Bnds(D.P.Rat.rad, D.P.Rat.roh, D.PAR.rewTargBnds(D.I.targ_last,:));
        
        % Initialize flag
        is_left_early = false;
        
        % Check if rat moved early
        if Sec_DT(now) < D.T.frg_rew+D.PAR.frgRewBlock
            
            % Check for out of bound sampels
            if ~any(check_last_inbound)
                
                % Get out of bound ts
                if D.T.frg_rew_outbnd_t1 == 0
                    D.T.frg_rew_outbnd_t1 = D.P.Rat.ts(find(~check_last_inbound, 1, 'first'));
                else
                    D.T.frg_rew_outbnd_t2 = D.P.Rat.ts(find(~check_last_inbound, 1, 'last'));
                end
                
                % Compute time in seconds
                outbndTim = (D.T.frg_rew_outbnd_t2 - D.T.frg_rew_outbnd_t1) / 10^6;
                
                % Check if rat has been out for min delay period
                if outbndTim < 1
                    return
                end
                
                % Reinitialize time
                D.T.frg_rew_outbnd_t1 = 0;
                
                % Set flag
                is_left_early = true;
                
                % Log/print
                Console_Write(sprintf('**WARNING** [Forage_Reward_Targ_Check] Rat Moved Out of Bounds Early: dt_rew=%0.2fsec dt_occ=%0.2fsec', ...
                    Sec_DT(now) - D.T.frg_rew, outbndTim));
                
            else
                % Reinitialize time
                D.T.frg_rew_outbnd_t1 = 0;
            end
            
        end
        
        % Bail to leave time to select manual targ select and rat still
        % in old bounds and not time to move
        if (Sec_DT(now) < D.T.frg_rew+D.PAR.frgRewBlock || ...
                get(D.UI.toggPickRewPos, 'Value') == 1) && ...
                ~is_left_early
            
            % Bail
            return
            
            % GET OPTIMAL NEW PATH
        elseif D.I.targ_now == D.I.targ_last
            
            % Reset target select button
            Safe_Set(D.UI.toggPickRewPos, 'Value', 0)
            Button_State(D.UI.toggPickRewPos, 'Disable');
            
            % Get binary map
            inv_occ = abs(D.P.frgOccMatBinary-1);
            occ_prod = ...
                squeeze(sum(sum(D.P.pathMat(:,:,:,D.I.targ_now) .* ...
                repmat(inv_occ,[1,1,size(D.P.pathMat,3)]),1),2));
            
            % Pull out optimal path
            path_ind = find(occ_prod == max(occ_prod));
            if length(path_ind) > 1
                path_ind = path_ind(ceil(rand(1,1)*length(path_ind)));
            end
            path_rad = wrapTo2Pi(deg2rad(D.PAR.frgPathDegArr(path_ind)*2));
            
            % Get new targ
            rad_last_targ = deg2rad(D.PAR.frgTargDegArr(D.I.targ_now));
            rad_new_targ = Rad_Sum(Rad_Sum(rad_last_targ,pi), path_rad);
            targ_ind_new = ...
                find(round(rad2deg(rad_new_targ)) == D.PAR.frgTargDegArr);
            
            % Store path mat for plotting
            D.P.pathNowMat = D.P.pathMat(:,:,path_ind,D.I.targ_now);
            D.P.pathNowMat(D.P.pathNowMat>0) = 0.05;
            
            % Update patches
            Patch_State(D.UI.ptchRewTargBnds, ...
                'Hide');
            Patch_State(D.UI.ptchRewTargBnds(targ_ind_new), ...
                'Active', D.UI.activeCol);
            
            % Store new targ
            D.I.targ_now = targ_ind_new;
            
            % Log/print
            Console_Write(sprintf('[Forage_Reward_Targ_Check] Set New Forage Target: targ_last=%ddeg targ_new=%ddeg', ...
                D.PAR.frgTargDegArr(D.I.targ_last), D.PAR.frgTargDegArr(D.I.targ_now)));
            
        end
        
        % CHECK IF ROBOT SHOULD BE MOVED
        
        % Move flaged, target updated and min time ellapsed or rat
        % already moving from dish
        if  D.F.move_to_targ && ...
                D.I.targ_now ~= D.I.targ_last && ...
                (Sec_DT(now) > D.T.frg_rew+D.PAR.frgRewBlock || ...
                ~any(check_last_inbound))
            
            % Itterate move count
            D.C.move = D.C.move+1;
            
            % Send move command
            targ_rad = deg2rad(D.PAR.frgTargDegArr(D.I.targ_now));
            Send_CS_Com('M', D.C.move, targ_rad);
            
            % Reset flag
            D.F.move_to_targ = false;
            
            % Log/print
            Console_Write(sprintf('[Forage_Reward_Targ_Check] Sent Moved Command: targ=%ddeg dt_rew=%0.2fs', ...
                targ_rad, Sec_DT(now) - D.T.frg_rew));
            
        end
        
    end

% ----------------------------LAP CHECK----------------------------
    function Lap_Check()
        
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
        
        %% UPDATE PLOT HISTORY AND PRINT LAP TIME INFO
        
        % Save time
        lap_tim_ellapsed = Sec_DT(now) - D.T.lap_str;
        
        % Get average
        D.T.lap_tot = D.T.lap_tot + lap_tim_ellapsed;
        lap_tim_average = D.T.lap_tot / sum([D.C.lap_cnt{:}]);
        
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
        Safe_Set(D.UI.popLapTim, 'String', infstr);
        
        % Update plot history
        VT_Plot_Hist()
        
        % reset timer
        D.T.lap_str = Sec_DT(now);
        
        % Update UI
        Update_UI(0, 'limitrate');
        
    end

% --------------------------RUN TEST CODE--------------------------
    function Test_Run()
        
        % RUN LOCAL TEST FUNCTION
        
        % Bail if not test run
        if ~D.DB.isTestRun
            return
        end
        
        % Run simulated rat test
        if D.DB.t1_doSimRatTest
            SimRatTest();
        end
        
        % Run pid calibration test
        if D.DB.t2_doPidCalibrationTest
            PidCalibrationTest();
        end
        
        % Run pid calibration test
        if D.DB.t3_doVTCalibrationTest
            VTCalibrationTest();
        end
        
        % Run halt error test
        if D.DB.t4_doHaltErrorTest
            HaltErrorTest();
        end
        
        % Wall image IR timing test
        if D.DB.t5_doWallIRTimingTest
            WallIRTimingTest()
        end
        
        % Sync IR timing test
        if D.DB.t6_doSyncIRTimingTest
            SyncIRTimingTest()
        end
        
        % Hardware test
        if D.DB.t7_doRobotHardwareTest
            HardwareTest()
        end
        
        % Cube Battery test
        if D.DB.t8_doCubeBatteryTest
            CubeBatteryTest()
        end
        
        % SIMULATED RAT TEST
        function SimRatTest()
            
            % Bail if rat out
            if D.F.task_done || D.F.do_quit
                return
            end
            
            % Bail if robot not in place
            if c2m.('K').dat1 < 3
                return
            end
            
            % Bail if poll data not new
            if ~D.F.poll_new_vt
                return
            end
            
            % Wait for 5 sec after setup
            if Sec_DT(now) - D.T.rec_tim < 5
                return
            end
            
            % Check for reward
            if D.F.rewarding && D.DB.SIM.t_resumeRun == 0
                
                % Get resume time
                D.DB.SIM.t_resumeRun = Sec_DT(now) + D.DB.SIM.dtRewPause;
                
                % Store slider value and set to zero
                D.DB.SIM.SldVelLast = round(get(D.UI.sldSimVel, 'Value'));
                set(D.UI.sldSimVel, 'Value', 0)
            end
            
            % Check if time to stop pausing
            if Sec_DT(now) > D.DB.SIM.t_resumeRun && ...
                    D.DB.SIM.t_resumeRun > 0
                
                % Set slider back to pervious value
                set(D.UI.sldSimVel, 'Value', D.DB.SIM.SldVelLast)
                
                % Reset resume time
                D.DB.SIM.t_resumeRun = 0;
            end
            
            % Local vars
            cm = 0;
            vel_now = 0;
            
            % Start rat in start quad
            if ~D.DB.isTestStarted
                
                % Setup vars
                if D.PAR.sesTask == 'Forage'
                    
                    % Store current target
                    D.DB.SIM.TargAng = D.PAR.frgTargDegArr(D.I.targ_now);
                    
                    % Compute angle between start pos and targ
                    ang_str = rad2deg(mean(D.PAR.strQuadBnds));
                    ang_targ = D.DB.SIM.TargAng;
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
                    D.DB.SIM.RunRad = deg2rad(ang_new);
                    
                    % Set start button active
                    Safe_Set(D.UI.btnStart, 'Value', 1)
                    
                    % Store forage roh
                    roh_now = mean(D.P.frgRohBnd);
                else
                    % Store track roh
                    roh_now = mean(D.P.trackRohBnd);
                end
                
                % Initialize roh
                if D.PAR.sesTask == 'Track'
                    D.DB.SIM.RohLast = mean(D.P.trackRohBnd);
                else
                    D.DB.SIM.RohLast = 0;
                end
                
                % Initialize rad
                if D.PAR.sesTask == 'Track'
                    D.DB.SIM.RadLast = mean(D.PAR.strQuadBnds);
                else
                    D.DB.SIM.RadLast = 0;
                end
                
                % Initialzie other vars
                D.DB.SIM.VelLast = 0;
                D.DB.SIM.TSStart = Sec_DT(now);
                D.DB.SIM.TSLast = 0;
                
                % Get inital x/y
                xy_pos = Pol_2_VT(wrapTo2Pi(mean(D.PAR.strQuadBnds)), roh_now);
                D.DB.SIM.XY = reshape(xy_pos', 1, []);
                
                % Make UI stuff visible
                Safe_Set(D.UI.sldSimVel, ...
                    'Visible', 'on',...
                    'Enable', 'on');
                Safe_Set(D.UI.txtSimVel, ...
                    'Visible', 'on');
                Safe_Set(D.UI.txtVelTick, ...
                    'Visible', 'on');
                
                % Send test info to robot once
                Send_CS_Com('T', SYSTEST(1), 0, 0);
                
                % Close rat stream
                D.F.vt_rat_streaming = NlxCloseStream(D.NLX.vt_rat_ent) ~= 1;
                
                % Unset flag
                D.DB.isTestStarted = true;
                
            end
            
            % Get Cheetah time stamp
            if D.F.cheetah_running
                
                % Get time stamp
                [pass, ts_string] = Send_NLX_Cmd('-GetTimestamp', false);
                
                % Convert ts
                if pass == 1
                    ts_now = int64(str2double(ts_string)) - D.T.poll_str_nlx;
                else
                    return
                end
                
            else
                % Get TS from MATLAB
                ts_now = int64(ceil((Sec_DT(now) - D.DB.SIM.TSStart)*10^6));
            end
            
            % Compute dt(s) form ts(us)
            dt_sec = double(ts_now - D.DB.SIM.TSLast) / 10^6;
            
            % Bail if too little time ellapsed
            if dt_sec < (1/30) * 0.75
                return
            end
            
            % Store last time
            D.DB.SIM.TSLast = ts_now;
            
            % Get slider val
            sld_vel = round(get(D.UI.sldSimVel, 'Value'));
            Safe_Set(D.UI.txtSimVel, 'String', num2str(sld_vel));
            
            % Update vel if not halted, holding for setup or
            % waiting for robot to move
            if ...
                    D.F.rat_in && ...
                    ~D.F.halted && ...
                    Sec_DT(now) > D.T.frg_rew+D.PAR.frgRewBlock+2.5 && ...
                    Sec_DT(now) - D.T.task_str > 2
                
                % Check vel
                if D.DB.SIM.VelLast == sld_vel
                    % Hold velocity
                    vel_now = D.DB.SIM.VelLast;
                elseif D.DB.SIM.VelLast < sld_vel
                    % Accelerate
                    vel_now = D.DB.SIM.VelLast + (D.DB.SIM.MaxAcc*dt_sec);
                elseif D.DB.SIM.VelLast > sld_vel
                    % Deccelerate
                    vel_now = D.DB.SIM.VelLast - (D.DB.SIM.MaxDec*dt_sec);
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
                    rad_now = Rad_Diff(rad_diff, D.DB.SIM.RadLast, 'wrap');
                    D.DB.SIM.RadLast = rad_now;
                    
                    % Get side to side movement
                    roh_diff = cm/D.UI.arnRad * D.DB.SIM.SwayDir;
                    
                    % Add a little noise
                    roh_diff = roh_diff + ...
                        (rand(1,1)-0.5)*(diff(D.P.trackRohBnd)/2);
                    
                    % Bias toward center
                    min = 0.1;
                    max = 0.5;
                    scale = D.DB.SIM.RohLast-mean(D.P.trackRohBnd);
                    scale = abs(scale / (diff(D.P.trackRohBnd)/2));
                    %scale = abs(1 - scale);
                    scale = (max-min)*(scale) + min;
                    if scale > 1
                        scale = 1;
                    elseif scale <= 0
                        scale = 0.001;
                    end
                    roh_diff = roh_diff*scale;
                    roh_now = D.DB.SIM.RohLast + roh_diff;
                    
                    % Change direction
                    thresh = rand(1,1)*0.5;
                    if roh_now > D.P.trackRohBnd(2) - diff(D.P.trackRohBnd)*thresh
                        D.DB.SIM.SwayDir = -1;
                    elseif roh_now < D.P.trackRohBnd(1) + diff(D.P.trackRohBnd)*thresh
                        D.DB.SIM.SwayDir = 1;
                    end
                    
                    % Keep in bounds
                    if roh_now > D.P.trackRohBnd(2) || ...
                            roh_now < D.P.trackRohBnd(1)
                        
                        % Set to last roh
                        roh_now = D.DB.SIM.RohLast;
                    end
                    
                    % Save value
                    D.DB.SIM.RohLast = roh_now;
                    
                    % Convert rad back to cart
                    xy_pos = Pol_2_VT(rad_now, roh_now);
                    D.DB.SIM.XY = reshape(xy_pos', 1, []);
                    
                elseif D.PAR.sesTask == 'Forage'
                    
                    % Check for changed targ
                    if D.DB.SIM.TargAng ~= D.PAR.frgTargDegArr(D.I.targ_now)
                        
                        % Get current rad pos
                        [rad_start, ~] = VT_2_Pol(D.DB.SIM.XY);
                        
                        % Store target angle
                        ang_end = D.PAR.frgTargDegArr(D.I.targ_now);
                        
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
                        
                        % Store new running direction
                        D.DB.SIM.RunRad = deg2rad(ang_new);
                        
                        % Store new target
                        D.DB.SIM.TargAng = D.PAR.frgTargDegArr(D.I.targ_now);
                        
                        % Reset roh
                        D.DB.SIM.RohLast = 0;
                        
                        % Reset rad
                        D.DB.SIM.RadLast = 0;
                    end
                    
                    % Move along roh
                    roh_diff = cm / D.UI.arnRad;
                    
                    % Path width/edge in rad
                    path_wdth = 4*deg2rad(D.PAR.frgPathWdt);
                    
                    % Get side to side movement
                    rad_diff = cm/path_wdth * D.DB.SIM.SwayDir;
                    
                    % Add a little noise
                    rad_diff = rad_diff + ...
                        (rand(1,1)-0.5)*path_wdth;
                    
                    % Bias toward center
                    min = (path_wdth/2)*0.1;
                    max = (path_wdth/2)*0.9;
                    scale = abs(D.DB.SIM.RadLast / path_wdth);
                    
                    % Get to
                    
                    % Keep scale in range
                    scale = (max-min)*(scale) + min;
                    if scale > 1
                        scale = 1;
                    elseif scale <= 0
                        scale = 0.001;
                    end
                    rad_diff = rad_diff*scale;
                    
                    % Get total rad diff
                    rad_now = D.DB.SIM.RadLast + rad_diff;
                    
                    % Change direction
                    thresh = rand(1,1)*0.5;
                    if rad_now > path_wdth - path_wdth*thresh
                        D.DB.SIM.SwayDir = -1;
                    elseif rad_now < -1*path_wdth + path_wdth*thresh
                        D.DB.SIM.SwayDir = 1;
                    end
                    
                    % Keep in bounds
                    if rad_now > path_wdth || ...
                            rad_now < -1*path_wdth
                        
                        % Set to last rad
                        rad_now = D.DB.SIM.RadLast;
                    end
                    
                    % Store rad
                    D.DB.SIM.RadLast = rad_now;
                    
                    % Orient to current run direction
                    rad_now = ...
                        Rad_Sum(D.DB.SIM.RunRad, rad_now);
                    
                    % Convert to cart
                    %rad_now = D.DB.SIM.RunRad;
                    rad = rad_now;
                    rad = abs(rad - 2*pi);
                    rad = wrapToPi(rad);
                    
                    % Convert to cart
                    [x_norm,y_norm] = pol2cart(rad, roh_diff);
                    
                    % Scale to pixel space
                    x_diff = x_norm.*D.PAR.R;
                    y_diff = y_norm.*D.PAR.R;
                    
                    % Check if out of bounds
                    [~, roh_now] = VT_2_Pol([D.DB.SIM.XY(1)+x_diff, D.DB.SIM.XY(2)+y_diff]);
                    if roh_now < D.P.frgRohBnd(2)
                        D.DB.SIM.XY = [D.DB.SIM.XY(1)+x_diff, D.DB.SIM.XY(2)+y_diff];
                    end
                    
                    % Store roh
                    D.DB.SIM.RohLast = roh_now;
                    
                    % Store current targ ang
                    D.DB.SIM.TargAng = D.PAR.frgTargDegArr(D.I.targ_now);
                    
                end
                
            end
            
            % Send Pos data to CS
            Send_CS_Com('p', ts_now, D.DB.SIM.XY(1), D.DB.SIM.XY(2), 0, false);
            
            % Update simulated rat data
            D.DB.SIM.VelLast = vel_now;
            D.P.Rat.vtTS = ts_now;
            D.P.Rat.vtPos = single(D.DB.SIM.XY);
            D.P.Rat.vtNRecs = single(1);
            
            % Run VT_Proc('Rat');
            VT_Proc('Rat');
            
        end
        
        % PID CALIBRATION
        function PidCalibrationTest()
            
            % Send begin test if robot in place
            if ~D.DB.isTestStarted && ...
                    c2m.('K').dat1 == 3
                
                % Start pid test
                Send_CS_Com('T', SYSTEST(1), 0, 0);
                
                % Set flag
                D.DB.isTestStarted = true;
                
            end
            
        end
        
        % VT CALIBRATION
        function VTCalibrationTest()
            
            % Bail if poll data not new
            if ~D.F.poll_new_vt
                return
            end
            
            % Send begin test if robot in place
            if ~D.DB.isTestStarted && ...
                    c2m.('K').dat1 == 3
                
                % Compute end time
                D.DB.CALVT.RunEnd = Sec_DT(now) + D.DB.CALVT.RunDur*60;
                
                % Close rat stream
                D.F.vt_rat_streaming = NlxCloseStream(D.NLX.vt_rat_ent) ~= 1;
                
                % Trigger start
                D.P.Rat.rad = mean(D.PAR.strQuadBnds);
                D.P.Rat.roh = mean(D.P.trackRohBnd);
                D.P.Rat.vtNRecs = 1;
                
                % Start robot running
                Send_CS_Com('T', SYSTEST(1), 1, D.DB.CALVT.RobVel);
                
                % Set flag
                D.DB.isTestStarted = true;
                
                % Log/print
                Console_Write(sprintf('[Test_Run] Start VT Calibration Test: dt_run=%0.2fmin rob_vel=%dcm/sec', ...
                    D.DB.CALVT.RunDur, D.DB.CALVT.RobVel));
                
            end
            
            % Get new data
            if D.DB.isTestStarted && ...
                    Sec_DT(now) < D.DB.CALVT.RunEnd
                
                % Bail if no new recs
                if D.P.Rob.vtNRecs == 0
                    return
                end
                
                % Store processed data
                if D.F.Rob.new_pos_data
                    
                    % Add to history
                    ind_set_str = find(isnan(D.DB.CALVT.PosHist(1,:)), 1, 'first');
                    ind_set_end = ind_set_str + length(D.P.Rob.x) - 1;
                    D.DB.CALVT.PosHist(1, ind_set_str:ind_set_end) = D.P.Rob.x;
                    D.DB.CALVT.PosHist(2, ind_set_str:ind_set_end) = D.P.Rob.y;
                    
                    % Replot robot data
                    if isfield(D.UI, 'vtRobPltTest')
                        delete(D.UI.vtRobPltTest);
                    end
                    D.UI.vtRobPltTest = ...
                        line(D.DB.CALVT.PosHist(1,:), D.DB.CALVT.PosHist(2,:), ...
                        'LineStyle', 'none', ...
                        'Marker', 'o', ...
                        'MarkerFaceColor', [0.5 ,0.5, 0.5], ...
                        'MarkerEdgeColor', [0, 0, 0], ...
                        'MarkerSize', 5, ...
                        'HitTest', 'off', ...
                        'Parent', D.UI.axH(4));
                    
                end
                
                % Itterate count
                D.DB.CALVT.RecCnt = D.DB.CALVT.RecCnt + D.P.Rob.vtNRecs;
                
                % Store raw vt pos data
                ind_set_str = find(isnan(D.DB.CALVT.VTHist), 1, 'first');
                ind_set_end = ind_set_str + 2*D.P.Rob.vtNRecs - 1;
                D.DB.CALVT.VTHist(ind_set_str:ind_set_end) = D.P.Rob.vtPos;
                
            end
            
            % End test and save
            if D.DB.isTestStarted && ...
                    Sec_DT(now) >= D.DB.CALVT.RunEnd
                
                % SEND END COMMAND
                Send_CS_Com('T', SYSTEST(1), 2, 0);
                
                % COMPUTE ARENA VALUES
                
                % Remove empty entries
                D.DB.CALVT.VTHist = D.DB.CALVT.VTHist(~isnan(D.DB.CALVT.VTHist));
                
                % Reshape data
                xy_pos = reshape(double(D.DB.CALVT.VTHist),2,[])';
                
                % Exlude jumps
                exc_ind = [true; any(diff(xy_pos,1,1) > 10, 2)];
                exc_ind = exc_ind | [exc_ind(2:end); true];
                xy_pos(exc_ind, :) = [];
                
                % Exclude corner outlyers
                exc_ind = any(xy_pos == 0, 2);
                exc_ind = exc_ind | any(xy_pos == 480, 2);
                xy_pos(exc_ind, :) = [];
                
                % Save x/y pos samples in seperate vars
                x = xy_pos(:,1);
                y = xy_pos(:,2);
                
                % Rescale y as VT data is compressed in y axis
                y = y*11/10;
                
                % Fit circle to data
                [XC, YC, R] = circfit(x,y);
                
                % PLOT CORRECTED DATA
                
                % Hide other graphics
                Safe_Set(D.UI.axZoneH, 'Visible', 'off')
                set(get(D.UI.axZoneH(1),'Children'),'Visible','off')
                set(get(D.UI.axZoneH(2),'Children'),'Visible','off')
                set(get(D.UI.axH(2),'Children'),'Visible','off')
                
                % Copy real time axis
                ax_copy = copyobj(D.UI.axH(3), D.UI.tabICR);
                set(get(ax_copy,'Children'),'Visible','off')
                
                % Plot new values
                plot_new_h = ...
                    line(x, y, ...
                    'LineStyle', 'none', ...
                    'Marker', 'o', ...
                    'MarkerFaceColor', D.UI.activeCol, ...
                    'MarkerEdgeColor', [0,0,0], ...
                    'MarkerSize', 5, ...
                    'Parent', ax_copy);
                
                % Plot circle
                circ = [0:.01:2*pi,0];
                xout = sin(circ)*R + XC;
                yout = cos(circ)*R + YC;
                D.UI.linTrkH(1) = ...
                    line(xout, yout, ...
                    'color', [1 0 0], ...
                    'LineWidth', 1, ...
                    'Parent', ax_copy);
                
                % Check if user wants axes rescaled
                dlg_h = dlgAWL(...
                    'Rescale Axis?', ...
                    'TEST INFO', ...
                    'Yes', 'No', [], 'No', ...
                    D.UI.dlgPos{4}, ...
                    'question');
                choice = Dlg_Wait(dlg_h);
                
                % Rescale track plot axes
                if strcmp(choice, 'Yes')
                    
                    % Old ax pos
                    D.UI.vtRes
                    D.UI.lowLeft
                    
                    % Get new ax pos info
                    vt_res_new = round(R*2);
                    low_left_new = round([XC-R,YC-R]);
                    
                    % Rescale x lim
                    ax_copy.XLim = [low_left_new(1), low_left_new(1)+vt_res_new];
                    
                    % Rescale x lim
                    ax_copy.YLim = [low_left_new(2), low_left_new(2)+vt_res_new];
                    
                    % Set square
                    axis(ax_copy, 'square');
                end
                
                % Check if values should be saved
                dlg_h = dlgAWL(...
                    'Save VT Calibration?', ...
                    'TEST INFO', ...
                    'Yes', 'No', [], 'No', ...
                    D.UI.dlgPos{4}, ...
                    'question');
                choice = Dlg_Wait(dlg_h);
                
                % Delete plots
                delete(plot_new_h)
                delete(ax_copy)
                delete(D.UI.vtRobPltTest);
                
                % SAVE DATA
                if strcmp(choice, 'Yes')
                    
                    % Save track bound info
                    fi_path = D.DIR.trkBnds;
                    save(fi_path, 'XC', 'YC', 'R')
                    
                    % Save second copy with date
                    fi_name = sprintf('track_bounds_%s.mat',datestr(now, 'yymmdd'));
                    fi_path = regexprep(fi_path, 'track_bounds.mat', fi_name);
                    save(fi_path, 'XC', 'YC', 'R')
                    
                end
                
                % Unset test flag
                D.DB.t3_doVTCalibrationTest = false;
                D.DB.isTestRun = false;
                
            end
            
        end
        
        % HALT ERROR TEST
        function HaltErrorTest()
            
            % Check if robot should be restarted
            if D.DB.HALT.IsHalted && ...
                    ~D.F.halted && ...
                    Sec_DT(now) - D.DB.HALT.t_halted > D.DB.HALT.Dur
                
                % Save and print halt error
                % Store halt error
                halt_err = ...
                    Rad_Diff(D.P.Rob.radLast, D.DB.HALT.SendPos, 'min') * ((140*pi)/(2*pi));
                D.DB.HALT.error(1) = halt_err;
                D.DB.HALT.error(2) = min(D.DB.HALT.error(1), D.DB.HALT.error(2));
                D.DB.HALT.error(3) = min(999, max(D.DB.HALT.error(1), D.DB.HALT.error(3)));
                D.DB.HALT.error(4) = D.DB.HALT.error(1) + D.DB.HALT.error(4);
                D.DB.HALT.error(5) = D.DB.HALT.error(5) + 1;
                % Print halt error
                D.DB.HALT.error_str = [D.DB.HALT.error_str, ...
                    sprintf('\r%4.0f, %4.0f, %4.0f, %0.4f, %4.0f\r', ...
                    D.DB.HALT.error(1), D.DB.HALT.error(2), D.DB.HALT.error(3), D.DB.HALT.error(4)/D.DB.HALT.error(5), D.DB.HALT.NowVel)];
                % Log/print
                Console_Write(D.DB.HALT.error_str);
                
                % Check if vel should be stepped
                if D.DB.HALT.Cnt == D.DB.HALT.StepTrials
                    
                    % Reset counter
                    D.DB.HALT.Cnt = 0;
                    
                    % Incriment step
                    D.DB.HALT.NowStep = D.DB.HALT.NowStep+1;
                    
                    % Incriment vel
                    if D.DB.HALT.NowStep <= length(D.DB.HALT.VelSteps)
                        D.DB.HALT.NowVel = D.DB.HALT.VelSteps(D.DB.HALT.NowStep);
                    else
                        % End test
                        D.DB.HALT.NowVel = 0;
                        D.DB.t4_doHaltErrorTest = false;
                        
                        % Save data to csv file
                        fi_out = fullfile(D.DIR.ioTestOut,'halt_error.csv');
                        file_id = fopen(fi_out,'w');
                        fprintf(file_id,'Err, Min, Max, Avg, Vel\r');
                        fprintf(file_id,D.DB.HALT.error_str(3:end));
                        fclose(file_id);
                        % Log/print
                        Console_Write(sprintf('[Test_Run] Saved Halt Error Test: %s', ...
                            fi_out));
                    end
                    
                    % Log/print
                    Console_Write(sprintf('[Test_Run] Halt Error Test: new_vel=%dcm/sec', ...
                        D.DB.HALT.NowVel));
                    
                end
                
                % Tell CS to resume run
                Send_CS_Com('T', SYSTEST(1), 1, D.DB.HALT.NowVel);
                
                % Set flag
                D.DB.HALT.IsHalted = false;
                
                % Check if robot has stopped
            elseif ~D.DB.HALT.IsHalted && ...
                    D.P.Rob.vel < 5
                
                % Tell CS to resume run
                Send_CS_Com('T', SYSTEST(1), 1, D.DB.HALT.NowVel);
                
            end
            
            % Check if robot has passed 0 deg
            if ~D.DB.HALT.IsHalted && ...
                    Sec_DT(now) - D.DB.HALT.t_halted > D.DB.HALT.Dur+1 && ...
                    any(Check_Pol_Bnds(D.P.Rob.rad, D.P.Rob.roh, [deg2rad(355), deg2rad(360)]))
                
                % Incriment counter
                D.DB.HALT.Cnt = D.DB.HALT.Cnt+1;
                
                % Tell CS to Halt Robot
                Send_CS_Com('T', SYSTEST(1), 1, 0);
                
                % Store robots current pos and time
                D.DB.HALT.SendPos = D.P.Rob.radLast;
                D.DB.HALT.t_halted = Sec_DT(now);
                
                % Set flag
                D.DB.HALT.IsHalted = true;
                
                % Log/print
                Console_Write('[Test_Run] Halt Error Test: Halting Robot');
            end
            
        end
        
        % WALL IMAGE IR TIMING TEST
        function WallIRTimingTest()
            
            % BEGIN TEST
            if ~D.DB.isTestStarted
                
                % Set port direction
                Send_NLX_Cmd(['-SetDigitalIOportDirection ', D.NLX.DevTTL, ' ', '2', ' Output']);
                
                % Setup LED on event
                Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' 2 0 ', D.DB.WALIR.NLXevtStr]);
                
                % Setup pulse duration to 1/2 dt pulse
                Send_NLX_Cmd(['-SetDigitalIOPulseDuration ', D.NLX.DevTTL, ' 2 ', num2str(round(D.DB.WALIR.DTPulse/2))]);
                
                % Set flag
                D.DB.isTestStarted = true;
                
                % Format message
                msg = ...
                    sprintf('WALL IR IMAGE TEST STARTS IN %d SECONDS\r\nTrials=%d Image Delay=%dsec', ...
                    D.DB.WALIR.DTImg, D.DB.WALIR.imgTrials, D.DB.WALIR.DTImg);
                
                % Prompt that first test starting
                dlg_h = dlgAWL( ...
                    msg, ...
                    'TEST INFO', ...
                    'OK', [], [], 'OK', ...
                    D.UI.dlgPos{4}, ...
                    'default');
                Dlg_Wait(dlg_h);
                
                % Send setup command
                Send_CS_Com('T', SYSTEST(1), 0, 0);
                
                % Set time
                D.DB.WALIR.t_next = Sec_DT(now) + D.DB.WALIR.DTImg;
                
                % Bail
                return
            end
            
            % DO IR IMAGE TEST
            if D.DB.isTestStarted && ~D.DB.WALIR.isImgTestDone
                
                % Check if ready to switch image
                if Sec_DT(now) > D.DB.WALIR.t_next
                    
                    % Itterate trial count
                    D.DB.WALIR.cntEvent(1) = D.DB.WALIR.cntEvent(1)+1;
                    
                    % Check if done
                    if D.DB.WALIR.cntEvent(1) > D.DB.WALIR.imgTrials
                        
                        % Get round trip times
                        dt_mat = ...
                            [(1:D.DB.WALIR.imgTrials)', ...
                            ( D.DB.WALIR.t_trig - D.DB.WALIR.t_send)/1000];
                        
                        % Show as plot
                        fg = figure();
                        ax = axes();
                        hold on;
                        set(ax, 'YLim', [0, max(dt_mat(:))]);
                        set(fg, 'Position', FIGH.Position);
                        line(dt_mat(:,1), dt_mat(:,2:end), ...
                            'LineStyle', 'none', ...
                            'Marker', 'o', ...
                            'Parent', ax);
                        ax.YLabel.String = 'Round Trip (ms)';
                        ax.XLabel.String = 'Trial Number';
                        
                        % Create legend with average times
                        dt_stats = [mean(dt_mat(:,2:end),1)', ...
                            min(dt_mat(:,2:end),[],1)', ...
                            max(dt_mat(:,2:end),[],1)'];
                        leg_str = D.DB.WALIR.PTttlStr;
                        for z_ev = 1:4
                            leg_str{z_ev} = ...
                                sprintf('%s: mu=%0.0f, min=%0.0f, max=%0.0f', ...
                                leg_str{z_ev}, dt_stats(z_ev,1), dt_stats(z_ev,2), dt_stats(z_ev,3));
                            leg_str{z_ev} = regexprep(leg_str{z_ev}, '_', ' ');
                        end
                        legend(leg_str, 'FontName', 'MonoSpace');
                        
                        % Create mesage string
                        msg = [...
                            sprintf('FINISHED WALL IR IMAGE TEST: \r\n'), ...
                            sprintf('\r\n%d N=%0.0fms W=%0.0fms S=%0.0fms E=%0.0fms', ...
                            dt_mat')];
                        
                        % Show prompt and dt info
                        dlg_h = dlgAWL( ...
                            msg, ...
                            'TEST INFO', ...
                            'OK', [], [], 'OK', ...
                            D.UI.dlgPos{4}, ...
                            'default');
                        Dlg_Wait(dlg_h);
                        
                        % Close figure
                        if(isgraphics(fg))
                            close(fg);
                        end
                        
                        % Set flag
                        D.DB.WALIR.isImgTestDone = true;
                        
                        % Reset data
                        D.DB.WALIR.t_send = NaN(D.DB.WALIR.pulseTrials,4);
                        D.DB.WALIR.t_trig = NaN(D.DB.WALIR.pulseTrials,4);
                        D.DB.WALIR.cntEvent(:) = 0;
                        
                        % Format message
                        msg = ...
                            sprintf('WALL IR PULSE TEST STARTS IN %d SECONDS\r\nTrials=%d Pulse Delay=%dms Switch Time=%dsec', ...
                            D.DB.WALIR.DTSensor, D.DB.WALIR.pulseTrials, D.DB.WALIR.DTPulse, D.DB.WALIR.DTSensor);
                        
                        % Prompt that second test starting
                        dlg_h = dlgAWL( ...
                            msg, ...
                            'TEST INFO', ...
                            'OK', [], [], 'OK', ...
                            D.UI.dlgPos{4}, ...
                            'default');
                        Dlg_Wait(dlg_h);
                        
                        % Set time and sensor
                        D.DB.WALIR.t_next = Sec_DT(now) + D.DB.WALIR.DTSensor;
                        D.DB.WALIR.cntSensor = 1;
                        
                        % Bail
                        return
                    end
                    
                    % Get new image
                    D.I.rot = find([1, 2] ~=  D.I.rot);
                    
                    % Update D.AC.data(2) and send command to rotate image
                    D.AC.data(2) = D.I.img_ind(D.I.rot);
                    Send_AC_Com();
                    
                    % Get time stamp
                    [pass, ts_string] = Send_NLX_Cmd('-GetTimestamp', false);
                    
                    % Convert ts
                    if pass == 1
                        
                        % Get time stamp
                        ts = int64(str2double(ts_string)) - D.T.poll_str_nlx;
                        D.DB.WALIR.t_send(D.DB.WALIR.cntEvent(1), :) = ts;
                        
                        % Log/print
                        Console_Write(sprintf('[Test_Run] WALL IR IMAGE TEST: Trial=%d Img Change TS=%0.0fms', ...
                            D.DB.WALIR.cntEvent(1), ts/1000));
                    end
                    
                    % Switch image in x sec
                    D.DB.WALIR.t_next = Sec_DT(now) + D.DB.WALIR.DTImg;
                    
                end
                
                % Get new events
                if D.E.evtNRecs > 0
                    
                    % Check for trigger events
                    for z_t = 1:4
                        if any(ismember(D.E.evtStr, D.DB.WALIR.PTttlStr{z_t}))
                            
                            % Get time stamp
                            ts = D.E.evtTS(ismember(D.E.evtStr, D.DB.WALIR.PTttlStr{z_t}));
                            D.DB.WALIR.t_trig(D.DB.WALIR.cntEvent(1), z_t) = ts;
                            
                            % Log/print
                            Console_Write(sprintf('[Test_Run] WALL IR IMAGE TEST: Sensor=%s Trial=%d Trigger TS=%0.0fms', ...
                                D.DB.WALIR.PTttlStr{z_t}, D.DB.WALIR.cntEvent(1), ts/1000));
                            
                        end
                    end
                    
                end
                
                % Bail
                return
                
            end
            
            % DO IR PULSE TEST
            if D.DB.isTestStarted && ~D.DB.WALIR.isIRTestDone
                
                % Do next event
                if Sec_DT(now) > D.DB.WALIR.t_next && ...
                        all(diff(D.DB.WALIR.cntEvent) == 0)
                    
                    % Do next pulse
                    if D.DB.WALIR.cntEvent(1) < D.DB.WALIR.pulseTrials
                        
                        % Itterate count
                        D.DB.WALIR.cntEvent(1) = D.DB.WALIR.cntEvent(1)+1;
                        
                        % Set next pulse time
                        if D.DB.WALIR.cntEvent(1) < D.DB.WALIR.pulseTrials
                            
                            % Set to pulse dt
                            D.DB.WALIR.t_next = Sec_DT(now) + D.DB.WALIR.DTPulse/1000;
                            
                        else
                            
                            % Set to wait for move
                            D.DB.WALIR.t_next = Sec_DT(now) + D.DB.WALIR.DTSensor;
                            
                            % Make last pulse longer
                            Send_NLX_Cmd(['-SetDigitalIOPulseDuration ', D.NLX.DevTTL, ' 2 ', '2000']);
                            
                        end
                        
                        % Send pulse command
                        Send_NLX_Cmd(D.DB.WALIR.LEDttlStr);
                        
                        % Check if ready to switch sensor
                    else
                        
                        % Itterate sensor count
                        D.DB.WALIR.cntSensor = D.DB.WALIR.cntSensor+1;
                        
                        % Reset trial count
                        D.DB.WALIR.cntEvent(:) = 0;
                        
                        % Reset pulse
                        Send_NLX_Cmd(['-SetDigitalIOPulseDuration ', D.NLX.DevTTL, ' 2 ', num2str(round(D.DB.WALIR.DTPulse/2))]);
                        
                        
                    end
                    
                end
                
                % Get new events
                if D.E.evtNRecs > 0 && ...
                        D.DB.WALIR.cntSensor > 0 && ...
                        D.DB.WALIR.cntSensor <= 4 && ...
                        D.DB.WALIR.cntEvent(1) > 0 && ...
                        D.DB.WALIR.cntEvent(1) <= D.DB.WALIR.pulseTrials
                    
                    % Reset events
                    D.E.evtNRecs = 0;
                    
                    % Check for pulse event
                    if any(ismember(D.E.evtStr, D.DB.WALIR.NLXevtStr))
                        
                        % Itterate count
                        D.DB.WALIR.cntEvent(2) = D.DB.WALIR.cntEvent(2)+1;
                        
                        % Get time stamp
                        ts = D.E.evtTS(ismember(D.E.evtStr, D.DB.WALIR.NLXevtStr));
                        D.DB.WALIR.t_send(D.DB.WALIR.cntEvent(2), D.DB.WALIR.cntSensor) = ts;
                        
                        % Log/print
                        Console_Write(sprintf('[Test_Run] WALL IR PULSE TEST: Sensor=%s Trial=%d Pulse TS=%0.0fms', ...
                            D.DB.WALIR.PTttlStr{D.DB.WALIR.cntSensor}, D.DB.WALIR.cntEvent(2), ts/1000));
                    end
                    
                    % Check for trigger events
                    if any(ismember(D.E.evtStr, D.DB.WALIR.PTttlStr{D.DB.WALIR.cntSensor}))
                        
                        % Itterate count
                        D.DB.WALIR.cntEvent(3) = D.DB.WALIR.cntEvent(3)+1;
                        
                        % Get time stamp
                        ts = D.E.evtTS(ismember(D.E.evtStr, D.DB.WALIR.PTttlStr{D.DB.WALIR.cntSensor}));
                        D.DB.WALIR.t_trig(D.DB.WALIR.cntEvent(3), D.DB.WALIR.cntSensor) = ts;
                        
                        % Log/print
                        Console_Write(sprintf('[Test_Run] WALL IR PULSE TEST: Sensor=%s Trial=%d Trigger TS=%0.0fms', ...
                            D.DB.WALIR.PTttlStr{D.DB.WALIR.cntSensor}, D.DB.WALIR.cntEvent(3), ts/1000));
                    end
                end
                
                % Check if done
                if D.DB.WALIR.cntSensor > 4
                    
                    % Get round trip times
                    dt_mat = ...
                        [(1:D.DB.WALIR.pulseTrials)', ...
                        ( D.DB.WALIR.t_trig - D.DB.WALIR.t_send)/1000];
                    
                    % Show as plot
                    fg = figure();
                    ax = axes();
                    hold on;
                    set(ax, 'YLim', [0, max(max(dt_mat(:,2:end)))]);
                    set(fg, 'Position', FIGH.Position);
                    line(dt_mat(:,1), dt_mat(:,2:end), ...
                        'LineStyle', 'none', ...
                        'Marker', 'o', ...
                        'Parent', ax);
                    ax.YLabel.String = 'Sensor Delay (ms)';
                    ax.XLabel.String = 'Trial Number';
                    
                    % Create legend with average times
                    dt_stats = [mean(dt_mat(:,2:end),1)', ...
                        min(dt_mat(:,2:end),[],1)', ...
                        max(dt_mat(:,2:end),[],1)'];
                    leg_str = D.DB.WALIR.PTttlStr;
                    for z_ev = 1:4
                        leg_str{z_ev} = ...
                            sprintf('%s: mu=%0.2f, min=%0.2f, max=%0.2f', ...
                            leg_str{z_ev}, dt_stats(z_ev,1), dt_stats(z_ev,2), dt_stats(z_ev,3));
                        leg_str{z_ev} = regexprep(leg_str{z_ev}, '_', ' ');
                    end
                    legend(leg_str, 'FontName', 'MonoSpace');
                    
                    % Create mesage string
                    msg = [...
                        sprintf('FINISHED WALL IR PULSE TEST:\r\n'), ...
                        sprintf('\r\n%d N=%0.2fms W=%0.2fms S=%0.2fms E=%0.2fms', ...
                        dt_mat')];
                    
                    % Show prompt and dt info
                    dlg_h = dlgAWL( ...
                        msg, ...
                        'TEST INFO', ...
                        'OK', [], [], 'OK', ...
                        D.UI.dlgPos{4}, ...
                        'default');
                    Dlg_Wait(dlg_h);
                    
                    % Close figure
                    if(isgraphics(fg))
                        close(fg);
                    end
                    
                    % Set flag
                    D.DB.WALIR.isIRTestDone = true;
                    
                    % Bail
                    return
                    
                end
                
                % Bail
                return
            end
            
            % END OF TEST
            if D.DB.isTestStarted && ...
                    D.DB.WALIR.isIRTestDone && ...
                    D.DB.WALIR.isImgTestDone
                
                % Send end command
                Send_CS_Com('T', SYSTEST(1), 2, 0);
                
                % Unset flag
                D.DB.t5_doWallIRTimingTest = false;
                
                % Bail
                return
                
            end
            
        end
        
        % SYNC IR TIMING TEST
        function SyncIRTimingTest()
            
            % BEGIN TEST
            if ~D.DB.isTestStarted
                
                % Set port direction
                Send_NLX_Cmd(['-SetDigitalIOportDirection ', D.NLX.DevTTL, ' ', '2', ' Input']);
                
                % Setup robot input event
                Send_NLX_Cmd(['-SetNamedTTLEvent ', D.NLX.DevTTL, ' 2 0 ', D.DB.SYNCIR.NLXevtStr]);
                
                % Set flag
                D.DB.isTestStarted = true;
                
                % Format message
                msg = ...
                    sprintf('START SYNC IR TEST\r\n\r\nTrials=%d Pulse Delay=%dsec', ...
                    D.DB.SYNCIR.pulseTrials, D.DB.SYNCIR.DTPulse);
                
                % Prompt that first test starting
                dlg_h = dlgAWL( ...
                    msg, ...
                    'TEST INFO', ...
                    'OK', [], [], 'OK', ...
                    D.UI.dlgPos{4}, ...
                    'default');
                Dlg_Wait(dlg_h);
                
                % Send setup command
                Send_CS_Com('T', SYSTEST(1), 0, 0);
                
                % Bail
                return
            end
            
            % RUN TEST
            if D.DB.isTestStarted
                
                % Check if ready to switch image
                if Sec_DT(now) > D.DB.SYNCIR.t_next && ...
                        all(diff(D.DB.SYNCIR.cntEvent) == 0)
                    
                    % Itterate trial count
                    D.DB.SYNCIR.cntEvent(1) = D.DB.SYNCIR.cntEvent(1)+1;
                    
                    % Send next pulse command
                    if D.DB.SYNCIR.cntEvent(1) <= D.DB.SYNCIR.pulseTrials
                        
                        % Send command
                        Send_CS_Com('T', SYSTEST(1), 1, 0);
                        
                        % Get next pulse time
                        D.DB.SYNCIR.t_next = Sec_DT(now) + D.DB.SYNCIR.DTPulse/1000;
                    end
                    
                    % Check if done
                    if D.DB.SYNCIR.cntEvent(1) > D.DB.SYNCIR.pulseTrials
                        
                        % Get round trip times
                        dt_mat = ...
                            [(1:D.DB.SYNCIR.pulseTrials)', ...
                            ( D.DB.SYNCIR.t_trig - D.DB.SYNCIR.t_pulse)/1000];
                        
                        % Show as plot
                        fg = figure();
                        ax = axes();
                        hold on;
                        set(ax, 'YLim', [0, max(dt_mat(:,2:end))*1.5]);
                        set(fg, 'Position', FIGH.Position);
                        line(dt_mat(:,1), dt_mat(:,2:end), ...
                            'LineStyle', 'none', ...
                            'Marker', 'o', ...
                            'Parent', ax);
                        ax.YLabel.String = 'Sensor Delay (ms)';
                        ax.XLabel.String = 'Trial Number';
                        
                        % Create legend with average times
                        dt_stats = [mean(dt_mat(:,2),1)', ...
                            min(dt_mat(:,2),[],1)', ...
                            max(dt_mat(:,2),[],1)'];
                        leg_str = ...
                            sprintf('Signal DT: mu=%0.2f, min=%0.2f, max=%0.2f', ...
                            dt_stats(1), dt_stats(2), dt_stats(3));
                        legend(leg_str, 'FontName', 'MonoSpace');
                        
                        % Create mesage string
                        msg = 'FINISHED SYNC IR TEST';
                        
                        % Show prompt
                        dlg_h = dlgAWL( ...
                            msg, ...
                            'TEST INFO', ...
                            'OK', [], [], 'OK', ...
                            D.UI.dlgPos{4}, ...
                            'default');
                        Dlg_Wait(dlg_h);
                        
                        % Close figure
                        if(isgraphics(fg))
                            close(fg);
                        end
                        
                        % Bail
                        return
                    end
                    
                end
                
                % Get new events
                if D.E.evtNRecs > 0
                    
                    % Check for pulse event
                    if any(ismember(D.E.evtStr, D.NLX.ir_ts_str))
                        
                        % Itterate count
                        D.DB.SYNCIR.cntEvent(2) = D.DB.SYNCIR.cntEvent(2)+1;
                        
                        % Get time stamp
                        ts = D.E.evtTS(ismember(D.E.evtStr, D.NLX.ir_ts_str));
                        D.DB.SYNCIR.t_pulse(D.DB.SYNCIR.cntEvent(2)) = ts(1);
                        
                        % Log/print
                        Console_Write(sprintf('[Test_Run] SYNC IR TEST: Event=%s Trial=%d TS=%0.0fms', ...
                            D.NLX.ir_ts_str, D.DB.SYNCIR.cntEvent(2), ts/1000));
                    end
                    
                    % Check for robot event
                    if any(ismember(D.E.evtStr, D.DB.SYNCIR.NLXevtStr))
                        
                        % Itterate count
                        D.DB.SYNCIR.cntEvent(3) = D.DB.SYNCIR.cntEvent(3)+1;
                        
                        % Get time stamp
                        ts = D.E.evtTS(ismember(D.E.evtStr, D.DB.SYNCIR.NLXevtStr));
                        D.DB.SYNCIR.t_trig(D.DB.SYNCIR.cntEvent(3)) = ts(1);
                        
                        % Log/print
                        Console_Write(sprintf('[Test_Run] SYNC IR TEST: Event=%s Trial=%d TS=%0.0fms', ...
                            D.DB.SYNCIR.NLXevtStr, D.DB.SYNCIR.cntEvent(3), ts/1000));
                    end
                    
                end
                
                % Bail
                return
                
            end
            
        end
        
        % HARDWARE TEST
        function HardwareTest()
            
            % Bail if session not setup
            if ~D.F.ses_setup_done
                return
            end
            
            % BEGIN TEST
            
            % Prompt that test starting
            dlg_h = dlgAWL( ...
                'START HARDWARE TEST', ...
                'TEST INFO', ...
                'OK', [], [], 'OK', ...
                D.UI.dlgPos{4}, ...
                'default');
            Dlg_Wait(dlg_h);
            
            % Set delay
            dt_send = 2;
            
            % TEST REWARD RELATED PINS
            
            Send_NLX_Cmd('-PostEvent TEST:_Reward_Tone:_DUE_PIN 255 0');
            Send_CS_Com('T', SYSTEST(1), 1, 1);
            pause(dt_send)
            Send_NLX_Cmd('-PostEvent TEST:_White_Noise:_DUE_PIN 254 0');
            Send_CS_Com('T', SYSTEST(1), 1, 2);
            pause(dt_send)
            Send_NLX_Cmd('-PostEvent TEST:_Reward_Event_Only:_PORT_WORD 253 0');
            Send_CS_Com('T', SYSTEST(1), 1, 3);
            pause(dt_send)
            Send_NLX_Cmd('-PostEvent TEST:_Reward_With_Sound:_PORT_WORD 252 0');
            Send_CS_Com('T', SYSTEST(1), 1, 4);
            
            % TEST IR PINS
            Send_NLX_Cmd('-PostEvent TEST:_IR:_DUE_PIN 251 0');
            Send_CS_Com('T', SYSTEST(1), 1, 5);
            pause(dt_send)
            Send_NLX_Cmd('-PostEvent TEST:_IR:_PORT_WORD 250 0');
            Send_CS_Com('T', SYSTEST(1), 1, 6);
            pause(dt_send)
            
            % TEST PID PINS
            Send_NLX_Cmd('-PostEvent TEST:_PID:_DUE_PIN 249 0');
            Send_CS_Com('T', SYSTEST(1), 1, 7);
            pause(dt_send)
            
            % TEST BULL PINS
            Send_NLX_Cmd('-PostEvent TEST:_BULL:_DUE_PIN 249 0');
            Send_CS_Com('T', SYSTEST(1), 1, 8);
            pause(dt_send)
            
            % TEST PT PINS
            Send_NLX_Cmd('-PostEvent TEST:_PT:_DUE_PIN:_N_W_S_E 249 0');
            Send_CS_Com('T', SYSTEST(1), 1, 9);
            pause(dt_send)
            
            % Send test end
            Send_CS_Com('T', SYSTEST(1), 2);
            
            % Unset flag
            D.DB.t7_doRobotHardwareTest = false;
            
        end
        
        % CUBE BATTERY TEST
        function CubeBatteryTest()
            
            % Bail if vcc == 0
            if D.PAR.cube_vcc == 0
                return
            end
            
            % Setup
            if ~D.DB.isTestStarted
                
                % Set start time
                if isempty(D.DB.CVCC.t_start)
                    D.DB.CVCC.t_start = Sec_DT(now);
                end
                
                % Wait 10 seconds to actually start
                if Sec_DT(now) - D.DB.CVCC.t_start < 10
                    return
                end
                
                % Set start vcc and dt
                D.DB.CVCC.dt_step(1) = 0;
                D.DB.CVCC.vcc_step(1) = D.PAR.cube_vcc;
                
                % Set flag
                D.DB.isTestStarted = true;
                
                % Log/print
                Console_Write(sprintf('[Test_Run] CUBE "%s" BATTERY TEST: Cube VCC Starting at %d%%', ...
                    D.PAR.cubeBatteryType, D.DB.CVCC.vcc_step(1)));
            end
            
            % Check for change
            change_ind = ...
                find(D.PAR.cube_vcc < D.DB.CVCC.prcSteps & ...
                ~D.DB.CVCC.flagPrc, 1, 'first');
            
            % Display
            if ~isempty(change_ind)
                
                % Set flag
                D.DB.CVCC.flagPrc(change_ind) = true;
                
                % Play longer tone after all passed
                if all(D.DB.CVCC.flagPrc)
                    sig_dur = 5;
                else
                    sig_dur = 1;
                end
                
                % Play tone
                fmax = 4000;
                Fs = fmax*10;
                t = 0:1/Fs:sig_dur;
                y = mean([...
                    sin(2*pi*t*fmax*0.25); ...
                    sin(2*pi*t*fmax*0.5); ...
                    sin(2*pi*t*fmax)],1);
                soundsc(y,Fs)
                
                % Get dt
                D.DB.CVCC.dt_step(change_ind+1) = Sec_DT(now) - D.DB.CVCC.t_start;
                dt_str = datestr(D.DB.CVCC.dt_step(change_ind+1)/(24*60*60), 'MM:SS');
                
                % Store voltage
                D.DB.CVCC.vcc_step(change_ind+1) = D.PAR.cube_vcc;
                
                % Update plot
                set(D.DB.CVCC.fg, 'Visible', 'on')
                line(D.DB.CVCC.dt_step/60, D.DB.CVCC.vcc_step, ...
                    'color', [1 0 0], ...
                    'LineWidth', 1, ...
                    'Marker', 'o', ...
                    'Parent', D.DB.CVCC.ax);
                set(D.DB.CVCC.ax, 'XLim', [0,ceil(max(D.DB.CVCC.dt_step)/60)]);
                
                % Log/print
                Console_Write(sprintf('[Test_Run] CUBE "%s" BATTERY TEST: [%s] Cube VCC = %d%%', ...
                    D.PAR.cubeBatteryType, dt_str, D.DB.CVCC.vcc_step(change_ind+1)));
            end
            
            % Check for end
            if all(D.DB.CVCC.flagPrc)
                
                % Create mesage string
                msg = sprintf('FINISHED: CUBE "%s" BATTERY REACHED %d%% AFTER %0.2f MIN', ...
                    D.PAR.cubeBatteryType, D.DB.CVCC.vcc_step(end), D.DB.CVCC.dt_step(end)/60);
                
                % Show prompt and dt info
                dlg_h = dlgAWL( ...
                    msg, ...
                    'TEST INFO', ...
                    'OK', [], [], 'OK', ...
                    D.UI.dlgPos{4}, ...
                    'default');
                Dlg_Wait(dlg_h);
                
                % Close figure
                close(D.DB.CVCC.fg);
                
                % Unset flag
                D.DB.t8_doCubeBatteryTest = false;
                
            end
            
        end
        
    end






%% ============================== EXIT FUNCTIONS ==========================

% -----------------------SAVE GENERAL DATA--------------------------
    function Save_General_Data()
        
        % Get row ind
        if strcmp(D.SS_IO_3.(D.PAR.ratLab).Date{1}, '')
            % is first entry
            rowInd = 1;
        else
            rowInd = size(D.SS_IO_3.(D.PAR.ratLab), 1) + 1;
            % add new row at end of table
            D.SS_IO_3.(D.PAR.ratLab) = ...
                [D.SS_IO_3.(D.PAR.ratLab); D.SS_IO_3.(D.PAR.ratLab)(end,:)];
        end
        
        % Get date
        date = datestr(TIMSTRLOCAL, 'yyyy-mm-dd_HH-MM-SS');
        
        % Store 'Date'
        D.SS_IO_3.(D.PAR.ratLab).Date{rowInd} = date;
        
        % Store 'Human'
        D.SS_IO_3.(D.PAR.ratLab).Human(rowInd) = D.PAR.sesHuman;
        
        % Store 'Start_Time'
        D.SS_IO_3.(D.PAR.ratLab).Start_Time{rowInd} = datestr(TIMSTRLOCAL, 'HH:MM:SS');
        
        % Store 'Total_Time'
        D.SS_IO_3.(D.PAR.ratLab).Total_Time(rowInd) = Sec_DT(now) / 60;
        
        % Sore new weight info
        if  get(D.UI.toggUpdateWeight, 'Value') == 1
            D.SS_IO_3.(D.PAR.ratLab).Weight(rowInd) = D.PAR.ratWeightNew;
            D.SS_IO_3.(D.PAR.ratLab).Weight_Baseline(rowInd) = D.PAR.ratWeightBaseline;
            D.SS_IO_3.(D.PAR.ratLab).Weight_Drive(rowInd) = D.PAR.driveWeight;
            D.SS_IO_3.(D.PAR.ratLab).Weight_Cap(rowInd) = D.PAR.capWeight;
            D.SS_IO_3.(D.PAR.ratLab).Weight_Corrected(rowInd) = D.PAR.ratWeightCorrected;
            D.SS_IO_3.(D.PAR.ratLab).Weight_Proportion(rowInd) = D.PAR.ratWeightProportion;
            Console_Write('[Save_Weight_Data] FINISHED: Update Rat Weight');
        else
            % Carry over old weight info
            Console_Write('[Save_Weight_Data] SKIPPED: Update Rat Weight');
        end
        
        % Store feed info
        if  get(D.UI.toggUpdateFed, 'Value') == 1
            D.SS_IO_3.(D.PAR.ratLab).Fed_Pellets(rowInd) = D.PAR.feedPellets;
            D.SS_IO_3.(D.PAR.ratLab).Fed_Mash(rowInd) = D.PAR.feedMash;
            D.SS_IO_3.(D.PAR.ratLab).Fed_Ensure(rowInd) = D.PAR.feedEnsure;
            D.SS_IO_3.(D.PAR.ratLab).Fed_STAT(rowInd) = D.PAR.feedSTAT;
            Console_Write('[Save_Weight_Data] FINISHED: Update Rat Food');
        else
            % Set to zeros
            var_ind =  contains(D.SS_IO_3.(D.PAR.ratLab).Properties.VariableNames, 'Fed_');
            D.SS_IO_3.(D.PAR.ratLab){end, var_ind} = 0;
            Console_Write('[Save_Weight_Data] SKIPPED: Update Rat Food');
        end
        
        % Store health info
        var_ind = find(contains(D.UI.tblSSIO3.ColumnName,'Health'));
        for z_v = 1:length(var_ind)
            ss_ind = ...
                ismember(D.SS_IO_3.(D.PAR.ratLab).Properties.VariableNames, ...
                D.UI.tblSSIO3.ColumnName{var_ind(z_v)});
            D.SS_IO_3.(D.PAR.ratLab){end, ss_ind}(:) = ...
                D.UI.tblSSIO3.Data{end, var_ind(z_v)};
        end
        
        % Store 'Notes'
        D.SS_IO_3.(D.PAR.ratLab).Notes{rowInd} = D.UI.editGenNotes.String;
        
        % Save out data
        SS_IO_3 = D.SS_IO_3; %#ok<NASGU>
        save(D.DIR.SS_IO_3, 'SS_IO_3');
        
    end

% -------------------------SAVE TASK DATA---------------------------
    function [was_ran] = Save_Task_Data()
        
        % Switch to main tab for gui image capture
        Tab_GrpChange(D.UI.tabICR);
        pause(0.1)
        
        % Save GUI window image
        export_fig(FIGH, fullfile(D.DIR.nlxTempTop, D.DIR.recFi, 'GUI.png'));
        
        % Initialize output
        was_ran = false;
        
        % Bail if not ICR Session
        if D.PAR.sesType ~= 'ICR_Session'
            return
        end
        
        % Set flag
        was_ran = true;
        
        % Get row ind
        if strcmp(D.SS_IO_2.(D.PAR.ratLab).Date{1}, '')
            % is first entry
            rowInd = 1;
        else
            rowInd = size(D.SS_IO_2.(D.PAR.ratLab), 1) + 1;
            % add new row at end of table
            D.SS_IO_2.(D.PAR.ratLab) = ...
                [D.SS_IO_2.(D.PAR.ratLab); D.SS_IO_2.(D.PAR.ratLab)(end,:)];
        end
        
        % Get date
        date = datestr(TIMSTRLOCAL, 'yyyy-mm-dd_HH-MM-SS');
        
        % Store 'Include_Analysis'
        D.SS_IO_2.(D.PAR.ratLab).Include_Analysis(rowInd) = true;
        
        % Store 'Implanted'
        D.SS_IO_2.(D.PAR.ratLab).Implanted(rowInd) = D.F.rat_implanted;
        
        % Store 'Date'
        D.SS_IO_2.(D.PAR.ratLab).Date{rowInd} = date;
        
        % Store 'Recording File'
        if D.F.do_nlx_save
            D.SS_IO_2.(D.PAR.ratLab).Recording_File{rowInd} = D.DIR.recFi;
        else
            D.SS_IO_2.(D.PAR.ratLab).Recording_File{rowInd} = '';
        end
        
        % Store 'Human'
        D.SS_IO_2.(D.PAR.ratLab).Human(rowInd) = D.PAR.sesHuman;
        
        % Store 'VT_Pixel_Coordinates'
        D.SS_IO_2.(D.PAR.ratLab).VT_Pixel_Coordinates{rowInd} = [D.PAR.R, D.PAR.XC, D.PAR.YC];
        
        % Store 'Camera_Orientation'
        D.SS_IO_2.(D.PAR.ratLab).Camera_Orientation(rowInd) = -20;
        
        % Store 'Image_Orientation'
        if D.PAR.ratRotDrc == 'CW'
            D.SS_IO_2.(D.PAR.ratLab).Image_Orientation{rowInd} = [0, 40];
        else
            D.SS_IO_2.(D.PAR.ratLab).Image_Orientation{rowInd} = [0, -40];
        end
        
        % Store 'Start_Time'
        D.SS_IO_2.(D.PAR.ratLab).Start_Time{rowInd} = datestr(TIMSTRLOCAL, 'HH:MM:SS');
        
        % Store 'Total_Time'
        D.SS_IO_2.(D.PAR.ratLab).Total_Time(rowInd) = (D.T.ses_end - D.T.ses_str) / 60;
        
        % Store 'Sleep_Time'
        D.SS_IO_2.(D.PAR.ratLab).Sleep_Time(rowInd,:) = (D.T.sleep_end - D.T.sleep_str) / 60;
        
        % Store 'Session_Type'
        D.SS_IO_2.(D.PAR.ratLab).Session_Type(rowInd) = D.PAR.sesType;
        
        % Store 'Session_Condition'
        D.SS_IO_2.(D.PAR.ratLab).Session_Condition(rowInd) = D.PAR.sesCond;
        
        % Store 'Session_Task'
        D.SS_IO_2.(D.PAR.ratLab).Session_Task(rowInd) = D.PAR.sesTask;
        
        % Store 'Session_X' Number
        if ~isundefined(D.PAR.sesCond) && ~isundefined(D.PAR.sesTask)
            var_ind = ...
                ismember(D.SS_IO_2.(D.PAR.ratLab).Properties.VariableNames, ['Session_',char(D.PAR.sesCond)]);
            col_ind = ismember([{'Track'},{'Forage'}], D.PAR.sesTask);
            D.SS_IO_2.(D.PAR.ratLab){rowInd, var_ind}(col_ind) = D.PAR.sesNum;
        end
        
        % Store 'Feeder_Condition'
        D.SS_IO_2.(D.PAR.ratLab).Feeder_Condition(rowInd) = D.PAR.ratFeedCnd;
        
        % Store 'Rotation_Direction'
        D.SS_IO_2.(D.PAR.ratLab).Rotation_Direction(rowInd) = D.PAR.ratRotDrc;
        
        % Store 'Reward_Delay'
        D.SS_IO_2.(D.PAR.ratLab).Reward_Delay(rowInd) = char(D.PAR.sesRewDel);
        
        % Store 'Cue_Condition'
        D.SS_IO_2.(D.PAR.ratLab).Cue_Condition(rowInd) = char(D.PAR.sesCue);
        
        % Store 'Sound_Conditions'
        D.SS_IO_2.(D.PAR.ratLab).Sound_Conditions(rowInd,:) = D.F.sound;
        
        % Store 'PID_Setpoint'
        D.SS_IO_2.(D.PAR.ratLab).PID_Setpoint(rowInd) = D.PAR.setPointCM;
        
        % Store 'Start_Quadrant'
        D.SS_IO_2.(D.PAR.ratLab).Start_Quadrant(rowInd) = D.PAR.ratStrQuad;
        
        % Store 'Bulldozings'
        D.SS_IO_2.(D.PAR.ratLab).Bulldozings(rowInd) = D.C.bull_cnt;
        
        % Store 'Zones_Rewarded'
        D.SS_IO_2.(D.PAR.ratLab).Zones_Rewarded{rowInd} = ...
            D.PAR.zone_hist(1:find(~isnan(D.PAR.zone_hist),1,'last'));
        
        % Store 'Cued_Rewards'
        D.SS_IO_2.(D.PAR.ratLab).Cued_Rewards{rowInd} = D.PAR.cued_rew;
        
        % Store 'Rewards_Missed'
        D.SS_IO_2.(D.PAR.ratLab).Rewards_Missed(rowInd) = sum(D.C.missed_rew_cnt);
        
        % Store 'Rewards_Standard'
        D.SS_IO_2.(D.PAR.ratLab).Rewards_Standard{rowInd} = sum(D.C.rew_cnt{3});
        
        % Store 'Laps_Standard'
        D.SS_IO_2.(D.PAR.ratLab).Laps_Standard{rowInd} = sum(D.C.lap_cnt{3});
        
        % Store 'Notes'
        D.SS_IO_2.(D.PAR.ratLab).Notes{rowInd} = D.UI.editGenNotes.String;
        
        % STORE ROTATION SESSION SPECIFIC VARS
        
        % Store 'Rotations_Per_Session'
        if D.PAR.sesCond == 'Rotation'
            D.SS_IO_2.(D.PAR.ratLab).Rotations_Per_Session(rowInd) = D.C.rot_cnt;
        else
            D.SS_IO_2.(D.PAR.ratLab).Rotations_Per_Session(rowInd) = NaN;
        end
        
        % Store 'Rotation_Positions'
        if D.PAR.sesCond == 'Rotation'
            D.SS_IO_2.(D.PAR.ratLab).Rotation_Positions{rowInd} = D.PAR.rotPos;
        else
            D.SS_IO_2.(D.PAR.ratLab).Rotation_Positions(rowInd) = {[]};
        end
        
        % Store 'Laps_Per_Rotation'
        if D.PAR.sesCond == 'Rotation'
            
            if length(D.C.lap_cnt{1}) < length(D.C.lap_cnt{2})
                D.C.lap_cnt{1} = [D.C.lap_cnt{1},NaN];
            elseif length(D.C.lap_cnt{1}) > length(D.C.lap_cnt{2})
                D.C.lap_cnt{2} = [D.C.lap_cnt{2},NaN];
            end
            D.SS_IO_2.(D.PAR.ratLab).Laps_Per_Rotation{rowInd} = ...
                [D.C.lap_cnt{3}, reshape([[D.C.lap_cnt{2}]; [D.C.lap_cnt{1}]], 1, [])];
        else
            
            D.SS_IO_2.(D.PAR.ratLab).Laps_Per_Rotation(rowInd) = {[]};
        end
        
        % Store 'Days_Till_Rotation'
        if D.PAR.sesCond == 'Rotation'
            D.SS_IO_2.(D.PAR.ratLab).Days_Till_Rotation(rowInd) = D.PAR.daysTilRot;
        else
            D.SS_IO_2.(D.PAR.ratLab).Days_Till_Rotation(rowInd) = '<undefined>';
        end
        
        % Store 'Rewards_40_Deg'
        if D.PAR.sesCond == 'Rotation'
            D.SS_IO_2.(D.PAR.ratLab).Rewards_40_Deg{rowInd} = D.C.rew_cnt{2};
        else
            D.SS_IO_2.(D.PAR.ratLab).Rewards_40_Deg{rowInd} = 0;
        end
        
        % Store 'Laps_40_Deg'
        if D.PAR.sesCond == 'Rotation'
            D.SS_IO_2.(D.PAR.ratLab).Laps_40_Deg{rowInd} = D.C.lap_cnt{2};
        else
            D.SS_IO_2.(D.PAR.ratLab).Laps_40_Deg{rowInd} = 0;
        end
        
        % Store 'Rewards_0_Deg'
        if D.PAR.sesCond == 'Rotation'
            D.SS_IO_2.(D.PAR.ratLab).Rewards_0_Deg{rowInd} = D.C.rew_cnt{1};
        else
            D.SS_IO_2.(D.PAR.ratLab).Rewards_0_Deg{rowInd} = 0;
        end
        
        % Store 'Laps_0_Deg'
        if D.PAR.sesCond == 'Rotation'
            D.SS_IO_2.(D.PAR.ratLab).Laps_0_Deg{rowInd} = D.C.lap_cnt{1};
        else
            D.SS_IO_2.(D.PAR.ratLab).Laps_0_Deg{rowInd} = 0;
        end
        
        % UPDATE SS_IO_1
        
        % Update 'Session_Type'
        D.SS_IO_1.Session_Type(D.PAR.ratIndSS) = D.PAR.sesType;
        
        % Update 'Human'
        D.SS_IO_1.Human(D.PAR.ratIndSS) = D.PAR.sesHuman;
        
        % Update 'Session_Condition'
        D.SS_IO_1.Session_Condition(D.PAR.ratIndSS) = D.PAR.sesCond;
        
        % Update 'Session_Task'
        D.SS_IO_1.Session_Task(D.PAR.ratIndSS) = D.PAR.sesTask;
        
        % Update 'Reward_Delay'
        D.SS_IO_1.Reward_Delay(D.PAR.ratIndSS) = char(D.PAR.sesRewDel);
        
        % Update 'Cue_Condition'
        D.SS_IO_1.Cue_Condition(D.PAR.ratIndSS) = char(D.PAR.sesCue);
        
        % Update 'Sound_Conditions'
        D.SS_IO_1.Sound_Conditions(D.PAR.ratIndSS,:) = D.F.sound;
        Console_Write('[Save_Weight_Data] FINISHED: Update "SS_IO_1"');
        
        % Save out data
        SS_IO_2 = D.SS_IO_2; %#ok<NASGU>
        save(D.DIR.SS_IO_2, 'SS_IO_2');
        SS_IO_1 = D.SS_IO_1; %#ok<NASGU>
        save(D.DIR.SS_IO_1, 'SS_IO_1');
        
    end

% -------------------------SAVE TT TRACK DATA---------------------------
    function[was_ran] = Save_TT_Track_Data()
        
        % Initialize output
        was_ran = false;
        
        % Bail if not implant or TT_track session
        if ~D.F.implant_session
            return
        end
        
        % Set flag
        was_ran = true;
        
        % Store 'Session'
        D.TT.ttLogNew.Session = D.TT.Ses;
        
        % Store 'Date'
        D.TT.ttLogNew.Date = {datestr(TIMSTRLOCAL, 'yyyy-mm-dd_HH-MM-SS')};
        
        % Store 'Recording File'
        % Store 'Recording File'
        if D.F.do_nlx_save
            D.TT.ttLogNew.Recording_File{:} = D.DIR.recFi;
        else
            D.TT.ttLogNew.Recording_File{:} = '';
        end
        
        % Store 'Human'
        D.TT.ttLogNew.Human = D.PAR.sesHuman;
        
        % Store 'Notes'
        D.TT.ttLogNew.Notes{:} = D.UI.editTTNotes.String;
        
        % Save back to main table
        if D.TT.Ses == 1
            D.TT_IO_2.(D.PAR.ratLab) = D.TT.ttLogNew;
        else
            D.TT_IO_2.(D.PAR.ratLab) = [D.TT_IO_2.(D.PAR.ratLab); D.TT.ttLogNew];
        end
        
        % Save out data
        TT_IO_1 = D.TT_IO_1; %#ok<NASGU>
        save(D.DIR.TT_IO_1, 'TT_IO_1');
        TT_IO_2 = D.TT_IO_2; %#ok<NASGU>
        save(D.DIR.TT_IO_2, 'TT_IO_2');
        
    end

% -------------------------SAVE CHEETAH DATA---------------------------
    function[was_ran] = Save_Cheetah_Data()
        
        % Initialize output
        was_ran = false;
        
        % Bail if cheetah not running
        if ~D.F.cheetah_running
            return
        end
        
        % Disconect NetCom
        Disconnect_NLX()
        
        % Confirm that NLX Programs closed
        Console_Write('[Save_Cheetah_Data] RUNNNING: Wait for NLX Programs to Close..');
        while true
            
            % Check Cheetah.EXE status
            D.F.cheetah_running = Check_EXE('Cheetah.exe');
            
            % Check SpikeSort3D.EXE status
            D.F.spikesort_running = Check_EXE('SpikeSort3D.exe');
            
            % Check if all closed
            [abort, pass] = ...
                Check_Flag(DOEXIT, ...
                ~D.F.cheetah_running && ~D.F.spikesort_running);
            if abort || pass; break; end
            
            % Get message string
            if D.F.cheetah_running && D.F.spikesort_running
                msg = 'Close Cheetah & SpikeSort3D';
            elseif D.F.cheetah_running
                msg = 'Close Cheetah';
            elseif D.F.spikesort_running
                msg = 'Close SpikeSort3D';
            end
            
            % Prompt user to close programs
            dlg_h = dlgAWL( ...
                msg, ...
                'CLOSE PROGRAMS', ...
                'OK', [], [], 'OK', ...
                D.UI.dlgPos{4}, ...
                'default');
            Dlg_Wait(dlg_h);
            
            % Pause for 1 sec
            pause(1);
            
        end
        
        % Bail if cheetah still running
        if D.F.cheetah_running
            Console_Write('**WARNING** [Save_Cheetah_Data] ABORTED: Wait for NLX Programs to Close');
            return
        else
            Console_Write('[Save_Cheetah_Data] FINISHED: Wait for NLX Programs to Close');
        end
        
        % SAVE CHEETAH CONFIG FILE
        
        % Get config file path
        nlx_cfg_last = ...
            fullfile(D.DIR.nlxTempTop, D.DIR.recFi, 'ConfigurationLog', 'CheetahLastConfiguration.cfg');
        
        % Copy file to rat directory
        if exist(nlx_cfg_last, 'file')
            
            % Log print source path
            Console_Write(sprintf('[Save_Cheetah_Data] RUNNING: Copy Config File: "%s"...', nlx_cfg_last))
            
            % Get rat file path
            rat_path = fullfile(D.DIR.nlxRecRat, 'CheetahLastConfiguration.cfg');
            
            % Delete existing file
            if exist(rat_path, 'file')
                delete(rat_path);
            end
            
            % Copy file
            copyfile(nlx_cfg_last, rat_path);
            
            % Log print target path
            Console_Write(sprintf('[Save_Cheetah_Data] FINISHED: Copying Config File to: "%s"', rat_path))
            
        else
            Console_Write(sprintf('[Save_Cheetah_Data] SKIPPED: Copy Config File: "%s"...', nlx_cfg_last))
        end
        
        % Bail if not saving
        if ~D.F.do_nlx_save
            return
        end
        
        % COPY OVER CHEETAH DIRECTORY
        
        % Get file size
        fiGigs = dir(fullfile(D.DIR.nlxTempTop, D.DIR.recFi));
        fiGigs = sum([fiGigs.bytes])/10^9;
        
        % Wait for Cheetah to fully close
        pause(3);
        
        % Log/print start
        Console_Write(sprintf('[Save_Cheetah_Data] RUNNING: Copy Cheetah File...: file=%s size=%0.2fGB', ...
            D.DIR.recFi, fiGigs));
        
        % Save to global for CS
        m2c_dir = fullfile(D.DIR.nlxRecRat, D.DIR.recFi);
        Console_Write(sprintf('[Save_Cheetah_Data] SET RECORDING DIR TO "%s"', ...
            m2c_dir));
        
        % Copy file
        copyfile(fullfile(D.DIR.nlxTempTop, D.DIR.recFi), m2c_dir)
        
        % Log/print end
        Console_Write(sprintf('[Save_Cheetah_Data] FINISHED: Copy Cheetah File: file=%s size=%0.2fGB', ...
            D.DIR.recFi, fiGigs));
        
        % Set output and bail
        was_ran = true;
        
    end

% -----------------------------Disconnect AC-------------------------------
    function Disconnect_AC()
        
        % Bail if 'D' var deleted
        if ~exist('D','var')
            return;
        end
        
        % Bail if not connected
        if ~D.F.ac_connected
            Console_Write('[Disconnect_AC] SKIPPED: Disconnect from AC Computer...');
            return
        end
        
        % Log/print
        Console_Write('[Disconnect_AC] RUNNING: Disconnect from AC Computer...');
        
        % Disconnect from AC computer
        if isfield(D, 'AC')
            if ~isempty(D.AC)
                if exist('TCPIP', 'var')
                    if isa(TCPIP, 'tcpip')
                        if isvalid(TCPIP)
                            
                            % Pause to allow image to close
                            D.AC.data = zeros(1, length(D.AC.data));
                            D.AC.data(1) = 1;
                            Send_AC_Com();
                            pause(0.1);
                            
                            % Send command to terminate run
                            D.AC.data = zeros(1, length(D.AC.data));
                            Send_AC_Com();
                            
                            % Close AC computer connection
                            fclose(TCPIP);
                            
                            % Show status disconnected
                            Console_Write(sprintf('[Disconnect_AC] FINISHED: Disconnect from AC Computer IP=%s', ...
                                D.AC.IP));
                            
                        else
                            Console_Write('**WARNING** [Disconnect_AC] \"TCPIP\" Does Not Exist');
                        end
                    else
                        Console_Write('**WARNING** [Disconnect_AC] \"TCPIP\" is Not a TCPIP Object');
                    end
                else
                    Console_Write('**WARNING** [Disconnect_AC] \"TCPIP\" Does Not Exist');
                end
            else
                Console_Write('**WARNING** [Disconnect_AC] \"D.AC\" is Empty');
            end
        else
            Console_Write('**WARNING** [Disconnect_AC] \"AC\" Not a Field of \"D\"');
        end
        
        % Unset flag
        D.F.ac_connected = false;
        
    end

% ------------------------Disconnect From NetCom---------------------------
    function Disconnect_NLX()
        
        
        % Bail if vars/fields not present
        if ~exist('D','var')
            return;
        elseif ~isfield(D, 'NLX')
            return
        elseif isempty(D.NLX)
            return
        end
        
        % End NLX polling
        D.F.poll_nlx = false;
        
        % Bail if not connected
        if NlxAreWeConnected() ~= 1
            Console_Write('[Disconnect_NLX] SKIPPED: Disconnect from NLX...');
            return
        end
        
        % Log/print
        Console_Write('[Disconnect_NLX] RUNNING: Disconnect from NLX...');
        
        % Stop recording and aquisition
        Send_NLX_Cmd('-StopRecording');
        Send_NLX_Cmd('-StopAcquisition');
        pause(0.1);
        
        % Close VT stream
        if isfield(D.NLX, 'vt_rat_ent')
            
            % Close rat vt stream
            D.F.vt_rat_streaming = NlxCloseStream(D.NLX.vt_rat_ent) ~= 1;
            Console_Write(sprintf('[Disconnect_NLX] Close NLX "%s" Stream: status=%d', ...
                D.NLX.vt_rat_ent, ~D.F.vt_rat_streaming ));
            
            % Close robot vt stream
            D.F.vt_rob_streaming  = NlxCloseStream(D.NLX.vt_rob_ent) ~= 1;
            Console_Write(sprintf('[Disconnect_NLX] Close NLX "%s" Stream: status=%d', ...
                D.NLX.vt_rob_ent, ~D.F.vt_rob_streaming ));
        end
        
        % Close event stream
        if isfield(D.NLX, 'event_ent')
            D.F.evt_streaming = NlxCloseStream(D.NLX.event_ent) ~= 1;
            Console_Write(sprintf('[Disconnect_NLX] Close NLX "%s" Stream: status=%d', ...
                D.NLX.event_ent, ~D.F.evt_streaming));
        end
        
        % Close TT stream
        if Safe_Get(D.UI.toggLoadClust, 'Value') == 1
            
            % Unset stream tt and run callback
            Safe_Set(D.UI.toggLoadClust, 'Value', 0)
            Togg_LoadClust(D.UI.toggLoadClust);
        end
        
        % Disconnect from the NLX server
        while true
            NlxDisconnectFromServer();
            [abort, pass] = ...
                Check_Flag([], ~NlxAreWeConnected());
            if abort || pass; break; end
        end
        
        % Show status disconnected
        if pass
            Console_Write('[Disconnect_NLX] FINISHED: Disconnect from NLX');
        else
            Console_Write('**WARNING** [Disconnect_NLX] ABORTED: Disconnect from NLX');
        end
        
    end

% -----------------------------FORCE CLOSE---------------------------------
    function ForceClose()
        
        % Log/print
        Console_Write('**WARNING** [ForceClose] RUNNING: ForceClose Exit Procedure...');
        
        % Disconnect AC computer
        Disconnect_AC();
        % Disconnect from NetCom
        Disconnect_NLX()
        
        % Stop graphics related timers
        if strcmp(D.timer_graphics.Running, 'on')
            stop(D.timer_graphics);
        end
        if strcmp(D.timer_save.Running, 'on')
            stop(D.timer_save);
        end
        if strcmp(D.timer_quit.Running, 'on')
            stop(D.timer_quit);
        end
        
        % Set flags
        SetExit()
        FORCECLOSE = true;
        
        % Send force close signal to CS and set quit flag
        D.F.do_quit = true;
        Send_CS_Com('X', 2);
        pause(1);
        
        % Check if called after error
        e = lasterror;
        ISCRASH = ~isempty(e.message);
        
        % Clear and close
        if ISCRASH
            Console_Write('**WARNING** [ForceClose] MATLAB CRASHED: CLEARING AND CLOSING');
            ClearCloseAll(false)
        end
        
        return
    end

% -----------------------------SET TO EXIT---------------------------------
    function SetExit()
        
        % Bail if already set
        if exist('DOEXIT', 'var')
            if isa(DOEXIT, 'logical')
                if DOEXIT
                    return
                end
            end
        end
        
        % Set exit flag
        DOEXIT = true;
        
        % Log/print
        Console_Write('[SetExit] SET EXIT FLAG');
    end

% -------------------------CLEAR AND CLOSE ALL-----------------------------
    function ClearCloseAll(clear_all)
        
        % Stop/delete c2m timer
        if exist('D', 'var')
            if isfield(D, 'timer_c2m')
                if isa(D.timer_c2m, 'timer')
                    if isvalid(D.timer_c2m)
                        if strcmp(D.timer_c2m.Running, 'on')
                            stop(D.timer_c2m);
                            Console_Write('[ClearCloseAll] Stopped "timer_c2m"');
                        end
                        delete(D.timer_c2m);
                        Console_Write('[ClearCloseAll] Deleted "timer_c2m"');
                    end
                end
            end
        end
        
        % Stop/delete graphics timer
        if exist('D', 'var')
            if isfield(D, 'timer_graphics')
                if isa(D.timer_graphics, 'timer')
                    if isvalid(D.timer_graphics)
                        if strcmp(D.timer_graphics.Running, 'on')
                            stop(D.timer_graphics);
                        end
                        delete(D.timer_graphics);
                    end
                end
            end
        end
        
        % Stop/delete save button timer
        if exist('D', 'var')
            if isfield(D, 'timer_save')
                if isa(D.timer_save, 'timer')
                    if isvalid(D.timer_save)
                        if strcmp(D.timer_save.Running, 'on')
                            stop(D.timer_save);
                        end
                        delete(D.timer_save);
                    end
                end
            end
        end
        
        % Stop/delete quit button timer
        if exist('D', 'var')
            if isfield(D, 'timer_quit')
                if isa(D.timer_quit, 'timer')
                    if isvalid(D.timer_quit)
                        if strcmp(D.timer_quit.Running, 'on')
                            stop(D.timer_quit);
                        end
                        delete(D.timer_quit);
                    end
                end
            end
        end
        
        % Specify variables to be cleared
        vars = whos;
        vars_list = {vars.name};
        vars_list = vars_list([vars.global]);
        
        % Clear only a subset of global vars
        if ~clear_all
            vars_clear = {'D', 'TCPIP'};
            vars_exc = vars_list(~ismember(vars_list, vars_clear));
        end
        
        % Clear all global variables but return var
        if clear_all
            vars_exc = {'STATUS', 'c2m_com'};
        end
        
        % Log/print vars cleared
        Console_Write(['[ClearCloseAll] CLEARING ALL VARS BUT:', sprintf(' "%s"', vars_exc{:})]);
        
        % Clear variables
        clearvars('-global', '-except', vars_exc{:});
        
        % Close anything else
        close all;
        
    end






%% ========================== UI/GRAPHICS FUNCTIONS =======================

% -------------------------PRINT SES INFO--------------------------
    function Inf_Print()
        
        %% BAIL IF SETUP NOT FINISHED OR DT NOT REACHED
        if ...
                ~D.F.ses_setup_done || ...
                Sec_DT(now) - D.T.info_txt_update < 0.1
            return
        end
        
        % Store time
        t_now = Sec_DT(now);
        
        %% PRINT PERFORMANCE INFO
        
        % total
        infstr = sprintf([...
            'Laps______All:%s%d\n', ...
            'Rewards___All:%s%d\n', ...
            'Rotations_All:%s%d\n', ...
            'Missed_Rewards_:%s%d|%d\n',...
            'Bulldozings____:%s%d'], ...
            repmat('_',1,1), sum([D.C.lap_cnt{:}]), ...
            repmat('_',1,1), sum([D.C.rew_cnt{:}]), ...
            repmat('_',1,1), D.C.rot_cnt, ...
            repmat('_',1,1), D.C.missed_rew_cnt(1),sum(D.C.missed_rew_cnt), ...
            repmat('_',1,1), D.C.bull_cnt);
        if ~strcmp(D.UI.txtPerfInf(4).String, infstr)
            Safe_Set(D.UI.txtPerfInf(4), 'String', infstr)
        end
        
        % standard
        infstr = sprintf([...
            'Laps______Stand:%s%d\n', ...
            'Rewards___Stand:%s%d'], ...
            repmat('_',1,1), D.C.lap_cnt{3}(end), ...
            repmat('_',1,1), D.C.rew_cnt{3}(end));
        if ~strcmp(D.UI.txtPerfInf(1).String, infstr)
            Safe_Set(D.UI.txtPerfInf(1), 'String', infstr)
        end
        
        % 40 deg
        infstr = sprintf([...
            'Laps______40%c:%s%d|%d\n', ...
            'Rewards___40%c:%s%d|%d'], ...
            char(176), repmat('_',1,3), D.C.lap_cnt{2}(end), sum(D.C.lap_cnt{2}), ...
            char(176), repmat('_',1,3), D.C.rew_cnt{2}(end), sum(D.C.rew_cnt{2}));
        if ~strcmp(D.UI.txtPerfInf(2).String, infstr)
            Safe_Set(D.UI.txtPerfInf(2), 'String', infstr)
        end
        
        % 0 deg
        infstr = sprintf([...
            'Laps______0%c:%s%d|%d\n', ...
            'Rewards___0%c:%s%d|%d'], ...
            char(176), repmat('_',1,4), D.C.lap_cnt{1}(end), sum(D.C.lap_cnt{1}), ...
            char(176), repmat('_',1,4), D.C.rew_cnt{1}(end), sum(D.C.rew_cnt{1}));
        if ~strcmp(D.UI.txtPerfInf(3).String, infstr)
            Safe_Set(D.UI.txtPerfInf(3), 'String', infstr)
        end
        
        % Rat vel
        infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', ...
            D.P.Rat.vel, D.P.Rat.vel_max_lap, D.P.Rat.vel_max_all);
        if ~strcmp(D.UI.txtPerfInf(5).String, infstr)
            Safe_Set(D.UI.txtPerfInf(5), 'String', infstr)
        end
        
        % Robot vel
        infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', ...
            D.P.Rob.vel, D.P.Rob.vel_max_lap, D.P.Rob.vel_max_all);
        if ~strcmp(D.UI.txtPerfInf(6).String, infstr)
            Safe_Set(D.UI.txtPerfInf(6), 'String', infstr)
        end
        
        %% PRINT VOLTAGE INFO
        
        % Print robot voltage
        if D.PAR.rob_vcc ~= D.PAR.rob_vcc_last
            infstr = sprintf('Rob_Battery_:_%0.1fV', D.PAR.rob_vcc);
            Safe_Set(D.UI.txtPerfInf(7), 'String', infstr)
        end
        
        % Print cube battery voltage
        if D.F.cube_connected && ...
                D.F.implant_session && ...
                D.PAR.cube_vcc ~= D.PAR.cube_vcc_last
            
            infstr = sprintf('Cube_Battery:_%d%%', D.PAR.cube_vcc);
            Safe_Set(D.UI.txtPerfInf(8), 'String', infstr)
        end
        
        %% PRINT TIME INFO
        
        % Get session time
        nowTim(1) =  Sec_DT(now) - D.T.ses_str;
        
        % Get recording elapsed time plus saved time
        if  D.F.rec
            nowTim(2) = (Sec_DT(now) - D.T.rec_tim)  + D.T.rec_tot_tim;
        else
            nowTim(2) = D.T.rec_tot_tim; % keep showing save time
        end
        
        % Get lap time
        if D.F.rat_in
            nowTim(3) = Sec_DT(now) - D.T.lap_str;
        else
            nowTim(3) = 0;
        end
        
        % Run time
        if D.F.rat_in && ~D.F.task_done
            nowTim(4) = Sec_DT(now) - D.T.task_str;
        else
            nowTim(4) = 0;
        end
        
        % Make string
        infstr = sprintf([ ...
            'SES:%s%s\n', ...
            'REC:%s%s\n', ...
            'LAP:%s%s\n'...
            'RUN:%s%s\n'], ...
            repmat('_',1,1), datestr(nowTim(1)/(24*60*60), 'HH:MM:SS'), ...
            repmat('_',1,1), datestr(nowTim(2)/(24*60*60), 'HH:MM:SS'), ...
            repmat('_',1,1), datestr(nowTim(3)/(24*60*60), 'HH:MM:SS'), ...
            repmat('_',1,1), datestr(nowTim(4)/(24*60*60), 'HH:MM:SS'));
        
        % Print timers
        Safe_Set(D.UI.txtTimeInf(1), 'String', infstr)
        
        % Save current time to UserData
        Safe_Set(D.UI.txtTimeInf(1), 'UserData', nowTim)
        
        % Update info print time
        D.DB.inf = Update_DB_DT(D.DB.inf, t_now);
        
        %% PRINT DEBUG INFO
        
        % Update poll dt
        D.DB.pollevt = Update_DB_DT(D.DB.pollevt, D.T.poll_last_evt);
        D.DB.pollvt = Update_DB_DT(D.DB.pollvt, D.T.poll_last_vt);
        D.DB.polltt = Update_DB_DT(D.DB.polltt, D.T.poll_last_tt);
        
        % Loop time info
        infstr = sprintf( ...
            [...
            'pEv: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'pVT: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'pTT: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'Inf: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'Plt: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'Drw: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            'Lop: %4.0f  mn|mx:%4.0f|%4.0f av:%4.0f\n', ...
            ], ...
            D.DB.pollevt(1), D.DB.pollevt(2), D.DB.pollevt(3), D.DB.pollevt(4)/D.DB.pollevt(5), ...
            D.DB.pollvt(1), D.DB.pollvt(2), D.DB.pollvt(3), D.DB.pollvt(4)/D.DB.pollvt(5), ...
            D.DB.polltt(1), D.DB.polltt(2), D.DB.polltt(3), D.DB.polltt(4)/D.DB.polltt(5), ...
            D.DB.inf(1), D.DB.inf(2), D.DB.inf(3), D.DB.inf(4)/D.DB.inf(5), ...
            D.DB.plot(1), D.DB.plot(2), D.DB.plot(3), D.DB.plot(4)/D.DB.plot(5), ...
            D.DB.draw(1), D.DB.draw(2), D.DB.draw(3), D.DB.draw(4)/D.DB.draw(5), ...
            D.DB.loop(1), D.DB.loop(2), D.DB.loop(3), D.DB.loop(4)/D.DB.loop(5) ...
            );
        Safe_Set(D.UI.txtPerfInf(9), 'String', infstr)
        
        % Reset timers every 10 sec
        if Sec_DT(now) - D.T.info_db_reset > 10
            D.DB.pollevt = [0, inf, 0, 0, 0];
            D.DB.pollvt = [0, inf, 0, 0, 0];
            D.DB.polltt = [0, inf, 0, 0, 0];
            D.DB.inf = [0, inf, 0, 0, 0];
            D.DB.plot = [0, inf, 0, 0, 0];
            D.DB.draw = [0, inf, 0, 0, 0];
            D.DB.loop = [0, inf, 0, 0, 0];
            D.T.info_db_reset = Sec_DT(now);
        end
        
        % Update UI
        Update_UI(0, 'limitrate');
        
    end

% ------------------------SET CUE BUTTONS----------------------------------
    function Object_Group_State(set_case, setting_str)
        
        % Note:
        %
        %   set_case = [
        %               'Setup_Objects',
        %               'Sleep1_Objects',
        %               'Sleep2_Objects'
        %               'Task_Objects'
        %               'Text_Objects'
        %               'TT_Track_Objects',
        %               'TT_Plot_Objects'
        %               'TT_Turn_Panel_Objects' ]
        %
        %   setting_str = ['Enable', 'Disable']
        
        % Handle input
        switch set_case
            
            
            case 'Setup_Objects'
                %% SETUP OBJECTS
                
                % Set to 'Enable
                if strcmp(setting_str, 'Enable')
                    
                    if isundefined(D.PAR.sesType)
                        
                        % Enable session tybe objects
                        Safe_Set(D.UI.popType, 'Enable', 'on')
                        Button_State(D.UI.toggType, 'Enable')
                        
                        % Enable panel
                        Panel_State(D.UI.panStup, 'Enable');
                    end
                    
                    % Enable other setup objects
                    if ~isundefined(D.PAR.sesType)
                        
                        % Enable main setup objects
                        Safe_Set(D.UI.popRat, 'Enable', 'on')
                        Safe_Set(D.UI.popHuman, 'Enable', 'on')
                        Button_State(D.UI.toggSetupDone, 'Enable');
                        
                        % Enable Quit
                        Safe_Set(D.UI.toggQuit, 'Enable', 'on')
                        
                        % Disable ICR objectes
                        if D.PAR.sesType ~= 'ICR_Session'
                            Safe_Set(D.UI.popCond, 'Enable', 'off')
                            Safe_Set(D.UI.popTask, 'Enable', 'off')
                            Safe_Set(D.UI.popRewDel, 'Enable', 'off')
                            Button_State(D.UI.toggCue, 'Disable');
                            Button_State(D.UI.toggSnd, 'Disable');
                        end
                        
                        % Enable ICR objectes
                        if D.PAR.sesType == 'ICR_Session'
                            Safe_Set(D.UI.popCond, 'Enable', 'on')
                            Safe_Set(D.UI.popTask, 'Enable', 'on')
                            Safe_Set(D.UI.popRewDel, 'Enable', 'on')
                            Button_State(D.UI.toggCue, 'Enable');
                            Button_State(D.UI.toggSnd, 'Enable');
                            Panel_State(D.UI.panConsole, 'Enable');
                        end
                        
                    end
                    
                end
                
                % Set to 'Disable'
                if strcmp(setting_str, 'Disable')
                    
                    % Disable all setup stuff
                    Panel_State(D.UI.panStup, 'Disable');
                    Safe_Set(D.UI.popType, 'Enable', 'off')
                    Safe_Set(D.UI.popRat, 'Enable', 'off')
                    Safe_Set(D.UI.popHuman, 'Enable', 'off')
                    Safe_Set(D.UI.popCond, 'Enable', 'off')
                    Safe_Set(D.UI.popTask, 'Enable', 'off')
                    Safe_Set(D.UI.popRewDel, 'Enable', 'off')
                    Button_State(D.UI.toggCue, 'Disable');
                    Button_State(D.UI.toggSnd, 'Disable');
                    Button_State(D.UI.toggSetupDone, 'Disable');
                    
                end
                
                
            case 'Sleep1_Objects'
                %% SLEEP 1 OBJECTS
                
                % Set to 'Enable'
                if strcmp(setting_str, 'Enable')
                    
                    % Enable run panel
                    Panel_State(D.UI.panRun, 'Enable');
                    
                    % Enable Cheetah buttons
                    Button_State(D.UI.toggAcq, 'Enable');
                    Button_State(D.UI.toggRec, 'Enable');
                    
                    % Enable sleep 1 button and show attention color
                    Button_State(D.UI.toggSleep(1), 'Enable', D.UI.attentionCol);
                    
                end
                
                % Set to 'Disable'
                if strcmp(setting_str, 'Disable')
                    
                    % Set to active and Unenable sleep 1 button
                    D.UI.toggSleep(1).Value = 1;
                    Button_State(D.UI.toggSleep(1), 'Update');
                    Button_State(D.UI.toggSleep(1), 'Disable');
                    
                end
                
                
            case 'Sleep2_Objects'
                %% SLEEP 2 OBJECTS
                
                % Set to 'Enable'
                if strcmp(setting_str, 'Enable')
                    
                    % Reenable run panel
                    Panel_State(D.UI.panRun, 'Enable');
                    
                    % Renable Cheetah buttons
                    Button_State(D.UI.toggAcq, 'Enable');
                    Button_State(D.UI.toggRec, 'Enable');
                    
                    % Enable sleep 2 button and show in red
                    Button_State(D.UI.toggSleep(2), 'Enable', D.UI.attentionCol);
                    
                end
                
                % Set to 'Disable'
                if strcmp(setting_str, 'Disable')
                    
                    % Unenable run panel
                    Panel_State(D.UI.panRun, 'Disable');
                    
                    % Unnable Cheetah buttons
                    Button_State(D.UI.toggAcq, 'Disable');
                    Button_State(D.UI.toggRec, 'Disable');
                    
                    % Set to active and Unenable sleep 2 button
                    D.UI.toggSleep(2).Value = 1;
                    Button_State(D.UI.toggSleep(2), 'Update');
                    Button_State(D.UI.toggSleep(2), 'Disable');
                    
                end
                
                
            case 'Task_Objects'
                %% TASK OBJECTS
                
                % Set to 'Enable
                if strcmp(setting_str, 'Enable')
                    
                    % Enable Run panel
                    Panel_State(D.UI.panRun, 'Enable');
                    
                    % Enable Cheetah buttons
                    Button_State(D.UI.toggAcq, 'Enable');
                    Button_State(D.UI.toggRec, 'Enable');
                    
                    % Enable Reward button
                    Safe_Set(D.UI.popReward, ...
                        'Enable', 'on', ...
                        'BackgroundColor', D.UI.enabledCol);
                    Button_State(D.UI.btnReward, 'Enable');
                    
                    % Enable Halt robot
                    if D.PAR.sesCond ~= 'Manual_Training'
                        Button_State(D.UI.toggHaltRob, 'Enable');
                    end
                    
                    % Enable task done button
                    Button_State(D.UI.toggTaskDone, 'Enable');
                    
                    % Clear VT
                    Button_State(D.UI.btnClrVT, 'Enable');
                    
                    % Enable Track Task objects
                    if D.PAR.sesTask == 'Track' && ...
                            D.PAR.sesCond ~= 'Manual_Training'
                        
                        % Enable bulldoze pop menue
                        Safe_Set(D.UI.popBulldoze, 'Enable', 'on')
                        Button_State(D.UI.toggBulldoze, 'Enable');
                        Safe_Set(D.UI.editBulldoze, 'Enable', 'on')
                        
                        % Enable rotation buttons
                        if D.PAR.sesCond == 'Rotation'
                            
                            % Make second button active
                            Button_State(D.UI.toggICR(1), 'Disable', D.UI.rotCol(1,:));
                            Button_State(D.UI.toggICR(2), 'Enable', D.UI.rotCol(2,:));
                            
                        end
                        
                        % Enable target select button
                        Button_State(D.UI.toggPickRewPos, 'Enable');
                        
                        % Enable cue buttons
                        if D.PAR.sesCond ~= 'Manual_Training' && ...
                                D.PAR.sesCond ~= 'Rotation'
                            
                            Button_State(D.UI.toggDoCue, 'Enable');
                            Button_State(D.UI.toggBlockCue, 'Enable');
                            Button_State(D.UI.toggForceCue, 'Enable');
                            
                        end
                        
                    end
                    
                end
                
                % Set to 'Disable'
                if strcmp(setting_str, 'Disable')
                    
                    % Disable run panel
                    Panel_State(D.UI.panRun, 'Disable');
                    
                    % Disable Cheetah objects
                    Button_State(D.UI.toggAcq, 'Disable');
                    Button_State(D.UI.toggRec, 'Disable');
                    
                    % Disable run done button
                    Button_State(D.UI.toggTaskDone, 'Disable');
                    
                    % Disable ICR buttons
                    Button_State(D.UI.toggICR, 'Disable');
                    
                    % Disable robot objects
                    Button_State(D.UI.toggHaltRob, 'Disable');
                    Safe_Set(D.UI.popBulldoze, 'Enable', 'off')
                    Button_State(D.UI.toggBulldoze, 'Disable');
                    Safe_Set(D.UI.editBulldoze, 'Enable', 'off')
                    
                    % Disable reward buttons
                    Button_State(D.UI.btnReward, 'Disable');
                    Safe_Set(D.UI.popReward, 'Enable', 'off');
                    Button_State(D.UI.toggDoCue, 'Disable');
                    Button_State(D.UI.toggBlockCue, 'Disable');
                    Button_State(D.UI.toggForceCue, 'Disable');
                    Button_State(D.UI.toggPickRewPos, 'Disable');
                    
                    % Disable clear VT button
                    Button_State(D.UI.btnClrTT, 'Disable');
                    
                end
                
                
            case 'Text_Objects'
                %% TEXT INFO
                
                % Set to 'Enable'
                if strcmp(setting_str, 'Enable')
                    
                    % Text panels
                    Panel_State(D.UI.panSesInf, 'Enable');
                    Panel_State(D.UI.panPerfInf, 'Enable');
                    Panel_State(D.UI.panTimInf, 'Enable');
                    
                    % Session info
                    Safe_Set(D.UI.txtSesInf, 'Visible', 'on')
                    Safe_Set(D.UI.popLapsPerRot, 'Visible', 'on');
                    Safe_Set(D.UI.popRotPos, 'Visible', 'on');
                    if D.PAR.sesCond ~= 'Rotation'
                        Safe_Set(D.UI.popLapsPerRot, 'Enable', 'off');
                        Safe_Set(D.UI.popRotPos, 'Enable', 'off');
                    end
                    
                    % Performance info
                    Safe_Set(D.UI.txtPerfInf, 'Visible', 'on')
                    Safe_Set(D.UI.popLapTim, 'Visible', 'on')
                    Safe_Set(D.UI.popRewInfo, 'Visible', 'on')
                    
                    % Timer info
                    Safe_Set(D.UI.txtTimeInf, 'Visible', 'on')
                    Safe_Set(D.UI.editTimeInf, 'Visible', 'on')
                    
                end
                
                % Set to 'Disable'
                if strcmp(setting_str, 'Disable')
                    
                    % Text panels
                    Panel_State(D.UI.panSesInf, 'Disable');
                    Panel_State(D.UI.panPerfInf, 'Disable');
                    Panel_State(D.UI.panTimInf, 'Disable');
                    
                end
                
                
            case 'TT_Track_Objects'
                %% TT TRACK OBJECTS
                
                % Set to 'Enable
                if strcmp(setting_str, 'Enable')
                    
                    % Enable Run panel
                    Panel_State(D.UI.panRun, 'Enable');
                    
                    % Enable Cheetah buttons
                    Button_State(D.UI.toggAcq, 'Enable');
                    Button_State(D.UI.toggRec, 'Enable');
                    
                    % Enable tt select panel objects
                    Panel_State(D.UI.panSelectTT, 'Enable');
                    
                    % Show TT select buttons
                    Safe_Set(D.UI.toggSelectTT, 'Visible', 'on');
                    
                    % Disable excluded tt select buttons
                    Safe_Set(D.UI.toggSelectTT(all(D.F.tt_chan_disable,2)), 'Enable', 'off');
                    
                    % Enable tt action buttons
                    Button_State(D.UI.toggDoTrackTT, 'Enable');
                    
                    % Enable flag feature buttons
                    Safe_Set(D.UI.toggSubFlagTT, 'Enable', 'on');
                    
                    % Enable hear buttons
                    Safe_Set(D.UI.toggSubHearSdTT, 'Enable', 'on');
                    Safe_Set(D.UI.toggSubHearChTT, 'Enable', 'on');
                    
                    % Enable show/hide tt buttons
                    Button_State(D.UI.toggHideTT, 'Enable');
                    
                    % Enable Quit
                    Safe_Set(D.UI.toggQuit, 'Enable', 'on')
                    
                end
                
                % Set to 'Disable'
                if strcmp(setting_str, 'Disable')
                    
                    % Disable run panel
                    Panel_State(D.UI.panRun, 'Disable');
                    
                    % Disable Cheetah objects
                    Button_State(D.UI.toggAcq, 'Disable');
                    Button_State(D.UI.toggRec, 'Disable');
                    
                    % Disable tt select panel objects
                    Panel_State(D.UI.panSelectTT, 'Disable');
                    
                    % Hide tt select buttons
                    Safe_Set(D.UI.toggSelectTT, 'Visible', 'off');
                    
                    % Disable tt action buttons
                    Button_State(D.UI.toggDoTrackTT, 'Disable');
                    
                    % Disable flag feature buttons
                    Button_State(D.UI.toggSubFlagTT, 'Disable');
                    
                    % Disable hear tt buttons
                    Safe_Set(D.UI.toggSubHearSdTT, 'Enable', 'off');
                    Safe_Set(D.UI.toggSubHearChTT, 'Enable', 'off');
                    
                    % Disable show/hide tt buttons
                    Button_State(D.UI.toggHideTT, 'Disable');
                    
                end
                
                % Hack to get toggle color to updae
                for z_p = 1:2
                    pos = D.UI.panSelectTT(z_p).Position;
                    D.UI.panSelectTT(z_p).Position = [0,0,0.01,0.01];
                    D.UI.panSelectTT(z_p).Position = pos;
                end
                
                
            case 'TT_Plot_Objects'
                %% TT PLOT OBJECTS
                
                % Set to 'Enable
                if strcmp(setting_str, 'Enable')
                    
                    % Show/Enable Clear TT
                    set(D.UI.btnClrTT, 'Visible', 'on');
                    Button_State(D.UI.btnClrTT, 'Enable');
                    
                    % Enable TT plot action
                    Button_State(D.UI.toggDoPlotTT, 'Enable');
                    
                    % Enable TT plot type select
                    Button_State(D.UI.toggPlotTypeTT, 'Enable');
                    
                    % Show Color bars
                    Safe_Set(D.UI.linColBarH, 'Visible', 'on')
                    
                end
                
                % Set to 'Disable'
                if strcmp(setting_str, 'Disable')
                    
                    % Disable Clear TT
                    Button_State(D.UI.btnClrTT, 'Disable');
                    
                    % Unset active plotting
                    if D.UI.toggDoPlotTT.Value == 1
                        D.UI.toggDoPlotTT.Value = 0;
                        Togg_MainActionTT(D.UI.toggDoPlotTT);
                    end
                    
                    % Disable TT plot action
                    Button_State(D.UI.toggDoPlotTT, 'Disable');
                    
                    % Disable TT plot type select
                    Button_State(D.UI.toggPlotTypeTT, 'Disable');
                    
                    % Hide Color bars
                    Safe_Set(D.UI.linColBarH, 'Visible', 'off')
                    
                end
                
                % Hack to get toggle color to updae
                for z_p = 1:2
                    pos = D.UI.panSelectTT(z_p).Position;
                    D.UI.panSelectTT(z_p).Position = [0,0,0.01,0.01];
                    D.UI.panSelectTT(z_p).Position = pos;
                end
                
                
            case 'TT_Turn_Panel_Objects'
                %% TT TURN PANEL OBJECTS
                
                % Set to 'Enable
                if strcmp(setting_str, 'Enable')
                    
                    % Reset object values
                    Safe_Set(D.UI.popTrn, 'Value', 1);
                    Safe_Set(D.UI.popOr, 'Value', 1);
                    Safe_Set(D.UI.popDir, 'Value', 1);
                    Safe_Set(D.UI.popRefTT, 'Value', 1);
                    Safe_Set(D.UI.editImpTT,'String','');
                    Safe_Set(D.UI.editNoteTT,'String','');
                    D.UI.tblNoteTT.Data = [];
                    D.UI.tblNoteTT.RowName = [];
                    
                    % Reset pannel title
                    Safe_Set(D.UI.panTrackTT, 'Title', 'TTXX')
                    
                    % Reset orientation list
                    Safe_Set(D.UI.popOr, 'String', {' '});
                    
                    % Reset TT update button string
                    Safe_Set(D.UI.toggUpdateLogTT, ...
                        'String', 'Update XX');
                    
                    % Update tt depth/orientation text object
                    Safe_Set([D.UI.txtPanTT(8),D.UI.txtPanTT(9)], ...
                        'String', 'XX', ...
                        'ForegroundColor', D.UI.disabledCol);
                    
                    % Enable objects
                    Safe_Set(D.UI.popOr, 'Enable', 'on');
                    Safe_Set(D.UI.popTrn, 'Enable', 'on');
                    Safe_Set(D.UI.popDir, 'Enable', 'on');
                    Safe_Set(D.UI.popRefTT, 'Enable', 'on');
                    Safe_Set(D.UI.editImpTT, 'Enable', 'on');
                    Safe_Set(D.UI.editNoteTT, 'Enable', 'on');
                    Button_State(D.UI.toggUpdateLogTT, 'Enable');
                    
                end
                
                % Set to 'Disable'
                if strcmp(setting_str, 'Disable')
                    
                    % Set panel to disabled
                    Panel_State(D.UI.panTrackTT, 'Disable');
                    
                    % Disable other objects
                    Safe_Set(D.UI.txtPanTT, 'ForegroundColor', D.UI.disabledCol)
                    Safe_Set(D.UI.popOr, 'Enable', 'off');
                    Safe_Set(D.UI.popTrn, 'Enable', 'off');
                    Safe_Set(D.UI.popDir, 'Enable', 'off');
                    Safe_Set(D.UI.popRefTT, 'Enable', 'off');
                    Safe_Set(D.UI.editImpTT, 'Enable', 'off');
                    Safe_Set(D.UI.editNoteTT, 'Enable', 'off');
                    
                end
                
                % Hack to get toggle color to updae
                for z_p = 1:2
                    pos = D.UI.panSelectTT(z_p).Position;
                    D.UI.panSelectTT(z_p).Position = [0,0,0.01,0.01];
                    D.UI.panSelectTT(z_p).Position = pos;
                end
                
        end
        
    end

% -----------------------SET GRAPHICS HANDLE-------------------------------
    function Safe_Set(varargin)
        
        % Get graphics handles
        hand = varargin{1}(isgraphics(varargin{1}));
        set(hand, varargin{2:end});
        
    end

% -----------------------GET GRAPHICS HANDLE-------------------------------
    function [arg_out] = Safe_Get(hand, arg_str)
        
        % Get values for graphics objects
        inc_ind = isgraphics(hand);
        inc_ind = (inc_ind(:));
        get_arr = get(hand(inc_ind),arg_str);
        
        % Bail if scalar
        if length(get_arr) <= 1
            arg_out = get_arr;
            return
        end
        
        % Preset correct data type
        if isa(get_arr{1}, 'double')
            arg_out = zeros(numel(hand),1);
            get_arr = [get_arr{:}];
        elseif isa(get_arr{1}, 'char')
            arg_out = cellstr(repmat('',numel(hand),1));
        end
        
        % Store values in correct sequence
        arg_out(inc_ind) = get_arr;
        
    end

% ------------------------SET BUTTON STATE---------------------------------
    function Button_State(hand, setting_str, col_bck_in)
        % Note:
        %   setting_str = ['Enable', 'Disable', 'Update']
        
        % Handle inputs
        if nargin < 3
            col_bck_in = [];
        end
        
        % Remove none graphics objects
        hand = hand(:);
        hand = hand(isgraphics(hand));
        
        % Bail if no handles
        if isempty(hand)
            return
        end
        
        % Loop through each handle
        for z_h = 1:length(hand)
            
            % Bail if setting to 'Update' or 'Disable' and 'Enable' = 'off'
            if (strcmp(setting_str, 'Update') || strcmp(setting_str, 'Disable')) && ...
                    strcmp(get(hand(z_h), 'Enable'), 'off')
                continue
            end
            
            % Set Enable = 'on' if doing 'Enable'
            if strcmp(get(hand(z_h), 'Enable'), 'off')
                set(hand(z_h), 'Enable', 'on')
            end
            
            % Do Disable
            if strcmp(setting_str, 'Disable')
                
                % Get backround color
                if ~isempty(col_bck_in)
                    
                    % Set to special color
                    col_bck = col_bck_in;
                    
                elseif hand(z_h).Value == 1
                    
                    % Set active button to duller 'activeCol'
                    col_bck = rgb2hsv(D.UI.activeCol);
                    col_bck = hsv2rgb([col_bck(1), 0.5, col_bck(3)]);
                    
                else
                    
                    % Set to disabled color
                    col_bck = D.UI.disabledCol;
                end
                
                % Set color
                set(hand(z_h), ...
                    'BackgroundColor', col_bck, ...
                    'ForegroundColor', D.UI.disabledBtnFrgCol)
                
                % Set Enable = 'off'
                set(hand(z_h), 'Enable', 'off')
            end
            
            % Set to 'Enable' or 'Update'
            if strcmp(setting_str, 'Enable') || strcmp(setting_str, 'Update')
                
                % Check if button active
                if get(hand(z_h), 'Value') == 1
                    
                    % Get backround color
                    if isempty(col_bck_in)
                        col_bck = D.UI.activeCol;
                    else
                        col_bck = col_bck_in;
                    end
                    
                    % Get foreground color
                    col_for = get(hand(z_h), 'ForegroundColor');
                    if all(col_for == D.UI.enabledBtnFrgCol) || ...
                            all(col_for == D.UI.disabledBtnFrgCol)
                        
                        col_for = D.UI.enabledBtnFrgCol;
                    end
                    
                    % Set values
                    set(hand(z_h),  ...
                        'Visible', 'on', ...
                        'BackgroundColor', col_bck, ...
                        'ForegroundColor' , col_for);
                    
                else
                    % Get backround color
                    if isempty(col_bck_in)
                        col_bck = D.UI.enabledCol;
                    else
                        col_bck = col_bck_in;
                    end
                    
                    % Get foreground color
                    col_for = get(hand(z_h), 'ForegroundColor');
                    if all(col_for == D.UI.enabledBtnFrgCol) || ...
                            all(col_for == D.UI.disabledBtnFrgCol)
                        
                        col_for = D.UI.enabledBtnFrgCol;
                    end
                    
                    % Set values
                    set(hand(z_h), ...
                        'BackgroundColor', col_bck, ...
                        'ForegroundColor' , col_for);
                end
                
            end
            
            % Hack to get button to update
            pos = hand(z_h).Position;
            hand(z_h).Position = [0,0,0.1,0.1];
            hand(z_h).Position = pos;
            
        end
        
    end

% ------------------------SET PATCH STATE----------------------------------
    function Patch_State(hand, setting_str, col)
        % Note:
        %   setting_str = ['Hide', 'ShowPartial', 'ShowAll', 'Select', 'Active']
        
        % Handle inputs
        if nargin < 3
            col = [0,0,0];
        end
        
        % Remove none graphics objects
        hand = hand(:);
        hand = hand(isgraphics(hand));
        
        % Bail if no handles
        if isempty(hand)
            return
        end
        
        % Loop through each handle
        for z_h = 1:length(hand)
            
            % Set default color
            set(hand(z_h), ...
                'EdgeColor', col, ...
                'FaceColor', col);
            
            % Set to 'Hide'
            if strcmp(setting_str, 'Hide')
                set(hand(z_h), ...
                    'EdgeAlpha', 0, ...
                    'FaceAlpha', 0.1, ...
                    'LineWidth', 1, ...
                    'Visible', 'off');
            end
            % Set to 'ShowPartial'
            if strcmp(setting_str, 'ShowPartial')
                set(hand(z_h), ...
                    'EdgeAlpha', 0, ...
                    'FaceAlpha', 0.05, ...
                    'LineWidth', 1, ...
                    'Visible', 'on');
            end
            % Set to 'ShowAll'
            if strcmp(setting_str, 'ShowAll')
                set(hand(z_h), ...
                    'EdgeAlpha', 0.1, ...
                    'FaceAlpha', 0.1, ...
                    'LineWidth', 1, ...
                    'Visible', 'on');
            end
            % Set to 'Select'
            if strcmp(setting_str, 'Select')
                set(hand(z_h), ...
                    'EdgeAlpha', 0.1, ...
                    'FaceAlpha', 0.5, ...
                    'LineWidth', 1, ...
                    'Visible', 'on');
            end
            % Set to 'Active'
            if strcmp(setting_str, 'Active')
                set(hand(z_h), ...
                    'EdgeAlpha', 0.5, ...
                    'FaceAlpha', 0.5, ...
                    'EdgeColor', D.UI.activeCol, ...
                    'LineWidth', 3, ...
                    'Visible', 'on');
            end
            
        end
        
    end

% ------------------------SET PANEL STATE----------------------------------
    function Panel_State(hand, setting_str)
        % Note:
        %   setting_str = ['Enable', 'Disable']
        
        % Remove none graphics objects
        hand = hand(:);
        hand = hand(isgraphics(hand));
        
        % Bail if no handles
        if isempty(hand)
            return
        end
        
        % Loop through each handle
        for z_h = 1:length(hand)
            
            % Set to 'Enable'
            if strcmp(setting_str, 'Enable')
                
                % Set values
                set(hand(z_h), ...
                    'Visible', 'on', ...
                    'ForegroundColor', D.UI.enabledCol, ...
                    'HighlightColor', D.UI.enabledCol)
                
            end
            
            % Set 'Disable'
            if strcmp(setting_str, 'Disable')
                
                % Set values
                set(hand(z_h), ...
                    'Visible', 'on', ...
                    'BackgroundColor', D.UI.figBckCol, ...
                    'ForegroundColor', D.UI.disabledCol, ...
                    'HighlightColor',D.UI.disabledCol)
                
            end
            
        end
        
    end

% ------------------------SET CUE BUTTONS----------------------------------
    function Cue_Button_State(setting_str)
        
        if D.PAR.sesCue == 'Half' || D.PAR.sesCue == 'None'
            
            % Set to enabled color and change user data
            if strcmp(setting_str, 'Enable')
                
                % Set to 'Enable'
                Button_State(D.UI.toggBlockCue, 'Enable');
                Button_State(D.UI.toggForceCue, 'Enable');
                Button_State(D.UI.toggDoCue, 'Enable');
                
                % Reenable pick reward pos
                if get(D.UI.toggDoCue, 'Value') == 1
                    Button_State(D.UI.toggPickRewPos, 'Enable');
                end
                
                % Set to disabled color
            elseif strcmp(setting_str, 'Disable')
                
                % Set to 'Disable'
                Button_State(D.UI.toggBlockCue, 'Disable');
                Button_State(D.UI.toggForceCue, 'Disable');
                Button_State(D.UI.toggDoCue, 'Disable');
                
                % Unset pick reward pos
                Safe_Set(D.UI.toggPickRewPos, 'Value', 0)
                Button_State(D.UI.toggPickRewPos, 'Disable');
                
            end
            
            % Log/print
            Update_Log(sprintf('[Cue_Button_State] Set Cue Buttons to \"%s\"', setting_str));
        end
    end

% ----------------------PLOT REWARD ZONE HIST------------------------------
    function [hand_arr] = Plot_Zone_Hist(x_arr, y_arr, offset, c, alph, ax, hand_arr)
        % NOTE:
        %       Addapted from "createPatches.m"
        
        % Handle inputs
        if nargin < 7
            hand_arr = gobjects(1, length(x_arr));
        end
        
        % Loop through each back
        for z_p = 1:length(x_arr)
            
            % Left Boundary of x
            leftX = x_arr(z_p) - offset;
            
            % Right Boundary of x
            rightX = x_arr(z_p) + offset;
            
            % Get patch bounds
            x = [leftX rightX rightX leftX];
            y = [0 0 y_arr(z_p) y_arr(z_p)];
            
            % Add or update patch
            if ~isgraphics(hand_arr(z_p))
                hand_arr(z_p) = patch(x, y, c, ...
                    'FaceAlpha', alph, ...
                    'Parent', ax);
            else
                set(hand_arr(z_p), ...
                    'FaceAlpha', alph, ...
                    'XData', x, ...
                    'YData', y);
            end
            
        end
        
    end

% ----------------------------- PLOT TT PATHS -----------------------------
    function Plot_TT_Path(tt_ind)
        
        % Get tt data
        tt_fld = D.TT.ttLab{tt_ind};
        bndl_ind = D.I.ttBndl(tt_ind);
        
        % Pull out all depths for this tt
        depths_old = D.TT_IO_2.(D.PAR.ratLab).([tt_fld,'_D'])(1:end);
        depths_old_mm = depths_old/1000; % convert to mm
        
        % Append new depth if tt has been updated
        if D.F.tt_updated(tt_ind)
            depths_new = D.TT.ttLogNew.([tt_fld,'_D']);
            depths_new_mm = depths_new/1000;
            depths_all_mm = [depths_old_mm; depths_new_mm];
        else
            depths_all_mm = depths_old_mm;
        end
        
        % Include start depth
        depths_all_mm = [0; depths_all_mm];
        
        % Get position in bundle
        [ap, ml] = find(ismember(D.TT.ttMap{bndl_ind}, tt_fld));
        
        % Determine position relative to bundle center
        bndl_center = ...
            (size(D.TT.ttMap{bndl_ind}) + 1 - 1*mod(size(D.TT.ttMap{bndl_ind}), 2)) / 2;
        center = [ap, ml] - bndl_center;
        ap_center = center(1);
        ml_center = center(2);
        
        % Flip A-P center
        ap_center = ap_center*-1;
        
        % Get x start pos with offset as a function of z pos
        x_offset = ml_center*D.UI.ttPlotLinOffset;
        x = D.TT.ttCoords{bndl_ind}(1) + ap_center*D.PAR.canSp + x_offset;
        
        % Get y pos
        y_offset = ap_center*D.UI.ttPlotLinOffset;
        y = D.TT.ttCoords{bndl_ind}(2) + ml_center*D.PAR.canSp + y_offset;
        
        % Align z to bundle implant pos
        z = D.TT.ttCoords{bndl_ind}(3);
        
        % Get implant pos based on bundle angle
        [xa, za1] = pol2cart(D.UI.bndAng{bndl_ind}(1), depths_all_mm);
        [ya, za2] = pol2cart(D.UI.bndAng{bndl_ind}(2), abs(za1));
        
        % Align x and y to bundle implant pos
        px = 100 * (x + xa);
        py = 100 * (y + ya);
        pz = 100 * (z + abs(za2)) * -1;
        
        % Delete existing plot
        delete(D.UI.ttTrkLin(tt_ind));
        delete(D.UI.ttTrkMrk(tt_ind,:));
        
        % Create cylinder tt for each
        is_updated = D.F.tt_updated(tt_ind);
        [D.UI.ttTrkLin(tt_ind), D.UI.ttTrkMrk(tt_ind,1:length(px))] = ...
            Get_3D_TT([px, py, pz], D.PAR.ttDiam*100, is_updated, D.UI.axe3dH);
        
        % Set rod color
        Safe_Set(D.UI.ttTrkLin(tt_ind), ...
            'FaceColor', D.UI.ttCol(tt_ind,:), ...
            'FaceAlpha', D.UI.ttFaceAlph(2), ...
            'Visible', 'on')
        
        % Set sphere color
        Safe_Set(D.UI.ttTrkMrk(tt_ind,1:length(px)), ...
            'FaceColor', D.UI.ttCol(tt_ind,:), ...
            'FaceAlpha', D.UI.ttFaceAlph(2), ...
            'Visible', 'on')
        
    end

% ---------------------------- CLEAR TT PLOTS -----------------------------
    function Clear_Plot_TT(action_str, tt_ind, clust_ind)
        
        % Clear all active clusters
        if nargin == 1
            
            % Get indeces of active tt/clust
            clear_mat = ...
                logical(reshape(Safe_Get(D.UI.toggSubPlotTT, 'Value'), size(D.UI.toggSubPlotTT)));
            
        end
        
        % Set to clear specific tt and clust
        if nargin == 3
            
            % Initialize clear mat
            clear_mat = false(size(D.UI.toggSubPlotTT));
            
            % Set specific tt and clust to clear
            clear_mat(tt_ind, clust_ind) = true;
            
        end
        
        % Clear spike plot
        if strcmp(action_str, 'Spike')
            
            % Reset spike plot values
            Safe_Set(D.UI.mrkClustH(clear_mat), ...
                'XData', NaN, 'YData', NaN);
            
        end
        
        % Clear rate plot
        if strcmp(action_str, 'Rate')
            
            % Reset 1D rate patches
            if D.PAR.sesTask == 'Track'
                patch_h = findobj(D.UI.ttClustAxRef(clear_mat),'Type','patch');
                Safe_Set(patch_h, 'FaceAlpha', 0);
            end
            
            % Reset 2D rate plot values
            if D.PAR.sesTask == 'Forage'
                Safe_Set(D.UI.imgClustH(clear_mat), ...
                    'CData', zeros(D.PAR.tt2dBins, D.PAR.tt2dBins), ...
                    'AlphaData', zeros(D.PAR.tt2dBins, D.PAR.tt2dBins))
            end
            
        end
        
    end

% ------------------------- CREATE 3D TT GRAPHICS -------------------------
    function [tt_rod_h, tt_sphere_h] = Get_3D_TT(ttXmat, rad, is_updated, ax)
        
        % Pull out start and values
        X1 = ttXmat(1,:);
        X2 = ttXmat(end,:);
        
        % Calculating the length of the cylinder
        length_cyl=norm(X2-X1);
        
        % Creating a circle in the YZ plane
        t=linspace(0,2*pi,20)';
        x2=rad*cos(t);
        x3=rad*sin(t);
        
        % Creating the points in the X-Direction
        x1=[0 length_cyl];
        
        % Creating (Extruding) the cylinder points in the X-Directions
        xx1=repmat(x1,length(x2),1);
        xx2=repmat(x2,1,2);
        xx3=repmat(x3,1,2);
        
        % Plotting the cylinder along the X-Direction with required length starting
        % from Origin
        tt_rod_h=mesh(ax, xx1, xx2, xx3, 'Visible', 'on');
        
        % Defining Unit vector along the X-direction
        unit_Vx=[1 0 0];
        
        % Calulating the angle between the x direction and the required direction
        % of cylinder through dot product
        angle_X1X2=acos( dot( unit_Vx,(X2-X1) )/( norm(unit_Vx)*norm(X2-X1)) )*180/pi;
        
        % Finding the axis of rotation (single rotation) to roate the cylinder in
        % X-direction to the required arbitrary direction through cross product
        axis_rot=cross([1 0 0],(X2-X1) );
        
        % Rotating the plotted cylinder and the end plate circles to the required
        % angles
        if angle_X1X2~=0 % Rotation is not needed if required direction is along X
            rotate(tt_rod_h,axis_rot,angle_X1X2,[0 0 0])
        end
        
        % Till now cylinder has only been aligned with the required direction, but
        % position starts from the origin. so it will now be shifted to the right
        % position
        set(tt_rod_h,'XData',get(tt_rod_h,'XData')+X1(1))
        set(tt_rod_h,'YData',get(tt_rod_h,'YData')+X1(2))
        set(tt_rod_h,'ZData',get(tt_rod_h,'ZData')+X1(3))
        
        % Setup sphere handle array
        tt_sphere_h = gobjects(1,size(ttXmat,1));
        
        % Create spheres
        for z_sph = 1:size(ttXmat,1)
            
            % Create sphere
            [sx,sy,sz] = sphere;
            
            % Get scaling factor
            if is_updated && z_sph == size(ttXmat,1)
                scale = rad*3;
            else
                scale = rad*2;
            end
            
            % Scale and position
            sx = sx*scale + ttXmat(z_sph,1);
            sy = sy*scale + ttXmat(z_sph,2);
            sz = sz*scale + ttXmat(z_sph,3);
            
            % Plot sphere
            tt_sphere_h(z_sph) = mesh(ax, sx, sy, sz, 'Visible', 'on');
        end
        
        % Set lighting stuff
        set([tt_rod_h, tt_sphere_h], ...
            'EdgeAlpha',0, ...
            'FaceLighting', 'gouraud', ...
            'EdgeLighting', 'gouraud', ...
            'DiffuseStrength', 0.9, ...
            'AmbientStrength', 0.5)
    end

% ------------------------- CREATE 3D TT GRAPHICS -------------------------
    function Show_Active_TT(tt_ind)
        
        % Handle input args
        if nargin == 0
            do_set_all = true;
        else
            do_set_all = false;
        end
        
        % Set all back to default
        for z_tt = 1:length(D.TT.ttLab)
            
            % Set rod color
            Safe_Set(D.UI.ttTrkLin(z_tt), ...
                'FaceAlpha', D.UI.ttFaceAlph(do_set_all+1));
            
            % Set sphere color
            Safe_Set(D.UI.ttTrkMrk(z_tt,:), ...
                'FaceAlpha', D.UI.ttFaceAlph(do_set_all+1));
            
            % Set legend markers
            Safe_Set(D.UI.ttBndlLegMrk, 'LineWidth', D.UI.ttBndlLegMrkWdth(1))
            Safe_Set(D.UI.ttDriveLegMrk, 'LineWidth', D.UI.ttBndlLegMrkWdth(1))
            
        end
        
        % Set current tt
        if ~do_set_all
            
            % Set active rod alpha
            Safe_Set(D.UI.ttTrkLin(tt_ind), ...
                'FaceAlpha', D.UI.ttFaceAlph(2));
            
            % Set active sphere alpha
            Safe_Set(D.UI.ttTrkMrk(tt_ind,:), ...
                'FaceAlpha', D.UI.ttFaceAlph(2));
            
            % Set active legend marker
            Safe_Set(D.UI.ttBndlLegMrk(tt_ind), 'LineWidth', D.UI.ttBndlLegMrkWdth(2))
            Safe_Set(D.UI.ttDriveLegMrk(tt_ind), 'LineWidth', D.UI.ttBndlLegMrkWdth(2))
            
        end
        
    end

% ------------------- CENTER PAXINOS IMAGE ON CURRENT TT ------------------
    function Move_Pax_To_TT(tt_ind)
        
        % Get tt data
        tt_fld = D.TT.ttLab{tt_ind};
        
        % Get bundle ind
        bndl_ind = D.I.ttBndl(tt_ind);
        
        % Get position in bundle
        [ap, ml] = find(ismember(D.TT.ttMap{bndl_ind}, tt_fld));
        
        % Determine position relative to bundle center
        bndl_center = ...
            (size(D.TT.ttMap{bndl_ind}) + 1 - 1*mod(size(D.TT.ttMap{bndl_ind}), 2)) / 2;
        center = [ap, ml] - bndl_center;
        ap_center = center(1);
        ml_center = center(2);
        
        % Flip A-P center
        ap_center = ap_center*-1;
        
        % Get x start pos with offset as a function of z pos
        x = D.TT.ttCoords{bndl_ind}(1) + ap_center*D.PAR.canSp;
        
        % Get y pos
        y = D.TT.ttCoords{bndl_ind}(2) + ml_center*D.PAR.canSp;
        
        % Align z to bundle implant pos
        z = D.TT.ttCoords{bndl_ind}(3);
        
        % Get most current depth
        depth_mm = D.TT.ttLogNew.([tt_fld,'_D'])/1000;
        
        % Get implant pos based on bundle angle
        [xa, za1] = pol2cart(D.UI.bndAng{bndl_ind}(1), depth_mm);
        [ya, za2] = pol2cart(D.UI.bndAng{bndl_ind}(2), abs(za1));
        
        % Align x and y to bundle implant pos
        px = (x + xa);
        py = (y + ya);
        pz = (z + abs(za2)) * -1;
        
        % Set paxinos sag
        img_ind = find(D.TT.imgCoor{1}' > py, 1, 'first');
        if isempty(img_ind)
            img_ind = length(D.TT.imgCoor{1});
        end
        Safe_Set(D.UI.sldSwtchImg(1), 'Value', img_ind);
        Sld_SwtchImg(D.UI.sldSwtchImg(1));
        
        % Set paxinos cor
        img_ind = find(D.TT.imgCoor{2}' < px, 1, 'last');
        if isempty(img_ind)
            img_ind = 1;
        end
        Safe_Set(D.UI.sldSwtchImg(2), 'Value', img_ind);
        Sld_SwtchImg(D.UI.sldSwtchImg(2));
        
        % Set hor to deepest tt in first bundle
        img_ind = find(D.TT.imgCoor{3}' < pz, 1, 'last');
        if isempty(img_ind)
            img_ind = 1;
        end
        Safe_Set(D.UI.sldSwtchImg(3), 'Value', img_ind);
        Sld_SwtchImg(D.UI.sldSwtchImg(3));
        
    end

% ---------------------------PRINT TO CONSOLE------------------------------
    function Console_Write(str, t_now, is_err)
        
        % Handle inputs
        if nargin < 3
            is_err = false;
        end
        if nargin < 2
            t_now = now;
            is_err = false;
        end
        
        % Update log
        Update_Log(str, t_now);
        
        % Get time centered to handshake time
        t_s = Sec_DT(t_now);
        t_s = t_s - DTHANDSHAKE;
        
        % Add to string array
        msg = sprintf('\r%0.2f %s', t_s, str);
        
        % Bail if following conditions not met
        if ~exist('D','var'); return; end
        if ~isfield(D, 'DB'); return; end
        if ~isfield(D.DB, 'consoleStr'); return; end
        
        % Itterate log count
        D.DB.consoleCount = D.DB.consoleCount+1;
        
        % Insert new print string into existing string
        if (D.DB.consoleCount>1)
            D.DB.consoleStr(2:D.DB.consoleCount+1,:) = D.DB.consoleStr(1:D.DB.consoleCount,:);
        end
        D.DB.consoleStr(1,1:150) = ' ';
        D.DB.consoleStr(1,1:length(msg)) = msg;
        
        % Bail if following conditions not met
        if ~isfield(D, 'UI'); return; end
        if ~isfield(D.UI, 'listConsole'); return; end
        if ~isvalid(D.UI.listConsole); return; end
        
        % Print normal
        Safe_Set(D.UI.listConsole, ...
            'String', D.DB.consoleStr);
        
        % Set to red bold for error
        if is_err
            Safe_Set(D.UI.listConsole, ...
                'ForegroundColor', D.UI.warningCol, ...
                'FontWeight','Bold');
        end
        
        % Update UI imediately if not running task
        if UPDATENOW
            Update_UI(0, 'force');
        else
            Update_UI(10, 'force');
        end
        
    end

% ---------------------------UPDATE LOG------------------------------
    function Update_Log(str, t_now)
        
        % Handle inputs
        if nargin < 2
            t_now = now;
        end
        
        % Get time centered to handshake time
        t_s = Sec_DT(t_now);
        t_s = t_s - DTHANDSHAKE;
        t_m = round(t_s*1000);
        
        % Write to Matlab window
        fprintf('%0.2f %s\n\r', t_s, str);
        
        % Bail if following conditions not met
        if ~exist('D','var'); return; end
        if ~isfield(D, 'DB'); return; end
        if ~isfield(D.DB, 'logCount'); return; end
        if ~isfield(D.DB, 'logStr'); return; end
        
        % Itterate log count
        D.DB.logCount = D.DB.logCount+1;
        
        % Add to strings
        msg = sprintf('[%d],%d,%s\r', D.DB.logCount, t_m, str);
        
        % Store log str
        D.DB.logStr{D.DB.logCount} = msg;
        
    end

% ---------------------------UPDATE UI-----------------------------
    function Update_UI(dt_min, arg_str)
        
        % Bail if vars not setup
        if ~isstruct(D)
            return
        end
        if ~isfield(D, 'T')
            return
        end
        
        % Convert to seconds
        dt_min = dt_min/1000;
        
        % Get current time
        t_now = Sec_DT(now);
        
        % Check if UI should be updated
        if t_now-D.T.ui_update >= dt_min
            
            % Run drawnnow
            if strcmp(arg_str, 'force')
                drawnow;
            elseif strcmp(arg_str, 'limitrate')
                drawnow limitrate;
            end
            
            % Store time
            D.T.ui_update = t_now;
            
            % Set flag
            D.F.ui_updated = true;
            
            % Track redraw time
            if isfield(D, 'DB')
                D.DB.draw = Update_DB_DT(D.DB.draw, t_now);
            end
            
        end
        
    end






%% =============================== COM FUNCTIONS ==========================

% --------------------------SEND CS COMMAND--------------------------------
    function[] = Send_CS_Com(id, dat1, dat2, dat3, pack, do_print)
        
        % Handle inputs
        if nargin < 6
            do_print = true;
        end
        if nargin < 5
            pack = 0;
        end
        if nargin < 4
            dat3 = -1;
        end
        if nargin < 3
            dat2 = -1;
            dat3 = -1;
        end
        if nargin < 2
            dat1 = -1;
            dat2 = -1;
            dat3 = -1;
        end
        
        % Wait a max of 1 sec for last message to clear
        if ~ISMATSOLO
            
            % Check for 1 sec
            t_str = Sec_DT(now) + 1;
            dt_check = 1; % (sec)
            
            % Check for pack to reset
            while true
                [abort, pass] = ...
                    Check_Flag(~exist('m2c_pack', 'var'), ...
                    m2c_pack(6) == 0 || Sec_DT(now) >= t_str + dt_check);
                if abort || pass; break; end
            end
            
            % Check status
            if abort
                Console_Write('**WARNING** [Setup] ABORTED: Wait For m2c Packet Reset');
                return
            elseif ~pass
                % Force reset packet
                m2c_pack(6) = 0;
                Console_Write(sprintf('**WARNING** [Send_CS_Com] Forced Reset m2c Packet: id=''%s'' dat1=%2.2f dat2=%2.2f dat3=%2.2f dt=%dms', ...
                    id, dat1, dat2, dat3, round((Sec_DT(now) - t_start)*1000)));
            end
            
        end
        
        % Set mesage ID and dat
        m2c_pack(1) = unicode2native(id,'UTF-8');
        m2c_pack(2) = double(dat1);
        m2c_pack(3) = double(dat2);
        m2c_pack(4) = double(dat3);
        
        % Get new packet number
        if pack == 0
            m2c.cnt_pack = m2c.cnt_pack+1;
            m2c.(id).packLast = m2c.(id).pack;
            m2c.(id).pack = m2c.cnt_pack;
            m2c_pack(5) = m2c.cnt_pack;
        else
            % Use packet input
            m2c_pack(5) = pack;
        end
        
        % Save dat to struct
        m2c.(id).dat1 = double(dat1);
        m2c.(id).dat2 = double(dat2);
        m2c.(id).dat3 = double(dat3);
        m2c.(id).t_send = Sec_DT(now);
        
        % Flag new data
        m2c_pack(6) = 1;
        
        % Bail if not printing
        if ~do_print
            return
        end
        
        % Log/print
        Console_Write(sprintf('   [SENT] m2c: id=''%s'' dat1=%2.2f dat2=%2.2f dat3=%2.2f pack=%d', ...
            id, dat1 ,dat2, dat3, m2c_pack(5)));
        
    end

% ------------------------PROCESS CS COMMAND-------------------------------
    function[] = Proc_CS_Com(id)
        
        % Check for handshake flag
        if ...
                strcmp(id, 'h') && ...
                c2m.(id).dat1 ~= 0
            
            % Store handshake time
            DTHANDSHAKE = Sec_DT(now);
            
        end
        
        % Check for exit/error flag
        if strcmp(id, 'E')
            
            % Check if this is exit flag
            if c2m.(id).dat1 == 1
                
                % Set exit flag
                SetExit()
                
                % Log/print exit received
                Console_Write(sprintf('[Proc_CS_Com] RECIEVED CS EXIT COMMAND: id=''%s'' dat1=%d', ...
                    id, c2m.(id).dat1), now);
            end
            
            % Check for save abort
            if c2m.(id).dat1 == 2
                
                % Format err string
                err_str = '!!ERROR: RUNTIME ERROR: CLICK DONE AND ATTEMPT SAVE!!';
                
                % Display message
                if exist('D', 'var')
                    if ~isfield(D, 'UI')
                        dlg_h = dlgAWL(...
                            err_str, ...
                            '!!ERROR!!', ...
                            'OK', [], [], 'OK', ...
                            D.UI.dlgPos{4}, ...
                            'error');
                        Dlg_Wait(dlg_h);
                    end
                end
                
                % Write to console
                Console_Write(err_str, now, true);
                
            end
            
            % Check for forced abort
            if c2m.(id).dat1 == 3
                
                % Format err string
                err_str = '!!ERROR: RUNTIME ERROR: SHUTTING DOWN!!';
                
                % Display message
                if exist('D', 'var')
                    if ~isfield(D, 'UI')
                        dlgAWL(...
                            err_str, ...
                            '!!ERROR!!', ...
                            'OK', [], [], 'OK', ...
                            D.UI.dlgPos{4}, ...
                            'error');
                    end
                end
                
                % Set exit flag
                SetExit()
                
                % Write to console
                Console_Write(err_str, now, true);
                
                % Pause for message to show
                pause(1);
            end
        end
        
        %Print new data
        Console_Write(sprintf('   [RCVD] c2m: id=''%s'' dat1=%0.2f dat2=%0.2f dat3=%0.2f pack=%0.0f', ...
            c2m.(id).id, c2m.(id).dat1, c2m.(id).dat2, c2m.(id).dat3, c2m.(id).pack), now);
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% --------------------------SEND DATA TO AC--------------------------------
    function[] = Send_AC_Com()
        
        % Bail if not connected
        if ~D.F.ac_connected
            return
        end
        
        % Bail if not tcpip object
        if ~exist('TCPIP', 'var')
            return
        elseif ~isvalid(TCPIP)
            return
        end
        
        % Send data
        fwrite(TCPIP,D.AC.data,'int8');
        
        % Log/print
        Console_Write(sprintf('   [SENT] m2ac: dat=|%d|%d|%d|', ...
            D.AC.data(1),D.AC.data(2),D.AC.data(3)));
    end

% ------------------------SEND COMMAND TO NLX------------------------------
    function[pass, arg_out] = Send_NLX_Cmd(msg, do_print)
        
        % Handle input args
        if nargin == 1
            do_print = true;
        end
        
        % Check if Cheetah open
        if ~D.F.cheetah_running
            
            % Log/print skipping
            Console_Write(sprintf('[Send_NLX_Cmd] SKIPPED: m2nlx: msg="%s"', msg));
            
            % Set outputs and bail
            pass = 0;
            arg_out = {[]};
            return
            
        end
        
        % Store start time
        t_start = Sec_DT(now);
        
        % Send command
        [pass, arg_out] = NlxSendCommand(msg);
        
        % Get send dt
        send_dt = round((Sec_DT(now) - t_start)*1000);
        
        % Bail if not printing
        if ~do_print
            return
        end
        
        % Get out argument
        if iscell(arg_out)
            arg_out = arg_out{:};
        else
            arg_out = 'NA';
        end
        
        % Log/print
        if pass == 1
            Console_Write(sprintf('   [SENT] m2nlx: dt=%d arg_out="%s" msg="%s"', send_dt, arg_out, msg));
        else
            Console_Write(sprintf('**WARNING** FAILED: m2nlx: dt=%d arg_out="%s" msg="%s"', send_dt, arg_out, msg));
        end
    end






%% ========================== MINOR SUPPORT FUNCTIONS =====================

% ---------------------------GET TIME NOW-------------------------------
    function [t_sec] = Sec_DT(t_now)
        
        % Convert from days to seconds
        t_sec = (t_now-TIMSTRLOCAL)*24*60*60;
        
    end

% --------------------------UPDATE DB DT INFO------------------------------
    function [t_arr] = Update_DB_DT(t_arr, t1)
        t_arr(1) = min([999, (Sec_DT(now) - t1) * 1000]);
        t_arr(2) = min([999, t_arr(1), t_arr(2)]);
        t_arr(3) = min([999, max([t_arr(1), t_arr(3)])]);
        t_arr(4) = t_arr(1) + t_arr(4);
        t_arr(5) = t_arr(5) + 1;
    end

% --------------------------HOLD FOR CONDITION-----------------------------
    function[abort, pass] = Check_Flag(abort_cond, pass_cond)
        
        % Handle inputs
        if isempty(abort_cond)
            abort_cond = false;
        end
        
        % Initialize flags
        abort = false;
        pass = false;
        
        % Check for abort cond cond
        if abort_cond
            abort = true;
        end
        
        % Check for pass cond
        if ~abort_cond && pass_cond
            pass = true;
        end
        
        % Pause and update
        Update_UI(0, 'limitrate');
        pause(0.001);
    end

% ----------------------HOLD FOR DIALOGUE RESPONSE-------------------------
    function[choice] = Dlg_Wait(dlg_h)
        
        % Keep checking for response
        while true
            
            % Get current choice
            choice = dlg_h.UserData;
            [abort, pass] = Check_Flag(DOEXIT, ~strcmp(choice, ''));
            
            % Bail if done
            if abort || pass
                break
            end
            
            % Make sure window stays on top
            set(dlg_h,'WindowStyle','modal')
        end
        
        % Make sure GUI is back on top
        if isvalid(FIGH)
            set(FIGH,'WindowStyle','modal')
            Update_UI(0, 'force')
            set(FIGH,'WindowStyle','normal')
            Update_UI(0, 'force')
        end
        
    end

% ----------------------CHECK FOR EXE RUN CONDITION------------------------
    function[is_running] = Check_EXE(exe_str)
        
        % Get EXE status
        cmd_str = sprintf('tasklist /FI "imagename eq %s" /fo table /nh', exe_str);
        [~,result] = system(cmd_str);
        
        % Check for any instance
        is_running = any(strfind(result, exe_str));
    end

% ---------------------CONVERT VT POS TO RAD POS---------------------------
    function [rad, roh] = VT_2_Pol(xy_pos)
        
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

% ---------------------CONVERT RAD POS TO VT POS---------------------------
    function [xy_pos] = Pol_2_VT(rad, roh)
        
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

% ------------------------COMPUTE RAD DIFF---------------------------------
    function [rad_diff] = Rad_Diff(rad1, rad2, type)
        
        % Handle inputs
        if isempty(rad2)
            rd = diff(rad1);
        else
            rd = rad2 - rad1;
        end
        
        % Compute min distance
        if strcmp(type, 'min')
            rad_diff = min(2*pi - abs(rd), abs(rd));
        end
        
        % Subtract and keep in range [0, 2*pi]
        if strcmp(type, 'wrap')
            rad_diff = wrapTo2Pi(rd);
        end
    end

% ------------------------COMPUTE RAD SUM----------------------------------
    function [rad_sum] = Rad_Sum(rad1, rad2)
        
        % Handle inputs
        if nargin < 2
            rs = sum(rad1);
        else
            rs = rad2 + rad1;
        end
        
        % Keep in range [0, 2*pi]
        rad_sum = wrapTo2Pi(rs);
        
        
    end

% ----------------------COMPUTE RAD VELOCITY-------------------------------
    function [vel_arr] = Rad_Vel(rad_arr, ts_arr)
        
        % Compute rad diff
        rad_diff = Rad_Diff(rad_arr, [], 'min');
        
        % Convert to cm
        cm_arr = rad_diff * ((140 * pi)/(2 * pi));
        
        % Get dt and convert from us to sec
        dt_arr = double(diff(ts_arr)) / 10^6;
        
        % Compute velocity
        vel_arr = cm_arr./dt_arr;
    end

% ----------------------GET CARTESIAN BOUNDS-------------------------------
    function [xbnd, ybnd] = Get_Cart_Bnds(radbnds, rohbnds)
        
        % Handle inputs
        if nargin < 2
            if D.PAR.sesTask == 'Track'
                rohbnds = D.P.trackRohBnd;
            else
                rohbnds = D.P.frgRohBnd;
            end
        end
        
        % Convert roh to cm
        in_cm_lim = rohbnds(1)*D.UI.arnRad;
        out_cm_lim = rohbnds(2)*D.UI.arnRad;
        
        % Convert to rad array
        if (length(radbnds) == 2)
            if radbnds(1) > radbnds(2)
                radbnds = wrapToPi(radbnds);
            end
            radDist = Rad_Diff(radbnds, [], 'min');
            nPoints =  round(360 * (radDist/(2*pi))); % 360 pnts per 2*pi
            radbnds = linspace(radbnds(1), radbnds(2), nPoints);
        end
        
        % Compute inner bounds
        [x(1,:),y(1,:)] = pol2cart(radbnds, ones(1,length(radbnds)) * in_cm_lim);
        xbnd(1,:) = x(1,:)*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
        ybnd(1,:) = y(1,:)*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
        
        % Compute outer bounds
        [x(2,:),y(2,:)] = pol2cart(radbnds, ones(1,length(radbnds)) * out_cm_lim);
        xbnd(2,:) = x(2,:)*D.UI.cm2pxl + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.cm2pxl;
        ybnd(2,:) = y(2,:)*D.UI.cm2pxl + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.cm2pxl;
        
    end

% ------------------------CHECK POS BOUNDS---------------------------------
    function [bool_arr] = Check_Pol_Bnds(rad_arr, roh_arr, rad_bnds, roh_bnds)
        
        % Handle defaults
        if nargin<4
            if D.PAR.sesTask == 'Track'
                roh_bnds =  D.P.trackRohBnd;
            else
                roh_bnds =  D.P.frgRohBnd;
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

% -------------------------CHECK DEFAULTS----------------------------------
    function Check_Setup_Defaults()
        
        % Local vars
        rew_del = D.UI.popRewDel.String{get(D.UI.popRewDel,'Value'),:};
        cue_cond = D.PAR.listCueCond{logical(cell2mat(get(D.UI.toggCue, 'Value')))};
        task_cond = D.UI.popTask.String{get(D.UI.popTask, 'Value')}; %#ok<NASGU>
        is_del_changed = false;
        is_cue_changed = false;
        is_task_changed = false;
        is_sound_changed = false;
        
        % Bail on recursive call
        stack = dbstack;
        if contains(stack(3).name, 'Check_Setup_Defaults')
            return
        end
        
        % Set empty values
        if  isempty(cue_cond)
            cue_cond = 'Null';
        end
        
        % CHANGE VALUES BASED ON CUE COND
        
        % Reward delay for 'Half' or 'None' Cue if zero delay set
        if (strcmp(cue_cond, 'Half') || strcmp(cue_cond, 'None')) && ...
                D.PAR.sesCond ~= 'Manual_Training' && ...
                strcmp(rew_del, D.UI.popRewDel.String{2})
            
            % Set reward delay popmenu to minimum delay
            Safe_Set(D.UI.popRewDel, 'Value', ...
                find(ismember(D.UI.popRewDel.String, D.UI.popRewDel.String(3))));
            Pop_RewDel();
            
            % Set flag
            is_del_changed = true;
            
        end
        
        % Reward delay for 'All' Cue
        if strcmp(cue_cond, 'All') && ...
                ~strcmp(rew_del, D.UI.popRewDel.String{2})
            
            % Set to no delay
            Safe_Set(D.UI.popRewDel, 'Value', ...
                find(ismember(D.UI.popRewDel.String, D.UI.popRewDel.String(2))));
            Pop_RewDel();
            
            % Set flag
            is_del_changed = true;
        end
        
        % CHANGE VALUES BASED ON SES COND
        rew_del = D.UI.popRewDel.String{get(D.UI.popRewDel,'Value'),:};
        cue_cond = D.PAR.listCueCond{logical(cell2mat(get(D.UI.toggCue, 'Value')))};
        task_cond = D.UI.popTask.String{get(D.UI.popTask, 'Value')};
        
        % Defaults for 'Manual_Training'
        if D.PAR.sesCond == 'Manual_Training'
            
            % Set to no delay
            if ~strcmp(rew_del, D.UI.popRewDel.String{2})
                
                % Change reward delay popmenu
                Safe_Set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.popRewDel.String, D.UI.popRewDel.String(2))));
                Pop_RewDel();
                
                % Set flag
                is_del_changed = true;
            end
            
            % Set cue condition to 'None'
            if ~strcmp(cue_cond, 'None')
                
                % Change cue buttons
                Safe_Set(D.UI.toggCue( ...
                    ismember(D.PAR.listCueCond, 'None')), 'Value', 1);
                Togg_Cue(D.UI.toggCue(ismember(D.PAR.listCueCond, 'None')));
                Togg_Cue();
                
                % Set flag
                is_cue_changed = true;
            end
        end
        
        % Defaults for 'Rotation'
        if D.PAR.sesCond == 'Rotation'
            
            % Set session task to 'Track'
            if ~strcmp(task_cond, 'Track')
                
                % Change task popmenu
                Safe_Set(D.UI.popTask, 'Value', ...
                    find(ismember(D.UI.popTask.String, 'Track')));
                Pop_Task();
                
                % Set flag
                is_task_changed = true;
            end
            
            % Set reward delay to max
            if ~strcmp(rew_del, D.UI.popRewDel.String{end})
                
                % Change reward delay popmenu
                Safe_Set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.popRewDel.String, D.UI.popRewDel.String(end))));
                Pop_RewDel();
                
                % Set flag
                is_del_changed = true;
            end
            
            % Set cue condition to 'None'
            if ~strcmp(cue_cond, 'None')
                
                % Change cue buttons
                Safe_Set(D.UI.toggCue( ...
                    ismember(D.PAR.listCueCond, 'None')), 'Value', 1);
                Togg_Cue(D.UI.toggCue(ismember(D.PAR.listCueCond, 'None')));
                
                % Set flag
                is_cue_changed = true;
            end
            
            % Set sound condition
            if ~all(D.F.sound(1:2))
                
                % Change buttons
                Safe_Set(D.UI.toggSnd(:), 'Value', 1);
                Togg_Sound();
                
                % Set flag
                is_sound_changed = true;
            end
            
        end
        
        % CHANGE VALUES BASED ON TASK
        rew_del = D.UI.popRewDel.String{get(D.UI.popRewDel,'Value'),:};
        cue_cond = D.PAR.listCueCond{logical(cell2mat(get(D.UI.toggCue, 'Value')))};
        task_cond = D.UI.popTask.String{get(D.UI.popTask, 'Value')};
        
        % Settings for 'Forage' Task
        if strcmp(task_cond, 'Forage')
            
            % Set cue condition to 'None'
            if ~strcmp(cue_cond, 'None')
                
                % Change cue buttons
                Safe_Set(D.UI.toggCue( ...
                    ismember(D.PAR.listCueCond, 'None')), 'Value', 1);
                Togg_Cue(D.UI.toggCue(ismember(D.PAR.listCueCond, 'None')));
                
                % Set flag
                is_cue_changed = true;
            end
            
            % Set reward delay to minimum
            if ~strcmp(rew_del, D.UI.popRewDel.String{3})
                
                % Set to minimum delay
                Safe_Set(D.UI.popRewDel, 'Value', ...
                    find(ismember(D.UI.popRewDel.String, D.UI.popRewDel.String(3))));
                Pop_RewDel();
                
                % Set flag
                is_del_changed = true;
            end
            
        end
        
        % Log/print
        if (is_del_changed)
            Update_Log('[Check_Setup_Defaults] Changed Reward Delay');
            Update_UI(0, 'force');
        end
        if (is_cue_changed)
            Update_Log('[Check_Setup_Defaults] Changed Cue Condition');
            Update_UI(0, 'force');
        end
        if (is_task_changed)
            Update_Log('[Check_Setup_Defaults] Changed Task Condition');
            Update_UI(0, 'force');
        end
        if (is_sound_changed)
            Update_Log('[Check_Setup_Defaults] Changed Sound Condition');
            Update_UI(0, 'force');
        end
        
    end

%%=========================================================================






%% ============================ CALLBACK FUNCTIONS ========================

% -----------------------------SWITCH MONITOR------------------------------
    function Togg_Mon(hObject, ~, ~)
        
        % Get user data
        mon_ind = get(hObject, 'UserData');
        
        % Make sure GUI back on top
        set(FIGH,'WindowStyle','modal')
        pause(0.01)
        set(FIGH,'WindowStyle','normal')
        
        % Can only set to select
        if hObject.Value == 0
            hObject.Value = 1;
        end
        
        % Move windows based on session type
        if NlxAreWeConnected() == 1
            
            % Dont move unless using
            if D.F.implant_session
                
                % Move Ephys windows
                if mon_ind == 1 || mon_ind == 2
                    cfg_fi = 'ICR_Set_Window_Position_Ephys_M1.cfg';
                else
                    cfg_fi = 'ICR_Set_Window_Position_Ephys_M3.cfg';
                end
                
            else
                
                % Move Behavior windows
                if mon_ind == 1 || mon_ind == 2
                    cfg_fi = 'ICR_Set_Window_Position_Behavior_M1.cfg';
                else
                    cfg_fi = 'ICR_Set_Window_Position_Behavior_M3.cfg';
                end
                
            end
            
            % Load ephys pos cfg
            Send_NLX_Cmd(['-ProcessConfigurationFile ', cfg_fi]);
            
        end
        
        % Update current pos
        D.UI.figGrpPos{4} = D.UI.figGrpPos{mon_ind};
        D.UI.dlgPos{4} = D.UI.dlgPos{mon_ind};
        
        % Set Figure postion
        FIGH.Position = D.UI.figGrpPos{4};
        
        % Bring UI to top
        uistack(FIGH, 'top')
        
        % Unset other buttons
        Safe_Set(D.UI.toggMon(1:3~=mon_ind), 'Value', 0);
        
        % Update buttons
        Button_State(D.UI.toggMon, 'Update');
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% -----------------------------LOAD SELECTION------------------------------
    function Togg_Type(hObject, ~, ~)
        
        % Bail if nothing selected
        if (strcmp(get(hObject, 'Style'),'popupmenu') && get(D.UI.popType,'Value') == 1) || ...
                (strcmp(get(hObject, 'Style'),'togglebutton') && get(D.UI.toggType,'Value') == 0)
            
            % Unset both
            Safe_Set(D.UI.popType,'Value', 1)
            Safe_Set(D.UI.toggType,'Value', 0)
            
            % Update button
            Button_State(D.UI.toggType, 'Update');
            
            % Update pop
            Safe_Set(D.UI.popType, ...
                'BackgroundColor',D.UI.figBckCol, ...
                'ForegroundColor', D.UI.enabledCol);
            
            return
        end
        
        % Change to active color
        Safe_Set(D.UI.popType, ...
            'BackgroundColor', D.UI.activeCol, ...
            'ForegroundColor', D.UI.enabledBtnFrgCol);
        
        % Bail here if called by pop
        if strcmp(get(hObject, 'Style'),'popupmenu')
            return
        end
        
        % Disable pop and button
        Safe_Set(D.UI.popType, 'Enable', 'off')
        Button_State(D.UI.toggType, 'Disable');
        
        % Store selected session type
        D.PAR.sesType(:) = D.UI.popType.String{get(D.UI.popType, 'Value')};
        
        % Set flag
        D.F.ses_type_confirmed = true;
        
        % Use default rat list
        if D.PAR.sesType ~= 'TT_Turn'
            Safe_Set(D.UI.popRat, 'String', [{''}; D.PAR.listRat])
        end
        
        % Include only implanted rats in popRat list
        if D.PAR.sesType == 'TT_Turn'
            
            % Get ind of rats to include
            inc_ind = ismember(D.SS_IO_1.Properties.RowNames(D.SS_IO_1.Include_Run), ...
                D.TT_IO_1.Properties.RowNames(D.TT_IO_1.Include_Run));
            tt_rat_list = regexprep(D.PAR.listRat(inc_ind), 'r', '');
            
            % Update pop menu
            Safe_Set(D.UI.popRat, 'String', [{''}; tt_rat_list])
            
        end
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% ------------------------------RAT SELECTION------------------------------
    function Pop_Rat(~, ~, ~)
        
        % Bail if nothing selected
        if get(D.UI.popRat,'Value') == 1
            return
        end
        
        % Change to active color
        Safe_Set(D.UI.popRat, ...
            'BackgroundColor', D.UI.activeCol, ...
            'ForegroundColor', D.UI.enabledBtnFrgCol);
        
        % Get Rat label
        D.PAR.ratLab = ... % ('r####')
            ['r',D.UI.popRat.String{get(D.UI.popRat,'Value')}(1:4)];
        
        % Get rat number
        D.PAR.ratNum = ... % (####)
            str2double(D.PAR.ratLab(2:end));
        
        % Get rat index in D.SS_IO_1
        D.PAR.ratIndSS = ...
            find(ismember(D.SS_IO_1.Properties.RowNames, D.PAR.ratLab));
        
        % Get rat index in D.TT_IO_1
        D.PAR.ratIndTT = ...
            find(ismember(D.TT_IO_1.Properties.RowNames, D.PAR.ratLab));
        
        % Get rat implant status
        D.F.rat_implanted = D.SS_IO_1.Implanted(D.PAR.ratIndSS);
        
        % Setup ICR Session stuff
        if D.PAR.sesType == 'ICR_Session'
            
            % Get rat age
            D.PAR.ratAgeGrp = ... % [Young,Old]
                D.SS_IO_1.Age_Group(D.PAR.ratIndSS);
            
            % Get rat dob
            D.PAR.ratDOB = ...
                D.SS_IO_1.DOB(D.PAR.ratIndSS);
            
            % Get feeder condition
            D.PAR.ratFeedCnd = ... % [C1 C2]
                D.SS_IO_1.Feeder_Condition(D.PAR.ratIndSS);
            
            % Get feeder condition number
            D.PAR.ratFeedCnd_Num = ... % [1 2]
                find(D.PAR.listFeedCnd == D.PAR.ratFeedCnd);
            
        end
        
        % SET UI TO LAST SESSION
        
        % Set human
        D.PAR.sesHuman = ...
            D.SS_IO_1.Human(D.PAR.ratIndSS);
        Safe_Set(D.UI.popHuman, 'Value', ...
            find(ismember(D.UI.popHuman.String, D.PAR.sesHuman)));
        Pop_Human();
        
        % Setup ICR Session stuff
        if D.PAR.sesType == 'ICR_Session'
            
            % Set cue condition
            D.PAR.sesCue = ...
                D.SS_IO_1.Cue_Condition(D.PAR.ratIndSS);
            Safe_Set(D.UI.toggCue( ...
                ismember(D.PAR.listCueCond, D.PAR.sesCue)), 'Value', 1);
            % Set all others to off
            Safe_Set(D.UI.toggCue( ...
                ~ismember(D.PAR.listCueCond, D.PAR.sesCue)), 'Value', 0);
            Togg_Cue();
            
            % Set reward delay
            D.PAR.sesRewDel = ...
                D.SS_IO_1.Reward_Delay(D.PAR.ratIndSS);
            Safe_Set(D.UI.popRewDel, 'Value', ...
                find(ismember(D.UI.popRewDel.String, D.PAR.sesRewDel)));
            % run callback
            Pop_RewDel();
            
            % Set sound conditions
            D.F.sound(1:2) = logical(...
                D.SS_IO_1.Sound_Conditions(D.PAR.ratIndSS,1:2));
            Safe_Set(D.UI.toggSnd(D.F.sound(1:2)), 'Value', 1);
            Togg_Sound();
            
            % Set Session Condition
            D.PAR.sesCond = ...
                D.SS_IO_1.Session_Condition(D.PAR.ratIndSS);
            val_ind = find(ismember(D.UI.popCond.String, D.PAR.sesCond));
            if ~isempty(val_ind)
                Safe_Set(D.UI.popCond, 'Value', val_ind);
                % run callback
                Pop_Cond();
            end
            
            % Set Session Task
            D.PAR.sesTask = ...
                D.SS_IO_1.Session_Task(D.PAR.ratIndSS);
            Safe_Set(D.UI.popTask, 'Value', ...
                find(ismember(D.UI.popTask.String, D.PAR.sesTask)));
            % run callback
            Pop_Task();
            
        end
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%s\"', 'Pop_Rat', D.PAR.ratLab));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% ---------------------------SESSION CONDITION-----------------------------
    function Pop_Human(~, ~, ~)
        
        % Bail if nothing selected
        if get(D.UI.popHuman,'Value') == 1
            return
        end
        
        % Change to active color
        Safe_Set(D.UI.popHuman, ...
            'BackgroundColor', D.UI.activeCol, ...
            'ForegroundColor', D.UI.enabledBtnFrgCol);
        
        % Store value
        D.PAR.sesHuman(:) = D.UI.popHuman.String(get(D.UI.popHuman, 'Value'));
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%s\"', 'Pop_Human', char(D.PAR.sesHuman)));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ---------------------------SESSION CONDITION-----------------------------
    function Pop_Cond(~, ~, ~)
        
        % Bail if nothing selected
        if get(D.UI.popCond,'Value') == 1
            return
        end
        
        % Change to active color
        Safe_Set(D.UI.popCond, ...
            'BackgroundColor', D.UI.activeCol, ...
            'ForegroundColor', D.UI.enabledBtnFrgCol);
        
        % Store value
        D.PAR.sesCond(:) = D.UI.popCond.String(get(D.UI.popCond, 'Value'));
        
        % Change other buttons for manual training
        if D.PAR.sesCond == 'Manual_Training'
            
            % Set to minimum reward delay
            Safe_Set(D.UI.popRewDel, 'Value', ...
                find(ismember(D.UI.popRewDel.String, D.UI.popRewDel.String(2))));
            % run callback
            Pop_RewDel();
            
            % Set cue condition
            Safe_Set(D.UI.toggCue( ...
                ismember(D.PAR.listCueCond, 'None')), 'Value', 1);
            % Set all others to off
            Safe_Set(D.UI.toggCue( ...
                ~ismember(D.PAR.listCueCond, 'None')), 'Value', 0);
            Togg_Cue();
            
            % Change other buttons for rot cond
        elseif D.PAR.sesCond == 'Rotation'
            
            % Set Session Task to Track
            Safe_Set(D.UI.popTask, 'Value', ...
                find(ismember(D.UI.popTask.String, 'Track')));
            % run callback
            Pop_Task();
            
            % Set to maximum reward delay
            Safe_Set(D.UI.popRewDel, 'Value', ...
                find(ismember(D.UI.popRewDel.String, D.UI.popRewDel.String(end))));
            % run callback
            Pop_RewDel();
            
            % Set cue condition
            Safe_Set(D.UI.toggCue( ...
                ismember(D.PAR.listCueCond, 'None')), 'Value', 1);
            % Set all others to off
            Safe_Set(D.UI.toggCue( ...
                ~ismember(D.PAR.listCueCond, 'None')), 'Value', 0);
            Togg_Cue();
            
            % Set sound condition
            Safe_Set(D.UI.toggSnd(D.F.sound(1:2)), 'Value', 1);
            Togg_Sound();
            
        end
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%s\"', 'Pop_Cond', char(D.PAR.sesCond)));
        
        % Check settings
        Check_Setup_Defaults();
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% -----------------------------SESSION TASK--------------------------------
    function Pop_Task(~, ~, ~)
        
        % Bail if nothing selected
        if get(D.UI.popTask,'Value') == 1
            return
        end
        
        % Change to active color
        Safe_Set(D.UI.popTask, ...
            'BackgroundColor', D.UI.activeCol, ...
            'ForegroundColor', D.UI.enabledBtnFrgCol);
        
        % Store value
        D.PAR.sesTask(:) = D.UI.popTask.String(get(D.UI.popTask, 'Value'));
        
        % Change settings for 'Forage'
        if D.PAR.sesTask == 'Forage'
            
            % Set to minimum reward delay
            Safe_Set(D.UI.popRewDel, 'Value', ...
                find(ismember(D.UI.popRewDel.String, D.UI.popRewDel.String(2))));
            % run callback
            Pop_RewDel();
            
            % Set cue condition
            Safe_Set(D.UI.toggCue( ...
                ismember(D.PAR.listCueCond, 'None')), 'Value', 1);
            % Set all others to off
            Safe_Set(D.UI.toggCue( ...
                ~ismember(D.PAR.listCueCond, 'None')), 'Value', 0);
            Togg_Cue();
            
        end
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%s\"', 'Pop_Task', char(D.PAR.sesTask)));
        
        % Check settings
        Check_Setup_Defaults();
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -----------------------------REWARD DELAY--------------------------------
    function Pop_RewDel(~, ~, ~)
        
        % Bail if nothing selected
        if get(D.UI.popRewDel,'Value') == 1
            return
        end
        
        % Change to active color
        Safe_Set(D.UI.popRewDel, ...
            'BackgroundColor', D.UI.activeCol, ...
            'ForegroundColor', D.UI.enabledBtnFrgCol);
        
        % Store as cat
        D.PAR.sesRewDel(:) = D.UI.popRewDel.String{get(D.UI.popRewDel,'Value'),:};
        
        % Convert to number
        D.PAR.rewDel = str2double(char(D.PAR.sesRewDel));
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%s\"', 'Pop_RewDel', char(D.PAR.sesRewDel)));
        
        % Check settings
        Check_Setup_Defaults();
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -----------------------------CUE CONDITION-------------------------------
    function Togg_Cue(hObject, ~, ~)
        
        % Get trigger button
        cue_cond = find(cell2mat(get(D.UI.toggCue, 'Value')) == 1);
        
        % Switching buttons
        if length(cue_cond)>1
            cue_cond = get(hObject, 'UserData');
        end
        
        % Change to active
        if get(D.UI.toggCue(cue_cond), 'Value') == 1
            
            % Store string
            cue_str = get(D.UI.toggCue(cue_cond), 'String');
            
            % Activate current button
            Button_State(D.UI.toggCue(cue_cond), 'Enable');
            
            % Inactivate other buttons
            Safe_Set(D.UI.toggCue(([1,2,3] ~= cue_cond)), 'Value',0);
            Button_State(D.UI.toggCue(([1,2,3] ~= cue_cond)), 'Update');
            
            % Save data
            D.PAR.sesCue(:) = cue_str;
            
            % Change to inactive
        else
            Safe_Set(D.UI.toggCue(cue_cond), 'Value',0);
            Button_State(D.UI.toggCue(cue_cond), 'Update');
        end
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%s\"', 'Togg_Cue', char(D.PAR.sesCue)));
        
        % Check settings
        Check_Setup_Defaults();
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ----------------------------SOUND CONDITION------------------------------
    function Togg_Sound(~, ~, ~)
        
        % Note: D.F.sound format
        %         {'White'}
        %         {'Reward'}
        
        for z_sa = 1:2
            
            % Get button ID and state value
            D.F.soundVal = get(D.UI.toggSnd(z_sa), 'Value');
            D.F.soundID = get(D.UI.toggSnd(z_sa), 'UserData');
            
            % Store logical
            if D.F.soundVal == 1
                D.F.sound(z_sa) = true;
            else
                D.F.sound(z_sa) = false;
            end
        end
        
        % Turn off all sound if white noise is off
        if D.F.sound(1) == false
            D.F.sound(2) = false;
            Safe_Set(D.UI.toggSnd(2), 'Value', 0);
        end
        
        % Update buttons
        Button_State(D.UI.toggSnd, 'Update');
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d\" \"%d\"', 'Togg_Sound', ...
            get(D.UI.toggSnd(1), 'Value'), get(D.UI.toggSnd(2), 'Value')));
        
        % Check settings
        Check_Setup_Defaults();
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -------------------------------SETUP DONE--------------------------------
    function Togg_SetupDone(~, ~, ~)
        
        % Confirm UI entries
        if ...
                get(D.UI.popRat,'Value') == 1 || ...
                get(D.UI.popHuman,'Value') == 1 || ...
                (D.PAR.sesType == 'ICR_Session' && ...
                (get(D.UI.popCond,'Value') == 1 || ...
                get(D.UI.popTask,'Value') == 1 || ...
                get(D.UI.popRewDel,'Value') == 1 || ...
                all(cell2mat(get(D.UI.toggCue,'Value')) == 0)))
            
            dlg_h = dlgAWL(...
                '!!WARNING: MISSING ENTRY!!', ...
                'MISSING SETUP ENTRIES', ...
                'OK', [], [], 'OK', ...
                D.UI.dlgPos{4}, ...
                'warning');
            Dlg_Wait(dlg_h);
            
            % Unset button
            Safe_Set(D.UI.toggSetupDone, 'Value', 0);
            
            % Bail
            return
            
        end
        
        % Disable Setup panel objects
        Button_State(D.UI.toggSetupDone, 'Update');
        Object_Group_State('Setup_Objects', 'Disable')
        
        % Flag if implant session
        D.F.implant_session = D.PAR.sesType == 'TT_Turn' || ...
            (D.F.rat_implanted && ...
            (D.PAR.sesCond == 'Rotation' || D.PAR.sesCond == 'Implant_Training'));
        
        % Set flag
        D.F.ses_data_loaded = true;
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d\"', 'Togg_SetupDone', get(D.UI.toggSetupDone,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% ----------------------------UPDATE RAT WEIGHT----------------------------
    function Togg_UpdateWeight(~, ~, ~)
        
        % Get new weight
        D.PAR.ratWeightNew = str2double(D.UI.editNewWeight.String);
        
        % Unset button and bail if value bad
        if isnan(D.PAR.ratWeightNew)
            Safe_Set(D.UI.toggUpdateWeight, 'Value', 0)
            return
        end
        
        % Get cap weight
        if D.F.rat_implanted
            
            % Get weight of cap
            D.PAR.capWeight = D.PAR.capWeightArr(1);
            
            % Add aditional weights
            if D.UI.popCapWeight.Value > 1
                D.PAR.capWeight = D.PAR.capWeight + D.PAR.capWeightArr(2) + ...
                    D.PAR.capWeightArr(3)*(D.UI.popCapWeight.Value-2);
            end
            
        end
        
        % Calculate new corrected weight
        D.PAR.ratWeightCorrected = ...
            D.PAR.ratWeightNew - D.PAR.driveWeight - D.PAR.capWeight;
        
        % Check for missing baseling
        if isnan(D.PAR.ratWeightBaseline)
            D.PAR.ratWeightBaseline =  D.PAR.ratWeightCorrected;
        end
        
        % Calculate weight proportion
        D.PAR.ratWeightProportion = ...
            D.PAR.ratWeightCorrected / D.PAR.ratWeightBaseline;
        weight_prcnt = round(D.PAR.ratWeightProportion*100);
        
        % Print new values
        D.UI.txtBaselineWeight(2).String = sprintf('%0.0f', D.PAR.ratWeightBaseline);
        D.UI.txtCorrectedWeight(2).String = sprintf('%0.0f', D.PAR.ratWeightCorrected);
        D.UI.txtLastWeight(2).String = sprintf('%0.0f', D.PAR.ratWeightLast);
        D.UI.txtPercentageWeight(2).String = sprintf('%0.0f%%', weight_prcnt);
        
        % Update button
        Button_State(D.UI.toggUpdateWeight, 'Update');
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ------------------------------SAVE FOOD INFO-----------------------------
    function Togg_UpdateFed(~, ~, ~)
        
        % Get pellet entry
        D.PAR.feedPellets = str2double(D.UI.editFeedPellets.String);
        
        % Get mash entry
        D.PAR.feedMash = str2double(D.UI.editFeedMash.String);
        
        % Get ensure entry
        D.PAR.feedEnsure = str2double(D.UI.editFeedEnsure.String);
        
        % Get stat entry
        D.PAR.feedSTAT = str2double(D.UI.editFeedSTAT.String);
        
        % Unset and bail if any bad values
        if any(isnan([D.PAR.feedPellets, D.PAR.feedMash, D.PAR.feedEnsure, D.PAR.feedSTAT]))
            Safe_Set(D.UI.toggUpdateFed, 'Value', 0)
            return
        end
        
        % Update button
        Button_State(D.UI.toggUpdateFed, 'Update');
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -------------------------------ACQ BUTTON--------------------------------
    function Togg_Acq(~, ~, ~)
        
        % Bail if recording
        if get(D.UI.toggRec,'Value') == 1
            
            % Bail if aqc active
            if get(D.UI.toggAcq,'Value') == 1
                return
            end
            
            % Enable acq
            Safe_Set(D.UI.toggAcq,'Value',1)
        end
        
        % Start aquisition
        if get(D.UI.toggAcq,'Value') == 1
            Send_NLX_Cmd('-StartAcquisition');
        else
            Send_NLX_Cmd('-StopAcquisition');
        end
        
        % Update button
        Button_State(D.UI.toggAcq, 'Update');
        
        % Set time tracking variables
        if D.F.acq
            % save out time before stopping
            D.T.acq_tot_tim = (Sec_DT(now) - D.T.acq_tim) + D.T.acq_tot_tim;
        end
        
        % Change aquiring status
        D.F.acq = ~D.F.acq;
        
        % Reset temp now
        D.T.acq_tim = Sec_DT(now);
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d\"', 'Togg_Acq', get(D.UI.toggAcq,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -------------------------------REC BUTTON--------------------------------
    function Togg_Rec(~, ~, ~)
        
        % Start acq if not running
        if get(D.UI.toggRec,'Value') == 1 && ...
                get(D.UI.toggAcq,'Value') == 0
            
            % run ToggAcq
            Togg_Acq(D.UI.toggAcq);
            
        end
        
        % Start recording
        if get(D.UI.toggRec,'Value') == 1
            Send_NLX_Cmd('-StartRecording');
        else
            Send_NLX_Cmd('-StopRecording');
        end
        
        % Update button
        Button_State(D.UI.toggRec, 'Update');
        
        % Set time tracking variables
        if  D.F.rec
            % save out time before stopping
            D.T.rec_tot_tim = (Sec_DT(now) - D.T.rec_tim) + D.T.rec_tot_tim;
        end
        D.F.rec = ~ D.F.rec;
        D.T.rec_tim = Sec_DT(now);
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d\"', 'Togg_Rec', get(D.UI.toggRec,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ----------------------------CELLS CUT BUTTON-----------------------------
    function Togg_LoadClust(hObject, ~, ~)
        
        % Get handle values
        val = get(hObject, 'Value');
        %is_ran = get(hObject, 'UserData') == 1;
        
        % Upate button
        Button_State(D.UI.toggLoadClust, 'Update');
        
        % Get a list of ncf files
        clust_fi_dir =  fullfile(D.DIR.nlxTempTop,'0000-00-00_00-00-00');
        D.TT.clust_files = dir(clust_fi_dir);
        D.TT.clust_files = {D.TT.clust_files.name};
        D.TT.clust_files = regexp(D.TT.clust_files,'TT\d\d.NCF','match');
        D.TT.clust_files = [D.TT.clust_files{:}];
        
        % Open stream
        for z_tt = 1:length(D.TT.ttLab)
            
            % Get tt label
            tt_lab = D.TT.ttLab{z_tt};
            
            % Parse clust file
            if val == 1
                
                % Check if file exists
                fi_ind = find(ismember(D.TT.clust_files, [tt_lab, '.NCF']));
                if ~isempty(fi_ind)
                    
                    % Read in file text
                    file_id = fopen(fullfile(clust_fi_dir,D.TT.clust_files{fi_ind}));
                    file_str = fread(file_id,'*char')';
                    fclose(file_id);
                    
                    % Parse file for number of clusters
                    clusts = regexp(file_str, sprintf('"%s" (\\d)', tt_lab), 'tokens');
                    
                    % Store number of clusters
                    D.TT.nClust(z_tt) = max(str2double([clusts{:}]));
                    
                end
                
                % Include cluster 0 in count
                D.TT.nClust(z_tt) = D.TT.nClust(z_tt)+1;
                
                % Include cluster 0 in all streaming tts
                D.F.clust_loaded(z_tt, 1) = val == 1;
                
                % Enable clust buttons
                Safe_Set(D.UI.toggSubPlotTT(z_tt,1:D.TT.nClust(z_tt)), 'Enable', 'on')
                for z_c = 1:D.TT.nClust(z_tt)
                    Safe_Set(D.UI.toggSubPlotTT(z_tt,z_c), ...
                        'ForegroundColor', D.UI.clustCol(z_tt, z_c,:));
                end
                
            end
            
            % Initialize flag
            succeeded = false;
            
            % Open tt stream
            if val == 1
                msg = sprintf('Open \"%s\" Stream', tt_lab);
                if any(contains(D.NLX.das_objects, tt_lab))
                    succeeded = NlxOpenStream(tt_lab) == 1;
                end
            end
            
            % Close tt stream
            if val == 0
                msg = sprintf('Close \"%s\" Stream', tt_lab);
                if any(contains(D.NLX.das_objects, tt_lab))
                    succeeded = NlxCloseStream(tt_lab) == 1;
                end
            end
            
            % Check status
            if succeeded
                
                % Log/print success
                Console_Write(sprintf('[Togg_LoadClust] FINISHED: %s', msg));
                
                % Set flag
                D.F.clust_loaded(z_tt, 1:D.TT.nClust(z_tt)) =  val == 1;
                
            else
                
                % Check for missing das
                if ~any(contains(D.NLX.das_objects, tt_lab))
                    Console_Write(sprintf('**WARNING** [Togg_LoadClust] ABORTED: %s: Missing DAS Object', msg));
                else
                    Console_Write(sprintf('**WARNING** [Togg_LoadClust] FAILED: %s', msg));
                end
                
                % Bail
                continue
            end
            
        end
        
        % Enable/setup plotting
        if val == 1
            
            % Set callback
            Safe_Set(D.UI.toggSubPlotTT, ...
                'Callback', {@Togg_SubPlotTT})
            
            % Initialize clust pos vals
            [D.TT.posClust{D.F.clust_loaded}] =  deal(cell2struct( ...
                {[1,0], [1,0], single(NaN(120*60*33,2)), single(NaN(120*60*33,2)), NaN(120*60*33,1)}, ...
                {'indLap', 'indAll', 'Cart', 'Pol', 'TS'}, 2));
            
            % Enable ephys plottng objects
            Object_Group_State('TT_Plot_Objects', 'Enable')
            
            % Set ran flag
            %D.UI.toggLoadClust.UserData = 1;
            
        end
        
        % Disable plotting
        if val == 0
            
            % Disable all flags
            D.F.clust_loaded(:) = false;
            
            % Reset count
            D.TT.nClust(:) = 0;
            
            % Disable ephys plottng objects
            Object_Group_State('TT_Plot_Objects', 'Disable')
            
        end
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ---------------------ENABLE CUBE TRACKER LED BUTTON----------------------
    function Togg_CubeLED(~, ~, ~)
        
        % Upate button
        Button_State(D.UI.toggCubeLED, 'Update');
        
        % Turn on Cube LEDs
        if get(D.UI.toggCubeLED, 'Value') == 1
            Send_NLX_Cmd('-SendLynxSXCommand AcqSystem1 -WHSSetTrackingLED 1 1');
        end
        
        % Turn off Cube LEDs
        if get(D.UI.toggCubeLED, 'Value') == 0
            Send_NLX_Cmd('-SendLynxSXCommand AcqSystem1 -WHSSetTrackingLED 1 0');
        end
        
    end

% -----------------------------SLEEP BUTTON--------------------------------
    function Togg_Sleep(hObject, ~, ~)
        
        % Get user data and value
        user_data = get(hObject, 'UserData');
        sleep_phase = user_data(1);
        sleep_done = user_data(2);
        val = get(hObject, 'Value');
        
        % Bail already run
        if sleep_done == 1
            return
        end
        
        % Update button
        Button_State(D.UI.toggSleep(sleep_phase), 'Update');
        
        % Start of sleep
        if val == 1
            
            % Store sleep start time
            D.T.sleep_str(sleep_phase) = Sec_DT(now);
            
            % Send NLX sleep start event
            Send_NLX_Cmd(D.NLX.sleep_start_evt{sleep_phase});
            
            % Enable load clust and show attention color
            if sleep_phase == 1
                Button_State(D.UI.toggLoadClust, 'Enable', D.UI.attentionCol);
            end
            
            % Switch to sleep tracking for second sleep
            if sleep_phase == 2
                
                % Log/print
                Console_Write('[Togg_Sleep] RUNNING: Load "ICR_Set_VT_Entities_Ephys_Sleep.cfg"');
                
                % Load sleep tracking config
                Send_NLX_Cmd('-ProcessConfigurationFile ICR_Set_VT_Entities_Ephys_Sleep.cfg');
                Console_Write('[NLX_Setup] FINISHED: Load "ICR_Set_VT_Entities_Ephys_Sleep.cfg"');
                
                % Log/print
                Console_Write('[Togg_Sleep] FINISHED: Load "ICR_Set_VT_Entities_Ephys_Sleep.cfg"');
                
            end
            
        end
        
        % End of sleep
        if val == 0
            
            % Disable Sleep button
            if sleep_phase == 1
                Object_Group_State('Sleep1_Objects', 'Disable');
            else
                Object_Group_State('Sleep2_Objects', 'Disable');
            end
            
            % Store end time
            D.T.sleep_end(sleep_phase) = Sec_DT(now);
            
            % Send NLX sleep end event
            Send_NLX_Cmd(D.NLX.sleep_end_evt{sleep_phase});
            
            % Switch back to task tracking after first sleep
            if sleep_phase == 1
                
                % Log/print
                Console_Write('[Togg_Sleep] RUNNING: Load "ICR_Set_VT_Entities_Ephys_Task.cfg"');
                
                % Load sleep tracking config
                pause(0.5);
                Send_NLX_Cmd('-ProcessConfigurationFile ICR_Set_VT_Entities_Ephys_Task.cfg');
                
                % Log/print
                Console_Write('[Togg_Sleep] FINISHED: Load "ICR_Set_VT_Entities_Ephys_Task.cfg"');
                
            end
            
            % Flag sleep done
            D.UI.toggSleep(sleep_phase).UserData(2) = 1;
            
        end
        
        % Log/print
        Console_Write(sprintf('[%s(%d)] Set to \"%d"', 'Togg_Sleep', sleep_phase, val));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ---------------------------ROTATION BUTTON-------------------------------
    function Togg_ICR(hObject, ~, ~)
        
        % Get value
        val = get(hObject, 'Value');
        
        % Get next rot ind
        rotNext = [1, 2] ~=  D.I.rot;
        
        % Enable rot pos select
        if (val==1)
            
            % Show rot pos options
            Patch_State(D.UI.ptchRtBnds(rotNext,:), ...
                'ShowAll', D.UI.rotCol(rotNext,:));
            Safe_Set(D.UI.ptchRtBnds(rotNext,:), ...
                'HitTest', 'on');
            Safe_Set(D.UI.txtRtBnds(rotNext, :), ...
                'Visible', 'on', ...
                'HitTest', 'on');
            
            % Make line width of preffered pos thicker
            pref_ind = D.UI.rotCatInd == str2double(char(D.PAR.rotPosList(D.C.rot_cnt+1)));
            Safe_Set(D.UI.ptchRtBnds(rotNext, pref_ind), ...
                'LineWidth', 3);
            
            % Update button to active color
            Button_State(D.UI.toggICR(rotNext), 'Update');
            
        else
            
            % Hide rot pos options
            Patch_State(D.UI.ptchRtBnds(rotNext,:), ...
                'Hide', D.UI.rotCol(rotNext,:));
            Safe_Set(D.UI.ptchRtBnds(rotNext,:), ...
                'HitTest', 'off');
            Safe_Set(D.UI.txtRtBnds(rotNext, :), ...
                'Visible', 'off', ...
                'HitTest', 'off');
            
            % Update button color back to default
            Button_State(D.UI.toggICR(rotNext), 'Update', D.UI.rotCol(rotNext,:));
            
        end
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d"', 'Togg_ICR', val));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ---------------------------UPDATE UI POS---------------------------------
    function SizeChanged_GetPosUI(~, ~, ~)
        
        % Initialzie
        mon_new_ind = 0;
        
        % Find moninotor window in
        for z_m = 1:3
            
            % Bail if monitor not present
            if z_m > size(D.UI.monPos,1)
                continue
            end
            
            % Move left
            if FIGH.Position(1) == D.UI.monPos(z_m, 1) + 1
                mon_new_ind = z_m - 1;
            end
            
            % Move right
            if sum(FIGH.Position([1,3])) == sum(D.UI.monPos(z_m, [1,3])) - 1
                mon_new_ind = z_m + 1;
            end
            
            % Check for change
            if mon_new_ind > 0 && mon_new_ind <= size(D.UI.monPos,1)
                
                % Update pos
                D.UI.toggMon(mon_new_ind).Value = 1;
                Togg_Mon(D.UI.toggMon(mon_new_ind));
                
                % Bail
                return
                
            end
            
        end
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -------------------------GET MOUSE POSITION------------------------------
    function Mouse_Track(~, ~, ~)
        
        % Block callback re-entry
        s = dbstack();
        if sum(cell2mat(cellfun(@(x) any(strfind(x, 'MouseTrack')), {s.name}, 'uni', false))) > 1
            return
        end
        
        % Bail if no buttons active
        if ...
                (get(D.UI.toggPickRewPos, 'Value') == 0 && ...
                all([D.UI.toggICR.Value] == 0)) || ...
                (strcmp(get(D.UI.toggPickRewPos, 'Enable'), 'off') && ...
                all(ismember({D.UI.toggICR.Enable}, 'off')))
            return
        end
        
        % Get new mouse data
        C = get (D.UI.axH(3), 'CurrentPoint');
        
        % Get cart coord
        x = C(1,1);
        y = C(1,2);
        
        % Get normalized pos data
        x_norm = (x-D.PAR.XC)./D.PAR.R;
        y_norm = (y-D.PAR.YC)./D.PAR.R;
        
        % Get position in radians
        [mouse_rad,mouse_roh] = cart2pol(x_norm, y_norm);
        
        % Convert radians between [0, 2*pi]
        mouse_rad = wrapTo2Pi(mouse_rad);
        
        % Track reward zone select
        if D.PAR.sesTask == 'Track' && ...
                get(D.UI.toggPickRewPos, 'Value') == 1
            
            % Run sub function
            doZoneSelect(mouse_rad, mouse_roh);
        end
        
        % Forage reward target select
        if D.PAR.sesTask == 'Forage' && ...
                get(D.UI.toggPickRewPos, 'Value') == 1
            
            % Run sub function
            doTargSelect(mouse_rad, mouse_roh);
        end
        
        % Rotation select
        if D.PAR.sesCond == 'Rotation' && ...
                any([D.UI.toggICR.Value] == 1)
            
            % Run sub function
            doRotSelect(mouse_rad, mouse_roh);
        end
        
        % Select track reward zone
        function doZoneSelect(rad, roh)
            
            % Check if mouse over any targ
            if ~(roh > D.P.trackRohBnd(1) && roh < D.P.trackRohBnd(2))
                Safe_Set(D.UI.txtFdDurH(D.I.rot, ~D.I.zone_active), 'Visible', 'off');
                Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, ~D.I.zone_active), ...
                    'ShowAll', D.UI.rotCol(D.I.rot,:));
                return
            end
            
            % Check each zone bounds
            for z_ptch = 1:size(D.PAR.rewZoneBnds,1)
                if any(Check_Pol_Bnds(rad, roh, D.PAR.rewZoneBnds(z_ptch,:,D.I.rot)))
                    
                    % Darken patch
                    Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, ~D.I.zone_active), ...
                        'ShowAll', D.UI.rotCol(D.I.rot,:));
                    Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, z_ptch), ...
                        'Select', D.UI.rotCol(D.I.rot,:));
                    
                    % Show text
                    Safe_Set(D.UI.txtFdDurH(D.I.rot, ~D.I.zone_active), 'Visible', 'off');
                    Safe_Set(D.UI.txtFdDurH(D.I.rot, z_ptch), 'Visible', 'on');
                    
                    % Bail
                    break
                    
                else
                    Safe_Set(D.UI.txtFdDurH(D.I.rot, ~D.I.zone_active), 'Visible', 'off');
                    Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, ~D.I.zone_active), ...
                        'ShowAll', D.UI.rotCol(D.I.rot,:));
                end
            end
            
        end
        
        % Select forrage reward target
        function doTargSelect(rad, roh)
            
            % Check if mouse over any targ
            if ~(roh > D.P.frgRohBnd(1) && roh < D.P.frgRohBnd(2))
                Patch_State(D.UI.ptchRewTargBnds(D.PAR.frgTargSelectInd), ...
                    'ShowAll');
                return
            end
            
            % Check each bound
            for z_ptch = 1:length(D.PAR.frgTargSelectInd)
                
                % Get current targ and path ind
                path_ind = D.PAR.frgPathSelectInd(z_ptch);
                targ_ind = D.PAR.frgTargSelectInd(z_ptch);
                
                if any(Check_Pol_Bnds(rad, roh, D.PAR.rewTargBnds(targ_ind,:)))
                    
                    % Darken patch
                    Patch_State(D.UI.ptchRewTargBnds(D.PAR.frgTargSelectInd), ...
                        'ShowAll');
                    Patch_State(D.UI.ptchRewTargBnds(targ_ind), ...
                        'Select');
                    
                    % Get path
                    D.P.pathNowMat = D.P.pathMat(:,:,path_ind,D.I.targ_now);
                    D.P.pathNowMat(D.P.pathNowMat>0) = 0.05;
                    
                    % Plot path
                    cdat = D.P.frgOccMatScale+D.P.pathNowMat;
                    if ~isgraphics(D.UI.imgFrgOcc)
                        D.UI.imgFrgOcc = imagesc(cdat, ...
                            'Parent', D.UI.axH(5));
                    else
                        Safe_Set(D.UI.imgFrgOcc, 'CData', cdat);
                    end
                    
                    % Bail
                    break
                    
                else
                    Patch_State(D.UI.ptchRewTargBnds(D.PAR.frgTargSelectInd), ...
                        'ShowAll');
                end
            end
            
            
        end
        
        % Select rot pos
        function doRotSelect(rad, roh)
            
            % Get next rot ind
            rotNext = [1, 2] ~=  D.I.rot;
            
            % Check if mouse over track
            if ~(roh > D.P.trackRohBnd(1) && roh < D.P.trackRohBnd(2))
                Patch_State(D.UI.ptchRtBnds(rotNext, :), ...
                    'ShowAll', D.UI.rotCol(rotNext,:));
                return
            end
            
            % Check each rot bounds
            for z_ptch = 1:size(D.UI.rotBnds,1)
                if any(Check_Pol_Bnds(rad, roh, D.UI.rotBnds(z_ptch,:,rotNext)))
                    
                    % Darken patch
                    Patch_State(D.UI.ptchRtBnds(rotNext, :), ...
                        'ShowAll', D.UI.rotCol(rotNext,:));
                    Patch_State(D.UI.ptchRtBnds(rotNext, z_ptch), ...
                        'Select', D.UI.rotCol(rotNext,:));
                    
                    % Bail
                    break
                else
                    Patch_State(D.UI.ptchRtBnds(rotNext, :), ...
                        'ShowAll', D.UI.rotCol(rotNext,:));
                end
            end
            
        end
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ------------------MOUSE CLICK TRACK REW ZONE SELECT----------------------
    function Mouse_SelectRewZone(hObject, ~, ~)
        
        % Bail if not checking for zone selection
        if D.PAR.sesTask ~= 'Track' || ...
                get(D.UI.toggPickRewPos, 'Value') == 0
            return
        end
        
        % Get patch user data
        user_data = get(hObject, 'UserData');
        D.I.zone_select = user_data(2);
        
        % Update active zone
        D.I.zone_active(D.I.zone_select) = true;
        
        % Update graphics
        Safe_Set(D.UI.txtFdDurH(D.I.rot, ~D.I.zone_active), 'Visible', 'off');
        Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, ~D.I.zone_active), ...
            'ShowAll', D.UI.rotCol(D.I.rot,:));
        Safe_Set(D.UI.txtFdDurH(D.I.rot, D.I.zone_select), 'Visible', 'on');
        Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, D.I.zone_select), ...
            'Active', D.UI.rotCol(D.I.rot,:));
        
        % Reset button
        Safe_Set(D.UI.toggPickRewPos, 'Value', 0)
        Button_State(D.UI.toggPickRewPos, 'Disable');
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ------------------MOUSE CLICK FORAGE REW ZONE SELECT---------------------
    function Mouse_SelectRewTarg(hObject, ~, ~)
        
        % Bail if not checking for target selection
        if D.PAR.sesTask ~= 'Forage' || ...
                get(D.UI.toggPickRewPos, 'Value') == 0
            return
        end
        
        % Get patch user data
        targ_ind_new = get(hObject, 'UserData');
        
        % Get current targ and path ind
        path_ind = D.PAR.frgPathSelectInd(D.PAR.frgTargSelectInd == targ_ind_new);
        
        % Store path mat for plotting
        D.P.pathNowMat = D.P.pathMat(:,:,path_ind,D.I.targ_now);
        D.P.pathNowMat(D.P.pathNowMat>0) = 0.05;
        
        % Update patches
        Patch_State(D.UI.ptchRewTargBnds, ...
            'Hide');
        Patch_State(D.UI.ptchRewTargBnds(targ_ind_new), ...
            'Active', D.UI.activeCol);
        
        % Reset button
        Safe_Set(D.UI.toggPickRewPos, 'Value', 0)
        Button_State(D.UI.toggPickRewPos, 'Disable');
        
        % Store new targ
        D.I.targ_now = targ_ind_new;
        
        % Log/print
        Console_Write(sprintf('[Mouse_SelectRewTarg] Set New Forage Target: targ_last=%ddeg targ_new=%ddeg', ...
            D.PAR.frgTargDegArr(D.I.targ_last), D.PAR.frgTargDegArr(D.I.targ_now)));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ----------------------MOUSE CLICK ROT POS SELECT-------------------------
    function Mouse_SelectRot(hObject, ~, ~)
        
        % Bail if not checking for rotaion
        if D.PAR.sesCond ~= 'Rotation' || ...
                all([D.UI.toggICR.Value] == 0)
            return
        end
        
        % Get patch user data
        user_data = get(hObject, 'UserData');
        rot_ind = user_data(1);
        pos_ind = user_data(2);
        
        % Store next rotaion bounds
        D.UI.rotBndNext = D.PAR.rotBnds(pos_ind, :, rot_ind);
        
        % Store rot pos
        D.PAR.rotPos = [D.PAR.rotPos, D.PAR.rotDistDeg(pos_ind)];
        
        % Set flag
        D.F.rotate = true;
        
        % Hide all patches and text
        Safe_Set(D.UI.txtRtBnds(:,:), ...
            'Visible', 'off', ...
            'HitTest', 'off');
        Patch_State(D.UI.ptchRtBnds(1, :), ...
            'Hide', D.UI.rotCol(1,:));
        Patch_State(D.UI.ptchRtBnds(2, :), ...
            'Hide', D.UI.rotCol(2,:));
        Safe_Set(D.UI.ptchRtBnds, 'HitTest', 'off')
        
        % Show active rote bounds patch and text
        Safe_Set(D.UI.txtRtBnds(rot_ind, pos_ind), ...
            'Visible', 'on');
        Patch_State(D.UI.ptchRtBnds(rot_ind, pos_ind), ...
            'Active', D.UI.rotCol(rot_ind,:));
        
        % Reset button
        Safe_Set(D.UI.toggICR(rot_ind), 'Value', 0)
        Button_State(D.UI.toggICR(rot_ind), 'Disable', D.UI.rotCol(rot_ind,:));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ---------------------------TASK DONE--------------------------------
    function Togg_TaskDone(~, ~, ~)
        
        % Check if session is done
        dlg_h = dlgAWL(...
            'End Task Run?', ...
            'END TASK RUN', ...
            'Yes', 'No', [], 'No', ...
            D.UI.dlgPos{4}, ...
            'question');
        choice = Dlg_Wait(dlg_h);
        
        % Handle response
        switch choice
            case 'Yes'
                % Post NLX event: session end
                Send_NLX_Cmd(D.NLX.ses_end_evt);
            case 'No'
                return
        end
        
        % Set flag
        D.F.task_done = true;
        
        % Print end time
        infstr = datestr(now, 'HH:MM:SS');
        Safe_Set(D.UI.editTimeInf(2), 'String', infstr)
        
        % Hanle Track task
        if D.PAR.sesTask == 'Track'
            
            % Halt robot if not already halted
            if strcmp(get(D.UI.toggHaltRob, 'Enable'), 'on')
                if (~D.F.halted)
                    Safe_Set(D.UI.toggHaltRob, 'Value', 1);
                    Togg_HaltRob();
                end
            end
            
            % Check if rat is out of room for track run
            dlg_h = dlgAWL(...
                'Take out rat', ...
                'RAT OUT', ...
                'OK', [], [], 'OK', ...
                D.UI.dlgPos{4}, ...
                'default');
            Dlg_Wait(dlg_h);
            
            % Stop halt if still active
            if strcmp(get(D.UI.toggHaltRob, 'Enable'), 'on')
                if (D.F.halted)
                    Safe_Set(D.UI.toggHaltRob, 'Value', 0);
                    Togg_HaltRob();
                end
            end
            
            % Stop bulldoze if still active
            if (get(D.UI.toggBulldoze, 'Value') == 1)
                Safe_Set(D.UI.toggBulldoze, 'Value', 0);
                Pop_Bulldoze();
            end
            
        end
        
        % Post NLX event: rat out
        Send_NLX_Cmd(D.NLX.rat_out_evt);
        
        % Save end time
        D.T.ses_end = Sec_DT(now);
        
        % Disable Run objects
        Object_Group_State('Task_Objects', 'Disable')
        
        % Disable recording objects
        if ~D.F.implant_session
            
            % Stop recording
            if D.F.rec
                Safe_Set(D.UI.toggRec,'Value', 0)
                Togg_Rec(D.UI.toggRec);
            end
            
        end
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d\"', 'Togg_TaskDone', get(D.UI.toggTaskDone,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -------------------------SAVE SESSION DATA-------------------------------
    function Togg_Save(~, ~, ~)
        
        % Only run once
        if get(D.UI.toggSave, 'UserData') > 0
            return
        end
        
        % "Disable" save button so only pressed once
        Safe_Set(D.UI.toggSave, 'UserData', 1);
        
        % Update button
        Button_State(D.UI.toggSave, 'Update');
        
        % Start save status timer
        start(D.timer_save);
        
        % Set flag to save at end of main loop
        D.F.do_save = true;
        
        % Check if cheetah data should be saved
        if D.F.cheetah_running
            
            % Show dialogue
            dlg_h = dlgAWL( ...
                'Save Cheetah Data?', ...
                'SAVE CHEETAH', ...
                'Yes', 'No', [], 'No', ...
                D.UI.dlgPos{4}, ...
                'question');
            choice = Dlg_Wait(dlg_h);
            
            % Handle response
            if strcmp(choice, 'Yes')
                % Set flag
                D.F.do_nlx_save = true;
            end
            
        end
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d\"', 'Togg_Save', get(D.UI.toggSave,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ------------------------------QUIT ALL-----------------------------------
    function Togg_Quit(~, ~, ~)
        
        % Only run once
        if get(D.UI.toggQuit, 'UserData') > 0
            return
        end
        
        % Check for save unless quit before setup done
        if (D.F.ses_data_loaded && ~D.F.ses_save_done)
            
            % Construct a questdlg with two options
            dlg_h = dlgAWL( ...
                '!!WARNING: QUIT WITHOUT SAVING?!!', ...
                'ABBORT RUN', ...
                'Yes', 'No', [], 'No', ...
                D.UI.dlgPos{4}, ...
                'warning');
            choice = Dlg_Wait(dlg_h);
            
            % Handle response
            if strcmp(choice, 'No')
                % Bail
                return
            end
            
            % Make sure rat is out
            if ~D.F.rat_out && D.PAR.sesTask ~= 'Forage'
                
                dlg_h = dlgAWL(...
                    '!!WARNING: TAKE OUT RAT BEFORE PRECEDING!!', ...
                    'RAT OUT', ...
                    'OK', [], [], 'OK', ...
                    D.UI.dlgPos{4}, ...
                    'warning');
                Dlg_Wait(dlg_h);
                
            end
            
        end
        
        % "Disable" save button so only pressed once
        Safe_Set(D.UI.toggQuit, 'UserData', 1);
        
        % Update button
        Button_State(D.UI.toggQuit, 'Update');
        
        % Start quit status timer
        start(D.timer_quit);
        
        % Print session aborting
        if ~D.F.ses_save_done
            Console_Write('**WARNING** [Togg_Quit] ABORTING SESSION...');
        end
        
        % Stop halt if still active
        if (D.F.halted)
            Safe_Set(D.UI.toggHaltRob, 'Value', 0);
            Togg_HaltRob();
        end
        
        % Stop bulldoze if still active
        if (get(D.UI.toggBulldoze, 'Value') == 1)
            Safe_Set(D.UI.toggBulldoze, 'Value', 0);
            Pop_Bulldoze();
        end
        
        % Stop recording
        if D.F.rec
            Safe_Set(D.UI.toggRec,'Value', 0)
            Togg_Rec(D.UI.toggRec);
        end
        
        % Check if battery should be replaced
        if ...
                c2m.('J').dat1 > 0 && ...
                c2m.('J').dat1 <= D.PAR.robVccReplace
            
            % Confirm that Cheetah is closed
            warn_str = ...
                sprintf('**WARNING** BATTERY IS AT %0.2fV AND NEEDS TO BE REPLACED', c2m.('J').dat1);
            
            dlg_h = dlgAWL(...
                warn_str, ...
                'BATTERY LOW', ...
                'OK', [], [], 'OK', ...
                D.UI.dlgPos{4}, ...
                'warning');
            Dlg_Wait(dlg_h);
            
        end
        
        % Tell C# to begin quit
        Send_CS_Com('X', 1);
        
        % Disconnect from AC computer
        Disconnect_AC();
        
        % Set flag
        D.F.do_quit = true;
        
        % Shut down if matlab run alone
        if ISMATSOLO
            SetExit()
        end
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d\"', 'Togg_Quit', get(D.UI.toggQuit,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -----------------------------HALT ROBOT----------------------------------
    function Togg_HaltRob(~, ~, ~)
        
        if (get(D.UI.toggHaltRob, 'Value') == 1)
            
            % Change backround color and text
            Safe_Set(D.UI.toggHaltRob, 'String', 'Halted...');
            
            % Tell CS to Halt Robot
            Send_CS_Com('H', 1);
            
            % Set flag
            D.F.halted = true;
            
        else
            
            % Change backround color and text
            Safe_Set(D.UI.toggHaltRob, 'String', 'Halt Robot')
            
            % Tell CS to stop halting
            Send_CS_Com('H', 0);
            
            % Set flag
            D.F.halted = false;
            
        end
        
        % Update button
        Button_State(D.UI.toggHaltRob, 'Update');
        
        % Log/print
        Update_Log(sprintf('[%s] Set to \"%d\"', 'Togg_HaltRob', get(D.UI.toggHaltRob,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ----------------------------BULLDOZE RAT---------------------------------
    function Pop_Bulldoze(~, ~, ~)
        
        if (get(D.UI.toggBulldoze, 'Value') == 1)
            
            % Get bulldoze delay
            delStr = D.UI.popBulldoze.String{get(D.UI.popBulldoze, 'Value')};
            delStr = regexp(delStr, '\d*', 'match');
            D.PAR.bullDel = str2double(delStr{:});
            D.PAR.bullSpeed = str2double(get(D.UI.editBulldoze, 'String'));
            
            % Change backround text
            Safe_Set(D.UI.toggBulldoze, 'String', sprintf('Bulldoze (%ds)', D.PAR.bullDel));
            
            % Tell CS to bulldoze
            Send_CS_Com('B', D.PAR.bullDel, D.PAR.bullSpeed);
            
        else
            
            % Change backround text
            Safe_Set(D.UI.toggBulldoze, 'String', 'Ceasefire');
            
            % Tell CS to stop bulldozing
            Send_CS_Com('B', 0, 0);
            
        end
        
        % Update button
        Button_State(D.UI.toggBulldoze, 'Update');
        
        % Log/print
        Update_Log(sprintf('[%s] Set to \"%d\" \"%d\"', 'Pop_Bulldoze', D.PAR.bullDel, D.PAR.bullSpeed));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -------------------------------REWARD------------------------------------
    function Btn_Reward(~, ~, ~)
        
        % Set reward pos and cond
        r_pos = 0;
        r_cond = 1;
        
        % Get reward zone/duration from dropdown handle
        z_ind = get(D.UI.popReward, 'Value');
        
        % Tell CS to trigger reward
        Send_CS_Com('R', r_pos, r_cond, z_ind);
        
        % Track round trip time
        D.T.btn_rew_sent = Sec_DT(now);
        
        % Log/print
        Console_Write(sprintf('[%s] Set to \"%d\"', 'Btn_Reward', get(D.UI.btnReward,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -------------------------------DO CUE------------------------------------
    function Togg_DoCue(~, ~, ~)
        
        % Update button
        Button_State(D.UI.toggDoCue, 'Update');
        
        % Set other buttons
        if get(D.UI.toggDoCue, 'Value') == 1
            
            % Unset block cue
            Safe_Set(D.UI.toggBlockCue, 'Value',0);
            Togg_BlockCue();
            
            % Enable pick rew zone
            Button_State(D.UI.toggPickRewPos, 'Enable');
            
        else
            
            % Unset force cue
            Safe_Set(D.UI.toggForceCue, 'Value',0);
            Togg_ForceCue();
            
            % Unset and disable pick cue pos
            Safe_Set(D.UI.toggPickRewPos, 'Value', 0);
            Button_State(D.UI.toggPickRewPos, 'Disable');
            
        end
        
        % Log/print
        Update_Log(sprintf('[%s] Set to \"%d\"', 'Togg_DoCue', get(D.UI.toggDoCue,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ------------------------------BLOCK CUE----------------------------------
    function Togg_BlockCue(~, ~, ~)
        
        % Update button
        Button_State(D.UI.toggBlockCue, 'Update');
        
        if (get(D.UI.toggBlockCue, 'Value') == 1)
            
            % Reset zone patches
            Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, ~D.I.zone_active), ...
                'ShowAll', D.UI.rotCol(D.I.rot,:));
            
            % Set button to not cue
            Safe_Set(D.UI.toggDoCue, 'Value', 0);
            Togg_DoCue();
        end
        
        % Log/print
        Update_Log(sprintf('[%s] Set to \"%d\"', 'Togg_BlockCue', get(D.UI.toggBlockCue,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ------------------------------FORCE CUE----------------------------------
    function Togg_ForceCue(~, ~, ~)
        
        % Update button
        Button_State(D.UI.toggForceCue, 'Update');
        
        if (get(D.UI.toggForceCue, 'Value') == 1)
            
            % Set button to cue
            Safe_Set(D.UI.toggDoCue, 'Value', 1);
            Togg_DoCue();
            
        end
        
        % Log/print
        Update_Log(sprintf('[%s] Set to \"%d\"', 'Togg_ForceCue', get(D.UI.toggForceCue,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% -------------------------PICK REWARD ZONE/TARG---------------------------
    function Togg_PickRewPos(~, ~, ~)
        
        % Update button
        Button_State(D.UI.toggPickRewPos, 'Update');
        
        if (get(D.UI.toggPickRewPos, 'Value') == 1)
            
            % Darken zone patches
            if D.PAR.sesTask == 'Track'
                Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, ~D.I.zone_active), ...
                    'ShowAll', D.UI.rotCol(D.I.rot,:));
                
                % Bring to top
                uistack(D.UI.ptchRewZoneBndsH(D.I.rot, :), 'top');
                uistack(D.UI.txtFdDurH(D.I.rot, :), 'top');
                
            end
            
            % Show forage targ patches
            if D.PAR.sesTask == 'Forage'
                
                % Get sorted index of subset of coresponding paths;
                sub_path = D.PAR.frgPathDegArr(1:D.PAR.frgPathWdt/D.PAR.frgPathSpace/2:end);
                D.PAR.frgPathSelectInd = find(ismember(D.PAR.frgPathDegArr, sub_path));
                
                % Get index of subset of targets;
                sub_targ = wrapTo360(D.PAR.frgTargDegArr(D.I.targ_now) + 180 + D.PAR.frgPathDegArr(D.PAR.frgPathSelectInd)*2);
                D.PAR.frgTargSelectInd = find(ismember(D.PAR.frgTargDegArr, sub_targ));
                
                % Sort paths to match targets
                path_sort = cell2mat(arrayfun(@(x) find(sub_targ == x), D.PAR.frgTargDegArr(D.PAR.frgTargSelectInd), 'uni', false));
                D.PAR.frgPathSelectInd = D.PAR.frgPathSelectInd(path_sort);
                
                % Make subset of targets visible
                Patch_State(D.UI.ptchRewTargBnds(D.PAR.frgTargSelectInd), ...
                    'ShowAll');
                
                % Reset path mat
                D.P.pathNowMat = zeros(size(D.P.pathNowMat));
                
            end
            
        else
            
            % Lighten zone patches
            if D.PAR.sesTask == 'Track'
                Patch_State(D.UI.ptchRewZoneBndsH(D.I.rot, ~D.I.zone_active), ...
                    'ShowAll', D.UI.rotCol(D.I.rot,:));
            end
            
        end
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ----------------------------CLEAR VT BUTTON------------------------------
    function Btn_ClrVT(~, ~, ~)
        
        % Reset handle data
        
        Safe_Set(D.UI.Rat.mrkPosLapH, ...
            'XData', NaN, 'YData', NaN);
        Safe_Set(D.UI.Rat.linVelLapH, ...
            'XData', NaN, 'YData', NaN);
        Safe_Set(D.UI.Rob.linVelLapH, ...
            'XData', NaN, 'YData', NaN);
        Safe_Set(D.UI.Rat.linPosAll, ...
            'XData', NaN, 'YData', NaN);
        Safe_Set(D.UI.Rat.linVelAll, ...
            'XData', NaN, 'YData', NaN);
        Safe_Set(D.UI.Rob.linVelAll, ...
            'XData', NaN, 'YData', NaN);
        Safe_Set(D.UI.Rat.linVelAvgH, ...
            'XData', NaN, 'YData', NaN);
        Safe_Set(D.UI.Rob.linVelAvgH, ...
            'XData', NaN, 'YData', NaN);
        
        % Reset pos data indeces
        D.P.Rat.posAll.indAll(1) = max(1, D.P.Rat.posAll.indAll(2));
        D.P.Rob.posAll.indAll(1) = max(1, D.P.Rob.posAll.indAll(2));
        D.P.Rat.posAll.indLap = D.P.Rat.posAll.indAll;
        D.P.Rob.posAll.indLap = D.P.Rob.posAll.indAll;
        
        % Reset vel data indeces
        D.P.Rat.velAll.indAll(1) = max(1, D.P.Rat.velAll.indAll(2));
        D.P.Rob.velAll.indAll(1) = max(1, D.P.Rob.velAll.indAll(2));
        D.P.Rat.velAll.indLap = D.P.Rat.velAll.indAll;
        D.P.Rob.velAll.indLap = D.P.Rob.velAll.indAll;
        
        % Reset vel hitogram data
        D.P.Rat.velAll.indHist(1) = max(1, D.P.Rat.velAll.indHist(2));
        D.P.Rob.velAll.indHist(1) = max(1, D.P.Rob.velAll.indHist(2));
        
        % Log/print
        Update_Log(sprintf('[%s] Set to \"%d\"', 'Btn_ClrVT', get(D.UI.btnClrVT,'Value')));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ----------------------------CLEAR VT BUTTON------------------------------
    function Btn_ClrTT(~, ~, ~)
        
        % Bail if cells not cut
        if get(D.UI.toggLoadClust, 'Value') == 0
            return
        end
        
        % Bail if not connected
        if ~NlxAreWeConnected() == 1
            return
        end
        
        % Clear spike plots
        Clear_Plot_TT('Spike');
        
        % Clear rate plots
        Clear_Plot_TT('Rate');
        
        % Get list of active tt/clust
        active_mat = ...
            logical(reshape(Safe_Get(D.UI.toggSubPlotTT, 'Value'), size(D.UI.toggSubPlotTT)));
        
        % Reset data inds for active buttons
        for z_tt = 1:size(D.TT.posClust,1)
            for z_c = 1:size(D.TT.posClust,2)
                if active_mat(z_tt,z_c)
                    D.TT.posClust{z_tt,z_c}.indAll(1) =  max(1,D.TT.posClust{z_tt,z_c}.indAll(2));
                    D.TT.posClust{z_tt,z_c}.indLap =  D.TT.posClust{z_tt,z_c}.indAll;
                end
            end
        end
        
    end

% ---------------------BUTTON PRESS TO LOAD TT DATA------------------------
    function Togg_SelectTT(hObject, ~, ~)
        
        %% GET HANDLE VALUES
        val = get(hObject, 'Value');
        x = get(hObject, 'UserData');
        tt_ind = x(1);
        
        % Get tt field
        D.TT.ttFldNow = D.TT.ttLab{tt_ind};
        
        % Check if not in 'Track TTs' or 'Plot Spikes' mode
        if D.UI.toggDoTrackTT.Value == 0 && ...
                D.UI.toggDoPlotTT.Value == 0
            
            % Set button back to deselect and bail
            set(hObject, 'Value', 0)
            return
        end
        
        %% HANDLE 'Plot Spikes'
        
        % Select/Deselect clust buttons if in 'Plot Spikes' mode
        if D.UI.toggDoPlotTT.Value == 1
            
            % Get active clusters
            stream_clusts = find(D.F.clust_loaded(tt_ind, :));
            
            % Set button back to inactive and bail
            if isempty(stream_clusts)
                set(hObject, 'Value', 0)
                return
            end
            
            % Unset any active clust buttons
            if val == 0
                for z_c = stream_clusts
                    Safe_Set(D.UI.toggSubPlotTT(tt_ind,z_c), 'Value', 0);
                    Togg_SubPlotTT(D.UI.toggSubPlotTT(tt_ind,z_c))
                end
            end
            
            % Update actively streaming tt flag
            D.F.tt_streaming(tt_ind) = val == 1;
            
            % Update curent TT select button
            Button_State(D.UI.toggSelectTT(tt_ind), 'Update');
            
            % Bail
            return
        end
        
        %% HANDLE 'Track TTs'
        
        % Enable tt panel objects
        Object_Group_State('TT_Turn_Panel_Objects', 'Enable')
        
        % Turn on/off NLX audio on left and right side for all chan
        if D.UI.toggDoTrackTT.Value == 1
            
            % Turn on audio for this tt
            if val == 1
                
                % Play left audio for this tt
                Safe_Set(D.UI.toggSubHearSdTT(tt_ind,1), 'Value', 1)
                Togg_SubHearTT(D.UI.toggSubHearSdTT(tt_ind,1));
                
                % Play right audio for this tt
                Safe_Set(D.UI.toggSubHearSdTT(tt_ind,2), 'Value', 1)
                Togg_SubHearTT(D.UI.toggSubHearSdTT(tt_ind,2));
                
            end
            
            % Turn off audio for this tt
            if val == 0
                
                % Turn left audio off
                if Safe_Get(D.UI.toggSubHearSdTT(tt_ind,1), 'Value') == 1
                    Safe_Set(D.UI.toggSubHearSdTT(tt_ind,1), 'Value', 0)
                    Togg_SubHearTT(D.UI.toggSubHearSdTT(tt_ind,1));
                end
                
                % Turn right audio off
                if Safe_Get(D.UI.toggSubHearSdTT(tt_ind,2), 'Value') == 1
                    Safe_Set(D.UI.toggSubHearSdTT(tt_ind,2), 'Value', 0)
                    Togg_SubHearTT(D.UI.toggSubHearSdTT(tt_ind,2));
                end
                
            end
            
        end
        
        % Bail here if button unset
        if val == 0
            
            % Re-enable all buttons
            Safe_Set(D.UI.toggSelectTT(~all(D.F.tt_chan_disable,2)), 'Enable', 'on');
            
            % Disable tt panel objects
            Object_Group_State('TT_Turn_Panel_Objects', 'Disable')
            Button_State(D.UI.toggUpdateLogTT, 'Disable');
            
            % Show all tts
            Show_Active_TT();
            
            % Update current TT select button
            if D.F.tt_updated(tt_ind)
                Button_State(D.UI.toggSelectTT(tt_ind), 'Update', D.UI.disabledCol);
            else
                Button_State(D.UI.toggSelectTT(tt_ind), 'Update');
            end
            
            % Hack to get toggle color to update
            for z_p = 1:2
                pos = D.UI.panSelectTT(z_p).Position;
                D.UI.panSelectTT(z_p).Position = [0,0,0.01,0.01];
                D.UI.panSelectTT(z_p).Position = pos;
            end
            
            % Bail
            return
            
        end
        
        % Disable all tt select buttons
        Safe_Set(D.UI.toggSelectTT(~all(D.F.tt_chan_disable,2)), 'Enable', 'off');
        
        % Enable this button
        Safe_Set(D.UI.toggSelectTT(tt_ind), 'Enable', 'on');
        
        % Show active tt
        Show_Active_TT(tt_ind);
        
        % Make sure only 1 button active
        Safe_Set(D.UI.toggSelectTT, 'Value', 0);
        Safe_Set(D.UI.toggSelectTT(tt_ind), 'Value', val);
        
        % Update all TT select button
        Button_State(D.UI.toggSelectTT, 'Update');
        
        % Change color of all updated buttons
        Safe_Set(D.UI.toggSelectTT(D.F.tt_updated), 'BackgroundColor', D.UI.disabledCol);
        
        % Update current TT select button
        Button_State(D.UI.toggSelectTT(tt_ind), 'Update');
        
        % TT has not been updated already
        if ~D.F.tt_updated(tt_ind)
            
            % Panel and text to enabled
            Panel_State(D.UI.panTrackTT, 'Enable');
            Safe_Set(D.UI.txtPanTT, 'ForegroundColor', D.UI.enabledCol)
            
            % Set update button to inactive
            Safe_Set(D.UI.toggUpdateLogTT, 'Value', 0)
            Button_State(D.UI.toggUpdateLogTT, 'Update');
            
        end
        
        % TT has been updated already
        if D.F.tt_updated(tt_ind)
            
            % Show stored orientation
            D.UI.popOr.Value = D.UI.popOr.UserData(tt_ind);
            
            % Show stored turns
            D.UI.popTrn.Value = D.UI.popTrn.UserData(tt_ind);
            
            % Show stored direction
            D.UI.popDir.Value = D.UI.popDir.UserData(tt_ind);
            
            % Show stored notes
            D.UI.editNoteTT.String = D.TT.ttLogNew.([D.TT.ttFldNow,'_N']);
            
            % Disable objects
            Object_Group_State('TT_Turn_Panel_Objects', 'Disable')
            
            % Set update button to active
            Safe_Set(D.UI.toggUpdateLogTT, 'Value', 1)
            Button_State(D.UI.toggUpdateLogTT, 'Update');
            
        end
        
        % Get last orientation
        if isundefined(D.TT.ttLogNew.([D.TT.ttFldNow,'_O']))
            
            % Use default orientation list for first session
            or_card_list =  D.PAR.orCardList;
        else
            
            % Change cardinal direction list
            orind = find(ismember(D.PAR.orCardList, D.TT.ttLogNew.([D.TT.ttFldNow,'_O'])) == 1);
            or_card_list = circshift(D.PAR.orCardList, [0,-(orind-1)]);
        end
        
        % Add 16ths measurements to orientation list
        or_list = cellfun(@(x,y) [x, blanks(4-length(x)) , y], ...
            or_card_list, D.PAR.orFracList, 'uni', false);
        
        % Get last depth
        depths_last = D.TT.ttLogNew.([D.TT.ttFldNow,'_D']);
        
        % Show last impedances
        imp_str = cellstr(num2str(D.TT.ttLogNew.([D.TT.ttFldNow,'_I'])'));
        [D.UI.editImpTT.String] = deal(imp_str{:});
        
        % Update pannel title
        Safe_Set(D.UI.panTrackTT, ...
            'Title',D.TT.ttFldNow)
        
        % Update orientation
        Safe_Set(D.UI.popOr, ...
            'String', or_list);
        
        % Update ref
        D.UI.popRefTT.Value = ...
            find(ismember(D.TT.refList, char(D.TT_IO_1.TT_Reference{D.PAR.ratIndTT,:}(ismember(D.TT.ttLab, D.TT.ttFldNow)))));
        
        % Get notes and exclude empty entries
        notes = D.TT_IO_2.(D.PAR.ratLab).([D.TT.ttFldNow,'_N']);
        notes = notes(cell2mat(cellfun(@(x) ~isempty(x), notes, 'uni', false)));
        notes = notes(~ismember(notes, 'NA'));
        inc_ind = ismember(D.TT_IO_2.(D.PAR.ratLab).([D.TT.ttFldNow,'_N']), notes);
        notes = flip(notes);
        
        % Show last note in table
        date_list = flip(D.TT_IO_2.(D.PAR.ratLab).Date(inc_ind));
        date_list = regexp(date_list, '\S*(?=_)', 'match');
        D.UI.tblNoteTT.Data = [[date_list{:}]', notes];
        D.UI.tblNoteTT.ColumnWidth = {55, 1000};
        
        % Update TT update button object
        Safe_Set(D.UI.toggUpdateLogTT, ...
            'UserData',tt_ind,...
            'String',['Update ', D.TT.ttFldNow]);
        
        % Update tt orientation text object
        Safe_Set(D.UI.txtPanTT(8), ...
            'String', or_card_list{1});
        
        % Update tt depth text object
        Safe_Set(D.UI.txtPanTT(9), ...
            'String', sprintf('%0.0f %sm', depths_last, char(181)));
        
        % Update pax view
        Move_Pax_To_TT(tt_ind);
        
        % Hack to get toggle color to update
        for z_p = 1:2
            pos = D.UI.panSelectTT(z_p).Position;
            D.UI.panSelectTT(z_p).Position = [0,0,0.01,0.01];
            D.UI.panSelectTT(z_p).Position = pos;
        end
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% --------------------BUTTON PRESS TO SAVE TT ENTRIES----------------------
    function Togg_UpdateLogTT(~, ~, ~)
        
        % Get values
        val = get(D.UI.toggUpdateLogTT, 'Value');
        x = get(D.UI.toggUpdateLogTT, 'UserData');
        tt_ind = x(1);
        
        % Check if data should be overwriten
        if val == 0
            
            % Unset flag
            D.F.tt_updated(tt_ind) = false;
            
            % Overwrite new entries
            D.TT.ttLogNew.([D.TT.ttFldNow,'_O']) = D.TT_IO_2.(D.PAR.ratLab).([D.TT.ttFldNow,'_O'])(end);
            D.TT.ttLogNew.([D.TT.ttFldNow,'_D']) = D.TT_IO_2.(D.PAR.ratLab).([D.TT.ttFldNow,'_D'])(end);
            D.TT.ttLogNew.([D.TT.ttFldNow,'_R']) = D.TT_IO_2.(D.PAR.ratLab).([D.TT.ttFldNow,'_R'])(end);
            D.TT.ttLogNew.([D.TT.ttFldNow,'_I'])(1:4) = D.TT_IO_2.(D.PAR.ratLab).([D.TT.ttFldNow,'_I'])(end,1:4);
            
            % Reset stored turns and lowering direction
            D.UI.popTrn.UserData(tt_ind) = 1;
            D.UI.popDir.UserData(tt_ind) = 1;
            D.UI.popOr.UserData(tt_ind) = 1;
            
            % Rerun TT select callback
            Togg_SelectTT(D.UI.toggSelectTT(tt_ind));
            
        end
        
        % Update values
        if val == 1
            
            % Set flag
            D.F.tt_updated(tt_ind) = true;
            
            % Update tt select button color
            Safe_Set(D.UI.toggSelectTT(tt_ind), 'BackgroundColor', D.UI.disabledCol)
            Safe_Set(D.UI.ttBndlLegMrk(tt_ind), 'MarkerFaceColor', D.UI.disabledCol);
            Safe_Set(D.UI.ttDriveLegMrk(tt_ind), 'MarkerFaceColor', D.UI.disabledCol);
            
            % Store number of turns and lowering direction
            D.UI.popTrn.UserData(tt_ind) = D.UI.popTrn.Value;
            D.UI.popDir.UserData(tt_ind) = D.UI.popDir.Value;
            
            % Disable objects
            Object_Group_State('TT_Turn_Panel_Objects', 'Disable')
            Button_State(D.UI.toggUpdateLogTT, 'Disable');
            
            % Reset and enable tt select buttons
            Safe_Set(D.UI.toggSelectTT, 'Value', 0);
            Safe_Set(D.UI.toggSelectTT(~all(D.F.tt_chan_disable,2)), 'Enable', 'on');
            
            % Show all tts
            Show_Active_TT();
            
            % Orientation list
            or_list = get(D.UI.popOr,'String');
            or_list = regexp(or_list, '[NSEW]*(?=\s)', 'match');
            or_list = [or_list{:}];
            
            % Store last and new orientation
            or_last = D.TT_IO_2.(D.PAR.ratLab).([D.TT.ttFldNow,'_O'])(end);
            or_new = char(or_list(get(D.UI.popOr,'Value')));
            
            % Get total turns
            turn_list = get(D.UI.popTrn,'String');
            turns = str2double(turn_list(get(D.UI.popTrn,'Value')));
            
            % Convert to um
            turn_um = turns*D.UI.umPerTurn;
            
            % Get turn direction
            direction_new = get(D.UI.popDir, 'Value');
            if direction_new == 1
                direction_new = 1;
            else
                direction_new = -1;
            end
            
            % Update change in orientation
            if any(ismember(or_list, or_last))
                
                % Compute delta orientation
                delta_orientation =  ...
                    (find(ismember(or_list, or_new) == 1)-1) / length(D.PAR.orCardList);
                
                % Convert to microns
                delta_orientation = delta_orientation * D.UI.umPerTurn;
                
            else
                delta_orientation = 0;
            end
            
            % Add turns
            delta_depth = direction_new * (delta_orientation + turn_um);
            
            % Get last depth
            depths_last = D.TT_IO_2.(D.PAR.ratLab).([D.TT.ttFldNow,'_D'])(end);
            
            % Compute total depth
            depth_new = delta_depth+depths_last;
            
            % Save values to temp table
            D.TT.ttLogNew.([D.TT.ttFldNow,'_O'])(:) = or_new;
            D.TT.ttLogNew.([D.TT.ttFldNow,'_D']) = depth_new;
            D.TT.ttLogNew.([D.TT.ttFldNow,'_N']){:} = get(D.UI.editNoteTT,'String');
            
            % Format impedances as numbers
            imp_str_new = Safe_Get(D.UI.editImpTT,'String');
            imp_num_new = cell2mat(cellfun(@(x) str2double(x), imp_str_new, 'uni', false));
            
            % Update ground impedance on all chanels
            if D.TT.ttLogNew.([D.TT.ttFldNow,'_I'])(5) ~= imp_num_new(5) && ...
                    ~isnan(imp_num_new(5))
                
                for z_tt = 1:length(D.TT.ttLab)
                    D.TT.ttLogNew.([D.TT.ttLab{z_tt},'_I'])(5) = imp_num_new(5);
                end
            end
            
            % Store remaining impedances
            D.TT.ttLogNew.([D.TT.ttFldNow,'_I'])(1:4) = imp_num_new(1:4);
            
            % Get reference
            ref_str = char(D.TT.refList(D.UI.popRefTT.Value));
            
            % Set all refs for this bundle
            tt_list = D.TT.ttLab(D.I.ttBndl==D.I.ttBndl(tt_ind));
            for z_tt = 1:length(tt_list)
                
                % Save for session
                tt_fld = tt_list{z_tt};
                D.TT_IO_1.TT_Reference{D.PAR.ratIndTT,:}(ismember(D.TT.ttLab,tt_fld)) = ref_str;
                
                % Save reference to table
                D.TT.ttLogNew.([tt_fld,'_R'])(:) = ref_str;
                
            end
            
            % Update pannel display
            Safe_Set(D.UI.txtPanTT(8), 'String', or_new);
            txt = sprintf('%0.0f %sm', depth_new, char(181));
            Safe_Set(D.UI.txtPanTT(9), 'String', txt);
            
        end
        
        % Update track plot
        Plot_TT_Path(tt_ind);
        
        % Update pax view
        Move_Pax_To_TT(tt_ind);
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ----------------------------SELECT TT ACTION-----------------------------
    function Togg_MainActionTT(hObject, ~, ~)
        
        %%  MAIN CODE
        
        % Handle args
        action_str = get(hObject, 'String');
        val = get(hObject, 'Value');
        
        % Enable/dissable button and turn objects
        if strcmp('Track TTs', action_str)
            
            if val == 1
                
                % Hide all 'Plot Spikes' objects
                SetPlotTTs('Inactivate');
                
                % Avtivate/Show all 'Track TTs' objects
                SetTrackTTs('Activate');
                
            else
                % Hide all 'Track TTs' objects
                SetTrackTTs('Inactivate');
                
            end
        end
        
        % Enable/dissable button and tt plot objects
        if strcmp('Plot Spikes', action_str)
            
            if val == 1
                
                % Hide all 'Track TTs' objects
                SetTrackTTs('Inactivate');
                
                % Avtivate/Show all 'Plot Spikes' objects
                SetPlotTTs('Activate');
                
            else
                % Hide all 'Plot Spikes' objects
                SetPlotTTs('Inactivate');
            end
            
        end
        
        % Hack to get toggle color to update
        for z_p = 1:2
            pos = D.UI.panSelectTT(z_p).Position;
            D.UI.panSelectTT(z_p).Position = [0,0,0.01,0.01];
            D.UI.panSelectTT(z_p).Position = pos;
        end
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
        %% NESTED FUNCTIONS
        
        % SET TT TRACK OBJECTS
        function SetTrackTTs(setting_str)
            
            % Set to 'Activate'
            if strcmp(setting_str, 'Activate')
                
                % Update main button
                Button_State(D.UI.toggDoTrackTT, 'Update')
                
                % Enable/show TT panel objects
                Panel_State(D.UI.panTrackTT, 'Enable');
                Safe_Set(D.UI.txtPanTT, 'Visible', 'on')
                Safe_Set(D.UI.spanTrackTT, 'Visible', 'on')
                Safe_Set(D.UI.popOr, 'Visible', 'on')
                Safe_Set(D.UI.popTrn, 'Visible', 'on')
                Safe_Set(D.UI.popDir, 'Visible', 'on')
                Safe_Set(D.UI.popRefTT, 'Visible', 'on')
                Safe_Set(D.UI.editImpTT, 'Visible', 'on')
                Safe_Set(D.UI.tblNoteTT, 'Visible', 'on')
                Safe_Set(D.UI.editNoteTT, 'Visible', 'on')
                Safe_Set(D.UI.toggUpdateLogTT, 'Visible', 'on')
                
                % Enable action buttons
                Button_State(D.UI.toggFlagTT, 'Enable');
                Button_State(D.UI.toggHearSourceTT, 'Enable');
            end
            
            % Set to 'Inactivate'
            if strcmp(setting_str, 'Inactivate')
                
                % Unset main button
                Safe_Set(D.UI.toggDoTrackTT, 'Value', 0)
                Button_State(D.UI.toggDoTrackTT, 'Update')
                
                % Unset any active tt select buttons
                active_ind = find(Safe_Get(D.UI.toggSelectTT, 'Value'));
                if ~isempty(active_ind)
                    Safe_Set(D.UI.toggSelectTT(active_ind), 'Value', 0)
                end
                
                % This will reset all tracking stuff
                D.UI.toggDoTrackTT.Value = 1;
                val_save = D.UI.toggDoPlotTT.Value;
                D.UI.toggDoPlotTT.Value = 0;
                Togg_SelectTT(D.UI.toggSelectTT(1));
                D.UI.toggDoTrackTT.Value = 0;
                D.UI.toggDoPlotTT.Value = val_save;
                
                % Disable and hide TT panel stuff
                Panel_State(D.UI.panTrackTT, 'Disable');
                Safe_Set(D.UI.txtPanTT, 'Visible', 'off')
                Safe_Set(D.UI.spanTrackTT, 'Visible', 'off')
                Safe_Set(D.UI.popOr, 'Visible', 'off')
                Safe_Set(D.UI.popTrn, 'Visible', 'off')
                Safe_Set(D.UI.popDir, 'Visible', 'off')
                Safe_Set(D.UI.popRefTT, 'Visible', 'off')
                Safe_Set(D.UI.editImpTT, 'Visible', 'off')
                Safe_Set(D.UI.tblNoteTT, 'Visible', 'off')
                Safe_Set(D.UI.editNoteTT, 'Visible', 'off')
                Safe_Set(D.UI.toggUpdateLogTT, 'Visible', 'off')
                
                % Unset/disable flag button
                Safe_Set(D.UI.toggFlagTT, 'Value', 0)
                Button_State(D.UI.toggFlagTT, 'Disable')
                
                % Hide flag sub buttons
                Safe_Set(D.UI.toggSubFlagTT, 'Visible', 'off')
                
                % Inactivate/disable hear source objects
                Safe_Set(D.UI.toggHearSourceTT, 'Value', 0)
                Button_State(D.UI.toggHearSourceTT, 'Disable');
                
                % Hide hear sub buttons
                Safe_Set(D.UI.toggSubHearSdTT, 'Visible', 'off')
                Safe_Set(D.UI.toggSubHearChTT, 'Visible', 'off')
                
            end
            
        end
        
        % SET TT PLOT OBJECTS
        function SetPlotTTs(setting_str)
            
            % Set to 'Activate'
            if strcmp(setting_str, 'Activate')
                
                % Update main button
                Button_State(D.UI.toggDoPlotTT, 'Update')
                
                % Reactivate active tts
                Safe_Set(D.UI.toggSelectTT(D.F.tt_streaming), 'Value', 1)
                Button_State(D.UI.toggSelectTT, 'Update')
                
                % Show clust select toggles
                Safe_Set(D.UI.toggSubPlotTT, 'Visible', 'on')
                
                % Enable both plot type toggles
                Button_State(D.UI.toggPlotTypeTT, 'Enable');
                
            end
            
            % Set to 'Inactivate'
            if strcmp(setting_str, 'Inactivate')
                
                % Unset main button
                Safe_Set(D.UI.toggDoPlotTT, 'Value', 0)
                Button_State(D.UI.toggDoPlotTT, 'Update')
                
                % Unset all all tt select buttons
                Safe_Set(D.UI.toggSelectTT, 'Value', 0)
                Button_State(D.UI.toggSelectTT, 'Update')
                
                % Hide clust select toggles
                Safe_Set(D.UI.toggSubPlotTT, 'Visible', 'off')
                Button_State(D.UI.toggSubPlotTT, 'Update');
                
                % Unset/disable plot type toggles
                Safe_Set(D.UI.toggPlotTypeTT, 'Value', 0)
                Button_State(D.UI.toggPlotTypeTT, 'Disable');
                
                % Clear spike plots
                Clear_Plot_TT('Spike');
                
                % Clear rate plots
                Clear_Plot_TT('Rate');
                
            end
            
        end
        
    end

% ---------------------------FLAG TT FEATURES------------------------------
    function Togg_FlagTT(hObject, ~, ~)
        
        %%  MAIN CODE
        
        % Get user data
        btn_str = get(hObject, 'String');
        val = get(hObject, 'Value');
        x = get(hObject, 'UserData');
        
        % Handle user data
        if strcmp(btn_str, 'Flags')
            
            % Cannot unset
            if val == 0
                Safe_Set(D.UI.toggFlagTT, 'Value', 1)
                return
            end
            
            % Unset hear button
            Safe_Set(D.UI.toggHearSourceTT, 'Value', 0)
            
            % Hide hear sub buttons
            Safe_Set(D.UI.toggSubHearSdTT, 'Visible', 'off')
            Safe_Set(D.UI.toggSubHearChTT, 'Visible', 'off')
            
            % Show flag sub buttons
            Safe_Set(D.UI.toggSubFlagTT, 'Visible', 'on')
            
        else
            
            % Get tt and button ind
            tt_ind = x(1);
            flag_btn_ind = x(2);
            
            % Update tt log value
            D.TT.ttLogNew.([D.TT.ttLab{tt_ind},'_F'])(flag_btn_ind) = val == 1;
            
            % Handle exlude tt/chan buttons
            if flag_btn_ind > 5
                
                % Initialize chanels to change
                nlx_change = false(4,1);
                
                % Handle tt flag button
                if flag_btn_ind == 6
                    
                    % Set to change all chans
                    nlx_change(:) = true;
                    
                    % Set all chan flags
                    D.F.tt_chan_disable(tt_ind,:) = val == 1;
                    
                    % Set all indevidual elecrode flag buttons
                    Safe_Set(D.UI.toggSubFlagTT(tt_ind,7:end), 'Value', val);
                    
                    % Update tt log value
                    D.TT.ttLogNew.([D.TT.ttLab{tt_ind},'_F'])(7:end) = val == 1;
                    
                end
                
                % Handle indevidual chan flag button
                if flag_btn_ind > 6
                    
                    % Set to change specific chans
                    nlx_change(flag_btn_ind-6) = true;
                    
                    % Set Specific chan flag
                    D.F.tt_chan_disable(tt_ind,flag_btn_ind-6) = val == 1;
                    
                    % Set tt flag button if all chans set the same
                    if all(D.F.tt_chan_disable(tt_ind,:)) || all(~D.F.tt_chan_disable(tt_ind,:))
                        Safe_Set(D.UI.toggSubFlagTT(tt_ind,6), 'Value', val);
                    end
                    
                end
                
                % Disable/enable tt select button
                if val == 1 && all(D.F.tt_chan_disable(tt_ind,:))
                    Safe_Set(D.UI.toggSelectTT(tt_ind), 'Enable', 'off')
                end
                if val == 0 && ~all(D.F.tt_chan_disable(tt_ind,:))
                    Safe_Set(D.UI.toggSelectTT(tt_ind), 'Enable', 'on')
                end
                
                % Initialize message vars
                if val == 1
                    set_str = 'False';
                else
                    set_str = 'True';
                end
                if tt_ind <= 9
                    win_ind = 1;
                else
                    win_ind = 2;
                end
                
                % Hide tt plots only if all chan disabled
                if all(D.F.tt_chan_disable(tt_ind,:))
                    
                    % Disable TT and times series tt plot
                    Send_NLX_Cmd(sprintf('-SetPlotEnabled "TT%d_Win" "TT%s" %s', ...
                        win_ind, D.TT.ttNum{tt_ind}, 'False'));
                    Send_NLX_Cmd(sprintf('-SetPlotEnabled "CSC%d_Win" "TT%s" %s', ...
                        win_ind, D.TT.ttNum{tt_ind}, 'False'));
                end
                
                % Show tt plots if any chan being enabled
                if val == 0
                    
                    % Enable TT and times series tt plot
                    Send_NLX_Cmd(sprintf('-SetPlotEnabled "TT%d_Win" "TT%s" %s', ...
                        win_ind, D.TT.ttNum{tt_ind}, 'True'));
                    Send_NLX_Cmd(sprintf('-SetPlotEnabled "CSC%d_Win" "TT%s" %s', ...
                        win_ind, D.TT.ttNum{tt_ind}, 'True'));
                end
                
                % Handle CSC plots
                for z_e = 1:4
                    
                    % Bail if not changing
                    if ~nlx_change(z_e)
                        continue
                    end
                    
                    % Bail after first chan if ref tt
                    if z_e > 1 && contains(D.TT.ttNum{tt_ind}, 'R')
                        continue
                    end
                    
                    % Set time series plot
                    Send_NLX_Cmd(sprintf('-SetPlotEnabled "CSC%d_Win" "CSC%s_0%d" %s', ...
                        win_ind, D.TT.ttNum{tt_ind}, z_e, set_str));
                    
                end
                
            end
        end
        
        % Update main buttons
        Button_State(D.UI.toggFlagTT, 'Update');
        Button_State(D.UI.toggSubFlagTT, 'Update');
        Button_State(D.UI.toggHearSourceTT, 'Update');
        
        % Hack to get toggle color to update
        for z_p = 1:2
            pos = D.UI.panSelectTT(z_p).Position;
            D.UI.panSelectTT(z_p).Position = [0,0,0.01,0.01];
            D.UI.panSelectTT(z_p).Position = pos;
        end
        
        % Log/print
        Update_Log(sprintf('[%s] Set \"%s\" to %d', 'Togg_FlagTT', btn_str, val));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% -----------------------SELECT TT SOUND SOURCE--------------------------
    function Togg_HearSourceTT(hObject, ~, ~)
        
        % Get user data
        btn_str = get(hObject, 'String');
        val = get(hObject, 'Value');
        btn_ind = get(hObject, 'UserData');
        
        % Cannot unset
        if val == 0
            Safe_Set(hObject, 'Value', 1)
            return
        end
        
        % Unset flag button
        Safe_Set(D.UI.toggFlagTT, 'Value', 0)
        
        % Hide flag sub buttons
        Safe_Set(D.UI.toggSubFlagTT, 'Visible', 'off')
        
        % Show hear sub buttons
        Safe_Set(D.UI.toggSubHearSdTT, 'Visible', 'on')
        Safe_Set(D.UI.toggSubHearChTT, 'Visible', 'on')
        
        % Turn off other hear source button
        Safe_Set(D.UI.toggHearSourceTT([1,2]~=btn_ind), 'Value', 0);
        
        % Handle TT audio
        if strcmp(btn_str, 'TT')
            
            % Raise volume
            Send_NLX_Cmd('-SetAudioVolume "Primary Sound Driver" Left 100');
            Send_NLX_Cmd('-SetAudioVolume "Primary Sound Driver" Right 100');
            
            % Show channel buttons
            Safe_Set(D.UI.toggSubHearChTT, 'Visible', 'on');
            
        end
        
        % Handle CSC audio
        if strcmp(btn_str, 'CS')
            
            % Lower volume
            Send_NLX_Cmd('-SetAudioVolume "Primary Sound Driver" Left 50');
            Send_NLX_Cmd('-SetAudioVolume "Primary Sound Driver" Right 50');
            
            % Unactivate/hide channel buttons
            Safe_Set(D.UI.toggSubHearChTT, 'Value', 0);
            Button_State(D.UI.toggSubHearChTT, 'Update');
            Safe_Set(D.UI.toggSubHearChTT, 'Visible', 'off');
            
        end
        
        % Get indeces of on tts
        val_mat = Safe_Get(D.UI.toggSubHearSdTT, 'Value');
        val_mat = reshape(val_mat, size(D.UI.toggSubHearSdTT));
        is_on_mat = logical(val_mat);
        tt_on_ind = find(any(is_on_mat,2));
        
        % Check if buttons should be reset
        if ~isempty(tt_on_ind)
            
            % Reset all channel buttons
            Safe_Set(D.UI.toggSubHearChTT, 'Value', 0)
            Button_State(D.UI.toggSubHearChTT, 'Update');
            
            % Turn on new source for these tts
            for z_tt = tt_on_ind'
                
                % Turn left audio for active side
                for z_sd = 1:2
                    if is_on_mat(z_tt,z_sd)
                        Safe_Set(D.UI.toggSubHearSdTT(z_tt, z_sd), 'Value', 1);
                        Togg_SubHearTT(D.UI.toggSubHearSdTT(z_tt, z_sd));
                    end
                end
                
            end
            
        end
        
        % Update buttons
        Button_State(D.UI.toggFlagTT, 'Update');
        Button_State(D.UI.toggHearSourceTT, 'Update');
        
        % Log/print
        Update_Log(sprintf('[%s] Set \"%s\" to %d', 'Togg_HearSourceTT', btn_str, val));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% -----------------------SELECT TT CHANNEL TO HEAR--------------------------
    function Togg_SubHearTT(hObject, ~, ~)
        
        % Get user data
        btn_str = get(hObject, 'String');
        val = get(hObject, 'Value');
        x = get(hObject, 'UserData');
        tt_ind = x(1);
        side_ind = x(3);
        
        % Create list of channels to turn on and off
        ent_set = ' ';
        side_str = {'Left', 'Right'};
        
        % Check what aquisition source
        if D.UI.toggHearSourceTT(1).Value == 1
            
            % Set source string
            source_str = 'TT';
            
        elseif D.UI.toggHearSourceTT(2).Value == 1
            
            % Set source string
            source_str = 'CS';
            
        else
            % Reset button if no sound source selected
            if length(x) > 1
                val = 0;
                set(hObject, 'Value', val);
            end
            
            % Bail
            return
        end
        
        % Get button values on this side
        val_mat = Safe_Get(D.UI.toggSubHearChTT(:,:,side_ind), 'Value');
        val_mat = reshape(val_mat, size(D.UI.toggSubHearChTT(:,:,side_ind)));
        is_on_mat = logical(val_mat);
        
        % Handle select side button
        if strcmp(hObject.String, 'L') || strcmp(hObject.String, 'R')
            
            % Unset all buttons for this side
            Safe_Set(D.UI.toggSubHearSdTT(:,side_ind),'Value',0)
            Safe_Set(D.UI.toggSubHearChTT(:,:,side_ind),'Value',0)
            
            if val == 0
                
                % No channels to turn off
                ent_set = 'None';
                
            elseif val == 1
                
                % Set side active for this TT active
                Safe_Set(D.UI.toggSubHearSdTT(tt_ind,side_ind),'Value',1)
                
                % Set all channels active for this TT active
                if strcmp(source_str, 'TT')
                    Safe_Set(D.UI.toggSubHearChTT(tt_ind,:,side_ind),'Value',1)
                end
                
            end
            
            % Handle indevidual channels
        elseif strcmp(source_str, 'TT')
            
            if val == 0
                
                % Check if all chan on this side off
                if ~any(is_on_mat(:))
                    Safe_Set(D.UI.toggSubHearSdTT(:,side_ind),'Value',0)
                    ent_set = 'None';
                end
                
            elseif val == 1
                
                % Unset side select button
                Safe_Set(D.UI.toggSubHearSdTT(:,side_ind),'Value',0)
                
                % Unset all chanels on this side that arent on this tt
                off_ind = true(size(D.UI.toggSubHearChTT(:,:,:)));
                off_ind(:,:,[1,2]~=side_ind) = false;
                off_ind(tt_ind,:,side_ind) = false;
                Safe_Set(D.UI.toggSubHearChTT(off_ind),'Value',0)
                
                % Reset active side select button
                Safe_Set(D.UI.toggSubHearSdTT(tt_ind,side_ind),'Value',1)
                
            end
            
        end
        
        % Get entity string
        if ~strcmp(ent_set, 'None')
            
            % Format 'TT' string
            if strcmp(source_str, 'TT')
                ent_set =  D.TT.ttLab{tt_ind};
            end
            
            % Format 'CS' string
            if strcmp(source_str, 'CS')
                ent_set =  sprintf('CSC%s_01', D.TT.ttNum{tt_ind});
            end
            
        end
        
        % Set device, side and entity
        msg_str = sprintf('%s %s', side_str{side_ind}, ent_set);
        Send_NLX_Cmd(['-SetAudioSource "Primary Sound Driver" ', msg_str]);
        
        % Check if any channels need setting
        if ~strcmp(ent_set, 'None') && ...
                strcmp(source_str, 'TT')
            
            % Get all chanels to turn on
            chan_set_on = ...
                Safe_Get(D.UI.toggSubHearChTT(tt_ind,:,side_ind),'Value') == 1;
            
            % Set each channel for this tt
            for z_chn = 1:length(chan_set_on)
                
                % Make state string
                if chan_set_on(z_chn)
                    state_str = 'true';
                else
                    state_str = 'false';
                end
                
                % Create chan setting string
                msg_str = sprintf('%s %d %s', ...
                    side_str{side_ind}, z_chn-1, state_str);
                
                % Send command
                Send_NLX_Cmd(['-SetAudioStreamEnabled "Primary Sound Driver" ', msg_str]);
                
            end
            
        end
        
        % Update buttons
        Button_State(D.UI.toggSubHearSdTT, 'Update');
        Button_State(D.UI.toggSubHearChTT, 'Update');
        
        % Log/print
        Update_Log(sprintf('[%s] Set \"%s\" to %d', 'Togg_SubHearTT', btn_str, val));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ------------------------TOGGLE TYPE OF TT PLOT---------------------------
    function Togg_PlotTypeTT(hObject, ~, ~)
        
        % Get user data
        btn_str = get(hObject, 'String');
        val = get(hObject, 'Value');
        
        % Clear spike plots
        if D.UI.toggDoPlotTT.Value == 0 || ...
                D.UI.toggPlotTypeTT(1).Value == 0
            
            % Clear spike plots
            Clear_Plot_TT('Spike');
            
        end
        
        % Clear rate plots
        if D.UI.toggDoPlotTT.Value == 0 || ...
                D.UI.toggPlotTypeTT(2).Value == 0
            
            % Clear rate plots
            Clear_Plot_TT('Rate');
            
        end
        
        % Reset axes and legends
        if D.UI.toggDoPlotTT.Value == 0
            
            % Remove all axes associations
            Safe_Set(D.UI.axClstH, 'UserData', 0)
            
            % Unset all legends and text
            for z_ax = 1:size(D.UI.axClstH)
                colormap(D.UI.axClstH(z_ax), D.UI.disabledCol)
            end
            Safe_Set(D.UI.linColBarH, 'Color', D.UI.disabledCol)
            Safe_Set(D.UI.txtColBarH, 'Visible', 'off')
            
        end
        
        % Get list of active tt/clust
        active_mat = ...
            logical(reshape(Safe_Get(D.UI.toggSubPlotTT, 'Value'), size(D.UI.toggSubPlotTT)));
        [tt_active_ind, clust_active_ind] = find(active_mat);
        
        % Set plot flag
        for z_tt = 1:length(tt_active_ind)
            for z_c = 1:length(clust_active_ind)
                D.F.plot_clust(tt_active_ind(z_tt), clust_active_ind(z_c)) = ...
                    val ~= 0 && ...
                    D.TT.posClust{tt_active_ind(z_tt), clust_active_ind(z_c)}.indLap(2) > ...
                    D.TT.posClust{tt_active_ind(z_tt), clust_active_ind(z_c)}.indLap(1);
            end
        end
        
        % Update buttons
        Button_State(D.UI.toggPlotTypeTT, 'Update');
        
        % Log/print
        Update_Log(sprintf('[%s] Set \"%s\" to %d', 'Togg_PlotTypeTT', btn_str, val));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% -----------------------------TOGGLE PLOT TT------------------------------
    function Togg_SubPlotTT(hObject, ~, ~)
        
        % Get user data
        btn_str = get(hObject, 'String');
        val = get(hObject, 'Value');
        user_dat = get(hObject, 'UserData');
        tt_ind = user_dat(1);
        clust_ind = user_dat(2);
        
        % Get axes in use
        active_ax = Safe_Get(D.UI.axClstH, 'UserData');
        
        % Bail if tt not enabled or more than 10 clust active
        if val == 1
            if Safe_Get(D.UI.toggSelectTT(tt_ind), 'Value') == 0 || ...
                    all(active_ax)
                
                % Unset button
                Safe_Set(hObject, 'Value', 0);
                Button_State(hObject, 'Update');
                
                % Bail
                return
            end
        end
        
        % Activate cluster
        if val == 1
            
            % Get index of next free axis
            ax_ind = find(active_ax == 0, 1, 'first');
            
            % Take over new axes
            D.UI.ttClustAxRef(tt_ind, clust_ind) = D.UI.axClstH(ax_ind);
            
            % Set current axis to clust color
            map = squeeze(D.UI.clustColMap(tt_ind,clust_ind,:,:));
            colormap(D.UI.axClstH(ax_ind), map)
            
            % Set legend colorbar to clust color
            map = mat2cell(map, ones(size(map,1),1), 3);
            [D.UI.linColBarH(ax_ind,:).Color] = map{:};
            
            % Set patch color
            Safe_Set(D.UI.ptchClustH(ax_ind,:), ...
                'FaceColor', D.UI.clustCol(tt_ind,clust_ind,:));
            
            % Set legend text
            set(D.UI.txtColBarH(ax_ind), ...
                'Visible', 'on', ...
                'ForegroundColor', D.UI.clustCol(tt_ind,clust_ind,:), ...
                'String', sprintf('%s-%d', D.TT.ttLab{tt_ind}, clust_ind-1));
            
            % Flag axis as in use
            set(D.UI.axClstH(ax_ind), 'UserData', 1);
            
        end
        
        % Inactivate cluster
        if val == 0
            
            % Get index of axis
            ax_ind = find(D.UI.axClstH == D.UI.ttClustAxRef(tt_ind, clust_ind));
            
            % Make sure value valid
            if ~isempty(ax_ind)
                
                % Set axis to disabled colormap
                colormap(D.UI.axClstH(ax_ind), D.UI.disabledCol)
                set(D.UI.linColBarH(ax_ind,:), 'Color', D.UI.disabledCol)
                
                % Hide legend text
                Safe_Set(D.UI.txtColBarH(ax_ind), 'Visible', 'off')
                
                % Flag axis as free
                Safe_Set(D.UI.axClstH(ax_ind), 'UserData', 0);
                
            end
            
            % Clear spike plots
            Clear_Plot_TT('Spike', tt_ind, clust_ind);
            
            % Clear rate plots
            Clear_Plot_TT('Rate', tt_ind, clust_ind);
            
        end
        
        % Set plot flag
        D.F.plot_clust(tt_ind, clust_ind) = ...
            val ~= 0 && ...
            D.TT.posClust{tt_ind, clust_ind}.indLap(2) > ...
            D.TT.posClust{tt_ind, clust_ind}.indLap(1);
        
        % Update current clust button
        Button_State(D.UI.toggSubPlotTT(tt_ind, clust_ind), 'Update');
        
        % Update tt select button
        Button_State(D.UI.toggSelectTT(tt_ind), 'Update');
        
        % Log/print
        Update_Log(sprintf('[%s] Set \"%s\" to %d', 'Togg_SubPlotTT', btn_str, val));
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
        
    end

% ----------------------SLIDER SWITCH PAXANOS IMAGES-----------------------
    function Sld_SwtchImg(hObject, ~, ~)
        
        % Get user data
        user_dat = get(hObject, 'UserData');
        obj_ind = user_dat(1);
        img_ind = round(get(hObject, 'Value'));
        
        % Change image visibility
        [D.UI.h_paxImg{obj_ind}(:).Visible] = deal('off');
        D.UI.h_paxImg{obj_ind}(img_ind).Visible = 'on';
        
        % Get coord sine
        if D.TT.imgCoor{obj_ind}(img_ind) == abs(D.TT.imgCoor{obj_ind}(img_ind))
            sin_str = '+';
        else
            sin_str = '-';
        end
        
        % Update image coordinates
        print_str = sprintf('%s%0.2f', sin_str, abs(D.TT.imgCoor{obj_ind}(img_ind)));
        Safe_Set(D.UI.txtImgCorr(obj_ind),'String',print_str)
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ----------------------------HIDE SHOW BUNDLE-----------------------------
    function Togg_HideShowBndl(hObject, ~, ~)
        
        % Get user data
        bndl_ind = get(hObject, 'UserData');
        
        % Handles
        tt_inc = ttBndl == bndl_ind;
        rod_arr_h = D.UI.ttTrkLin(tt_inc);
        sph_arr_h = D.UI.ttTrkMrk(tt_inc, :);
        
        % Set visibility
        if get(hObject, 'Value') == 0
            Safe_Set(rod_arr_h, 'Visible', 'on')
            Safe_Set(sph_arr_h, 'Visible', 'on')
        else
            Safe_Set(rod_arr_h, 'Visible', 'off')
            Safe_Set(sph_arr_h, 'Visible', 'off')
        end
        
        % Update button
        Button_State(D.UI.toggHideTT, 'Update');
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ---------------------------SET 3D PLOT VIEW------------------------------
    function Sld_Set3dView(hObject, ~, ~)
        
        % Get user data
        user_dat = get(hObject,'UserData');
        
        % Handle empty value
        if isempty(user_dat)
            user_dat = 0;
        end
        
        % Set view
        if user_dat == 1
            
            % Set to saggital
            Safe_Set(D.UI.sldYaw, 'Value', 0);
            Safe_Set(D.UI.sldPitch, 'Value', 0);
            
        elseif user_dat == 2
            
            % Set to corronal
            Safe_Set(D.UI.sldYaw, 'Value', 90);
            Safe_Set(D.UI.sldPitch, 'Value', 0);
            
        elseif user_dat == 3
            
            % Set to horizontal
            Safe_Set(D.UI.sldYaw, 'Value', 90);
            Safe_Set(D.UI.sldPitch, 'Value', -90);
            
        elseif user_dat == 4
            
            % Set to orthoginal
            Safe_Set(D.UI.sldYaw, 'Value', 45);
            Safe_Set(D.UI.sldPitch, 'Value', -45);
            
        elseif strcmp(hObject.Type, 'figure')
            
            % Get view
            view_now = -1*D.UI.axe3dH.View;
            
            % Keep in slider lims
            view_now(view_now>90) = 90;
            view_now(view_now<-90) = -90;
            
            % Set sliders
            Safe_Set(D.UI.sldYaw, 'Value', view_now(1));
            Safe_Set(D.UI.sldPitch, 'Value', view_now(2));
            
        end
        
        % Get values
        view_now(1) = -1*get(D.UI.sldYaw, 'Value');
        view_now(2) = -1*get(D.UI.sldPitch, 'Value');
        
        % Set toggle
        Safe_Set(D.UI.btnSetView(1:4 ~= user_dat), 'Value', 0);
        
        % Set axis view
        view(D.UI.axe3dH, view_now);
        
        % Update UI
        if UPDATENOW; Update_UI(0, 'force'); end
    end

% ---------------------------CHECK FOR NEW C2M-----------------------------
    function TimerFcn_CheckCSCom(~,~)
        
        % Have to explicitely catch errors
        try
            
            % Bail if c2m_com deleted or not initialized
            if ~exist('c2m_com', 'var')
                return
            elseif isempty(c2m_com)
                return
            end
            
            % Check for new packet
            c2m_com_mat = reshape(cell2mat(struct2cell(c2m_com)),1,[]);
            c2m_mat = reshape(cell2mat(struct2cell(c2m)),1,[]);
            new_ind = [c2m_com_mat.pack] ~= [c2m_mat.packLast];
            
            % Bail if no new packets
            if ~any(new_ind)
                return
            end
            
            % Process oldest message
            id_ind = [c2m_com_mat.pack] == min([c2m_com_mat(new_ind).pack]);
            
            % Get new id
            id = c2m_com_mat(id_ind).id;
            
            % Copy over data
            c2m.(id).dat1 = c2m_com.(id).dat1;
            c2m.(id).dat2 = c2m_com.(id).dat2;
            c2m.(id).dat3 = c2m_com.(id).dat3;
            c2m.(id).pack = c2m_com.(id).pack;
            
            % Update last packet
            c2m.(id).packLast = c2m.(id).pack;
            
            % Update recieve time
            c2m.(id).t_rcvd = Sec_DT(now);
            
            % Process data
            Proc_CS_Com(id)
            
        catch ME
            Console_Write('!!ERROR!! [Timer_FcnGetCSCom] FAILED', now);
            SetExit()
        end
        
    end

% ------------------------BUTTON STATUS RUN TIMER--------------------------
    function TimerFcn_Graphics(~, ~, agent)
        
        % Have to explicitely catch errors
        try
            
            % Bail if figure closed
            if ~exist('FIGH', 'var')
                return
            elseif ~ishandle(FIGH)
                return
            end
            
            % Bail if UI field not exist
            if ~exist('D', 'var')
                return
            elseif ~isfield(D, 'UI')
                return
            end
            
            % Handle input
            switch agent
                
                % Update Vcc text
                case 'Other'
                    
                    % Check for low robot voltage
                    if D.PAR.rob_vcc <= D.PAR.robVccWarning && D.PAR.rob_vcc > 0
                        
                        % Set to warning color
                        Safe_Set(D.UI.txtPerfInf(7), 'ForegroundColor', D.UI.warningCol);
                        
                        % Flicker
                        if strcmp(get(D.UI.txtPerfInf(7), 'Visible'), 'on')
                            Safe_Set(D.UI.txtPerfInf(7), 'Visible', 'off')
                        else
                            Safe_Set(D.UI.txtPerfInf(7), 'Visible', 'on')
                        end
                        
                        % Highlight new value
                    elseif c2m.('J').t_rcvd < 5.5
                        
                        % Check if printing a new value is new
                        if Sec_DT(now) - c2m.('J').t_rcvd < 5
                            
                            % Set to attention color
                            Safe_Set(D.UI.txtPerfInf(7), 'ForegroundColor', D.UI.attentionCol);
                        else
                            
                            % Set back to default color
                            Safe_Set(D.UI.txtPerfInf(7), 'ForegroundColor', D.UI.enabledCol);
                        end
                    end
                    
                    % Check for low cube voltage
                    if D.PAR.cube_vcc <= D.PAR.cubeVccWarning && D.PAR.cube_vcc > 0
                        
                        % Set to warning color
                        Safe_Set(D.UI.txtPerfInf(8), 'ForegroundColor', D.UI.warningCol);
                        
                        % Flicker
                        if strcmp(get(D.UI.txtPerfInf(8), 'Visible'), 'on')
                            Safe_Set(D.UI.txtPerfInf(8), 'Visible', 'off')
                        else
                            Safe_Set(D.UI.txtPerfInf(8), 'Visible', 'on')
                        end
                        
                        % Highlight new value
                    elseif Sec_DT(now) - D.T.cube_vcc_check < 5.5
                        
                        % Check if printing a new value is new
                        if Sec_DT(now) - D.T.cube_vcc_check < 5
                            
                            % Set to attention color
                            Safe_Set(D.UI.txtPerfInf(8), 'ForegroundColor', D.UI.attentionCol);
                        else
                            
                            % Set back to default color
                            Safe_Set(D.UI.txtPerfInf(8), 'ForegroundColor', D.UI.enabledCol);
                        end
                    end
                    
                    % Update Quit button
                case 'Quit'
                    
                    % Itterate count
                    D.UI.toggQuit.UserData = D.UI.toggQuit.UserData +1;
                    if D.UI.toggQuit.UserData>4
                        D.UI.toggQuit.UserData = 1;
                    end
                    
                    % Change button string
                    bnt_str = ...
                        sprintf('%s%s%s', agent, ...
                        repmat('.',1,D.UI.toggQuit.UserData), ...
                        blanks(4-D.UI.toggQuit.UserData));
                    Safe_Set(D.UI.toggQuit, ...
                        'String', bnt_str);
                    
                    % Update Save button
                case 'Save'
                    
                    % Itterate count
                    D.UI.toggSave.UserData = D.UI.toggSave.UserData +1;
                    if D.UI.toggSave.UserData>4
                        D.UI.toggSave.UserData = 1;
                    end
                    
                    % Change button string
                    bnt_str = ...
                        sprintf('%s%s%s', agent, ...
                        repmat('.',1,D.UI.toggSave.UserData), ...
                        blanks(4-D.UI.toggSave.UserData));
                    Safe_Set(D.UI.toggSave, ...
                        'String', bnt_str);
                    Safe_Set(D.UI.toggSave, ...
                        'String', bnt_str);
            end
            
            % Update UI
            if UPDATENOW; Update_UI(0, 'force'); end
            
        catch ME
            Console_Write('!!ERROR!! [Timer_FcnGraphics] FAILED', now);
            SetExit()
        end
        
    end

% -----------------------BUTTON STATUS STOP TIMER--------------------------
    function TimerStop_BtnStatus(~, ~, btn_str)
        
        % Have to explicitely catch errors
        try
            
            % Bail if figure closed
            if ~exist('FIGH', 'var')
                return
            elseif ~ishandle(FIGH)
                return
            end
            
            % Bail if UI field not exist
            if ~exist('D', 'var')
                return
            elseif ~isfield(D, 'UI')
                return
            end
            
            if (strcmp(btn_str,'Quit'))
                % Disable quit button
                if isfield(D.UI, 'toggQuit')
                    if ishandle(D.UI.toggQuit)
                        Safe_Set(D.UI.toggQuit, 'String', btn_str);
                        Button_State(D.UI.toggQuit, 'Disable');
                    end
                end
            end
            
            if (strcmp(btn_str,'Save'))
                % Disable save button
                if isfield(D.UI, 'toggSave')
                    if ishandle(D.UI.toggSave)
                        Safe_Set(D.UI.toggSave, 'String', btn_str);
                        Button_State(D.UI.toggSave, 'Disable');
                    end
                end
            end
            
            % Update UI
            if UPDATENOW; Update_UI(0, 'force'); end
            
        catch ME
            Console_Write('!!ERROR!! [Timer_StopBtnStatus] FAILED', now);
            SetExit()
        end
        
    end

% -------------------CHANGE FEATURES WITH TAB CHANGE-----------------------
    function Tab_GrpChange(hObject, ~, ~)
        
        % Have to explicitely catch errors
        try
            
            % Handle input
            if isa(hObject, 'matlab.ui.container.TabGroup')
                tab_now =  D.UI.tabgp.SelectedTab;
            else
                tab_now = hObject;
                D.UI.tabgp.SelectedTab = hObject;
            end
            
            % Bail if 'TABLE' tab not setup
            if ~D.F.table_tab_setup
                return
            end
            
            % Get last tab
            %tab_last = get(D.UI.tabgp, 'UserData');
            
            % Update last tab
            Safe_Set(D.UI.tabgp, 'UserData', tab_now)
            
            % Change main shared button parent
            Safe_Set(D.UI.toggMon, 'Parent', tab_now)
            Safe_Set(D.UI.toggSave, 'Parent', tab_now)
            Safe_Set(D.UI.toggQuit, 'Parent', tab_now)
            
            % Bail if on 'TABLE' tab 'TT TRACK' not setup
            if strcmp(tab_now.Title, 'TABLE') || ~D.F.tt_tab_setup
                return
            end
            
            % Enable/Disable axis rotations
            if strcmp(tab_now.Title, 'TT TRACK')
                set(D.UI.mouseRotView, 'Enable', 'on')
            else
                set(D.UI.mouseRotView, 'Enable', 'off')
            end
            
            % Change tt select objects parent
            Safe_Set(D.UI.panSelectTT, 'Parent', tab_now)
            Safe_Set(D.UI.toggDoTrackTT, 'Parent', tab_now)
            Safe_Set(D.UI.toggDoPlotTT, 'Parent', tab_now)
            Safe_Set(D.UI.toggFlagTT, 'Parent', tab_now)
            Safe_Set(D.UI.toggHearSourceTT, 'Parent', tab_now)
            Safe_Set(D.UI.toggPlotTypeTT, 'Parent', tab_now)
            
            % Set button and panel positions
            if strcmp(tab_now.Title, 'ICR ARENA')
                
                % Move panel
                for z_p = 1:2
                    D.UI.panSelectTT(z_p).Position = D.UI.tab_1_tt_select_pan_pos(z_p,:);
                end
                
                % Move action type buttons
                D.UI.toggDoTrackTT.Position = D.UI.tab_1_main_act_pos(1,:);
                D.UI.toggDoPlotTT.Position = D.UI.tab_1_main_act_pos(2,:);
                
                % Move action select buttons
                D.UI.toggFlagTT.Position = D.UI.tab_1_flag_tt_pos;
                for z_p = 1:2
                    D.UI.toggHearSourceTT(z_p).Position = D.UI.tab_1_hear_pos(z_p,:);
                    D.UI.toggPlotTypeTT(z_p).Position = D.UI.tab_1_plot_pos(z_p,:);
                end
                
            elseif strcmp(tab_now.Title, 'TT TRACK')
                
                % Move panel
                for z_p = 1:2
                    D.UI.panSelectTT(z_p).Position = D.UI.tab_2_tt_select_pan_pos(z_p,:);
                end
                
                % Move action type buttons
                D.UI.toggDoTrackTT.Position = D.UI.tab_2_main_act_pos(1,:);
                D.UI.toggDoPlotTT.Position = D.UI.tab_2_main_act_pos(2,:);
                
                % Move action select buttons
                D.UI.toggFlagTT.Position = D.UI.tab_2_flag_tt_pos;
                for z_p = 1:2
                    D.UI.toggHearSourceTT(z_p).Position = D.UI.tab_2_hear_pos(z_p,:);
                    D.UI.toggPlotTypeTT(z_p).Position = D.UI.tab_2_plot_pos(z_p,:);
                end
                
            end
            
            % Update UI
            if UPDATENOW; Update_UI(0, 'force'); end
            
        catch ME
            Console_Write('!!ERROR!! [Tab_GrpChange] FAILED', now);
            SetExit()
        end
        
    end

% -----------------------------FORCE CLOSE---------------------------------
    function DeleteFcn_ForceClose(~, ~, ~)
        
        % Dont run if global vars already deleted
        if size(who('global'),1) == 0
            return
        end
        
        % Dont run if gui closed in correct sequence
        if D.F.close
            return
        end
        
        % Run ForceClose function
        ForceClose();
        
    end






end