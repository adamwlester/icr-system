
function[] = ICR_GUI

%% =========================== MAIN SCRIPT ================================

%% --------------------------SET PARAMETERS--------------------------------

% Print that script is running
fprintf('\r\nRUNNING ICR_BEHAVIOR.m\n  Time: %s\r\n', ...
    datestr(now, 'HH:MM:SS'))

% DEBUGING
dbstop if error;
% specify if running code to debug
% run in debug mode
D.DB.run = false;
% step through iterations
D.DB.step = false;
% list each function call
D.DB.list = false;
% rat
D.DB.rat = '9859';
% ses file
D.DB.sesFi = '2015-10-06_15-34-03';
% 9857 '2015-11-10_12-55-06';
% 9859 '2015-10-06_15-34-03';
% ses file dir
D.DB.sesDir = 'E:\BehaviorPilot';
% sampling interleve
D.DB.subSmp = 1;

% Feeder paramiters
% time it should take to backfill feeders (sec)
D.PAR.fdFillDur = 30;
% boundary before and after rew feeder (deg)
D.PAR.feedBnds = [10,5];

% Timer paramiters
% pulse duration for small and large rewards (ms)
D.PAR.fdPlsDur = 500; % for arduino
% tone duration (ms)
D.PAR.toneDur = 1000; % for arduino
% ozone frequency (ms)
D.PAR.ozFreq = 1000; % for arduino
% ozone duration (ms)
D.PAR.ozDur = 1000; % for arduino
% delay for vac on (ms)
D.PAR.vacDel = 1500; %
% vac duration (ms)
D.PAR.vacDur = 500;

% NLX paramiters
D.PAR.polRate = 0.05; % poll fs (sec)

% Directories
% top dir
D.DIR.ioTop = regexp(pwd,'.*(?=\MATLAB)','match');
D.DIR.ioTop = fullfile(D.DIR.ioTop{:},'MATLAB\IOfiles');
% i/o dirs
D.DIR.ssSD = 'SessionData';
D.DIR.oppSD = 'Operational';
D.DIR.trkBndsFi = 'track_bounds(new_cam).mat';
% temp dir
D.DIR.cheetTempTop = 'C:\CheetahData\Temp';
% save dir
D.DIR.cheetSaveTop = 'E:\BehaviorPilot';

%---------------------Important variable formats---------------------------
%...........................D.UI.sndair....................................
%   val 1 = White Noise [true, false]
%   val 2 = Reward Tone [true, false]
%   val 3 = Aversive Sound [true, false]
%   val 4 = Pump [true, false]
%   val 5 = Vacuum [true, false]
%   val 6 = Ozone [true, false]
%...........................D.AC.data......................................
%   val 1 = conection [0, 1], [no, yes]
%   val 2 = display image [0, 1, 2], [Close all, 0-deg, 40-deg]
%   val 3 = rotation direction [-1, 1], [ACW, CW]
%   val 4 = feeder cue stimuli [0, 1, 2], [None or stop, Cue Feed C1, Cue Feed C2]
%   val 5 = right chan state [0, 1, 2], [no sound, no noise, noise]
%   val 6 = left chan state [1, 2], [reward tone, aversive sound]
%....................D.NLX.arduino_solnd_bit_ind..............................
%   val 1 = feeder solonoid 1
%   val 2 = feeder solonoid 2
%...................D.NLX.arduino_snd_air_bit_ind.............................
%   val 1 = speaker right chanel (white noise)
%   val 2 = speaker left chanel (reward tone)
%   val 3 = pump
%   val 4 = ozone
%   val 5 = vacuum
%--------------------------------------------------------------------------


%% ------------------------------SETUP-------------------------------------

% Run variable setup code
D = SF_Var_Setup(D);

% Run NLX setup code
D = SF_NLX_Setup(D);

% Run Arduino setup code
D = SF_Arduino_Setup(D);

% Start Arduino Polling
NlxSendCommand(D.NLX.arduino_poll_str_cmd);

% Run UI setup code
D = SF_UI_Setup(D);

% Wait for UI setup
while ~D.B.setup
    try D = guidata(D.UI.figH); catch; break; end
    drawnow;
end

% Run vacuum
% post NLX vac on command
NlxSendCommand(D.NLX.arduino_str_vac_cmd);
% post NLX reset command
NlxSendCommand(D.NLX.arduino_reset_cmd);

if D.B.setup
    
    % Run AC setup code
    D = SF_AC_Setup(D);
    
    % Run Finish setup code
    D = SF_Finish_Setup(D);
    
    % Set to start polling NLX
    D.B.poll_nlx = true;
    
    % Save out handle data
    guidata(D.UI.figH,D)
    
end


%% ---------------------------POLL NETCOM----------------------------------

while D.B.poll_nlx
    
    % Pause for NetCom's buffers.
    pause(D.PAR.polRate);
    
    % Force update UI
    drawnow;
    
    % Update UI handl data
    try
        D = guidata(D.UI.figH);
    catch; break;
    end
    
    % PROCESS VT
    
    % Run VT processing code
    D = SF_VT_Proc(D);
    
    % PLOT POSITION
    
    % Run rat in check code
    D = SF_Rat_In_Check(D);
    
    
    % -----------------------ONCE RAT IN-----------------------------------
    
    if D.B.ratIn
        
        % ROTATION TRIGGER CHECK
        
        % Run rotation check code
        D = SF_Rotation_Trig_Check(D);
        
        % REWARD FEEDER CHECK
        
        % Run reward feeder check code
        D = SF_Reward_Feeder_Check(D);
        
        
        % RAT HEADING CHECK
        
        % Run rad heading check code
        D = SF_Rat_Heading_Check(D);
        
        % LAP CHECK
        
        % Run lap check code
        D = SF_Lap_Check(D);
        
        % REWARD RESET CHECK
        
        % Run reward reset check code
        D = SF_Reward_Reset_Check(D);
        
        % CUE TRIGGER CHECK
        
        % Run cue trigger check code
        D = SF_Cue_Trigger_Check(D);
        
    end
    
    % PLOT POSITION
    
    % Run plot position code
    D = SF_Pos_Plot(D);
    
    % PRINT SES INFO
    
    % Run info print code
    D = SF_Inf_Print(D);
    
    % Store main GUI data
    guidata(D.UI.figH,D)
    
end

% =========================================================================






%% ========================= SETUP FUNCTIONS ==============================

% ------------------------------VAR SETUP----------------------------------

    function [D] = SF_Var_Setup(D)
        
        % LOAD AND DEFINE VARIABLES
        
        % Load data tables
        T = load(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_In_All.mat'));
        D.SS_In_All = T.SS_In_All;
        T = load(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_Out_ICR.mat'));
        D.SS_Out_ICR = T.SS_Out_ICR;
        clear T;
        
        % Get session variable categories
        % age group
        D.PAR.catAgeGrp = categories(D.SS_In_All.Age_Group); % [Young,Old];
        % feeder condition
        D.PAR.catFeedCnd = categories(D.SS_In_All.Feeder_Condition); % [C1,C2];
        % rotation direction
        D.PAR.catRotDrc = categories(D.SS_In_All.Rotation_Direction); % [CCW,CW];
        % cue condition
        D.PAR.catCueCond = categories(D.SS_In_All.Cue_Condition); % [All, Half, None]
        % start qauadrant
        D.PAR.catStrQuad = categories(D.SS_In_All.Start_Quadrant{1}); % [NE,SE,SW,NW];
        % rotation position
        D.PAR.catRotPos = categories(D.SS_In_All.Rotation_Positions{1}); % [90,180,270];
        
        % Get track bounds data
        % load track bound data
        S = load(fullfile(D.DIR.ioTop, D.DIR.oppSD, D.DIR.trkBndsFi));
        % track bound radius
        D.PAR.R = S.R;
        % track bound center X
        D.PAR.XC = S.XC;
        % track bound center Y
        D.PAR.YC = S.YC;
        clear S;
        
        % Initialize counter variables
        % track icr events
        D.C.rot_cnt = 0;
        % lap by rotation
        D.C.lap_cnt = num2cell(zeros(1,3));
        % rew by rotation
        D.C.rew_cnt = num2cell(zeros(1,3));
        % track reversals
        D.C.rev_cnt = 0;
        % feeder bounds counter
        D.C.inbnd_cnt = 0;
        % record counter
        D.B.rec_cnt = 0;
        
        % Initialize boolean variables
        % rotation sesion
        D.B.rot_ses = false;
        % rotation has occured
        D.B.rotated = false;
        % setup
        D.B.setup = false;
        % polling nlx
        D.B.poll_nlx = false;
        % acquiring nlx
        D.B.acq = false;
        % recording nlx
        D.B.rec = false;
        % track if rat is in arena
        D.B.ratIn = false;
        % track if next lap should be cued for half cue cond
        D.B.cue_next = true;
        % track if lap has been cued
        D.B.cue_checked = false;
        % track turn around events
        D.B.rev_reset = true;
        % track if rat reversed
        D.B.rat_rev = false;
        % track if rat is forward
        D.B.rat_fwd = true;
        % track reward reset bounds crossing
        D.B.track_rew_reset = true(3,1);
        % track lap bounds
        D.B.track_lap = false(4,1);
        % track start quadrant bounds
        D.B.track_strqd = false;
        % track rew reset bounds
        D.B.track_rset = false;
        % track feeder bounds
        D.B.track_fd = false;
        % track rotation bounds
        D.B.track_rot = false;
        % track if reward point should be plotted
        D.B.plot_rew = false;
        
        % Initialize  timers
        % time session starts
        D.T.ses_str_tim = clock;
        % total acq time
        D.T.acq_tim = 0;
        % acq restart time
        D.T.acq_tim_reset = 0;
        % total rec time
        D.T.rec_tim = 0;
        % rec restart time
        D.T.rec_tim_reset = 0;
        % track lap times
        D.T.lap_tim = 0;
        % track feeder event timing
        D.T.fd_str_tim = 0;
        % track feeder off event timing
        D.T.fd_end_tim = 0;
        % track vacuum timing
        D.T.vac_str_tim = 0;
        % track averse sound timing
        D.T.avrs_str_tim = 0;
        
        % Initialize index vars
        % current wall image index
        D.I.rot = 1;
        % feeder index
        D.I.feed_ind = [1, 2];
        % current lap quadrant for lap track
        D.I.lap_hunt_ind = 1;
        % hd sample index
        D.I.rv_smp_ind = 0;
        % current cue trigger bound
        D.I.cue_bnd_ind = 1;
        
        % Initialize arrays
        % angle data sample length (sec * vt samp rate)
        D.A.rv_nsmp = 1*30;
        % store pos data for track (c1) and heading angle (c2)
        D.A.rv_samples = NaN(D.A.rv_nsmp,2);
        % index of matching track and head angles
        D.A.rv_angmatch = circshift(1:360, 90, 2);
        
    end






% ------------------------------NLX SETUP----------------------------------

    function[D] = SF_NLX_Setup(D)
        
        %% DEFINE IO PARAMETERS
        
        % Note: Opto issolator is powered of of Port-0 Bit-3
        % Each event has a unique bit associated with it
        
        % DygitalLynx server IP
        D.NLX.IP = '192.168.3.100'; %the server we are going to connect with to get streaming data
        
        % Aquisition ent names
        D.NLX.vt_ent = 'VT1';
        D.NLX.evt_ent = 'Events';
        
        % TTL board/port parameters
        D.NLX.DevName = 'AcqSystem1_0'; % TTL board name
        
        % Port labels
        D.NLX.port_out = '0'; % output port for feeders and audio
        D.NLX.port_in1 = '1'; % input port for feeders and audio
        D.NLX.port_in2 = '2'; % input port for photo transducers
        
        % Event labels
        % feeders
        D.NLX.solnd_evt_lab = [ ...
            {'TTL_Feeder_1_Open'}, ...
            {'TTL_Feeder_2_Open'}];
        % sound/air
        D.NLX.snd_air_evt_lab = [ ...
            {'TTL_Sound_Right_On'}, ...
            {'TTL_Sound_Left_On'}, ...
            {'TTL_Sound_Left_On'}, ...
            {'TTL_Air_Pump_On'}, ...
            {'TTL_Air_Ozone_On'}, ...
            {'TTL_Air_Vacuum_On'}];
        
        % INPUT PIN/BIT
        
        % feeder solenoids
        D.NLX.solnd_bit(1) = 0;
        D.NLX.solnd_bit(2) = 1;
        % audio channels
        D.NLX.snd_rt_bit = 2;
        D.NLX.snd_lft_bit = 3;
        % air pump, ozone and vacuum
        D.NLX.air_pump_bit = 4;
        D.NLX.air_vac_bit = 5;
        D.NLX.air_ozone_bit = 6;
        
        % OUTPUT BIN/BIT
        
        % Do poll arduino pins
        D.NLX.arduino_poll_str_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['101010','00'], '"'];
        
        % Don't poll arduino pins
        D.NLX.arduino_poll_stop_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['010101','00'], '"'];
        
        % Dynamic setup command for arduino
        D.NLX.arduino_setup_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['000000','01'], '"'];
        
        % Index cammand bype string
        % feeder solenoids
        D.NLX.arduino_solnd_bit_ind = 46;
        % sound/air
        D.NLX.arduino_snd_air_bit_ind = ...
            [47 - D.NLX.snd_rt_bit, ...
            47 - D.NLX.snd_lft_bit, ...
            47 - D.NLX.snd_lft_bit, ...
            47 - D.NLX.air_pump_bit, ...
            47 - D.NLX.air_vac_bit, ...
            47 - D.NLX.air_ozone_bit];
        
        % Reset TTL pins and arduino program
        D.NLX.arduino_reset_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['000000','00'], '"'];
        
        % Run reward sequence from arduino
        D.NLX.arduino_run_rew_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['000001','10'], '"'];
        
        % Open solonoid from arduino
        D.NLX.arduino_open_slnd_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['000010','10'], '"'];
        
        % Start aversive sound from arduino
        D.NLX.arduino_str_avrs_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['000100','10'], '"'];
        
        % Start vacuum from arduino
        D.NLX.arduino_str_vac_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['001000','10'], '"'];
        
        % Stop/close sol, vac, averse
        D.NLX.arduino_stop_cmd = [ ...
            '-SetDigitalIOportString ', ...
            D.NLX.DevName, ' ', ...
            D.NLX.port_out, ...
            ' "', ['000000','10'], '"'];
        
        % PHOTO TRANSDUCER
        
        % input pins % 1 in 24 chance I guessed these correctly
        D.NLX.west_bit = '3'; % dec = 64
        D.NLX.north_bit = '2'; % dec = 16
        D.NLX.east_bit = '1'; % dec = 4
        D.NLX.south_bit = '0'; % dec = 1
        
        % Rat in event command strings
        D.NLX.rat_in_cmd{1} = '-PostEvent Post_Rat_In 201 0';
        D.NLX.rat_in_cmd{2} = '-PostEvent Post_Rat_Out 202 0';
        
        % Feeder cue event command strings
        D.NLX.feed_cue_cmd{1} = '-PostEvent Post_Feeder_Cue_On 203 0';
        D.NLX.feed_cue_cmd{2} = '-PostEvent Post_Feeder_Cue_Off 204 0';
        
        % ICR event command strings
        D.NLX.rot_cmd{1} = '-PostEvent Post_Rotation_0_Deg 205 0';
        D.NLX.rot_cmd{2} = '-PostEvent Post_Rotation_40_Deg 206 0';
        
        % Reversal event command strings
        D.NLX.rev_cmd{1} = '-PostEvent Post_Reversal_Start 207 0';
        D.NLX.rev_cmd{2} = '-PostEvent Post_Reversal_End 208 0';
        
        % Session end command string
        D.NLX.ses_end_cmd = '-PostEvent Post_Session_End 211 0';
        
        %% CONNECT TO NETCOM
        
        if ~D.DB.run
            
            % NLX setup
            top_cfg_fi = 'Cheetah.cfg'; % top level config file to load
            % Check if Cheetah is already running and if not, run Cheetah.exe
            % Start Cheetah
            curdir = pwd;
            [~,result] = system('tasklist /FI "imagename eq cheetah.exe" /fo table /nh');
            notrun = strfind(result, 'INFO');
            if notrun == 1
                cd('C:\Program Files\Neuralynx\Cheetah5');
                % Run Cheetah.exe with specific cfg file
                system(fullfile(['Cheetah.exe "' 'C:\Program Files\Neuralynx\Cheetah5\Configuration'], [top_cfg_fi,'"','&']));            end
            
            
            % Load NetCom into Matlab, and connect to the NetCom server if we aren’t connected
            while NlxAreWeConnected() ~= 1
                succeeded = NlxConnectToServer(D.NLX.IP);
                %Every NetCom function returns a value indicating if the command succeeded.
                %It is a good practice to check the succeeded value after every function call.
                %To keep the tutorial short, we will not do this for every function.
                if succeeded == 1
                    fprintf('\r\nConnected To DigitalLynx SX\n  IP: %s\n  Time: %s\r\n', ...
                        D.NLX.IP, datestr(now, 'HH:MM:SS'))
                    % Go pack to working directory
                    cd(curdir);
                end
            end
            
            %Identify this program to the server.
            NlxSetApplicationName('ICR_GUI in MATLAB');
            
            % Pair Cube headstage (Should be commented out unless router has been unpluged)
            %NlxSendCommand('-SendLynxSXCommand AcqSystem1 -InitWHSPairing 30')
            % Turn on Cube LEDs
            NlxSendCommand('-SendLynxSXCommand AcqSystem1 -WHSSetTrackingLED 1 1');
            
            % Open the data stream for the VT acquisition entity.  This tells Cheetah to begin %streaming
            % data for the VT acq ent.
            NlxOpenStream(D.NLX.vt_ent);
            NlxOpenStream(D.NLX.evt_ent);
            %             [Timestamps, EventIDs, TTLs, Extras, EventStrings, Header] =
            %              Nlx2MatEV('test.nev', [1 1 1 1 1], 1, 1, [] );
            
            % Get current folder name
            dirs = dir(D.DIR.cheetTempTop);
            D.DIR.recFi = dirs([dirs.datenum] == max([dirs.datenum])).name;
            
            % Start C# app
            %D.DIR.vbEXE = 'C:\Users\Lester\MeDocuments\Research\BarnesLab\Projects\Arena\ArenaControl\CodeVB\MyNetComStream\bin\x86\Debug\MyNetComStream.exe';
            %system(D.DIR.vbEXE);
            
        end
        
        %% CONFIGURE DIGITAL IO
        
        %         THINK THIS IS WRONG
        %         % Make sure arduino parameters are initials to everything off
        %         NlxSendCommand(D.NLX.arduino_stop_cmd);
        %         NlxSendCommand(D.NLX.arduino_setup_cmd);
        
        % Set port directions
        NlxSendCommand(['-SetDigitalIOportDirection ', D.NLX.DevName, ' ', D.NLX.port_out, ' Output']);
        NlxSendCommand(['-SetDigitalIOportDirection ', D.NLX.DevName, ' ', D.NLX.port_in2, ' Input']);
        
        % Enable digital io events
        NlxSendCommand(['-SetDigitalIOEventsEnabled ', D.NLX.DevName, ' ', D.NLX.port_in2, ' True']);
        NlxSendCommand(['-SetDigitalIOEventsEnabled ', D.NLX.DevName, ' ', D.NLX.port_out, ' True']);
        
        % Send output port command to set all TTL to zero
        NlxSendCommand(['-SetDigitalIOportString ', D.NLX.DevName, ' ', D.NLX.port_out, ' "00000000"']);
        
        % Set TTL Event strings
        % feeders
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.solnd_bit(1)), ' ', D.NLX.solnd_evt_lab{1}]);
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.solnd_bit(2)), ' ', D.NLX.solnd_evt_lab{2}]);
        % audio channels
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.snd_rt_bit), ' ', D.NLX.snd_air_evt_lab{1}]);
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.snd_lft_bit), ' ', D.NLX.snd_air_evt_lab{2}]);
        % air pump, ozone and vacuum
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.air_pump_bit), ' ', D.NLX.snd_air_evt_lab{4}]);
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.air_ozone_bit), ' ', D.NLX.snd_air_evt_lab{5}]);
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.air_vac_bit), ' ', D.NLX.snd_air_evt_lab{6}]);
        % photo transdicers
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in2, ' ', D.NLX.north_bit, ' TTL_North_On']);
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in2, ' ', D.NLX.east_bit, ' TTL_East_On']);
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in2, ' ', D.NLX.south_bit, ' TTL_South_On']);
        NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in2, ' ', D.NLX.west_bit, ' TTL_West_On']);
        
    end






% ---------------------------ARDUINO SETUP---------------------------------

    function[D] = SF_Arduino_Setup(D)
        
        % Create serial port object
        D.ARD.serObj = serial('COM18','BaudRate',9600);
        
        set(D.ARD.serObj, 'Timeout', 1);
        set(D.ARD.serObj, 'InputBufferSize', 50000);
        
        fopen(D.ARD.serObj);
        
        % Specify arduino log file
        D.DIR.logPath = fullfile(D.DIR.cheetTempTop, D.DIR.recFi, 'ArduinoLogFile.txt');
        
        % Open write-to file
        D.ARD.logID = fopen(D.DIR.logPath, 'w');
        
    end






% -------------------------------UI SETUP-----------------------------------

    function[D] = SF_UI_Setup(D)
        
        %% GENERATE FIGURE AND AXES
        
        % Bounding box for video tracker (in pixels)
        % track plot bounds (width, hight)
        D.UI.vtRes = round([D.PAR.R*2,D.PAR.R*2]);
        % track plot bounds (left, bottom)
        D.UI.lowLeft = round([D.PAR.XC-D.PAR.R,D.PAR.YC-D.PAR.R]);
        
        % Calculate pixel to cm conversion factors
        D.UI.xCMcnv = D.UI.vtRes(1)/140;
        D.UI.yCMcnv = D.UI.vtRes(2)/140;
        
        % Specify arena dimensions
        % radius (cm)
        D.UI.arnRad = 70;
        % track width (cm)
        D.UI.trkWdt = 10;
        
        % Get screen dimensions
        sc = get(0,'MonitorPositions');
        sc1 = sc(1,:);
        sc2 = sc(2,:);
        
        % Generate figure
        D.UI.figH = figure(...
            'Name', 'ICR Behavior Pilot', ...
            'MenuBar', 'none', ...
            'Color', [1, 1, 1], ...
            'Visible', 'Off');
        
        % Generate backround axis
        % backround axis
        D.UI.axH(1) = axes(...
            'Units', 'Normalized', ...
            'Visible', 'Off');
        hold on;
        
        % Generate path axis
        % Note: used for path and other dynamic features
        D.UI.axH(2) = axes(...
            'Units', 'Normalized', ...
            'Visible', 'Off');
        hold on;
        
        % Specigy figure width/hight
        fg_wh = [1240 800];
        
        % Set figure pos
        fg_pos = [ ...
            sc1(3) + (sc2(3)-fg_wh(1))/2, ...
            (sc2(4) - fg_wh(2))/2, ...
            fg_wh(1), ...
            fg_wh(2)];
        set(D.UI.figH,'Position',fg_pos);
        
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
        
        % Add cardinal coordinates text
        text(xy_max(1)+15, xy_mid(2), 'E', 'FontSize', 16, ...
            'HorizontalAlignment','center', 'VerticalAlignment','middle')
        text(xy_mid(1), xy_min(2)-15, 'S', 'FontSize', 16, ...
            'HorizontalAlignment','center', 'VerticalAlignment','middle')
        text(xy_min(1)-15, xy_mid(2), 'W', 'FontSize', 16, ...
            'HorizontalAlignment','center', 'VerticalAlignment','middle')
        text(xy_mid(1), xy_max(2)+15, 'N', 'FontSize', 16, ...
            'HorizontalAlignment','center', 'VerticalAlignment','middle')
        
        % CREATE TRACK OUTLINE
        
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
        
        %% DEFINE UI VARIABLES
        
        % GUI features
        
        % Questions dialogue pos
        D.UI.qstDlfPos = [sc1(3) + sc2(3)/2, sc2(4)/2];
        
        % Bounds of plot space
        plot_bnds = [ ...
            ax_pos(1) - 0.05, ...
            ax_pos(2) - 0.05, ...
            ax_pos(3) + 0.1, ...
            ax_pos(4) + 0.1];
        
        % Panel positions
        % recording
        D.UI.rec_pan_pos = ...
            [0, ...
            0.05+0.01, ...
            plot_bnds(1), ...
            0.275];
        % setup
        D.UI.stup_pan_pos = ...
            [0,  ...
            D.UI.rec_pan_pos(2)+ D.UI.rec_pan_pos(4)+0.01, ...
            plot_bnds(1), ...
            0.46];
        % feeder
        D.UI.feed_pan_pos = ...
            [0, ...
            D.UI.stup_pan_pos(2) + D.UI.stup_pan_pos(4) + 0.01, ...
            plot_bnds(1), ...
            1 - (D.UI.stup_pan_pos(2)+D.UI.stup_pan_pos(4)+0.01)];
        
        % Main feature colors
        % color for rotation conditions
        D.UI.rotCol = [0, 0, 1;0, 0.5, 0];
        % color of marker for current rat pos
        D.UI.posCol = [0, 0, 0];
        % cue trigger color
        D.UI.cuTrgCol = [1, 1, 0];
        
        % Other feature colors
        % primary backround color
        D.UI.backroundCol = [0.9, 0.9, 0.9];
        % default light text color
        D.UI.dfltTxtLightCol = [0.95, 0.95, 0.95];
        % print text color
        D.UI.printTxtCol = [0.1, 0.1, 0.1];
        % print text backround color
        D.UI.printBckCol = [0.95, 0.95, 0.95];
        % default light button color
        D.UI.dfltBtnLightCol = [0.75 0.75 0.75];
        % default dark button color
        D.UI.dfltBtnDarkCol = [0.3, 0.3, 0.3];
        % defualt disabled object color
        D.UI.enabledCol = [0.3, 0.3, 0.3];
        % defualt disabled object color
        D.UI.disabledCol = [0.75 0.75 0.75];
        
        % Pixel to normalzied unit conversion factor
        D.UI.pxl2norm_x = 1/fg_wh(1);
        D.UI.pxl2norm_y = 1/fg_wh(2);
        
        % VT handles
        % vt plot handle array
        D.UI.vtPltH = NaN(1,60*120/D.PAR.polRate);
        % bool for tracking VT plot handles of turn around points
        D.UI.vtPltHrev = false(1,60*120/D.PAR.polRate);
        % bool for tracking VT plot handles of reward delivery points
        D.UI.vtPltHrew = false(1,60*120/D.PAR.polRate);
        
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
        D.UI.ratList = ...
            [D.SS_In_All.Properties.RowNames(D.SS_In_All.Include_Run & D.SS_In_All.ICRb), ...
            cellstr(char(D.SS_In_All.Yoke_Mate(D.SS_In_All.Include_Run & D.SS_In_All.ICRb)))];
        D.UI.ratList = [{blanks(16)}; ...
            cellfun(@(x,y) sprintf('%s (Yk:%s)', x, y), ...
            D.UI.ratList(:,1), D.UI.ratList(:,2), 'Uni', false)];
        
        % popCond
        D.UI.condList = {''; 'ICRb Session'; 'Rotation Session'};
        D.B.rot_ses = false;
        
        % popRewDel
        D.UI.delList = {''; '0.1 '; '1.0 '; '2.0'; '3.0'};
        
        % radCue
        D.UI.cueFeed = categorical({'<undefined>'}, D.PAR.catCueCond);
        
        % radSndAir
        % labels
        D.UI.sndairLabs = [...
            {'White Noise'}, ...
            {'Reward Tone'}, ...
            {'Aversive Sound'}, ...
            {'Pump'}, ...
            {'Vacuum'}, ...
            {'Ozone'}];
        % state strings
        D.UI.sndairState = [{'Off'},{'On'}];
        % for storing sound settings
        D.UI.sndair = false(1,6);
        
        % btnAcq and btnRec
        % run BtnCol function
        [D.UI.radBtnCmap] = BtnCol(D);
        
        % btnICR
        % string
        D.UI.btnICRstr = cell(2,1);
        
        % btnSav
        % session saved
        D.UI.saved = false;
        
        % popLapTim
        % list lap times
        D.UI.lapTim = {'Lap Times'};
        
        
        %% ========================= ADD UI OBJECTS ===============================
        
        %% --------------------------FEEDER PANEL----------------------------------
        
        % Position settings
        obj_gap_ht = 0.05;
        obj_gap_wd = 0.1;
        pos_lft_dflt = 0.05;
        pos_ht_dflt = (1 - 2*obj_gap_ht) / 4 ;
        pos_wd_dflt = 0.5-pos_lft_dflt-0.05;
        
        % Font settings
        head_font_sz = ...
            [12, 16/(fg_pos(4)*D.UI.rec_pan_pos(4))*1.15] ;
        text1_font_sz = ...
            [10, 13/(fg_pos(4)*D.UI.rec_pan_pos(4))*1.15] ;
        
        % FEEDER PANEL
        D.UI.panFeed = uipanel(...
            'Parent',D.UI.figH, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'HighlightColor', D.UI.enabledCol, ...
            'FontSize',15, ...
            'FontWeight','Bold', ...
            'Title','Feeders', ...
            'TitlePosition','centertop', ...
            'Clipping','on', ...
            'Position', D.UI.feed_pan_pos);
        
        % FEEDER BUTTONS
        % feeder 1
        botm = pos_ht_dflt*3 + obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
        txtFd1H = uicontrol('Parent',D.UI.panFeed,'Style','text');
        set(txtFd1H,'String','F1', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontSize', head_font_sz(1));
        botm = pos_ht_dflt*2 + obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
        D.UI.btnFillFd(1) = uicontrol('Style','push', ...
            'Parent', D.UI.panFeed, ...
            'Enable', 'On', ...
            'String',sprintf('Fill'), ...
            'Callback', {@BtnFillFd}, ...
            'UserData', 1, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.btnFlcol(2,:), ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',text1_font_sz(1));
        botm = pos_ht_dflt*1 + obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
        D.UI.btnPlsFd(1) = uicontrol('Style','push', ...
            'Parent', D.UI.panFeed, ...
            'Enable', 'On', ...
            'String',sprintf('Pulse'), ...
            'Callback', {@BtnPlsFd}, ...
            'UserData', 1, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',text1_font_sz(1));
        botm = obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
        D.UI.btnCleanFd(1) = uicontrol('Style','push', ...
            'Parent', D.UI.panFeed, ...
            'Enable', 'On', ...
            'String',sprintf('Clean'), ...
            'Callback', {@BtnCleanFd}, ...
            'UserData', [1, 2], ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.btnFlcol(2,:), ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',text1_font_sz(1));
        
        % feeder 2
        botm = pos_ht_dflt*3 + obj_gap_ht;
        pos = [pos_lft_dflt+pos_wd_dflt+obj_gap_wd, botm, pos_wd_dflt, pos_ht_dflt];
        txtFd2H = uicontrol('Parent',D.UI.panFeed,'Style','text');
        set(txtFd2H,'String','F2', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontWeight','Bold', ...
            'FontSize', head_font_sz(1));
        botm = pos_ht_dflt*2 + obj_gap_ht;
        pos = [pos_lft_dflt+pos_wd_dflt+obj_gap_wd, botm, pos_wd_dflt, pos_ht_dflt];
        D.UI.btnFillFd(2) = uicontrol('Style','push', ...
            'Parent', D.UI.panFeed, ...
            'Enable', 'On', ...
            'String',sprintf('Fill'), ...
            'Callback', {@BtnFillFd}, ...
            'UserData', 2, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.btnFlcol(2,:), ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',text1_font_sz(1));
        botm = pos_ht_dflt*1 + obj_gap_ht;
        pos = [pos_lft_dflt+pos_wd_dflt+obj_gap_wd, botm, pos_wd_dflt, pos_ht_dflt];
        D.UI.btnPlsFd(2) = uicontrol('Style','push', ...
            'Parent', D.UI.panFeed, ...
            'Enable', 'On', ...
            'String','Pulse', ...
            'Callback', {@BtnPlsFd}, ...
            'UserData', 2, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',text1_font_sz(1));
        botm = obj_gap_ht;
        pos = [pos_lft_dflt+pos_wd_dflt+obj_gap_wd, botm, pos_wd_dflt, pos_ht_dflt];
        D.UI.btnCleanFd(2) = uicontrol('Style','push', ...
            'Parent', D.UI.panFeed, ...
            'Enable', 'On', ...
            'String','Clean', ...
            'Callback', {@BtnCleanFd}, ...
            'UserData', [2, 2], ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.btnFlcol(2,:), ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',text1_font_sz(1));
        
        %% ---------------------------SETUP PANEL----------------------------------
        
        % Position settings
        obj_gap_ht = 0.025;
        pos_ht_dflt = (1 - 2*obj_gap_ht) / 8;
        pos_lft_dflt = 0.05;
        pos_wd_dflt = 1-pos_lft_dflt*2;
        pxl2normStup_y = D.UI.pxl2norm_y/D.UI.stup_pan_pos(4);
        
        % Font settings
        head_font_sz = ...
            [10, 13/(fg_pos(4)*D.UI.stup_pan_pos(4))*1.15] ;
        text1_font_sz = ....
            [9, 12/(fg_pos(4)*D.UI.stup_pan_pos(4))] ;
        text2_font_sz = ...
            [9, 12/(fg_pos(4)*D.UI.stup_pan_pos(4))] ;
        
        % SETUP PANEL
        D.UI.panStup = uipanel(...
            'Parent',D.UI.figH, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'HighlightColor',D.UI.enabledCol, ...
            'FontSize',15, ...
            'FontWeight','Bold', ...
            'Title','Setup', ...
            'TitlePosition','centertop', ...
            'Clipping','on', ...
            'Position', D.UI.stup_pan_pos);
        
        % RAT SELECTION
        % text
        botm = pos_ht_dflt*8 + obj_gap_ht - head_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
        D.UI.txtRat = uicontrol('Style','text', ...
            'Parent',D.UI.panStup, ...
            'String','Rat Number', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName','MS Sans Serif', ...
            'FontSize', head_font_sz(1));
        % popupmenu
        botm = pos_ht_dflt*7 + obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)*1.9];
        D.UI.popRat = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.panStup, ...
            'Callback', {@PopRat}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',[1 1 1], ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',text1_font_sz(1), ...
            'FontWeight','Bold', ...
            'String',D.UI.ratList, ...
            'Value',1);
        
        % ICR CONDITION
        % text
        botm = pos_ht_dflt*7 + obj_gap_ht - head_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
        D.UI.txtCond = uicontrol('Style','text', ...
            'Parent',D.UI.panStup, ...
            'String','ICR Condition', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName','MS Sans Serif', ...
            'FontSize', head_font_sz(1));
        % popupmenu
        botm = pos_ht_dflt*6 + obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)*1.9];
        D.UI.popCond = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.panStup, ...
            'Callback', {@PopCond}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',[1 1 1], ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',text1_font_sz(1), ...
            'FontWeight','Bold', ...
            'String',D.UI.condList, ...
            'Value',1);
        
        % REWARD DELAY
        % text
        botm = pos_ht_dflt*6 + obj_gap_ht - head_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
        D.UI.txtRwDl = uicontrol('Style','text', ...
            'Parent',D.UI.panStup, ...
            'String','Reward Delay', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'HorizontalAlignment', 'Center', ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontWeight','Bold', ...
            'FontName','MS Sans Serif', ...
            'FontSize', head_font_sz(1));
        % popupmenu
        botm = pos_ht_dflt*5 + obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)*1.9];
        D.UI.popRwDl = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.panStup, ...
            'Callback', {@PopRewDel}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',[1 1 1], ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',text1_font_sz(1), ...
            'FontWeight','Bold', ...
            'String',D.UI.delList, ...
            'Value',1);
        
        % CUE BUTTON PANEL
        offset = 0.05;
        botm = pos_ht_dflt*4 + obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
        D.UI.spanCue = uibuttongroup(...
            'Parent',D.UI.panStup, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'HighlightColor',D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Cue Condition', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',head_font_sz(1), ...
            'Clipping','off');
        % CUE CONDITION
        % all
        botm = botm + text2_font_sz(2)*0.75;
        wdth = pos_wd_dflt/3.25;
        pos = [pos_lft_dflt + offset, botm, wdth, text2_font_sz(2)*1.1];
        D.UI.radCue(1) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@BtnCue}, ...
            'String','All', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        % half
        pos = [pos_lft_dflt + offset + wdth, botm, wdth, text2_font_sz(2)*1.1];
        D.UI.radCue(2) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@BtnCue}, ...
            'String','Half', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        % none
        pos = [pos_lft_dflt + offset + wdth*2, botm, wdth, text2_font_sz(2)*1.1];
        D.UI.radCue(3) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@BtnCue}, ...
            'String','None', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        
        % SOUND/AIR BUTTON PANEL
        offset = 0.05;
        panBtm = pos_ht_dflt*1.5 + obj_gap_ht;
        % set pos for each radiobutton
        botm([1,4]) = panBtm + pos_ht_dflt*1.25 + obj_gap_ht;
        botm([2,5]) = panBtm + pos_ht_dflt*0.75 + obj_gap_ht;
        botm([3,6]) = panBtm + pos_ht_dflt*0.25 + obj_gap_ht;
        lft(1:3) = pos_lft_dflt + offset;
        lft(4:6) = pos_lft_dflt + pos_wd_dflt/1.75;
        % panel
        pos = [pos_lft_dflt, ...
            panBtm, ...
            pos_wd_dflt, ...
            pos_ht_dflt*2.5];
        D.UI.spanSndAir = uibuttongroup(...
            'Parent',D.UI.panStup, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'HighlightColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Sound/Air', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',head_font_sz(1), ...
            'Clipping','off');
        % SOUND/AIR CONDITION
        % white
        pos = [lft(1), botm(1), pos_wd_dflt/2.25, text2_font_sz(2)*1.1];
        D.UI.radSndAir(1) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@RadSndAir}, ...
            'UserData', 1, ...
            'String','White', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        % reward
        pos = [lft(2), botm(2), pos_wd_dflt/2.25, text2_font_sz(2)*1.1];
        D.UI.radSndAir(2) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@RadSndAir}, ...
            'UserData', 2, ...
            'String','Reward', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        % aversive
        pos = [lft(3), botm(3), pos_wd_dflt/2, text2_font_sz(2)*1.1];
        D.UI.radSndAir(3) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@RadSndAir}, ...
            'UserData', 3, ...
            'String','Aversive', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        % pump
        pos = [lft(4), botm(4), pos_wd_dflt/2.5, text2_font_sz(2)*1.1];
        D.UI.radSndAir(4) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@RadSndAir}, ...
            'UserData', 4, ...
            'String','Pump', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        % vac
        pos = [lft(5), botm(5), pos_wd_dflt/2.5, text2_font_sz(2)*1.1];
        D.UI.radSndAir(5) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@RadSndAir}, ...
            'UserData', 5, ...
            'String','Vacuum', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        % ozone
        pos = [lft(6), botm(6), pos_wd_dflt/2.5, text2_font_sz(2)*1.1];
        D.UI.radSndAir(6) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@RadSndAir}, ...
            'UserData', 6, ...
            'String','Ozone', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        
        % SETUP DONE
        botm = 15*pxl2normStup_y;
        pos = [0.25, botm, 0.5, 45*pxl2normStup_y];
        D.UI.btnStupDn = uicontrol('Style','push', ...
            'Parent', D.UI.panStup, ...
            'String','DONE', ...
            'Callback', {@BtnStupDn}, ...
            'UserData', 0, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',12, ...
            'Value',0);
        
        %% --------------------------RECORD PANEL----------------------------------
        
        % Position settings
        obj_gap_ht = 0.025;
        pos_ht_dflt = (1 - 2*obj_gap_ht) / 4;
        pos_lft_dflt = 0.1;
        pos_wd_dflt = 1-pos_lft_dflt*2;
        pxl2normRec_y = D.UI.pxl2norm_y/D.UI.rec_pan_pos(4);
        
        % Font settings
        head_font_sz = ...
            [12, 16/(fg_pos(4)* D.UI.rec_pan_pos(4))*1.15];
        text1_font_sz = ...
            [10, 13/(fg_pos(4)* D.UI.rec_pan_pos(4))];
        btn_rot_px_ht = 40/(fg_pos(4)* D.UI.rec_pan_pos(4));
        
        % RECORD PANEL
        D.UI.panRec = uipanel(...
            'Parent',D.UI.figH, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'FontSize',15, ...
            'FontWeight','Bold', ...
            'Title','Record', ...
            'TitlePosition','centertop', ...
            'Clipping','on', ...
            'Position',  D.UI.rec_pan_pos);
        
        % NEURALYNX BUTTON PANEL
        botm = pos_ht_dflt*3 + obj_gap_ht;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
        D.UI.spanNLX = uibuttongroup(...
            'Parent',D.UI.panRec, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'HighlightColor', D.UI.enabledCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'Title','Cheetah', ...
            'TitlePosition','centertop', ...
            'FontWeight','Bold', ...
            'FontSize',head_font_sz(1), ...
            'Clipping','off');
        
        % ACQ BUTTON
        offst = 0.1;
        botm = pos_ht_dflt*3 + obj_gap_ht + text1_font_sz(2);
        wdth = pos_wd_dflt*0.4;
        pos = [pos_lft_dflt + offst, botm, wdth, text1_font_sz(2)*1.2];
        D.UI.btnAcq = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panRec, ...
            'UserData', 0, ...
            'Enable', 'Off', ...
            'Visible', 'Off', ...
            'Callback', {@BtnAcq}, ...
            'UserData', 0, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'String','ACQ', ...
            'CData',D.UI.radBtnCmap{1}, ...
            'FontWeight','Bold', ...
            'FontWeight','Bold', ...
            'FontSize', text1_font_sz(1), ...
            'Value',0);
        % REC BUTTON
        botm = pos_ht_dflt*3 + obj_gap_ht + text1_font_sz(2);
        wdth = pos_wd_dflt*0.4;
        pos = [pos_lft_dflt + wdth + offst, botm, wdth, text1_font_sz(2)*1.2];
        D.UI.btnRec = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panRec, ...
            'Enable', 'Off', ...
            'Visible', 'Off', ...
            'Callback', {@BtnRec}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
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
            'Enable', 'Off', ...
            'Callback', {@BtnICR}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.dfltBtnLightCol, ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',12, ...
            'UserData', false);
        
        % SESSION DONE
        botm = 15*pxl2normRec_y;
        pos = [0.25, botm, 0.5, 45*pxl2normRec_y];
        D.UI.btnRecDn = uicontrol('Style','push', ...
            'Parent', D.UI.panRec, ...
            'Enable', 'Off', ...
            'String','DONE', ...
            'Callback', {@BtnRecDn}, ...
            'UserData', 0, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.dfltBtnLightCol, ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize',12);
        
        %% --------------------------OTHER BUTTONS---------------------------------
        
        % SAVE SESSION DATA
        pos = [0, plot_bnds(2), plot_bnds(1)/2, 0.05];
        D.UI.btnSav = uicontrol('Style','push', ...
            'Enable', 'Off', ...
            'String','SAVE', ...
            'Callback', {@BtnSav}, ...
            'Unit', 'Normalized', ...
            'Position', pos, ...
            'FontSize',14, ...
            'Value',0);
        
        % QUIT ALL
        pos = [plot_bnds(1)/2, plot_bnds(2), plot_bnds(1)/2, 0.05];
        D.UI.btnQuit = uicontrol('Style','push', ...
            'String','QUIT', ...
            'Callback', {@BtnQuit}, ...
            'Unit', 'Normalized', ...
            'Position', pos, ...
            'FontSize',14);
        
        % CLEAR VT BUTTON
        pos = [plot_bnds(3)+plot_bnds(1)-0.16, 0.01, 0.14, 0.03];
        D.UI.btnClrVT = uicontrol('Style','push', ...
            'Parent',D.UI.figH, ...
            'Enable', 'Off', ...
            'Callback', {@BtnClrVT}, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'String','Clear VT Data', ...
            'BackgroundColor', D.UI.dfltBtnLightCol, ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
            'FontWeight','Bold', ...
            'FontSize', 10, ...
            'Value',0);
        
        %% --------------------------PRINTED INFO----------------------------------
        
        % Position settings
        obj_gap_ht = 0.025;
        pos_lft_dflt = (plot_bnds(1)+plot_bnds(3)) + 0.005;
        pos_wd_dflt = 1 - (plot_bnds(1)+plot_bnds(3)) - 2*0.005;
        pan_lft = (plot_bnds(1)+plot_bnds(3));
        pan_wd = 1 - (plot_bnds(1)+plot_bnds(3));
        
        % Font settings
        head_font_sz = ...
            [12, 16*D.UI.pxl2norm_y*1.25];
        text_font_sz = ...
            [10, 13*D.UI.pxl2norm_y*1.25];
        dd_font_sz = ...
            [10, 22*D.UI.pxl2norm_y];
        
        % RAT INFO
        
        % Number of text lines not including header
        nlines(1) = 11;
        
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
            'Parent',D.UI.figH, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'Position',  D.UI.sesInfPanPos);
        % heading
        botm = D.UI.sesInfPanBtm + ...
            D.UI.sesInfPanHt - ...
            head_font_sz(2) - ...
            4*D.UI.pxl2norm_y;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
        D.UI.txtSesInfHed = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','Session Info', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', head_font_sz(1));
        
        % ICR ses info
        botm = botm - 6*text_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*5];
        D.UI.txtSesInf(1) = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.printTxtCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'FontSize', text_font_sz(1));
        
        % Rotation ses info
        botm = botm - 3*text_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*2];
        D.UI.txtSesInf(2) = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.printTxtCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'FontSize', text_font_sz(1));
        % rewards per dropdown
        botm = botm - dd_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
        D.UI.popRewPerRot = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.figH, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',[1 1 1], ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',dd_font_sz(1), ...
            'FontWeight','Light', ...
            'Visible','Off', ...
            'Value',1);
        % rotation position dropdown
        botm = botm - dd_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
        D.UI.popRotPos = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.figH, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',[1 1 1], ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',dd_font_sz(1), ...
            'FontWeight','Light', ...
            'Visible','Off', ...
            'Value',1);
        
        % PERFORMANCE INFO
        
        % Number of text lines not including header
        nlines(2) = 13;
        
        % pannel
        D.UI.perfInfPanHt = ...
            (nlines(2)-1)*text_font_sz(2) + ...
            1*dd_font_sz(2) + ...
            head_font_sz(2) + ...
            4*D.UI.pxl2norm_y*2;
        D.UI.perfInfPanBtm = ...
            D.UI.sesInfPanBtm - ...
            D.UI.perfInfPanHt - ...
            obj_gap_ht;
        D.UI.perfInfPanPos = [...
            pan_lft, ...
            D.UI.perfInfPanBtm, ...
            pan_wd, ...
            D.UI.perfInfPanHt];
        D.UI.panPerfInf = uipanel(...
            'Parent',D.UI.figH, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'Position',  D.UI.perfInfPanPos);
        % heading
        botm = D.UI.perfInfPanBtm + ...
            D.UI.perfInfPanHt - ...
            head_font_sz(2) - ...
            4*D.UI.pxl2norm_y;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
        D.UI.txtPerfInfHed = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','Performance Info', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', head_font_sz(1));
        % total
        botm = botm - 5*text_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*4];
        D.UI.txtPerfInf(4) = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.printTxtCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text_font_sz(1));
        % lap time dropdown
        botm = botm - dd_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
        D.UI.popLapTim = uicontrol('Style','popupmenu', ...
            'Parent',D.UI.figH, ...
            'String',D.UI.lapTim, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor',[1 1 1], ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontSize',dd_font_sz(1), ...
            'FontWeight','Light', ...
            'Visible','Off', ...
            'Value',1);
        % standard
        botm = botm - 3*text_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*2];
        D.UI.txtPerfInf(1) = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', [0.5,0.5,0.5], ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text_font_sz(1));
        % 40 deg
        botm = botm - 2*text_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*2];
        D.UI.txtPerfInf(2) = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.rotCol(2,:), ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text_font_sz(1));
        % 0 deg
        botm = botm - 2*text_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*2];
        D.UI.txtPerfInf(3) = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.rotCol(1,:), ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text_font_sz(1));
        
        % TIMER INFO
        
        % Number of text lines not including header
        nlines(3) = 8;
        
        % pannel
        D.UI.timInf_pan_pos = [...
            pan_lft, ...
            0, ...
            pan_wd, ...
            nlines(3)*text_font_sz(2) + head_font_sz(2) + 4*D.UI.pxl2norm_y*2];
        D.UI.panTimInf = uipanel(...
            'Parent',D.UI.figH, ...
            'Units','Normalized', ...
            'BorderType','line', ...
            'BorderWidth',4, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'HighlightColor', D.UI.disabledCol, ...
            'Position',  D.UI.timInf_pan_pos);
        % heading
        botm = text_font_sz(2)*nlines(3)+4*D.UI.pxl2norm_y;
        pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
        D.UI.txtTimInfHed = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','Timer Info', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', head_font_sz(1));
        % time elapsed info
        botm = botm - text_font_sz(2)*5;
        pos = [pos_lft_dflt, botm, ...
            pos_wd_dflt, text_font_sz(2)*4];
        D.UI.txtTimElspInf = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.printTxtCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text_font_sz(1));
        % time start info
        botm = botm - text_font_sz(2)*2;
        pos = [pos_lft_dflt, botm, ...
            pos_wd_dflt, text_font_sz(2)*1];
        D.UI.txtTimStrInf = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.printTxtCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'FontSize', text_font_sz(1));
        % time stop info
        botm = 4*D.UI.pxl2norm_y;
        pos = [pos_lft_dflt, botm, ...
            pos_wd_dflt, text_font_sz(2)*1];
        D.UI.txtTimEndInf = uicontrol('Style','text', ...
            'Parent',D.UI.figH, ...
            'String','', ...
            'Units','Normalized', ...
            'HorizontalAlignment', 'Left', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.printBckCol, ...
            'ForegroundColor', D.UI.printTxtCol, ...
            'FontName','Courier New', ...
            'FontWeight','Light', ...
            'FontSize', text_font_sz(1));
        
        %% -----------------------------TIMERS-------------------------------------
        
        % Fill stop timer
        D.UI.timFdFillDur = timer('TimerFcn',@TimStopFill, ...
            'UserData', 0, ...
            'StartDelay', D.PAR.fdFillDur);
        
        %% ========================================================================
        
        
        %% MAKE FIGURE VISIBLE
        
        % Set vidibility
        set(D.UI.figH, 'Visible', 'On');
        
        % Bring UI to top
        uistack(D.UI.figH, 'top')
        
        % Store main GUI data
        guidata(D.UI.figH, D)
        
        
        
        
    end






% ----------------------------RAT IN CHECK---------------------------------

    function [D] = SF_Rat_In_Check(D)
        
        % Keep checking if rat is in the arena
        D.B.track_strqd = ...
            D.P.rad >= min(D.UI.strQuadBnds) & ...
            D.P.rad <= max(D.UI.strQuadBnds);
        if ...
                any(D.B.track_strqd) && ...
                ~D.B.ratIn &&  ...
                D.B.rec;
            
            % Change bool so code will only be run once
            D.B.ratIn = true;
            
            % Keep start quad patch but lighten
            set(D.UI.ptchStQ, ...
                'FaceAlpha', 0.05, ...
                'EdgeAlpha', 0.05);
            
            % Post NLX event: rat in
            NlxSendCommand(D.NLX.rat_in_cmd{1});
            
            % Start tracking lap times
            D.T.lap_tim = clock;
            
        end
        
    end






% -------------------------------AC SETUP----------------------------------

    function [D] = SF_AC_Setup(D)
        
        %% Setup communication with ARENACONTROLLER
        D.AC.IP = '172.17.0.3';
        if D.DB.run
            D.AC.IP = '192.168.0.107';
        end
        
        % Initialize parameters
        % signal to connect to ARENACONTROLLER
        D.AC.data(1) = 1;
        % signal for ICR condition
        D.AC.data(2) = 0;
        % signal for rotation direction
        D.AC.data(3) = 0;
        % signal for feeder cue
        D.AC.data(4) = 0;
        % right chan (white noise) sound state
        D.AC.data(5) = 0;
        % left chan (reward/aversive) sound state
        D.AC.data(6) = 0;
        % unused
        D.AC.data(7:10) = 0;
        
        % Move to data variable
        data = D.AC.data;
        
        % Get byte size
        s = whos('data');
        
        % Clear data var
        clear('data')
        
        % Create tcpip object
        D.AC.tcpIP = tcpip(D.AC.IP,30000,'NetworkRole','Server');
        
        % Set bitesize
        set(D.AC.tcpIP,'OutputBufferSize',s.bytes);
        
        % Establish connection
        if ~D.DB.run
            % will loop here until D.AC.data(1) established
            fopen(D.AC.tcpIP);
        end
        
        % Write initial data to AC computer
        if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
        
        % Print that AC computer is connected
        fprintf('\r\nConnected To AC Computer\n  IP: %s\n  Time: %s\r\n', ...
            D.AC.IP, datestr(now, 'HH:MM:SS'))
        
        %% Update AC.data values when ready to poll
        
        % Display image
        D.AC.data(2) = D.I.rot;
        
        % Rotation direction
        D.AC.data(3) = D.PAR.ratRotDrc_Num;
        
        % Feeder cue stimuli (start without cue)
        D.AC.data(4) = 0;
        
        % Left chan state
        D.AC.data(6) = 1; % default to reward noise
        
        % Post to AC computer
        if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
        
        % Reset rotation direction after first run
        D.AC.data(3) = 0; % rotation direction
        
    end






% ----------------------------FINISH SETUP---------------------------------
    function [D] = SF_Finish_Setup(D)
        
        
        %% Update UI objects
        
        % Disable all setup buttons but not sound buttons
        set(D.UI.panStup, 'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor',D.UI.disabledCol)
        set(D.UI.popRat, 'Enable', 'Off')
        set(D.UI.popCond, 'Enable', 'Off')
        set(D.UI.popRwDl, 'Enable', 'Off')
        set(D.UI.radCue, 'Enable', 'Off')
        set(D.UI.btnStupDn, 'Enable', 'Off', ...
            'BackgroundColor', D.UI.disabledCol)
        
        % Enable acq record buttons
        set(D.UI.panRec, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'HighlightColor', D.UI.enabledCol)
        set(D.UI.btnAcq, ...
            'Enable', 'On', ...
            'Visible', 'On')
        set(D.UI.btnRec, ...
            'Enable', 'On', ...
            'Visible', 'On')
        
        % Enable inf panels
        set(D.UI.panSesInf, 'HighlightColor', D.UI.enabledCol)
        set(D.UI.panPerfInf, 'HighlightColor', D.UI.enabledCol)
        set(D.UI.panTimInf, 'HighlightColor', D.UI.enabledCol)
        
        % Enable other popup menues
        set(D.UI.popRewPerRot, 'Visible', 'on');
        set(D.UI.popRotPos, 'Visible', 'on');
        set(D.UI.popLapTim, 'Visible', 'on');
        
        %% Print session and performance and time info
        
        % Print session info
        % get values to print
        dobNum = datenum(D.PAR.ratDOB, 'yyyy/mm/dd');
        agemnth = num2str(floor((now - dobNum)/365*12));
        ses = num2str(D.PAR.sesNum);
        infstr = sprintf([...
            'ICRb_Session:%s%s\n', ...
            'Age_Months:%s%s\n', ...
            'Age_Group:%s%s\n', ...
            'Feeder:%s%s\n', ...
            'Rotation:%s%s\n', ...
            'Start_Quad:%s%s\n'], ...
            repmat('_',1,5), ses, ...
            repmat('_',1,7), agemnth, ...
            repmat('_',1,8), char(D.PAR.ratAgeGrp), ...
            repmat('_',1,11), char(D.PAR.ratFeedCnd), ...
            repmat('_',1,9), char(D.PAR.ratRotDrc), ...
            repmat('_',1,7), char(D.PAR.ratStrQuad));
        set(D.UI.txtSesInf(1),'String', infstr)
        
        if D.B.rot_ses
            rotSes = num2str(D.PAR.sesNumICR);
            rots = num2str(D.PAR.rotPerSes);
        else
            rotSes = 'NA';
            rots = 'NA';
            set(D.UI.popRewPerRot, 'Enable', 'Off');
            set(D.UI.popRotPos, 'Enable', 'Off');
        end
        infstr = sprintf([...
            'Session_Rotation:%s%s\n', ...
            'Rotations:%s%s\n'], ...
            repmat('_',1,1), rotSes, ...
            repmat('_',1,8), rots);
        set(D.UI.txtSesInf(2),'String', infstr)
        
        
        % Print performance info
        % total
        infstr = sprintf([...
            'Laps______Total:%s%d\n', ...
            'Rewards___Total:%s%d\n', ...
            'Reversals_Total:%s%d\n', ...
            'Rotations_Total:%s%d'], ...
            repmat('_',1,2), 0, ...
            repmat('_',1,2), 0, ...
            repmat('_',1,2), 0, ...
            repmat('_',1,2), 0);
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
        
        % Print time info
        % elapsed
        infstr = sprintf([ ...
            '(SES):%s%s\n', ...
            '(ACQ):%s%s\n', ...
            '(REC):%s%s\n', ...
            '(LAP):%s%s\n'], ...
            repmat('_',1,6), datestr(0, 'HH:MM:SS'), ...
            repmat('_',1,6), datestr(0, 'HH:MM:SS'), ...
            repmat('_',1,6), datestr(0, 'HH:MM:SS'), ...
            repmat('_',1,6), datestr(0, 'HH:MM:SS'));
        set(D.UI.txtTimElspInf, 'String', infstr)
        % start
        infstr = sprintf( ...
            'Start:%s%s\n', ...
            repmat('_',1,6), datestr(D.T.ses_str_tim, 'HH:MM:SS'));
        set(D.UI.txtTimStrInf, 'String', infstr)
        % end
        infstr = sprintf( ...
            'Stop_:%s%s\n', ...
            repmat('_',1,6), datestr(0, 'HH:MM:SS'));
        set(D.UI.txtTimEndInf, 'String', infstr)
        
        %% Get bounds for various opperations
        
        % Specify reward feeder locations
        % NOTE: Feeder index is based on the position of the feeder with
        % respect to the 0 deg point (East quadrant)in the arena
        rewFeeds = [11, 15; 29, 33];
        
        % Get reward and unrewarded feeder based on rotation direction
        % corrected index
        D.UI.rewFeed = rewFeeds(D.PAR.ratFeedCnd_Num, D.I.feed_ind);
        D.UI.oppFeed = rewFeeds([1, 2] ~=  D.PAR.ratFeedCnd_Num, D.I.feed_ind);
        
        % Calculate feeder locations
        
        % Calculate all feeder locations
        fdLocs = circshift((0:10:350)+5,[0,0]);
        
        % Calculate all feed locs
        [fd_X,fd_Y] = pol2cart(deg2rad(fdLocs), ones(1,length(fdLocs)) * D.UI.arnRad);
        % all feeders x
        D.UI.fd_x = fd_X*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
        % all feeders y
        D.UI.fd_y = fd_Y*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
        
        % REWARD FEEDER BOUNDS
        D.UI.rewBnds = [...
            deg2rad(fdLocs(D.UI.rewFeed(1))) - deg2rad(D.PAR.feedBnds(2)), ...
            deg2rad(fdLocs(D.UI.rewFeed(1))) + deg2rad(D.PAR.feedBnds(1)); ...
            deg2rad(fdLocs(D.UI.rewFeed(2))) - deg2rad(D.PAR.feedBnds(2)), ...
            deg2rad(fdLocs(D.UI.rewFeed(2))) + deg2rad(D.PAR.feedBnds(1))];
        % set to range [0 2*pi]
        D.UI.rewBnds = wrapTo2Pi(D.UI.rewBnds);
        
        % REWARD RESET BOUNDS
        
        % Calculate crossing points for 90 180 and 270 deg from rew feeders
        D.UI.rewRstLocs = [...
            (fdLocs(D.UI.rewFeed(1))-90:-90:fdLocs(D.UI.rewFeed(1))-270)', ...
            (fdLocs(D.UI.rewFeed(2))-90:-90:fdLocs(D.UI.rewFeed(2))-270)'];
        
        % Calculate 10 deg wide bounds for each reward feeder
        D.UI.rewRstBnds = arrayfun(@(x,y) cat(3, [x, x-10], [y, y-10]), ...
            D.UI.rewRstLocs(:,1), D.UI.rewRstLocs(:,2), 'Uni', false);
        D.UI.rewRstBnds = cell2mat(D.UI.rewRstBnds);
        % set range to [0, 360]
        D.UI.rewRstBnds = wrapTo360(D.UI.rewRstBnds);
        % convert to radians
        D.UI.rewRstBnds = deg2rad(D.UI.rewRstBnds);
        
        % CUE ROTATION BOUNDS
        
        % Copy reward reset bounds
        if D.B.rot_ses
            D.UI.rotBnds = flip(D.UI.rewRstBnds, 1);
        end
        
        % LAP BOUNDS
        
        % Calculate lap count crossing points for every 90 deg
        % shift so first ind in NE
        quadBndLocs = circshift(360-45:-90:45,[0, 1]);
        % shift so that start quadrant is last entry
        strQuadInd = find(D.PAR.catStrQuad == D.PAR.ratStrQuad);
        lapBndLocs = circshift(quadBndLocs,[0,-1*strQuadInd])';
        
        % Calculate 30 deg wide bounds
        D.UI.lapBnds = arrayfun(@(x) [x, x-10], lapBndLocs, 'Uni', false);
        D.UI.lapBnds = cell2mat(D.UI.lapBnds);
        % set range to [0, 360]
        D.UI.lapBnds = wrapTo360(D.UI.lapBnds);
        % convert to radians
        D.UI.lapBnds = deg2rad(D.UI.lapBnds);
        
        % START QUADRANT BOUNDS
        
        % Set start quadrant bound to 90 deg
        D.UI.strQuadBnds = [lapBndLocs(end) - 45, ...
            lapBndLocs(end) + 45];
        % convert to radians
        D.UI.strQuadBnds = deg2rad(D.UI.strQuadBnds);
        
        % CUE BOUNDS
        
        % Calculate crossing points for 90 180 and 270 deg from rew feeders
        cueBndLocs = [...
            (fdLocs(D.UI.rewFeed(1))-65:-10:fdLocs(D.UI.rewFeed(1))-285)', ...
            (fdLocs(D.UI.rewFeed(2))-65:-10:fdLocs(D.UI.rewFeed(2))-285)'];
        
        % Calculate 10 deg wide bounds for each reward feeder
        D.UI.cueBnds = arrayfun(@(x,y) cat(3, [x, x-10], [y, y-10]), ...
            cueBndLocs(:,1), cueBndLocs(:,2), 'Uni', false);
        D.UI.cueBnds = cell2mat(D.UI.cueBnds);
        % set range to [0, 360]
        D.UI.cueBnds = wrapTo360(D.UI.cueBnds);
        % make sure col 1 > col 2
        D.UI.cueBnds(any(diff(D.UI.cueBnds,1,2)>0,3),1,:) = ...
            D.UI.cueBnds(any(diff(D.UI.cueBnds,1,2)>0,3),2,:) + 10;
        % convert to radians
        D.UI.cueBnds = deg2rad(D.UI.cueBnds);
        
        % Set initial cue bounds to start quad
        D.UI.cueBnd = D.UI.strQuadBnds;
        
        %% Plot bounds data
        
        % Plot start quadrant
        [xbnd, ybnd] =  Get_Trk_Bnds(D, D.UI.strQuadBnds);
        D.UI.ptchStQ = ...
            patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
            [ybnd(1,:),fliplr(ybnd(2,:))], ...
            [0.5,0.5,0.5], ...
            'FaceAlpha',0.5, ...
            'Parent',D.UI.axH(1));
        % add star text
        D.UI.txtStQ = ...
            text(mean(xbnd(:,round(size(xbnd,2)/2))), ...
            mean(ybnd(:,round(size(ybnd,2)/2))), ...
            'Start', ...
            'FontSize', 12, ...
            'FontWeight', 'Bold', ...
            'Color', [0.3,0.3,0.3], ...
            'HorizontalAlignment', 'Center');
        
        % Plot 0-deg feeder bounds
        [xbnd, ybnd] =  ...
            Get_Trk_Bnds(D, D.UI.rewBnds(1,:));
        D.UI.ptchFdH(1) = ...
            patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
            [ybnd(1,:),fliplr(ybnd(2,:))], ...
            D.UI.rotCol(1,:), ...
            'FaceAlpha', 0.25, ...
            'Parent', D.UI.axH(1));
        
        % Plot 40-deg feeder bounds
        % set alpha to 0 (transparent)
        [xbnd, ybnd] =  ...
            Get_Trk_Bnds(D, D.UI.rewBnds(2,:));
        D.UI.ptchFdH(2) = ...
            patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
            [ybnd(1,:),fliplr(ybnd(2,:))], ...
            D.UI.rotCol(2,:), ...
            'FaceAlpha', 0, ...
            'EdgeAlpha', 0, ...
            'Parent', D.UI.axH(1));
        
        % Plot reward reset bounds
        % set alpha to 0 (transparent)
        D.UI.ptchFdRstH = gobjects(3,2);
        for z_fd = 1:2
            for z_bnd = 1:3
                [xbnd, ybnd] =  ...
                    Get_Trk_Bnds(D, D.UI.rewRstBnds(z_bnd,:,z_fd));
                D.UI.ptchFdRstH(z_bnd,z_fd) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    [0.5,0.5,0.5], ...
                    'FaceAlpha', 0, ...
                    'EdgeAlpha', 0, ...
                    'Parent', D.UI.axH(1));
            end
        end
        
        % Plot cue trigger bounds
        % set alpha to 0 (transparent)
        D.UI.ptchCueTrg = gobjects(size(D.UI.cueBnds,1),2);
        for z_fd = 1:2
            for z_bnd = 1:size(D.UI.cueBnds,1)
                [xbnd, ybnd] =  ...
                    Get_Trk_Bnds(D, D.UI.cueBnds(z_bnd,:,z_fd));
                D.UI.ptchCueTrg(z_bnd,z_fd) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    'none', ...
                    'FaceAlpha', 0, ...
                    'EdgeAlpha', 0, ...
                    'Parent', D.UI.axH(1));
            end
        end
        
        %% Plot remaining features
        
        % Plot all feeders
        D.UI.fdAllH = plot(D.UI.fd_x, D.UI.fd_y, 'o', ...
            'MarkerFaceColor', [0.5 0.5 0.5], ...
            'MarkerEdgeColor', 'k', ...
            'MarkerSize', 10, ...
            'Parent', D.UI.axH(2));
        
        % Plot cue marker
        % Note: indicates when cue on
        % 0-deg
        D.UI.cueH(1) = ...
            plot(D.UI.fd_x(D.UI.rewFeed(1)), ...
            D.UI.fd_y(D.UI.rewFeed(1)), 's', ...
            'MarkerFaceColor', 'none', ...
            'MarkerEdgeColor', 'none', ...
            'MarkerSize', 25, ...
            'Parent',D.UI.axH(2));
        % 40-deg
        D.UI.cueH(2) = ...
            plot(D.UI.fd_x(D.UI.rewFeed(2)), ...
            D.UI.fd_y(D.UI.rewFeed(2)), 's', ...
            'MarkerFaceColor', 'none', ...
            'MarkerEdgeColor', 'none', ...
            'MarkerSize', 25, ...
            'Parent',D.UI.axH(2));
        
        % Feeder 0-Deg marker
        D.UI.fdH(1) = ...
            plot(D.UI.fd_x(D.UI.rewFeed(1)), ...
            D.UI.fd_y(D.UI.rewFeed(1)), 'o', ...
            'MarkerFaceColor', D.UI.rotCol(1,:), ...
            'MarkerEdgeColor', [0, 0, 0], ...
            'LineWidth', 2, ...
            'MarkerSize', 15, ...
            'Parent',D.UI.axH(2));
        
        % Feeder 40-Deg marker
        D.UI.fdH(2) = ...
            plot(D.UI.fd_x(D.UI.rewFeed(2)), ...
            D.UI.fd_y(D.UI.rewFeed(2)), 'o', ...
            'MarkerFaceColor', D.UI.rotCol(2,:), ...
            'MarkerEdgeColor', [0, 0, 0], ...
            'LineWidth', 1, ...
            'MarkerSize', 10, ...
            'Parent',D.UI.axH(2));
        
        % Plot opposite unrewarded feeders with thicker line
        plot(D.UI.fd_x(D.UI.oppFeed), D.UI.fd_y(D.UI.oppFeed), 'o', ...
            'MarkerFaceColor', [0.5 0.5 0.5], ...
            'MarkerEdgeColor', 'k', ...
            'LineWidth', 2, ...
            'MarkerSize', 10, ...
            'Parent',D.UI.axH(2));
        
        % Plot feeder condition text
        if D.PAR.ratFeedCnd == 'C1'
            x1 = D.UI.fd_x(D.UI.rewFeed(D.I.feed_ind(1))) - 18;
            x2 = D.UI.fd_x(D.UI.rewFeed(D.I.feed_ind(2))) - 18;
        else
            x1 = D.UI.fd_x(D.UI.rewFeed(D.I.feed_ind(1))) + 18;
            x2 = D.UI.fd_x(D.UI.rewFeed(D.I.feed_ind(2))) + 18;
        end
        text(x1, D.UI.fd_y(D.UI.rewFeed(D.I.feed_ind(1))), ...
            [char(D.PAR.ratFeedCnd), ' F1'], ...
            'FontSize', 10, ...
            'FontWeight', 'Bold', ...
            'Color', [0.3,0.3,0.3], ...
            'HorizontalAlignment', 'Center', ...
            'Parent',D.UI.axH(2));
        text(x2, D.UI.fd_y(D.UI.rewFeed(D.I.feed_ind(2))), ...
            [char(D.PAR.ratFeedCnd), ' F2'], ...
            'FontSize', 10, ...
            'FontWeight', 'Bold', ...
            'Color', [0.3,0.3,0.3], ...
            'HorizontalAlignment', 'Center', ...
            'Parent',D.UI.axH(2));
        
        %% Start air and sound
        
        % Turn off vacuum
        % post NLX vac off command
        NlxSendCommand(D.NLX.arduino_stop_cmd);
        % post NLX reset command
        NlxSendCommand(D.NLX.arduino_reset_cmd);
        
        % Run RadSndAir
        % save data to handle
        guidata(D.UI.figH,D);
        % run function
        RadSndAir(D.UI.radSndAir)
        % load out data from handle
        D = guidata(D.UI.figH);
        
        
        %% Post starting feeder to NLX/arduino
        
        % Update setup command
        % update active feeder
        D.NLX.arduino_setup_cmd(D.NLX.arduino_solnd_bit_ind) = ...
            num2str(D.I.feed_ind(D.I.rot) - 1);
        % post NLX update command
        NlxSendCommand(D.NLX.arduino_setup_cmd);
        % post NLX reset command
        NlxSendCommand(D.NLX.arduino_reset_cmd);
        
        
    end

% =========================================================================






%% ======================== ONGOING FUNCTIONS =============================

% ------------------------------PROCESS VT---------------------------------

    function [D] = SF_VT_Proc(D)
        
        %% GET VT DATA
        
        % Pull out vt data for debuggin
        D = DBing(D,2);
        
        % Request all new VT data that has been acquired since the last pass
        if ~D.DB.run
            
            % Get NXL vt data
            [~, ~, vtPos, vtHD, D.vtNRecs, ~] = NlxGetNewVTData(D.NLX.vt_ent);
            
            % Reformat data with samples in column vectors
            D.P.xy_pos = reshape(single(vtPos),2,[])';
            D.P.hd_deg = vtHD';
            
        end
        
        %% TRANSFORM X/Y AND RAD
        
        % Save x/y pos samples in seperate vars
        D.P.x = D.P.xy_pos(:,1);
        D.P.y = D.P.xy_pos(:,2);
        
        % Rescale y as VT data is compressed in y axis
        D.P.y = D.P.y*11/10;
        
        % Get nnormalized pos data
        D.P.x_norm = (D.P.x-D.PAR.XC)./D.PAR.R;
        D.P.y_norm = (D.P.y-D.PAR.YC)./D.PAR.R;
        
        % Get position in radians
        [D.P.rad,D.P.roh] = cart2pol(D.P.x_norm, D.P.y_norm);
        
        % Convert radians between [0, 2*pi]
        D.P.rad = wrapTo2Pi(D.P.rad);
        
        % Flip radian values to acount for inverted y values from Cheetah
        D.P.rad = abs(D.P.rad - 2*pi);
        
        % Exclude outlyer values > || < track bounds plus 5 cm
        excInd = D.P.roh > 1.1 | D.P.roh < (1 - (D.UI.trkWdt+5)/D.UI.arnRad);
        D.P.rad(excInd) = NaN;
        D.P.roh(excInd) = NaN;
        
        % Recalculate cartisian values
        [D.P.x, D.P.y] = pol2cart(D.P.rad,D.P.roh);
        D.P.x =  D.P.x.*D.PAR.R + D.PAR.XC;
        D.P.y =  D.P.y.*D.PAR.R + D.PAR.YC;
        
        %% TRANSFORM HD
        
        % Flip HD angle
        D.P.hd_deg = abs(single(D.P.hd_deg)-360);
        
        % Shift HD by 90 deg to align with track/matlab 0 deg
        D.P.hd_deg = D.P.hd_deg + 90;
        if any(D.P.hd_deg > 360)
            D.P.hd_deg(D.P.hd_deg > 360)  = D.P.hd_deg(D.P.hd_deg > 360) - 360;
        end
        
        % Convert HD to radians
        D.P.hd_rad = deg2rad(D.P.hd_deg);
        
        % Update HD angle samples
        % Note: this mainains a continuous sample of HD samples
        rv_str_ind = D.I.rv_smp_ind + 1;
        rv_end_ind = rv_str_ind + D.vtNRecs-1;
        
        % Keep only max samples
        if D.vtNRecs >= D.A.rv_nsmp
            
            % Get sample indeces
            % start ind
            rv_str_ind = 1;
            % end ind
            rv_end_ind = D.A.rv_nsmp;
            
            % Get radian values
            % track
            D.A.rv_samples(:,1) = D.P.rad(1:D.A.rv_nsmp);
            % hd
            D.A.rv_samples(:,2) = D.P.hd_rad(1:D.A.rv_nsmp);
            
            % Use existing samples as well
        elseif rv_end_ind > D.A.rv_nsmp
            
            % Get shift range
            shft = rv_end_ind - D.A.rv_nsmp;
            
            % Shift samples
            D.A.rv_samples = circshift(D.A.rv_samples, -shft);
            
            % Change indeces
            % start ind
            rv_str_ind = rv_str_ind - shft;
            % end ind
            rv_end_ind = rv_end_ind - shft;
            
        end
        
        % Update samples
        if D.vtNRecs <= D.A.rv_nsmp
            
            % Get radian values
            % track
            D.A.rv_samples(rv_str_ind:rv_end_ind,1) = D.P.rad;
            % hd
            D.A.rv_samples(rv_str_ind:rv_end_ind,2) = D.P.hd_rad;
            
        end
        
        % Update index
        D.I.rv_smp_ind = D.I.rv_smp_ind + D.vtNRecs;
        % cap to max sample range
        if D.I.rv_smp_ind > D.A.rv_nsmp;
            D.I.rv_smp_ind = D.A.rv_nsmp;
        end
        
    end







% -----------------------ROTATION TRIGGER CHECK----------------------------

    function [D] = SF_Rotation_Trig_Check(D)
        
        % Check if rotation trigger button has been pressed
        if get(D.UI.btnICR, 'UserData')
            
            % Check if rat in rotation bounds
            D.B.track_rot = ...
                D.P.rad >= min(D.UI.rotBndNext) & ...
                D.P.rad <= max(D.UI.rotBndNext);
            
            % Trigger rotation if boundary crossed
            if any(D.B.track_rot)
                
                % Set bool to true
                D.B.rotated = true;
                
                % Save current image
                rotLast = D.I.rot;
                
                % Get new image
                D.I.rot = find([1, 2] ~=  D.I.rot);
                
                % Update D.AC.data(2) and send command to rotate image
                D.AC.data(2) = D.I.rot;
                if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
                
                % Post NLX event: rotaion *deg
                NlxSendCommand(D.NLX.rot_cmd{D.I.rot});
                
                % Update NLX/arduino setup command
                D.NLX.arduino_setup_cmd(D.NLX.arduino_solnd_bit_ind) = ...
                    num2str(D.I.feed_ind(D.I.rot) - 1);
                % post NLX update command
                NlxSendCommand(D.NLX.arduino_setup_cmd);
                % post NLX reset command
                NlxSendCommand(D.NLX.arduino_reset_cmd);
                
                % Change plot marker size
                % active feeder
                set(D.UI.fdH(D.I.rot), ...
                    'MarkerSize', 15, ...
                    'LineWidth', 2);
                % inactive feeder
                set(D.UI.fdH([1, 2] ~=  D.I.rot), ...
                    'MarkerSize', 10, ...
                    'MarkerEdgeColor', [0, 0, 0], ...
                    'LineWidth', 1)
                
                % Delete old patches
                if isfield(D.UI, 'ptchRtTrgH')
                    delete(D.UI.ptchRtTrgH)
                end
                
                % Plot rotation pos mark
                r = D.P.rad(D.B.track_rot);
                [x,y] = Get_Trk_Bnds(D,[r(1),r(1)]);
                plot(x,y, ...
                    'Color', D.UI.rotCol(D.I.rot,:), ...
                    'LineWidth', 3, ...
                    'Parent',D.UI.axH(1));
                
                % Change reward bounds patch
                % active feeder
                set(D.UI.ptchFdH(D.I.rot), ...
                    'FaceAlpha', 0.5, ...
                    'EdgeAlpha', 1)
                % inactive feeder
                set(D.UI.ptchFdH([1, 2] ~=  D.I.rot), ...
                    'FaceAlpha', 0, ...
                    'EdgeAlpha',0)
                
                % Change button features
                set(D.UI.btnICR,'string', D.UI.btnICRstr{rotLast}, ...
                    'BackgroundColor', D.UI.rotCol(rotLast,:), ...
                    'UserData', false);
                
                % Change reward reset
                % reset reward bool
                D.B.track_rew_reset = true(3,1);
                % remove reward reset patch
                set(D.UI.ptchFdRstH(:,rotLast), ...
                    'FaceAlpha', 0, ...
                    'EdgeAlpha', 0);
                
                % Set cue patches to transparent
                set(D.UI.ptchCueTrg, ...
                    'FaceColor', 'none', ...
                    'FaceAlpha', 0, ...
                    'EdgeAlpha', 0)
                
                % Change session info font weight
                % active feeder
                set(D.UI.txtPerfInf(D.I.rot), 'FontWeight', 'Bold')
                % inactive feeder
                set(D.UI.txtPerfInf([1, 2] ~=  D.I.rot), 'FontWeight', 'Light')
                
                % Add to roation counter
                D.C.rot_cnt = D.C.rot_cnt + 1;
                
                % Add new lap counter
                D.C.lap_cnt{D.I.rot} = [D.C.lap_cnt{D.I.rot}, 0];
                D.C.rew_cnt{D.I.rot} = [D.C.rew_cnt{D.I.rot}, 0];
                
                
            end
            
        end
        
    end







% -------------------------REWARD FEEDER CHECK-----------------------------

    function [D] = SF_Reward_Feeder_Check(D)
        
        % Check if rat in feeder bounds
        D.B.track_fd = ...
            D.P.rad >= min(D.UI.rewBnds(D.I.rot,:)) & ...
            D.P.rad <= max(D.UI.rewBnds(D.I.rot,:));
        
        if ...
                any(D.B.track_fd) && ...
                D.B.rec_cnt < 30*(D.PAR.rewDel+1)
            
            % Get count of in-bounds samples
            D.C.inbnd_cnt = D.C.inbnd_cnt + sum(D.B.track_fd);
            
            
            % Add to record counter
            D.B.rec_cnt = D.B.rec_cnt + D.vtNRecs;
            
            if ...
                    D.B.rec_cnt >= 30*D.PAR.rewDel && ...
                    D.C.inbnd_cnt >= 30*D.PAR.rewDel && ...
                    all(D.B.track_rew_reset)
                
                % Turn on feeder through NLX ttl comand
                % post NLX command
                NlxSendCommand(D.NLX.arduino_run_rew_cmd);
                % track timing
                D.T.fd_str_tim = clock;
                % post NLX reset command
                NlxSendCommand(D.NLX.arduino_reset_cmd);
                % print solenoid open time
                fprintf('\r\nF%d_OPEN:%s%3.2f_min\n', ...
                    D.I.feed_ind(D.I.rot), ...
                    repmat('_',1,9), ...
                    etime(clock,D.T.ses_str_tim)/60);
                
                % Baloon and set color to red wile feeder is active
                set(D.UI.fdH(D.I.rot), ...
                    'MarkerFaceColor', [1,0,0], ...
                    'MarkerSize', 30);
                
                % Lighten reward feeder patch
                set(D.UI.ptchFdH(D.I.rot), ...
                    'FaceAlpha', 0.25, ...
                    'EdgeAlpha', 1)
                
                % Force update GUI
                drawnow;
                
                % Check if this reward was cued
                if ...
                        (D.UI.cueFeed == 'All' || ...
                        (D.UI.cueFeed == 'Half' && D.B.cue_checked))
                    
                    % Post NLX event: cue off
                    NlxSendCommand(D.NLX.feed_cue_cmd{2});
                    
                    % send AC command to turn off cue
                    D.AC.data(4) = 0;
                    if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
                    
                    % Set cue marker face color to none for both feeders just
                    % in case one is left on after rotation
                    set(D.UI.cueH, ...
                        'MarkerFaceColor', 'none', ...
                        'MarkerEdgeColor', 'none');
                    
                    % Dont cue next lap
                    if D.UI.cueFeed == 'Half'
                        D.B.cue_next = false;
                    end
                end
                
                % Check if next reward is cued
                if ...
                        (D.UI.cueFeed == 'All' || ...
                        (D.UI.cueFeed == 'Half' && ~D.B.cue_checked))
                    
                    % Get new bounds
                    D.I.cue_bnd_ind = ceil(rand(1,1)*size(D.UI.cueBnds,1));
                    D.UI.cueBnd = ...
                        D.UI.cueBnds(D.I.cue_bnd_ind, :, D.I.rot);
                    
                    % Set new bound patch to visible
                    set(D.UI.ptchCueTrg(D.I.cue_bnd_ind, D.I.rot), ...
                        'FaceColor', D.UI.cuTrgCol, ...
                        'FaceAlpha', 0.1, ...
                        'EdgeAlpha', 1)
                    
                    % Cue next lap
                    if D.UI.cueFeed == 'Half'
                        D.B.cue_next = true;
                    end
                end
                
                % Update bool so cue is checked next lap
                D.B.cue_checked = false;
                
                % Set reset patches to visible
                set(D.UI.ptchFdRstH(:, D.I.rot), ...
                    'FaceAlpha', 0.1, ...
                    'EdgeAlpha', 1);
                
                % Add to total reward count
                if D.B.rotated
                    D.C.rew_cnt{D.I.rot}(end) = D.C.rew_cnt{D.I.rot}(end) + 1;
                else
                    D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
                end
                
                % Update bool values
                % reset next reward opportunity
                D.B.track_rew_reset = false(3,1);
                % plot last point where rewarded
                D.B.plot_rew = true;
                
                % Reinitialize counters
                D.C.inbnd_cnt = 0;
                D.B.rec_cnt = 0;
                
                % Return feeder marker color to normal
                set(D.UI.fdH(D.I.rot), ...
                    'MarkerFaceColor', D.UI.rotCol(D.I.rot,:), ...
                    'MarkerSize', 15);
                
            end
        else
            
            % Reinitialize counters
            D.C.inbnd_cnt = 0;
            D.B.rec_cnt = 0;
            
        end
        
    end






% -------------------------RAT HEADING CHECK-------------------------------

    function [D] = SF_Rat_Heading_Check(D)
        
        % Check that there is VT data
        if ...
                ~isempty(D.P.rad) && ...
                ~isempty(D.P.hd_rad)
            
            % Turn on/off aversive sound
            % make sure there is at least 75% of max sample data
            if sum(~isnan(D.A.rv_samples(:,1))) >= D.A.rv_nsmp*0.75
                
                % Calculate mean track
                trk_ang_mu = ...
                    rad2deg(circ_mean(D.A.rv_samples(~isnan(D.A.rv_samples(:,1)),1)));
                trk_ang_mu(trk_ang_mu<0) = 360 + trk_ang_mu(trk_ang_mu<0);
                
                % Calculate mean hd angle
                hed_ang_mu = ...
                    rad2deg(circ_mean(D.A.rv_samples(~isnan(D.A.rv_samples(:,2)),2)));
                hed_ang_mu(hed_ang_mu<0) = 360 + hed_ang_mu(hed_ang_mu<0);
                
                % Check if hd is > 135 (reversed) or < 90 (righted)
                rev_ind = ceil(trk_ang_mu);
                rev_ind(rev_ind>360) = 360; rev_ind(rev_ind<1) = 1;
                % is reversed
                D.B.rat_rev = ...
                    abs(circ_diff(D.A.rv_angmatch(rev_ind),hed_ang_mu,360)) > 135;
                % is righted
                D.B.rat_fwd = ...
                    abs(circ_diff(D.A.rv_angmatch(rev_ind),hed_ang_mu,360)) < 90;
                
                % If reversed or righted post NLX and D.B.rev_reset
                if ...
                        D.B.rat_rev && ...
                        D.B.rev_reset
                    
                    % Post NLX event: reversal start
                    NlxSendCommand(D.NLX.rev_cmd{1});
                    
                    % Add to reversal tracker
                    D.C.rev_cnt = D.C.rev_cnt+1;
                    
                    % Set reversal reset to false
                    D.B.rev_reset = false;
                    
                    % Set pos marker color to red so reversal samples will
                    % be ploted in red
                    D.UI.posCol = [0.5, 0, 0];
                    
                    % Check if rat corrected heading
                elseif D.B.rat_fwd && ~D.B.rev_reset
                    
                    % Post NLX event: reversal end
                    NlxSendCommand(D.NLX.rev_cmd{2});
                    
                    % Set reversal reset to true
                    D.B.rev_reset = true;
                    
                    % change pos marker back to defualt color
                    D.UI.posCol = [0, 0, 0];
                end
                
                % Turn sound on/off
                if D.B.rat_rev
                    
                    % Get curent left channel sound bit value from NLX
                    [~, ttl_str] = NlxSendCommand( ...
                        ['-GetDigitalIOPortString ', D.NLX.DevName, ' ', D.NLX.port_in1]);
                    % keep bit value
                    ttl_val = str2double(ttl_str{:}(8 - D.NLX.snd_lft_bit));
                    
                    % Check if sound should be turned on
                    if ...
                            D.UI.sndair(3) && ...
                            D.B.rat_rev && ...
                            ttl_val ~= 1
                        
                        % Change AC left channel data and post
                        D.AC.data(6) = 2;
                        if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
                        
                        % Pause to let AC computer change sound
                        pause(0.1)
                        
                        % Turn on averse sound
                        % post NLX command
                        NlxSendCommand(D.NLX.arduino_str_avrs_cmd);
                        % track timing
                        D.T.avrs_str_tim = clock;
                        % post NLX reset command
                        NlxSendCommand(D.NLX.arduino_reset_cmd);
                        % print averse on time
                        fprintf('\r\nAverse_ON:%s%3.2f_min\n', ...
                            repmat('_',1,8), ...
                            etime(D.T.avrs_str_tim, D.T.ses_str_tim)/60);
                        
                    end
                    
                    % Check is AC.data(6) needs to be changed  back to
                    % reward sound value
                elseif ...
                        D.B.rat_fwd && ...
                        D.AC.data(6) ~= 1
                    
                    % Set AC.data(6) back to reward sound value
                    D.AC.data(6) = 1;
                    if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
                    
                    % Turn off averse sound
                    % post NLX command
                    NlxSendCommand(D.NLX.arduino_stop_cmd);
                    % post NLX reset command
                    NlxSendCommand(D.NLX.arduino_reset_cmd);
                    % print averse on time
                    fprintf('\r\nAverse_OFF:%s%3.2f_min\n', ...
                        repmat('_',1,8), ...
                        etime(clock, D.T.avrs_str_tim)/60);
                    
                end
            end
        end
        
    end






% ----------------------------LAP CHECK------------------------------------

    function [D] = SF_Lap_Check(D)
        
        % Check if rat in lap quad bound
        track_quad = ...
            D.P.rad >= min(D.UI.lapBnds(D.I.lap_hunt_ind, :)) & ...
            D.P.rad <= max(D.UI.lapBnds(D.I.lap_hunt_ind, :));
        
        % If in quad bounds
        if any(track_quad)
            
            % Set specific bound bool to true
            D.B.track_lap(D.I.lap_hunt_ind) = true;
            
        end
        
        % Set later quads to false to prevent backwards runs
        D.B.track_lap(1:4 > D.I.lap_hunt_ind) = false;
        
        % Update current quadrant
        if any(D.B.track_lap)
            D.I.lap_hunt_ind = find(~D.B.track_lap, 1, 'first');
        else
            D.I.lap_hunt_ind = 1;
        end
        
        % Update lap count
        if all(D.B.track_lap);
            
            % Add lap to counter
            if D.B.rotated
                D.C.lap_cnt{D.I.rot}(end) = D.C.lap_cnt{D.I.rot}(end) + 1;
            else
                D.C.lap_cnt{3}(end) = D.C.lap_cnt{3}(end) + 1;
            end
            
            % Reset lap track bool
            D.B.track_lap = false(1,4);
            
            % Reset lap quad index
            D.I.lap_hunt_ind = 1;
            
            % Set existing pos plot data to gray
            Hind = ~isnan(D.UI.vtPltH(:));
            set(D.UI.vtPltH(Hind), 'MarkerEdgeColor', [0.75,0.75,0.75]);
            set(D.UI.vtPltH(D.UI.vtPltHrev), 'MarkerEdgeColor', [1,0.75,0.75]);
            
            % Update lap time dropdown
            D.UI.lapTim = [D.UI.lapTim; ...
                {sprintf('%d:    %1.1f', sum([D.C.lap_cnt{:}]), etime(clock,D.T.lap_tim))}];
            set(D.UI.popLapTim, 'String', D.UI.lapTim);
            % reset timer
            D.T.lap_tim = clock;
            
        end
        
    end






% ------------------------REWARD RESET CHECK-------------------------------

    function [D] = SF_Reward_Reset_Check(D)
        
        % Check if rat is in any quad
        track_quad = ...
            arrayfun(@(x, y) D.P.rad >= x & D.P.rad <= y, ...
            min(D.UI.rewRstBnds(:, :, D.I.rot), [], 2), ...
            max(D.UI.rewRstBnds(:, :, D.I.rot), [], 2), ...
            'uni', false);
        % Convert to one logical for each quad
        track_quad = ...
            cell2mat(cellfun(@(x) any(x), track_quad, 'uni', false));
        
        % If rat is in any quad bounds
        if any(track_quad)
            
            % Get current occupied quad
            cur_quad = find(track_quad == 1, 1, 'last');
            
            % Set any current occupied quad to true
            D.B.track_rew_reset(cur_quad) = true;
            
            % Set later quads to false to prevent backwards runs
            D.B.track_rew_reset(1:3 > cur_quad) = false;
            
            % Hide patches of passed quadrants
            set(D.UI.ptchFdRstH(D.B.track_rew_reset, D.I.rot), ...
                'FaceAlpha', 0, ...
                'EdgeAlpha', 0);
            
            % Show patches of remaining quadrants
            set(D.UI.ptchFdRstH(~D.B.track_rew_reset, D.I.rot), ...
                'FaceAlpha', 0.1, ...
                'EdgeAlpha', 1);
            
            % If all bounds triggered
            if all(D.B.track_rew_reset)
                
                % Darken reward feeder patch
                set(D.UI.ptchFdH(D.I.rot), ...
                    'FaceAlpha', 0.5, ...
                    'EdgeAlpha', 1)
                
            else
                
                % Lighten reward feeder patch
                set(D.UI.ptchFdH(D.I.rot), ...
                    'FaceAlpha', 0.25, ...
                    'EdgeAlpha', 1)
                
            end
            
        end
        
    end






% ------------------------CUE TRIGGER CHECK--------------------------------

    function [D] = SF_Cue_Trigger_Check(D)
        
        % Check if this is a cued session and not already checked
        if ...
                (D.UI.cueFeed == 'All' || ...
                (D.UI.cueFeed == 'Half' && D.B.cue_next)) && ...
                ~D.B.cue_checked
            
            % Check if cue trigger bounds are crossed
            track_cue = ...
                D.P.rad >= min(D.UI.cueBnd) & ...
                D.P.rad <= max(D.UI.cueBnd);
            
            if any(track_cue);
                
                % Post NLX event: cue on
                NlxSendCommand(D.NLX.feed_cue_cmd{1});
                
                % Update D.AC.data(4) and flicker cue
                D.AC.data(4) = D.PAR.ratFeedCnd_Num;
                if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
                
                % Set cue marker face color to yellow
                set(D.UI.cueH(D.I.rot), ...
                    'MarkerFaceColor',  D.UI.cuTrgCol, ...
                    'MarkerEdgeColor', [0, 0, 0]);
                
                % Set patch to transparent
                set(D.UI.ptchCueTrg(D.I.cue_bnd_ind, D.I.rot), ...
                    'FaceColor', 'none', ...
                    'FaceAlpha', 0, ...
                    'EdgeAlpha', 0)
                
                % plot mark
                r = D.P.rad(track_cue);
                [x,y] = Get_Trk_Bnds(D,[r(1),r(1)]);
                plot(x,y, ...
                    'Color', D.UI.cuTrgCol, ...
                    'LineWidth', 1, ...
                    'Parent',D.UI.axH(1));
                
                % Specify cue has been checked
                D.B.cue_checked = true;
                
            end
            
        end
        
    end






% --------------------------PLOT POSITION----------------------------------

    function [D] = SF_Pos_Plot(D)
        
        if ...
                ~isempty(D.P.x) && ...
                ~isempty(D.P.y)
            
            % Get handle inex to save pos data to to
            Hind = find(isnan(D.UI.vtPltH(:)), 1, 'first');
            
            % Set pos marker color
            % if rat not turned around
            if ~D.B.rat_rev;
                
                % if in feeder bounds use rot condition color
                if any(D.B.track_fd)
                    D.UI.posCol = D.UI.rotCol(D.I.rot,:);
                    % otherwise use default color
                else
                    D.UI.posCol = [0, 0, 0];
                end
            else
                D.UI.vtPltHrev(Hind) = true; % track turn around points
            end
            
            % Plot arrow
            if any(~isnan(D.A.rv_samples(:,2)))
                
                % Delete plot object
                if isfield(D.UI, 'vtPltHD');
                    delete(D.UI.vtPltHD(:));
                end
                
                % Get current heading
                hd_deg = rad2deg(D.A.rv_samples(~isnan(D.A.rv_samples(:,2)),2));
                % use last value
                hd_deg = hd_deg(end);
                
                % Calculate coordinates for line
                % x start/end
                xs = D.P.x(1,end);
                xe = cos(deg2rad(hd_deg))*10 + D.P.x(1,end);
                x = [xs, xe, ...
                    cos(deg2rad(hd_deg-90))*2 + xe, ...
                    cos(deg2rad(hd_deg))*4 + xe, ...
                    cos(deg2rad(hd_deg+90))*2 + xe, ...
                    xe];
                % y start/end
                ys = D.P.y(end);
                ye = sin(deg2rad(hd_deg))*10 + D.P.y(end);
                y = [ys, ye, ...
                    sin(deg2rad(hd_deg-90))*2 + ye, ...
                    sin(deg2rad(hd_deg))*4 + ye, ...
                    sin(deg2rad(hd_deg+90))*2 + ye, ...
                    ye];
                
                % Plot thick backround line
                D.UI.vtPltHD(1) = ...
                    plot(x, y, '-', ...
                    'Color', [0.5, 0.5, 0.5], ...
                    'LineWidth', 3, ...
                    'Parent', D.UI.axH(2));
                
                % Plot thin foreground line
                D.UI.vtPltHD(2) = ...
                    plot(D.UI.axH(2), x, y, '-', ...
                    'Color', D.UI.posCol, ....
                    'LineWidth', 1, ...
                    'Parent', D.UI.axH(2));
            end
            
            % Plot all new VT data
            D.UI.vtPltH(Hind) = ...
                plot(D.P.x, D.P.y, '.', ...
                'MarkerFaceColor', D.UI.posCol, ...
                'MarkerEdgeColor', D.UI.posCol, ...
                'MarkerSize', 6, ...
                'Parent', D.UI.axH(1));
            
            % Plot current position with larger marker
            % delete last pos
            if isfield(D.UI, 'vtPltNow');
                delete(D.UI.vtPltNow);
            end
            % plot current position
            D.UI.vtPltNow = ...
                plot(D.P.x(1,end), D.P.y(end), 'o', ...
                'MarkerFaceColor', D.UI.posCol, ...
                'MarkerEdgeColor', [0, 0, 0], ...
                'MarkerSize', 10, ...
                'Parent', D.UI.axH(2));
            
            % Plot inbound points larger in rot cond colar
            if any(D.B.plot_rew)
                ind = find(D.B.track_fd, 1, 'last');
                D.UI.vtPltH(Hind+1) = ...
                    plot(D.P.x(ind), D.P.y(ind), 'o', ...
                    'MarkerFaceColor', D.UI.posCol, ...
                    'MarkerEdgeColor', [0, 0, 0], ...
                    'MarkerSize', 8, ...
                    'Parent', D.UI.axH(1));
                
                % Get index reward plot handles
                D.UI.vtPltHrew(Hind+1) = true;
                
                % Reset bool
                D.B.plot_rew = false;
                
            end
            
        end
        
    end






% -------------------------PRINT SES INFO----------------------------------

    function [D] = SF_Inf_Print(D)
        
        %% Print performance info
        
        % total
        infstr = sprintf([...
            'Laps______Total:%s%d\n', ...
            'Rewards___Total:%s%d\n', ...
            'Reversals_Total:%s%d\n', ...
            'Rotations_Total:%s%d'], ...
            repmat('_',1,2), sum([D.C.lap_cnt{:}]), ...
            repmat('_',1,2), sum([D.C.rew_cnt{:}]), ...
            repmat('_',1,2), D.C.rev_cnt, ...
            repmat('_',1,2), D.C.rot_cnt);
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
        
        %% Print time info
        
        % Get session time
        nowTim(1) =  etime(clock,D.T.ses_str_tim);
        
        % Get elapsed time plus saved time
        % acquistion
        if D.B.acq
            nowTim(2) = etime(clock, D.T.acq_tim_reset) + D.T.acq_tim;
        else nowTim(2) = D.T.acq_tim; % keep showing save time
        end
        % recording
        if  D.B.rec
            nowTim(3) = etime(clock, D.T.rec_tim_reset)  + D.T.rec_tim;
        else nowTim(3) = D.T.rec_tim; % keep showing save time
        end
        
        % Get lap time
        if D.B.ratIn
            nowTim(4) = etime(clock, D.T.lap_tim);
        else nowTim(4) = 0;
        end
        
        % Make string
        infstr = sprintf([ ...
            '(SES):%s%s\n', ...
            '(ACQ):%s%s\n', ...
            '(REC):%s%s\n'...
            '(LAP):%s%s\n'], ...
            repmat('_',1,6), datestr(nowTim(1)/(24*60*60), 'HH:MM:SS'), ...
            repmat('_',1,6), datestr(nowTim(2)/(24*60*60), 'HH:MM:SS'), ...
            repmat('_',1,6), datestr(nowTim(3)/(24*60*60), 'HH:MM:SS'), ...
            repmat('_',1,6), datestr(nowTim(4)/(24*60*60), 'HH:MM:SS'));
        
        % Print time
        set(D.UI.txtTimElspInf, 'String', infstr)
        
        % Save current time to UserData
        set(D.UI.txtTimElspInf, 'UserData', nowTim)
        
    end

% =========================================================================






%% ======================== CALLBACK FUNCTIONS ============================

% RAT SELECTION
    function [] = PopRat(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        if get(D.UI.popRat,'Value') ~= 1
            
            % Change font
            set(D.UI.popRat, 'FontName','MS Sans Serif','FontSize',8);
            
            % Set rat list to rat and session to selected perf file
            D = DBing(D,3);
            
            % Get rat data
            % rat label
            D.PAR.ratLab = ... % ('r####')
                D.UI.ratList{get(D.UI.popRat,'Value')}(1:5);
            % rat number
            D.PAR.ratNum = ... % (####)
                str2double(D.PAR.ratLab(2:end));
            % index in D.SS_In_All
            D.PAR.ratInd = ...
                find(ismember(D.SS_In_All.Properties.RowNames, D.PAR.ratLab));
            % rat age
            D.PAR.ratAgeGrp = ... % [Young,Old]
                D.SS_In_All.Age_Group(D.PAR.ratInd);
            % rat dob
            D.PAR.ratDOB = ...
                D.SS_In_All.DOB(D.PAR.ratInd);
            % feeder condition
            D.PAR.ratFeedCnd = ... % [C1,C2]
                D.SS_In_All.Feeder_Condition(D.PAR.ratInd);
            % feeder condition
            D.PAR.ratFeedCnd_Num = ... % [1,2]
                find(D.PAR.catFeedCnd == D.PAR.ratFeedCnd);
            % direction of rotation string
            D.PAR.ratRotDrc = ... % [CCW,CW]
                D.SS_In_All.Rotation_Direction(D.PAR.ratInd);
            % direction of rotation number
            D.PAR.ratRotDrc_Num = ... % [1,2]
                find(D.PAR.catRotDrc == D.PAR.ratRotDrc);
            % session number
            if isnan(D.SS_In_All{D.PAR.ratInd, 'Session_ICRb_T'})
                D.PAR.sesNum = 1; % first session entry
            else
                D.PAR.sesNum = ...
                    D.SS_In_All.Session_ICRb_T(D.PAR.ratInd) + 1;
            end
            % icr session
            D.PAR.sesNumICR = ...  % [1:200]
                D.SS_In_All.Session_Rotation(D.PAR.ratInd) + 1;
            % start quadrant
            D.PAR.ratStrQuad = ... % [NE,SE,SW,NW]
                D.SS_In_All.Start_Quadrant{D.PAR.ratInd}(D.PAR.sesNum);
            % rotations per session
            D.PAR.rotPerSes = ... % [2,4,6]
                str2double(char(D.SS_In_All.Rotations_Per_Session{D.PAR.ratInd}(D.PAR.sesNumICR,:)));
            % rotations position
            D.PAR.rotPos = ... % [90,180,270]
                D.SS_In_All.Rotation_Positions{D.PAR.ratInd}(D.PAR.sesNumICR,:)';
            % rewards per session
            D.PAR.rewPerRot = ... % [5:8,6:9,7:10]
                D.SS_In_All.Rewards_Per_Rotation{D.PAR.ratInd}(D.PAR.sesNumICR,:)';
            % days till rotation
            D.PAR.daysTilRot = ... % [5:8,6:9,7:10]
                D.SS_In_All.Days_Till_Rotation{D.PAR.ratInd}(D.PAR.sesNumICR);
            
            % Set Setup pannel UI entries to last sessions
            if D.PAR.sesNum > 1
                
                % Set reward delay
                D.PAR.rewDel = ...
                    D.SS_In_All.Reward_Delay(D.PAR.ratInd);
                set(D.UI.popRwDl, 'Value', ...
                    find(ismember(D.UI.delList, D.PAR.rewDel)));
                % convert to number
                D.PAR.rewDel = str2double(char(D.PAR.rewDel));
                
                % Set cue condition
                D.UI.cueFeed = ...
                    D.SS_In_All.Cue_Condition(D.PAR.ratInd);
                set(D.UI.radCue( ...
                    ismember(D.PAR.catCueCond, D.UI.cueFeed)), 'Value', 1);
                
                % Set sound/air condition
                D.UI.sndair(1:6) = logical(...
                    [D.SS_In_All.Sound_Conditions(D.PAR.ratInd,:), ...
                    D.SS_In_All.Air_Conditions(D.PAR.ratInd,:)]);
                set(D.UI.radSndAir(D.UI.sndair(1:6)), 'Value', 1);
                
            end
            
            % Make rew per ses list
            txt = [{'Rewards Per Rotation'}; ...
                cellfun(@(x,y) sprintf('%d:    %s', x, y), ...
                num2cell(1:length(D.PAR.rotPos))', ...
                cellstr(char(D.PAR.rewPerRot)), 'Uni', false)];
            set(D.UI.popRewPerRot, 'String', txt);
            
            % Make rot pos list
            txt = [{'Rotation Positions'}; ...
                cellfun(@(x,y) sprintf('%d:    %s', x, y), ...
                num2cell(1:length(D.PAR.rotPos))', ...
                cellstr(char(D.PAR.rotPos)), 'Uni', false)];
            set(D.UI.popRotPos, 'String', txt);
            
            % Genertate rotation button string
            D.UI.btnICRstr = ...
                [{sprintf('ROTATE 0%c %s', char(176), char(D.PAR.catRotDrc(D.PAR.catRotDrc ~= D.PAR.ratRotDrc)))}; ...
                {sprintf('ROTATE 40%c %s', char(176), char(D.PAR.ratRotDrc))}];
            
            % Seed random number generator based on session number
            rng(D.PAR.sesNum)
            
            % Reverse start feeder if rotation is CW
            if D.PAR.ratRotDrc == 'CCW';
                D.I.feed_ind = [1, 2];
            elseif D.PAR.ratRotDrc == 'CW';
                D.I.feed_ind = [2, 1];
            end
            
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% ICR CONDITION
    function [] = PopCond(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        if get(D.UI.popCond, 'Value') ~= 1
            
            % Change font
            set(D.UI.popCond, 'FontName','MS Sans Serif','FontSize',8);
            
            % Get condition and change D.B.rot_ses to true if testing
            if get(D.UI.popCond, 'Value') == 3;
                D.B.rot_ses = true;
            end
            
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% REWARD DELAY
    function [] = PopRewDel(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        if get(gcbo,'Value') ~= 1
            
            % Change font
            set(D.UI.popRwDl, 'FontName','MS Sans Serif','FontSize',8);
            
            % Get selected minimum time in bounds for reward
            D.PAR.rewDel = str2double(D.UI.delList{get(gcbo,'Value'),:});
            
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% CUE CONDITION
    function [] = BtnCue(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Get radial button data
        cueStr = get(gcbo,'String');
        
        % Turn off other buttons
        if strcmp(cueStr, 'All')
            % Set to None
            set(D.UI.radCue([2,3]),'Value',0)
            D.UI.cueFeed(:) = 'All';
        elseif strcmp(cueStr, 'Half')
            % Set to Half
            set(D.UI.radCue([1,3]),'Value',0)
            D.UI.cueFeed(:) = 'Half';
        elseif strcmp(cueStr, 'None')
            % Set to All
            set(D.UI.radCue([1,2]),'Value',0)
            D.UI.cueFeed(:) = 'None';
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% SOUND/AIR CONDITION
    function [] = RadSndAir(~, ~, ~)
        
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Note: D.UI.sndair format
        %         {'White Noise'}
        %         {'Reward Tone'}
        %         {'Aversive Sound'}
        %         {'Pump'}
        %         {'Vacuum'}
        %         {'Ozone'}
        
        % Initialize bools
        % change other rad buttons
        updateRad = false;
        % change NLX command string
        updateNLX = false;
        % change AC.data vals
        updateAC = false;
        
        % Update buttons right after setup completed before polling
        if get(D.UI.btnStupDn, 'UserData') == 1  && ~D.B.poll_nlx;
            
            % Loop through radial buttons
            for z_sa = 1:6
                % Get button ID and state value
                D.UI.sndairVal = get(D.UI.radSndAir(z_sa), 'Value');
                D.UI.sndairID = get(D.UI.radSndAir(z_sa), 'UserData');
                if D.UI.sndairVal == 1
                    D.UI.sndair(z_sa) = true;
                else
                    D.UI.sndair(z_sa) = false;
                end
            end
            
            % Change other rad buttons bellow
            updateRad = true;
            
            % Change NLX commands bellow
            updateNLX = true;
            
            % Change AC data bellow
            updateAC = true;
            
            % If already polling
        elseif get(D.UI.btnStupDn, 'UserData') == 1 && D.B.poll_nlx
            
            % Get button ID and state value
            D.UI.sndairVal = get(gcbo, 'Value');
            D.UI.sndairID = get(gcbo, 'UserData');
            
            % Check if value should really be changed
            qststr = sprintf('Are you sure you want to turn %s to %s?', ...
                D.UI.sndairLabs{D.UI.sndairID}, D.UI.sndairState{D.UI.sndairVal+1});
            choice = questdlgAWL(qststr, ...
                'CHANGE SOUND', 'Yes', 'No', [], 'No', D.UI.qstDlfPos);
            drawnow; % force update UI
            
            % Handle response
            switch choice
                case 'No'
                    % Set back to old state and return
                    set(gcbo, 'Value', find([0, 1] ~=  D.UI.sndairVal)-1)
                case 'Yes'
                    % Update vars
                    if D.UI.sndairVal == 1
                        D.UI.sndair(D.UI.sndairID) = true;
                    else
                        D.UI.sndair(D.UI.sndairID) = false;
                    end
                    
                    % Change other rad buttons bellow
                    updateRad = true;
                    
                    % Change NLX commands bellow
                    updateNLX = true;
                    
                    % Change AC data bellow
                    updateAC = true;
            end
        end
        
        % Turn off all sound if white noise is off
        if updateRad
            if D.UI.sndair(1) == false
                D.UI.sndair(2:3) = false;
                set(D.UI.radSndAir(2:3), 'Value', 0);
            end
            % Turn off all air if pump or vac is off
            if ~all(D.UI.sndair(4:5))
                D.UI.sndair(4:6) = false;
                set(D.UI.radSndAir(4:6), 'Value', 0);
            end
        end
        
        % Change NLX/arduino command strings based on sound settings
        if updateNLX
            
            % Loop through D.UI.sndair
            for z_sa = [1,2,4,5,6] % skip averse sound
                % update setup command
                D.NLX.arduino_setup_cmd(D.NLX.arduino_snd_air_bit_ind(z_sa)) = ...
                    num2str(D.UI.sndair(z_sa));
            end
            
            % Post NLX update command
            NlxSendCommand(D.NLX.arduino_setup_cmd);
            % post NLX reset command
            NlxSendCommand(D.NLX.arduino_reset_cmd);
            
        end
        
        % Update AC data
        if updateAC
            % Check if white noise should be turned on/off
            if D.AC.data(5) ~= 1 && ~D.UI.sndair(1) % turn sound off
                D.AC.data(5) = 0; % turn off all sound
            elseif D.AC.data(5) ~= 2 && D.UI.sndair(1) % turn sound on
                D.AC.data(5) = 2;
            end
            if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% SETUP DONE
    function [] = BtnStupDn(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Load vt data for debugging
        D = DBing(D,1);
        
        % Confirm UI entries
        
        % Check if all fields entered with pop-up window
        if get(D.UI.popRat,'Value') == 1 || ...
                get(D.UI.popCond,'Value') == 1 || ...
                get(D.UI.popRwDl,'Value') == 1
            wrnstr = '!!WARNING: A Setup Entry Is Missing!!';
            choice = questdlgAWL(wrnstr, ...
                'MISSING SETUP ENTRIES', 'OK', [], [], 'OK', D.UI.qstDlfPos);
            drawnow; % force update UI
            % Handle response
            switch choice
                case 'OK'
                    return
            end
        end
        
        % Confirm if all fields entered correctly with pop-up window
        qststr = sprintf(['IS THIS CORRECT:\n\n'...
            'Rat_Number:_______%d\n'...
            'ICR_Condition:____%s\n', ...
            'Feeder_Delay:_____%1.1f\n', ...
            'Cue_Conditon:_____%s\n'...
            'Sound_White:______%s\n'...
            'Sound_Reward:_____%s\n'...
            'Sound_Averse:_____%s\n'...
            'Pump:_____________%s\n'...
            'Vacuum:___________%s\n'...
            'Ozone:____________%s\n'], ...
            D.PAR.ratNum, ...
            D.UI.condList{get(D.UI.popCond, 'Value')}, ...
            D.PAR.rewDel, ...
            char(D.UI.cueFeed), ...
            D.UI.sndairState{get(D.UI.radSndAir(1),'Value')+1}, ...
            D.UI.sndairState{get(D.UI.radSndAir(2),'Value')+1}, ...
            D.UI.sndairState{get(D.UI.radSndAir(3),'Value')+1}, ...
            D.UI.sndairState{get(D.UI.radSndAir(4),'Value')+1}, ...
            D.UI.sndairState{get(D.UI.radSndAir(5),'Value')+1}, ...
            D.UI.sndairState{get(D.UI.radSndAir(6),'Value')+1});
        choice = questdlgAWL(qststr, ...
            'CHECK SETTINGS', 'Yes', 'No', [], 'No', D.UI.qstDlfPos);
        drawnow; % force update UI
        % Handle response
        switch choice
            case 'Yes'
            case 'No'
                return
        end
        
        % Change button state to indicate setup done
        set(D.UI.btnStupDn, 'UserData', 1);
        
        % Signal to continue setup
        D.B.setup = true;
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% ACQ BUTTON
    function [] = BtnAcq(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Run only if not recording or not aquiring during recording
        if get(D.UI.btnRec,'Value') ~= 1 || ...
                get(D.UI.btnAcq,'UserData') == 1
            
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
                D.T.acq_tim = etime(clock,D.T.acq_tim_reset) + D.T.acq_tim;
            end
            
            % Change aquiring status
            D.B.acq = ~D.B.acq;
            
            % Reset temp clock
            D.T.acq_tim_reset = clock;
            
            % Set userdata back to 0
            set(D.UI.btnAcq,'UserData', 0)
            
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% REC BUTTON
    function [] = BtnRec(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Start aquisition
        if get(D.UI.btnRec,'Value') == 1
            NlxSendCommand('-StartRecording');
            set(D.UI.btnRec,'CData',D.UI.radBtnCmap{2})
        else NlxSendCommand('-StopRecording');
            set(D.UI.btnRec,'CData',D.UI.radBtnCmap{1})
        end
        
        % Set time tracking variables
        if  D.B.rec
            % save out time before stopping
            D.T.rec_tim = etime(clock, D.T.rec_tim_reset) + D.T.rec_tim;
        end
        D.B.rec = ~ D.B.rec;
        D.T.rec_tim_reset = clock;
        
        % Disable feeder fill buttons
        D.UI.btnCleanFd;
        % fill
        set(D.UI.btnFillFd(1), 'Enable', 'Off')
        set(D.UI.btnFillFd(2), 'Enable', 'Off')
        set(D.UI.btnFillFd, 'BackgroundColor', D.UI.disabledCol)
        % clean
        set(D.UI.btnCleanFd(1), 'Enable', 'Off')
        set(D.UI.btnCleanFd(2), 'Enable', 'Off')
        set(D.UI.btnCleanFd, 'BackgroundColor', D.UI.disabledCol)
        
        % Enable other session buttons
        % clear vt
        set(D.UI.btnClrVT, ...
            'Enable', 'On', ...
            'BackgroundColor', D.UI.enabledCol);
        % icr button
        if D.B.rot_ses
            set(D.UI.btnICR, ...
                'Enable', 'On', ...
                'String', D.UI.btnICRstr{2}, ...
                'BackgroundColor', D.UI.rotCol(2,:));
        end
        % recording done button
        set(D.UI.btnRecDn, ...
            'Enable', 'On', ...
            'BackgroundColor', D.UI.enabledCol)
        
        % Save out handle data
        guidata(D.UI.figH,D)
        
        % Start acq if not running
        if get(D.UI.btnRec,'Value') == 1 && ...
                get(D.UI.btnAcq,'Value') == 0
            
            % Set aquire button value to 1
            set(D.UI.btnAcq,'Value', 1)
            
            % Set aquire button userdata to 1
            % Note: this indicates it still needs to be modified
            set(D.UI.btnAcq,'UserData', 1)
            
            % Run BtnAcq
            % save out handle data
            guidata(D.UI.figH,D)
            % run function
            BtnAcq(D.UI.btnAcq);
            
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% ROTATION BUTTON
    function [] = BtnICR(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % get image and color for post rotation feeder
        rotNext = [1, 2] ~=  D.I.rot;
        
        % Determine rotation trigger bounds for this event
        D.UI.rotBndNext = D.UI.rotBnds(...
            find(D.PAR.catRotPos == D.PAR.rotPos(D.C.rot_cnt+1)), ...
            :, rotNext);
        
        % Plot selected rotation trigger bounds
        [xbnd, ybnd] =  Get_Trk_Bnds(D,D.UI.rotBndNext);
        D.UI.ptchRtTrgH = ...
            patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
            [ybnd(1,:),fliplr(ybnd(2,:))], ...
            D.UI.rotCol(rotNext,:), ...
            'FaceAlpha',0.5, ...
            'Parent', D.UI.axH(1));
        
        % Make button red and set user data to true
        set(D.UI.btnICR,'string', 'Rotation Ready', ...
            'BackgroundColor', [1,0,0], ...
            'UserData', true);
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% RECORDING DONE
    function [] = BtnRecDn(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Check if session is done
        choice = questdlgAWL(...
            'Are you sure you want to end session?', ...
            'SESSION END?', ...
            'Yes', 'No', [], 'No', ...
            D.UI.qstDlfPos);
        
        % Force update UI
        drawnow;
        
        % Handle response
        switch choice
            case 'Yes'
                
                % Post NLX event: session end
                NlxSendCommand(D.NLX.ses_end_cmd);
                
            case 'No'
                
                return
                
        end
        
        % Save end time
        D.T.ses_end_tim = clock;
        
        % Check if rat is out of room
        choice = questdlgAWL(...
            'Is the rat out of the room?', ...
            'RAT OUT?', ...
            'Yes', [], [], 'Yes', ...
            D.UI.qstDlfPos);
        
        % Force update UI
        drawnow;
        
        % Handle response
        switch choice
            case 'Yes'
                
                % Post NLX event: rat out
                NlxSendCommand(D.NLX.rat_in_cmd{2});
                
            case 'No'
                return
        end
        
        % Disable all recording buttons
        set(D.UI.panRec, ...
            'ForegroundColor', D.UI.disabledCol, ...
            'HighlightColor', D.UI.disabledCol)
        set(D.UI.btnAcq, ...
            'Enable', 'Off', ...
            'Visible', 'Off')
        set(D.UI.btnRec, ...
            'Enable', 'Off', ...
            'Visible', 'Off')
        set(D.UI.btnClrVT, ...
            'Enable', 'Off', ...
            'BackgroundColor', D.UI.disabledCol);
        set(D.UI.btnICR, ...
            'Enable', 'Off', ...
            'BackgroundColor', D.UI.disabledCol);
        set(D.UI.btnRecDn, ...
            'Enable', 'Off', ...
            'BackgroundColor', D.UI.disabledCol);
        % Disable radSndAir buttons
        set(D.UI.radSndAir, 'Enable', 'Off')
        
        % Disable inf panels
        set(D.UI.panSesInf, 'HighlightColor', D.UI.disabledCol)
        set(D.UI.panPerfInf, 'HighlightColor', D.UI.disabledCol)
        set(D.UI.panTimInf, 'HighlightColor', D.UI.disabledCol)
        
        % Enable save button
        set(D.UI.btnSav, 'Enable', 'On')
        
        % Enable feeder clean buttons
        set(D.UI.btnCleanFd(1), 'Enable', 'On')
        set(D.UI.btnCleanFd(2), 'Enable', 'On')
        set(D.UI.btnCleanFd(1), ...
            'String', D.UI.btnFdstr{2}, ...
            'BackgroundColor', D.UI.btnFdcol(2,:), ...
            'UserData', [1, 2])
        set(D.UI.btnCleanFd(2), ...
            'String', D.UI.btnFdstr{2}, ...
            'BackgroundColor', D.UI.btnFdcol(2,:), ...
            'UserData', [2, 2])
        
        % End NLX polling
        D.B.poll_nlx = false;
        
        % Stop recording
        NlxSendCommand('-StopRecording');
        % Stop aquisition
        NlxSendCommand('-StopAcquisition');
        
        % Turn off sound on AC computer
        D.AC.data(5) = 0;
        if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
        
        % Print end time
        infstr = sprintf( ...
            'Stop_:%s%s\n', ...
            repmat('_',1,6), datestr(D.T.ses_end_tim, 'HH:MM:SS'));
        set(D.UI.txtTimEndInf, 'String', infstr)
        
        % Make sure all TTL bits but update bit are off
        % feed solenoid bit
        D.NLX.arduino_setup_cmd(D.NLX.arduino_solnd_bit_ind) = '0';
        % air/sound bits
        D.NLX.arduino_setup_cmd(D.NLX.arduino_snd_air_bit_ind) = '0';
        % post NLX update command
        NlxSendCommand(D.NLX.arduino_setup_cmd);
        % stop arduino polling
        NlxSendCommand(D.NLX.arduino_poll_stop_cmd);
        % post NLX reset command
        NlxSendCommand(D.NLX.arduino_reset_cmd);
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% SAVE SESSION DATA
    function [] = BtnSav(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Confirm that Cheetah is closed
        choice = questdlgAWL('Cheetah Closed?', ...
            'CHEETAH CLOSED', ...
            'OK', [], [], 'OK', ...
            D.UI.qstDlfPos);
        drawnow; % force update UI
        % Handle response
        switch choice
            case 'OK'
                pause(5)
        end
        
        % Read/write arduino seriol com data
        % read all arduino serial log data
        serDat = fread(D.ARD.serObj);
        % write to file
        fprintf(D.ARD.logID, '%s', serDat);
        % close arduino connection
        fclose(D.ARD.serObj);
        % close arduino log file
        fclose(D.ARD.logID);
        
        % Change NLX recording directory
        
        % Save directory var
        % formt: datestr(clock, 'yyyy-mm-dd_HH-MM-SS', 'local');
        D.DIR.rec = fullfile(D.DIR.cheetSaveTop, D.PAR.ratLab(2:end));
        
        % Make directory if none exists
        if exist(D.DIR.rec, 'dir') == 0
            mkdir(D.DIR.rec);
        end
        
        % Save GUI window image
        export_fig(D.UI.figH, fullfile(D.DIR.cheetTempTop, D.DIR.recFi, 'GUI.jpg'))
        
        % Save sesion data to tables
        Save_Ses_Dat(D)
        
        % Set bool to true
        D.UI.saved = true;
        
        % Copy Cheetah file to rat directory
        copyfile(fullfile(D.DIR.cheetTempTop, D.DIR.recFi),fullfile(D.DIR.rec, D.DIR.recFi))
        
        % Disable buttons
        % save button
        set(gcbo, 'Enable', 'Off')
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% QUIT ALL
    function [] = BtnQuit(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Check if recording and sesion data have been saved
        if ~D.UI.saved
            % Construct a questdlg with two options
            % Note: based on built in function
            choice = questdlgAWL('QUIT WITHOUT SAVING?', ...
                'QUIT?', ...
                'Yes', 'No', [], 'No', ...
                D.UI.qstDlfPos);
            drawnow; % force update UI
            % Handle response
            switch choice
                case 'Yes'
                case 'No'
                    return
            end
        end
        
        % End NLX polling
        D.B.poll_nlx = false;
        
        % Save out handle data
        guidata(D.UI.figH,D)
        
        % Disable
        % quit button
        set(gcbo, 'Enable', 'Off');
        
        % pause to let Cheetah end recording
        pause(1);
        
        Disconect_All(D)
        
    end

% CLEAR VT BUTTON
    function [] = BtnClrVT(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Delete all tracker data
        delete(D.UI.vtPltH(~isnan(D.UI.vtPltH(1,:))))
        % Reinitialize handle array back to NAN
        D.UI.vtPltH = NaN(1,60*120/D.PAR.polRate); % VT plot handle array
        D.UI.vtPltHrev = false(1,60*120/D.PAR.polRate); % bool for tracking VT plot handles of turn around points
        D.UI.vtPltHrew = false(1,60*120/D.PAR.polRate); % bool for tracking VT plot handles of reward delivery points
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% FEEDER FILL
    function [] = BtnFillFd(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Get selected button UserData
        ud_actv = get(gcbo, 'UserData'); % get selected button user data
        ud_othr = ... % other button user data
            get(D.UI.btnFillFd([1, 2] ~=  ud_actv), 'UserData');
        
        % Set timer UserData to specify running feeder
        set(D.UI.timFdFillDur, 'UserData', ud_actv)
        
        % Check that timer is not running
        if strcmpi(get(D.UI.timFdFillDur, 'Running'), 'Off')
            
            % Disable other button
            set(D.UI.btnFillFd(ud_othr), 'Enable', 'Off')
            set(D.UI.btnFillFd(ud_othr), 'BackgroundColor', D.UI.disabledCol)
            
            % Update setup command to selected feeder
            D.NLX.arduino_setup_cmd(D.NLX.arduino_solnd_bit_ind) = ...
                num2str(ud_actv - 1);
            % post NLX update command
            NlxSendCommand(D.NLX.arduino_setup_cmd);
            % post NLX reset command
            NlxSendCommand(D.NLX.arduino_reset_cmd);
            
            % Turn on feeder through NLX ttl comand
            % post NLX command
            NlxSendCommand(D.NLX.arduino_open_slnd_cmd);
            % track timing
            D.T.fd_str_tim = clock;
            % post NLX reset command
            NlxSendCommand(D.NLX.arduino_reset_cmd);
            % print solenoid open time
            fprintf('\r\nF%d_OPEN:%s%3.2f_min\n', ...
                ud_actv, ...
                repmat('_',1,9), ...
                etime(clock,D.T.ses_str_tim)/60);
            
            % Set selected button properties on
            set(D.UI.btnFillFd(ud_actv), 'String', D.UI.btnFlstr{1}, ...
                'BackgroundColor', D.UI.btnFlcol(1,:));
            drawnow; % force update UI
            
            %Start timer
            start(D.UI.timFdFillDur)
            
        else
            % If timer is running button press is turning feeder off
            % Stop timer
            stop(D.UI.timFdFillDur)
            % Run stop fill function
            TimStopFill();
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% FEEDER PULSE
    function [] = BtnPlsFd(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Get and change handle data
        % get selected button UserData
        ud_actv = get(gcbo, 'UserData');
        
        % Update setup command to selected feeder
        D.NLX.arduino_setup_cmd(D.NLX.arduino_solnd_bit_ind) = ...
            num2str(ud_actv - 1);
        % post NLX update command
        NlxSendCommand(D.NLX.arduino_setup_cmd);
        % post NLX reset command
        NlxSendCommand(D.NLX.arduino_reset_cmd);
        
        % Turn on feeder through NLX ttl comand
        % post NLX command
        NlxSendCommand(D.NLX.arduino_run_rew_cmd);
        % track timing
        D.T.fd_str_tim = clock;
        % post NLX reset command
        NlxSendCommand(D.NLX.arduino_reset_cmd);
        % print solenoid open time
        fprintf('\r\nF%d_PULSED:%s%3.2f_min\n', ...
            ud_actv, ...
            repmat('_',1,9), ...
            etime(clock,D.T.ses_str_tim)/60);
        
        % Revert back to current feeder
        D.NLX.arduino_setup_cmd(D.NLX.arduino_solnd_bit_ind) = ...
            num2str(D.I.feed_ind(D.I.rot) - 1);
        % post NLX update command
        NlxSendCommand(D.NLX.arduino_setup_cmd);
        pause(1)
        % post NLX reset command
        NlxSendCommand(D.NLX.arduino_reset_cmd);
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% FEEDER CLEAN
    function [] = BtnCleanFd(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Get and change handle data
        % get selected button UserData
        ud_actv = get(gcbo, 'UserData');
        % other button user data
        ud_othr = ...
            get(D.UI.btnCleanFd([1, 2] ~=  ud_actv(1)), 'UserData');
        % update user data for selected button
        ud_actv = [ud_actv(1), find([1, 2] ~=  ud_actv(2))];
        
        % Update setup command to selected feeder
        D.NLX.arduino_setup_cmd(D.NLX.arduino_solnd_bit_ind) = ...
            num2str(ud_actv(1) - 1);
        % post NLX update command
        NlxSendCommand(D.NLX.arduino_setup_cmd);
        % post NLX reset command
        NlxSendCommand(D.NLX.arduino_reset_cmd);
        
        % Turn on/off feeder by changing/posting NLX ttl comand
        if ud_actv(2) == 1 % turn on
            
            % Turn on feeder through NLX ttl comand
            % post NLX command
            NlxSendCommand(D.NLX.arduino_open_slnd_cmd);
            % track timing
            D.T.fd_str_tim = clock;
            % post NLX reset command
            NlxSendCommand(D.NLX.arduino_reset_cmd);
            % print solenoid open time
            fprintf('\r\nF%d_OPEN:%s%3.2f_min\n', ...
                ud_actv(1), ...
                repmat('_',1,9), ...
                etime(clock,D.T.ses_str_tim)/60);
            
        else % turn off
            
            % Turn off feeder through NLX ttl comand
            % post NLX command
            NlxSendCommand(D.NLX.arduino_stop_cmd);
            % post NLX reset command
            NlxSendCommand(D.NLX.arduino_reset_cmd);
            % print solenoid close time
            fprintf('F%d_CLOSE:%s%3.2f_sec\n', ...
                ud_actv(1), ...
                repmat('_',1,8), ...
                etime(clock,D.T.fd_str_tim));
            
        end
        
        % Set and save UI and handle data
        % set selected button properties
        set(D.UI.btnCleanFd(ud_actv(1)), ...
            'String', D.UI.btnFdstr{ud_actv(2)}, ...
            'BackgroundColor', D.UI.btnFdcol(ud_actv(2),:), ...
            'UserData', ud_actv);
        
        % Set other button paramiters to off if running
        if ud_othr(2) == 1;
            ud_othr = [ud_othr(1), find([1, 2] ~=  ud_othr(2))];
            % set other button properties
            set(D.UI.btnCleanFd(ud_othr(1)), ...
                'String', D.UI.btnFdstr{2}, ...
                'BackgroundColor', D.UI.btnFdcol(2,:), ...
                'UserData', ud_othr);
        end
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% FEEDER FILL TIMER STOP
    function [] = TimStopFill(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % get selected button UserData
        ud_actv = get(D.UI.timFdFillDur, 'UserData');
        % other button user data
        ud_othr = ...
            get(D.UI.btnFillFd([1, 2] ~=  ud_actv), 'UserData');
        % set button paramiters
        set(D.UI.btnFillFd(ud_actv), ...
            'String', D.UI.btnFlstr{2}, ...
            'BackgroundColor', D.UI.btnFlcol(2,:));
        
        % Turn off feeder through NLX ttl comand
        % post NLX command
        NlxSendCommand(D.NLX.arduino_stop_cmd);
        % post NLX reset command
        NlxSendCommand(D.NLX.arduino_reset_cmd);
        % print solenoid close time
        fprintf('F%d_CLOSE:%s%3.2f_sec\n', ...
            ud_actv, ...
            repmat('_',1,8), ...
            etime(clock,D.T.fd_str_tim));
        
        % Enable other button
        set(D.UI.btnFillFd(ud_othr), 'Enable', 'On');
        set(D.UI.btnFillFd(ud_othr), 'BackgroundColor', D.UI.btnFlcol(2,:));
        drawnow; % force update UI
        
        % Save out handle data
        guidata(D.UI.figH,D)
    end

% BUTTON COLOR
    function [C] = BtnCol(D)
        
        [X,Y] = meshgrid(linspace(-(2*pi),(2*pi),20));
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
        scl = (1 - D.UI.backroundCol(1)) / (1 - Hi(1,1));
        Hi = 1 - scl*(1 - Hi);
        Hi(mask) = D.UI.backroundCol(1);
        Lo = Lo * (D.UI.backroundCol(1)/Lo(1,1));
        scl = (1 - D.UI.backroundCol(1)) / (1 - Lo(1,1));
        Lo = 1 - scl*(1 - Lo);
        Lo(mask) = D.UI.backroundCol(1);
        % remove NaN and set max to 1
        Hi(isnan(Hi) | Hi>1) = 1;
        Lo(isnan(Lo)) = 0;
        Lo(Lo>1) = 1;
        
        % Gray
        C{1}(:,:,1) = Hi;
        C{1}(:,:,2) = Hi;
        C{1}(:,:,3) = Hi;
        
        % Red
        C{2}(:,:,1) = Hi;
        C{2}(:,:,2) = Lo;
        C{2}(:,:,3) = Lo;
        
        C = C';
    end






%% ========================== MINOR FUNCTIONS =============================

% ---------------------------GET TRACK BOUNDS------------------------------
    function [xbnd, ybnd] = Get_Trk_Bnds(D,polbnds)
        
        polbnds = linspace(polbnds(1), polbnds(2), 10);
        % inner bounds 0 deg
        [x(1,:),y(1,:)] = pol2cart(polbnds, ones(1,length(polbnds)) * (D.UI.arnRad-D.UI.trkWdt));
        xbnd(1,:) = x(1,:)*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
        ybnd(1,:) = y(1,:)*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
        % outer bounds 0 deg
        [x(2,:),y(2,:)] = pol2cart(polbnds, ones(1,length(polbnds)) * D.UI.arnRad);
        xbnd(2,:) = x(2,:)*D.UI.xCMcnv + D.UI.lowLeft(1) + D.UI.arnRad*D.UI.xCMcnv;
        ybnd(2,:) = y(2,:)*D.UI.yCMcnv + D.UI.lowLeft(2) + D.UI.arnRad*D.UI.yCMcnv;
    end






% -----------------------------DISCONECT ALL-------------------------------
    function [] = Disconect_All(D)
        
        if isfield(D, 'AC')
            if ~isempty(D.AC)
                
                % Close AC connection
                D.AC.data = zeros(1, length(D.AC.data));
                % keep connection
                D.AC.data(1) = 1;
                % post to AC computer
                if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
                % pause to allow image to close
                pause(1);
                % close all
                D.AC.data = zeros(1, length(D.AC.data));
                % post to AC computer
                if ~D.DB.run; fwrite(D.AC.tcpIP,D.AC.data,'double'); end;
                % close AC computer connection
                fclose(D.AC.tcpIP);
                
                % Show status disconnected
                fprintf('\r\nDisconnected From AC Computer\n  IP: %s\n  Time: %s\r\n', ...
                    D.AC.IP, datestr(now, 'HH:MM:SS'))
            end
        end
        
        if isfield(D, 'NLX')
            if ~isempty(D.NLX)
                
                % Close VT stream
                if isfield(D.NLX, 'vt_ent')
                    NlxCloseStream(D.NLX.vt_ent);
                end
                
                % Close event stream
                if isfield(D.NLX, 'evt_ent')
                    NlxCloseStream(D.NLX.evt_ent);
                end
                
                % Make sure all TTL bits but update bit are off
                % feed solenoid bit
                D.NLX.arduino_setup_cmd(D.NLX.arduino_solnd_bit_ind) = '0';
                % air/sound bits
                D.NLX.arduino_setup_cmd(D.NLX.arduino_snd_air_bit_ind) = '0';
                % post NLX update command
                NlxSendCommand(D.NLX.arduino_setup_cmd);
                % stop arduino polling
                NlxSendCommand(D.NLX.arduino_poll_stop_cmd);
                % post NLX reset command
                NlxSendCommand(D.NLX.arduino_reset_cmd);
                
                % Stop recording
                NlxSendCommand('-StopRecording');
                
                % Stop aquisition
                NlxSendCommand('-StopAcquisition');
                
                % Disconnect from the NLX server
                NlxDisconnectFromServer();
                
                % Show status disconnected
                if NlxAreWeConnected() ~= 1
                    fprintf('\r\nDisconnected From DigitalLynx SX\n  IP: %s\n  Time: %s\r\n', ...
                        D.NLX.IP, datestr(now, 'HH:MM:SS'))
                end
                
            end
            
        end
        
        if isfield(D, 'ARD')
            if ~isempty(D.ARD)
                openFi = fopen('all');
                if ~isempty(openFi)
                    if openFi == D.ARD.logID
                        % Read/write arduino seriol com data
                        % read all arduino serial log data
                        serDat = fread(D.ARD.serObj);
                        % write to file
                        fprintf(D.ARD.logID, '%s', serDat);
                        % close arduino connection
                        fclose(D.ARD.serObj);
                        % close arduino log file
                        fclose(D.ARD.logID);
                    end
                end
            end
        end
        
        if isfield(D, 'UI')
            
            % Delete timers
            delete(D.UI.timFdFillDur);
            
            % Close figure
            close(D.UI.figH)
        end
        
        % Exit MATLAB
        % Get pop-up position
        sc = get(0,'MonitorPositions');
        sc1 = sc(1,:);
        sc2 = sc(2,:);
        qstDlfPos = [sc1(3) + sc2(3)/2, sc1(4)/2];
        choice = questdlgAWL('Do you want to exit MATLAB?', ...
            'EXIT MATLAB', 'Yes', 'No', [], 'Yes', qstDlfPos);
        % Handle response
        switch choice
            case 'Yes'
                exit; % exit MATLAB
            case 'No'
        end
    end






% ---------------------------SAVE SESSION DATA-----------------------------
    function [] =  Save_Ses_Dat(D)
        
        % Get row ind
        % determine if this is first entry
        if D.PAR.sesNum == 1
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
        D.SS_Out_ICR.(D.PAR.ratLab).Total_Time(rowInd) = etime(D.T.ses_end_tim ,D.T.ses_str_tim) / 60;
        D.SS_Out_ICR.(D.PAR.ratLab).ICRb(rowInd) = true;
        D.SS_Out_ICR.(D.PAR.ratLab).ICRi(rowInd) = false;
        D.SS_Out_ICR.(D.PAR.ratLab).Track(rowInd) = true;
        D.SS_Out_ICR.(D.PAR.ratLab).Forage(rowInd) = false;
        D.SS_Out_ICR.(D.PAR.ratLab).Rotation(rowInd) = D.B.rot_ses;
        D.SS_Out_ICR.(D.PAR.ratLab).Session_ICRb_T(rowInd) = D.PAR.sesNum;
        D.SS_Out_ICR.(D.PAR.ratLab).Session_ICRb_F(rowInd) = NaN;
        D.SS_Out_ICR.(D.PAR.ratLab).Session_ICRi_T(rowInd) = NaN;
        D.SS_Out_ICR.(D.PAR.ratLab).Session_ICRi_F(rowInd) = NaN;
        D.SS_Out_ICR.(D.PAR.ratLab).Feeder_Condition(rowInd) = D.PAR.ratFeedCnd;
        D.SS_Out_ICR.(D.PAR.ratLab).Rotation_Direction(rowInd) = D.PAR.ratRotDrc;
        D.SS_Out_ICR.(D.PAR.ratLab).Reward_Delay(rowInd) = sprintf('%1.1f', D.PAR.rewDel);
        D.SS_Out_ICR.(D.PAR.ratLab).Cue_Condition(rowInd) = char(D.UI.cueFeed);
        D.SS_Out_ICR.(D.PAR.ratLab).Pulse_Duration(rowInd) = D.PAR.fdPlsDur;
        D.SS_Out_ICR.(D.PAR.ratLab).Sound_Conditions(rowInd,:) = D.UI.sndair(1:3);
        D.SS_Out_ICR.(D.PAR.ratLab).Air_Conditions(rowInd,:) = D.UI.sndair(4:6);
        D.SS_Out_ICR.(D.PAR.ratLab).Start_Quadrant(rowInd) = D.PAR.ratStrQuad;
        D.SS_Out_ICR.(D.PAR.ratLab).Reversals(rowInd) = D.C.rev_cnt;
        D.SS_Out_ICR.(D.PAR.ratLab).Rewards_Standard{rowInd} = sum(D.C.rew_cnt{3});
        D.SS_Out_ICR.(D.PAR.ratLab).Laps_Standard{rowInd} = sum(D.C.lap_cnt{3});
        D.SS_Out_ICR.(D.PAR.ratLab).Notes{rowInd} = '';
        
        % rotation session specifir vars
        if D.B.rot_ses
            D.SS_Out_ICR.(D.PAR.ratLab).Session_Rotation(rowInd) = D.PAR.sesNumICR;
            D.SS_Out_ICR.(D.PAR.ratLab).Rotations_Per_Session(rowInd) = D.C.rot_cnt;
            D.SS_Out_ICR.(D.PAR.ratLab).Rotation_Positions{rowInd}(1:D.C.rot_cnt) = ...
                D.PAR.rotPos(1:D.C.rot_cnt);
            D.SS_Out_ICR.(D.PAR.ratLab).Rewards_Per_Rotation{rowInd} = ...
                [D.C.rew_cnt{3}, reshape([[D.C.rew_cnt{1}]; [D.C.rew_cnt{2}]], 1, [])];
            D.SS_Out_ICR.(D.PAR.ratLab).Days_Till_Rotation(rowInd) = D.PAR.daysTilRot;
            D.SS_Out_ICR.(D.PAR.ratLab).Rewards_40_Deg{rowInd} = D.C.rew_cnt{2};
            D.SS_Out_ICR.(D.PAR.ratLab).Laps_40_Deg{rowInd} = D.C.lap_cnt{2};
            D.SS_Out_ICR.(D.PAR.ratLab).Rewards_0_Deg{rowInd} = D.C.rew_cnt{1};
            D.SS_Out_ICR.(D.PAR.ratLab).Laps_0_Deg{rowInd} = D.C.lap_cnt{1};
        else
            D.SS_Out_ICR.(D.PAR.ratLab).Session_Rotation(rowInd) = NaN;
            D.SS_Out_ICR.(D.PAR.ratLab).Rotations_Per_Session(rowInd) = NaN;
            D.SS_Out_ICR.(D.PAR.ratLab).Rotation_Positions{rowInd} = ...
                categorical({'<undefined>'}, {'90', '180', '270'});
            D.SS_Out_ICR.(D.PAR.ratLab).Rewards_Per_Rotation(rowInd) = {[]};
            D.SS_Out_ICR.(D.PAR.ratLab).Days_Till_Rotation(rowInd) = '<undefined>';
            D.SS_Out_ICR.(D.PAR.ratLab).Rewards_40_Deg{rowInd} = 0;
            D.SS_Out_ICR.(D.PAR.ratLab).Rewards_0_Deg{rowInd} = 0;
            D.SS_Out_ICR.(D.PAR.ratLab).Laps_40_Deg{rowInd} = 0;
            D.SS_Out_ICR.(D.PAR.ratLab).Laps_0_Deg{rowInd} = 0;
        end
        
        % Update SS_In_All
        D.SS_In_All.Session_ICRb_T(D.PAR.ratInd) = D.PAR.sesNum;
        if D.B.rot_ses
            D.SS_In_All.Session_Rotation(D.PAR.ratInd) = D.PAR.sesNumICR;
        end
        D.SS_In_All.Reward_Delay(D.PAR.ratInd) = sprintf('%1.1f', D.PAR.rewDel);
        D.SS_In_All.Cue_Condition(D.PAR.ratInd) = char(D.UI.cueFeed);
        D.SS_In_All.Pulse_Duration(D.PAR.ratInd) = D.PAR.fdPlsDur;
        D.SS_In_All.Sound_Conditions(D.PAR.ratInd,:) = D.UI.sndair(1:3);
        D.SS_In_All.Air_Conditions(D.PAR.ratInd,:) = D.UI.sndair(4:6);
        
        % Save out data
        SS_Out_ICR = D.SS_Out_ICR;
        save(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_Out_ICR.mat'), 'SS_Out_ICR');
        SS_In_All = D.SS_In_All;
        save(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_In_All.mat'), 'SS_In_All');
        
    end






% ---------------------------------DEBUG-----------------------------------
    function [D] =  DBing(D,arg)
        if D.DB.run
            
            % Load vt data for debugging
            if arg == 1
                
                % Track start pol time
                D.DB.startPolTim = clock;
                
                % Load vt data from file
                [D.DB.VT.TS, D.DB.VT.X, D.DB.VT.Y, D.DB.VT.HD, ~, ~, ~] = Nlx2MatVT( ...
                    fullfile(D.DB.sesDir, D.DB.rat, D.DB.sesFi, 'VT1.nvt'), ...
                    [1 1 1 1 1 1], 1, 1, [] );
                
                % Initialize sample point index
                D.DB.vtInd = 1;
                
                % Save out handle data
                guidata(D.UI.figH,D)
                
            end
            
            % Pull out vt data for debuggin
            if arg == 2
                
                % Track end poll time
                D.DB.endPolTim = clock;
                
                % If set to step pause here
                if D.DB.step
                    pause
                end
                
                % Get vt data for given pass
                D.vtNRecs = round(etime(D.DB.endPolTim, D.DB.startPolTim)*30);
                D.P.xy_pos = NaN(ceil(D.vtNRecs/D.DB.subSmp),2);
                D.P.xy_pos(:,1) = D.DB.VT.X(D.DB.vtInd:D.DB.subSmp:D.DB.vtInd+D.vtNRecs-1)';
                D.P.xy_pos(:,2) = D.DB.VT.Y(D.DB.vtInd:D.DB.subSmp:D.DB.vtInd+D.vtNRecs-1)';
                D.P.hd_deg = NaN(D.vtNRecs,1);
                D.P.hd_deg = D.DB.VT.HD(D.DB.vtInd:D.DB.subSmp:D.DB.vtInd+D.vtNRecs-1)';
                
                % Reset start time
                D.DB.startPolTim = clock;
                
                % Update vt sample ind
                D.DB.vtInd = D.DB.vtInd+D.vtNRecs;
                
                % Scale record number by sub sample
                D.vtNRecs = ceil(D.vtNRecs/D.DB.subSmp);
            end
            
            % Get and set session variables
            if arg == 3
                
                % Set rat pop list to specified rat
                list = get(D.UI.popRat,'String');
                listInd = cellfun(@(x) x(1:5), list, 'Uni', false);
                listInd = find(ismember(listInd, ['r',D.DB.rat]));
                set(D.UI.popRat,'Value',listInd);
                % rat label
                D.PAR.ratLab = ... % ('r####')
                    D.UI.ratList{get(D.UI.popRat,'Value')}(1:5);
                % index in D.SS_In_All
                D.PAR.ratInd = ...
                    find(ismember(D.SS_In_All.Properties.RowNames, D.PAR.ratLab));
                
                % Get index for specified ses
                oldSesInd = find(ismember(D.SS_Out_ICR.(['r',D.DB.rat]).Date, D.DB.sesFi));
                
                % Get index of new ses=
                D.PAR.sesNum = ...
                    D.SS_In_All.Session_ICRb_T(D.PAR.ratInd) + 1;
                
                % Set SS_In_All settings to past ses
                % feeder condition
                D.SS_In_All.Feeder_Condition(D.PAR.ratInd) = ...
                    D.SS_Out_ICR.(['r',D.DB.rat]).Feeder_Condition(oldSesInd);
                % rotation direction
                D.SS_In_All.Rotation_Direction(D.PAR.ratInd) = ...
                    D.SS_Out_ICR.(['r',D.DB.rat]).Rotation_Direction(oldSesInd);
                % rotation session
                D.SS_In_All.Session_Rotation(D.PAR.ratInd) = ...
                    max(D.SS_Out_ICR.(['r',D.DB.rat]).Session_Rotation(1:oldSesInd));
                if isnan(D.SS_In_All.Session_Rotation(D.PAR.ratInd))
                    D.SS_In_All.Session_Rotation(D.PAR.ratInd) = 1;
                end
                % start quadrant
                D.SS_In_All.Start_Quadrant{D.PAR.ratInd}(D.PAR.sesNum) = ...
                    D.SS_Out_ICR.(['r',D.DB.rat]).Start_Quadrant(oldSesInd);
            end
            
            % Get and set session variables
            if arg == 4
                dbstep
            end
        end
    end

% =========================================================================

end