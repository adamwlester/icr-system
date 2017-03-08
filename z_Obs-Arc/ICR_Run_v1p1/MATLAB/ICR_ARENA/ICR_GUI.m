function[] = ICR_GUI(doDebug)

%% ============================= TOP LEVEL =================================

% Declare global vars
global FigH; % UI figure handle
global D; % Main data struct
global shouldExit; % bool to exit
global caughtError; % bool for error handling
global enableSave; % enable save button
global m2c_id; % message out to CS
global m2c_dat1; % data out to CS
global m2c_dat2; % data out to CS
global m2c_flag; % new data flag out to CS
global consoleText; % console text
global tcpIP; % server tcpip object

% Set globals
shouldExit = false;
caughtError = false;
enableSave = false;
consoleText = ' ';

% ---------------------------SET MAIN PARAMETERS---------------------------

% Time
D.PAR.fdPlsDur = 1500; % open time (ms)
D.PAR.polRate = 0.1; % poll fs (sec)
D.PAR.strQdDel = 0.5; % min time in start quad (sec)

% Directories

% IO dirs
D.DIR.top = pwd;
D.DIR.ioTop = fullfile(D.DIR.top,'IOfiles');
D.DIR.ioWallImage = fullfile(D.DIR.ioTop,'Images\plot_images\wall_top_down.png');
D.DIR.ioSS_In_All = fullfile(D.DIR.ioTop, 'SessionData', 'SS_In_All.mat');
D.DIR.ioSS_Out_ICR = fullfile(D.DIR.ioTop, 'SessionData', 'SS_Out_ICR.mat');
D.DIR.ioTrkBnds = fullfile(D.DIR.ioTop, 'Operational', 'track_bounds(new_cam).mat');

% Cheetah dirs
D.DIR.nlxTempTop = 'C:\CheetahData\Temp';
D.DIR.nlxSaveTop = 'E:\BehaviorPilot';

%---------------------Important variable formats---------------------------
%...........................D.UI.snd....................................
%   val 1 = White Noise [true, false]
%   val 2 = Reward Tone [true, false]
%...........................D.AC.data......................................
%   val 1 = conection [0, 1], [no, yes]
%   val 2 = display image [0, 1, 2], [Close all, 0-deg, 40-deg]
%   val 3 = rotation direction [-1, 1], [ACW, CW]
%   val 4 = sound state [0, 1], [no sound, sound]
%...........................m2c_id...................................
%    'S', // start session
%    'Q', // quit session
%    'M', // move to position
%    'R', // dispense reward
%    'H', // halt movement
%    'B', // bulldoze rat
%    'I', // start/end pid
%    'N', // matlab not loaded
%    'L', // matlab loaded
%    'A', // connected to AC computer
%    'Z', // data saved
%--------------------------------------------------------------------------

% ---------------------------RUN MAIN SCRIPT-------------------------------

% Determine if code should be run in debug mode
if nargin == 0
    doDebug = true;
end

% Initilize top level vars
FigH = figure('Visible', 'Off', ...
    'DeleteFcn', {@ExitCallback});

% RUN MAIN FUNCTION
% Print start
Update_Console(sprintf('\rSTART ICR_GUI.m\rTime: %s\r', ...
    datestr(now, 'HH:MM:SS')));

% Run with
if doDebug
    % Stop on error
    dbstop if error;
    Update_Console(sprintf('\rRUNNING IN DEBUG MODE\r'));
    RunScript();
else
    % Catch and print error in console
    dbclear if caught error;
    Update_Console(sprintf('\rRUNNING IN NORMAL MODE\r'));
    try
        RunScript();
    catch ME
        caughtError = true;
        err = sprintf(['\r!!!!!!!!!!!!!ERROR!!!!!!!!!!!!!\r\r'...
            'ID: %s\r\r'...
            'Msg: %s\r\r'...
            'Line: %d\r\r'...
            'Time: %s\r\r' ...
            '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r'], ...
            ME.identifier, ...
            ME.message, ...
            ME.stack(1).line, ...
            datestr(now, 'HH:MM:SS'));
        Update_Console(err);
        % Wait if error occured before exiting
        if ~shouldExit && exist('D', 'var')
            wait = true;
            while wait;
                drawnow
                if D.B.do_quit
                    pause(5);
                    wait = false;
                end
                if exist('FigH', 'var')
                    if ishghandle(FigH)
                        if strcmp(FigH.Visible, 'off')
                            wait = false;
                        end
                    else
                        wait = false;
                    end
                else
                    wait = false;
                end
            end
        end
    end
end

% Start exiting
Update_Console(sprintf('\rStarting exiting...\rTime: %s\r', ...
    datestr(now, 'HH:MM:SS')));

% Pause then shut it all down
pause(1);
Disconnect_All();

% Finish exiting
Update_Console(sprintf('\rFinished exiting\rTime: %s\r', ...
    datestr(now, 'HH:MM:SS')));

% CLEAR EVERYTHING

% Close figure
if exist('FigH','var')
    if ishghandle(FigH)
        close(FigH)
    end
end

% Clear all variables
clearvars -global;
clearvars;
close all;

% For added measure
Vars=whos;
PersistentVarNames={Vars.name};
clear(PersistentVarNames{:});





%% ==========================MAIN FUNCTION==================================

    function[] = RunScript()
        
        % RUN MAIN LOOP SCRIPT
        MainLoop();
        
        %% =========================== MAIN LOOP ================================
        
        function[] = MainLoop()
            
            
            % ---------------------------SETUP & RUN----------------------------------
            
            % Run variable setup code
            SF_Var_Setup();
            
            % Run UI setup code
            SF_UI_Setup();
            
            % Run AC setup code
            SF_AC_Setup();
            
            % Tell CS Matlab is connected to AC
            Mat2CS('A');
            
            % Run NLX setup code
            SF_NLX_Setup();
            
            % Tell CS Matlab is connected to Cheetah
            Mat2CS('L');
            
            while ~shouldExit
                
                % -----------------------CHECK FOR UI SETUP---------------------------
                
                if ~D.B.setup
                    pause(0.01);
                    
                elseif ~D.B.poll_nlx && ~D.B.rec_done
                    
                    % Run Finish setup code
                    SF_Finish_Setup();
                    
                    % Dump initial 1 sec of vt data
                    dump_str = clock;
                    while etime(clock, dump_str) < 1
                        NlxGetNewVTData(D.NLX.vt_rat_ent);
                        NlxGetNewVTData(D.NLX.vt_rob_ent);
                    end
                    % Set initial poll time
                    D.T.last_poll_tim = clock;
                    
                    % ---------------------------POLL NETCOM------------------------------
                    
                elseif D.B.poll_nlx
                    
                    % GET ELLAPSED TIME
                    D.T.loop = etime(clock, D.T.last_poll_tim);
                    
                    % PROCESS NLX EVENTS
                    SF_Evt_Proc();
                    
                    % HOLD FOR NETCOM BUFFERS
                    if (D.T.loop >= D.PAR.polRate)
                        D.T.last_poll_tim = clock;
                        
                        % WAIT TO SEND FIRST MOVE COMMAND
                        if etime(clock, D.C.moveDel) > 2.5 &&  ~D.C.doneMove
                            
                            % Send C# command to move robot to start quad or reward loc
                            if D.PAR.sesCond ~= 'Manual_Training' %#ok<*STCMP>
                                Mat2CS('M', D.UI.strQuadBnds(1));
                            else
                                Mat2CS('M', D.UI.rewFeedRad(1));
                            end
                            D.C.doneMove = true;
                            
                        end
                        
                        % PROCESS NLX VT
                        
                        % Run VT processing code
                        % Robot
                        SF_VT_Proc('Rob');
                        % Rat
                        SF_VT_Proc('Rat');
                        
                        % CHECK IF RAT IN
                        
                        % Run rat in check code
                        if ...
                                ~D.B.rat_in && ...
                                D.B.rec && ...
                                ~D.B.rec_done
                            
                            SF_Rat_In_Check();
                        end
                        
                        % -----------------------ONCE RAT IN-----------------------------------
                        
                        if D.B.rat_in
                            
                            % ROTATION TRIGGER CHECK
                            
                            % Run rotation check code
                            SF_Rotation_Trig_Check();
                            
                            % REWARD FEEDER CHECK
                            
                            % Run reward feeder check code
                            SF_Reward_Feeder_Check();
                            
                            % LAP CHECK
                            
                            % Run lap check code
                            SF_Lap_Check();
                            
                            % REWARD RESET CHECK
                            
                            % Run reward reset check code
                            SF_Reward_Reset_Check();
                            
                        end
                        
                        % PLOT POSITION
                        
                        % Run plot position code
                        SF_Pos_Plot();
                        
                        % PRINT SES INFO
                        
                        % Run info print code
                        SF_Inf_Print();
                        
                    end
                    
                    % Check if CS has enabled save
                    if enableSave
                        % Enable save button
                        set(D.UI.btnSave, ...
                            'Enable', 'on', ...
                            'ForegroundColor' , D.UI.dfltTxtLightCol, ...
                            'BackgroundColor', D.UI.dfltActiveCol);
                        enableSave = false;
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
        
        function [] = SF_Var_Setup()
            
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
            % rotation direction
            D.PAR.catRotDrc = categories(D.SS_In_All.Rotation_Direction); % [CCW,CW];
            % cue condition
            D.PAR.catCueCond = categories(D.SS_In_All.Cue_Condition); % [All, Half, None]
            % start qauadrant
            D.PAR.catStrQuad = categories(D.SS_In_All.Start_Quadrant{1}); % [NE,SE,SW,NW];
            % rotation position
            D.PAR.catRotPos = categories(D.SS_In_All.Rotation_Positions{1}); % [90,180,270];
            
            % COUNTERS
            
            % track icr events
            D.C.rot_cnt = 0;
            % lap by rotation
            D.C.lap_cnt = num2cell(zeros(1,3));
            % rew by rotation
            D.C.rew_cnt = num2cell(zeros(1,3));
            % missed rewards
            D.C.missed_rew_cnt = 0;
            % bulldozing event count
            D.C.bull_cnt = 0;
            
            % BOOLEANS
            
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
            % recording done
            D.B.rec_done = false;
            % track if rat is in arena
            D.B.rat_in = false;
            % flag to do save
            D.B.do_save = false;
            % flag to quit
            D.B.do_quit = false;
            % track if reward in progress
            D.B.is_rewarding = false;
            % block any cueing
            D.B.do_block_cue = false;
            % cue every lap
            D.B.do_all_cue = false;
            % track if next lap should be cued for half cue cond
            D.B.is_cued_rew = false;
            % track if cue sent to cs
            D.B.flag_cue_sent = false;
            % track reward reset
            D.B.flag_rew_reset = false;
            % track reset crossing
            D.B.flag_reset_crossed = false;
            % track reward crossing
            D.B.flag_rew_crossed = false;
            % track lap bounds
            D.B.check_inbound_lap = false(4,1);
            % store rew inbound
            D.B.check_inbound_rew = false;
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
            D.T.last_poll_tim = clock;
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
            % track feeder event timing
            D.T.fd_str_tim = 0;
            % track feeder off event timing
            D.T.fd_end_tim = 0;
            % start quad tim
            D.T.strqd_inbnd_t1 = 0;
            D.T.strqd_inbnd_t2 = 0;
            % rew bound tim
            D.T.fd_inbnd_t1 = 0;
            D.T.fd_inbnd_t2 = 0;
            % track last pos update
            D.T.Rat.last_pos_update = clock;
            D.T.Rob.last_pos_update = clock;
            
            % INDEXING
            
            % current wall image index
            D.I.rot = 1;
            % feeder index
            D.I.feed_ind = [1, 2];
            % current lap quadrant for lap track
            D.I.lap_hunt_ind = 1;
            
            % POSITION DATA
            
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
            
            % Robot guard dist
            D.PAR.guardDist = 4.5 * ((2 * pi)/(140 * pi));
            % PID setPoint
            D.PAR.setPoint = 48 * ((2 * pi)/(140 * pi));
            % Robot butt dist
            D.PAR.buttDist = 18 * ((2 * pi)/(140 * pi));
            % Feeder dist from rob tracker
            D.PAR.feedDist = 66 * ((2 * pi)/(140 * pi));
            % Cue dist
            D.PAR.cueDist = 30 * ((2 * pi)/(140 * pi));
            % pos now
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
            % store pos x,y
            D.P.Rat.pos_hist = NaN(60*60*33,2);
            % store vel by roh
            D.P.Rat.vel_hist = NaN(60*60*33,2);
            D.P.Rob.vel_hist = NaN(60*60*33,2);
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
            % primary backround color
            D.UI.backroundCol = [0.9, 0.9, 0.9];
            % primary backround color
            D.UI.enabledCol = [0.3, 0.3, 0.3];
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
            % default button activated color
            D.UI.dfltActiveCol = [0.75, 0, 0];
            % defualt disabled object color
            D.UI.enabledCol = [0.5, 0.5, 0.5];
            % defualt disabled object color
            D.UI.disabledCol = [0.75 0.75 0.75];
            
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
                (D.SS_In_All.Session_Condition == 'Manual_Training' | ...
                D.SS_In_All.Session_Condition == 'Behavior_Training' );
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
            
        end
        
        % -------------------------------UI SETUP-----------------------------------
        
        function[] = SF_UI_Setup()
            
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
                'Color', 'white', ...
                'HorizontalAlignment','center', ...
                'VerticalAlignment','middle')
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
            
            % GUI feature pos
            
            % Questions dialogue pos
            D.UI.qstDlfPos = [D.UI.sc1(3) + D.UI.sc2(3)/2, D.UI.sc2(4)/2];
            
            % Bounds of plot space
            plot_bnds = [ ...
                ax_pos(1) - 0.05, ...
                ax_pos(2) - 0.05, ...
                ax_pos(3) + 0.1, ...
                ax_pos(4) + 0.1];
            
            % Panel positions
            % setup
            D.UI.stup_pan_pos = ...
                [0,  ...
                1 - 0.46, ...
                plot_bnds(1), ...
                0.46];
            % recording
            D.UI.rec_pan_pos = ...
                [0, ...
                D.UI.stup_pan_pos(2) - 0.01 - 0.275, ...
                plot_bnds(1), ...
                0.275];
            % velocity
            vel_plt_ht = 1 - ...
                D.UI.stup_pan_pos(4) - 0.01 - ...
                D.UI.rec_pan_pos(4) - 0.01 - ...
                0.05 - 0.01;
            D.UI.vel_pan_pos = ...
                [0, ...
                0.05 + 0.01, ...
                plot_bnds(1), ...
                vel_plt_ht];
            
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
            img = imread(D.DIR.ioWallImage, 'BackgroundColor', [1 1 1]);
            set(D.UI.axH(3), 'XLim', [0,size(img,2)], 'YLim', [0,size(img,1)]);
            img = flip(img, 1);
            % 0 deg
            img0 = img;
            % 40 deg
            mask = true(size(img));
            img = imrotate(img, -40, 'crop');
            maskR = ~imrotate(mask, -40, 'crop');
            img(maskR) = 255;
            img40 = img;
            
            % Store for later
            D.UI.wallImgH(1) = image(img0, 'Parent', D.UI.axH(3), 'Visible', 'off');
            D.UI.wallImgH(2) = image(img40, 'Parent', D.UI.axH(3), 'Visible', 'off');
            
            %% ========================= ADD UI OBJECTS ===============================
            
            %% ---------------------------SETUP PANEL----------------------------------
            
            % Position settings
            obj_gap_ht = 0.025;
            pos_ht_dflt = (1 - 2*obj_gap_ht) / 8;
            pos_lft_dflt = 0.05;
            pos_wd_dflt = 1-pos_lft_dflt*2;
            pxl2normStup_y = D.UI.pxl2norm_y/D.UI.stup_pan_pos(4);
            
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
                'BackgroundColor', D.UI.backroundCol, ...
                'ForegroundColor', D.UI.disabledCol, ...
                'HighlightColor',D.UI.disabledCol, ...
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
                'Enable', 'off', ...
                'Callback', {@PopRat}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',[1 1 1], ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
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
                'String','Condition', ...
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
                'Enable', 'off', ...
                'Callback', {@PopCond}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',[1 1 1], ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',text1_font_sz(1), ...
                'FontWeight','Bold', ...
                'String',D.UI.condList, ...
                'Value',1);
            
            % ICR TASK
            % text
            botm = pos_ht_dflt*6 + obj_gap_ht - head_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)];
            D.UI.txtTask = uicontrol('Style','text', ...
                'Parent',D.UI.panStup, ...
                'String','Task', ...
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
            D.UI.popTask = uicontrol('Style','popupmenu', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback', {@PopTask}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',[1 1 1], ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',text1_font_sz(1), ...
                'FontWeight','Bold', ...
                'String',D.UI.taskList, ...
                'Value',1);
            
            % REWARD DELAY
            % text
            botm = pos_ht_dflt*5 + obj_gap_ht - head_font_sz(2);
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
            botm = pos_ht_dflt*4 + obj_gap_ht;
            pos = [pos_lft_dflt, botm, pos_wd_dflt, head_font_sz(2)*1.9];
            D.UI.popRwDl = uicontrol('Style','popupmenu', ...
                'Parent',D.UI.panStup, ...
                'Enable', 'off', ...
                'Callback', {@PopRewDel}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',[1 1 1], ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName',D.UI.popFont, ...
                'FontSize',text1_font_sz(1), ...
                'FontWeight','Bold', ...
                'String',D.UI.delList, ...
                'Value',1);
            
            % CUE BUTTON PANEL
            offset = 0.05;
            botm = pos_ht_dflt*2.5 + obj_gap_ht*2;
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', text2_font_sz(1), ...
                'Value',0);
            
            % SOUND BUTTON PANEL
            offset = 0.05;
            botm = pos_ht_dflt*1.5 + obj_gap_ht;
            pos = [pos_lft_dflt, botm, pos_wd_dflt, pos_ht_dflt];
            % panel
            D.UI.spanSnd = uibuttongroup(...
                'Parent',D.UI.panStup, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.backroundCol, ...
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', text2_font_sz(1), ...
                'Value',0);
            
            % SETUP DONE
            botm = 15*pxl2normStup_y;
            pos = [0.25, botm, 0.5, 45*pxl2normStup_y];
            D.UI.btnSetupDone = uicontrol('Style','push', ...
                'Parent', D.UI.panStup, ...
                'Enable', 'off', ...
                'String','DONE', ...
                'Callback', {@BtnSetupDone}, ...
                'UserData', 0, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol, ...
                'FontName', D.UI.btnFont, ...
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
                [12, 16/(D.UI.fg_pos(4)* D.UI.rec_pan_pos(4))*1.15];
            text1_font_sz = ...
                [10, 13/(D.UI.fg_pos(4)* D.UI.rec_pan_pos(4))];
            btn_rot_px_ht = 40/(D.UI.fg_pos(4)* D.UI.rec_pan_pos(4));
            
            % RECORD PANEL
            D.UI.panRec = uipanel(...
                'Parent',FigH, ...
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
            botm = pos_ht_dflt*3 + obj_gap_ht + text1_font_sz(2)*0.75;
            wdth = pos_wd_dflt*0.45;
            Ht = text1_font_sz(2)*1.5;
            pos = [pos_lft_dflt + offst*0.3, botm, wdth, Ht];
            D.UI.btnAcq = uicontrol('Style','radiobutton', ...
                'Parent',D.UI.panRec, ...
                'UserData', 0, ...
                'Enable', 'on', ...
                'Visible', 'on', ...
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
                'FontSize', text1_font_sz(1)+2, ...
                'Value',0);
            % REC BUTTON
            pos = [pos_lft_dflt + wdth + offst*0.4, botm, wdth, Ht];
            D.UI.btnRec = uicontrol('Style','radiobutton', ...
                'Parent',D.UI.panRec, ...
                'Enable', 'on', ...
                'Visible', 'on', ...
                'Callback', {@BtnRec}, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.backroundCol, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'String','REC', ...
                'CData',D.UI.radBtnCmap{1}, ...
                'FontWeight','Bold', ...
                'FontWeight','Bold', ...
                'FontSize', text1_font_sz(1)+2, ...
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
                'BackgroundColor', D.UI.dfltBtnLightCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize',12, ...
                'UserData', false);
            
            % SESSION DONE
            botm = 15*pxl2normRec_y;
            pos = [0.25, botm, 0.5, 45*pxl2normRec_y];
            D.UI.btnRecDone = uicontrol('Style','push', ...
                'Parent', D.UI.panRec, ...
                'Enable', 'off', ...
                'String','DONE', ...
                'Callback', {@BtnRecDone}, ...
                'UserData', 0, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.dfltBtnLightCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol, ...
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
                'BackgroundColor', D.UI.backroundCol, ...
                'ForegroundColor', D.UI.disabledCol, ...
                'HighlightColor', D.UI.disabledCol, ...
                'FontSize',15, ...
                'FontWeight','Bold', ...
                'Title','Console', ...
                'TitlePosition','centertop', ...
                'Clipping','on', ...
                'Position', D.UI.vel_pan_pos);
            
            D.UI.editConsole = uicontrol(...
                'Parent',D.UI.panConsole,...
                'Units','normalized',...
                'Position',[0,0,1,1],...
                'Style','edit',...
                'HorizontalAlignment', 'Left', ...
                'FontSize', 7, ...
                'FontName','Monospaced', ...
                'Max', 1000, ...
                'Enable','inactive',...
                'String',consoleText);
            
            %% --------------------------OTHER BUTTONS---------------------------------
            
            % SAVE SESSION DATA
            pos = [0, plot_bnds(2), plot_bnds(1)/2, 0.05];
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
            pos = [plot_bnds(1)/2, plot_bnds(2), plot_bnds(1)/2, 0.05];
            D.UI.btnQuit = uicontrol('Style','push', ...
                'String','QUIT', ...
                'Callback', {@BtnQuit}, ...
                'Unit', 'Normalized', ...
                'Position', pos, ...
                'FontName', D.UI.btnFont, ...
                'FontSize',14);
            
            % HALT ROBOT
            wdth = 0.1;
            pos = [plot_bnds(1)+0.01, plot_bnds(4)-0.01-0.06, 0.1, 0.06];
            D.UI.btnHaltRob = uicontrol('Style','togglebutton', ...
                'Parent',FigH, ...
                'String','Halt Robot', ...
                'Callback', {@BtnHaltRob}, ...
                'Enable', 'off', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
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
                'BackgroundColor',[1 1 1], ...
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
                'FontName', D.UI.btnFont, ...
                'FontWeight','Bold', ...
                'FontSize', 10, ...
                'Value', 0);
            
            % REWARD BUTTON
            pos = [pos(1), pos(2)-0.03-0.01, wdth, 0.03];
            D.UI.btnReward = uicontrol('Style','push', ...
                'Parent',FigH, ...
                'String','Reward', ...
                'Callback', {@BtnReward}, ...
                'Enable', 'off', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol, ...
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol - 0.01, ...
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
                'BackgroundColor', D.UI.disabledCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol, ...
                'FontName', D.UI.btnFont, ...
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
                'Parent',FigH, ...
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
            botm = botm - 7*text_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*6];
            D.UI.txtSesInf(1) = uicontrol('Style','text', ...
                'Parent',FigH, ...
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
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.printBckCol, ...
                'ForegroundColor', D.UI.printTxtCol, ...
                'FontName','Courier New', ...
                'FontWeight','Light', ...
                'FontSize', text_font_sz(1));
            % laps per dropdown
            botm = botm - dd_font_sz(2);
            pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
            D.UI.lapsRewPerRot = uicontrol('Style','popupmenu', ...
                'Parent',FigH, ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',[1 1 1], ...
                'ForegroundColor', D.UI.enabledCol, ...
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
                'BackgroundColor',[1 1 1], ...
                'ForegroundColor', D.UI.enabledCol, ...
                'FontName','Courier New', ...
                'FontSize',dd_font_sz(1), ...
                'FontWeight','Light', ...
                'Visible','off', ...
                'Value',1);
            
            % PERFORMANCE INFO
            
            % Number of text lines not including header
            nlines(2) = 16;
            
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
                'Parent',FigH, ...
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
                'Parent',FigH, ...
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
                'Parent',FigH, ...
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
                'Parent',FigH, ...
                'String','Lap Times', ...
                'Units','Normalized', ...
                'Position', pos, ...
                'BackgroundColor',[1 1 1], ...
                'ForegroundColor', D.UI.enabledCol, ...
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
                'BackgroundColor', D.UI.printBckCol, ...
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
                'BackgroundColor', D.UI.printBckCol, ...
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
                'BackgroundColor', D.UI.printBckCol, ...
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
                'BackgroundColor', D.UI.printBckCol, ...
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
                'BackgroundColor', D.UI.printBckCol, ...
                'ForegroundColor', D.UI.robNowCol, ...
                'FontName','Courier New', ...
                'FontWeight','Bold', ...
                'FontSize', text_font_sz(1));
            
            % TIMER INFO
            
            % Number of text lines not including header
            nlines(3) = 10;
            
            % pannel
            D.UI.timInf_pan_pos = [...
                pan_lft, ...
                0, ...
                pan_wd, ...
                nlines(3)*text_font_sz(2) + head_font_sz(2) + 4*D.UI.pxl2norm_y*2];
            D.UI.panTimInf = uipanel(...
                'Parent',FigH, ...
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
                'Parent',FigH, ...
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
                'Parent',FigH, ...
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
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Left', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.printBckCol, ...
                'ForegroundColor', D.UI.printTxtCol, ...
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
                'BackgroundColor', D.UI.printBckCol, ...
                'ForegroundColor', D.UI.printTxtCol, ...
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
            % loop time
            botm = botm - text_font_sz(2)*2;
            pos = [pos_lft_dflt, botm, ...
                pos_wd_dflt, text_font_sz(2)*1];
            D.UI.txtTimLoop = uicontrol('Style','text', ...
                'Parent',FigH, ...
                'String','', ...
                'Units','Normalized', ...
                'HorizontalAlignment', 'Right', ...
                'Position', pos, ...
                'BackgroundColor', D.UI.printBckCol, ...
                'ForegroundColor', D.UI.printTxtCol, ...
                'FontName','Courier New', ...
                'FontWeight','Light', ...
                'FontSize', 6);
            
            %% ========================================================================
            
            %% MAKE FIGURE VISIBLE
            
            % Set position
            movegui(FigH,'center')
            % Bring UI to top
            uistack(FigH, 'top')
            drawnow;
            % Make visible
            set(FigH, 'Visible', 'on');
            
        end
        
        % -------------------------------AC SETUP----------------------------------
        
        function [] = SF_AC_Setup()
            
            % Setup communication with ARENACONTROLLER
            D.AC.IP = '172.17.0.3';
            
            % Initialize parameters
            % signal to connect to ARENACONTROLLER
            data(1) = 1;
            % signal for ICR condition
            data(2) = 0;
            % signal for rotation direction
            data(3) = 0;
            % signal for sound condition
            data(4) = 0;
            
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
            Update_Console(sprintf('\rConnecting To AC Computer...\rIP: %s\rTime: %s\r', ...
                D.AC.IP, datestr(now, 'HH:MM:SS')));
            
            % Establish connection
            % will loop here until D.AC.data(1) established
            fopen(tcpIP);
            
            % Print that AC computer is connected
            Update_Console(sprintf('\rConnected To AC Computer\rIP: %s\rTime: %s\r', ...
                D.AC.IP, datestr(now, 'HH:MM:SS')));
            
        end
        
        % ------------------------------NLX SETUP----------------------------------
        
        function[] = SF_NLX_Setup()
            
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
            
            % Audio channels
            D.NLX.snd_rt_wn_bit = '1';
            D.NLX.snd_lft_rt_bit = '2';
            
            % Rew
            D.NLX.rew_on_bit = '3';
            D.NLX.rew_off_bit = '4';
            
            % Bulldozer state
            D.NLX.bull_run_bit = '5';
            D.NLX.bull_stop_bit = '6';
            
            % PID mode
            D.NLX.pid_run_bit = '0';
            D.NLX.pid_stop_bit = '7';
            
            % Photo Transducers
            D.NLX.north_bit = '7';
            D.NLX.west_bit = '6';
            D.NLX.south_bit = '5';
            D.NLX.east_bit = '4';
            
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
            
            % Mat to CS event command string
            D.NLX.m2cs_evt = '-PostEvent Post_Mat2CS_ID:%s_D1:%0.4f_D2:%0.4f 210 0';
            
            % Session end command string
            D.NLX.ses_end_evt = '-PostEvent Post_Session_End 211 0';
            
            %% CONNECT TO NETCOM
            
            Update_Console(sprintf('\rConnecting To NLX...\rIP: %s\rTime: %s\r', ...
                D.NLX.IP, datestr(now, 'HH:MM:SS')));
            
            % Wait for Cheetah to open
            is_running = false;
            while ~is_running && ~shouldExit
                [~,result] = system('tasklist /FI "imagename eq cheetah.exe" /fo table /nh');
                is_running = any(strfind(result, 'Cheetah.exe'));
            end
            
            % Load NetCom into Matlab, and connect to the NetCom server if we aren’t connected
            if NlxAreWeConnected() ~= 1
                while NlxAreWeConnected() ~= 1 && ~shouldExit
                    succeeded = NlxConnectToServer(D.NLX.IP);
                    if succeeded == 1
                        Update_Console(sprintf('\rConnected To NLX\rIP: %s\rTime: %s\r', ...
                            D.NLX.IP, datestr(now, 'HH:MM:SS')));
                        %Identify this program to the server.
                        NlxSetApplicationName('ICR_GUI');
                    end
                end
            else
                Update_Console(sprintf('\rAlready Connected to NLX\rIP: %s\rTime: %s\r', ...
                    D.NLX.IP, datestr(now, 'HH:MM:SS')));
            end
            
            %% CONFIGURE DIGITAL IO
            
            % Pair Cube headstage (Should be commented out unless router has been unpluged)
            %NlxSendCommand('-SendLynxSXCommand AcqSystem1 -InitWHSPairing 30')
            
            % Turn on Cube LEDs
            %NlxSendCommand('-SendLynxSXCommand AcqSystem1 -WHSSetTrackingLED 1 1');
            
            % Open the data stream for the VT acquisition entity.  This tells Cheetah to begin %streaming
            % data for the VT acq ent.
            NlxOpenStream(D.NLX.vt_rat_ent);
            NlxOpenStream(D.NLX.vt_rob_ent);
            NlxOpenStream(D.NLX.event_ent);
            
            % Get current folder name
            dirs = dir(D.DIR.nlxTempTop);
            D.DIR.recFi = dirs([dirs.datenum] == max([dirs.datenum])).name;
            
            % Set port directions
            NlxSendCommand(['-SetDigitalIOportDirection ', D.NLX.DevName, ' ', D.NLX.port_0, ' Input']);
            NlxSendCommand(['-SetDigitalIOportDirection ', D.NLX.DevName, ' ', D.NLX.port_1, ' Input']);
            
            % Enable digital io events
            NlxSendCommand(['-SetDigitalIOEventsEnabled ', D.NLX.DevName, ' ', D.NLX.port_0, ' True']);
            NlxSendCommand(['-SetDigitalIOEventsEnabled ', D.NLX.DevName, ' ', D.NLX.port_1, ' True']);
            
            % Set TTL Event strings
            % audio channels
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_1, ' ', D.NLX.snd_rt_wn_bit, ' ', D.NLX.snd_rt_wn_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_1, ' ', D.NLX.snd_lft_rt_bit, ' ', D.NLX.snd_lft_rt_str]);
            % reward
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_1, ' ', D.NLX.rew_on_bit, ' ', D.NLX.rew_on_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_1, ' ', D.NLX.rew_off_bit, ' ', D.NLX.rew_off_str]);
            % pid mode
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_1, ' ', D.NLX.pid_run_bit, ' ', D.NLX.pid_run_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_1, ' ', D.NLX.pid_stop_bit, ' ', D.NLX.pid_stop_str]);
            % bulldozer state
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_1, ' ', D.NLX.bull_run_bit, ' ', D.NLX.bull_run_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_1, ' ', D.NLX.bull_stop_bit, ' ', D.NLX.bull_stop_str]);
            % photo transdicers
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_0, ' ', D.NLX.north_bit, ' ', D.NLX.north_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_0, ' ', D.NLX.west_bit, ' ', D.NLX.west_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_0, ' ', D.NLX.south_bit, ' ', D.NLX.south_str]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_0, ' ', D.NLX.east_bit, ' ', D.NLX.east_str]);
            
            %% START ACQUISITION
            % Run BtnAcq
            set(D.UI.btnAcq, 'Value', 1)
            BtnAcq(D.UI.btnAcq);
            
            %% ENABLE SETUP BUTTONS ONCE CONNECTED
            % Enable all setup buttons
            set(D.UI.panStup, 'ForegroundColor', D.UI.enabledCol, ...
                'HighlightColor',D.UI.enabledCol)
            set(D.UI.popRat, 'Enable', 'on')
            set(D.UI.popCond, 'Enable', 'on')
            set(D.UI.popTask, 'Enable', 'on')
            set(D.UI.popRwDl, 'Enable', 'on')
            set(D.UI.toggCue, 'Enable', 'on', ...
                'ForegroundColor', D.UI.dfltTxtLightCol, ...
                'BackgroundColor', D.UI.enabledCol)
            set(D.UI.toggSnd, 'Enable', 'on', ...
                'ForegroundColor', D.UI.dfltTxtLightCol, ...
                'BackgroundColor', D.UI.enabledCol)
            set(D.UI.btnSetupDone, 'Enable', 'on', ...
                'BackgroundColor', D.UI.enabledCol)
            drawnow;
            
            % Print GUI ready
            Update_Console(sprintf('\rGUI Ready\rTime: %s\r', ...
                datestr(now, 'HH:MM:SS')));
        end
        
        % ----------------------------FINISH SETUP---------------------------------
        function [] = SF_Finish_Setup()
            
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
            
            % icr session
            D.PAR.sesNumICR = ...  % [1:200]
                D.SS_In_All.Session_Rotation(D.PAR.ratInd) + 1;
            
            % start quadrant
            D.PAR.ratStrQuad = categorical({'<undefined>'}, D.PAR.catStrQuad);
            % check for manual training session
            if D.PAR.sesCond ~= 'Manual_Training'
                D.PAR.ratStrQuad = ... % [NE,SE,SW,NW]
                    D.SS_In_All.Start_Quadrant{D.PAR.ratInd}(D.PAR.sesNum);
            else
                % set start quad to feeder pos
                if D.PAR.ratFeedCnd == 'C1'
                    D.PAR.ratStrQuad(:) = 'NW';
                else
                    D.PAR.ratStrQuad(:) = 'SE';
                end
            end
            
            % rotations per session
            D.PAR.rotPerSes = ... % [2,4,6]
                str2double(char(D.SS_In_All.Rotations_Per_Session{D.PAR.ratInd}(D.PAR.sesNumICR,:)));
            
            % rotations position
            D.PAR.rotPos = ... % [90,180,270]
                D.SS_In_All.Rotation_Positions{D.PAR.ratInd}(D.PAR.sesNumICR,:)';
            
            % laps per session
            D.PAR.lapsPerRot = ... % [5:8,6:9,7:10]
                D.SS_In_All.Laps_Per_Rotation{D.PAR.ratInd}(D.PAR.sesNumICR,:)';
            
            % days till rotation
            D.PAR.daysTilRot = ... % [5:8,6:9,7:10]
                D.SS_In_All.Days_Till_Rotation{D.PAR.ratInd}(D.PAR.sesNumICR);
            
            % Seed random number generator based on session number
            rng(D.PAR.sesNum)
            
            % Make rew per ses list
            txt = [{'Laps Per Rotation'}; ...
                cellfun(@(x,y) sprintf('%d:    %s', x, y), ...
                num2cell(1:length(D.PAR.rotPos))', ...
                cellstr(char(D.PAR.lapsPerRot)), 'Uni', false)];
            set(D.UI.lapsRewPerRot, 'String', txt);
            
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
            
            % Reverse start feeder if rotation is CW
            if D.PAR.ratRotDrc == 'CCW';
                D.I.feed_ind = [1, 2];
            elseif D.PAR.ratRotDrc == 'CW';
                D.I.feed_ind = [2, 1];
            end
            
            % Reverse wall image order
            if D.PAR.ratRotDrc == 'CW';
                D.UI.wallImgH = flip(D.UI.wallImgH);
            end
            % Show wall image
            set(D.UI.wallImgH(1), 'Visible', 'on');
            
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
            Mat2CS('S', d1, d2);
            
            %% Update and send AC.data values
            
            % Display image
            D.AC.data(2) = D.I.rot;
            
            % Rotation direction
            D.AC.data(3) = D.PAR.ratRotDrc_Num;
            
            % Sound stimuli (start without sound)
            D.AC.data(4) = single(D.UI.snd(1));
            
            % Post to AC computer
            fwrite(tcpIP,D.AC.data,'int8');
            
            % Reset rotation direction after first run
            D.AC.data(3) = 0; % rotation direction
            
            %% Update UI objects
            
            % Disable all setup buttons
            set(D.UI.panStup, 'ForegroundColor', D.UI.disabledCol, ...
                'HighlightColor',D.UI.disabledCol)
            set(D.UI.popRat, 'Enable', 'off')
            set(D.UI.popCond, 'Enable', 'off')
            set(D.UI.popTask, 'Enable', 'off')
            set(D.UI.popRwDl, 'Enable', 'off')
            set(D.UI.toggCue, 'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol)
            set(D.UI.toggSnd, 'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol)
            
            % Enable other buttons
            
            if D.PAR.sesCond ~= 'Manual_Training'
                
                % Halt Robot
                set(D.UI.btnHaltRob, ...
                    'Enable', 'on', ...
                    'BackgroundColor', D.UI.enabledCol, ...
                    'ForegroundColor' , D.UI.dfltTxtLightCol);
                
                % Bulldoze
                set(D.UI.popBulldoze, ...
                    'Enable', 'on')
                set(D.UI.btnBulldoze, ...
                    'Enable', 'on', ...
                    'BackgroundColor', D.UI.enabledCol, ...
                    'ForegroundColor' , D.UI.dfltTxtLightCol, ...
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
                        'ForegroundColor' , D.UI.dfltTxtLightCol);
                    
                    % Block Cue
                    set(D.UI.btnAllCue, ...
                        'Enable', 'on', ...
                        'BackgroundColor', D.UI.enabledCol, ...
                        'ForegroundColor' , D.UI.dfltTxtLightCol);
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
            set(D.UI.lapsRewPerRot, 'Visible', 'on');
            set(D.UI.popRotPos, 'Visible', 'on');
            set(D.UI.popLapTim, 'Visible', 'on');
            
            %% Print session and performance and time info
            
            % Print session info
            % get values to print
            dobNum = datenum(D.PAR.ratDOB, 'yyyy/mm/dd');
            agemnth = num2str(floor((now - dobNum)/365*12));
            ses = num2str(D.PAR.sesNum);
            ses_tot = num2str(D.PAR.sesNumAll);
            infstr = sprintf([...
                'Session:%s%s/%s\n', ...
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
            
            if D.PAR.sesCond == 'Rotation'
                rots = num2str(D.PAR.rotPerSes);
            else
                rots = 'NA';
                set(D.UI.lapsRewPerRot, 'Enable', 'off');
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
                'Bulldozings____:%s%d'], ...
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
            
            % rat vel
            infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', 0, 0, 0);
            set(D.UI.txtPerfInf(5), 'String', infstr)
            % rob vel
            infstr = sprintf('Velocity:_%0.2f(%0.0f/%0.0f)', 0, 0, 0);
            set(D.UI.txtPerfInf(6), 'String', infstr)
            
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
                'loop_: %4.0f  min:%4.0f  max:%4.0f\n', ...
                0, 0, 0);
            set(D.UI.txtTimLoop, 'String', infstr)
            
            %% Get bounds for various opperations
            
            % Specify reward feeder locations
            % NOTE: Feeder index is based on the position of the feeder with
            % respect to the 0 deg point (East quadrant)in the arena
            rewFeeds = [11, 15; 29, 33];
            
            % Feeder paramiters
            % boundary before and after rew feeder (deg)
            D.PAR.feedBnds = [...
                rad2deg(D.PAR.feedDist - D.PAR.setPoint) - 5, ...
                rad2deg(D.PAR.feedDist - D.PAR.setPoint) + 20];
            
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
            
            % Save reward feeder rad pos
            D.UI.rewFeedRad(1) = deg2rad(fdLocs(D.UI.rewFeed(1)));
            D.UI.rewFeedRad(2) = deg2rad(fdLocs(D.UI.rewFeed(2)));
            
            % REWARD FEEDER BOUNDS
            D.UI.rewBnds = [...
                D.UI.rewFeedRad(1) + deg2rad(D.PAR.feedBnds(1)), ...
                D.UI.rewFeedRad(1) + deg2rad(D.PAR.feedBnds(2)); ...
                D.UI.rewFeedRad(2) + deg2rad(D.PAR.feedBnds(1)), ...
                D.UI.rewFeedRad(2) + deg2rad(D.PAR.feedBnds(2))];
            % set to range [0 2*pi]
            D.UI.rewBnds = wrapTo2Pi(D.UI.rewBnds);
            
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
            
            % COPY FOR REWARD RESET
            D.UI.rewRstBnds = squeeze(D.UI.rotBnds(1,:,:))';
            
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
            
            % START QUADRANT BOUNDS
            
            % Set start quadrant bound to 60 deg
            D.UI.strQuadBnds = [lapBndLocs(end) - 30, ...
                lapBndLocs(end) + 30];
            % convert to radians
            D.UI.strQuadBnds = deg2rad(D.UI.strQuadBnds);
            
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
            
            % Plot 0-deg feeder bounds
            [xbnd, ybnd] =  ...
                Get_Rad_Bnds(D.UI.rewBnds(1,:));
            D.UI.ptchFdH(1) = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                D.UI.rotCol(1,:), ...
                'FaceAlpha', 0.75, ...
                'Parent', D.UI.axH(2));
            
            % Plot 40-deg feeder bounds
            % set alpha to 0 (transparent)
            [xbnd, ybnd] =  ...
                Get_Rad_Bnds(D.UI.rewBnds(2,:));
            D.UI.ptchFdH(2) = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                D.UI.rotCol(2,:), ...
                'FaceAlpha', 0, ...
                'EdgeAlpha', 0, ...
                'Parent', D.UI.axH(2));
            
            % Plot reward reset bounds
            % set alpha to 0 (transparent)
            D.UI.ptchFdRstH = gobjects(2,1);
            for z_fd = 1:2
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds(D.UI.rewRstBnds(z_fd,:));
                D.UI.ptchFdRstH(z_fd) = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    [0.5,0.5,0.5], ...
                    'FaceAlpha', 0.1, ...
                    'EdgeAlpha', 1, ...
                    'Parent', D.UI.axH(2), ...
                    'Visible', 'off');
            end
            
            % Plot lap bounds
            % set alpha to 0 (transparent)
            D.UI.ptchLapBnds = gobjects(4,1);
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
                'MarkerEdgeColor', 'k', ...
                'MarkerSize', 10, ...
                'Parent', D.UI.axH(2));
            
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
            
            D.UI.cueH = gobjects(2,2);
            % Plot cue distance lines
            for z_fd = 1:2
                [xbnd, ybnd] =  ...
                    Get_Rad_Bnds([D.UI.rewFeedRad(z_fd), D.UI.rewFeedRad(z_fd)+D.PAR.cueDist]);
                % line
                D.UI.cueH(z_fd,1) = ...
                    plot(xbnd(2,:), ybnd(2,:), ...
                    'Color', D.UI.dfltActiveCol, ...
                    'LineWidth', 5, ...
                    'Parent',D.UI.axH(2),...
                    'Visible', 'off');
                % marker
                D.UI.cueH(z_fd,2) = ...
                    plot(D.UI.fd_x(D.UI.rewFeed(z_fd)), ...
                    D.UI.fd_y(D.UI.rewFeed(z_fd)), 'o', ...
                    'MarkerFaceColor', D.UI.dfltActiveCol, ...
                    'MarkerEdgeColor', 'None', ...
                    'MarkerSize', 25, ...
                    'Parent',D.UI.axH(2),...
                    'Visible', 'off');
                % move down
                uistack(D.UI.cueH(z_fd,:),'down',5);
            end
            
            
            %% Start recording and set remaining vars
            
            % Check cue condition and set vars
            if D.PAR.sesCond ~= 'Manual_Training'
                if D.PAR.cueFeed == 'All' || D.PAR.cueFeed == 'Half'
                    
                    % Cueing first lap
                    D.B.is_cued_rew = true;
                    
                    % Check if condition and start quad leave time to send cue
                    % command
                    if ...
                            (D.PAR.ratFeedCnd == 'C1' && D.PAR.ratStrQuad == 'NW' && D.PAR.ratRotDrc == 'CCW') || ...
                            (D.PAR.ratFeedCnd == 'C2' && D.PAR.ratStrQuad == 'SE' && D.PAR.ratRotDrc == 'CCW')
                        
                        % Set flag
                        D.B.flag_rew_reset = true;
                        
                        % Set reset patch to not visible
                        set(D.UI.ptchFdRstH(D.I.rot), 'Visible', 'off');
                        
                        % Show cue marker
                        set(D.UI.cueH(D.I.rot,:), 'Visible', 'off');
                        
                    else
                        
                        % Set flag
                        D.B.flag_rew_reset = false;
                        
                        % Set reset patch to visible
                        set(D.UI.ptchFdRstH(D.I.rot), 'Visible', 'on');
                        
                    end
                end
            end
            
            D.C.moveDel = clock;
            D.C.doneMove = false;
            % Set to start polling NLX
            D.B.poll_nlx = true;
            
            % Run BtnRec
            set(D.UI.btnRec,'Value', 1);
            BtnRec(D.UI.btnRec);
            
            % Update GUI
            drawnow;
            
            % Print setup completed
            Update_Console(sprintf('\rSetup Completed\rTime: %s\r', ...
                datestr(now, 'HH:MM:SS')));
            
        end
        
        % ----------------------------RAT IN CHECK---------------------------------
        
        function [] = SF_Rat_In_Check()
            
            if all(~isnan(D.P.Rat.rad))
                
                % Keep checking if rat is in the arena
                check_inbound = Check_Rad_Bnds(D.P.Rat.rad, D.UI.strQuadBnds);
                
                if any(check_inbound)
                    
                    % Get first in-bound ts
                    if D.T.strqd_inbnd_t1 == 0
                        D.T.strqd_inbnd_t1 = D.P.Rat.ts(find(check_inbound, 1, 'first'));
                    end
                    
                    % Get most recent in-bounds ts
                    D.T.strqd_inbnd_t2 = D.P.Rat.ts(find(check_inbound, 1, 'last'));
                    
                    % Compute time in seconds
                    inbndTim = (D.T.strqd_inbnd_t2 - D.T.strqd_inbnd_t1) / 10^6;
                    
                    % Check if rat has been in for the min delay period
                    if inbndTim >= D.PAR.strQdDel
                        
                        % Tell CS rat is in
                        Mat2CS('I', 1);
                        
                        % Post NLX event: rat in
                        NlxSendCommand(D.NLX.rat_in_evt);
                        
                        % Change bool so code will only be run once
                        D.B.rat_in = true;
                        
                        % Save time
                        D.T.run_str = clock;
                        
                        % Start tracking lap times
                        D.T.lap_tim = clock;
                        
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
                        
                        Update_Console(sprintf('\rRat In \rInbounds for %0.2fsec \rTime: %s\r', ...
                            inbndTim, datestr(now, 'HH:MM:SS')));
                        
                    end
                else
                    % Reinitialize
                    D.T.strqd_inbnd_t1 = 0;
                end
                
            end
            
        end
        
        
        
        
        
        
        
        
        %% ======================== ONGOING FUNCTIONS =============================
        
        % ------------------------------PROCESS NLX VT---------------------------------
        
        function [] = SF_VT_Proc(fld)
            
            %% GET VT DATA
            
            % Get NXL vt data and reformat data with samples in column vectors
            if strcmp(fld, 'Rat')
                [~, vtTS, vtPos, vtHD, vtNRecs, ~] = NlxGetNewVTData(D.NLX.vt_rat_ent);
                D.P.(fld).hd_deg = vtHD';
            else
                [~, vtTS, vtPos, ~, vtNRecs, ~] = NlxGetNewVTData(D.NLX.vt_rob_ent);
            end
            
            if vtNRecs > 0
                % convdert to [r = samp, c = dim]
                xy_pos = reshape(double(vtPos),2,[])';
                ts = double(vtTS)';
                recs = vtNRecs;
                
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
                
                % Exclude all data durring reward events
                if D.B.is_rewarding
                    exc_4 = ones(size(rad_diff_last,1), 1);
                else
                    exc_4 = zeros(size(rad_diff_last,1), 1);
                end
                
                % Do not use exc_3/4 if more than 1.5 sec of unused data
                if etime(clock, D.T.(fld).last_pos_update) > 1.5
                    exc_3 = zeros(size(exc_3,1), 1);
                    exc_4 = zeros(size(exc_3,1), 1);
                end
                
                % Combine exclusion criteria
                exc_ind = exc_1 | exc_2 | exc_3 | exc_4;
                
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
                    
                    % Exit function
                    return;
                    
                else
                    % Plot pos data this loop
                    D.B.(fld).plot_pos = true;
                    
                    % Keep track of updates
                    D.T.(fld).last_pos_update = clock;
                    
                    % Save last usable rad value
                    D.P.(fld).radLast = rad(end);
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
                if strcmp(fld, 'Rob') && ~D.B.rat_in
                    % use setpoint pos
                    D.P.(fld).velRad = D.P.Rob.setRad;
                    % Use rat pos
                else
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
                    
                else
                    
                    % Set to plot this vel
                    D.B.(fld).plot_vel = true;
                    
                    % Save average
                    D.P.(fld).vel = nanmean(vel);
                    
                    if D.P.(fld).vel < 150
                        D.P.(fld).vel_max_lap = max(D.P.(fld).vel_max_lap, D.P.(fld).vel);
                        D.P.(fld).vel_max_all = max(D.P.(fld).vel_max_lap, D.P.(fld).vel_max_all);
                    end
                    
                    % Shift stored averages
                    D.P.(fld).vel_pol_arr = circshift(D.P.(fld).vel_pol_arr, -1, 1);
                    
                    % Map values to circle
                    % convert vel to roh range
                    velRoh = D.P.(fld).vel;
                    if velRoh>D.P.velMax; velRoh = D.P.velMax;
                    elseif velRoh<D.P.velMin; velRoh = D.P.velMin;
                    end
                    velRoh = velRoh/D.P.velMax;
                    velRoh = velRoh*(D.P.velRohMax-D.P.velRohMin) + D.P.velRohMin;
                    % cap to max
                    if velRoh > D.P.velRohMax;
                        velRoh = D.P.velRohMax;
                    end
                    % convert to car
                    [D.P.(fld).vel_pol_arr(end, 1), D.P.(fld).vel_pol_arr(end, 2)] = ...
                        pol2cart(D.P.(fld).velRad, velRoh);
                    D.P.(fld).vel_pol_arr(end, 1) =  D.P.(fld).vel_pol_arr(end, 1).*D.PAR.R + D.PAR.XC;
                    D.P.(fld).vel_pol_arr(end, 2) =  D.P.(fld).vel_pol_arr(end, 2).*D.PAR.R + D.PAR.YC;
                    
                    % Save history
                    ind = find(isnan(D.P.(fld).vel_hist(:,1)), 1, 'first');
                    D.P.(fld).vel_hist(ind, 1) = D.P.(fld).velRad;
                    D.P.(fld).vel_hist(ind, 2) = velRoh;
                    
                end
                
                %% TRANSFORM HD
                
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
        
        function [] = SF_Evt_Proc()
            
            % Read in new event data
            %[evtPass, evtTS, evtID, evtTTL, evtStr, evtNRecs, evtDropped]
            [~, ~, ~, ~ , evtStr, evtNRecs, ~] = ...
                NlxGetNewEventData('Events');
            
            if evtNRecs > 0
                
                % CHECK FOR REWARD
                
                % Started
                if any(ismember(evtStr, D.NLX.rew_on_str))
                    
                    % Set flag to deal with tracking issues
                    D.B.is_rewarding = true;
                    
                    % Change feeder plot color and marker size
                    D.UI.feedPosCol = D.UI.dfltActiveCol;
                    D.UI.feedPosMarkSize = 30;
                    if isfield(D.UI, 'feedPltNow')
                        set(D.UI.feedPltNow, ...
                            'MarkerFaceColor', D.UI.feedPosCol, ...
                            'MarkerSize', D.UI.feedPosMarkSize);
                    end
                    
                    % Add to reward count for manual session
                    if D.PAR.sesCond == 'Manual_Training'
                        D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
                    end
                    
                    % Flicker reward button backround color
                    set(D.UI.btnReward, ...
                        'BackgroundColor', D.UI.dfltActiveCol);
                end
                % Stopped
                if any(ismember(evtStr, D.NLX.rew_off_str))
                    
                    % Set flag
                    D.B.is_rewarding = false;
                    
                    % Change feeder plot color and marker size
                    D.UI.feedPosCol = D.UI.rotCol(D.I.rot,:);
                    D.UI.feedPosMarkSize = 10;
                    
                    if isfield(D.UI, 'feedPltNow')
                        set(D.UI.feedPltNow, ...
                            'MarkerFaceColor', D.UI.feedPosCol, ...
                            'MarkerSize', D.UI.feedPosMarkSize);
                    end
                    
                    % Flicker reward button backround color
                    set(D.UI.btnReward, ...
                        'BackgroundColor', D.UI.enabledCol);
                    
                end
                
                % CHECK FOR PID
                
                % Running
                if any(ismember(evtStr, D.NLX.pid_run_str))
                    % Change setpoint plot color and width
                    D.UI.setPosCol = D.UI.dfltActiveCol;
                    D.UI.setPosLineWidth = 4;
                end
                % Stopped
                if any(ismember(evtStr, D.NLX.pid_stop_str))
                    % Change setpoint plot color
                    D.UI.setPosCol = D.UI.robNowCol;
                    D.UI.setPosLineWidth = 2;
                end
                
                % CHECK FOR BULLDOZE
                
                % Running
                if any(ismember(evtStr, D.NLX.bull_run_str))
                    % Change guard plot color and width
                    D.UI.guardPosCol = D.UI.dfltActiveCol;
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
        
        function [] = SF_Rotation_Trig_Check()
            
            % Check if rotation trigger button has been pressed
            if get(D.UI.btnICR, 'UserData')
                
                % Check if rat in rotation bounds
                check_inbound = Check_Rad_Bnds(D.P.Rat.rad, D.UI.rotBndNext);
                
                % Trigger rotation if boundary crossed
                if any(check_inbound)
                    
                    % Set bool to true
                    D.B.rotated = true;
                    
                    % Save current image
                    rotLast = D.I.rot;
                    
                    % Get new image
                    D.I.rot = find([1, 2] ~=  D.I.rot);
                    
                    % Update D.AC.data(2) and send command to rotate image
                    D.AC.data(2) = D.I.rot;
                    fwrite(tcpIP,D.AC.data,'int8');
                    
                    % Post NLX event: rotaion *deg
                    NlxSendCommand(D.NLX.rot_evt{D.I.rot});
                    
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
                    r = D.P.Rat.rad(check_inbound);
                    [x,y] = Get_Rad_Bnds(r(1));
                    plot(x,y, ...
                        'Color', D.UI.rotCol(D.I.rot,:), ...
                        'LineWidth', 3, ...
                        'Parent',D.UI.axH(2));
                    
                    % Change reward bounds patch
                    % active feeder
                    set(D.UI.ptchFdH(D.I.rot), ...
                        'FaceAlpha', 0.75, ...
                        'EdgeAlpha', 1)
                    % inactive feeder
                    set(D.UI.ptchFdH([1, 2] ~=  D.I.rot), ...
                        'FaceAlpha', 0, ...
                        'EdgeAlpha',0)
                    
                    % Change button features
                    set(D.UI.btnICR,'string', D.UI.btnICRstr{rotLast}, ...
                        'BackgroundColor', D.UI.rotCol(rotLast,:), ...
                        'UserData', false);
                    
                    % Change wall image
                    set(D.UI.wallImgH(rotLast), 'Visible', 'off');
                    set(D.UI.wallImgH(D.I.rot), 'Visible', 'on');
                    
                    % Change reward reset
                    % reset reward bool
                    D.B.flag_rew_reset = true;
                    % hide reward reset patch
                    set(D.UI.ptchFdRstH(:), 'Visible', 'off');
                    
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
        
        function [] = SF_Reward_Feeder_Check()
            
            % Skip for manual training
            if D.PAR.sesCond ~= 'Manual_Training'
                
                % Skip if no new data
                if any(~isnan(D.P.Rat.rad))
                    
                    % Check if rat in feeder bounds
                    D.B.check_inbound_rew = Check_Rad_Bnds(D.P.Rat.rad, D.UI.rewBnds(D.I.rot,:));
                    
                    % Check for inbounds data
                    if any(D.B.check_inbound_rew)
                        
                        % Track reward crossing
                        if ~D.B.flag_rew_crossed
                            
                            % Force cue
                            if D.B.do_all_cue && D.B.is_cued_rew == false
                                D.B.is_cued_rew = true;
                            elseif D.B.do_block_cue && D.B.is_cued_rew == true
                                D.B.is_cued_rew = false;
                            end
                            
                            % Reinitialize inbound timer
                            D.T.fd_inbnd_t1 = 0;
                            
                            % Set flags
                            D.B.flag_rew_crossed = true;
                            D.B.flag_reset_crossed = false;
                            
                        end
                        
                        % Check if reward has been reset
                        if D.B.flag_rew_reset
                            
                            % Get first in-bound ts
                            if D.T.fd_inbnd_t1 == 0
                                D.T.fd_inbnd_t1 = D.P.Rat.ts(find(D.B.check_inbound_rew, 1, 'first'));
                            end
                            
                            % Get most recent in-bounds ts
                            D.T.fd_inbnd_t2 = D.P.Rat.ts(find(D.B.check_inbound_rew, 1, 'last'));
                            
                            % Compute time in seconds
                            inbndTim = (D.T.fd_inbnd_t2 - D.T.fd_inbnd_t1) / 10^6;
                            
                            % Check if rat has been in for the min delay
                            % period or this was a cued lap
                            if inbndTim >= D.PAR.rewDel || ...
                                    D.B.is_cued_rew
                                
                                % Lighten reward feeder patch
                                set(D.UI.ptchFdH(D.I.rot), ...
                                    'FaceAlpha', 0.15, ...
                                    'EdgeAlpha', 1)
                                
                                % Set reset patch to visible
                                set(D.UI.ptchFdRstH(D.I.rot), 'Visible', 'on');
                                
                                % Check for end of cue lap
                                if D.B.is_cued_rew
                                    
                                    % Hide cue marker
                                    set(D.UI.cueH(D.I.rot,:), 'Visible', 'off');
                                    
                                    % Post NLX event: cue off
                                    NlxSendCommand(D.NLX.cue_off_evt);
                                    
                                end
                                
                                % Force update GUI
                                drawnow;
                                
                                % Reward now if cue was not sent cued or cue
                                % was not sent and reward
                                if D.PAR.cueFeed == 'None' || ...
                                        D.PAR.cueFeed == 'Half' || ...
                                        (D.PAR.cueFeed == 'All' && ~D.B.flag_cue_sent)
                                    
                                    % Tell CS to reward now
                                    Mat2CS('R', 0);
                                end
                                
                                % Check if next reward is cued
                                if ...
                                        D.PAR.cueFeed == 'All' || ...
                                        (D.PAR.cueFeed == 'Half' && ~D.B.is_cued_rew)
                                    
                                    % Reset cue sent bool
                                    D.B.flag_cue_sent = false;
                                    
                                    % Cue next lap
                                    D.B.is_cued_rew = true;
                                    
                                else
                                    
                                    % Dont cue next lap
                                    D.B.is_cued_rew = false;
                                    
                                end
                                
                                % Add to total reward count
                                if D.B.rotated
                                    D.C.rew_cnt{D.I.rot}(end) = D.C.rew_cnt{D.I.rot}(end) + 1;
                                else
                                    D.C.rew_cnt{3}(end) = D.C.rew_cnt{3}(end) + 1;
                                end
                                
                                % Update bool values
                                % reset next reward opportunity
                                D.B.flag_rew_reset = false;
                                % plot last point where rewarded
                                D.B.plot_rew = true;
                                
                                % Reinitialize timer
                                D.T.fd_inbnd_t1 = 0;
                                
                                Update_Console(sprintf('\rReward \rInbounds for %0.2fsec \rTime: %s\r', ...
                                    inbndTim, datestr(now, 'HH:MM:SS')));
                                
                            end
                            
                        end
                        
                    end
                    
                end
                
            end
            
        end
        
        % ------------------------REWARD RESET CHECK-------------------------------
        
        function [] = SF_Reward_Reset_Check()
            
            % Skip for manual training
            if D.PAR.sesCond ~= 'Manual_Training'
                
                % Skip if no new data
                if any(~isnan(D.P.Rat.rad))
                    
                    % Check if rat is in quad
                    check_inbound = Check_Rad_Bnds(D.P.Rat.rad, D.UI.rewRstBnds(D.I.rot,:));
                    
                    % If rat is in bounds
                    if any(check_inbound)
                        
                        % Check if reward was missed
                        if ~D.B.flag_reset_crossed
                            
                            % Check if reward was crossed but reset was not
                            % reset
                            if D.B.flag_rew_crossed && ...
                                    D.B.flag_rew_reset
                                
                                % Add to missed reward count
                                D.C.missed_rew_cnt = D.C.missed_rew_cnt+1;
                                
                                % Cue next lap
                                if D.PAR.cueFeed == 'Half'
                                    D.B.is_cued_rew = true;
                                end
                                
                                % Reset now
                                D.B.flag_rew_reset = false;
                            end
                            
                            % Force cue
                            if D.B.do_all_cue && D.B.is_cued_rew == false
                                D.B.is_cued_rew = true;
                            elseif D.B.do_block_cue && D.B.is_cued_rew == true
                                D.B.is_cued_rew = false;
                            end
                            
                            % Reset flag
                            D.B.flag_reset_crossed = true;
                            D.B.flag_rew_crossed = false;
                            
                        end
                        
                        if ~D.B.flag_rew_reset
                            
                            % Check if this is cued reward
                            if D.B.is_cued_rew
                                
                                % Set cue maker to visible
                                set(D.UI.cueH(D.I.rot,:), 'Visible', 'on');
                                
                                % Post NLX event: cue on
                                NlxSendCommand(D.NLX.cue_on_evt);
                                
                                % Tell robot to cue
                                if D.PAR.cueFeed == 'All'
                                    
                                    % Tell CS to have robot slow to reward
                                    Mat2CS('R', D.UI.rewFeedRad(D.I.rot));
                                    
                                    % Set flag
                                    D.B.flag_cue_sent = true;
                                    
                                end
                            end
                            
                            % Set flag to true
                            D.B.flag_rew_reset = true;
                            
                            % Hide reset patch
                            set(D.UI.ptchFdRstH(D.I.rot), 'Visible', 'off');
                            
                            % Darken reward feeder patch
                            set(D.UI.ptchFdH(D.I.rot), ...
                                'FaceAlpha', 0.75, ...
                                'EdgeAlpha', 1)
                        end
                        
                    end
                    
                end
                
            end
            
        end
        
        % ----------------------------LAP CHECK------------------------------------
        
        function [] = SF_Lap_Check()
            
            % Check if rat in lap quad bound
            track_quad = Check_Rad_Bnds(D.P.Rat.rad, D.UI.lapBnds(D.I.lap_hunt_ind, :));
            
            % If in quad bounds
            if any(track_quad)
                
                % Set specific bound bool to true
                D.B.check_inbound_lap(D.I.lap_hunt_ind) = true;
                
            end
            
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
            
            % Update lap count
            if all(D.B.check_inbound_lap);
                
                % Add lap to counter
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
                
                % Delete all tracker data from this lap
                delete(D.UI.ratPltH(isgraphics(D.UI.ratPltH)))
                delete(D.UI.ratPltHvel(isgraphics(D.UI.ratPltHvel)))
                delete(D.UI.robPltHvel(isgraphics(D.UI.robPltHvel)))
                
                % REPLOT CUMULATIVE VT DATA
                
                % Rat pos
                delete(D.UI.Rat.pltHposAll);
                D.UI.Rat.pltHposAll = ...
                    plot(D.P.Rat.pos_hist(:,1), D.P.Rat.pos_hist(:,2), '-', ...
                    'Color', D.UI.ratPosHistCol, ...
                    'LineWidth', 1, ...
                    'Parent', D.UI.axH(1));
                
                % PlLOT VEL ALL
                cols = [D.UI.ratHistCol; D.UI.robHistCol];
                flds = [{'Rat'},{'Rob'}];
                for i = [2,1]
                    f = flds{i};
                    % delete old handle
                    delete(D.UI.(f).pltHvelAll);
                    % Convert to cart
                    [x, y] = pol2cart(D.P.(f).vel_hist(:,1), D.P.(f).vel_hist(:,2));
                    x =  x.*D.PAR.R + D.PAR.XC;
                    y =  y.*D.PAR.R + D.PAR.YC;
                    % Plot
                    D.UI.(f).pltHvelAll = ...
                        plot(x, y, '-', ...
                        'Color', cols(i,:), ...
                        'LineWidth', 1, ...
                        'Parent', D.UI.axH(1));
                end
                
                % PLOT VEL AVG
                cols = [D.UI.ratAvgCol; D.UI.robAvgCol];
                flds = [{'Rat'},{'Rob'}];
                for i = [2,1]
                    f = flds{i};
                    
                    % Compute average velocity and plot
                    ind = ~isnan(D.P.(f).vel_hist(:, 1));
                    bnd = linspace(0, 2*pi, 101);
                    vel_rad = D.P.(f).vel_hist(ind, 1);
                    vel_roh = D.P.(f).vel_hist(ind, 2);
                    [~,h_inds]= histc(vel_rad, bnd);
                    roh_avg = cell2mat(arrayfun(@(x) nanmean(vel_roh(h_inds==x)), ...
                        1:100, 'Uni', false));
                    roh_avg = [roh_avg, roh_avg(1)]; %#ok<AGROW>
                    ind = ~isnan(roh_avg);
                    
                    % Convert to cart
                    [x, y] = pol2cart(bnd(ind), roh_avg(ind));
                    x =  x.*D.PAR.R + D.PAR.XC;
                    y =  y.*D.PAR.R + D.PAR.YC;
                    
                    % Plot
                    delete(D.UI.(f).pltHvelAvg)
                    D.UI.(f).pltHvelAvg = ...
                        plot(x, y, '-', ...
                        'Color', cols(i,:), ...
                        'LineWidth', 4, ...
                        'Parent', D.UI.axH(1));
                end
                
                % Reset max velocity
                D.P.Rat.vel_max_lap = 0;
                D.P.Rob.vel_max_lap = 0;
                
                % Save time
                lap_tim_ellapsed = etime(clock, D.T.lap_tim);
                
                % Get average
                D.T.lap_tim_sum = D.T.lap_tim_sum + lap_tim_ellapsed;
                lap_tim_average = D.T.lap_tim_sum / sum([D.C.lap_cnt{:}]);
                
                % Update lap time dropdown
                D.UI.lapTimList = [...
                    D.UI.lapTimList; ...
                    {sprintf('%d:    %1.1f', sum([D.C.lap_cnt{:}]), lap_tim_ellapsed)}];
                infstr = [...
                    sprintf('Lap Times (%1.1f)', lap_tim_average); ...
                    D.UI.lapTimList];
                set(D.UI.popLapTim, 'String', infstr);
                
                % reset timer
                D.T.lap_tim = clock;
                
            end
            
        end
        
        % --------------------------PLOT POSITION----------------------------------
        
        function [] = SF_Pos_Plot()
            
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
                    ind = find(D.B.check_inbound_rew, 1, 'last');
                    plot(D.P.Rat.x(ind), D.P.Rat.y(ind), 'o', ...
                        'MarkerFaceColor', D.UI.rotCol(D.I.rot,:), ...
                        'MarkerEdgeColor', [0, 0, 0], ...
                        'MarkerSize', 8, ...
                        'Parent', D.UI.axH(2));
                    
                    % Reset bool
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
        
        function [] = SF_Inf_Print()
            
            %% Print performance info
            
            % total
            infstr = sprintf([...
                'Laps______Total:%s%d\n', ...
                'Rewards___Total:%s%d\n', ...
                'Rotations_Total:%s%d\n', ...
                'Bulldozings____:%s%d'], ...
                repmat('_',1,2), sum([D.C.lap_cnt{:}]), ...
                repmat('_',1,2), sum([D.C.rew_cnt{:}]), ...
                repmat('_',1,2), D.C.rot_cnt, ...
                repmat('_',1,2), D.C.bull_cnt);
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
            
            %% Print time info
            
            % Get session time
            nowTim(1) =  etime(clock,D.T.ses_str_tim);
            
            % Get recording elapsed time plus saved time
            if  D.B.rec
                nowTim(2) = etime(clock, D.T.rec_tim_reset)  + D.T.rec_tim;
            else nowTim(2) = D.T.rec_tim; % keep showing save time
            end
            
            % Get lap time
            if D.B.rat_in
                nowTim(3) = etime(clock, D.T.lap_tim);
            else nowTim(3) = 0;
            end
            
            % Run time
            if D.B.rat_in && ~D.B.rec_done
                D.T.run_tim = etime(clock, D.T.run_str);
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
            
            % Loop time
            if D.T.loop > 0;
                D.T.loop_min = min(D.T.loop, D.T.loop_min);
            else
            end
            D.T.loop_max = max(D.T.loop, D.T.loop_max);
            infstr = sprintf( ...
                'loop_: %4.0f  min:%4.0f  max:%4.0f\n', ...
                D.T.loop*1000, D.T.loop_min*1000, D.T.loop_max*1000);
            set(D.UI.txtTimLoop, 'String', infstr)
            
            
            % Save current time to UserData
            set(D.UI.txtTimElspInf, 'UserData', nowTim)
            
        end
        
        
        
        
        
        
        
        %% ======================== CALLBACK FUNCTIONS ============================
        
        % RAT SELECTION
        function [] = PopRat(~, ~, ~)
            
            if get(D.UI.popRat,'Value') ~= 1
                
                % Change to active color
                set(D.UI.popRat, ...
                    'BackgroundColor', D.UI.dfltActiveCol, ...
                    'ForegroundColor', D.UI.dfltTxtLightCol);
                
                % Get rat data
                % rat label
                D.PAR.ratLab = ... % ('r####')
                    ['r',D.UI.ratList{get(D.UI.popRat,'Value')}(1:4)];
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
                
                % SET UI TO LAST SESSION
                
                % Set Session Condition
                D.PAR.sesCond = ...
                    D.SS_In_All.Session_Condition(D.PAR.ratInd);
                set(D.UI.popCond, 'Value', ...
                    find(ismember(D.UI.condList, D.PAR.sesCond)));
                % run callback
                PopCond();
                
                % Set Session Task to Track
                set(D.UI.popTask, 'Value', ...
                    find(ismember(D.UI.taskList, 'Track')));
                % run callback
                PopTask();
                
                % Set reward delay
                D.PAR.rewDel = ...
                    D.SS_In_All.Reward_Delay(D.PAR.ratInd);
                set(D.UI.popRwDl, 'Value', ...
                    find(ismember(D.UI.delList, D.PAR.rewDel)));
                % run callback
                PopRewDel();
                
                % Set cue condition
                D.PAR.cueFeed = ...
                    D.SS_In_All.Cue_Condition(D.PAR.ratInd);
                set(D.UI.toggCue( ...
                    ismember(D.PAR.catCueCond, D.PAR.cueFeed)), 'Value', 1);
                % Set all others to off
                set(D.UI.toggCue( ...
                    ~ismember(D.PAR.catCueCond, D.PAR.cueFeed)), 'Value', 0);
                ToggCue();
                
                % Set SOUND condition
                D.UI.snd(1:2) = logical(...
                    D.SS_In_All.Sound_Conditions(D.PAR.ratInd,1:2));
                set(D.UI.toggSnd(D.UI.snd(1:2)), 'Value', 1);
                ToggSnd();
                
            end
            
        end
        
        % SESSION CONDITION
        function [] = PopCond(~, ~, ~)
            
            % Change to active color
            set(D.UI.popCond, ...
                'BackgroundColor', D.UI.dfltActiveCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol);
            
            % Store value
            D.PAR.sesCond(:) = D.UI.condList(get(D.UI.popCond, 'Value'));
            
            % Change other buttons for manual training
            if D.PAR.sesCond == 'Manual_Training'
                
                % Set reward delay
                set(D.UI.popRwDl, 'Value', ...
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
                set(D.UI.popRwDl, 'Value', ...
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
            
        end
        
        % SESSION TASK
        function [] = PopTask(~, ~, ~)
            
            % Change to active color
            set(D.UI.popTask, ...
                'BackgroundColor', D.UI.dfltActiveCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol);
            
            % Store value
            D.PAR.sesTask(:) = D.UI.taskList(get(D.UI.popTask, 'Value'));
            
        end
        
        % REWARD DELAY
        function [] = PopRewDel(~, ~, ~)
            
            % Change to active color
            set(D.UI.popRwDl, ...
                'BackgroundColor', D.UI.dfltActiveCol, ...
                'ForegroundColor', D.UI.dfltTxtLightCol);
            
            if get(D.UI.popRwDl,'Value') ~= 1
                % Get selected minimum time in bounds for reward
                D.PAR.rewDel = str2double(D.UI.delList{get(D.UI.popRwDl,'Value'),:});
            end
            
        end
        
        % CUE CONDITION
        function [] = ToggCue(~, ~, ~)
            
            % Get trigger button
            cue_cond = get(gcbo, 'UserData');
            
            % Chack if callback was run without button press
            if isempty(cue_cond)
                cue_cond = find(cell2mat(get(D.UI.toggCue, 'Value')) == 1);
            end
            
            % Change button
            if get(D.UI.toggCue(cue_cond), 'Value') == 1
                cue_str = get(D.UI.toggCue(cue_cond), 'String');
                
                % Activate current button
                set(D.UI.toggCue(cue_cond), ...
                    'BackgroundColor', D.UI.dfltActiveCol);
                
                % Inactivate other buttons
                set(D.UI.toggCue(([1,2,3] ~= cue_cond)), ...
                    'BackgroundColor', D.UI.enabledCol, ...
                    'Value',0);
                
                % Save data
                D.PAR.cueFeed(:) = cue_str;
            end
            
            % Change Reward delay
            if (D.PAR.cueFeed == 'Half' || ...
                    D.PAR.cueFeed == 'None') && ...
                    D.PAR.sesCond ~= 'Manual_Training' && ...
                    D.PAR.rewDel == 0
                
                % Set reward delay
                set(D.UI.popRwDl, 'Value', ...
                    find(ismember(D.UI.delList, D.UI.delList(3))));
                % run callback
                PopRewDel();
                
            elseif D.PAR.cueFeed == 'All' && ...
                    D.PAR.rewDel ~= 0
                
                % 'All' should have 0 delay
                % Set reward delay
                set(D.UI.popRwDl, 'Value', ...
                    find(ismember(D.UI.delList, D.UI.delList(2))));
                % run callback
                PopRewDel();
                
            end
            
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
                        'BackgroundColor', D.UI.dfltActiveCol);
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
            
        end
        
        % SETUP DONE
        function [] = BtnSetupDone(~, ~, ~)
            
            % Disable button so only pressed once
            set(D.UI.btnSetupDone, 'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            
            % Confirm UI entries
            all(cell2mat(get(D.UI.toggCue,'Value')) == 0)
            % Check if all fields entered with pop-up window
            if ...
                    get(D.UI.popRat,'Value') == 1 || ...
                    get(D.UI.popCond,'Value') == 1 || ...
                    get(D.UI.popTask,'Value') == 1 || ...
                    get(D.UI.popRwDl,'Value') == 1 || ...
                    all(cell2mat(get(D.UI.toggCue,'Value')) == 0)
                
                wrnstr = '!!WARNING: MISSING ENTRY!!';
                choice = questdlgAWL(wrnstr, ...
                    'MISSING SETUP ENTRIES', 'OK', [], [], 'OK', D.UI.qstDlfPos);
                drawnow; % force update UI
                % Handle response
                switch choice
                    case 'OK'
                        return
                end
            end
            
            %         % Confirm if all fields entered correctly with pop-up window
            %         qststr = sprintf(['IS THIS CORRECT:\n\n'...
            %             'Rat_Number:_______%d\n'...
            %             'ICR_Condition:____%s\n', ...
            %             'Feeder_Delay:_____%1.1f\n', ...
            %             'Cue_Conditon:_____%s\n'...
            %             'Sound_White:______%s\n'...
            %             'Sound_Reward:_____%s\n'], ...
            %             D.PAR.ratNum, ...
            %             D.UI.condList{get(D.UI.popCond, 'Value')}, ...
            %             D.PAR.rewDel, ...
            %             char(D.PAR.cueFeed), ...
            %             D.UI.sndState{get(D.UI.toggSnd(1),'Value')+1}, ...
            %             D.UI.sndState{get(D.UI.toggSnd(2),'Value')+1});
            %         choice = questdlgAWL(qststr, ...
            %             'CHECK SETTINGS', 'Yes', 'No', [], 'No', D.UI.qstDlfPos);
            %         drawnow; % force update UI
            %         % Handle response
            %         switch choice
            %             case 'Yes'
            %             case 'No'
            %                 return
            %         end
            
            % Change button state to indicate setup done
            %set(D.UI.btnSetupDone, 'UserData', 1);
            
            
            
            % Signal to continue setup
            D.B.setup = true;
            
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
                    D.T.acq_tim = etime(clock,D.T.acq_tim_reset) + D.T.acq_tim;
                end
                
                % Change aquiring status
                D.B.acq = ~D.B.acq;
                
                % Reset temp clock
                D.T.acq_tim_reset = clock;
                
            end
            
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
                D.T.rec_tim = etime(clock, D.T.rec_tim_reset) + D.T.rec_tim;
            end
            D.B.rec = ~ D.B.rec;
            D.T.rec_tim_reset = clock;
            
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
                'FaceAlpha',0.5, ...
                'Parent', D.UI.axH(2));
            
            % Make button red and set user data to true
            set(D.UI.btnICR,'string', 'Rotation Ready', ...
                'BackgroundColor', D.UI.dfltActiveCol, ...
                'UserData', true);
            
        end
        
        % RECORDING DONE
        function [] = BtnRecDone(~, ~, ~)
            
            % Check if session is done
            choice = questdlgAWL(...
                'End Session?', ...
                'END SESSION', ...
                'Yes', 'No', [], 'No', ...
                D.UI.qstDlfPos);
            % Force update UI
            drawnow;
            
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
                'BackgroundColor', D.UI.disabledCol);
            
            % Set bool
            D.B.rec_done = true;
            
            % Save end time
            D.T.ses_end_tim = clock;
            
            % Print end time
            infstr = datestr(D.T.ses_end_tim, 'HH:MM:SS');
            set(D.UI.editTimEndInf, 'String', infstr)
            
            % Halt robot if not already halted
            if strcmp(get(D.UI.btnHaltRob, 'Enable'), 'on')
                if (get(D.UI.btnHaltRob, 'Value') ~= 1)
                    set(D.UI.btnHaltRob, 'Value', 1);
                    BtnHaltRob();
                end
            end
            
            % Force update UI
            drawnow;
            
            % Check if rat is out of room
            questdlgAWL(...
                'Take out rat', ...
                'RAT OUT', ...
                'OK', [], [], 'OK', ...
                D.UI.qstDlfPos);
            
            % Force update UI
            drawnow;
            
            % Post NLX event: rat out
            NlxSendCommand(D.NLX.rat_out_evt);
            
            % Tell CS rat is out
            Mat2CS('I', 0)
            
            % Stop recording
            if D.B.rec
                set(D.UI.btnRec,'Value', 0)
                BtnRec(D.UI.btnRec);
            end
            
            % Make sure main loop ends
            D.B.rat_in = false;
            
            % Disable all recording buttons
            set(D.UI.panRec, ...
                'ForegroundColor', D.UI.disabledCol, ...
                'HighlightColor', D.UI.disabledCol)
            set(D.UI.btnAcq, ...
                'Enable', 'off', ...
                'Visible', 'off')
            set(D.UI.btnRec, ...
                'Enable', 'off', ...
                'Visible', 'off')
            set(D.UI.btnICR, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            % Disable extra buttons
            set(D.UI.btnHaltRob, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            set(D.UI.popBulldoze, ...
                'Enable', 'off')
            set(D.UI.btnBulldoze, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            set(D.UI.btnBlockCue, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            set(D.UI.btnAllCue, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            set(D.UI.btnReward, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            set(D.UI.btnClrVT, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            % Disable toggSnd buttons
            set(D.UI.toggSnd, 'Enable', 'off')
            
            % Disable inf panels
            set(D.UI.panSesInf, 'HighlightColor', D.UI.disabledCol)
            set(D.UI.panPerfInf, 'HighlightColor', D.UI.disabledCol)
            set(D.UI.panTimInf, 'HighlightColor', D.UI.disabledCol)
            
            % Stop halt if still active
            if (get(D.UI.btnHaltRob, 'Value') == 1)
                set(D.UI.btnHaltRob, 'Value', 0);
                BtnHaltRob();
            end
            
            % Stop bulldoze if still active
            if (get(D.UI.btnBulldoze, 'Value') == 1)
                set(D.UI.btnBulldoze, 'Value', 0);
                Bulldoze();
            end
            
        end
        
        % SAVE SESSION DATA
        function [] = BtnSave(~, ~, ~)
            
            % Disable save button so only pressed once
            set(D.UI.btnSave, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            
            % Set flag to save at end of main loop
            D.B.do_save = true;
            
        end
        
        % QUIT ALL
        function [] = BtnQuit(~, ~, ~)
            
            % Check if recording and sesion data have been saved but skip
            % if Quit pressed before setup done
            if ~D.UI.save_done && D.B.setup
                % Construct a questdlg with two options
                % Note: based on built in function
                choice = questdlgAWL('!!QUIT WITHOUT SAVING?!!', ...
                    'ABBORT RUN', ...
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
            
            % Print session aborting
            if ~D.UI.save_done
                Update_Console(sprintf('\rABORTING SESSION...\rTime: %s\r', ...
                    datestr(now, 'HH:MM:SS')));
            end
            
            % Disable
            % quit button
            set(D.UI.btnQuit, ...
                'Enable', 'off', ...
                'BackgroundColor', D.UI.disabledCol);
            
            % Stop recording
            if D.B.rec
                set(D.UI.btnRec,'Value', 0)
                BtnRec(D.UI.btnRec);
            end
            
            % Make sure main loop ends
            D.B.rat_in = false;
            
            % Stop halt if still active
            if (get(D.UI.btnHaltRob, 'Value') == 1)
                set(D.UI.btnHaltRob, 'Value', 0);
                BtnHaltRob();
            end
            
            % Stop bulldoze if still active
            if (get(D.UI.btnBulldoze, 'Value') == 1)
                set(D.UI.btnBulldoze, 'Value', 0);
                Bulldoze();
            end
            
            % Tell C# to begin quit
            Mat2CS('Q');
            
            % Set flag
            D.B.do_quit = true;
            
        end
        
        % HALT ROBOT
        function [] = BtnHaltRob(~, ~, ~)
            if (get(D.UI.btnHaltRob, 'Value') == 1)
                % Change backround color and text
                set(D.UI.btnHaltRob, ...
                    'String', 'Halted...', ...
                    'BackgroundColor', D.UI.dfltActiveCol);
                % Tell CS to Halt Robot
                Mat2CS('H', 1);
            else
                % Change backround color and text
                set(D.UI.btnHaltRob, ...
                    'String', 'Halt Robot', ...
                    'BackgroundColor', D.UI.enabledCol);
                % Tell CS to stop halting
                Mat2CS('H', 0);
            end
        end
        
        % BULLDOZE RAT
        function [] = Bulldoze(~, ~, ~)
            
            if (get(D.UI.btnBulldoze, 'Value') == 1)
                
                % Get bulldoze delay
                delStr = D.UI.bullList(get(D.UI.popBulldoze, 'Value'));
                delStr = regexp(delStr, '\d*', 'match');
                D.PAR.bullDel = str2double(delStr{:}{:});
                
                % Change backround color and text
                set(D.UI.btnBulldoze, ...
                    'String', sprintf('Bulldoze (%ds)', D.PAR.bullDel), ...
                    'BackgroundColor', D.UI.dfltActiveCol);
                
                % Tell CS to bulldoze
                Mat2CS('B', D.PAR.bullDel, D.PAR.bullSpeed);
                
            else
                
                % Change backround color and text
                set(D.UI.btnBulldoze, ...
                    'String', 'Ceasefire', ...
                    'BackgroundColor', D.UI.enabledCol);
                
                % Tell CS to stop bulldozing
                Mat2CS('B', 0, 0);
                
            end
        end
        
        % REWARD
        function [] = BtnReward(~, ~, ~)
            
            % Tell CS to trigger reward
            Mat2CS('R', 0);
            
        end
        
        % BLOCK CUE
        function [] = BlockCue(~, ~, ~)
            if (get(D.UI.btnBlockCue, 'Value') == 1)
                % Change backround color and text
                set(D.UI.btnBlockCue, ...
                    'BackgroundColor', D.UI.dfltActiveCol);
                
                % Set main flag
                D.B.do_block_cue = true;
                
                % Set cue maker to not visible
                set(D.UI.cueH(D.I.rot,:), 'Visible', 'off');
                
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
        end
        
        % ALL CUE
        function [] = AllCue(~, ~, ~)
            if (get(D.UI.btnAllCue, 'Value') == 1)
                % Change backround color and text
                set(D.UI.btnAllCue, ...
                    'BackgroundColor', D.UI.dfltActiveCol);
                
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
            D.P.Rat.pos_hist = NaN(60*60*33,2);
            D.P.Rat.vel_hist = NaN(60*60*33,2);
            D.P.Rob.vel_hist = NaN(60*60*33,2);
            
        end
        
        
        
        
        
        
        
        %% ========================== MINOR FUNCTIONS =============================
        
        % --------------------------SEND CS COMMAND--------------------------------
        
        function[] = Mat2CS(id, data1, data2)
            
            % Set mesage data
            if nargin < 3
                data2 = 9999;
            end
            if nargin < 2
                data1 = 9999;
            end
            m2c_dat1 = data1;
            m2c_dat2 = data2;
            
            % Set mesage ID
            m2c_id = id;
            
            % Set flag
            m2c_flag = 1;
            
            % Post command to NLX
            if isfield(D, 'NLX')
                if ~isempty(D.NLX)
                    if NlxAreWeConnected() == 1
                        NlxSendCommand(sprintf(D.NLX.m2cs_evt, id, data1, data2));
                    end
                end
            end
            
            Update_Console(sprintf('\rMat2CS\rm2c_id = %s\rm2c_dat1 = %2.2f\rm2c_dat2 = %2.2f\rTime: %s\r', ...
                id, data1 , data2, datestr(now, 'HH:MM:SS')));
            
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
            D.UI.scl = (1 - D.UI.backroundCol(1)) / (1 - Hi(1,1));
            Hi = 1 - D.UI.scl*(1 - Hi);
            Hi(mask) = D.UI.backroundCol(1);
            Lo = Lo * (D.UI.backroundCol(1)/Lo(1,1));
            D.UI.scl = (1 - D.UI.backroundCol(1)) / (1 - Lo(1,1));
            Lo = 1 - D.UI.scl*(1 - Lo);
            Lo(mask) = D.UI.backroundCol(1);
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
        
        % ---------------------------GET TRACK BOUNDS------------------------------
        function [xbnd, ybnd] = Get_Rad_Bnds(polbnds)
            
            if (length(polbnds) == 2)
                if polbnds(1) > polbnds(2)
                    polbnds = wrapToPi(polbnds);
                end
                radDist = min(2*pi - abs(polbnds(2)-polbnds(1)), abs(polbnds(2)-polbnds(1)));
                nPoints =  round(100 * (radDist/(2*pi))); % 40 pnts per 2*pi
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
        
        % ---------------------------GET TRACK BOUNDS------------------------------
        function [bool_arr] = Check_Rad_Bnds(arr, polbnds)
            
            if any(isnan(arr))
                bool_arr = false(size(arr));
            else
                if polbnds(1) > polbnds(2)
                    polbnds = wrapToPi(polbnds);
                    arr = wrapToPi(arr);
                end
                bool_arr = arr > polbnds(1) & arr < polbnds(2);
            end
            
        end
        
        % ---------------------------SAVE SESSION DATA-----------------------------
        function [] =  Save_Ses_Data()
            
            %% Disconnect from NetCom
            
            Disconnect_NLX()
            
            % Confirm that Cheetah is closed
            choice = questdlgAWL('Close Cheetah', ...
                'CLOSE CHEETAH', ...
                'OK', [], [], 'OK', ...
                D.UI.qstDlfPos);
            drawnow; % force update UI
            % Handle response
            switch choice
                case 'OK'
            end
            
            %% Change NLX recording directory
            
            % Save directory var
            % formt: datestr(clock, 'yyyy-mm-dd_HH-MM-SS', 'local');
            D.DIR.rec = fullfile(D.DIR.nlxSaveTop, D.PAR.ratLab(2:end));
            
            % Make directory if none exists
            if exist(D.DIR.rec, 'dir') == 0
                mkdir(D.DIR.rec);
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
            D.SS_Out_ICR.(D.PAR.ratLab).Total_Time(rowInd) = etime(D.T.ses_end_tim ,D.T.ses_str_tim) / 60;
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
            D.SS_Out_ICR.(D.PAR.ratLab).Pulse_Duration(rowInd) = D.PAR.fdPlsDur;
            D.SS_Out_ICR.(D.PAR.ratLab).Sound_Conditions(rowInd,:) = D.UI.snd;
            D.SS_Out_ICR.(D.PAR.ratLab).Start_Quadrant(rowInd) = D.PAR.ratStrQuad;
            D.SS_Out_ICR.(D.PAR.ratLab).Bulldozings(rowInd) = D.C.bull_cnt;
            D.SS_Out_ICR.(D.PAR.ratLab).Rewards_Standard{rowInd} = sum(D.C.rew_cnt{3});
            D.SS_Out_ICR.(D.PAR.ratLab).Laps_Standard{rowInd} = sum(D.C.lap_cnt{3});
            D.SS_Out_ICR.(D.PAR.ratLab).Notes{rowInd} = '';
            
            % Rotation session specific vars
            if D.PAR.sesCond == 'Rotation'
                D.SS_Out_ICR.(D.PAR.ratLab).Session_Rotation(rowInd) = D.PAR.sesNumICR;
                D.SS_Out_ICR.(D.PAR.ratLab).Rotations_Per_Session(rowInd) = D.C.rot_cnt;
                D.SS_Out_ICR.(D.PAR.ratLab).Rotation_Positions{rowInd}(1:D.C.rot_cnt) = ...
                    D.PAR.rotPos(1:D.C.rot_cnt);
                D.SS_Out_ICR.(D.PAR.ratLab).Laps_Per_Rotation{rowInd} = ...
                    [D.C.lap_cnt{3}, reshape([[D.C.lap_cnt{1}]; [D.C.lap_cnt{2}]], 1, [])];
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
            D.SS_In_All.Pulse_Duration(D.PAR.ratInd) = D.PAR.fdPlsDur;
            D.SS_In_All.Sound_Conditions(D.PAR.ratInd,:) = D.UI.snd;
            
            % Save out data
            SS_Out_ICR = D.SS_Out_ICR;
            save(D.DIR.ioSS_Out_ICR, 'SS_Out_ICR');
            SS_In_All = D.SS_In_All;
            save(D.DIR.ioSS_In_All, 'SS_In_All');
            
            
            % Print saved table data
            Update_Console(sprintf('\rSaved Table Data\rTime: %s\r', ...
                datestr(now, 'HH:MM:SS')));
            
            %% Copy Cheetah file to rat directory
            
            % Get file size
            fiGigs = dir(fullfile(D.DIR.nlxTempTop, D.DIR.recFi));
            fiGigs = sum([fiGigs.bytes])/10^9;
            
            % Print coppied file
            Update_Console(sprintf('\rCopying Cheetah File...\rFile: %s\rDirectory: %s\rSize: %0.4f GB\rTime: %s\r', ...
                D.DIR.recFi, D.DIR.rec, fiGigs, datestr(now, 'HH:MM:SS')));
            
            copyfile(fullfile(D.DIR.nlxTempTop, D.DIR.recFi),fullfile(D.DIR.rec, D.DIR.recFi))
            
            % Set flags
            D.UI.save_done = true;
            
            % Tell CS Matlab session saved
            Mat2CS('Z');
            
            % Highlight Quit button
            set(D.UI.btnQuit, ...
                'ForegroundColor' , D.UI.dfltTxtLightCol, ...
                'BackgroundColor', D.UI.dfltActiveCol);
            drawnow;
            
            % Print save completed
            Update_Console(sprintf('\rSave Completed\rTime: %s\r', ...
                datestr(now, 'HH:MM:SS')));
            
        end
        
    end







%% =======================TOP LEVEL FUNCTIONS==============================

% ---------------------------PRINT TO CONSOLE------------------------------
    function [] = Update_Console(str)
        % Print new info to console
        if exist('consoleText','var')
            consoleText = [str, consoleText];
            disp(str);
            if exist('FigH','var')
                if isfield(D,'UI')
                    if isfield(D.UI, 'editConsole')
                        try
                            set(D.UI.editConsole, ...
                                'String', consoleText);
                            if caughtError
                                set(D.UI.editConsole, ...
                                    'ForegroundColor', [1 0 0], ...
                                    'FontWeight','Bold');
                            end
                            drawnow;
                        catch
                        end
                    end
                end
            end
        end
    end

% ------------------------Disconnect From NetCom---------------------------
    function [] = Disconnect_NLX()
        
        % End NLX polling
        D.B.poll_nlx = false;
        
        if isfield(D, 'NLX')
            if ~isempty(D.NLX)
                if NlxAreWeConnected() == 1
                    
                    % Close VT stream
                    if isfield(D.NLX, 'vt_rat_ent')
                        NlxCloseStream(D.NLX.vt_rat_ent);
                        NlxCloseStream(D.NLX.vt_rob_ent);
                    end
                    
                    % Close event stream
                    if isfield(D.NLX, 'event_ent')
                        NlxCloseStream(D.NLX.event_ent);
                    end
                    
                    % Stop recording and aquisition
                    NlxSendCommand('-StopRecording');
                    % Do not run if triggered before save
                    if D.B.do_save
                        NlxSendCommand('-StopAcquisition');
                    end
                    
                    % Disconnect from the NLX server
                    while NlxAreWeConnected() == 1
                        NlxDisconnectFromServer();
                    end
                    
                    % Show status disconnected
                    if NlxAreWeConnected() ~= 1
                        Update_Console(sprintf('\rDisconnected From DigitalLynx SX\rIP: %s\rTime: %s\r', ...
                            D.NLX.IP, datestr(now, 'HH:MM:SS')));
                    end
                end
            end
            
        end
    end

% -----------------------------Disconnect ALL-------------------------------
    function [] = Disconnect_All()
        
        if exist('D','var')
            
            Update_Console(sprintf('\rEXITING.......\rTime: %s\r', ...
                datestr(now, 'HH:MM:SS')));
            
            % Disconnect from AC computer
            if isfield(D, 'AC')
                if ~isempty(D.AC)
                    if exist('tcpIP', 'var')
                        if isa(tcpIP, 'tcpip')
                            if isvalid(tcpIP)
                                % Close AC connection
                                D.AC.data = zeros(1, length(D.AC.data));
                                % keep connection
                                D.AC.data(1) = 1;
                                % post to AC computer
                                try fwrite(tcpIP,D.AC.data,'int8'); catch; end;
                                % pause to allow image to close
                                pause(0.01);
                                % close all
                                D.AC.data = zeros(1, length(D.AC.data));
                                % post to AC computer
                                try fwrite(tcpIP,D.AC.data,'int8'); catch; end;
                                % close AC computer connection
                                try fclose(tcpIP); catch; end;
                                
                                % Show status disconnected
                                Update_Console(sprintf('\rDisconnected From AC Computer\rIP: %s\rTime: %s\r', ...
                                    D.AC.IP, datestr(now, 'HH:MM:SS')));
                            end
                        end
                    end
                end
            end
            try fclose(tcpIP); catch; end;
            try delete(tcpIP); catch; end;
            try clear tcpIP; catch; end;
            
            % Disconnect from NetCom
            Disconnect_NLX()
            
        end
    end

% ------------------------------FORCE QUIT---------------------------------
    function [] = ExitCallback(~, ~, ~)
        
        % Dont run if Quit button was used to ext
        if exist('D','var')
            if isfield(D, 'UI')
                if isfield(D.UI, 'btnQuit')
                    if D.B.do_quit
                        return;
                    end
                end
            end
        end
        
        % Trigger GUI exit
        shouldExit = true;
        drawnow;
        % Disconnect all
        Disconnect_All();
        return;
    end






end