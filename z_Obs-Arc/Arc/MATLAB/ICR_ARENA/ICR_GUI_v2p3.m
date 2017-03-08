function[] = ICR_GUI

%% ======================== INITIALIZE GLOBALS ============================

global FigH; % UI figure handle
global D; % Main data struct
global shouldExit; % bool to exit
global m2c_id; % message out to CS
global c2m_id; % message in from CS
global m2c_dat1; % data out to CS
global m2c_dat2; % data out to CS
global m2c_flag; % new data flag out to CS
global consoleText; % console text
global tcpIP; % server tcpip object

% Run main function
FigH = figure('Visible', 'Off');
RunScript();






%% =========================== MAIN SCRIPT ================================

    function[] = RunScript()
        
        % --------------------------SET PARAMETERS-------------------------
        
        % Set globals
        shouldExit = false;
        c2m_id = ' ';
        consoleText = ' ';
        
        % Print that script is running
        Update_Console(sprintf('\nRUNNING ICR_BEHAVIOR.m\nTime: %s\n', ...
            datestr(now, 'HH:MM:SS')));
        
        % Feeder paramiters
        % boundary before and after rew feeder (deg)
        D.PAR.feedBnds = [10,5];
        
        % NLX paramiters
        D.PAR.polRate = 0.1; % poll fs (sec)
        
        % Directories
        % top dir
        D.DIR.top = regexp(pwd,'.*(?=\\MATLAB)','match');
        D.DIR.top = D.DIR.top{:};
        D.DIR.ioTop = fullfile(D.DIR.top,'MATLAB\IOfiles');
        % i/o dirs
        D.DIR.ssSD = 'SessionData';
        D.DIR.oppSD = 'Operational';
        D.DIR.trkBndsFi = 'track_bounds(new_cam).mat';
        % temp dir
        D.DIR.cheetTempTop = 'C:\CheetahData\Temp';
        % save dir
        D.DIR.cheetSaveTop = 'E:\BehaviorPilot';
        % vs exe dir
        D.DIR.vsEXE_Fi = 'MeStreamNLX.exe';
        D.DIR.vsEXE = fullfile(D.DIR.top,'VisualStudio\MeStreamNLX\bin\x86\Release');
        
        %---------------------Important variable formats---------------------------
        %...........................D.UI.snd....................................
        %   val 1 = White Noise [true, false]
        %   val 2 = Reward Tone [true, false]
        %...........................D.AC.data......................................
        %   val 1 = conection [0, 1], [no, yes]
        %   val 2 = display image [0, 1, 2], [Close all, 0-deg, 40-deg]
        %   val 3 = rotation direction [-1, 1], [ACW, CW]
        %   val 4 = sound state [0, 1], [no sound, sound]
        %...........................D.CS.m2c_id...................................
        %           'S', ... % start session
        %           'Q', ... % quit session
        %           'M', ... % move to position
        %           'R', ... % dispense reward
        %           'H', ... % halt movement
        %           'I', ... % rat in
        %           'N', ... % matlab not loaded
        %           'L', ... % matlab loaded
        %--------------------------------------------------------------------------
        
        
        % ---------------------------SETUP & RUN----------------------------------
        
        % Run variable setup code
        SF_Var_Setup();
        
        % Run UI setup code
        SF_UI_Setup();
        
        % Run AC setup code
        SF_AC_Setup();
        
        % Run NLX setup code
        SF_NLX_Setup();
        
        % Tell CS Matlab is connected to Cheetah
        CS_Send(8);
        
        while ~shouldExit
            
            %% -----------------------CHECK FOR UI SETUP---------------------------
            if ~D.B.setup
                pause(0.01);
                drawnow;
                
            elseif ~D.B.poll_nlx
                
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
            end
            
            
            %% ---------------------------POLL NETCOM------------------------------
            
            if D.B.poll_nlx
                
                % Pause for NetCom's buffers.
                loop_tim = etime(clock, D.T.last_poll_tim);
                if (loop_tim < D.PAR.polRate)
                    pause(D.PAR.polRate - loop_tim);
                end
                D.T.last_poll_tim = clock;
                
                % Wait to send CS move command
                if etime(clock, D.C.moveDel) > 2.5 &&  ~D.C.doneMove
                    % Send C# command to move robot to start quad or reward loc
                    if ~D.B.manual_ses
                        CS_Send(3, mean(D.UI.strQuadBnds));
                    else
                        CS_Send(3, mean(D.UI.rewBnds(D.I.rot,:)));
                    end
                    D.C.doneMove = true;
                end
                
                % PROCESS VT
                
                % Run VT processing code
                % Rat
                SF_VT_Proc('Rat');
                % Robot
                SF_VT_Proc('Rob');
                % Update_Console(sprintf('Rat: %2.2f, Rob: %2.2f\n', D.P.Rat.rad(end), D.P.Rob.rad(end)));
                
                % CHECK IF RAT IN
                
                % Run rat in check code
                if ~D.B.ratIn
                    SF_Rat_In_Check();
                end
                
                % -----------------------ONCE RAT IN-----------------------------------
                
                if D.B.ratIn
                    
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
            drawnow;
            
        end
        
        % Run Disconect_All() to exit script
        Disconect_All();
    end

% =========================================================================






%% ========================= SETUP FUNCTIONS ==============================

% ------------------------------VAR SETUP----------------------------------

    function [] = SF_Var_Setup()
        
        % Load data tables
        T = load(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_In_All.mat'));
        D.SS_In_All = T.SS_In_All;
        T = load(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_Out_ICR.mat'));
        D.SS_Out_ICR = T.SS_Out_ICR;
        clear T;
        
        % Matlab 2 C# vars
        % id chars
        D.CS.m2c_id = [ ... % prefix giving masage id
            'S', ... % start session
            'Q', ... % quit session
            'M', ... % move to position
            'R', ... % dispense reward
            'H', ... % halt movement
            'I', ... % rat in
            'N', ... % matlab not loaded
            'L', ... % matlab loaded
            ];
        
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
        
        % Get position data
        % load track bound data
        S = load(fullfile(D.DIR.ioTop, D.DIR.oppSD, D.DIR.trkBndsFi));
        % track bound radius
        D.PAR.R = S.R;
        % track bound center X
        D.PAR.XC = S.XC;
        % track bound center Y
        D.PAR.YC = S.YC;
        clear S;
        % Robot guard dist
        D.PAR.guardDist = 0.05;
        % PID setPoint
        D.PAR.setPoint = 0.81;
        % Feeder dist from rob tracker
        D.PAR.feedDist = 1;
        
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
        D.C.rec_cnt = 0;
        
        % Initialize boolean variables
        % plot HD
        D.B.doPlotHD = false;
        % manual feed sesion
        D.B.manual_ses = false;
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
        % track lap times
        D.T.lap_tim = 0;
        % track feeder event timing
        D.T.fd_str_tim = 0;
        % track feeder off event timing
        D.T.fd_end_tim = 0;
        
        % Initialize index vars
        % current wall image index
        D.I.rot = 1;
        % feeder index
        D.I.feed_ind = [1, 2];
        % current lap quadrant for lap track
        D.I.lap_hunt_ind = 1;
        
        % Initialize arrays
        % angle data sample length (sec * vt samp rate)
        D.A.rv_nsmp = 1*30;
        % store pos data for track (c1) and heading angle (c2)
        D.A.rv_samples = NaN(D.A.rv_nsmp,2);
        
    end






% -------------------------------UI SETUP-----------------------------------

    function[] = SF_UI_Setup()
        
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
        
        % Set figure
        set(FigH,...
            'Name', 'ICR Behavior Pilot', ...
            'DeleteFcn', {@BtnExit}, ...
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
        D.UI.fg_pos = [ ...
            sc1(3) + (sc2(3)-fg_wh(1))/2, ...
            (sc2(4) - fg_wh(2))/2, ...
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
        
        % Main feature colors
        % color for rotation conditions
        D.UI.rotCol = [0, 0, 1;0, 0.5, 0];
        % color of marker for rat pos
        D.UI.ratPosHistCol = [0, 0, 0];
        % color of marker for current rat pos
        D.UI.ratPosNowCol = [0.5, 0, 0];
        % color of marker for current rob pos
        D.UI.robPosCol = [1, 0.56, 0];
        
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
        D.UI.condList = {''; 'Manual Feed'; 'ICRb Session'; 'Rotation Session'};
        
        % popRewDel
        D.UI.delList = {''; '0.1 '; '1.0 '; '2.0'; '3.0'};
        
        % radCue
        D.UI.cueFeed = categorical({'<undefined>'}, D.PAR.catCueCond);
        
        % radSnd
        % labels
        D.UI.sndLabs = [...
            {'White Noise'}, ...
            {'Reward Tone'}];
        % state strings
        D.UI.sndState = [{'Off'},{'On'}];
        % for storing sound settings
        D.UI.snd = false(1,2);
        
        % btnAcq and btnRec
        % run Btn_Col function
        [D.UI.radBtnCmap] = Btn_Col();
        
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
        botm = pos_ht_dflt*3.5 + obj_gap_ht*2;
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
            'Callback',{@RadCue}, ...
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
            'Callback',{@RadCue}, ...
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
            'Callback',{@RadCue}, ...
            'String','None', ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.backroundCol, ...
            'ForegroundColor', D.UI.enabledCol, ...
            'FontName','Courier New', ...
            'FontWeight','Bold', ...
            'FontSize', text2_font_sz(1), ...
            'Value',0);
        
        % SOUND BUTTON PANEL
        offset = 0.05;
        botm = pos_ht_dflt*2.5 + obj_gap_ht;
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
        botm = botm + text2_font_sz(2)*0.75;
        wdth = pos_wd_dflt/2.25;
        pos = [pos_lft_dflt + offset, botm, wdth, text2_font_sz(2)*1.1];
        D.UI.radSnd(1) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@RadSnd}, ...
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
        pos = [pos_lft_dflt + offset + wdth, botm, wdth, text2_font_sz(2)*1.1];
        D.UI.radSnd(2) = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panStup, ...
            'Callback',{@RadSnd}, ...
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
        botm = pos_ht_dflt*3 + obj_gap_ht + text1_font_sz(2);
        wdth = pos_wd_dflt*0.4;
        pos = [pos_lft_dflt + offst, botm, wdth, text1_font_sz(2)*1.2];
        D.UI.btnAcq = uicontrol('Style','radiobutton', ...
            'Parent',D.UI.panRec, ...
            'UserData', 0, ...
            'Enable', 'On', ...
            'Visible', 'On', ...
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
            'Enable', 'On', ...
            'Visible', 'On', ...
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
        D.UI.btnSesDone = uicontrol('Style','push', ...
            'Parent', D.UI.panRec, ...
            'Enable', 'Off', ...
            'String','DONE', ...
            'Callback', {@BtnSesDone}, ...
            'UserData', 0, ...
            'Units','Normalized', ...
            'Position', pos, ...
            'BackgroundColor', D.UI.dfltBtnLightCol, ...
            'ForegroundColor', D.UI.dfltTxtLightCol, ...
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
            'FontSize', 7, ...
            'FontName','Monospaced', ...
            'Max', 1000, ...
            'Enable','inactive',...
            'String',consoleText);
        
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
            'Parent',FigH, ...
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
        
        % PULSE FEEDER BUTTON
        pos = [pos(1), pos(2)+pos(4), pos(3), pos(4)];
        D.UI.btnPlsFd = uicontrol('Style','push', ...
            'Parent',FigH, ...
            'String','Pulse Feeder', ...
            'Callback', {@BtnPlsFd}, ...
            'Enable', 'Off', ...
            'Units','Normalized', ...
            'Position', pos, ...
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
        botm = botm - 6*text_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, text_font_sz(2)*5];
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
        % rewards per dropdown
        botm = botm - dd_font_sz(2);
        pos = [pos_lft_dflt, botm, pos_wd_dflt, dd_font_sz(2)];
        D.UI.popRewPerRot = uicontrol('Style','popupmenu', ...
            'Parent',FigH, ...
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
            'Parent',FigH, ...
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
        % time stop info
        botm = 4*D.UI.pxl2norm_y;
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
        
        %% ========================================================================
        
        
        %% MAKE FIGURE VISIBLE
        
        % Set position
        movegui(FigH,'southeast')
        % Bring UI to top
        uistack(FigH, 'top')
        drawnow;
        % Make visible
        set(FigH, 'Visible', 'On');
        
    end






% -------------------------------AC SETUP----------------------------------

    function [] = SF_AC_Setup()
        
        %% Setup communication with ARENACONTROLLER
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
        
        % Establish connection
        % will loop here until D.AC.data(1) established
        fopen(tcpIP);
        
        % Print that AC computer is connected
        Update_Console(sprintf('\nConnected To AC Computer\nIP: %s\nTime: %s\n', ...
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
        D.NLX.port_out = '0'; % output port for feeders and audio
        D.NLX.port_in1 = '1'; % input port for feeders and audio
        D.NLX.port_in2 = '2'; % input port for photo transducers
        
        % Event labels
        % SOUND
        D.NLX.snd_air_evt_lab = [ ...
            {'TTL_Sound_Right_On'}, ...
            {'TTL_Sound_Left_On'}];
        
        % INPUT PIN/BIT
        % audio channels
        D.NLX.snd_rt_bit = 2;
        D.NLX.snd_lft_bit = 3;
        
        % Index cammand bype string
        % SOUND
        D.NLX.arduino_snd_air_bit_ind = ...
            [47 - D.NLX.snd_rt_bit, ...
            47 - D.NLX.snd_lft_bit];
        
        % PHOTO TRANSDUCER
        
        % input pins % 1 in 24 chance I guessed these correctly
        D.NLX.west_bit = '3'; % dec = 64
        D.NLX.north_bit = '2'; % dec = 16
        D.NLX.east_bit = '1'; % dec = 4
        D.NLX.south_bit = '0'; % dec = 1
        
        % Rat in event command strings
        D.NLX.rat_in_evt{1} = '-PostEvent Post_Rat_In 201 0';
        D.NLX.rat_in_evt{2} = '-PostEvent Post_Rat_Out 202 0';
        
        % Feeder cue event command strings
        D.NLX.feed_cue_evt{1} = '-PostEvent Post_Feeder_Cue_On 203 0';
        D.NLX.feed_cue_evt{2} = '-PostEvent Post_Feeder_Cue_Off 204 0';
        
        % ICR event command strings
        D.NLX.rot_evt{1} = '-PostEvent Post_Rotation_0_Deg 205 0';
        D.NLX.rot_evt{2} = '-PostEvent Post_Rotation_40_Deg 206 0';
        
        % Reversal event command strings
        D.NLX.rev_evt{1} = '-PostEvent Post_Reversal_Start 207 0';
        D.NLX.rev_evt{2} = '-PostEvent Post_Reversal_End 208 0';
        
        % Session end command string
        D.NLX.ses_end_evt = '-PostEvent Post_Session_End 211 0';
        
        %% CONNECT TO NETCOM
        
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
                    Update_Console(sprintf('\nConnected To NLX\nIP: %s\nTime: %s\n', ...
                        D.NLX.IP, datestr(now, 'HH:MM:SS')));
                    %Identify this program to the server.
                    NlxSetApplicationName('ICR_GUI');
                end
            end
        else
            Update_Console(sprintf('\nAlready Connected to NLX\nIP: %s\nTime: %s\n', ...
                D.NLX.IP, datestr(now, 'HH:MM:SS')));
        end
        
        %% CONFIGURE DIGITAL IO
        if NlxAreWeConnected() && ~shouldExit
            
            % Pair Cube headstage (Should be commented out unless router has been unpluged)
            %NlxSendCommand('-SendLynxSXCommand AcqSystem1 -InitWHSPairing 30')
            % Turn on Cube LEDs
            NlxSendCommand('-SendLynxSXCommand AcqSystem1 -WHSSetTrackingLED 1 1');
            
            % Open the data stream for the VT acquisition entity.  This tells Cheetah to begin %streaming
            % data for the VT acq ent.
            NlxOpenStream(D.NLX.vt_rat_ent);
            NlxOpenStream(D.NLX.vt_rob_ent);
            NlxOpenStream(D.NLX.event_ent);
            %             [Timestamps, EventIDs, TTLs, Extras, EventStrings, Header] =
            %              Nlx2MatEV('test.nev', [1 1 1 1 1], 1, 1, [] );
            
            % Get current folder name
            dirs = dir(D.DIR.cheetTempTop);
            D.DIR.recFi = dirs([dirs.datenum] == max([dirs.datenum])).name;
            
            % Set port directions
            NlxSendCommand(['-SetDigitalIOportDirection ', D.NLX.DevName, ' ', D.NLX.port_out, ' Output']);
            NlxSendCommand(['-SetDigitalIOportDirection ', D.NLX.DevName, ' ', D.NLX.port_in2, ' Input']);
            
            % Enable digital io events
            NlxSendCommand(['-SetDigitalIOEventsEnabled ', D.NLX.DevName, ' ', D.NLX.port_in2, ' True']);
            NlxSendCommand(['-SetDigitalIOEventsEnabled ', D.NLX.DevName, ' ', D.NLX.port_out, ' True']);
            
            % Send output port command to set all TTL to zero
            NlxSendCommand(['-SetDigitalIOportString ', D.NLX.DevName, ' ', D.NLX.port_out, ' "00000000"']);
            
            % Set TTL Event strings
            % audio channels
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.snd_rt_bit), ' ', D.NLX.snd_air_evt_lab{1}]);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in1, ' ', num2str(D.NLX.snd_lft_bit), ' ', D.NLX.snd_air_evt_lab{2}]);
            % photo transdicers
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in2, ' ', D.NLX.north_bit, ' TTL_North_On']);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in2, ' ', D.NLX.east_bit, ' TTL_East_On']);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in2, ' ', D.NLX.south_bit, ' TTL_South_On']);
            NlxSendCommand(['-SetNamedTTLEvent ', D.NLX.DevName, ' ', D.NLX.port_in2, ' ', D.NLX.west_bit, ' TTL_West_On']);
            
            %% Start aquisition
            % Run BtnAcq
            set(D.UI.btnAcq,'Value', 1)
            BtnAcq(D.UI.btnAcq);
            
        end
    end





% ----------------------------FINISH SETUP---------------------------------
    function [] = SF_Finish_Setup()
        
        if ~shouldExit
            
            %% Send setup command to robot
            CS_Send(1, ...
                double(~D.B.manual_ses), ... % session cond
                double(D.UI.snd(2))); % reward tone
            
            %% Updatea and send AC.data values
            
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
            
            % Disable all setup buttons but not sound buttons
            set(D.UI.panStup, 'ForegroundColor', D.UI.disabledCol, ...
                'HighlightColor',D.UI.disabledCol)
            set(D.UI.popRat, 'Enable', 'Off')
            set(D.UI.popCond, 'Enable', 'Off')
            set(D.UI.popRwDl, 'Enable', 'Off')
            set(D.UI.radCue, 'Enable', 'Off')
            set(D.UI.radSnd, 'Enable', 'Off')
            set(D.UI.btnStupDn, 'Enable', 'Off', ...
                'BackgroundColor', D.UI.disabledCol)
            
            % Enable acq record buttons
            set(D.UI.panRec, ...
                'ForegroundColor', D.UI.enabledCol, ...
                'HighlightColor', D.UI.enabledCol)
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
            
            %% Plot bounds data
            
            % Plot start quadrant
            [xbnd, ybnd] =  Get_Trk_Bnds( D.UI.strQuadBnds);
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
                Get_Trk_Bnds( D.UI.rewBnds(1,:));
            D.UI.ptchFdH(1) = ...
                patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                [ybnd(1,:),fliplr(ybnd(2,:))], ...
                D.UI.rotCol(1,:), ...
                'FaceAlpha', 0.25, ...
                'Parent', D.UI.axH(1));
            
            % Plot 40-deg feeder bounds
            % set alpha to 0 (transparent)
            [xbnd, ybnd] =  ...
                Get_Trk_Bnds( D.UI.rewBnds(2,:));
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
                        Get_Trk_Bnds( D.UI.rewRstBnds(z_bnd,:,z_fd));
                    D.UI.ptchFdRstH(z_bnd,z_fd) = ...
                        patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                        [ybnd(1,:),fliplr(ybnd(2,:))], ...
                        [0.5,0.5,0.5], ...
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
            
            %% Set poling vars
            D.C.moveDel = clock;
            D.C.doneMove = false;
            % Set to start polling NLX
            D.B.poll_nlx = true;
            
            drawnow;
            % Print setup completed
            Update_Console(sprintf('\nSetup Completed\nTime: %s\n', ...
                datestr(now, 'HH:MM:SS')));
            
        end
        
    end






% ----------------------------RAT IN CHECK---------------------------------

    function [] = SF_Rat_In_Check()
        
        % Keep checking if rat is in the arena
        D.B.track_strqd = ...
            D.P.Rat.rad >= min(D.UI.strQuadBnds) & ...
            D.P.Rat.rad <= max(D.UI.strQuadBnds);
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
            NlxSendCommand(D.NLX.rat_in_evt{1});
            
            % Tell CS rat is in
            CS_Send(6);
            
            % Start tracking lap times
            D.T.lap_tim = clock;
            
        end
        
    end






% =========================================================================






%% ======================== ONGOING FUNCTIONS =============================

% ------------------------------PROCESS VT---------------------------------

    function [] = SF_VT_Proc(fldStr)
        
        %% GET VT DATA
        
        % Get NXL vt data and reformat data with samples in column vectors
        if strcmp(fldStr, 'Rat')
            [~, ~, vtPos, vtHD, vtNRecs, ~] = NlxGetNewVTData(D.NLX.vt_rat_ent);
            D.P.(fldStr).xy_pos = reshape(single(vtPos),2,[])';
            D.P.(fldStr).hd_deg = vtHD';
        else
            [~, ~, vtPos, ~, vtNRecs, ~] = NlxGetNewVTData(D.NLX.vt_rob_ent);
            D.P.(fldStr).xy_pos = reshape(single(vtPos),2,[])';
        end
        D.P.(fldStr).recs = vtNRecs;
        
        %% TRANSFORM X/Y AND RAD
        
        % Save x/y pos samples in seperate vars
        D.P.(fldStr).x = D.P.(fldStr).xy_pos(:,1);
        D.P.(fldStr).y = D.P.(fldStr).xy_pos(:,2);
        
        % Rescale y as VT data is compressed in y axis
        D.P.(fldStr).y = D.P.(fldStr).y*11/10;
        
        % Get nnormalized pos data
        D.P.(fldStr).x_norm = (D.P.(fldStr).x-D.PAR.XC)./D.PAR.R;
        D.P.(fldStr).y_norm = (D.P.(fldStr).y-D.PAR.YC)./D.PAR.R;
        
        % Get position in radians
        [D.P.(fldStr).rad,D.P.(fldStr).roh] = cart2pol(D.P.(fldStr).x_norm, D.P.(fldStr).y_norm);
        
        % Convert radians between [0, 2*pi]
        D.P.(fldStr).rad = wrapTo2Pi(D.P.(fldStr).rad);
        
        % Flip radian values to acount for inverted y values from Cheetah
        D.P.(fldStr).rad = abs(D.P.(fldStr).rad - 2*pi);
        
        % Exclude outlyer values > || < track bounds plus 5 cm
        excInd = D.P.(fldStr).roh > 1.1 | D.P.(fldStr).roh < (1 - (D.UI.trkWdt+5)/D.UI.arnRad);
        % Exclude valuse based on diff
        radDif = [0;diff(D.P.(fldStr).rad)];
        excInd = excInd & (radDif > (150/33)*((2 * pi)/(140 * pi)));
        D.P.(fldStr).rad(excInd) = NaN;
        D.P.(fldStr).roh(excInd) = NaN;
        
        % Recalculate cartisian values
        [D.P.(fldStr).x, D.P.(fldStr).y] = pol2cart(D.P.(fldStr).rad,D.P.(fldStr).roh);
        D.P.(fldStr).x =  D.P.(fldStr).x.*D.PAR.R + D.PAR.XC;
        D.P.(fldStr).y =  D.P.(fldStr).y.*D.PAR.R + D.PAR.YC;
        
        %% GET SETPOINT, FEEDER POS AND GUARD POS
        
        if strcmp(fldStr, 'Rob')
            
            % Get setpoint in radians
            D.P.Rob.setRad = D.P.Rob.rad(end) - D.PAR.setPoint;
            if D.P.Rob.setRad < 0
                D.P.Rob.setRad = 2*pi + D.P.Rob.setRad;
            end
            
            % Get feeder pos
            D.P.Rob.feedRad = D.P.Rob.rad(end) - D.PAR.feedDist;
            if D.P.Rob.feedRad < 0
                D.P.Rob.feedRad = 2*pi + D.P.Rob.feedRad;
            end
            
            % Get guard pos
            D.P.Rob.guardRad = D.P.Rob.rad(end) - D.PAR.guardDist;
            if D.P.Rob.guardRad < 0
                D.P.Rob.guardRad = 2*pi + D.P.Rob.guardRad;
            end
            
        end
        
        %% TRANSFORM HD
        
        if D.B.doPlotHD && strcmp(fldStr, 'Rat')
            % Flip HD angle
            D.P.Rat.hd_deg = abs(single(D.P.Rat.hd_deg)-360);
            
            % Shift HD by 90 deg to align with track/matlab 0 deg
            D.P.Rat.hd_deg = D.P.Rat.hd_deg + 90;
            if any(D.P.Rat.hd_deg > 360)
                D.P.Rat.hd_deg(D.P.Rat.hd_deg > 360)  = D.P.Rat.hd_deg(D.P.Rat.hd_deg > 360) - 360;
            end
            
            % Convert HD to radians
            D.P.Rat.hd_rad = deg2rad(D.P.Rat.hd_deg);
            
            % Update HD angle samples
            % Note: this mainains a continuous sample of HD samples
            rv_str_ind = D.I.rv_smp_ind + 1;
            rv_end_ind = rv_str_ind + vtNRecs-1;
            
            % Keep only max samples
            if vtNRecs >= D.A.rv_nsmp
                
                % Get sample indeces
                % start ind
                rv_str_ind = 1;
                % end ind
                rv_end_ind = D.A.rv_nsmp;
                
                % Get radian values
                % track
                D.A.rv_samples(:,1) = D.P.Rat.rad(1:D.A.rv_nsmp);
                % hd
                D.A.rv_samples(:,2) = D.P.Rat.hd_rad(1:D.A.rv_nsmp);
                
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
            if vtNRecs <= D.A.rv_nsmp
                
                % Get radian values
                % track
                D.A.rv_samples(rv_str_ind:rv_end_ind,1) = D.P.Rat.rad;
                % hd
                D.A.rv_samples(rv_str_ind:rv_end_ind,2) = D.P.Rat.hd_rad;
                
            end
            
            % Update index
            D.I.rv_smp_ind = D.I.rv_smp_ind + vtNRecs;
            % cap to max sample range
            if D.I.rv_smp_ind > D.A.rv_nsmp;
                D.I.rv_smp_ind = D.A.rv_nsmp;
            end
        end
        
    end







% -----------------------ROTATION TRIGGER CHECK----------------------------

    function [] = SF_Rotation_Trig_Check()
        
        % Check if rotation trigger button has been pressed
        if ~shouldExit
            if get(D.UI.btnICR, 'UserData')
                
                % Check if rat in rotation bounds
                D.B.track_rot = ...
                    D.P.Rat.rad >= min(D.UI.rotBndNext) & ...
                    D.P.Rat.rad <= max(D.UI.rotBndNext);
                
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
                    fwrite(tcpIP,D.AC.data,'int8');
                    
                    % Post NLX event: rotaion *deg
                    NlxSendCommand(D.NLX.rot_evt{D.I.rot});
                    
                    % Change plot marker size
                    % active feeder
                    if ~shouldExit
                        set(D.UI.fdH(D.I.rot), ...
                            'MarkerSize', 15, ...
                            'LineWidth', 2);
                    end
                    % inactive feeder
                    if ~shouldExit
                        set(D.UI.fdH([1, 2] ~=  D.I.rot), ...
                            'MarkerSize', 10, ...
                            'MarkerEdgeColor', [0, 0, 0], ...
                            'LineWidth', 1)
                    end
                    
                    % Delete old patches
                    if isfield(D.UI, 'ptchRtTrgH')
                        delete(D.UI.ptchRtTrgH)
                    end
                    
                    % Plot rotation pos mark
                    r = D.P.Rat.rad(D.B.track_rot);
                    [x,y] = Get_Trk_Bnds([r(1),r(1)]);
                    if ~shouldExit
                        plot(x,y, ...
                            'Color', D.UI.rotCol(D.I.rot,:), ...
                            'LineWidth', 3, ...
                            'Parent',D.UI.axH(1));
                    end
                    
                    % Change reward bounds patch
                    % active feeder
                    if ~shouldExit
                        set(D.UI.ptchFdH(D.I.rot), ...
                            'FaceAlpha', 0.5, ...
                            'EdgeAlpha', 1)
                    end
                    % inactive feeder
                    if ~shouldExit
                        set(D.UI.ptchFdH([1, 2] ~=  D.I.rot), ...
                            'FaceAlpha', 0, ...
                            'EdgeAlpha',0)
                    end
                    
                    % Change button features
                    if ~shouldExit
                        set(D.UI.btnICR,'string', D.UI.btnICRstr{rotLast}, ...
                            'BackgroundColor', D.UI.rotCol(rotLast,:), ...
                            'UserData', false);
                    end
                    
                    % Change reward reset
                    % reset reward bool
                    D.B.track_rew_reset = true(3,1);
                    % remove reward reset patch
                    if ~shouldExit
                        set(D.UI.ptchFdRstH(:,rotLast), ...
                            'FaceAlpha', 0, ...
                            'EdgeAlpha', 0);
                    end
                    
                    % Change session info font weight
                    % active feeder
                    if ~shouldExit
                        set(D.UI.txtPerfInf(D.I.rot), 'FontWeight', 'Bold')
                    end
                    % inactive feeder
                    if ~shouldExit
                        set(D.UI.txtPerfInf([1, 2] ~=  D.I.rot), 'FontWeight', 'Light')
                    end
                    
                    % Add to roation counter
                    D.C.rot_cnt = D.C.rot_cnt + 1;
                    
                    % Add new lap counter
                    D.C.lap_cnt{D.I.rot} = [D.C.lap_cnt{D.I.rot}, 0];
                    D.C.rew_cnt{D.I.rot} = [D.C.rew_cnt{D.I.rot}, 0];
                    
                    
                end
                
            end
            
        end
        
    end







% -------------------------REWARD FEEDER CHECK-----------------------------

    function [] = SF_Reward_Feeder_Check()
        
        % Check if rat in feeder bounds
        D.B.track_fd = ...
            D.P.Rat.rad >= min(D.UI.rewBnds(D.I.rot,:)) & ...
            D.P.Rat.rad <= max(D.UI.rewBnds(D.I.rot,:));
        
        if ...
                any(D.B.track_fd) && ...
                D.C.rec_cnt < 30*(D.PAR.rewDel+1)
            
            % Get count of in-bounds samples
            D.C.inbnd_cnt = D.C.inbnd_cnt + sum(D.B.track_fd);
            
            
            % Add to record counter
            D.C.rec_cnt = D.C.rec_cnt + D.P.Rat.recs;
            
            if ...
                    D.C.rec_cnt >= 30*D.PAR.rewDel && ...
                    D.C.inbnd_cnt >= 30*D.PAR.rewDel && ...
                    all(D.B.track_rew_reset)
                
                % Baloon and set color to red wile feeder is active
                if ~shouldExit
                    set(D.UI.feedPltNow, ...
                        'MarkerFaceColor', [1,0,0], ...
                        'MarkerSize', 30);
                end
                
                % Lighten reward feeder patch
                if ~shouldExit
                    set(D.UI.ptchFdH(D.I.rot), ...
                        'FaceAlpha', 0.25, ...
                        'EdgeAlpha', 1)
                end
                
                % Force update GUI
                if ~shouldExit
                    drawnow;
                end
                
                % Check if this reward was cued
                if ...
                        (D.UI.cueFeed == 'All' || ...
                        (D.UI.cueFeed == 'Half' && D.B.cue_next))
                    
                    % Post NLX event: cue off
                    NlxSendCommand(D.NLX.feed_cue_evt{2});
                    
                    % Dont cue next lap
                    if D.UI.cueFeed == 'Half'
                        D.B.cue_next = false;
                    end
                end
                
                % Check if next reward is cued
                if ...
                        (D.UI.cueFeed == 'All' || ...
                        (D.UI.cueFeed == 'Half' && ~D.B.cue_next))
                    
                    % Post NLX event: cue off
                    NlxSendCommand(D.NLX.feed_cue_evt{1});
                    
                    % Tell CS to reward now
                    CS_Send(4, 0);
                    
                    % Cue next lap
                    if D.UI.cueFeed == 'Half'
                        D.B.cue_next = true;
                    end
                end
                
                % Set reset patches to visible
                if ~shouldExit
                    set(D.UI.ptchFdRstH(:, D.I.rot), ...
                        'FaceAlpha', 0.1, ...
                        'EdgeAlpha', 1);
                end
                
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
                D.C.rec_cnt = 0;
                
                % Return feeder marker color to normal
                if ~shouldExit
                    set(D.UI.feedPltNow, ...
                        'MarkerFaceColor', D.UI.rotCol(D.I.rot,:), ...
                        'MarkerSize', 15);
                end
                
            end
        else
            
            % Reinitialize counters
            D.C.inbnd_cnt = 0;
            D.C.rec_cnt = 0;
            
        end
        
    end






% ----------------------------LAP CHECK------------------------------------

    function [] = SF_Lap_Check()
        
        % Check if rat in lap quad bound
        track_quad = ...
            D.P.Rat.rad >= min(D.UI.lapBnds(D.I.lap_hunt_ind, :)) & ...
            D.P.Rat.rad <= max(D.UI.lapBnds(D.I.lap_hunt_ind, :));
        
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
            if ~shouldExit
                Hind = ~isnan(D.UI.vtPltH(:));
            end
            if ~shouldExit
                set(D.UI.vtPltH(Hind), 'MarkerEdgeColor', [0.75,0.75,0.75]);
            end
            if ~shouldExit
                set(D.UI.vtPltH(D.UI.vtPltHrev), 'MarkerEdgeColor', [1,0.75,0.75]);
            end
            
            % Update lap time dropdown
            D.UI.lapTim = [D.UI.lapTim; ...
                {sprintf('%d:    %1.1f', sum([D.C.lap_cnt{:}]), etime(clock,D.T.lap_tim))}];
            if ~shouldExit
                set(D.UI.popLapTim, 'String', D.UI.lapTim);
            end
            % reset timer
            D.T.lap_tim = clock;
            
        end
        
    end






% ------------------------REWARD RESET CHECK-------------------------------

    function [] = SF_Reward_Reset_Check()
        
        % Check if rat is in any quad
        track_quad = ...
            arrayfun(@(x, y) D.P.Rat.rad >= x & D.P.Rat.rad <= y, ...
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
            
            % Tell CS that this is a cued reward lap
            if D.B.cue_next && cur_quad == 3 && ~D.B.track_rew_reset(cur_quad)
                % Tell CS to have robot slow to reward
                CS_Send(4, mean(D.UI.rewBnds(D.I.rot,:)));
                % Temp show dark patch
                if ~shouldExit
                    set(D.UI.ptchFdRstH(cur_quad, D.I.rot), ...
                        'FaceAlpha', 0.5, ...
                        'EdgeAlpha', 0);
                end
                pause(0.1);
                if ~shouldExit
                    set(D.UI.ptchFdRstH(cur_quad, D.I.rot), ...
                        'FaceAlpha', 0, ...
                        'EdgeAlpha', 0);
                end
            end
            
            % Set any current occupied quad to true
            D.B.track_rew_reset(cur_quad) = true;
            
            % Set later quads to false to prevent backwards runs
            D.B.track_rew_reset(1:3 > cur_quad) = false;
            
            % Hide patches of passed quadrants
            if ~shouldExit
                set(D.UI.ptchFdRstH(D.B.track_rew_reset, D.I.rot), ...
                    'FaceAlpha', 0, ...
                    'EdgeAlpha', 0);
            end
            
            % Show patches of remaining quadrants
            if ~shouldExit
                set(D.UI.ptchFdRstH(~D.B.track_rew_reset, D.I.rot), ...
                    'FaceAlpha', 0.1, ...
                    'EdgeAlpha', 1);
            end
            
            % If all bounds triggered
            if all(D.B.track_rew_reset)
                
                % Darken reward feeder patch
                if ~shouldExit
                    set(D.UI.ptchFdH(D.I.rot), ...
                        'FaceAlpha', 0.5, ...
                        'EdgeAlpha', 1)
                end
                
            else
                
                % Lighten reward feeder patch
                if ~shouldExit
                    set(D.UI.ptchFdH(D.I.rot), ...
                        'FaceAlpha', 0.25, ...
                        'EdgeAlpha', 1)
                end
            end
            
        end
        
    end






% --------------------------PLOT POSITION----------------------------------

    function [] = SF_Pos_Plot()
        
        if ...
                ~isempty(D.P.Rat.x) && ...
                ~isempty(D.P.Rat.y)
            
            % Get handle inex to save pos data to to
            Hind = find(isnan(D.UI.vtPltH(:)), 1, 'first');
            
            % HEADING
            
            % Plot arrow
            if D.B.doPlotHD
                if any(~isnan(D.A.rv_samples(:,2)))
                    
                    % Delete plot object
                    if isfield(D.UI, 'vtPltHD');
                        delete(D.UI.vtPltHD(:));
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
                    if ~shouldExit
                        D.UI.vtPltHD(1) = ...
                            plot(x, y, '-', ...
                            'Color', [0.5, 0.5, 0.5], ...
                            'LineWidth', 3, ...
                            'Parent', D.UI.axH(2));
                    end
                    
                    % Plot thin foreground line
                    if ~shouldExit
                        D.UI.vtPltHD(2) = ...
                            plot(D.UI.axH(2), x, y, '-', ...
                            'Color', D.UI.ratPosHistCol, ....
                            'LineWidth', 1, ...
                            'Parent', D.UI.axH(2));
                    end
                end
            end
            
            % RAT
            
            % Plot all new VT data
            if ~shouldExit
                D.UI.vtPltH(Hind) = ...
                    plot(D.P.Rat.x, D.P.Rat.y, '.', ...
                    'MarkerFaceColor', D.UI.ratPosHistCol, ...
                    'MarkerEdgeColor', D.UI.ratPosHistCol, ...
                    'MarkerSize', 6, ...
                    'Parent', D.UI.axH(1));
            end
            
            % Plot current rat position with larger marker
            if isfield(D.UI, 'vtRatPltNow');
                delete(D.UI.vtRatPltNow);
            end
            % plot current position
            if ~shouldExit
                D.UI.vtRatPltNow = ...
                    plot(D.P.Rat.x(1,end), D.P.Rat.y(end), 'o', ...
                    'MarkerFaceColor', D.UI.ratPosNowCol, ...
                    'MarkerEdgeColor', [0, 0, 0], ...
                    'MarkerSize', 10, ...
                    'Parent', D.UI.axH(2));
            end
            
            % Plot last reward point
            if any(D.B.plot_rew)
                ind = find(D.B.track_fd, 1, 'last');
                if ~shouldExit
                    D.UI.vtPltH(Hind+1) = ...
                        plot(D.P.Rat.x(ind), D.P.Rat.y(ind), 'o', ...
                        'MarkerFaceColor', D.UI.ratPosHistCol, ...
                        'MarkerEdgeColor', [0, 0, 0], ...
                        'MarkerSize', 8, ...
                        'Parent', D.UI.axH(1));
                end
                
                % Get index reward plot handles
                D.UI.vtPltHrew(Hind+1) = true;
                
                % Reset bool
                D.B.plot_rew = false;
                
            end
            
            % Plot feeder inbound points in feed cond color
            if isfield(D.UI, 'feedPltNow');
                delete(D.UI.feedPltNow);
            end
            [xbnd, ybnd] =  Get_Trk_Bnds( [D.P.Rob.feedRad,D.P.Rob.feedRad]);
            if ~shouldExit
                D.UI.feedPltNow = ...
                    plot(xbnd(1), ybnd(1), 'o', ...
                    'MarkerFaceColor', D.UI.rotCol(D.I.rot,:), ...
                    'MarkerEdgeColor', [0, 0, 0], ...
                    'MarkerSize', 8, ...
                    'Parent', D.UI.axH(2));
            end
            
% ROBOT
            
            % Plot current robot pos
            if isfield(D.UI, 'vtRobPltNow');
                delete(D.UI.vtRobPltNow);
            end
            if ~shouldExit
                D.UI.vtRobPltNow = ...
                    plot(D.P.Rob.x(1,end), D.P.Rob.y(end), 'o', ...
                    'MarkerFaceColor', D.UI.robPosCol, ...
                    'MarkerEdgeColor', [0, 0, 0], ...
                    'MarkerSize', 10, ...
                    'Parent', D.UI.axH(2));
            end
            
            % Plot rob patch
            if isfield(D.UI, 'ptchRobPos');
                delete(D.UI.ptchRobPos);
            end
            [xbnd, ybnd] =  Get_Trk_Bnds( [D.P.Rob.setRad, D.P.Rob.guardRad]);
            if ~shouldExit
                D.UI.ptchRobPos = ...
                    patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
                    [ybnd(1,:),fliplr(ybnd(2,:))], ...
                    D.UI.robPosCol, ...
                    'FaceAlpha',0.5, ...
                    'Parent',D.UI.axH(1));
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
            'Reversals_Total:%s%d\n', ...
            'Rotations_Total:%s%d'], ...
            repmat('_',1,2), sum([D.C.lap_cnt{:}]), ...
            repmat('_',1,2), sum([D.C.rew_cnt{:}]), ...
            repmat('_',1,2), D.C.rev_cnt, ...
            repmat('_',1,2), D.C.rot_cnt);
        if ~shouldExit
            set(D.UI.txtPerfInf(4), 'String', infstr)
        end
        
        % standard
        infstr = sprintf([...
            'Laps______Stand:%s%d\n', ...
            'Rewards___Stand:%s%d'], ...
            repmat('_',1,2), D.C.lap_cnt{3}(end), ...
            repmat('_',1,2), D.C.rew_cnt{3}(end));
        if ~shouldExit
            set(D.UI.txtPerfInf(1), 'String', infstr)
        end
        
        % 40 deg
        infstr = sprintf([...
            'Laps______40%c:%s%d|%d\n', ...
            'Rewards___40%c:%s%d|%d'], ...
            char(176), repmat('_',1,4), D.C.lap_cnt{2}(end), sum(D.C.lap_cnt{2}), ...
            char(176), repmat('_',1,4), D.C.rew_cnt{2}(end), sum(D.C.rew_cnt{2}));
        if ~shouldExit
            set(D.UI.txtPerfInf(2), 'String', infstr)
        end
        % 0 deg
        infstr = sprintf([...
            'Laps______0%c:%s%d|%d\n', ...
            'Rewards___0%c:%s%d|%d'], ...
            char(176), repmat('_',1,5), D.C.lap_cnt{1}(end), sum(D.C.lap_cnt{1}), ...
            char(176), repmat('_',1,5), D.C.rew_cnt{1}(end), sum(D.C.rew_cnt{1}));
        if ~shouldExit
            set(D.UI.txtPerfInf(3), 'String', infstr)
        end
        
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
        if ~shouldExit
            set(D.UI.txtTimElspInf, 'String', infstr)
        end
        
        % Save current time to UserData
        if ~shouldExit
            set(D.UI.txtTimElspInf, 'UserData', nowTim)
        end
        
    end

% =========================================================================






%% ======================== CALLBACK FUNCTIONS ============================

% RAT SELECTION
    function [] = PopRat(~, ~, ~)
        
        if get(D.UI.popRat,'Value') ~= 1
            
            % Change font
            set(D.UI.popRat, 'FontName','MS Sans Serif','FontSize',8);
            
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
            if D.PAR.sesNum > 0
                
                % Set defualt ICR Condition to training
                set(D.UI.popCond, 'Value', ...
                    find(ismember(D.UI.condList, 'ICRb Session'))); %'Manual Feed' 'ICRb Session'
                % run callback
                PopCond();
                
                % Set reward delay
                D.PAR.rewDel = ...
                    D.SS_In_All.Reward_Delay(D.PAR.ratInd);
                set(D.UI.popRwDl, 'Value', ...
                    find(ismember(D.UI.delList, D.PAR.rewDel)));
                % convert to number
                D.PAR.rewDel = str2double(char(D.PAR.rewDel));
                % run callback
                PopRewDel();
                
                % Set cue condition
                D.UI.cueFeed = ...
                    D.SS_In_All.Cue_Condition(D.PAR.ratInd);
                set(D.UI.radCue( ...
                    ismember(D.PAR.catCueCond, D.UI.cueFeed)), 'Value', 1);
                RadCue();
                
                % Set SOUND condition
                D.UI.snd(1:2) = logical(...
                    D.SS_In_All.Sound_Conditions(D.PAR.ratInd,1:2));
                set(D.UI.radSnd(D.UI.snd(1:2)), 'Value', 1);
                RadSnd();
                
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
        
    end

% ICR CONDITION
    function [] = PopCond(~, ~, ~)
        
        if get(D.UI.popCond, 'Value') ~= 1
            D.B.manual_ses = false;
            D.B.rot_ses = false;
            
            % Change font
            set(D.UI.popCond, 'FontName','MS Sans Serif','FontSize',8);
            
            % Set condition bools
            if get(D.UI.popCond, 'Value') == 2; % manual session
                D.B.manual_ses = true;
            elseif get(D.UI.popCond, 'Value') == 4; % rotatin session
                D.B.rot_ses = true;
            end
            
        end
        
    end

% REWARD DELAY
    function [] = PopRewDel(~, ~, ~)
        
        if get(D.UI.popRwDl,'Value') ~= 1
            
            % Change font
            set(D.UI.popRwDl, 'FontName','MS Sans Serif','FontSize',8);
            
            % Get selected minimum time in bounds for reward
            D.PAR.rewDel = str2double(D.UI.delList{get(D.UI.popRwDl,'Value'),:});
            
        end
        
    end

% CUE CONDITION
    function [] = RadCue(~, ~, ~)
        
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
        
    end

% SOUND CONDITION
    function [] = RadSnd(~, ~, ~)
        
        % Note: D.UI.snd format
        %         {'White'}
        %         {'Reward'}
        
        for z_sa = 1:2
            % Get button ID and state value
            D.UI.sndVal = get(D.UI.radSnd(z_sa), 'Value');
            D.UI.sndID = get(D.UI.radSnd(z_sa), 'UserData');
            if D.UI.sndVal == 1
                D.UI.snd(z_sa) = true;
            else
                D.UI.snd(z_sa) = false;
            end
        end
        
        % Turn off all sound if white noise is off
        if D.UI.snd(1) == false
            D.UI.snd(2) = false;
            set(D.UI.radSnd(2), 'Value', 0);
        end
        
    end

% SETUP DONE
    function [] = BtnStupDn(~, ~, ~)
        
        % Confirm UI entries
        all(cell2mat(get(D.UI.radCue,'Value')) == 0)
        % Check if all fields entered with pop-up window
        if ...
                get(D.UI.popRat,'Value') == 1 || ...
                get(D.UI.popCond,'Value') == 1 || ...
                get(D.UI.popRwDl,'Value') == 1 || ...
                all(cell2mat(get(D.UI.radCue,'Value')) == 0)
            
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
        %             char(D.UI.cueFeed), ...
        %             D.UI.sndState{get(D.UI.radSnd(1),'Value')+1}, ...
        %             D.UI.sndState{get(D.UI.radSnd(2),'Value')+1});
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
        set(D.UI.btnStupDn, 'UserData', 1);
        
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
        set(D.UI.btnSesDone, ...
            'Enable', 'On', ...
            'BackgroundColor', D.UI.enabledCol)
        
        % Start acq if not running
        if get(D.UI.btnRec,'Value') == 1 && ...
                get(D.UI.btnAcq,'Value') == 0
            
            % Set aquire button value to 1
            set(D.UI.btnAcq,'Value', 1)
            
            % Run BtnAcq
            BtnAcq(D.UI.btnAcq);
            
        end
        
    end

% ROTATION BUTTON
    function [] = BtnICR(~, ~, ~)
        
        % get image and color for post rotation feeder
        rotNext = [1, 2] ~=  D.I.rot;
        
        % Determine rotation trigger bounds for this event
        D.UI.rotBndNext = D.UI.rotBnds(...
            find(D.PAR.catRotPos == D.PAR.rotPos(D.C.rot_cnt+1)), ...
            :, rotNext);
        
        % Plot selected rotation trigger bounds
        [xbnd, ybnd] =  Get_Trk_Bnds(D.UI.rotBndNext);
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
        
    end

% RECORDING DONE
    function [] = BtnSesDone(~, ~, ~)
        
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
                NlxSendCommand(D.NLX.ses_end_evt);
                
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
                NlxSendCommand(D.NLX.rat_in_evt{2});
                
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
        set(D.UI.btnSesDone, ...
            'Enable', 'Off', ...
            'BackgroundColor', D.UI.disabledCol);
        % Disable radSnd buttons
        set(D.UI.radSnd, 'Enable', 'Off')
        
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
        D.AC.data(4) = 0;
        fwrite(tcpIP,D.AC.data,'int8');
        
        % Print end time
        infstr = sprintf( ...
            'Stop_:%s%s\n', ...
            repmat('_',1,6), datestr(D.T.ses_end_tim, 'HH:MM:SS'));
        set(D.UI.txtTimEndInf, 'String', infstr)
        
    end

% SAVE SESSION DATA
    function [] = BtnSav(~, ~, ~)
        
        % Confirm that Cheetah is closed
        choice = questdlgAWL('Cheetah Closed?', ...
            'CHEETAH CLOSED', ...
            'OK', [], [], 'OK', ...
            D.UI.qstDlfPos);
        drawnow; % force update UI
        % Handle response
        switch choice
            case 'OK'
        end
        
        % Change NLX recording directory
        
        % Save directory var
        % formt: datestr(clock, 'yyyy-mm-dd_HH-MM-SS', 'local');
        D.DIR.rec = fullfile(D.DIR.cheetSaveTop, D.PAR.ratLab(2:end));
        
        % Make directory if none exists
        if exist(D.DIR.rec, 'dir') == 0
            mkdir(D.DIR.rec);
        end
        
        % Save GUI window image
        export_fig(FigH, fullfile(D.DIR.cheetTempTop, D.DIR.recFi, 'GUI.jpg'))
        
        % Save sesion data to tables
        Save_Ses_Dat()
        
        % Set bool to true
        D.UI.saved = true;
        
        % Copy Cheetah file to rat directory
        copyfile(fullfile(D.DIR.cheetTempTop, D.DIR.recFi),fullfile(D.DIR.rec, D.DIR.recFi))
        
        % Disable buttons
        % save button
        set(gcbo, 'Enable', 'Off')
        
    end

% QUIT ALL
    function [] = BtnQuit(~, ~, ~)
        
        %         % Check if recording and sesion data have been saved
        %         if ~D.UI.saved
        %             % Construct a questdlg with two options
        %             % Note: based on built in function
        %             choice = questdlgAWL('QUIT WITHOUT SAVING?', ...
        %                 'QUIT?', ...
        %                 'Yes', 'No', [], 'No', ...
        %                 D.UI.qstDlfPos);
        %             drawnow; % force update UI
        %             % Handle response
        %             switch choice
        %                 case 'Yes'
        %                 case 'No'
        %                     return
        %             end
        %         end
        
        % Disable
        % quit button
        set(gcbo, 'Enable', 'Off');
        
        % Tell C# to begin quit
        CS_Send(2);
        
        % Make sure main loop ends
        D.B.ratIn = false;
        
    end

% CLEAR VT BUTTON
    function [] = BtnClrVT(~, ~, ~)
        
        % Delete all tracker data
        delete(D.UI.vtPltH(~isnan(D.UI.vtPltH(1,:))))
        % Reinitialize handle array back to NAN
        D.UI.vtPltH = NaN(1,60*120/D.PAR.polRate); % VT plot handle array
        D.UI.vtPltHrev = false(1,60*120/D.PAR.polRate); % bool for tracking VT plot handles of turn around points
        D.UI.vtPltHrew = false(1,60*120/D.PAR.polRate); % bool for tracking VT plot handles of reward delivery points
        
    end

% FEEDER PULSE
    function [] = BtnPlsFd(~, ~, ~)
        
    end

% FEEDER PULSE
    function [] = BtnExit(~, ~, ~)
        shouldExit = true;
    end






%% ========================== MINOR FUNCTIONS =============================

% BUTTON COLOR
    function [C] = Btn_Col()
        
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
        
        % Green
        C{2}(:,:,1) = Lo;
        C{2}(:,:,2) = Hi;
        C{2}(:,:,3) = Lo;
        
        C = C';
    end







% ---------------------------GET TRACK BOUNDS------------------------------
    function [] = Update_Console(str)
        % Print new info to console
        if exist('consoleText','var')
            consoleText = [str, consoleText];
            if exist('FigH','var')
                if isfield(D,'UI')
                    if isfield(D.UI, 'editConsole')
                        try
                            set(D.UI.editConsole, ...
                                'String', consoleText);
                            drawnow;
                        catch
                        end
                    end
                end
            end
        end
    end







% ---------------------------GET TRACK BOUNDS------------------------------
    function [xbnd, ybnd] = Get_Trk_Bnds(polbnds)
        
        if polbnds(1) > polbnds(2)
            polbnds = wrapToPi(polbnds);
        end
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






% --------------------------SEND CS COMMAND--------------------------------

    function[] = CS_Send(id_ind, data1, data2)
        
        if ~shouldExit
            
            % Get id
            id = D.CS.m2c_id(id_ind);
            
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
            
            Update_Console(sprintf('\nCS_Send\nm2c_id = %s\nm2c_dat1 = %2.2f\nm2c_dat2 = %2.2f\nTime: %s\n', ...
                id, data1 , data2, datestr(now, 'HH:MM:SS')));
            
        end
        
    end






% ---------------------------SAVE SESSION DATA-----------------------------
    function [] =  Save_Ses_Dat()
        
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
        D.SS_Out_ICR.(D.PAR.ratLab).Sound_Conditions(rowInd,:) = D.UI.snd(1:3);
        D.SS_Out_ICR.(D.PAR.ratLab).Air_Conditions(rowInd,:) = D.UI.snd(4:6);
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
        D.SS_In_All.Sound_Conditions(D.PAR.ratInd,:) = [D.UI.snd(1:2),false];
        D.SS_In_All.Air_Conditions(D.PAR.ratInd,:) = false(1,3);
        
        % Save out data
        SS_Out_ICR = D.SS_Out_ICR;
        save(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_Out_ICR.mat'), 'SS_Out_ICR');
        SS_In_All = D.SS_In_All;
        save(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_In_All.mat'), 'SS_In_All');
        
    end






% -----------------------------DISCONECT ALL-------------------------------
    function [] = Disconect_All()
        
        Update_Console(sprintf('\nEXITING.......\nTime: %s\n', ...
            datestr(now, 'HH:MM:SS')));
        
        % End NLX polling
        D.B.poll_nlx = false;
        
        if isfield(D, 'AC')
            if ~isempty(D.AC)
                
                % Close AC connection
                D.AC.data = zeros(1, length(D.AC.data));
                % keep connection
                D.AC.data(1) = 1;
                % post to AC computer
                fwrite(tcpIP,D.AC.data,'int8');
                % pause to allow image to close
                pause(0.01);
                % close all
                D.AC.data = zeros(1, length(D.AC.data));
                % post to AC computer
                fwrite(tcpIP,D.AC.data,'int8');
                % close AC computer connection
                fclose(tcpIP);
                delete(tcpIP);
                clear tcpIP;
                
                % Show status disconnected
                Update_Console(sprintf('\nDisconnected From AC Computer\nIP: %s\nTime: %s\n', ...
                    D.AC.IP, datestr(now, 'HH:MM:SS')));
            end
        end
        
        if isfield(D, 'NLX')
            if ~isempty(D.NLX)
                
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
                while NlxAreWeConnected() == 1
                    NlxDisconnectFromServer();
                end
                
                % Show status disconnected
                if NlxAreWeConnected() ~= 1
                    Update_Console(sprintf('\nDisconnected From DigitalLynx SX\nIP: %s\nTime: %s\n', ...
                        D.NLX.IP, datestr(now, 'HH:MM:SS')));
                end
                
            end
            
        end
        
        % Close figure
        if exist('FigH','var')
            if ishghandle(FigH)
                close(FigH)
            end
        end
        
        clear('FigH', ...
            'D', ...
            'tcpIP', ...
            'm2c_id', ...
            'c2m_id', ...
            'm2c_dat1', ...
            'm2c_id', ...
            'm2c_dat1', ...
            'consoleText');
        
    end






% ========================================================================

end