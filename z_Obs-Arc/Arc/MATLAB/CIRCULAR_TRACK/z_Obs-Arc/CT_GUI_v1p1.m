function[] = CT_GUI

%% Set parameters

% Print that script is running
fprintf('\r\nCT_GD.PAR.m\n  Time: %s\r\n', ...
    datestr(now, 'HH:MM:SS AM'))

% Directories
% top dir
D.DIR.ioTop = fullfile(pwd,'\IOfiles');
% i/o dirs
D.DIR.ssSD = 'SessionData';

% Turn of p-code warning
warning('off','MATLAB:oldPfileVersion')

% Load data tables
T = load(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_In_All.mat'));
D.SS_In_All = T.SS_In_All;
T = load(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_Out_CT.mat'));
D.SS_Out_CT = T.SS_Out_CT;
clear T;

%% Setup figure

% Figure
D.UI.figH = figure( ...
    'Name', 'External Track', ...
    'MenuBar', 'none', ...
    'Color', [0.9, 0.9, 0.9], ...
    'Visible', 'On');

% Axis
D.UI.axH = axes( ...
    'Units', 'Normalized', ...
    'Visible', 'Off'); 

% Get figure pos
sc = get(0,'MonitorPositions');
sc1 = sc(1,:);
sc2 = sc(2,:);
% fig width/hight
f_wh = [300 500]; 
f_pos = [...
    (sc1(3)-f_wh(1))/2, ...
    (sc1(4)-f_wh(2))/2, ...
    f_wh(1), ...
    f_wh(2)];
set(D.UI.figH,'Position',f_pos)

% Get screen center for question dialogue
D.UI.qstDlfPos = [(sc1(3)/2), (sc1(4)/2)];

%% Add UI controls

% Save data to figure
% Note: this will be used to pass data between UI functions
guidata(D.UI.figH,D)

% Make rat list
ratInc = D.SS_In_All.Include & D.SS_In_All.CT;
D.UI.ratList = [{'Rat Number'}; ...
    D.SS_In_All.Properties.RowNames(ratInc)];

% Create matrix for saving time stamps from arduino
% values: [IR1_TS, IR2_TS, RevStrTS, RevEndTS]
% save time stamps (sec)
D.PAR.TSmat = NaN(4,10000);
% save counts
D.PAR.CntMat = zeros(4,1);

% ARDUINO VAR SETUP

% aquire arduino data
D.PAR.acquire = false;
% quit session
D.PAR.quit = false;
% session saved
D.PAR.saved = false;
% start session time
D.PAR.ses_str_tim = 0;

% Define all pins
% 'output' to speakers
toneOutPin = 11; 
% 'input' to trigger negative tone
toneNegPin = 12; 
% 'output' to led
ledPin = 9; 
% 'output' to opto for solonoid
solPin = 13; 
% 'input' from ir 1 (feeder ir)
ir1Pin = 7; 
% 'input' from ir 2 (half lap ir)
ir2Pin = 8; 

% Posetive tone variables
% frequency
tonePosFrq = 2500;
% duration (sec)
tonePosDur = 1000/1000;

% Negative tone variables
toneNegFrq = 4000;
% duration (sec)
toneNegDur = 100/1000;
% delay (sec)
toneNegDel = 100/1000;
% reset to 'listen' for neg tone on
toneNegOnCheck = 1; 
% reset to 'listen' for neg tone off
toneNegOffCheck = 0; 
% time and lap up alert tone variables
toneAlertFrq = 50;
toneAlertDur = 3000/1000;
% check for max time
toneTimUpCheck = true;
% check for max laps
toneLapUpCheck = true;

% LED variables
% value between
ledDuty = 1; % [0, 1]
% interval at which to blink (ms)
ledDur = 1000/1000; 
% store last time LED was updated (ms)
ledTim = clock;     
% state that led toggles between
ledState = ledDuty; 

% Sololonoid variables
% duration solonoid should stay open
solDur = 200/1000; 

% IR sensor variables
% reset to 'listen' to ir 1
ir1reset = 0; 
% reset to 'listen' to ir 2
ir2reset = 1;
% dispense reward
trigRew = 0; 
% track if LED should be on
trigLED = 0; 

%% Create UI objects

% CONNECT ARDUINO
pos = [0.05 0.915 0.9 0.05];
D.UI.btnArdConnect = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','Connect Arduino',...
    'Callback', {@BtnArdConnect},...
    'UserData', 0, ...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [0, 0, 0], ...
    'FontWeight','bold',...
    'FontSize',12,...
    'Enable', 'On');

% RAT SELECTION
pos = [0.05 0.83 0.9 0.05];
D.UI.popRat = uicontrol(...
    'Parent',D.UI.figH,...
    'Callback', {@PopRat},...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor',[1 1 1],...
    'ForegroundColor', [0.1, 0.1, 0.1], ...
    'FontName','Courier New',...
    'FontSize',14,...
    'FontWeight','bold',...
    'String',D.UI.ratList,...
    'Style','popupmenu',...
    'Value',1,...
    'Enable', 'Off');

% RAT INFO
pos = [0.05 0.745 0.9 0.0675];
D.UI.txtSesInf = uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtSesInf,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position', pos, ...
    'BackgroundColor', [0.9, 0.9, 0.9], ...
    'ForegroundColor', [0.3, 0.3, 0.3], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize', 12);
% start/end time
pos = [0.05 0.675 0.9 0.07];
D.UI.txtStrEndTim = uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtStrEndTim,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position', pos, ...
    'BackgroundColor', [0.9, 0.9, 0.9], ...
    'ForegroundColor', [0, 0, 0], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize', 12);

% pannel
panpos = [0.05 0.225 0.9 0.45];
D.UI.panSesStart = uipanel(...
    'Parent',D.UI.figH,...
    'Units','Normalized',...
    'BorderType','line',...
    'BorderWidth',10,...
    'BackgroundColor', [0.9, 0.9, 0.9], ...
    'ForegroundColor', [0, 0, 0], ...
    'HighlightColor',[0.75, 0.75, 0.75],...
    'FontSize',20,...
    'FontWeight','bold',...
    'TitlePosition','centertop',...
    'Clipping','on',...
    'Position', panpos);

% elapsed time
txtpos = [0.1 panpos(2)+0.035+0.075*3+0.0267*3+0.015 0.8 0.05];
infstr = sprintf('Time:');
D.UI.txtElpsTim = uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtElpsTim,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position',txtpos, ...
    'BackgroundColor', [0.9, 0.9, 0.9], ...
    'ForegroundColor', [0.3, 0.3, 0.3], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize',18,...
    'String', infstr);
txtpos = [0.33 panpos(2)+0.035+0.075*3+0.0267*3 0.57 0.075];
infstr = sprintf('00:00:00');
D.UI.txtElpsTimNum = uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtElpsTimNum,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position',txtpos, ...
    'BackgroundColor', [1, 1, 1], ...
    'ForegroundColor', [0.5, 0.5, 0.5], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize',26,...
    'String', infstr);

% lap count
txtpos = [0.1 panpos(2)+0.035+0.075*2+0.0267*2+0.015 0.8 0.05];
infstr = sprintf('Lap Run:');
D.UI.txtLapCnt= uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtLapCnt,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position',txtpos, ...
    'BackgroundColor', [0.9, 0.9, 0.9], ...
    'ForegroundColor', [0.3, 0.3, 0.3], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize', 18,...
    'String', infstr);
lapcnttxtpos = [0.47 panpos(2)+0.035+0.075*2+0.0267*2 0.43 0.075];
infstr = sprintf('0');
D.UI.txtLapCntNum = uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtLapCntNum,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position',lapcnttxtpos, ...
    'BackgroundColor', [1, 1, 1], ...
    'ForegroundColor', [0.5, 0.5, 0.5], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize',26,...
    'String', infstr);

% reward count
txtpos = [0.1 panpos(2)+0.035+0.075+0.0267+0.015 0.8 0.05];
infstr = sprintf('Rewards:');
D.UI.txtRewCnt= uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtRewCnt,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position',txtpos, ...
    'BackgroundColor', [0.9, 0.9, 0.9], ...
    'ForegroundColor', [0.3, 0.3, 0.3], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize', 18,...
    'String', infstr);
rewcnttxtpos = [0.47 panpos(2)+0.035+0.075+0.0267 0.43 0.075];
infstr = sprintf('0');
D.UI.txtRewCntNum = uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtRewCntNum,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position',rewcnttxtpos, ...
    'BackgroundColor', [1, 1, 1], ...
    'ForegroundColor', [0.5, 0.5, 0.5], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize',26,...
    'String', infstr);

% reversal count
txtpos = [0.1 panpos(2)+0.035+0.015 0.8 0.05];
infstr = sprintf('Reverse:');
D.UI.txtRevCnt= uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtRevCnt,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position',txtpos, ...
    'BackgroundColor', [0.9, 0.9, 0.9], ...
    'ForegroundColor', [0.3, 0.3, 0.3], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize', 18,...
    'String', infstr);
revcnttxtpos = [0.47 panpos(2)+0.035 0.43 0.075];
infstr = sprintf('0');
D.UI.txtRevCntNum = uicontrol('Parent',D.UI.figH,'Style','text');
set(D.UI.txtRevCntNum,'String','', ...
    'Units','Normalized', ...
    'HorizontalAlignment', 'Left',...
    'Position',revcnttxtpos, ...
    'BackgroundColor', [1, 1, 1], ...
    'ForegroundColor', [0.5, 0.5, 0.5], ...
    'FontName','Courier New',...
    'FontWeight','bold',...
    'FontSize',26,...
    'String', infstr);

% LAP BUTTONS
% add
pos = [lapcnttxtpos(3)+lapcnttxtpos(1)-0.075, lapcnttxtpos(2)+0.0375, 0.075, 0.0375];
D.UI.btnAddLap = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','+',...
    'Callback', {@BtnAddSubLap},...
    'UserData', 1, ...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [0, 0, 0], ...
    'FontWeight','bold',...
    'FontSize',18,...
    'Enable', 'Off');
% subtract
pos = [lapcnttxtpos(3)+lapcnttxtpos(1)-0.075, lapcnttxtpos(2), 0.075, 0.0375];
D.UI.btnSubLap = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','-',...
    'Callback', {@BtnAddSubLap},...
    'UserData', -1, ...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [0, 0, 0], ...
    'FontWeight','bold',...
    'FontSize',18,...
    'Enable', 'Off');

% REWARD BUTTONS
% add
pos = [rewcnttxtpos(3)+rewcnttxtpos(1)-0.075, rewcnttxtpos(2)+0.0375, 0.075, 0.0375];
D.UI.btnAddRew = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','+',...
    'Callback', {@BtnAddSubRew},...
    'UserData', 1, ...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [0, 0, 0], ...
    'FontWeight','bold',...
    'FontSize',18,...
    'Enable', 'Off');
% subtract
pos = [rewcnttxtpos(3)+rewcnttxtpos(1)-0.075, rewcnttxtpos(2), 0.075, 0.0375];
D.UI.btnSubRew = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','-',...
    'Callback', {@BtnAddSubRew},...
    'UserData', -1, ...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [0, 0, 0], ...
    'FontWeight','bold',...
    'FontSize',18,...
    'Enable', 'Off');

% REVERSAL BUTTONS
% add
pos = [revcnttxtpos(3)+revcnttxtpos(1)-0.075, revcnttxtpos(2)+0.0375, 0.075, 0.0375];
D.UI.btnAddRev = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','+',...
    'Callback', {@BtnAddSubRev},...
    'UserData', 1, ...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [0, 0, 0], ...
    'FontWeight','bold',...
    'FontSize',18,...
    'Enable', 'Off');
% subtract
pos = [revcnttxtpos(3)+revcnttxtpos(1)-0.075, revcnttxtpos(2), 0.075, 0.0375];
D.UI.btnSubRev = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','-',...
    'Callback', {@BtnAddSubRev},...
    'UserData', -1, ...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [0, 0, 0], ...
    'FontWeight','bold',...
    'FontSize',18,...
    'Enable', 'Off');

% START SESSION
pos = [0.05 0.145 0.9 0.075];
D.UI.btnStrtSes = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','START SESSION',...
    'Callback', {@BtnStrtSes},...
    'UserData', 0, ...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [0, 0.5, 0], ...
    'FontWeight','bold',...
    'FontSize',18,...
    'Enable', 'Off');

% QUIT SESSION
pos = [0.325 0.035 0.35 0.075];
D.UI.btnQuitSes = uicontrol('style','push', ...
    'Parent', D.UI.figH, ...
    'String','QUIT',...
    'Callback', {@BtnQuitSes},...
    'Units','Normalized', ...
    'Position', pos, ...
    'BackgroundColor', [0.75, 0.75, 0.75], ...
    'ForegroundColor', [1, 1, 1], ...
    'FontWeight','bold',...
    'FontSize',20,...
    'Enable', 'On');

% Set figure visibility on
set(D.UI.figH, 'Visible', 'On');
% Bring to top
uistack(D.UI.figH,'top')

% Save out data
guidata(D.UI.figH,D)

%% Aquire Arduino data

while ~D.PAR.quit
    
    D = guidata(D.UI.figH);
    drawnow; % force update UI
    
    %% Set up arduino
    if D.PAR.acquire
        
        % Initial setup
        
        % Set pin direction
        configureDigitalPin(D.UI.a, toneOutPin, 'output');
        configureDigitalPin(D.UI.a, toneNegPin, 'input');
        configureDigitalPin(D.UI.a, ledPin, 'output');
        configureDigitalPin(D.UI.a, solPin, 'output');
        configureDigitalPin(D.UI.a, ir1Pin, 'input');
        configureDigitalPin(D.UI.a, ir2Pin, 'input');
        
        % Set initial pin state
        writeDigitalPin(D.UI.a, toneOutPin, 0);
        writeDigitalPin(D.UI.a, ledPin, 0);
        writeDigitalPin(D.UI.a, solPin, 0);
        
        % Save out handle data
        guidata(D.UI.figH,D);
        
    end
    
    %% Loop until quit or finished acquiring
    while D.PAR.acquire
        
        % Update gui data
        D = guidata(D.UI.figH);
        
        % Monitor IR Sensors
        % feeder IR (Rew)
        ir1Val = readDigitalPin(D.UI.a, ir1Pin);
        if (ir1reset == 1 && ir1Val == 0)
            % Serial.println("IR1 On");
            ir1reset = 0;
            ir2reset = 1;
            trigRew = 1;
            trigLED = 0;
            % update vars
            D.PAR.CntMat(1) = D.PAR.CntMat(1) + 1;
            D.PAR.TSmat(1,D.PAR.CntMat(1)) = etime(clock ,D.PAR.ses_str_tim);
        end
        
        % half track IR (Lap)
        ir2Val = readDigitalPin(D.UI.a, ir2Pin);
        if (ir2reset == 1 && ir2Val == 0)
            % Serial.println("IR2 On");
            ir1reset = 1;
            ir2reset = 0;
            trigLED = 1;
            % update vars
            D.PAR.CntMat(2) = D.PAR.CntMat(2) + 1;
            D.PAR.TSmat(2,D.PAR.CntMat(2)) = etime(clock ,D.PAR.ses_str_tim);
        end
        
        % Aversive Tone
        toneNegVal = readDigitalPin(D.UI.a, toneNegPin);
        while toneNegVal == 1
            toneNegVal = readDigitalPin(D.UI.a, toneNegPin);
            if toneNegOnCheck == 1
                toneNegOnCheck = 0;
                toneNegOffCheck = 1;
                % update vars
                D.PAR.CntMat(3) = D.PAR.CntMat(3) + 1;
                D.PAR.TSmat(3,D.PAR.CntMat(3)) = etime(clock ,D.PAR.ses_str_tim);
            end
            playTone(D.UI.a, toneOutPin, toneNegFrq, toneNegDur);
            pause(toneNegDel);
        end
        if toneNegOffCheck == 1
            toneNegOnCheck = 1;
            toneNegOffCheck = 0;
            % update vars
            D.PAR.CntMat(4) = D.PAR.CntMat(4) + 1;
            D.PAR.TSmat(4,D.PAR.CntMat(4)) = etime(clock ,D.PAR.ses_str_tim);
        end
        
        % Dispense Reward
        if (trigRew == 1)
            playTone(D.UI.a, toneOutPin, tonePosFrq, tonePosDur);
            writeDigitalPin(D.UI.a, solPin, 1);
            pause(solDur);
            writeDigitalPin(D.UI.a, solPin, 0);
        end
        
        
        % Track LED State
        if (trigLED == 1)
            if etime(clock ,ledTim) > ledDur
                
                % if the LED is off turn it on and vice-versa:
                if (ledState == 0)
                    ledState = ledDuty;
                    
                else
                    ledState = 0;
                end
                
                
                % set the LED with the ledState of the variable:
                % writeDigitalPin(D.UI.a, ledPin, ledState);
                writePWMDutyCycle(D.UI.a, ledPin, ledState);
                
                ledTim = clock ; % reset time
                
            end
        end
        
        if (trigLED == 0 && ledState == ledDuty)
            ledState = 0;
            writePWMDutyCycle(D.UI.a, ledPin, ledState);
        end
        
        trigRew = 0;
        
        % Update time
        nowTim = etime(clock ,D.PAR.ses_str_tim);
        % Set color to red and play tone if 20 min has elapsed
        if nowTim > 20*60 && toneTimUpCheck
            set(D.UI.txtElpsTimNum, 'ForegroundColor', [1 0 0])
            playTone(D.UI.a, toneOutPin, toneAlertFrq, toneAlertDur);
            toneTimUpCheck = false;
        end
        infstr = sprintf('%s', datestr(nowTim/(24*60*60), 'HH:MM:SS'));
        set(D.UI.txtElpsTimNum, 'String', infstr)
        
        % Update laps
        if D.PAR.CntMat(2) > 0
            lap = D.PAR.CntMat(2) - 1;
            if lap == 20 && toneLapUpCheck
                set(D.UI.txtLapCntNum, 'ForegroundColor', [1 0 0])
                playTone(D.UI.a, toneOutPin, toneAlertFrq, toneAlertDur);
                toneLapUpCheck = false;
            end
            infstr = sprintf('%d',lap);
            set(D.UI.txtLapCntNum, 'String', infstr)
        end
        
        % Update rewards
        infstr = sprintf('%d',D.PAR.CntMat(1));
        set(D.UI.txtRewCntNum, 'String', infstr)
        
        % Update reversals
        infstr = sprintf('%d',D.PAR.CntMat(3));
        set(D.UI.txtRevCntNum, 'String', infstr)
        
        drawnow; % force update UI
        
        % Save out handle data
        guidata(D.UI.figH,D);
        
    end
    
end

% close gui
close(D.UI.figH)

% clear arduino variable
clear D.UI.a;

% Close MATLAB
% Exit MATLAB
% Get pop-up position
sc = get(0,'MonitorPositions');
sc1 = sc(1,:);
qstDlfPos = [sc1(3)/2, sc1(4)/2];
choice = questdlgAWL('Do you want to exit MATLAB?', ...
    'EXIT MATLAB', 'Yes', 'No', [], 'Yes', qstDlfPos);
% Handle response
switch choice
    case 'Yes'
        exit; % exit MATLAB
    case 'No'
end

%% Callback functions

% CONNECT ARDUINO
    function [] = BtnArdConnect(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        % Connect
        try
            D.UI.a = arduino('com3', 'uno');
        catch
            questdlgAWL('CHECK THAT ARDUINO IS TURNED ON', ...
                'ARDUINO ON?', 'OK', [], [], 'OK', D.UI.qstDlfPos);
            return
        end
        
        % Disable button
        set(D.UI.btnArdConnect, 'String', 'Arduino Connected');
        set(D.UI.btnArdConnect, 'Enable', 'Off');
        
        % Enable other gui features
        set(D.UI.popRat, 'Enable', 'On');
        
        % Save out handle data
        guidata(D.UI.figH,D);
    end

% RAT SELECTION
    function [] = PopRat(~, ~, ~)
        
        % Get handle data
        D = guidata(D.UI.figH);
        
        if get(gcbo,'Value') ~= 1
            
            % Store rat data
            D.PAR.ratLab = D.UI.ratList{get(gcbo,'Value'),:};
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
            % session number
            if size(D.SS_Out_CT.(D.PAR.ratLab),1) == 1
                D.PAR.sesNum = 1; % first session entry
            else
                D.PAR.sesNum = ...
                    D.SS_In_All.Session_CT_T(D.PAR.ratInd) + 1;
            end
            
            % Print session info
            % get values to print
            dobNum = datenum(D.PAR.ratDOB, 'yyyy/mm/dd');
            agemnth = num2str(floor((now - dobNum)/365*12));
            ses = num2str(D.PAR.sesNum);
            infstr = sprintf([...
                'CT_Session:%s%s\n', ...
                'Age_Months:%s%s\n', ...
                'Age_Group:%s%s\n'], ...
                repmat('_',1,7), ses, ...
                repmat('_',1,7), agemnth, ...
                repmat('_',1,8), char(D.PAR.ratAgeGrp));
            set(D.UI.txtSesInf,'String', infstr)
            
            % Change font
            set(D.UI.popRat, 'FontName','MS Sans Serif');
            
            % Enable other gui features
            set(D.UI.btnStrtSes, 'Enable', 'On');
            
        end
        
        % Save out handle data
        guidata(D.UI.figH,D);
    end

% LAP BUTTONS
    function [] = BtnAddSubLap(hObject, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        userdata = get(hObject, 'UserData');
        
        if userdata == 1
            D.PAR.CntMat(2) = D.PAR.CntMat(2)+1;
            D.PAR.TSmat(2,D.PAR.CntMat(2)) = etime(clock ,D.PAR.ses_str_tim);
        elseif userdata == -1
            if D.PAR.CntMat(2) > 0
                D.PAR.CntMat(2) = D.PAR.CntMat(2)-1;
                D.PAR.TSmat(2,D.PAR.CntMat(2)+1) = NaN;
            end
        end
        
        % Save out handle data
        guidata(D.UI.figH,D);
    end

% REWARD BUTTONS
    function [] = BtnAddSubRew(hObject, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        userdata = get(hObject, 'UserData');
        
        if userdata == 1
            D.PAR.CntMat(1) = D.PAR.CntMat(1)+1;
            D.PAR.TSmat(1,D.PAR.CntMat(1)) = etime(clock ,D.PAR.ses_str_tim);
        elseif userdata == -1
            if D.PAR.CntMat(1) > 0
                D.PAR.CntMat(1) = D.PAR.CntMat(1)-1;
                D.PAR.TSmat(1,D.PAR.CntMat(1)+1) = NaN;
            end
        end
        
        % Save out handle data
        guidata(D.UI.figH,D);
    end

% REVERSAL BUTTONS
    function [] = BtnAddSubRev(hObject, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        userdata = get(hObject, 'UserData');
        
        if userdata == 1
            % rev start
            D.PAR.CntMat(3) = D.PAR.CntMat(3)+1;
            D.PAR.TSmat(3,D.PAR.CntMat(3)) = etime(clock ,D.PAR.ses_str_tim);
            % rev end (add 1 uS to time)
            D.PAR.CntMat(4) = D.PAR.CntMat(4)+1;
            D.PAR.TSmat(4,D.PAR.CntMat(4)) = etime(clock ,D.PAR.ses_str_tim) + (1/10^6);
        elseif userdata == -1
            if D.PAR.CntMat(3) > 0
                % rev start
                D.PAR.CntMat(3) = D.PAR.CntMat(3)-1;
                D.PAR.TSmat(3,D.PAR.CntMat(3)+1) = NaN;
                % rev end
                D.PAR.CntMat(4) = D.PAR.CntMat(4)-1;
                D.PAR.TSmat(4,D.PAR.CntMat(4)+1) = NaN;
            end
        end
        
        % Save out handle data
        guidata(D.UI.figH,D);
    end

% START/END SESSION
    function [] = BtnStrtSes(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        userdata = get(D.UI.btnStrtSes, 'UserData');
        
        if userdata == 0
            
            % save start time
            D.PAR.ses_str_tim = clock ;
            
            % start aquiring data
            D.PAR.acquire = true;
            
            % change string
            set(D.UI.btnStrtSes, 'String','END SESSION',...
                'ForegroundColor', [0.5, 0, 0]);
            
            % Disable over features
            set(D.UI.popRat, 'Enable', 'Off')
            
            % Ungray counters
            set(D.UI.txtElpsTimNum, 'ForegroundColor', [0 0 0])
            set(D.UI.txtLapCntNum, 'ForegroundColor', [0 0 1])
            set(D.UI.txtRewCntNum, 'ForegroundColor', [0 0.5 0])
            set(D.UI.txtRevCntNum, 'ForegroundColor', [1 0 0])
            set(D.UI.btnAddLap, 'Enable', 'On')
            set(D.UI.btnSubLap, 'Enable', 'On')
            set(D.UI.btnAddRew, 'Enable', 'On')
            set(D.UI.btnSubRew, 'Enable', 'On')
            set(D.UI.btnAddRev, 'Enable', 'On')
            set(D.UI.btnSubRev, 'Enable', 'On')
            
            % Add start time to rat info
            infstr = sprintf('Start Time:%s%s\n',...
                repmat(' ',1,1), datestr(D.PAR.ses_str_tim, 'HH:MM:SS', 'local'));
            set(D.UI.txtStrEndTim, 'String', infstr)
            
            % Set user data == 1
            set(D.UI.btnStrtSes, 'UserData', 1);
            
        end
        
        if userdata == 1
            
            % Gray counters
            set(D.UI.txtElpsTimNum, 'ForegroundColor', [0.5 0.5 0.5])
            set(D.UI.txtLapCntNum, 'ForegroundColor', [0.5 0.5 0.5])
            set(D.UI.txtRewCntNum, 'ForegroundColor', [0.5 0.5 0.5])
            set(D.UI.txtRevCntNum, 'ForegroundColor', [0.5 0.5 0.5])
            % Disable buttons
            set(D.UI.btnAddLap, 'Enable', 'Off')
            set(D.UI.btnSubLap, 'Enable', 'Off')
            set(D.UI.btnAddRew, 'Enable', 'Off')
            set(D.UI.btnSubRew, 'Enable', 'Off')
            set(D.UI.btnAddRev, 'Enable', 'Off')
            set(D.UI.btnSubRev, 'Enable', 'Off')
            
            % Darken quit button
            set(D.UI.btnQuitSes, 'ForegroundColor', [0, 0, 0])
            
            % stop acquiring
            D.PAR.acquire = false;
            
            % save time
            % Save end time
            D.PAR.ses_end_tim = clock;
            
            % Add start time to rat info
            infstr = sprintf('Start Time:%s\nEnd Time:%s%s',...
                datestr(D.PAR.ses_str_tim, 'HH:MM:SS', 'local'), ...
                repmat(' ',1,2), datestr(D.PAR.ses_end_tim, 'HH:MM:SS', 'local'));
            set(D.UI.txtStrEndTim, 'String', infstr)
            
            % Construct a questdlg with two options
            % Note: based on built in function
            choice = questdlgAWL('DO YOU WANT TO SAVE SESSION?', ...
                'SAVE?', 'Yes', 'No', [], 'Yes', D.UI.qstDlfPos);
            
            % Save data to table
            switch choice
                case 'Yes'
                    
                    % Add new row at end of table
                    D.SS_Out_CT.(D.PAR.ratLab) = ...
                        [D.SS_Out_CT.(D.PAR.ratLab); D.SS_Out_CT.(D.PAR.ratLab)(end,:)];
                    
                    % Save out data to D.SS_Out_CT
                    D.SS_Out_CT.(D.PAR.ratLab).Include(end) = true;
                    D.SS_Out_CT.(D.PAR.ratLab).Date{end} = ...
                        datestr(D.PAR.ses_str_tim, 'yyyy-mm-dd_HH-MM-SS', 'local');
                    D.SS_Out_CT.(D.PAR.ratLab).Start_Time{end} = ...
                        datestr(D.PAR.ses_str_tim, 'HH:MM:SS');
                    D.SS_Out_CT.(D.PAR.ratLab).Total_Time(end) = ...
                        etime(D.PAR.ses_end_tim ,D.PAR.ses_str_tim) / 60;
                    D.SS_Out_CT.(D.PAR.ratLab).Track(end) = true;
                    D.SS_Out_CT.(D.PAR.ratLab).Forage(end) = false;
                    D.SS_Out_CT.(D.PAR.ratLab).Session_CT_T(end) = D.PAR.sesNum;
                    D.SS_Out_CT.(D.PAR.ratLab).Session_CT_F(end) = NaN;
                    D.SS_Out_CT.(D.PAR.ratLab).Lap_TS{end} = ...
                        D.PAR.TSmat(2,~isnan(D.PAR.TSmat(2,:)));
                    D.SS_Out_CT.(D.PAR.ratLab).Reward_TS{end} = ...
                        D.PAR.TSmat(1,~isnan(D.PAR.TSmat(1,:)));
                    D.SS_Out_CT.(D.PAR.ratLab).Reversal_Start_TS{end} = ...
                        D.PAR.TSmat(3,~isnan(D.PAR.TSmat(3,:)));
                    D.SS_Out_CT.(D.PAR.ratLab).Reversal_End_TS{end} = ...
                        D.PAR.TSmat(4,~isnan(D.PAR.TSmat(4,:)));
                    D.SS_Out_CT.(D.PAR.ratLab).Laps_Total(end) = D.PAR.CntMat(2)-1;
                    D.SS_Out_CT.(D.PAR.ratLab).Rewards_Total(end) = D.PAR.CntMat(1);
                    D.SS_Out_CT.(D.PAR.ratLab).Reversals_Total(end) = D.PAR.CntMat(3);
                    
                    % Update SS_In_All
                    D.SS_In_All.Session_CT_T(D.PAR.ratInd) = D.PAR.sesNum;
                    
                    % Save out data
                    SS_Out_CT = D.SS_Out_CT;
                    save(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_Out_CT.mat'), 'SS_Out_CT');
                    SS_In_All = D.SS_In_All;
                    save(fullfile(D.DIR.ioTop, D.DIR.ssSD, 'SS_In_All.mat'), 'SS_In_All');
                    
                    % Indicate session saved
                    D.PAR.saved = true;
                    
                    % Change and disable button
                    set(D.UI.btnStrtSes, 'STRING', 'DONE',...
                        'Enable', 'Off');
            
                case 'No'
                    return
                    
            end
            
        end
        
        % Save out handle data
        guidata(D.UI.figH,D);
    end

% QUIT
    function [] = BtnQuitSes(~, ~, ~)
        % Get handle data
        D = guidata(D.UI.figH);
        
        if ~D.PAR.saved
            % Construct a questdlg with two options
            % Note: based on built in function
            choice = questdlgAWL('QUIT WITHOUT SAVING?', ...
                'QUIT?', 'Yes', 'No', [], 'No', D.UI.qstDlfPos);
            % Handle response
            switch choice
                case 'Yes'
                case 'No'
                    return
            end
        end
        
        % stop acquiring
        D.PAR.acquire = false;
        
        % set to quit
        D.PAR.quit = true;
        
        % Save out handle data
        guidata(D.UI.figH,D);
        
    end

end