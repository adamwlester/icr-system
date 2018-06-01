%% PARAMETERS

% DygitalLynx server IP
D.NLX.ServerIP = '192.168.3.100';

% Aquisition ent names
D.NLX.vt_rat_ent = 'VT1';
D.NLX.vt_rob_ent = 'VT2';
D.NLX.event_ent = 'Events';

% TT objects
D.TT.ttList = { ...
    'TT01', ...
    'TT02', ...
    'TT03', ...
    'TT04', ...
    'TT05', ...
    'TT06', ...
    'TT07', ...
    'TT08', ...
    'TT09', ...
    'TT10', ...
    'TT11', ...
    'TT12', ...
    'TT13', ...
    'TT14', ...
    'TT15', ...
    'TT16' ...
    };

%% START CHEETAH WITH SPECIFIED CONFIG
   
% NLX setup
%cfg_sub_dir = 'Cheetah.cfg';
%cfg_sub_dir = 'testing\Cheetah_Raw_Playback.cfg';
cfg_sub_dir = 'ICR_Cheetah_Ephys.cfg';

% Get path to test config directory
cfg_path = fullfile('C:\Users\Public\Documents\cheetah\configuration',cfg_sub_dir);

% Open Cheetah
curdir = pwd;
[~,result] = system('tasklist /FI "imagename eq cheetah.exe" /fo table /nh');
not_running = strfind(result, 'INFO');
if not_running == 1
    cd('C:\Program Files\Neuralynx\Cheetah');
    system(['Cheetah.exe ', cfg_path, '&']);
    cd(curdir)
end

%% CONNECT TO NETCOM
succeeded = NlxDisconnectFromServer()
% Connect
D.NLX.connected = false;
if NlxAreWeConnected() ~= 1
    
    % Keep attempting till success
    while ~D.NLX.connected
        
        % Attempt connection
        D.NLX.connected = NlxConnectToServer(D.NLX.ServerIP) == 1;
        
        % Identify id to server
        if D.NLX.connected
            NlxSetApplicationName('ICR_GUI');
        end
        
    end
end

%% LOAD BEHAVIOR FILES


% Load behavior tracking config
%   NlxSendCommand('-ProcessConfigurationFile ICR_Behavior_Tracking.cfg');

% Window pos
%   cfg_fi = 'ICR_WinPos_Behavior_M1.cfg';
%   cfg_fi = 'ICR_WinPos_Behavior_M3.cfg';
%   NlxSendCommand(['-ProcessConfigurationFile ', cfg_fi])

%% LOAD CONFIG FILE


% Load implant tracking config
   NlxSendCommand('-ProcessConfigurationFile ICR_Ephys_Tracking.cfg');

% Load ephys setup
   NlxSendCommand('-ProcessConfigurationFile ICR_Ephys_Setup.cfg');

% Load ephys graphics
   NlxSendCommand('-ProcessConfigurationFile ICR_Ephys_Graphics.cfg');

% Window pos
   cfg_fi = 'ICR_WinPos_Ephys_M1.cfg';
%   cfg_fi = 'ICR_WinPos_Ephys_M3.cfg';
   NlxSendCommand(['-ProcessConfigurationFile ', cfg_fi])

%% OPEN STREAM

% Get list of DAS objects
[succeeded, dasObjects, dasTypes] = NlxGetDASObjectsAndTypes();

% VT 
NlxOpenStream(D.NLX.vt_rat_ent);
NlxOpenStream(D.NLX.vt_rob_ent);

% Events
NlxOpenStream(D.NLX.event_ent);

% TT
for z_tt = 1:length(D.TT.ttList)
    succeeded = NlxOpenStream(D.TT.ttList{z_tt});
    if succeeded == 1
        fprintf('SUCCEEDED NlxOpenStream(%s)\n', D.TT.ttList{z_tt});
    else
        fprintf('!!FAILED!! NlxOpenStream(%s)\n', D.TT.ttList{z_tt});
    end
    pause(0.1);
end

%% CUBE SETUP

% Pair Cube headstage (Should be commented out unless router has been unpluged)
%NlxSendCommand('-SendLynxSXCommand AcqSystem1 -InitWHSPairing 30')

%TEMP
% Check if cube sending data
NlxSendCommand('-SendLynxSXCommand -GetDataTransmissionEnabled AcqSystem1')
NlxSendCommand('-SendLynxSXCommand -GetHardwareSubSystemInformation "AcqSystem1"')

% Turn on Cube LEDs
NlxSendCommand('-SendLynxSXCommand AcqSystem1 -WHSSetTrackingLED 1 1');
% Turn off Cube LEDs
%NlxSendCommand('-SendLynxSXCommand AcqSystem1 -WHSSetTrackingLED 1 0');

%% ACQUIRE AND RECORD

% Start Acquisition
NlxSendCommand('-StartAcquisition');

% Start Recording
%NlxSendCommand('-StartRecording');

% Stop Acquisition
%NlxSendCommand('-StopAcquisition');

% Stop Recording
%NlxSendCommand('-StopRecording');

%% GET CUBE VCC

% Set Cube battery type
%   Note:
%       "C": Small (green dot)
%       "A": Medium (blue dot)
NlxSendCommand('-SetBatteryType AcqSystem1 "A"');

[a,b] = NlxSendCommand('-SendLynxSXCommand AcqSystem1 -WHSGetStateOfCharge 1');
disp(b)

while true
    pause(1);
    [a,b] = NlxSendCommand('-SendLynxSXCommand AcqSystem1 -WHSGetStateOfCharge 1');
    disp(b)
end

%% Disconnect from the NLX server
NlxDisconnectFromServer();

%% AUDIO
% 
% % Audio left
% NlxSendCommand('-SetAudioSource "Primary Sound Driver" Right None');
% NlxSendCommand('-SetAudioSource "Primary Sound Driver" Left TT02');
% NlxSendCommand('-SetAudioStreamEnabled "Primary Sound Driver" Left 0 true');
% NlxSendCommand('-SetAudioStreamEnabled "Primary Sound Driver" Left 1 false');
% NlxSendCommand('-SetAudioStreamEnabled "Primary Sound Driver" Left 2 false');
% NlxSendCommand('-SetAudioStreamEnabled "Primary Sound Driver" Left 3 false');
% 
% % Audio right
% NlxSendCommand('-SetAudioSource "Primary Sound Driver" Left None');
% NlxSendCommand('-SetAudioSource "Primary Sound Driver" Right TT01');
% NlxSendCommand('-SetAudioStreamEnabled "Primary Sound Driver" Right 0 true');
% NlxSendCommand('-SetAudioStreamEnabled "Primary Sound Driver" Right 1 false');
% NlxSendCommand('-SetAudioStreamEnabled "Primary Sound Driver" Right 2 false');
% NlxSendCommand('-SetAudioStreamEnabled "Primary Sound Driver" Right 3 false');
% 
% 
% NlxSendCommand('-SetAudioVolume "AcqSystem1_Audio0" Left 100');
% NlxSendCommand('-SetAudioMute "AcqSystem1_Audio0" Left Off');
% NlxSendCommand('-SetAudioSource "AcqSystem1_Audio0" Right None');
% NlxSendCommand('-SetAudioVolume "AcqSystem1_Audio0" Right 100');
% NlxSendCommand('-SetAudioMute "AcqSystem1_Audio0" Right Off');




