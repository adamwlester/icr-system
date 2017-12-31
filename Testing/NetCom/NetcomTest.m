%% PARAMETERS

% DygitalLynx server IP
D.NLX.ServerIP = '192.168.3.100';

% Aquisition ent names
D.NLX.vt_rat_ent = 'VT1';
D.NLX.vt_rob_ent = 'VT2';
D.NLX.event_ent = 'Events';

%% START CHEETAH
   
% NLX setup
top_cfg_fi = 'Cheetah.cfg';
%top_cfg_fi = 'z_AWL_Test_Cheetah-Raw_Cube_Playback.cfg';

% Open Cheetah
curdir = pwd;
[~,result] = system('tasklist /FI "imagename eq cheetah.exe" /fo table /nh');
not_running = strfind(result, 'INFO');
if not_running == 1
    cd('C:\Program Files\Neuralynx\Cheetah');
    system(fullfile('Cheetah.exe C:\Users\Public\Documents\Cheetah\Configuration\', [top_cfg_fi,'&']));
end

%% CONNECT TO NETCOM

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

%% LOAD CONFIG FILE

% Load behavior tracking config
%   NlxSendCommand('-ProcessConfigurationFile AWL-ICR_Behavior_Tracking.cfg');

% Load implant tracking config
%   NlxSendCommand('-ProcessConfigurationFile AWL-ICR_Ephys_Tracking.cfg');

% Load ephys setup
%   NlxSendCommand('-ProcessConfigurationFile AWL-ICR_Ephys_Setup.cfg');

% Load ephys graphics
%   NlxSendCommand('-ProcessConfigurationFile AWL-ICR_Ephys_Graphics.cfg');

% Window pos
%   cfg_fi = 'AWL-WinPos_VT_M1.cfg';
%   cfg_fi = 'AWL-WinPos_VT_M3.cfg';
%   cfg_fi = 'AWL-WinPos_Ephys_M1.cfg';
%   cfg_fi = 'AWL-WinPos_Ephys_M3.cfg';
%   NlxSendCommand(['-ProcessConfigurationFile ', cfg_fi])

%% OPEN STREAM

% Open the data stream for the VT acquisition entities
NlxOpenStream(D.NLX.vt_rat_ent);
NlxOpenStream(D.NLX.vt_rob_ent);
NlxOpenStream(D.NLX.event_ent);

%% CUBE SETUP

% Pair Cube headstage (Should be commented out unless router has been unpluged)
%NlxSendCommand('-SendLynxSXCommand AcqSystem1 -InitWHSPairing 30')

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




