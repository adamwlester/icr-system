function[] = TestGetTrackBounds_ICRcomp()

outDir = 'C:\Users\Lester\MeDocuments\Research\BarnesLab\Study_ICR\MATLAB\IOfiles\Operational';
fiName = 'TrackBounds.mat';

%% Set up NLX 
% NOTE: Comment all aquisition entity configs as well as 
% "-ProcessConfigurationFile VideoTracker.cfg" in config.cfg and use
% TestTracker.cfg. 

% VT acq ent name
NLX.vt_acq_ent = 'VT2';

top_cfg_fi = 'Cheetah.cfg'; % top level config file to load
% Check if Cheetah is already running and if not, run Cheetah.exe
% Start Cheetah
[~,result] = system('tasklist /FI "imagename eq cheetah.exe" /fo table /nh');
notrun = strfind(result, 'INFO');
if notrun == 1
    cd('C:\Program Files (x86)\Neuralynx\Cheetah5');
    system(fullfile('Cheetah.exe C:\Program Files\Neuralynx\Cheetah5\Configuration\', [top_cfg_fi,'&']));
    %'C:\WIndows\system32\cmd.exe'
end

% Connect to Cheetah via NetCom
fprintf('\r\nAttempting connection\r\n');
succeeded = 0;
while succeeded ~= 1 
succeeded = NlxConnectToServer('192.168.3.100');
end
if succeeded == 1
    fprintf('\r\nConnected to Cheetah\r\n');
    NlxSetApplicationName('MATLAB');
else
    fprintf('\r\nCheetah connection failed\r\n');
end


% Open the data stream for the VT acquisition entity.  This tells Cheetah to begin %streaming
% data for the VT acq ent.
NlxOpenStream(NLX.vt_acq_ent);

% Set thresholds
NlxSendCommand('-SetRedThreshold "VT1" 5');
NlxSendCommand('-SetBlueThreshold "VT1" 40');

%% Get tracker data

% Loop for 2 min
vtPos = NaN(1,3600*2);
nRecs = 0;
t1 = clock;
while etime(clock, t1) < 120
    ind = find(isnan(vtPos), 1, 'first');
    % request all new VT data that has been acquired since the last pass.
    [~, ~, vtpos, ~, vtNRecs, ~] = NlxGetNewVTData(NLX.vt_acq_ent);
    nRecs = nRecs + vtNRecs;
    vtPos(ind:(ind+length(vtpos))-1) = vtpos;
    pause(5)
end

vtPos = vtPos(~isnan(vtPos));
%% Process data

% reshape data
VT.xy_pos = reshape(single(vtPos),2,[]);
x = VT.xy_pos(1,:)';
% rescale y pixels
y = VT.xy_pos(2,:)'*1.0976;

[XC, YC, R] = circfit(x,y);
XC = double(XC); YC = double(YC); R = double(R);

% Save out track bound info
save(fullfile(outDir, fiName), 'XC', 'YC', 'R')

VT.xNorm = (x-XC)./R;
VT.yNorm = (y-YC)./R;
% Get position in radians
[VT.rad_pos,VT.roh_pos] = cart2pol(VT.xNorm, VT.yNorm);
% Flip radian values to acount for inverted y values from Cheetah
VT.rad_pos = abs(VT.rad_pos-2*pi);

polar(VT.rad_pos,VT.roh_pos); hold on;
for i = 1:length(VT.roh_pos)
    point = polar(VT.rad_pos(i),VT.roh_pos(i),'or');
    pause(0.1)
    delete(point)
end
