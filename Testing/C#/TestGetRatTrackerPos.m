function[] = TestGetRatTrackerPos()
%% Save a csv file with rat pos data in radians
% will be read inot C# myStreamNLX and sent to arduino for PID controller
% testing
% NOTE: be sure to start and end in the east quadrant at 0 deg crossing

outDir = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\Code\C#\Testing';

%% Set up NLX
% NOTE: Comment all aquisition entity configs as well as
% "-ProcessConfigurationFile VideoTracker.cfg" in config.cfg and use
% TestTracker.cfg.

% VT acq ent name
NLX.vt_acq_ent = 'VT1';

% Check if Cheetah is already running and if not, run Cheetah.exe
% Start Cheetah
[~,result] = system('tasklist /FI "imagename eq cheetah.exe" /fo table /nh');
notrun = strfind(result, 'INFO');
if notrun == 1
    cd('C:\Program Files\Neuralynx\Cheetah5');
    system(fullfile(['Cheetah.exe "' 'C:\Program Files\Neuralynx\Cheetah5\Configuration'], ['Cheetah.cfg"','&']));
    %'C:\WIndows\system32\cmexe'
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

%% Get tracker data
pause(5)
NlxSendCommand('-StartAcquisition');

% Loop for 2 min
maxTim = 60; % (sec)
vtPos = NaN(1,3600*2);
vtTS = NaN(1,3600*2);
nRecs = 0;
t1 = clock;
while etime(clock, t1) < maxTim
    ind = find(isnan(vtPos), 1, 'first');
    % request all new VT data that has been acquired since the last pass.
    [~, vtts, vtpos, ~, vtNRecs, ~] = NlxGetNewVTData(NLX.vt_acq_ent);
    nRecs = nRecs + vtNRecs;
    vtPos(ind:(ind+length(vtpos))-1) = vtpos;
    vtTS(ind:(ind+length(vtts))-1) = vtts;
    pause(5)
end

NlxSendCommand('-StopAcquisition');

vtPos = vtPos(~isnan(vtPos));
vtTS = vtTS(~isnan(vtTS))';

succeeded = 0;
while succeeded ~= 1
    succeeded = NlxDisconnectFromServer();
end
%% Process data

% reshape data
VT.xy_pos = reshape(single(vtPos),2,[]);
x = VT.xy_pos(1,:)';
% rescale y pixels
y = VT.xy_pos(2,:)';
% convert ts to double
ts = vtTS;

% Interp bad samples
badSamps = find(diff(x)>5 == 1 | diff(y)>5 == 1);
badSamps = sort([badSamps; badSamps-1]);
x(badSamps) = NaN;
y(badSamps) = NaN;
% x
xx=x;
bd=isnan(x);
gd=find(~bd);
bd([1:(min(gd)-1) (max(gd)+1):end])=0;
xx(bd)=interp1(gd,x(gd),find(bd));
x = xx;
% y
yy=y;
bd=isnan(y);
gd=find(~bd);
bd([1:(min(gd)-1) (max(gd)+1):end])=0;
yy(bd)=interp1(gd,y(gd),find(bd));
y = yy;

% Load into matrix (ts, x, y)
csvMat = [ts, x, y];
csvMat = csvMat(350:end,:);

x = csvMat(:,2);
y = csvMat(:,3);
y = y*1.0976;
%[XC, YC, R] = circfit(x,y);
%XC = double(XC); YC = double(YC); R = double(R);
XC = 359.5553;
YC = 260.2418;
R = 179.4922;

% Get nnormalized pos data
x_norm = (x-XC)./R;
y_norm = (y-YC)./R;

y_norm = y_norm*-1;

% Get position in radians
rad = atan2(y_norm, x_norm);

% Make sure data starts and end at east quad
strInd = find(abs(diff(wrapTo2Pi(rad)))>(2*pi*0.9), 1, 'first')+1;
endInd = find(abs(diff(wrapTo2Pi(rad)))>(2*pi*0.9), 1, 'last')-1;
csvMat = csvMat(strInd:endInd,:);

% Add header with number of samples
csvMat = [[size(csvMat,1),NaN,NaN]; csvMat];

% Save out track bound info
fiName = 'simRatPos.csv';
dlmwrite(fullfile(outDir, fiName),csvMat,'delimiter',',','precision',15,'newline','pc')


% Check data
plot(csvMat(:,2),csvMat(:,3))
hold on
plot(csvMat(2,2),csvMat(2,3),'og')
plot(csvMat(end,2),csvMat(end,3),'or')

% for i = 1:1:length(rad)
%     polar(rad(i),1,'.r');
%     pause(0.01)
% end

%% Create fixed velocity datasets
vel = 0;
dt = 33;

nSamp = round(((140*pi)/vel)/(dt/1000));
ts = round(linspace(0,(nSamp*33*1000), nSamp))';
rad = linspace(2*pi - ((2*pi)/(nSamp*2)),(2*pi)/(nSamp*2), nSamp)';
rad = wrapTo2Pi(rad);
roh = ones(nSamp,1)*0.9985;
[x,y] = pol2cart(rad,roh);
y = y*-1;
x = (x.*R)+XC;
y = (y.*R)+YC;
y = y/1.0976;
csvMat = [ts, x, y];
csvMat = [[nSamp,NaN,NaN]; csvMat];
% Save out track bound info
fiName = ['simConstVel_', num2str(vel), '.csv'];
dlmwrite(fullfile(outDir, fiName),csvMat,'delimiter',',','precision',15,'newline','pc')


