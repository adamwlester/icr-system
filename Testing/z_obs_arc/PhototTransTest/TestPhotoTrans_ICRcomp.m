function[] = TestPhotoTrans_ICRcomp()

%% Set up NLX 

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

% Set up pins

% TTL board/port parameters
NLX.DevName = 'AcqSystem1_0'; % TTL board name

% Ports
NLX.port_photo = '2'; % input port for photo transducers
NLX.port_led = '3'; % input port for audio

% Photo transducer input pins
NLX.west_bit = '3'; % brown wire, dec = 8
NLX.north_bit = '2'; % green wire, dec = 4
NLX.east_bit = '1'; % brown wire, dec = 2
NLX.south_bit = '0'; % green wire, dec = 1

% LED output pins
NLX.led_on_bit = '5'; % dec = 32

% Set port direction
NlxSendCommand(['-SetDigitalIOportDirection ', NLX.DevName, ' ', NLX.port_photo, ' Input']);
NlxSendCommand(['-SetDigitalIOportDirection ', NLX.DevName, ' ', NLX.port_led, ' Output']);

% Enable digital io events
NlxSendCommand(['-SetDigitalIOEventsEnabled ', NLX.DevName, ' ', NLX.port_photo, ' True']);
NlxSendCommand(['-SetDigitalIOEventsEnabled ', NLX.DevName, ' ', NLX.port_led, ' True']);

% Define events
% photo sensor
% bit triggered event
NlxSendCommand(['-SetNamedTTLEvent ', NLX.DevName, ' ', NLX.port_photo, ' ', NLX.west_bit, ' West_On']);
NlxSendCommand(['-SetNamedTTLEvent ', NLX.DevName, ' ', NLX.port_photo, ' ', NLX.north_bit, ' North_On']);
NlxSendCommand(['-SetNamedTTLEvent ', NLX.DevName, ' ', NLX.port_photo, ' ', NLX.east_bit, ' East_On']);
NlxSendCommand(['-SetNamedTTLEvent ', NLX.DevName, ' ', NLX.port_photo, ' ', NLX.south_bit, ' South_On']);
% manual event
NLX.Event{1} = '-PostEvent West_Start 100 0'; % dec = 100
NLX.Event{2} = '-PostEvent North_Start 101 0'; % dec = 101
NLX.Event{3} = '-PostEvent East_Start 102 0'; % dec = 102
NLX.Event{4} = '-PostEvent South_Start 103 0'; % dec = 103
NLX.Event{5} = '-PostEvent Start_Test 200 0'; % dec = 200
NLX.Event{6} = '-PostEvent End_Test 201 0'; % dec = 201
% led
NlxSendCommand(['-SetNamedTTLEvent ', NLX.DevName, ' ', NLX.port_led, ' ', NLX.led_on_bit, ' LED_On']);

% LED command
NLX.LED_Pulse = ... 
    ['-DigitalIOTtlPulse ', NLX.DevName, ' ', NLX.port_led, ' ', NLX.led_on_bit, ' High'];

% Open stream for NLX events
NlxOpenStream('Events');

%% LED Test

% Start acquisition
NlxSendCommand('-StartAcquisition');

nPulse = 50;
del = 0.25;

timeStampArray = cell(4,1);
ttlValueArray = cell(4,1);
eventStringArray = cell(4,1);

% Start at west sensor and after 10 seconds light will pulse. When pulse
% stops you will have 10 seconds to move to next sensor (north) and so on
% clockwise around the arena
NlxSendCommand(NLX.Event{5}); % start test
pause(5)
for z_wall = 1:4
    pause(5);
    NlxSendCommand(['-SetDigitalIOPulseDuration ', NLX.DevName, ' ', NLX.port_led, ' ', '100']);
    NlxSendCommand(NLX.Event{z_wall});
    pause(0.5);
    for z_pulse = 1:nPulse
        if z_pulse == nPulse
            NlxSendCommand(['-SetDigitalIOPulseDuration ', NLX.DevName, ' ', NLX.port_led, ' ', '2000']);
        end
        NlxSendCommand(NLX.LED_Pulse);
        pause(del);
    end
    if z_wall == 4
        NlxSendCommand(NLX.Event{6}); % end test
        pause(1)
    end
    % save events
    [~, timeStampArray{z_wall}, ~, ttlValueArray{z_wall}, eventStringArray{z_wall}, ~, ~] = NlxGetNewEventData('Events');
end

NlxSendCommand('-StopAcquisition')

%% Calculate sensor timing

outDir = 'C:\Users\lester\Documents\Research\BarnesLab\Projects\Arena\ArenaControl\Testing\PhototTransTest\WithArduino5';

% dec values
wall_dec = [8,4,2,1]; 
led_dec = 32;

LEDts = cell(4,1);
PTts = cell(4,1);
for z_wall = 1:4
LEDts{z_wall} = timeStampArray{z_wall}(ttlValueArray{z_wall} == led_dec);
PTts{z_wall} = timeStampArray{z_wall}(ttlValueArray{z_wall} == wall_dec(z_wall));
end

fig1 = figure('Position', [10 10 1500 1000]); movegui('center') 
% west
subplot(2,2,1); title('West'); hold on; grid on
plot((LEDts{1} - min(LEDts{1}))/1000, 'd', 'Color', [0.5 0 0]); 
plot((PTts{1} - min(LEDts{1}))/1000, 'd', 'Color', [1 0 0]); 
set(gca, 'YTick', linspace(0, max(get(gca,'YTick')), 40), ...
    'XTick', 1:max(get(gca,'XTick')), 'FontSize', 6)
% north
subplot(2,2,2); title('North'); hold on; grid on
plot((LEDts{2} - min(LEDts{2}))/1000, 'd', 'Color', [0 0.5 0]); 
plot((PTts{2} - min(LEDts{2}))/1000, 'd', 'Color', [0 1 0]); 
set(gca, 'YTick', linspace(0, max(get(gca,'YTick')), 40), ...
    'XTick', 1:max(get(gca,'XTick')), 'FontSize', 6)
% east
subplot(2,2,3); title('East'); hold on; grid on
plot((LEDts{3} - min(LEDts{3}))/1000, 'd', 'Color', [0 0 0.5]); 
plot((PTts{3} - min(LEDts{3}))/1000, 'd', 'Color', [0 0 1]); 
set(gca, 'YTick', linspace(0, max(get(gca,'YTick')), 40), ...
    'XTick', 1:max(get(gca,'XTick')), 'FontSize', 6)
% south
subplot(2,2,4); title('South'); hold on; grid on
plot((LEDts{4} - min(LEDts{4}))/1000, 'd', 'Color', [0.5 0 0.5]); 
plot((PTts{4} - min(LEDts{4}))/1000, 'd', 'Color', [1 0 1]); 
set(gca, 'YTick', linspace(0, max(get(gca,'YTick')), 40), ...
    'XTick', 1:max(get(gca,'XTick')), 'FontSize', 6)

% get matching pt values
ptind{1} = knnsearch(PTts{1}',LEDts{1}');
ptind{2} = knnsearch(PTts{2}',LEDts{2}');
ptind{3} = knnsearch(PTts{3}',LEDts{3}');
ptind{4} = knnsearch(PTts{4}',LEDts{4}');

% Get distrebution of diff values
fig2 = figure('Position', [10 10 1500 1000]); movegui('center') 
% west
diffs = PTts{1}(ptind{1}) - LEDts{1};
[N,edges] = histcounts(diffs,5);
subplot(2,2,1); title('West'); hold on; grid on
bar(linspace(min(edges), max(edges), length(N)),N, 'FaceColor', [0.5 0 0])
set(gca, 'YTick', 0:max(get(gca,'YTick')))
% north
diffs = PTts{2}(ptind{2}) - LEDts{2};
[N,edges] = histcounts(diffs,5);
subplot(2,2,2); title('North'); hold on; grid on
bar(linspace(min(edges), max(edges), length(N)),N, 'FaceColor', [0 0.5 0])
set(gca, 'YTick', 0:max(get(gca,'YTick')))
% east
diffs = PTts{3}(ptind{3}) - LEDts{3};
[N,edges] = histcounts(diffs,5);
subplot(2,2,3); title('East'); hold on; grid on
bar(linspace(min(edges), max(edges), length(N)),N, 'FaceColor', [0 0 0.5])
set(gca, 'YTick', 0:max(get(gca,'YTick')))
% south
diffs = PTts{4}(ptind{4}) - LEDts{4};
[N,edges] = histcounts(diffs,5);
subplot(2,2,4); title('South'); hold on; grid on
bar(linspace(min(edges), max(edges), length(N)),N, 'FaceColor', [0.5 0 0.5])
set(gca, 'YTick', 0:max(get(gca,'YTick')))

% plot distrebution of difference values
sprintf('\nWest:  %0.2f\n\nNorth: %0.2f\n\nEast:  %0.2f\n\nSouth: %0.2f\n\n', ...
    mean(PTts{1}(ptind{1}) - LEDts{1}), mean(PTts{2}(ptind{2}) - LEDts{2}),...
    mean(PTts{3}(ptind{3}) - LEDts{3}),mean(PTts{4}(ptind{4}) - LEDts{4}))

% Save
save(fullfile(outDir,'Fig1.fig'), 'fig1')
export_fig(fig1, fullfile(outDir,'Fig1.png'))
save(fullfile(outDir,'Fig2.fig'), 'fig2')
export_fig(fig2, fullfile(outDir,'Fig2.png'))
save(fullfile(outDir,'Data.mat'),'LEDts', 'PTts') 

%% Projected Trigger test
% Run with TestPhotoTrans_ACcomp ARENACONTROLLER computer

% Run TestPhotoTrans_ACcomp

% save events
[~, timeStampArray, ~, ttlValueArray, eventStringArray, ~, ~] = NlxGetNewEventData('Events');

%wall_dec = [8,4,2,1]; 
wall_str = [{'West_On'}, {'North_On'}, {'East_On'}, {'South_On'}];

PTts = cell(4,1);
for z_wall = 1:4
PTts{z_wall} = timeStampArray(ismember(eventStringArray, wall_str{z_wall}));
end

figure; axes; hold on
plot((PTts{1}-  timeStampArray(1))/10^6, 'o', 'Color', [0.5 0 0], 'MarkerSize', 3)
plot((PTts{2}-  timeStampArray(1))/10^6, 'o', 'Color', [0 0.5 0], 'MarkerSize', 6)
plot((PTts{3}-  timeStampArray(1))/10^6, 'o', 'Color', [0 0 0.5], 'MarkerSize', 9)
plot((PTts{4}-  timeStampArray(1))/10^6, 'o', 'Color', [0.5 0 0.5], 'MarkerSize', 12)
legend('west', 'north', 'east', 'south', 'Location', 'SouthEast')
ylabel('Time (sec)'); xlabel('Event')

