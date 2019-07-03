function[] = AC_NETMON% NOTE: packet contains [1,4] numiric data()


%   val 1 = conection [0, 1], [no, yes]
%   val 2 = display image [0, 1, 2, 3], [Close all, 0-deg, -40-deg, 40-deg]
%   val 3 = sound state [0, 1, 2], [no sound, white only, all sound]

%% Wall image paramiters

% NOTE: imgRunFiles trigger images will trigger the follwing phototransducers:
%   Img1 & Img2: N, S, E, W

% image directory
imgDir = ...
    '\\ICRCHEETAH\Study_ICR\ICR_Code\ICR_Running\IOfiles\Images\WallImages';

% Subdirectories
subDir{1} = 'Main';
subDir{2} = 'Trigger';

% Backround image stuff
dt_keepDarkImg = 30;
t_darkBackImg = clock;
t_lightBackImg = clock;

%% Set tcip then run indefinaitely

% Network paramiters
DgLynxIP = '172.17.0.2';
% number of data packets
NDat = 3;

% Turn warnings off
warning('off','all');

%% Generate noise data and handles

% White noise
% noise fs (Hz)
nsfs = 40*10^3;
% noise length (min)
nsmin = 3.1;
% noise signal
noise_rt = rand(nsfs*nsmin*60,1) * 0.1;

% Reward tone
% tone signal blend
% reward_lft = sin(2*pi*(1/nsfs:1/nsfs:nsmin*60)*2500)';
t = 1/nsfs:1/nsfs:nsmin*60;
s1 = sin(2*pi*t*1000)';
s2 = sin(2*pi*t*2500)';
s3 = sin(2*pi*t*5000)';
s4 = sin(2*pi*t*10000)';
reward_lft = (s1 + s2 + s3 + s4) / 4;
% Set gain/amplitude
rew_gain = 0.3;
reward_lft = reward_lft * rew_gain;

% Sound object
soundAllH = audioplayer([reward_lft,noise_rt], nsfs);
soundWhiteH = audioplayer([noise_rt,noise_rt], nsfs);

% Start sound
%

%stop(soundH);

%% Loop indefinitely
tic;
while true
    
    %% Set/reinitialize paramiters
    
    % Delay between trigger and image (sec)
    trgDel = 0.1; %#ok<*NASGU>
    
    % Backround image files
    imgBackFiles = [{'back_light.bmp'}, {'back_dark.bmp'}];
    
    % Main image files
    imgRunFiles = [{'Img1.bmp'}, {'Img2.bmp'}, {'Img3.bmp'}];
    
    % Initialize main vars
    % var to save sent data
    data = int8(zeros(1,NDat));
    % index of image to show
    img_ind = 0;
    % timer for adioplayer
    t_sound = clock;
    
    % Put up light backround
    if ~exist('frame_java', 'var') || ...
            (etime(clock, t_darkBackImg) >= dt_keepDarkImg && ...
            etime(t_darkBackImg, t_lightBackImg) > 0)
        
        % Show image and save time
        frame_java = fullscreenAWL(fullfile(imgDir,subDir{1},imgBackFiles{1}));
         t_lightBackImg = clock;
         
        % Print status message
        fprintf('\r\tchange image to \"%s\": %s\r', ...
            imgBackFiles{1}, datestr(now, 'HH:MM:SS AM'))
    end
    
    % Bring java window back to front
    if exist('frame_java', 'var') && toc > 10
        frame_java.setState(frame_java.NORMAL)
        tic;
    end
    
    % Wait for other comp to open connection
    if ~exist('tcpIP', 'var')
        try
            tcpIP = tcpclient( ...
                DgLynxIP, 55000, ...
                'ConnectTimeout', 1, ...
                'Timeout', 1);
        catch
            % Print attempt message
            fprintf('\r\n.........WAITING FOR CONNECTION: %s\r\n', ...
                datestr(now, 'HH:MM:SS AM'))
            % Return to top
            continue;
        end
        
        % Print status message
        fprintf('\r\nCONNECTED: %s\r\n', ...
            datestr(now, 'HH:MM:SS AM'))
        
        % Put up dark backround
        frame_java = fullscreenAWL(fullfile(imgDir,subDir{1},imgBackFiles{2}));
        
        % Print status message
        fprintf('\r\tchange image to \"%s\": %s\r', ...
            imgBackFiles{2}, datestr(now, 'HH:MM:SS AM'))
    end
    
    % Wait for first data packet
    if (tcpIP.BytesAvailable == 0) %#ok<*NODEF>
        % Return to top
        continue;
    end
    
    
    %% WHILE CONNECTED
    while true
        
        % Check for new incoming data
        if tcpIP.BytesAvailable > 0
            data = read(tcpIP,NDat,'int8');
        end
        
        % Check if connection should be closed
        if data(1) == 0
            % Print status message
            fprintf('\r\nCLOSING CONNECTION: %s\r\n', ...
                datestr(now, 'HH:MM:SS AM'))
            
            % Put up dark backround
            frame_java = fullscreenAWL(fullfile(imgDir,subDir{1},imgBackFiles{2}));
            t_darkBackImg = clock;
            
            % Print status message
            fprintf('\r\tchange image to \"%s\": %s\r', ...
                imgBackFiles{2}, datestr(now, 'HH:MM:SS AM'))
            
            % Stop noise
            if isplaying(soundAllH)
                stop(soundAllH)
            end
            
            % Close connection
            delete(tcpIP);
            clear tcpIP;
            
            % Break out of loop
            break
        end
        
        %% EXECUTE CHANGES WITH NEW INCOMING DATA
        
        % UPDATE DISPLAY IMAGE
        if ...
                data(1) == 1 && ...
                data(2) ~= img_ind && ...
                data(2) ~= 0
            
            % Update current image
            img_ind = data(2);
            
            % Show image trigger
            frame_java = fullscreenAWL(fullfile(imgDir,subDir{2},imgRunFiles{img_ind}));
            
            % Pause to allow trigger to register
            pause(trgDel);
            
            % Show main image
            frame_java = fullscreenAWL(fullfile(imgDir,subDir{1},imgRunFiles{img_ind}));
            
            % Print status message
            fprintf('\r\tchange image to \"%s\": %s\r', ...
                imgRunFiles{img_ind}, datestr(now, 'HH:MM:SS AM'))
            
        end
        
        % STOP/START/RESTART SOUND
        
        % Stop all sound
        if data(3) == 0
            
            if isplaying(soundAllH)
                stop(soundAllH)
            elseif isplaying(soundWhiteH)
                stop(soundWhiteH)
            end
            fprintf('\r\tstop sound: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        end
        
        % Play white only
        if data(3) == 1
            
            % Start sound
            if ~isplaying(soundWhiteH)
                play(soundWhiteH);
                t_sound = clock;
                % Print status message
                fprintf('\r\tstart white only sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
            end
            
            % Restart sound
            if etime(clock, t_sound) >= 60*floor(nsmin)
                % stop whatever was playing
                stop(soundWhiteH);
                % restart existing sound
                play(soundWhiteH);
                t_sound = clock;
                % Print status message
                fprintf('\r\trestart white only sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
            end
            
        end
        
        % Play all sound
        if data(3) == 2
            
            % Start sound
            if ~isplaying(soundAllH)
                play(soundAllH);
                t_sound = clock;
                % Print status message
                fprintf('\r\tstart all sound sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
            end
            
            % Restart sound
            if etime(clock, t_sound) >= 60*floor(nsmin)
                % stop whatever was playing
                stop(soundAllH);
                % restart existing sound
                play(soundAllH);
                t_sound = clock;
                % Print status message
                fprintf('\r\trestart all sound sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
            end
            
        end
        
        % Bring java window back to front
        if exist('frame_java', 'var')
            frame_java.setState(frame_java.NORMAL)
        end
        
    end
    
end
