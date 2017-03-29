function[] = AC_NETMON()

% NOTE: packet contains [1,4] numiric data
%   val 1 = conection [0, 1], [no, yes] 
%   val 2 = display image [0, 1, 2], [Close all, 0-deg, 40-deg]
%   val 3 = rotation direction [-1, 1], [ACW, CW]
%   val 4 = sound state [0, 1], [no sound, sound]

%% Wall image paramiters
    
    % NOTE: fiIn trigger images will trigger the follwing phototransducers:
    %   Img1 & Img2: N, S, E, W
    % image directory
    imgDir = ...
        '\\ICR_CHEETAH\Study_ICR\ICR_Code\ICR_Running\IOfiles\Images\WallImages';
    % subdirectories
    subDir{1} = 'Main';
    subDir{2} = 'Trigger';

%% Set tcip then run indefinaitely

% Network paramiters
DgLynxIP = '172.17.0.2';
% number of data packets
NDat = 4;
% buffer size based on recieved packets
tcpipBuff = NDat;

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
reward_lft = reward_lft * 0.2;

% Sound object
soundH = audioplayer([reward_lft,noise_rt], nsfs);

% Start sound
%play(soundH);
%stop(soundH);

%% Loop indefinitely
while true
    
    %% Set/reinitialize paramiters
    
    % Setup network connection object
    tcpIP = tcpip(DgLynxIP,55000,'NetworkRole','Client');
    % sets buffer size
    set(tcpIP,'InputBufferSize',tcpipBuff);
    % sets data transfer to zero
    set(tcpIP,'Timeout',0.1);
    
    % delay between trigger and image (sec)
    trgDel = 0.1;
    
    % Image files
    fiIn = [{'Img1.bmp'}, {'Img2.bmp'}];
    
    % Initialize main vars
    % var to save sent data
    data = int8(zeros(1,NDat));
    % index of image to show
    img_ind = 0;
    % timer for adioplayer
    t_sound = clock;
    
    % Wait for other comp to open connection
    while (strcmp(tcpIP.Status, 'closed'))
        % Print attempt message
        fprintf('\r\n.........CHECKING CONNECTION: %s\r\n', ...
            datestr(now, 'HH:MM:SS AM'))
        try
             fopen(tcpIP);
        catch
        end
    end
    % Print status message
    fprintf('\r\nCONNECTED: %s\r\n', ...
        datestr(now, 'HH:MM:SS AM'))
    
    % Wait for first data packet
    while (tcpIP.BytesAvailable == 0)
    end
    
    
    %% WHILE CONNECTED
    while true
        
        % Check for new incoming data
        if tcpIP.BytesAvailable > 0
            data = fread(tcpIP,NDat,'int8');
        end
        
        % Check if connection should be closed
        if data(1) == 0
            % Print status message
            fprintf('\r\nCLOSING CONNECTION: %s\r\n', ...
                datestr(now, 'HH:MM:SS AM'))
            
            % Close image
            closescreenAWL
            
            % Stop noise
            if isplaying(soundH)
                stop(soundH)
            end
            
            % Close connection
            fclose(tcpIP);
            delete(tcpIP);
            clear tcpIP;
            
            % Break out of loop
            break
        end
        
        %% EXECUTE CHANGES WITH NEW INCOMING DATA
        
        % CHANGE FILE ORDER
        % NOTE: for rats with CW rotation, image files are flipped
        if ...
                data(1) == 1 && ...
                data(3) ~= 0
            
            % Flip file list
            if data(3) == 2
                fiIn = fliplr(fiIn);
            end
            
            % Set to zero so only run once
            data(3) = 0;
            
            % Print status message
            fprintf('\r\tset file order: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        end
        
        % UPDATE DISPLAY IMAGE
        if ...
                data(1) == 1 && ...
                data(2) ~= img_ind && ...
                data(2) ~= 0
            
            % Update current image
            img_ind = data(2);
            
            % Show image trigger
            frame_java = fullscreenAWL(fullfile(imgDir,subDir{2},fiIn{1,img_ind}));
            
            % Pause to allow trigger to register
            pause(trgDel);
            
            % Show main image
            frame_java = fullscreenAWL(fullfile(imgDir,subDir{1},fiIn{1,img_ind}));
            
            % Print status message
            fprintf('\r\tchange image: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        end
        
        % STOP/START/RESTART SOUND
        if data(4) == 1
            % Start sound
            if ~isplaying(soundH)
                play(soundH);
                t_sound = clock;
                % Print status message
                fprintf('\r\tstart sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
                % Restart sound
            elseif etime(clock, t_sound) >= 60*floor(nsmin)
                % stop whatever was playing
                stop(soundH);
                % restart existing sound
                play(soundH);
                t_sound = clock;
                % Print status message
                fprintf('\r\trestart sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
            end
        elseif isplaying(soundH)
            % stop whatever was playing
            stop(soundH)
            fprintf('\r\tstop sound: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
        end
        
        % Bring java window back to front
        if exist('frame_java', 'var')
            frame_java.setState(frame_java.NORMAL)
        end
        
    end
    
end
