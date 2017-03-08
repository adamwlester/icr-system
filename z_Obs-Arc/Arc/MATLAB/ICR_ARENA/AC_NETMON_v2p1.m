function[] = AC_NETMON()

% NOTE: packet contains [1,10] numiric data
%   val 1 = conection [0, 1], [no, yes]
%   val 2 = display image [0, 1, 2], [Close all, 0-deg, 40-deg]
%   val 3 = rotation direction [-1, 1], [ACW, CW]
%   val 4 = feeder cue stimuli [0, 1, 2], [None or stop, Cue Feed C1, Cue Feed C2]
%   val 5 = right chan state [0, 1, 2], [no sound, no noise, noise]
%   val 6 = left chan state [1, 2], [reward tone, aversive sound]
%   val 7:10 = unused

%% Set tcip then run indefinaitely

% Network paramiters
DgLynxIP = '172.17.0.2';
% number of data packets
NDat = 10;
% buffer size based on recieved packets
tcpipBuff = NDat;

% Setup network connection object
tcpIP = tcpip(DgLynxIP,30000,'NetworkRole','Client');
% sets buffer size
set(tcpIP,'InputBufferSize',tcpipBuff);
% sets data transfer to zero
set(tcpIP,'Timeout',0);

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
% tone signal (2500 Hz)
reward_lft = sin(2*pi*(1/nsfs:1/nsfs:nsmin*60)*2500)' * 0.125;

% Sound object
soundH = audioplayer([reward_lft,noise_rt], nsfs);

% Start sound
%play(soundH);

%% Loop indefinitely
while true
    
    %% Set/reinitialize paramiters
    % rate of cue flicker (sec)
    flckrRt = 0.5;
    % delay between trigger and image (sec)
    trgDel = 0.1;
    
    % Wall image paramiters
    % NOTE: fiIn trigger images will trigger the follwing phototransducers:
    %   Img1 & Img2: N, S, E, W
    % image directory
    imgDir = ...
        '\\Icr_cheetah\icr_arena\ArenaControl\WallImages';
    % subdirectories
    subDir{1} = 'Main';
    subDir{2} = 'Trigger';
    % files
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
        fprintf('\r\n...CHECKING CONNECTION: %s\r\n', ...
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
            
            % Break out of loop
            break
        end
        
        %% EXECUTE CHANGES WITH NEW INCOMING DATA
        
        % STOP/START/RESTART SOUND
        % Start sound
        if ~isplaying(soundH)
            play(soundH);
            % Restart sound
        elseif etime(clock, t_sound) >= 60*floor(nsmin)
            % stop whatever was playing
            stop(soundH);
            % restart existing sound
            play(soundH);
        end
        
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
            fprintf('\rset file order: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        end
        
        % Display main image
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
            fprintf('\rdisplay main image: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        end
        
        % Bring java window back to front
        if exist('frame_java', 'var')
            frame_java.setState(frame_java.NORMAL)
        end
        
    end
    
end
