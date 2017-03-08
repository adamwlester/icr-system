function[] = AC_NETMON()

% NOTE: packet contains [1,10] numiric data
%   val 1 = conection [0, 1], [no, yes]
%   val 2 = display image [0, 1, 2], [Close all, 0-deg, 40-deg]
%   val 3 = rotation direction [-1, 1], [ACW, CW]
%   val 4 = feeder cue stimuli [0, 1, 2], [None or stop, Cue Feed C1, Cue Feed C2]
%   val 5 = right chan state [0, 1, 2], [no sound, no noise, noise]
%   val 6 = left chan state [1, 2], [reward tone, aversive sound]
%   val 7:10 = unused

%% Set paramiters

% Network paramiters
DgLynxIP = '172.17.0.2';
% number of data packets
NDat = 10;
% buffer size based on recieved packets
Bunsfsz = NDat*8;
% rate of cue flicker (sec)
flckrRt = 0.5;
% delay between trigger and image (sec)
trgDel = 0.1;

% Wall image paramiters
% NOTE: fiIn trigger images will trigger the follwing phototransducers:
%   Img1 & Img2: N, S, E, W
%   Img1_C1: N
%   Img1_C2: S
%   Img2_C1: W
%   Img2_C2: E
% image directory
imgDir = ...
    '\\ICR_CHEETAH\Study_ICR\MATLAB\ICR_ARENA\ArenaControl\WallImages';
% subdirectories
subDir{1} = 'Main';
subDir{2} = 'Trigger';
% files
fiIn = [{'Img1.bmp'}, {'Img2.bmp'}; ...
    {'Img1_C1.bmp'}, {'Img2_C1.bmp'}; ...
    {'Img1_C2.bmp'}, {'Img2_C2.bmp'}];

%% Generate noise data and handles

% White noise
% noise fs (Hz)
nsfs = 40*10^3;
% noise length (min)
nsmin = 1.1;
% noise signal
noise_rt = rand(nsfs*nsmin*60,1)/30;
% no noise signal
nonoise_rt = noise_rt/1000;

% Reward tone
% tone signal (2500 Hz for 1 sec)
reward_lft = sin(2*pi*(1/nsfs:1/nsfs:nsmin*60)*2500)'/4;
% object with noise signal
soundH{1}(1) = audioplayer([reward_lft,nonoise_rt], nsfs);
% object without noise signal
soundH{2}(1) = audioplayer([reward_lft,noise_rt], nsfs);

% Aversive tone
% sound signal (4000 Hz with 0.1 beep)
avrs_lft = sin(2*pi*(1/nsfs:1/nsfs:nsmin*60)*4000)'/1;
d = round(nsfs/10);
l = length(avrs_lft);
avrs_lft((1:l/2) + reshape(repmat(0:d:(l/2)-1,d,1),1,[])) = 0;
% object with noise signal
soundH{1}(2) = audioplayer([avrs_lft,nonoise_rt], nsfs);
% object without noise signal
soundH{2}(2) = audioplayer([avrs_lft,noise_rt], nsfs);

% Setup network connection object
tcpIP = tcpip(DgLynxIP,30000,'NetworkRole','Client');
% sets buffer size
set(tcpIP,'InputBufferSize',Bunsfsz);
% sets data transfer longer timeout
set(tcpIP,'Timeout',1);

% Initialize main vars
% ICR computer connection
connection = 0; % [true, false]
% var to save sent data
data = zeros(1,NDat);

% Set up java object


% Turn warnings off
warning('off','all');

%% Loop indefinitely
while true
    
    %% WHILE NOT CONNECTED
    while connection == 0 % attempt connection
        
        % Print attempt message
        fprintf('\r\nATTEMPT CONNECTION: %s\r\n', ...
            datestr(now, 'HH:MM:SS AM'))
        check = []; % reset
        
        % Try to open connection
        try
            fopen(tcpIP);
            check = fread(tcpIP,NDat,'double');
        catch
        end
        
        % Get data if any sent
        if ~isempty(check)
            
            % New data
            data = check;
            
            % Get connection status
            connection = check(1);
            
            % Print status message
            fprintf('\r\nCONNECTED: %s\r\n', ...
                datestr(now, 'HH:MM:SS AM'))
            
        else
            connection = 0;
        end
        
        % Re-initialize variables with new session
        if connection ==1
            
            % Set timeout to zero
            set(tcpIP,'Timeout',0); % sets data transfer timeout
            
            % Reinitialize
            % file list if flipped
            fiIn = [{'Img1.bmp'}, {'Img2.bmp'}; ...
                {'Img1_C1.bmp'}, {'Img2_C1.bmp'}; ...
                {'Img1_C2.bmp'}, {'Img2_C2.bmp'}];
            % index of image to show
            img_ind = 0;
            % index of image cue to show
            cue_ind = 0;
            % track if cue shoud be shown
            cue_fd = false;
            % track if cue showing during cue flicker
            cue_state = [0,1];
            % timer for cue flicker
            t_cue = clock;
            % timer for adioplayer
            t_sound = clock;
            % track right left chan noise state
            % Note: sound states initialize at -1 which is unused val
            sn_now_st = [-1, -1];
        end
    end
    
    %% WHILE CONNECTED
    while connection == 1
        
        % Check for new incoming data
        check = fread(tcpIP,NDat,'double');
        
        % Get data if any sent
        if ~isempty(check)
            
            % New data
            data = check;
            
            % Check if connection dropped
            if check(1) == 0
                
                % Sets data transfer longer timeout
                set(tcpIP,'Timeout',1);
                
                % Will take you out of loop
                connection = 0;
                
                % Print status message
                fprintf('\r\nDROPPED CONNECTION: %s\r\n', ...
                    datestr(now, 'HH:MM:SS AM'))
                
                % Re-initialize (for reasons I dont remember)
                data = zeros(1,NDat);
                clear('tcpIP','check')
                tcpIP = tcpip(DgLynxIP,30000,'NetworkRole','Client');
                set(tcpIP,'InputBufferSize',Bunsfsz);
                
            end
            
        end
        
        % EXECUTE CHANGES WITH NEW INCOMING DATA
        
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
        
        % SET CUE VARIABLES
        if ...
                data(1) == 1 && ...
                ~cue_fd && data(4) ~= 0
            
            % Update current cue
            cue_ind = data(4)+1;
            
            % Start cueing feeder
            cue_fd = true;
            
            % Initialize flicker timer
            t_cue = clock;
            
            % Print status message
            fprintf('\rset cue true: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        elseif ...
                data(1) == 1 && ...
                cue_fd && data(4) == 0
            
            % Stop cueing feeder
            cue_fd = false;
            
            % Print status message
            fprintf('\rset cue false: %s\r', ...
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
            fullscreenAWL(fullfile(imgDir,subDir{2},fiIn{1,img_ind}))
            
            % Pause to allow trigger to register
            pause(trgDel);
            
            % Show main image
            fullscreenAWL(fullfile(imgDir,subDir{1},fiIn{1,img_ind}))
            
            % Print status message
            fprintf('\rdisplay main image: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        end
        
        % STOP/START/RESTART SOUND
        % Setup noise vars
        % get last sound settings
        sn_lst_st = sn_now_st;
        % white noise data
        sn_now_st(1) = data(5);
        % rew/aversive data
        sn_now_st(2) = data(6);
        
        if ...
                sn_now_st(1) ~= 0
            
            % Start or change sound
            if ~isplaying(soundH{sn_now_st(1)}(sn_now_st(2)))
                
                % Stop whatever was playing
                    stop(soundH{sn_lst_st(1)}(sn_lst_st(2)));
                
                % Start new sound
                play(soundH{sn_now_st(1)}(sn_now_st(2)));
                
                % Update sound timer
                t_sound = clock;
                
                % Print status message
                fprintf('\rstart/change sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
                
                % Restart sound
            elseif etime(clock, t_sound) >= 60*floor(nsmin)
                
                % Stop whatever was playing
                stop(soundH{sn_now_st(1)}(sn_now_st(2)));
                
                % Restart existing sound
                play(soundH{1}(1));
                
                % Update sound timer
                t_sound = clock;
                
                % Print status message
                fprintf('\rrestart sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
                
            end
            
            % Stop all sound
        elseif all(sn_lst_st > 0)
            
            % Stop all sound if any playing
            if isplaying(soundH{sn_lst_st(1)}(sn_lst_st(2)))
                
                % Stop whatever was playing
                stop(soundH{sn_lst_st(1)}(sn_lst_st(2)));
                
                % Print status message
                fprintf('\rstop sound: %s\r', ...
                    datestr(now, 'HH:MM:SS AM'))
                
            end
            
        end
        
        % FLICKER CUE
        if cue_fd
            if ... % flicker cue
                    etime(clock, t_cue) >= ...
                    flckrRt
                
                % Show cue image
                if cue_state(1) == 0
                    
                    % Show image cue trigger
                    fullscreenAWL(fullfile(imgDir,subDir{2},fiIn{cue_ind,img_ind}))
                    
                    % Pause to allow trigger to register
                    pause(trgDel);
                    
                    % Show main cue image
                    fullscreenAWL(fullfile(imgDir,subDir{1},fiIn{cue_ind,img_ind}))
                    fprintf('\rcue on: %s\r', ...
                        datestr(now, 'HH:MM:SS AM'))
                    
                    % Show main image
                else
                    
                    % Show main image
                    fullscreenAWL(fullfile(imgDir,subDir{1},fiIn{1,img_ind}))
                    
                    % Print status message
                    fprintf('\rcue off: %s\r', ...
                        datestr(now, 'HH:MM:SS AM'))
                    
                end
                
                % Update cue state
                cue_state = fliplr(cue_state);
                
                % Update cue timer
                t_cue = clock;
                
            end
            
            % Turn off cue
        elseif ...
                ~cue_fd && ...
                cue_state(1) == 1
            
            % Show main image
            fullscreenAWL(fullfile(imgDir,subDir{1},fiIn{1,img_ind}))
            
            % Update cue state
            cue_state = fliplr(cue_state);
            
            % Print status message
            fprintf('\rcue off: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        end
        
        % Close all images
        if data(1) == 0
            
            % Close imige
            closescreenAWL
            
            % Stop white noise
            if sn_now_st(1) >0
                if isplaying(soundH{sn_now_st(1)}(sn_now_st(2)))
                    stop(soundH{sn_now_st(1)}(sn_now_st(2)))
                end
            end
            
            % Print status message
            fprintf('\rclose all images: %s\r', ...
                datestr(now, 'HH:MM:SS AM'))
            
        end
        
    end
    
end