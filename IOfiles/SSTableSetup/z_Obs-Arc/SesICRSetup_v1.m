function[] = SesICRSetup()

% This function is to be run before rats enter the ICR arena
% The fallowing feilds should be updated in SS_In_All.mat before
% running:
%   ICRbehavior:   set to true
%   Yoke_Mate:      set to the number of the rats mate

%% Set paramiters
InOutDir = ...
    fullfile(pwd, '\IOfiles\SessionData');

% Load in SS_In_All
% Note: after it has been edited
S = load(fullfile(InOutDir, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
clear S

% Max sesions
nses = 200; % number of total sessions

% Max ICR events per session
rots = 9;

%% Create condition paramiters

% Strings for conditions
fdarr = [1, 2]; % feeder condition
rdstr = [{'CW'}, {'CCW'}]; % rotation direction
sqstr = [{'NE'}, {'SE'},{'SW'}, {'NW'}]; % start quadrant
rparr = [90, 180, 270]; % rotation position
rotsarr = 2:2:6; % number rotation
rwarr = [5,8; 6,9; 7,10]; % rewards per rotation
dsoffarr = [1,2; 2,3; 3,4]; % days till rotation

% Rat list
% will include only rats which are going into ICR and have not had
% parameters set already
icrrats = ... % find rats going into icr
    {SS_In_All.ICRb};
unmod = ... % find rats that have not already been updated
    {SS_In_All.Feeder_Condition};
newrats = ... list of rats to update
    find(cell2mat(icrrats) == 1 & ...
    isempty(cell2mat(unmod)) == 1);
pastrats = ... list of rats to update
    find(cell2mat(icrrats) == 1 & ...
    isempty(cell2mat(unmod)) ~= 1);
ratlist = {SS_In_All(newrats).Rat};
yokelist = {SS_In_All(newrats).Yoke_Mate};

% Get number of unique conditions needed
ncond = length(ratlist) - sum(~strcmp(ratlist,yokelist))/2;

% Get yoked pair indeces
% NOTE: Each row denotes the rat (col 1) and its yoked pair (col 2).
% non-yoked rats should be paired with themselves.
[~,mateind] = ismember(ratlist, yokelist);
yokeprs = [(1:length(ratlist))',mateind'];
yokeind = diff(yokeprs,[],2);
yokeind(yokeind>=0) = 1:ncond;
yokeind(yokeind<0) = find(yokeind<0 == 1) + yokeind(yokeind<0);

%% Asign semi-random condition for yoked pairs

% FEEDER CONDITION
prms = [1, 1; 1, 2; 2, 1; 2, 2]; % col1 = feedcond; col2 = rotdrc
fddrc = [];
for i = 1:ceil(ncond/4)
    fddrc = [fddrc; shuffle(prms,1)];
end
fddrc = fddrc(1:ncond,:); % remove unneaded entries

% START QUADRANT
% Note: the start quad for the first day will be counterbalanced accross
% rats
strqd = NaN(4,ceil(nses/4),ncond);
for i = 1:ncond;
    for j = 1:ceil(nses/4)
        strqd(:,j,i) = randperm(4);
    end
end
% reshape
strqd = reshape(strqd,[],ncond,1);
strqd = strqd(1:nses,:); % remove unneaded entries

% ROTATION POSITION
% Note: the order of rotation posisitions for the first rotation trial will
% be counter balanced accross rats
rotps = NaN(3,rots/3,nses,ncond);
for i = 1:ncond
    for j = 1:ceil(nses)
        for k = 1:rots/3
            rotps(:,k,j,i) = rparr(randperm(3));
        end
    end
end
% reshape
rotps = reshape(rotps,nses,rots,ncond);
rotps = rotps(1:nses,:,:); % remove unneaded entries

% NUMBER ROTATIONS
% Note: will perform 2 4 or 6 rotations per session
nrot = NaN(3,ceil(nses/3),ncond);
for i = 1:ncond;
    for j = 1:ceil(nses/3)
        nrot(:,j,i) = rotsarr(randperm(3));
    end
end
% reshape
nrot = reshape(nrot,[],ncond,1);
nrot = nrot(1:nses,:); % remove unneaded entries

% REWARDS PER ROTATION
% Note: a range of laps will be saved and the first entry for each session
% will be set to max number of rewards to collect more data for standard
% configuration laps
nrew = cell(3,rots/3,nses,ncond);
for i = 1:ncond;
    for j = 1:ceil(nses)
        for k = 1:rots/3
            nrew(:,k,j,i) = mat2cell(rwarr(randperm(3),:),[1,1,1],2);
        end
    end
end
% reshape
nrew = reshape(nrew,nses,rots,ncond);
nrew(:,1,:) = {rwarr(end,:)}; % set first entry each session to 7-10 rew
nrew = nrew(1:nses,:,:); % remove unneaded entries

% DAYS TILL ROTATION
% Note: a range of days will be saved between 1-4 days
ndays = cell(3,ceil(nses/3),ncond);
for i = 1:ncond;
    for j = 1:ceil(nses/3)
        ndays(:,j,i) = mat2cell(dsoffarr(randperm(3),:),[1,1,1],2);
    end
end
% reshape
ndays = reshape(ndays,[],ncond,1);
ndays = ndays(1:nses,:); % remove unneaded entries

% Add to main data sets
for i = 1:length(ratlist)
    SS_In_All(i).CT = false; 
    SS_In_All(i).Feeder_Condition = fdarr(fddrc(yokeind(i),1)); 
    SS_In_All(i).Rotation_Direction = rdstr{fddrc(yokeind(i),2)}; 
    SS_In_All(i).Reward_Ratio = '1:1'; 
    SS_In_All(i).Reward_Delay = 0.01;
    SS_In_All(i).Cue_Condition = 1;
    SS_In_All(i).Pulse_Duration = [];
    SS_In_All(i).Sound_Conditions = [0,0,0];
    SS_In_All(i).Smell_Conditions = 0;
    SS_In_All(i).Start_Quadrant = strqd(:,yokeind(i)); 
    SS_In_All(i).Rotation_Positions = rotps(:,:,yokeind(i)); 
    SS_In_All(i).Rotations_Per_Session = nrot(:,yokeind(i)); 
    SS_In_All(i).Rewards_Per_Rotation = nrew(:,:,yokeind(i));
    SS_In_All(i).Days_Till_Rotation = ndays(:,yokeind(i)); 
end

% Save out variables
save(fullfile(InOutDir,'SS_In_All'), 'SS_In_All')

% Check field class
% flds = fieldnames(SS_In_All);
% for i = 1:length(flds)
%     sprintf('%s %s', flds{i}, class(SS_In_All(1).(flds{i})))
%     pause
% end


