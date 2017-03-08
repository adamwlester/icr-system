function[] = SesICRSetup()

% This function is to be run before rats enter the ICR arena
% The fallowing feilds should be updated in SS_In_All.mat before
% running:
%   SS_In_All.Session_Condition:   set to 'Manual_Training'
%   Yoke_Mate:      set to the number of the rats mate

%% Set paramiters
OutDir = regexp(pwd,'.*(?=\MATLAB)','match');
OutDir = fullfile(OutDir{:},'MATLAB\IOfiles\SessionData');


% Load in SS_In_All
% Note: after it has been edited
S = load(fullfile(OutDir, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;

% Max sesions
nses = 200; % number of total sessions

% Max ICR events per session
rots = 9;

%% Create condition paramiters

% Rat list
% will include only rats which are going into ICR and have not had
% parameters set already
icrRats = ... % find rats going into icr
    SS_In_All.Session_Condition == 'Manual_Training';
unmod = ... % find rats that have not already been updated
    isundefined(SS_In_All.Feeder_Condition);
newRats = icrRats & unmod; % list of rats to update  
ratList = categorical(SS_In_All.Properties.RowNames(newRats));
yokeList = SS_In_All{newRats, {'Yoke_Mate'}};
yokeList(yokeList == 'None') = ratList(yokeList == 'None');

% Get number of unique conditions needed
nCond = length(ratList) - sum(ratList ~= yokeList)/2;

% Get yoked pair indeces
% NOTE: Each row denotes the rat (col 1) and its yoked pair (col 2).
% non-yoked rats should be paired with themselves.
[~,mateInd] = ismember(ratList, yokeList);
yokePrs = [(1:length(ratList))',mateInd];
yokeInd = diff(yokePrs,[],2);
yokeInd(yokeInd>=0) = 1:nCond;
yokeInd(yokeInd<0) = find(yokeInd<0 == 1) + yokeInd(yokeInd<0);

%% Asign semi-random condition for yoked pairs

% FEEDER CONDITION
fdCtg = categories(SS_In_All.Feeder_Condition); % feeder condition
rdCtg = categories(SS_In_All.Rotation_Direction); % rotation direction
prms = [1, 1; 1, 2; 2, 1; 2, 2]; % col1 = feedcond; col2 = rotdrc
fdcnd = cell(4,ceil(nCond/4));
rtdir = cell(4,ceil(nCond/4));
for i = 1:ceil(nCond/4)
    ind = prms(randperm(4),:);
    fdcnd(:,i) = fdCtg(ind(:,1));
    rtdir(:,i) = rdCtg(ind(:,2));
end
% remove unneaded entries
fdcnd = fdcnd(:);
fdcnd = fdcnd(1:nCond); 
rtdir = rtdir(:);
rtdir = rtdir(1:nCond); 

% START QUADRANT
% Note: the start quad for the first day will be counterbalanced accross
% rats
sqCtg = categories(SS_In_All.Start_Quadrant{1,1}); % start quadrant
strqd = cell(4,ceil(nses/4),nCond);
% give each rat a different start quad on day 1
day1 = repmat(1:4,1,ceil(nCond/4));
day1 = day1(:); 
for i = 1:nCond;
    for j = 1:ceil(nses/4)
        ind = randperm(4);
        if j == 1
            while ind(1) ~= day1(i)
                ind = randperm(4);
            end
        end
        strqd(:,j,i) = sqCtg(ind);
    end
end
% reshape
strqd = reshape(strqd,[],nCond,1);
strqd = strqd(1:nses,:); % remove unneaded entries

% ROTATION POSITION
% Note: the order of rotation posisitions for the first rotation trial will
% be counter balanced accross rats
rpCtg = categories(SS_In_All.Rotation_Positions{1,1}); % rotation position
rotps = cell(3,rots/3,nses,nCond);
for i = 1:nCond
    for j = 1:ceil(nses)
        for k = 1:rots/3
            rotps(:,k,j,i) = rpCtg(randperm(3));
        end
    end
end
% reshape
rotps = reshape(rotps,nses,rots,nCond);
rotps = rotps(1:nses,:,:); % remove unneaded entries

% ROTATIONS PER SESSION
% Note: will perform 2 4 or 6 rotations per session
rsCtg = categories(SS_In_All.Rotations_Per_Session{1,1}); % rotation per session
nrot = cell(3,ceil(nses/3),nCond);
for i = 1:nCond;
    for j = 1:ceil(nses/3)
        nrot(:,j,i) = rsCtg(randperm(3));
    end
end
% reshape
nrot = reshape(nrot,[],nCond,1);
nrot = nrot(1:nses,:); % remove unneaded entries

% REWARDS PER ROTATION
% Note: a range of laps will be saved and the first entry for each session
% will be set to max number of rewards to collect more data for standard
% configuration laps
rprCtg = categories(SS_In_All.Rewards_Per_Rotation{1,1}); % rewards per rotation
nrew = cell(3,rots/3,nses,nCond);
for i = 1:nCond;
    for j = 1:ceil(nses)
        for k = 1:rots/3
            nrew(:,k,j,i) = rprCtg(randperm(3),:);
        end
    end
end
% reshape
nrew = reshape(nrew,nses,rots,nCond);
nrew(:,1,:) = rprCtg(end,:); % set first entry each session to 7-10 rew
nrew = nrew(1:nses,:,:); % remove unneaded entries

% DAYS TILL ROTATION
% Note: a range of days will be saved between 1-4 days
dtrCtg = [{'1:2'}; {'2:3'}; {'3:4'}]; % days till rotation
ndays = cell(3,ceil(nses/3),nCond);
for i = 1:nCond;
    for j = 1:ceil(nses/3)
        ndays(:,j,i) = dtrCtg(randperm(3),:);
    end
end
% reshape
ndays = reshape(ndays,[],nCond,1);
ndays = ndays(1:nses,:); % remove unneaded entries

% Add to main data sets
for i = 1:length(ratList)
    SS_In_All{char(ratList(i)), {'Feeder_Condition'}} = ...
        fdcnd(yokeInd(i)); 
    SS_In_All{char(ratList(i)), {'Rotation_Direction'}} = ...
        rtdir(yokeInd(i)); 
    SS_In_All{char(ratList(i)), {'Reward_Delay'}} = {'0.0'};
    SS_In_All{char(ratList(i)), {'Cue_Condition'}} = {'All'};
    SS_In_All{char(ratList(i)), {'Pulse_Duration'}} = 3000;
    SS_In_All{char(ratList(i)), {'Start_Quadrant'}}{:} = ...
        categorical(strqd(:,yokeInd(i))); 
    SS_In_All{char(ratList(i)), {'Rotation_Positions'}}{:} = ...
        categorical(rotps(:,:,yokeInd(i))); 
    SS_In_All{char(ratList(i)), {'Rotations_Per_Session'}}{:} = ...
        categorical(nrot(:,yokeInd(i))); 
    SS_In_All{char(ratList(i)), {'Rewards_Per_Rotation'}}{:} = ...
        categorical(nrew(:,:,yokeInd(i)));
    SS_In_All{char(ratList(i)), {'Days_Till_Rotation'}}{:} = ...
        categorical(ndays(:,yokeInd(i)), {'1:2', '2:3', '3:4'}); 
end

%% Add entries to SS_Out_ICR
% Load existing dataset
if exist(fullfile(OutDir, 'SS_Out_ICR.mat'), 'file')
    S = load(fullfile(OutDir, 'SS_Out_ICR.mat'));
    SS_Out_ICR = S.SS_Out_ICR;
end

% Store variables
T = table;
T.Include_Analysis = true;
T.Date = {''};
T.Start_Time = {''};
T.Total_Time = NaN;
T.Session_Condition = categorical({'<undefined>'}, ...
    {'Manual_Training', 'Behavior_Training', 'Implant_Training', 'Rotation'});
T.Session_Task = categorical({'<undefined>'}, {'Track', 'Forage'});
T.Session_Manual_Training = ...
    table(NaN, NaN, 'VariableNames', [{'T'}, {'F'}]);
T.Session_Manual_Training = [T.Session_Manual_Training.T, T.Session_Manual_Training.F];
T.Session_Behavior_Training = ...
    table(NaN, NaN, 'VariableNames', [{'T'}, {'F'}]);
T.Session_Behavior_Training = [T.Session_Behavior_Training.T, T.Session_Behavior_Training.F];
T.Session_Implant_Training = ...
    table(NaN, NaN, 'VariableNames', [{'T'}, {'F'}]);
T.Session_Implant_Training = [T.Session_Implant_Training.T, T.Session_Implant_Training.F];
T.Session_Rotation = NaN;
T.Feeder_Condition = categorical({'<undefined>'}, {'C1', 'C2'});
T.Rotation_Direction = categorical({'<undefined>'}, {'CCW', 'CW'});
T.Reward_Delay = ...
    categorical({'<undefined>'}, {'0.0', '0.5', '1.0', '1.5', '2.0', '3.0'}, 'Ordinal', true);
T.Cue_Condition = ...
    categorical({'<undefined>'}, {'All', 'Half', 'None'}, 'Ordinal', true);
T.Pulse_Duration = NaN;
T.Sound_Conditions = ...
    table(false, false, false,  'VariableNames', [{'S1'}, {'S2'}]);
T.Sound_Conditions = ...
    [T.Sound_Conditions.S1, T.Sound_Conditions.S2];
T.Start_Quadrant = categorical({'<undefined>'}, {'NE', 'SE', 'SW', 'NW'});
T.Rotation_Positions = {categorical({'<undefined>'}, {'90', '180', '270'})};
T.Rotations_Per_Session = NaN;
T.Rewards_Per_Rotation = {[]};
T.Days_Till_Rotation = categorical({'<undefined>'}, {'1:2', '2:3', '3:4'});
T.Bulldozings = NaN;
T.Rewards_Standard = {[]};
T.Rewards_40_Deg = {[]};
T.Rewards_0_Deg = {[]};
T.Laps_Standard = {[]};
T.Laps_40_Deg = {[]};
T.Laps_0_Deg = {[]};
T.Notes = {''};

% Set variable units
T.Properties.VariableUnits{'Total_Time'} = 'min';
T.Properties.VariableUnits{'Reward_Delay'} = 'sec';
T.Properties.VariableUnits{'Pulse_Duration'} = 'ms';

% Set variable descriptions
T.Properties.VariableDescriptions{'Session_Manual_Training'} = '[Track, Forage]';
T.Properties.VariableDescriptions{'Session_Behavior_Training'} = '[Track, Forage]';
T.Properties.VariableDescriptions{'Session_Implant_Training'} = '[Track, Forage]';
T.Properties.VariableDescriptions{'Sound_Conditions'} = '[White, Reward]';
T.Properties.VariableDescriptions{'Rotations_Per_Session'} = '[2,4,6]';
T.Properties.VariableDescriptions{'Rewards_Per_Rotation'} = '[5:10]';
T.Properties.VariableDescriptions{'Days_Till_Rotation'} = '[1:4]';

% Loop through and create a field for each rat
ratList = categories(ratList);
for z_rat = 1:length(ratList)
    SS_Out_ICR.(ratList{z_rat}) = T;
end

% Sort fields by rat
[~, ind] = sort(fieldnames(SS_Out_ICR));
SS_Out_ICR = orderfields(SS_Out_ICR, ind);

%% Save out tables
save(fullfile(OutDir,'SS_Out_ICR'), 'SS_Out_ICR')
save(fullfile(OutDir,'SS_In_All'), 'SS_In_All')

% % Check SS_In_All field class
% flds = SS_In_All.Properties.VariableNames;
% for i = 1:length(flds)
%     sprintf('%s %s', flds{i}, class(SS_In_All.(flds{i})))
%     pause
% end

