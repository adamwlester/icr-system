function[] = SS_IO_Setup()

%% ============================== NOTES ===================================
% Used for initializing and/or adding rats to files for daily running
% on the ICR. Data is saved to "SS_IO_1.mat", "SS_IO_2.mat" and
% "SS_IO_3.mat". Change parameters below to include the rat labels, which
% is formated using the last 4 digits of the rat number preceded with an
% "r" (e.g. 10556 = "r0556").


%% ========================= SET PARAMETERS ===============================
topDir = 'C:\Users\lester\repos\icr-system\main\matlab';
ioDir = regexp(topDir,'.*(?=\icr-system)','match');
ioDir = fullfile(ioDir{:},'icr-system\data\session');

% Rat numbers (must be preceded by an 'r')
RAT.ratList = [...
    {'r0752'}; ...
    {'r0753'}; ...
    {'r0758'}; ...
    ];

% Yoke mate list
RAT.yokeList = [...
    {'None'}; ...
    {'None'}; ...
    {'None'}; ...
    ];

% Age group [Young, Old]
RAT.ageList = [...
    {'Young'}; ...
    {'Young'}; ...
    {'Old'}; ...
    ];

% DOB date string
RAT.dobList = [...
    {'12/1/2019'}; ...
    {'12/1/2019'}; ...
    {'10/1/2018'}; ...
    ];

% Tail marking string
RAT.tailMrkList = [...
    {'3P'}; ...
    {'4P'}; ...
    {'3G'}; ...
    ];

% Baseline weight
RAT.weightList = [...
    372; ...
    355; ...
    387; ...
    ];

% Max sesions
nses = 200; % number of total sessions

% Max ICR events per session
rots = 9;

% List for categorical array containing initials of people who will be running
human_cats = [...
    {'Other'}; ...
    {'AWL'}; ...
    {'ZP'}; ...
    {'CF'}; ...
    {'ALD'}; ...
    {'JWM'}; ...
    {'GJ'}; ...
    ];


%% =========================== SETUP VARS =================================

% convert to yy/mm/dd format
RAT.dobList = cellstr(datestr(RAT.dobList, 'yyyy/mm/dd'));

% Define categories for SS_IO_1 and SS_IO_2
feeder_version_cats = {'Static_Feeder', 'Mobile_Feeder'};
age_group_cats = {'Young', 'Old'};
session_type_cats = {'ICR_Session', 'TT_Turn', 'Table_Update'};
session_condition_cats = {'Manual_Training', 'Behavior_Training', 'Implant_Training', 'Rotation', 'Dark_Control'};
session_task_cats = {'Track', 'Forage'};
feeder_condition_cats = {'C1', 'C2'};
rotation_direction_cats = {'CCW', 'CW'};
reward_delay_cats = {'0.0', '0.5', '1.0', '1.5', '2.0', '2.5', '3.0'};
cue_condition_cats = {'All', 'Half', 'None'};
start_quadrant_cats = {'NE', 'NW', 'SW', 'SE'};
rotation_positions_cats = {'90', '180', '270'};
rotations_per_session_cats = {'2', '4', '6'};
laps_per_rotation_cats = {'5:8', '6:9', '7:10'};
days_till_rotation_cats = {'1:2', '2:3', '3:4'};

% Variable descriptions for SS_IO_1 and SS_IO_2
session_number_description = '[Track, Forage]';
sound_conditions_description = '[White, Reward]';
rotations_per_session_description = '{[2,4,6]}';
laps_per_rotation_description = '{[5:10]}';
days_till_rotation_description = '{[1:4]}';


% Define categories for SS_IO_3
nose_cats{1} = {'Normal', 'Red-rimmed'};
nose_cats{2} = {'Clear', 'Clogged or Sneezing'};
teeth_cats{1} = {'Correct Length', 'Overgrown'};
teeth_cats{2} = {'Intact', 'Broken Bottom', 'Broken Top'};
eyes_cats{1} = {'Bright', 'Dull'};
eyes_cats{2} = {'Clear', 'Red-rimmed'};
eyes_cats{3} = {'Pink', 'Pale'};
eyes_cats{4} = {'No Cataract', 'Cataract present'};
ears_cats{1} = {'Clean', 'Exudate'};
throat_cheeks_cats{1} = {'Normal', 'Swelling'};
chest_cats{1} = {'Normal', 'Wheezing'};
kidneys_bladder_cats{1} = {'Normal Urine', 'Yellow-Stained Fur', 'Blood in Urine', 'Not Seen'};
penis_cats{1} = {'No Plug', 'Plug Present'};
testicles_cats{1} = {'Left Normal ', 'Left Enlarged', 'Left Atrophied'};
testicles_cats{1} = {'Right Normal ', 'Right Enlarged', 'Right Atrophied'};
tail_cats{1} = {'Normal', 'Sores/Necrotic'};
body_cats{1} = {'Normal', 'Lump', 'Tumor', 'Cyst'};
body_cats{2} = {'Size: 0 cm', 'Size: < 1 cm', 'Size: 1-2 cm', 'Size: 2-3 cm', 'Size: > 3 cm'};
grooming_cats{1} = {'Good', 'Fair', 'Poor'};
skin_cats{1} = {'Hydrated', 'Dehydrated'};
feces_cats{1} = {'Firm', 'Soft', 'Present', 'Not Seen'};
posture_cats{1} = {'Normal', 'Hunched'};

%% ========================= UPDATE SS_IO_1 ===============================

% Initialize table
T = table('RowNames',RAT.ratList);

% ADD/INITIALIZE TABLE ENTRIES

T.Include_Run = true(length(RAT.ratList),1);

T.Include_Analysis = true(length(RAT.ratList),1);

T.Implanted = false(length(RAT.ratList),1);

T.Yoke_Mate = categorical(RAT.yokeList, ...
    [{'None'}; RAT.ratList]);

T.Feeder_Version = categorical(repmat({'Mobile_Feeder'},length(RAT.ratList),1), ...
    feeder_version_cats);

T.Age_Group = categorical(RAT.ageList, age_group_cats);

T.DOB = RAT.dobList;

T.Tail_Mark = RAT.tailMrkList;

T.Weight_Baseline = RAT.weightList;

T.Human = categorical(repmat({'<undefined>'},length(RAT.ratList),1), ...
    human_cats);

T.Session_Type = categorical(repmat({'ICR_Session'},length(RAT.ratList),1), ...
    session_type_cats);

T.Session_Condition = categorical(repmat({'Manual_Training'},length(RAT.ratList),1), ...
    session_condition_cats);

T.Session_Task = categorical(repmat({'Track'},length(RAT.ratList),1), ...
    session_task_cats);

T.Session_Manual_Training = repmat([0, 0],length(RAT.ratList),1);

T.Session_Behavior_Training = repmat([0, 0],length(RAT.ratList),1);

T.Session_Implant_Training = repmat([0, 0],length(RAT.ratList),1);

T.Session_Rotation = zeros(length(RAT.ratList),1);

T.Session_Dark_Control = zeros(length(RAT.ratList),1);

T.Finished_Manual_Training = false(length(RAT.ratList),1);

T.Finished_Behavior_Training = false(length(RAT.ratList),1);

T.Finished_Study = false(length(RAT.ratList),1);

T.Feeder_Condition = categorical(repmat({'<undefined>'},length(RAT.ratList),1), ...
    feeder_condition_cats);

T.Reward_Delay = categorical(repmat({'0.0'},length(RAT.ratList),1), ...
    reward_delay_cats, 'Ordinal', true);

T.Cue_Condition = categorical(repmat({'None'},length(RAT.ratList),1), ...
    cue_condition_cats, 'Ordinal', true);

T.Sound_Conditions = repmat([false, false], length(RAT.ratList) ,1);

T.Start_Quadrant = repmat({categorical(repmat({'<undefined>'},200,1), ...
    start_quadrant_cats)}, length(RAT.ratList),1);

T.Rotation_Direction = repmat({categorical(repmat({'<undefined>'},200,1), ...
    rotation_direction_cats)}, length(RAT.ratList),1);

T.Rotation_Positions = repmat({categorical(repmat({'<undefined>'},200,9), ...
    rotation_positions_cats)}, length(RAT.ratList),1);

T.Rotations_Per_Session = repmat({categorical(repmat({'<undefined>'},200,1), ...
    rotations_per_session_cats)}, length(RAT.ratList),1);

T.Laps_Per_Rotation = repmat({categorical(repmat({'<undefined>'},200,9), ...
    laps_per_rotation_cats)}, length(RAT.ratList),1);

T.Days_Till_Rotation = repmat({categorical(repmat({'<undefined>'},200,1), ...
    days_till_rotation_cats)}, length(RAT.ratList),1);

T.Notes = repmat({''}, length(RAT.ratList), 1);

% Set variable units
T.Properties.VariableUnits{'Weight_Baseline'} = 'g';
T.Properties.VariableUnits{'Reward_Delay'} = 'sec';

% Set variable descriptions
T.Properties.VariableDescriptions{'Session_Manual_Training'} = session_number_description;
T.Properties.VariableDescriptions{'Session_Behavior_Training'} = session_number_description;
T.Properties.VariableDescriptions{'Session_Implant_Training'} = session_number_description;
T.Properties.VariableDescriptions{'Sound_Conditions'} = sound_conditions_description;
T.Properties.VariableDescriptions{'Rotations_Per_Session'} = rotations_per_session_description;
T.Properties.VariableDescriptions{'Laps_Per_Rotation'} = laps_per_rotation_description;
T.Properties.VariableDescriptions{'Days_Till_Rotation'} = days_till_rotation_description;
T.Properties.VariableDescriptions{'Days_Till_Rotation'} = days_till_rotation_description;

% Load existing dataset
if exist(fullfile(ioDir, 'SS_IO_1.mat'), 'file')
    S = load(fullfile(ioDir, 'SS_IO_1.mat'));
    SS_IO_1 = S.SS_IO_1;
    
    % Exclude rats already in table
    exc_rats = RAT.ratList(ismember(RAT.ratList,SS_IO_1.Properties.RowNames));
    if ~isempty(exc_rats)
        fprintf('**WARNING**: Rats %s Already in List\n', exc_rats{:});
    end
    T(ismember(T.Properties.RowNames,SS_IO_1.Properties.RowNames), :) = [];
    
    % Get include rat indeces
    inc_ind = ~ismember(RAT.ratList, exc_rats);
    
    % Bail if no new rats
    if ~any(inc_ind)
        fprintf('**WARNING**: Exited With Nothing Changed\n');
        return
    end
    
    % Update PAR variables and exclude redundant rats
    fld_list = fieldnames(RAT);
    for z_p = 1:length(fld_list)
        RAT.(fld_list{z_p}) = ...
            RAT.(fld_list{z_p})(inc_ind);
    end
    
    % Add into main table
    T = [SS_IO_1; T];
end

% Copy sorted rat data
SS_IO_1 = sortrows(T, 'RowNames');

%% ========================== GERNATE ENTRIES =============================

%  ------------------  Create condition paramiters ------------------------

% Get number of unique conditions needed
nUnique = sum(ismember(RAT.yokeList, 'None')) + sum(ismember(RAT.ratList, RAT.yokeList))/2;

% Get yoked pair indeces
% NOTE: Each row denotes the rat (col 1) and its yoked pair (col 2).
% non-yoked rats should be paired with themselves.
[~,mateInd] = ismember(RAT.ratList, RAT.yokeList);
yokePrs = [(1:length(RAT.ratList))',mateInd];
yokePrs(mateInd==0, 2) = yokePrs(mateInd==0, 1);
yokeInd = diff(yokePrs,[],2);
yokeInd(yokeInd>=0) = 1:nUnique;
yokeInd(yokeInd<0) = find(yokeInd<0 == 1) + yokeInd(yokeInd<0);

% ------------ Asign semi-random condition for yoked pairs ----------------

% FEEDER CONDITION
fdcnd = cell(2,ceil(nUnique/2));
for i = 1:ceil(nUnique/2)
    fdcnd(:,i) = feeder_condition_cats(randperm(2));
end
% remove unneaded entries
fdcnd = reshape(fdcnd(:),[],1);
fdcnd = fdcnd(1:nUnique);

% START QUADRANT
% Note: the start quad for the first day will be counterbalanced accross
% rats
strqd = cell(4,ceil(nses/4),nUnique);
% give each rat a different start quad on day 1
day1 = repmat(1:4,1,ceil(nUnique/4));
day1 = day1(:);
for i = 1:nUnique
    for j = 1:ceil(nses/4)
        ind = randperm(4);
        if j == 1
            while ind(1) ~= day1(i)
                ind = randperm(4);
            end
        end
        strqd(:,j,i) = start_quadrant_cats(ind);
    end
end
% Reshape and remove unneaded entries
strqd = reshape(strqd,[],nUnique,1);
strqd = strqd(1:nses,:);

% ROTATION DIRECTION
rtdir = cell(2,ceil(nses/2),nUnique);
% give each pair a different rotation directin on day 1
rstr = ceil(rand(1,1)*2);
day1 = repmat([rstr, find([1,2]~=rstr)],1,ceil(nUnique/2));
day1 = day1(:);
for i = 1:nUnique
    for j = 1:ceil(nses/2)
        ind = randperm(2);
        if j == 1
            while ind(1) ~= day1(i)
                ind = randperm(2);
            end
        end
        rtdir(:,j,i) = rotation_direction_cats(ind);
    end
end
% Reshape and remove unneaded entries
rtdir = reshape(rtdir,[],nUnique,1);
rtdir = rtdir(1:nses,:);

% ROTATION POSITION
% Note: the order of rotation posisitions for the first rotation trial will
% be counter balanced accross rats
rotps = cell(3,rots/3,nses,nUnique);
for i = 1:nUnique
    for j = 1:ceil(nses)
        for k = 1:rots/3
            rotps(:,k,j,i) = rotation_positions_cats(randperm(3));
        end
    end
end
% reshape
rotps = reshape(rotps,nses,rots,nUnique);
rotps = rotps(1:nses,:,:); % remove unneaded entries

% ROTATIONS PER SESSION
% Note: will perform 2 4 or 6 rotations per session
nrot = cell(3,ceil(nses/3),nUnique);
for i = 1:nUnique
    for j = 1:ceil(nses/3)
        nrot(:,j,i) = rotations_per_session_cats(randperm(3));
    end
end
% reshape
nrot = reshape(nrot,[],nUnique,1);
nrot = nrot(1:nses,:); % remove unneaded entries

% REWARDS PER ROTATION
% Note: a range of laps will be saved and the first entry for each session
% will be set to max number of laps to collect more data for standard
% configuration laps
nlap = cell(3,rots/3,nses,nUnique);
for i = 1:nUnique
    for j = 1:ceil(nses)
        for k = 1:rots/3
            nlap(:,k,j,i) = laps_per_rotation_cats(randperm(3));
        end
    end
end
% reshape
nlap = reshape(nlap,nses,rots,nUnique);
nlap(:,1,:) = laps_per_rotation_cats(end); % set first entry each session to 7-10 rew
nlap = nlap(1:nses,:,:); % remove unneaded entries

% DAYS TILL ROTATION
% Note: a range of days will be saved between 1-4 days
ndays = cell(3,ceil(nses/3),nUnique);
for i = 1:nUnique
    for j = 1:ceil(nses/3)
        ndays(:,j,i) = days_till_rotation_cats(randperm(3));
    end
end
% reshape
ndays = reshape(ndays,[],nUnique,1);
ndays = ndays(1:nses,:); % remove unneaded entries

% Add to main data sets
for i = 1:length(RAT.ratList)
    
    SS_IO_1{char(RAT.ratList(i)), {'Feeder_Condition'}} = ...
        fdcnd(yokeInd(i));
    
    SS_IO_1{char(RAT.ratList(i)), {'Start_Quadrant'}}{:} = ...
        categorical(strqd(:,yokeInd(i)), start_quadrant_cats);
    
    SS_IO_1{char(RAT.ratList(i)), {'Rotation_Direction'}}{:} = ...
        categorical(rtdir(:,yokeInd(i)), rotation_direction_cats);
    
    SS_IO_1{char(RAT.ratList(i)), {'Rotation_Positions'}}{:} = ...
        categorical(rotps(:,:,yokeInd(i)), rotation_positions_cats);
    
    SS_IO_1{char(RAT.ratList(i)), {'Rotations_Per_Session'}}{:} = ...
        categorical(nrot(:,yokeInd(i)), rotations_per_session_cats);
    
    SS_IO_1{char(RAT.ratList(i)), {'Laps_Per_Rotation'}}{:} = ...
        categorical(nlap(:,:,yokeInd(i)), laps_per_rotation_cats);
    
    SS_IO_1{char(RAT.ratList(i)), {'Days_Till_Rotation'}}{:} = ...
        categorical(ndays(:,yokeInd(i)), days_till_rotation_cats);
    
end

%% ========================= UPDATE SS_IO_2 ===============================

% Load existing dataset
if exist(fullfile(ioDir, 'SS_IO_2.mat'), 'file')
    S = load(fullfile(ioDir, 'SS_IO_2.mat'));
    SS_IO_2 = S.SS_IO_2;
end

% Reinitialize table
T = table;

% STORE/ADD TABLE ENTRIES

T.Include_Analysis = true;

T.Implanted = false;

T.Feeder_Version = categorical({'<undefined>'}, ...
    feeder_version_cats);

T.Date = {''};

T.Recording_Dir = {''};

T.Raw_Data_Dir = {''};

T.Ephys_Recorded = false;

T.Human = categorical({'<undefined>'}, ...
    human_cats);

T.VT_Pixel_Coordinates = {nan(1,3)};

T.Camera_Orientation = nan;

T.Image_Orientation = {nan(1,2)};

T.Start_Time = {''};

T.Total_Time = nan;

T.Sleep_Time = nan(1,2);

T.Session_Type = categorical({'<undefined>'}, ...
    session_type_cats);

T.Session_Condition = categorical({'<undefined>'}, ...
    session_condition_cats);

T.Session_Task = categorical({'<undefined>'}, ...
    session_task_cats);

T.Session_Manual_Training = nan(1,2);

T.Session_Behavior_Training = nan(1,2);

T.Session_Implant_Training =  nan(1,2);

T.Session_Rotation = nan;

T.Session_Dark_Control = nan;

T.Feeder_Condition = categorical({'<undefined>'}, ...
    feeder_condition_cats);

T.Reward_Delay = categorical({'<undefined>'}, ...
    reward_delay_cats, 'Ordinal', true);

T.Cue_Condition = categorical({'<undefined>'}, ...
    cue_condition_cats, 'Ordinal', true);

T.Sound_Conditions =  [false, false];

T.PID_Setpoint = nan;

T.Start_Quadrant = categorical({'<undefined>'}, ...
    start_quadrant_cats);

T.Rotation_Direction = categorical({'<undefined>'}, ...
    rotation_direction_cats);

T.Rotation_Positions = {[]};

T.Rotations_Per_Session = nan;

T.Laps_Per_Rotation = {[]};

T.Days_Till_Rotation = categorical({'<undefined>'}, ...
    days_till_rotation_cats);

T.Bulldozings = nan;

T.Zones_Rewarded = {[]};

T.Cued_Rewards = {[]};

T.Rewards_Missed = nan;

T.Rewards_Standard = {[]};

T.Rewards_40_Deg = {[]};

T.Rewards_0_Deg = {[]};

T.Laps_Standard = {[]};

T.Laps_40_Deg = {[]};

T.Laps_0_Deg = {[]};

T.Notes = {''};

% Set variable units
T.Properties.VariableUnits{'VT_Pixel_Coordinates'} = 'pixels';
T.Properties.VariableUnits{'Camera_Orientation'} = 'deg';
T.Properties.VariableUnits{'Image_Orientation'} = 'deg';
T.Properties.VariableUnits{'Total_Time'} = 'min';
T.Properties.VariableUnits{'Reward_Delay'} = 'sec';
T.Properties.VariableUnits{'PID_Setpoint'} = 'cm';

% Set variable descriptions
T.Properties.VariableDescriptions{'Session_Manual_Training'} = session_number_description;
T.Properties.VariableDescriptions{'Session_Behavior_Training'} = session_number_description;
T.Properties.VariableDescriptions{'Session_Implant_Training'} = session_number_description;
T.Properties.VariableDescriptions{'Sound_Conditions'} = sound_conditions_description;
T.Properties.VariableDescriptions{'Rotations_Per_Session'} = rotations_per_session_description;
T.Properties.VariableDescriptions{'Laps_Per_Rotation'} = laps_per_rotation_description;
T.Properties.VariableDescriptions{'Days_Till_Rotation'} = days_till_rotation_description;

% Loop through and create a field for each rat
for z_rat = 1:length(RAT.ratList)
    SS_IO_2.(RAT.ratList{z_rat}) = T;
end

% Sort fields by rat
[~, ind] = sort(fieldnames(SS_IO_2));
SS_IO_2 = orderfields(SS_IO_2, ind);

%% ============================= SS_IO_3 ==================================

% Load existing dataset
if exist(fullfile(ioDir, 'SS_IO_3.mat'), 'file')
    S = load(fullfile(ioDir, 'SS_IO_3.mat'));
    SS_IO_3 = S.SS_IO_3;
end

% Reinitialize table
T = table;

% STORE/ADD TABLE ENTRIES

T.Date = {''};

T.Human = categorical({'<undefined>'}, ...
    human_cats);

T.Start_Time = {''};

T.Total_Time = nan;

T.Weight = nan;

T.Weight_Baseline = nan;

T.Weight_Drive = nan;

T.Weight_Cap = nan;

T.Weight_Corrected = nan;

T.Weight_Proportion = nan;

T.Fed_Pellets = nan;

T.Fed_Mash = nan;

T.Fed_Ensure = nan;

T.Fed_STAT = nan;

for z_c = 1:length(nose_cats)
    T.(['Health_Nose_', num2str(z_c)]) = categorical(nose_cats{z_c}(1), ...
        nose_cats{z_c});
end

for z_c = 1:length(teeth_cats)
    T.(['Health_Teeth_', num2str(z_c)]) = categorical(teeth_cats{z_c}(1), ...
        teeth_cats{z_c});
end

for z_c = 1:length(eyes_cats)
    T.(['Health_Eyes_', num2str(z_c)]) = categorical(eyes_cats{z_c}(1), ...
        eyes_cats{z_c});
end

for z_c = 1:length(ears_cats)
    T.(['Health_Ears_', num2str(z_c)]) = categorical(ears_cats{z_c}(1), ...
        ears_cats{z_c});
end

for z_c = 1:length(throat_cheeks_cats)
    T.(['Health_Throat_Cheeks_', num2str(z_c)]) = categorical(throat_cheeks_cats{z_c}(1), ...
        throat_cheeks_cats{z_c});
end

for z_c = 1:length(chest_cats)
    T.(['Health_Chest_', num2str(z_c)]) = categorical(chest_cats{z_c}(1), ...
        chest_cats{z_c});
end

for z_c = 1:length(kidneys_bladder_cats)
    T.(['Health_Kidneys_Bladder_', num2str(z_c)]) = categorical(kidneys_bladder_cats{z_c}(1), ...
        kidneys_bladder_cats{z_c});
end

for z_c = 1:length(penis_cats)
    T.(['Health_Penis_', num2str(z_c)]) = categorical(penis_cats{z_c}(1), ...
        penis_cats{z_c});
end

for z_c = 1:length(testicles_cats)
    T.(['Health_Testicles_', num2str(z_c)]) = categorical(testicles_cats{z_c}(1), ...
        testicles_cats{z_c});
end

for z_c = 1:length(tail_cats)
    T.(['Health_Tail_', num2str(z_c)]) = categorical(tail_cats{z_c}(1), ...
        tail_cats{z_c});
end

for z_c = 1:length(body_cats)
    T.(['Health_Body_', num2str(z_c)]) = categorical(body_cats{z_c}(1), ...
        body_cats{z_c});
end

for z_c = 1:length(grooming_cats)
    T.(['Health_Grooming_', num2str(z_c)]) = categorical(grooming_cats{z_c}(1), ...
        grooming_cats{z_c});
end

for z_c = 1:length(skin_cats)
    T.(['Health_Skin_', num2str(z_c)]) = categorical(skin_cats{z_c}(1), ...
        skin_cats{z_c});
end

for z_c = 1:length(feces_cats)
    T.(['Health_Feces_', num2str(z_c)]) = categorical(feces_cats{z_c}(1), ...
        feces_cats{z_c});
end

for z_c = 1:length(posture_cats)
    T.(['Health_Posture_', num2str(z_c)]) = categorical(posture_cats{z_c}(1), ...
        posture_cats{z_c});
end

T.Notes = {''};

% Set variable units
T.Properties.VariableUnits{'Weight'} = 'g';
T.Properties.VariableUnits{'Weight_Baseline'} = 'g';
T.Properties.VariableUnits{'Weight_Corrected'} = 'g';
T.Properties.VariableUnits{'Fed_Pellets'} = 'pellets';
T.Properties.VariableUnits{'Fed_Mash'} = 'TB';
T.Properties.VariableUnits{'Fed_Ensure'} = 'ml';
T.Properties.VariableUnits{'Fed_STAT'} = 'ml';

% Loop through and create a field for each rat
for z_rat = 1:length(RAT.ratList)
    SS_IO_3.(RAT.ratList{z_rat}) = T;
end

% Sort fields by rat
[~, ind] = sort(fieldnames(SS_IO_3));
SS_IO_3 = orderfields(SS_IO_3, ind);

%% ============================== SAVE ====================================

% Save out tables
save(fullfile(ioDir,'SS_IO_1'), 'SS_IO_1')
save(fullfile(ioDir,'SS_IO_2'), 'SS_IO_2')
save(fullfile(ioDir,'SS_IO_3'), 'SS_IO_3')

% Print saved changes
for i = 1:length(RAT.ratList)
    fprintf('FINISHED: Adding Rat %s\n', char(RAT.ratList(i)));
end

end