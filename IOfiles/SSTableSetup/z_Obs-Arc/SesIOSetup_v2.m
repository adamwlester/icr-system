function[] = SesIOSetup()

%% Set paramiters
OutDir = regexp(pwd,'.*(?=\MATLAB)','match');
OutDir = fullfile(OutDir{:},'MATLAB\IOfiles\SessionData');

% Rat numbers (must be preceded by an 'r')
ratlist = [...
    {'r0138'}; ...
    {'r0139'}; ...
    {'r0140'}; ...
    {'r0141'}; ...
    {'r0142'}; ...
    {'r0145'}; ...
    {'r0146'}; ...
    {'r0173'}; ...
    {'r0203'}; ...
    {'r0204'}; ...
    {'r0205'}; ...
    {'r0207'} ...
    ];

% Age group
agestr = [...
    {'Young'}; ...
    {'Young'}; ...
    {'Young'}; ...
    {'Young'}; ...
    {'Young'}; ...
    {'Old'}; ...
    {'Old'}; ...
    {'Old'}; ...
    {'Young'}; ...
    {'Young'}; ...
    {'Old'}; ...
    {'Old'} ...
    ];

% DOB date string
DOBstr = [...
    {'09/01/2015'}; ...
    {'09/01/2015'}; ...
    {'09/01/2015'}; ...
    {'09/01/2015'}; ...
    {'09/01/2015'}; ...
    {'01/01/2014'}; ...
    {'01/01/2014'}; ...
    {'01/01/2014'}; ...
    {'10/01/2015'}; ...
    {'10/01/2015'}; ...
    {'08/01/2014'}; ...
    {'08/01/2014'} ...
    ];
% convert to yy/mm/dd format
DOBstr = cellstr(datestr(DOBstr, 'yyyy/mm/dd'));

%% ======================== SS_In_All =====================================

% Store variables
T = table('RowNames',ratlist);
T.Include_Run = true(length(ratlist),1);
T.Include_Analysis = true(length(ratlist),1);
T.Age_Group = categorical(agestr, {'Young', 'Old'});
T.DOB = DOBstr;
T.Yoke_Mate = categorical(repmat({'<undefined>'},length(ratlist),1), ...
    [{'None'}; ratlist]);
T.CT = true(length(ratlist),1);
T.ICRb = false(length(ratlist),1);
T.ICRi = false(length(ratlist),1);
T.Session_CT_T = zeros(length(ratlist),1);
T.Session_CT_F = zeros(length(ratlist),1);
T.Session_ICRb_T = zeros(length(ratlist),1);
T.Session_ICRb_F = zeros(length(ratlist),1);
T.Session_ICRi_T = zeros(length(ratlist),1);
T.Session_ICRi_F = zeros(length(ratlist),1);
T.Session_Rotation = zeros(length(ratlist),1);
T.Finished_CT = false(length(ratlist),1);
T.Finished_ICRb = false(length(ratlist),1);
T.Finished_ICRi = false(length(ratlist),1);
T.Feeder_Condition = categorical(repmat({'<undefined>'},length(ratlist),1), ...
    {'C1', 'C2'});
T.Rotation_Direction = categorical(repmat({'<undefined>'},length(ratlist),1), ...
    {'CCW', 'CW'});
T.Reward_Delay = categorical(repmat({'<undefined>'},length(ratlist),1), ...
    {'0.1', '0.5', '1.0', '1.5', '2.0', '3.0'}, 'Ordinal', true);
T.Cue_Condition = categorical(repmat({'<undefined>'},length(ratlist),1), ...
    {'All', 'Half', 'None'}, 'Ordinal', true);
T.Pulse_Duration = NaN(length(ratlist),1);
T.Sound_Conditions = ...
    table(false(length(ratlist),1), false(length(ratlist),1), false(length(ratlist),1), ...
    'VariableNames', [{'S1'}, {'S2'}, {'S3'}]);
T.Sound_Conditions = [T.Sound_Conditions.S1, T.Sound_Conditions.S2, T.Sound_Conditions.S3];
T.Air_Conditions = ...
    table(false(length(ratlist),1), false(length(ratlist),1), false(length(ratlist),1), ...
    'VariableNames', [{'A1'}, {'A2'}, {'A3'}]);
T.Air_Conditions = [T.Air_Conditions.A1, T.Air_Conditions.A2, T.Air_Conditions.A3];
T.Start_Quadrant = repmat({categorical(repmat({'<undefined>'},200,1), ...
    {'NE', 'SE', 'SW', 'NW'})}, length(ratlist),1);
T.Rotation_Positions = repmat({categorical(repmat({'<undefined>'},200,9), ...
    {'90', '180', '270'})}, length(ratlist),1);
T.Rotations_Per_Session = repmat({categorical(repmat({'<undefined>'},200,1), ...
    {'2', '4', '6'})}, length(ratlist),1);
T.Rewards_Per_Rotation = repmat({categorical(repmat({'<undefined>'},200,9), ...
    {'5:8', '6:9', '7:10'})}, length(ratlist),1);
T.Days_Till_Rotation = repmat({categorical(repmat({'<undefined>'},200,1), ...
    {'1:2', '2:3', '3:4'})}, length(ratlist),1);
T.Implant_Coordinates = cell(length(ratlist),1);
T.Implant_Configuration = cell(length(ratlist),1);
T.Notes = cell(length(ratlist),1);

% Set variable units
T.Properties.VariableUnits{'Reward_Delay'} = 'sec';
T.Properties.VariableUnits{'Pulse_Duration'} = 'ms';

% Set variable descriptions
T.Properties.VariableDescriptions{'Sound_Conditions'} = '[White, Reward, Aversive]';
T.Properties.VariableDescriptions{'Air_Conditions'} = '[Pump, Vacuum, Ozone]';

% Load existing dataset
if exist(fullfile(OutDir, 'SS_In_All.mat'), 'file')
    S = load(fullfile(OutDir, 'SS_Out_CT.mat'));
    SS_Out_CT = S.SS_Out_CT;
    S = load(fullfile(OutDir, 'SS_Out_ICR.mat'));
    SS_Out_ICR = S.SS_Out_ICR;
    S = load(fullfile(OutDir, 'SS_In_All.mat'));
    SS_In_All = S.SS_In_All;
    
    % Add into main struct
    T = [SS_In_All; T];
end

% Copy sorted rat data
SS_In_All = sortrows(T, 'RowNames');


%% ======================== SS_Out_CT =====================================

% Load existing dataset
if exist(fullfile(OutDir, 'SS_Out_CT.mat'), 'file')
    S = load(fullfile(OutDir, 'SS_Out_CT.mat'));
    SS_Out_CT = S.SS_Out_CT;
end

% Store variables
T = table;
T.Include_Analysis = true;
T.Date = {''};
T.Start_Time = {''};
T.Total_Time = NaN;
T.Track = false;
T.Forage = false;
T.Session_CT_T = NaN;
T.Session_CT_F = NaN;
T.Lap_TS = {[]};
T.Reward_TS = {[]};
T.Reversal_Start_TS = {[]};
T.Reversal_End_TS = {[]};
T.Laps_Total = NaN;
T.Rewards_Total = NaN;
T.Reversals_Total = NaN;
T.Notes = {''};

% Set variable units
T.Properties.VariableUnits{'Total_Time'} = 'min';
T.Properties.VariableUnits{'Lap_TS'} = 'sec';
T.Properties.VariableUnits{'Reward_TS'} = 'sec';
T.Properties.VariableUnits{'Reversal_Start_TS'} = 'sec';
T.Properties.VariableUnits{'Reversal_End_TS'} = 'sec';

% Loop through and create a field for each rat
for z_rat = 1:length(ratlist)
    SS_Out_CT.(ratlist{z_rat}) = T;
end

% Sort fields by rat
[~, ind] = sort(fieldnames(SS_Out_CT));
SS_Out_CT = orderfields(SS_Out_CT, ind);

%% =========================== Save =======================================

% Save out tables
save(fullfile(OutDir,'SS_In_All'), 'SS_In_All')
save(fullfile(OutDir,'SS_Out_CT'), 'SS_Out_CT')


% % Check SS_In_All field class
% flds = SS_In_All.Properties.VariableNames;
% for i = 1:length(flds)
%     sprintf('%s %s', flds{i}, class(SS_In_All.(flds{i})))
%     pause
% end
% % Check SS_Out_ICR field class
% rats = fieldnames(SS_Out_ICR);
% rat = 1;
% flds = SS_Out_ICR.(rats{rat}).Properties.VariableNames;
% for i = 1:length(flds)
%     sprintf('%s %s', flds{i}, class(SS_Out_ICR.(rats{rat}).(flds{i})))
%     pause
% end


