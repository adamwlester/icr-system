% Use this code to convert from old CT session data
% Make sure to create a SS_In_All and SS_Out_CT MAT file with at least one
% rat. This will be overwriten with imported rat data
function[] = CT_IOReformat()

%% Set paramiters
OutDir = ...
    fullfile(pwd, '\IOfiles\SessionData');
InDir = ...
    fullfile(pwd, '\EXTERNAL_TRACK\IOfiles');

% Load SS_In_All and SS_Out_CT
S = load(fullfile(OutDir, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
S = load(fullfile(OutDir, 'SS_Out_CT.mat'));
SS_Out_CT = S.SS_Out_CT;
clear S

% Get list of CT rats
ctRatList = dir(InDir);
ctRatList = {ctRatList(3:end).name}';
ctRatList = ctRatList(1:end-2);

flds = fieldnames(SS_Out_CT);
template = SS_Out_CT.(flds {1});
SS_Out_CT = struct;

% Loop through rat files
for z_rat = 1:length(ctRatList)
    load(fullfile(InDir,ctRatList{z_rat}))
    ratLab = ['r',ctRatList{z_rat}(1:4)];
    SS_Out_CT.(ratLab) = repmat(template,size(CT_Ses,2),1);
    SS_Out_CT.(ratLab).Include = true(size(CT_Ses,2),1);
    SS_Out_CT.(ratLab).Date = {CT_Ses(:).Date}';
    Start_Time = regexp(SS_Out_CT.(ratLab).Date, '(?<=_)\d*.*', 'match');
    Start_Time = cellfun(@(x) x{:}, Start_Time, 'Uni', false);
    SS_Out_CT.(ratLab).Start_Time = ...
        cellfun(@(x) datestr(datenum(x,'HH-MM-SS'),'HH:MM:SS'), Start_Time, 'Uni', false);
    SS_Out_CT.(ratLab).Total_Time = [CT_Ses(:).TotalTime]' / 60;
    SS_Out_CT.(ratLab).CT_Track = true(size(CT_Ses,2),1);
    SS_Out_CT.(ratLab).CT_Track_Session = (1:size(CT_Ses,2))';
    Lap_TS = {CT_Ses(:).IR2_TS}';
    Lap_TS = cellfun(@(x) x/10^6, Lap_TS, 'Uni', false);
    SS_Out_CT.(ratLab).Lap_TS = Lap_TS;
    Reward_TS = {CT_Ses(:).IR1_TS}';
    Reward_TS = cellfun(@(x) x/10^6, Reward_TS, 'Uni', false);
    SS_Out_CT.(ratLab).Reward_TS = Reward_TS;
    Reversal_Start_TS = {CT_Ses(:).RevStrTS}';
    Reversal_Start_TS = cellfun(@(x) x/10^6, Reversal_Start_TS, 'Uni', false);
    SS_Out_CT.(ratLab).Reversal_Start_TS = Reversal_Start_TS;
    Reversal_End_TS = {CT_Ses(:).RevEndTS}';
    Reversal_End_TS = cellfun(@(x) x/10^6, Reversal_End_TS, 'Uni', false);
    SS_Out_CT.(ratLab).Reversal_End_TS = Reversal_End_TS;
    SS_Out_CT.(ratLab).Laps_Total = ...
        cell2mat(cellfun(@(x) length(x), Lap_TS, 'Uni', false));
    SS_Out_CT.(ratLab).Rewards_Total = ...
        cell2mat(cellfun(@(x) length(x), Reward_TS, 'Uni', false));
    SS_Out_CT.(ratLab).Reversals_Total = ...
        cell2mat(cellfun(@(x) length(x), Reversal_Start_TS, 'Uni', false));
    % Update SS_In_All
    SS_In_All.CT_Track_Session(ismember(SS_In_All.Properties.RowNames, ratLab)) = ...
        SS_Out_CT.(ratLab).CT_Track_Session(end);
end

% Save data
save(fullfile(OutDir, 'SS_In_All.mat'), 'SS_In_All')
save(fullfile(OutDir, 'SS_Out_CT.mat'), 'SS_Out_CT')
