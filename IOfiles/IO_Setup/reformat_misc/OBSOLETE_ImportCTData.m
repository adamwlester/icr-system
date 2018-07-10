PAR.DIR.topRun = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running';
PAR.DIR.io = [PAR.DIR.topRun, '\IOfiles\SessionData'];
PAR.DIR.opp = [PAR.DIR.topRun, '\IOfiles\Operational'];
PAR.DIR.ctold = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\z_Obs-Arc\SessionData_Backup\EXTERNAL_TRACK_v1\IOfiles\SessionData';
S = load(fullfile(PAR.DIR.io, 'SS_IO_CT.mat'));
SS_IO_CT = S.SS_IO_CT;
clear S;

ratList = dir(PAR.DIR.ctold);
ratList = {ratList(:).name};
ratList = ratList(3:end);
ratList = cellfun(@(x) x(1:4), ratList, 'uni', false);

for z_r = 1:length(ratList)
    
    r_fld = ['r',ratList{z_r}];
    
    S = load(fullfile(PAR.DIR.ctold, [ratList{z_r},'.mat']));
    CT_Ses = S.CT_Ses;
    clear S;
    SS_IO_CT.(r_fld) = struct2table(CT_Ses);
    SS_IO_CT.(r_fld).Properties.VariableNames{ismember(SS_IO_CT.(r_fld).Properties.VariableNames, 'Session')} = 'Include_Analysis';
    SS_IO_CT.(r_fld).Properties.VariableNames{ismember(SS_IO_CT.(r_fld).Properties.VariableNames, 'IR1_TS')} = 'Lap_TS';
    SS_IO_CT.(r_fld).Properties.VariableNames{ismember(SS_IO_CT.(r_fld).Properties.VariableNames, 'IR2_TS')} = 'Reward_TS';
    SS_IO_CT.(r_fld).Properties.VariableNames{ismember(SS_IO_CT.(r_fld).Properties.VariableNames, 'RevStrTS')} = 'Reversal_Start_TS';
    SS_IO_CT.(r_fld).Properties.VariableNames{ismember(SS_IO_CT.(r_fld).Properties.VariableNames, 'RevEndTS')} = 'Reversal_End_TS';
    
    SS_IO_CT.(r_fld).Include_Analysis(:) = 1;
    SS_IO_CT.(r_fld).Include_Analysis = logical(SS_IO_CT.(r_fld).Include_Analysis);
    SS_IO_CT.(r_fld).Laps_Standard = cell2mat(cellfun(@(x) length(x), SS_IO_CT.(r_fld).Lap_TS, 'uni', false));
    SS_IO_CT.(r_fld).Rewards_Standard = cell2mat(cellfun(@(x) length(x), SS_IO_CT.(r_fld).Reward_TS, 'uni', false));
    if isa(SS_IO_CT.(r_fld).Reversal_Start_TS, 'double')
        SS_IO_CT.(r_fld).Reversal_Start_TS = num2cell(SS_IO_CT.(r_fld).Reversal_Start_TS);
    end
    if ~isempty(SS_IO_CT.(r_fld).Reversal_Start_TS)
        SS_IO_CT.(r_fld).Reversals_Total = cell2mat(cellfun(@(x) max(length(x), ~isempty(x)), SS_IO_CT.(r_fld).Reversal_Start_TS, 'uni', false));
    else
        SS_IO_CT.(r_fld).Reversals_Total = zeros(size(SS_IO_CT.(r_fld), 1),1);
    end
    SS_IO_CT.(r_fld).Notes = repmat({''}, size(SS_IO_CT.(r_fld), 1),1);
    
end
[~, ind] = sort(fieldnames(SS_IO_CT));
SS_IO_CT = orderfields(SS_IO_CT, ind);
save(fullfile(PAR.DIR.io, 'SS_IO_CT.mat'), 'SS_IO_CT');