PAR.DIR.topRun = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running';
PAR.DIR.io = [PAR.DIR.topRun, '\IOfiles\SessionData'];
S = load(fullfile(PAR.DIR.io, 'SS_IO_CT.mat'));
SS_IO_CT = S.SS_IO_CT;
clear S;

ratList = fieldnames(SS_IO_CT);
for z_r = 1:length(ratList)
    
    r_fld = ratList{z_r};
    
    SS_IO_CT.(r_fld).Track = [];
    SS_IO_CT.(r_fld).Forage = [];
    SS_IO_CT.(r_fld).Session_CT_T = [];
    SS_IO_CT.(r_fld).Session_CT_F = [];
    SS_IO_CT.(r_fld).Properties.VariableNames{ismember(SS_IO_CT.(r_fld).Properties.VariableNames, 'Laps_Total')} = 'Laps_Standard';
    SS_IO_CT.(r_fld).Properties.VariableNames{ismember(SS_IO_CT.(r_fld).Properties.VariableNames, 'Rewards_Total')} = 'Rewards_Standard';
    
end
save(fullfile(PAR.DIR.io, 'SS_IO_CT.mat'), 'SS_IO_CT');