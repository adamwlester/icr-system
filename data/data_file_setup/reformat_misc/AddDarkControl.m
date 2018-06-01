PAR.DIR.topRun = 'C:\Users\lester\repos\icr-system';
PAR.DIR.io = [PAR.DIR.topRun, '\data\session'];
PAR.DIR.opp = [PAR.DIR.topRun, '\data\operational'];
S = load(fullfile(PAR.DIR.io, 'SS_IO_1.mat'));
SS_IO_1 = S.SS_IO_1;
S = load(fullfile(PAR.DIR.io, 'SS_IO_2.mat'));
SS_IO_2 = S.SS_IO_2;
clear S;

SS_IO_1.Session_Condition = addcats(SS_IO_1.Session_Condition,'Dark_Control');

ratList = fieldnames(SS_IO_2);
for z_r = 1:length(ratList)
            SS_IO_2.(ratList{z_r}).Session_Condition = addcats(SS_IO_2.(ratList{z_r}).Session_Condition,'Dark_Control');
end
save(fullfile(PAR.DIR.io, 'SS_IO_1.mat'), 'SS_IO_1');
save(fullfile(PAR.DIR.io, 'SS_IO_2.mat'), 'SS_IO_2');