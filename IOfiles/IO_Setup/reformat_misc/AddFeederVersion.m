PAR.DIR.topRun = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running';
PAR.DIR.io = [PAR.DIR.topRun, '\IOfiles\SessionData'];
PAR.DIR.opp = [PAR.DIR.topRun, '\IOfiles\Operational'];
S = load(fullfile(PAR.DIR.io, 'SS_IO_1.mat'));
SS_IO_1 = S.SS_IO_1;
S = load(fullfile(PAR.DIR.io, 'SS_IO_2.mat'));
SS_IO_2 = S.SS_IO_2;
clear S;

feedChangeDate = '05/01/16';

SS_IO_1.Feeder_Version = addcats(SS_IO_1.Feeder_Version,'Both');

ratList = fieldnames(SS_IO_2);
for z_r = 1:length(ratList)
    
    for z_s = 1:size(SS_IO_2.(ratList{z_r}), 1)
        
        
        if datenum(SS_IO_2.(ratList{z_r}).Date{z_s}, 'yyyy-mm-dd_HH-MM-SS') > datenum(feedChangeDate, 'mm/dd/yy')
            
            SS_IO_2.(ratList{z_r}).Feeder_Version(z_s) = 'Mobile_Feeder';
            
        end
        
    end
    
    if any(SS_IO_2.(ratList{z_r}).Feeder_Version == 'Static_Feeder') && any(SS_IO_2.(ratList{z_r}).Feeder_Version == 'Mobile_Feeder')
    
    SS_IO_1.Feeder_Version(ismember(SS_IO_1.Properties.RowNames,ratList{z_r})) = 'Both';
    
    end
end
save(fullfile(PAR.DIR.io, 'SS_IO_1.mat'), 'SS_IO_1');
save(fullfile(PAR.DIR.io, 'SS_IO_2.mat'), 'SS_IO_2');