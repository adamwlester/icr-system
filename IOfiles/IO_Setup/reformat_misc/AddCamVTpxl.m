PAR.DIR.topRun = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running';
PAR.DIR.io = [PAR.DIR.topRun, '\IOfiles\SessionData'];
PAR.DIR.opp = [PAR.DIR.topRun, '\IOfiles\Operational'];
S = load(fullfile(PAR.DIR.io, 'SS_IO_1.mat'));
SS_IO_1 = S.SS_IO_2;
S = load(fullfile(PAR.DIR.io, 'SS_IO_2.mat'));
SS_IO_2 = S.SS_IO_2;
clear S;

camCfgDate = '05/01/16';



ratList = fieldnames(SS_IO_2);
for z_r = 1:length(ratList)
    for z_s = 1:size(SS_IO_2.(ratList{z_r}), 1)
        
        
        if datenum(SS_IO_2.(ratList{z_r}).Recording_File{z_s}, 'yyyy-mm-dd_HH-MM-SS') < datenum(camCfgDate{2}, 'mm/dd/yy')
            
            load(fullfile(oppDir, trkBndsFi{3}))
            SS_IO_2.(ratList{z_r}).VT_Pixel_Coordinates{z_s} = [R, XC, YC];
            
        elseif datenum(SS_IO_2.(ratList{z_r}).Recording_File{z_s}, 'yyyy-mm-dd_HH-MM-SS') < datenum(camCfgDate{1}, 'mm/dd/yy')
            
            load(fullfile(oppDir, trkBndsFi{2}))
            SS_IO_2.(ratList{z_r}).VT_Pixel_Coordinates{z_s} = [R, XC, YC];
            
        else
            
            load(fullfile(oppDir, trkBndsFi{1}))
            SS_IO_2.(ratList{z_r}).VT_Pixel_Coordinates{z_s} = [R, XC, YC];
            
        end
        
        
    end
end
save(fullfile(PAR.DIR.io, 'SS_IO_2.mat'), 'SS_IO_2');