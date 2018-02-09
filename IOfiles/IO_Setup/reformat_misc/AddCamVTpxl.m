PAR.DIR.topRun = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running';
PAR.DIR.io = [PAR.DIR.topRun, '\IOfiles\SessionData'];
PAR.DIR.opp = [PAR.DIR.topRun, '\IOfiles\Operational'];
S = load(fullfile(PAR.DIR.io, 'SS_IO_2.mat'));
SS_IO_2 = S.SS_IO_2;
clear S;

oppDir = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running\IOfiles\Operational';

trkBndsFi = [...
    {'track_bounds_171219.mat'}, ...
    {'track_bounds_150804.mat'}, ...
    {'track_bounds_140101.mat'}];

camCfgDate = ...
    {'12/01/17', ...
    '08/01/15'};

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