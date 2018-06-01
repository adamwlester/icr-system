PAR.DIR.topRun = 'C:\Users\lester\repos\icr-system';
PAR.DIR.io = [PAR.DIR.topRun, '\data\session'];
PAR.DIR.opp = [PAR.DIR.topRun, '\data\operational'];
S = load(fullfile(PAR.DIR.io, 'SS_IO_2.mat'));
SS_IO_2 = S.SS_IO_2;
S = load(fullfile(PAR.DIR.io, 'SS_IO_1.mat'));
SS_IO_1 = S.SS_IO_1;
clear S;

camRotChangeDate = '08/01/15';
imgChangeDate = '07/02/17';

ratList = fieldnames(SS_IO_2);
for z_r = 1:length(ratList)
    for z_s = 1:size(SS_IO_2.(ratList{z_r}), 1)
       
        
        if datenum(SS_IO_2.(ratList{z_r}).Date{z_s}, 'yyyy-mm-dd_HH-MM-SS') < datenum(camRotChangeDate, 'mm/dd/yy')
            SS_IO_2.(ratList{z_r}).Camera_Orientation(z_s) = 90;
        elseif SS_IO_1.Feeder_Version(ismember(SS_IO_1.Properties.RowNames, ratList{z_r})) == 'Mobile_Feeder'
            SS_IO_2.(ratList{z_r}).Camera_Orientation(z_s) = -20;
        else
            SS_IO_2.(ratList{z_r}).Camera_Orientation(z_s) = 0;
        end
        
        if datenum(SS_IO_2.(ratList{z_r}).Date{z_s}, 'yyyy-mm-dd_HH-MM-SS') < datenum(imgChangeDate, 'mm/dd/yy')
            if SS_IO_2.(ratList{z_r}).Rotation_Direction(z_s) == 'CCW'
                SS_IO_2.(ratList{z_r}).Image_Orientation{z_s} = [0, -40];
            else
                SS_IO_2.(ratList{z_r}).Image_Orientation{z_s} = [-40, 0];
            end
        else
            if SS_IO_2.(ratList{z_r}).Rotation_Direction(z_s) == 'CCW'
                SS_IO_2.(ratList{z_r}).Image_Orientation{z_s} = [0, -40];
            else
                SS_IO_2.(ratList{z_r}).Image_Orientation{z_s} = [0, 40];
            end
        end
        
        
    end
end
save(fullfile(PAR.DIR.io, 'SS_IO_2.mat'), 'SS_IO_2');