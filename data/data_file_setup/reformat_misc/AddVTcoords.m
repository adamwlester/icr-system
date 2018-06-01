PAR.DIR.topRun = 'C:\Users\lester\repos\icr-system';
PAR.DIR.io = [PAR.DIR.topRun, '\data\session'];
PAR.DIR.opp = [PAR.DIR.topRun, '\data\operational'];
S = load(fullfile(PAR.DIR.io, 'SS_IO_2.mat'));
SS_IO_2 = S.SS_IO_2;
clear S;

PAR.DIR.trkBndsFi = [...
    {'track_bounds_140101.mat'}, ...
    {'track_bounds_150804.mat'}];

load(fullfile(PAR.DIR.opp, PAR.DIR.trkBndsFi{1}))
R_1 = R;
XC_1 = XC;
YC_1 = YC;
load(fullfile(PAR.DIR.opp, PAR.DIR.trkBndsFi{2}))
R_2 = R;
XC_2 = XC;
YC_2 = YC;

ratList = fieldnames(SS_IO_2);
for z_r = 1:length(ratList)
%     fprintf('%s R: %0.2f XC: %0.2f YC: %0.2f\n', ratList{z_r}, ...
%         SS_IO_2.(ratList{z_r}).VT_Pixel_Coordinates{1});
    for z_s = 1:size(SS_IO_2.(ratList{z_r}), 1)
        if ~all(isnan(SS_IO_2.(ratList{z_r}).VT_Pixel_Coordinates{z_s}))
            continue;
        end
        if datenum(SS_IO_2.(ratList{z_r}).Date{z_s}, 'yyyy-mm-dd_HH-MM-SS') < datenum('08/01/15', 'mm/dd/yy')
            SS_IO_2.(ratList{z_r}).VT_Pixel_Coordinates{z_s} = [R_1, XC_1, YC_1];
        else
            SS_IO_2.(ratList{z_r}).VT_Pixel_Coordinates{z_s} = [R_2, XC_2, YC_2];
        end
    end
end
%save(fullfile(PAR.DIR.io, 'SS_IO_2.mat'), 'SS_IO_2');