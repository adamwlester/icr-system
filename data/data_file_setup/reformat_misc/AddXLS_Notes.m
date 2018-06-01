PAR.DIR.topRun = 'C:\Users\lester\repos\icr-system';
PAR.DIR.io = [PAR.DIR.topRun, '\data\session'];
PAR.DIR.xls = 'C:\Users\lester\repos\z_Obs-Arc\ICR_BP_Spreadsheet\ICR_BP_Spreadsheet.xlsx';
S = load(fullfile(PAR.DIR.io, 'SS_IO_CT.mat'));
SS_IO_CT = S.SS_IO_CT;
clear S;

ratList = fieldnames(SS_IO_CT);
for z_r = 1:length(ratList)
    
    rat_lab = ratList{z_r}(2:end);
    if strcmp(rat_lab(1), '0')
        rat_lab = ['1', rat_lab];
    end
    
    fprintf('\r\nChecking Rat %s\r\n',rat_lab)
    
    try
        [num,txt,xls_in] = xlsread(PAR.DIR.xls,rat_lab);
    catch
        continue;
    end
    
    SS_IO_CT.(ratList{z_r}).Notes = repmat({''}, length(SS_IO_CT.(ratList{z_r}).Notes), 1);
    
    xls_dates_str = [];
    xls_notes = [];
    for i = 1:size(xls_in,1)
        for j = 1:size(xls_in,2)
            if ischar(xls_in{i,j})
                if strcmp(xls_in{i,j}, 'Date')
                    xls_dates_str = xls_in(i+1:size(xls_in,1),j);
                end
                if strcmp(xls_in{i,j}, 'Notes')
                    xls_notes = xls_in(i+1:size(xls_in,1),j);
                end
            end
        end
    end
    
    tab_dates_num = cell2mat(cellfun(@(x) floor(datenum(x, 'yyyy-mm-dd_HH-MM-SS')), ...
        SS_IO_CT.(ratList{z_r}).Date, 'uni', false));
    
    for z_xls = 1:length(xls_dates_str)
        try
            xls_dates_num = datenum(xls_dates_str{z_xls},'mm/dd/yyyy');
        catch
            continue;
        end
        
        date_ind = knnsearch(tab_dates_num, xls_dates_num);
        if abs(tab_dates_num(date_ind) - xls_dates_num) > 1
            continue;
        end
        
        fprintf('Updated %s %s\n',rat_lab,xls_dates_str{z_xls})
        SS_IO_CT.(ratList{z_r}).Notes(date_ind) = xls_notes(z_xls);
        
    end
    
end
save(fullfile(PAR.DIR.io, 'SS_IO_CT.mat'), 'SS_IO_CT');