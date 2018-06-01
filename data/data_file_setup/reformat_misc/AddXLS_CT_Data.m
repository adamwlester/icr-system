PAR.DIR.topRun = 'C:\Users\lester\repos\icr-system';
PAR.DIR.io = [PAR.DIR.topRun, '\data\session'];
PAR.DIR.xls = 'C:\Users\lester\repos\z_Obs-Arc\ICR_BP_Spreadsheet\ICR_BP_Spreadsheet.xlsx';
S = load(fullfile(PAR.DIR.io, 'SS_IO_CT.mat'));
SS_IO_CT = S.SS_IO_CT;
S = load(fullfile(PAR.DIR.io, 'SS_IO_2.mat'));
SS_IO_2 = S.SS_IO_2;
clear S;

ratList = fieldnames(SS_IO_2);
for z_r = 1:length(ratList)
    
    rat_lab = ratList{z_r}(2:end);
    if strcmp(rat_lab(1), '0')
        rat_lab = ['1', rat_lab];
    end
    
    if ~any(SS_IO_2.(ratList{z_r}).Feeder_Version=='Static_Feeder')
        continue;
    end
    if any(ismember(fieldnames(SS_IO_CT), ratList{z_r}))
        continue;
    end
    
    fprintf('\r\nChecking Rat %s\r\n',rat_lab)
    
    try
        [num,txt,xls_in] = xlsread(PAR.DIR.xls,rat_lab);
    catch
        continue;
    end
    
    xls_feilds = {'CT', 'Date', '0-Deg', 'Notes', 'Start (clock)', 'End (clock)', 'Run (min)'};
    var_col = NaN(1,length(xls_feilds));
    ct_rows = [];
    
    for z_f = 1:length(xls_feilds)
        for i = 1:size(xls_in,1)
            for j = 1:size(xls_in,2)
                if ischar(xls_in{i,j})
                    if strcmp(xls_in{i,j}, 'CT') && strcmp(xls_feilds{z_f}, 'CT')
                        for ii = i+1:size(xls_in,1)
                            if isnumeric(xls_in{ii,j})
                                if xls_in{ii,j} == 1
                                    ct_rows = [ct_rows, ii];
                                end
                            end
                        end
                    end
                    
                    if strcmp(xls_in{i,j}, xls_feilds{z_f})
                        var_col(z_f) = j;
                        break;
                    end
                end
                if strcmp(xls_in{i,j}, xls_feilds{z_f})
                    break;
                end
            end
            if strcmp(xls_in{i,j}, xls_feilds{z_f})
                break;
            end
        end
    end
    
    SS_IO_CT.(ratList{z_r}) = SS_IO_CT.r0071(1,:);
    SS_IO_CT.(ratList{z_r}).Lap_TS(:) = {[]}; 
    SS_IO_CT.(ratList{z_r}).Reward_TS(:) = {[]};
    SS_IO_CT.(ratList{z_r}).Reversal_Start_TS(:) = {[]};
    SS_IO_CT.(ratList{z_r}).Reversal_End_TS(:) = {[]};
    SS_IO_CT.(ratList{z_r}) = repmat(SS_IO_CT.(ratList{z_r}), length(ct_rows), 1);
   
    for i = 1:length(ct_rows)
        
        for z_f = 2:length(xls_feilds)-1
            col = var_col(z_f);
            
            if strcmp(xls_feilds{z_f}, 'Date')
                date_num = datenum(xls_in{ct_rows(i), col}, 'mm/dd/yyyy');
                date_str = datestr(date_num, 'yyyy-mm-dd_HH-MM-SS');
                SS_IO_CT.(ratList{z_r}).Date{i} = date_str;
            end
            
            if strcmp(xls_feilds{z_f}, '0-Deg')
                SS_IO_CT.(ratList{z_r}).Laps_Standard(i) = xls_in{ct_rows(i), col};
                SS_IO_CT.(ratList{z_r}).Rewards_Standard(i) = xls_in{ct_rows(i), col};
            end
            
            if strcmp(xls_feilds{z_f}, 'Notes')
                SS_IO_CT.(ratList{z_r}).Notes{i} = xls_in{ct_rows(i), col};
            end
            
            if strcmp(xls_feilds{z_f}, 'Start (clock)')
                if strcmp(xls_in{ct_rows(i), col}, 'N/A')
                    t_string = '';
                else
                    t_str_num = xls_in{ct_rows(i), col};
                    if ischar(t_str_num)
                        t_str_num = datenum(t_str_num, 'HH:MM:SS');
                    end
                    t_string = datestr(t_str_num, 'HH:MM:SS');
                end
                SS_IO_CT.(ratList{z_r}).Start_Time{i} = t_string;
            end
            
            if strcmp(xls_feilds{z_f}, 'End (clock)')
                if strcmp(xls_in{ct_rows(i), col}, 'N/A')
                    if isnumeric(xls_in{ct_rows(i), var_col(ismember(xls_feilds,'Run (min)'))})
                        tot_min = xls_in{ct_rows(i), var_col(ismember(xls_feilds,'Run (min)'))};
                    else
                        tot_min = NaN;
                    end
                else
                    t_end_num = xls_in{ct_rows(i), col};
                    if ischar(t_end_num)
                        t_end_num = datenum(t_end_num, 'HH:MM:SS');
                    end
                    tot_min = (t_end_num - t_str_num)*24*60;
                end
                SS_IO_CT.(ratList{z_r}).Total_Time(i) = tot_min;
            end
            
            fprintf('Updated %s %s\n',rat_lab,date_str)
        end
    end
    
end
[~, ind] = sort(fieldnames(SS_IO_CT));
SS_IO_CT = orderfields(SS_IO_CT, ind);
save(fullfile(PAR.DIR.io, 'SS_IO_CT.mat'), 'SS_IO_CT');