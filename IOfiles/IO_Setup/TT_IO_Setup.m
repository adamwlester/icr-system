function[] = TT_IO_Setup()

%% =========================== SET PARAMETERS =============================
topDir = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running\Main\MATLAB';
ioDir = regexp(topDir,'.*(?=\ICR_Running)','match');
ioDir = fullfile(ioDir{:},'ICR_Running\IOfiles\SessionData');

% Rat numbers (must be preceded by an 'r')
ratList = [...
    {'r0495'} ...
    ];

%% =========================== SETUP VARS =================================

% Define categories
human_cats = {'AWL', 'CB', 'Other'};
bundle_cats = {'HIPP', 'MEC'};
tetrode_cats = ...
    {'TT01','TT02','TT03','TT04','TT05','TT06', ...
    'TT07','TT08','TT09','TT10','TT11','TT12', ...
    'TT13','TT14','TT15','TT16','R1','R2',};
turn_cats = [{'N'},{'NNE'},{'NE'},{'ENE'},{'E'},{'ESE'},{'SE'},{'SSE'},{'S'},...
    {'SSW'},{'SW'},{'WSW'},{'W'},{'WNW'},{'NW'},{'NNW'}];
reference_tt_cats = {'R01_1','R01_2','R02_1','R02_2','CE01','CE02'};
reference_eib_cats = {'R1','R2','R3','R4','R5','R6','R7','R8'};

% Variable descriptions
cap_weights_description = '[Cap, Housing, Washer]';
implant_coordinates_description = '[Bndl_1{A-P, M-L, D-V}, Bndl_2{A-P, M-L, D-V}]';
implant_angle_description = '[Bndl_1{A-P, M-L, D-V}, Bndl_2{A-P, M-L, D-V}]';
implant_configuration_description = '[Bndl_1{A-P, M-L}, Bndl_2{A-P, M-L}]';
tetrode_cats_description = '[Bndl_1{A-P, M-L}, Bndl_2{A-P, M-L}]';
turn_log_F_description = '[Units, Theta, Sharp Waves, Noise, ?, ?]';

% Initialize table
T = table('RowNames',ratList);

% ADD/INITIALIZE TABLE ENTRIES

T.Include_Run = true(length(ratList),1);

T.Weight_Drive = nan(length(ratList),1);

T.Cap_Weights = repmat({nan(1,3)},length(ratList),1);

T.Implant_Coordinates = repmat({nan(1,3), nan(1,3)},length(ratList),1);

T.Implant_Angle = repmat({nan(1,2), nan(1,2)},length(ratList),1);

T.Implant_Configuration = repmat({nan(1,2), nan(1,2)},length(ratList),1);

T.Bundle_Label = repmat({bundle_cats'}, ...
    length(ratList), 1);

T.TT_Label = repmat({tetrode_cats'}, ...
    length(ratList), 1);

T.TT_Mapping = repmat({categorical(repmat({'<undefined>'},18,18), tetrode_cats)}, ...
    length(ratList), 2);

T.TT_Include = repmat({true(18,1)}, length(ratList), 1);

T.Reference_Mapping = ...
    repmat({[reference_tt_cats', ...
    categorical(repmat({'<undefined>'},length(reference_tt_cats),1), reference_eib_cats)]}, ...
    length(ratList), 1);

T.TT_Reference = repmat({categorical(repmat({'<undefined>'},18,1), reference_tt_cats)}, ...
    length(ratList), 1);

T.Thread_Pitch = nan(length(ratList),1);

T.Implant_Notes = cell(length(ratList),1);

% Create turn log
log_var_values = ...
    [ ...
    0; ...
    {''}; ...
    {categorical({'AWL'}, human_cats)}; ...
    repmat( ...
    [ ...
    {0}; ...
    {categorical({'<undefined>'}, turn_cats)}; ...
    {categorical({'<undefined>'}, reference_tt_cats)}; ...
    {false(1,6)}; ...
    {''}], ...
    length(tetrode_cats), 1); ...
    {''} ...
    ];
log_var_names = cellfun(@(x) cellstr([repmat(x, 5, 1), ['_D'; '_O'; '_R'; '_F'; '_N']]), ...
    tetrode_cats', 'uni', false);
log_var_names = [{'Session'}, {'Date'}, {'Human'}, reshape([log_var_names{:}], 1, []), {'Notes'}];
log_table = cell2table(log_var_values', 'VariableNames', log_var_names);

% Set variable units
T.Properties.VariableUnits{'Weight_Drive'} = 'g';
T.Properties.VariableUnits{'Cap_Weights'} = 'g';
T.Properties.VariableUnits{'Implant_Coordinates'} = 'mm';
T.Properties.VariableUnits{'Implant_Angle'} = 'deg';
T.Properties.VariableUnits{'Thread_Pitch'} = 'mm';
log_table.Properties.VariableUnits(3:3:end) = {'um'};

% Set variable descriptions
T.Properties.VariableDescriptions{'Cap_Weights'} = cap_weights_description;
T.Properties.VariableDescriptions{'Implant_Coordinates'} = implant_coordinates_description;
T.Properties.VariableDescriptions{'Implant_Angle'} = implant_angle_description;
T.Properties.VariableDescriptions{'Implant_Configuration'} = implant_configuration_description;
T.Properties.VariableDescriptions{'TT_Mapping'} = tetrode_cats_description;
for z_tt = 1:length(tetrode_cats)
    log_table.Properties.VariableDescriptions{[tetrode_cats{z_tt}, '_F']} = turn_log_F_description;
end

% Add turn log table
T.Turn_Log = ...
    repmat({log_table}, length(ratList), 1);

% Load existing dataset
if exist(fullfile(ioDir, 'TT_IO.mat'), 'file')
    S = load(fullfile(ioDir, 'TT_IO.mat'));
    TT_IO = S.TT_IO;
    
    % Exclude rats already in table
    exc_rats = ratList(ismember(ratList,TT_IO.Properties.RowNames));
    if size(exc_rats,1) > 0
        fprintf('WARNING Rats %s Already in List\n', exc_rats{:});
    end
    T(ismember(T.Properties.RowNames,TT_IO.Properties.RowNames), :) = [];
    
    % Check if nothing changed
    if size(T,1) == 0
        fprintf('WARNING Exited With Nothing Changed\n');
        return
    end
    
    % Add into main table
    T = [TT_IO; T];
    
    % Print saved changes
    ratList = ratList(~ismember(ratList, exc_rats));
    for i = 1:length(ratList)
        fprintf('FINISHED Adding Rat %s\n', char(ratList(i)));
    end
    
end

% Copy sorted rat data
TT_IO = sortrows(T, 'RowNames'); %#ok<NASGU>

%% ========================= UPDATE SS_IO_1 ===============================

% Load existing dataset
if exist(fullfile(ioDir, 'SS_IO_1.mat'), 'file')
    S = load(fullfile(ioDir, 'SS_IO_1.mat'));
    SS_IO_1 = S.SS_IO_1;
    
    % Set implanted flag
    SS_IO_1.Implanted(ismember(SS_IO_1.Properties.RowNames, ratList)) = true;
end
%% =========================== SAVE =======================================

% Save out tables
save(fullfile(ioDir,'SS_IO_1'), 'SS_IO_1')
save(fullfile(ioDir,'TT_IO'), 'TT_IO')

%% =========================== TEMP =======================================

% % Add log tt variable
% S = load(fullfile(ioDir, 'TT_IO.mat'));
% TT_IO = S.TT_IO;
% log_table = repmat(log_table,size(TT_IO.Turn_Log{2,1},1),1);
% for z_tt = 1:length(tetrode_cats)
%     log_table.([tetrode_cats{z_tt},'_D']) = TT_IO.Turn_Log{2,1}.([tetrode_cats{z_tt},'_D']);
%     log_table.([tetrode_cats{z_tt},'_O']) = TT_IO.Turn_Log{2,1}.([tetrode_cats{z_tt},'_O']);
%     log_table.([tetrode_cats{z_tt},'_R']) = TT_IO.Turn_Log{2,1}.([tetrode_cats{z_tt},'_R']);
%     log_table.([tetrode_cats{z_tt},'_N']) = TT_IO.Turn_Log{2,1}.([tetrode_cats{z_tt},'_N']);
% end
% TT_IO.Turn_Log{2,1} = log_table;
% TT_IO(1,1:end-2) = TT_IO(2,1:end-2)

end