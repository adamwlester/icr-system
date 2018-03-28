function[] = TT_IO_Setup()

%% =========================== SET PARAMETERS =============================
topDir = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running\Main\MATLAB';
ioDir = regexp(topDir,'.*(?=\ICR_Running)','match');
ioDir = fullfile(ioDir{:},'ICR_Running\IOfiles\SessionData');

% Rat numbers (must be preceded by an 'r')
PAR.ratList = [...
    {'r9999'}; ...
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
tt_D_description = '[TT Depth]';
tt_O_description = '[TT Screw Orientation]';
tt_R_description = '[TT Reference]';
tt_F_description = '[Units, Theta, Ripples, Sharp Waves, Noise, Exclude TT, Exclude E1, Exclude E2, Exclude E3, Exclude E4]';
tt_I_description = '[E1 Impedance, E2 Impedance, E3 Impedance, E4 Impedance]';

%% ========================= UPDATE TT_IO_1 ===============================

% Initialize table
T = table('RowNames',PAR.ratList);

% ADD/INITIALIZE TABLE ENTRIES

T.Include_Run = true(length(PAR.ratList),1);

T.Weight_Drive = nan(length(PAR.ratList),1);

T.Cap_Weights = repmat({nan(1,3)},length(PAR.ratList),1);

T.Implant_Coordinates = repmat({nan(1,3), nan(1,3)},length(PAR.ratList),1);

T.Implant_Angle = repmat({nan(1,2), nan(1,2)},length(PAR.ratList),1);

T.Implant_Configuration = repmat({nan(1,2), nan(1,2)},length(PAR.ratList),1);

T.Bundle_Label = repmat({bundle_cats'}, ...
    length(PAR.ratList), 1);

T.TT_Label = repmat({tetrode_cats'}, ...
    length(PAR.ratList), 1);

T.TT_Mapping = repmat({categorical(repmat({'<undefined>'},18,18), tetrode_cats)}, ...
    length(PAR.ratList), 2);

T.Reference_Mapping = ...
    repmat({[reference_tt_cats', ...
    categorical(repmat({'<undefined>'},length(reference_tt_cats),1), reference_eib_cats)]}, ...
    length(PAR.ratList), 1);

T.TT_Reference = repmat({categorical(repmat({'<undefined>'},18,1), reference_tt_cats)}, ...
    length(PAR.ratList), 1);

T.Thread_Pitch = nan(length(PAR.ratList),1);

T.Implant_Notes = cell(length(PAR.ratList),1);

% Set variable units
T.Properties.VariableUnits{'Weight_Drive'} = 'g';
T.Properties.VariableUnits{'Cap_Weights'} = 'g';
T.Properties.VariableUnits{'Implant_Coordinates'} = 'mm';
T.Properties.VariableUnits{'Implant_Angle'} = 'deg';
T.Properties.VariableUnits{'Thread_Pitch'} = 'mm';

% Set variable descriptions
T.Properties.VariableDescriptions{'Cap_Weights'} = cap_weights_description;
T.Properties.VariableDescriptions{'Implant_Coordinates'} = implant_coordinates_description;
T.Properties.VariableDescriptions{'Implant_Angle'} = implant_angle_description;
T.Properties.VariableDescriptions{'Implant_Configuration'} = implant_configuration_description;
T.Properties.VariableDescriptions{'TT_Mapping'} = tetrode_cats_description;

% Load existing dataset
if exist(fullfile(ioDir, 'TT_IO_1.mat'), 'file')
    S = load(fullfile(ioDir, 'TT_IO_1.mat'));
    TT_IO_1 = S.TT_IO_1;
    
    % Exclude rats already in table
    exc_rats = PAR.ratList(ismember(PAR.ratList,TT_IO_1.Properties.RowNames));
    if ~isempty(exc_rats)
        fprintf('**WARNING**: Rats %s Already in List\n', exc_rats{:});
    end
    T(ismember(T.Properties.RowNames,TT_IO_1.Properties.RowNames), :) = [];
    
    % Get include rat indeces
     inc_ind = ~ismember(PAR.ratList, exc_rats);
     
    % Bail if no new rats
    if ~any(inc_ind)
        fprintf('**WARNING**: Exited With Nothing Changed\n');
        return
    end
    
    % Update PAR variables and exclude redundant rats
    fld_list = fieldnames(PAR);
    for z_p = 1:length(fld_list)
        PAR.(fld_list{z_p}) = ...
            PAR.(fld_list{z_p})(inc_ind);
    end
    
    % Add into main table
    T = [TT_IO_1; T];
    
end

% Copy sorted rat data
TT_IO_1 = sortrows(T, 'RowNames'); %#ok<NASGU>

%% ========================= UPDATE TT_IO_2 ===============================

% Load existing dataset
if exist(fullfile(ioDir, 'TT_IO_2.mat'), 'file')
    S = load(fullfile(ioDir, 'TT_IO_2.mat'));
    TT_IO_2 = S.TT_IO_2;
end

% Reinitialize table
T = table;

% CREATE TT SPECIFIC ENTRIES

% Create cell mat
tt_var_values = ...
    repmat( ...
    [ ...
    {0}; ...
    {categorical({'<undefined>'}, turn_cats)}; ...
    {categorical({'<undefined>'}, reference_tt_cats)}; ...
    {false(1,10)}; ...
    {nan(1,4)}; ...
    {''}], ...
    length(tetrode_cats), 1);

% Create list of var names
tt_var_suffix = ['_D'; '_O'; '_R'; '_F'; '_I'; '_N'];
tt_var_names = ...
    cellfun(@(x) cellstr([repmat(x, length(tt_var_suffix), 1), tt_var_suffix])', ...
    tetrode_cats, 'uni', false);
tt_var_names = ([tt_var_names{:}])';

% Convert to table
t = ...
    cell2table(tt_var_values', ...
    'VariableNames', tt_var_names);

% STORE/ADD TABLE ENTRIES

T.Session = 0;

T.Date = {''};

T.Recording_Dir = {''};

T.Raw_Data_Dir = {''};

T.Human = categorical({'<undefined>'}, ...
    human_cats);

T = [T, t];

T.Notes = {''};

% Set variable units
T.Properties.VariableUnits = repmat({''}, 1, size(T,2));
for z_tt = 1:length(tetrode_cats)
    T.Properties.VariableUnits{[tetrode_cats{z_tt}, '_D']} = 'um';
    T.Properties.VariableUnits{[tetrode_cats{z_tt}, '_O']} = 'cardinal';
    T.Properties.VariableUnits{[tetrode_cats{z_tt}, '_I']} = 'MOhm';
end

% Set variable descriptions
for z_tt = 1:length(tetrode_cats)
    T.Properties.VariableDescriptions{[tetrode_cats{z_tt}, '_D']} = tt_D_description;
    T.Properties.VariableDescriptions{[tetrode_cats{z_tt}, '_O']} = tt_O_description;
    T.Properties.VariableDescriptions{[tetrode_cats{z_tt}, '_R']} = tt_R_description;
    T.Properties.VariableDescriptions{[tetrode_cats{z_tt}, '_F']} = tt_F_description;
    T.Properties.VariableDescriptions{[tetrode_cats{z_tt}, '_I']} = tt_I_description;
end

% Loop through and create a field for each rat
for z_rat = 1:length(PAR.ratList)
    TT_IO_2.(PAR.ratList{z_rat}) = T;
end

% Sort fields by rat
[~, ind] = sort(fieldnames(TT_IO_2));
TT_IO_2 = orderfields(TT_IO_2, ind); %#ok<NASGU>

%% ========================= UPDATE SS_IO_1 ===============================

% Load existing dataset
if exist(fullfile(ioDir, 'SS_IO_1.mat'), 'file')
    S = load(fullfile(ioDir, 'SS_IO_1.mat'));
    SS_IO_1 = S.SS_IO_1;
    
    % Set implanted flag
    SS_IO_1.Implanted(ismember(SS_IO_1.Properties.RowNames, PAR.ratList)) = true;
end

%% =========================== SAVE =======================================

% Save out tables
save(fullfile(ioDir,'TT_IO_1'), 'TT_IO_1')
save(fullfile(ioDir,'TT_IO_2'), 'TT_IO_2')
save(fullfile(ioDir,'SS_IO_1'), 'SS_IO_1')

% Print saved changes
for i = 1:length(PAR.ratList)
    fprintf('FINISHED: Adding Rat %s\n', char(PAR.ratList(i)));
end


%% =========================== TEMP =======================================

% Modify flag variable
% S = load(fullfile(ioDir, 'TT_IO_2.mat'));
% TT_IO_2 = S.TT_IO_2;
% ratList = fieldnames(TT_IO_2);
% for z_r = 1:length(ratList)
%     log_table = TT_IO_2.(ratList{z_r});
%     for z_tt = 1:length(tetrode_cats)
%         log_table.([tetrode_cats{z_tt},'_F']) = ...
%             [TT_IO_2.(ratList{z_r}).([tetrode_cats{z_tt},'_F'])(:,1:5), false(size(log_table,1), 5)];
%         log_table.Properties.VariableDescriptions{[tetrode_cats{z_tt}, '_F']} = tt_F_description;
%     end
%     TT_IO_2.(ratList{z_r}) = log_table;
% end

S = load(fullfile(ioDir, 'TT_IO_2.mat'));
TT_IO_2 = S.TT_IO_2;
ratList = fieldnames(TT_IO_2);
for z_r = 1:length(ratList)
    log_table = TT_IO_2.(ratList{z_r});
    for z_tt = 1:length(tetrode_cats)
          log_table.Properties.VariableDescriptions{[tetrode_cats{z_tt}, '_F']} = tt_F_description;
    end
    TT_IO_2.(ratList{z_r}) = log_table;
end


end