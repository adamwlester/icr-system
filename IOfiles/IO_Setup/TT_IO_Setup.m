function[] = TT_IO_Setup()

%% =========================== Set paramiters =============================
topDir = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running\Main\MATLAB';
ioDir = regexp(topDir,'.*(?=\ICR_Running)','match');
ioDir = fullfile(ioDir{:},'ICR_Running\IOfiles\SessionData');

% Rat numbers (must be preceded by an 'r')
ratList = [...
    {'r0000'} ...
    {'r0499'} ...
    {'r0278'} ...
    {'r9999'} ...
    ];

%% =========================== Setup vars =================================

% Define categories
bundle_labels = {'HIPP', 'Bndl_2'};
tetrode_labels = ...
    {'TT01','TT02','TT03','TT04','TT05','TT06', ...
    'TT07','TT08','TT09','TT10','TT11','TT12', ...
    'TT13','TT14','TT15','TT16','R1','R2',};
turn_labels = [{'N'},{'NNE'},{'NE'},{'ENE'},{'E'},{'ESE'},{'SE'},{'SSE'},{'S'},...
    {'SSW'},{'SW'},{'WSW'},{'W'},{'WNW'},{'NW'},{'NNW'}];
reference_labels = {'R1','R2','CE','Other'};

% Variable descriptions
implant_coordinates_description = '[Bndl_1{A-P, M-L, D-V}, Bndl_2{A-P, M-L, D-V}]';
implant_angle_description = '[Bndl_1{A-P, M-L, D-V}, Bndl_2{A-P, M-L, D-V}]';
implant_configuration_description = '[Bndl_1{A-P, M-L}, Bndl_2{A-P, M-L}]';
tetrode_labels_description = '[Bndl_1{A-P, M-L}, Bndl_2{A-P, M-L}]';

% Initialize table
T = table('RowNames',ratList);

% Add rat level entries
T.Include_Run = true(length(ratList),1);
T.Implant_Coordinates = repmat({nan(1,3), nan(1,3)},length(ratList),1);
T.Implant_Angle = repmat({nan(1,2), nan(1,2)},length(ratList),1);
T.Implant_Configuration = repmat({nan(1,2), nan(1,2)},length(ratList),1);
T.Bundle_Label = repmat({bundle_labels'}, ...
     length(ratList), 1);
T.TT_Label = repmat({tetrode_labels'}, ...
     length(ratList), 1);
T.TT_Mapping = repmat({categorical(repmat({'<undefined>'},18,18), tetrode_labels)}, ...
    length(ratList), 2);
T.TT_Include = repmat({true(18,1)}, length(ratList), 1);
T.TT_Reference = repmat({categorical(repmat({'<undefined>'},18,1), reference_labels)}, ...
     length(ratList), 1);
T.Thread_Pitch = NaN(length(ratList),1);
T.Turn_Log = repmat(...
    {cell2struct(...
    [{NaN}; ...
    {cell2struct(repmat({categorical({'<undefined>'}, turn_labels)},1,18), tetrode_labels, 2)}; ...
    {cell2struct(num2cell(nan(1,18)), tetrode_labels, 2)}; ...
    {cell2struct(cell(1,18), tetrode_labels, 2)}], ...
    {'Date', 'Orientation', 'Depth', 'Notes'})}, ...
    length(ratList), 1);

% Create turn log
log_var_values = ...
    [ ...
    0; ...
    {''}; ...
    repmat( ...
    [ ...
    {0}; ...
    {categorical({'<undefined>'}, turn_labels)}; ...
    {categorical({'<undefined>'}, reference_labels)}; ...
    {''}], ...
    length(tetrode_labels), 1), ...
    ];
log_var_names = cellfun(@(x) cellstr([repmat(x, 4, 1), ['_D'; '_O'; '_R'; '_N']]), ...
    tetrode_labels', 'uni', false);
log_var_names = [{'Session'},{'Date'}, reshape([log_var_names{:}], 1, [])];
log_table = cell2table(log_var_values', 'VariableNames', log_var_names);

% Set variable units
T.Properties.VariableUnits{'Implant_Coordinates'} = 'mm';
T.Properties.VariableUnits{'Implant_Angle'} = 'deg';
T.Properties.VariableUnits{'Thread_Pitch'} = 'mm';
log_table.Properties.VariableUnits(3:3:end) = {'um'};

% Set variable descriptions
T.Properties.VariableDescriptions{'Implant_Coordinates'} = implant_coordinates_description;
T.Properties.VariableDescriptions{'Implant_Angle'} = implant_angle_description;
T.Properties.VariableDescriptions{'Implant_Configuration'} = implant_configuration_description;
T.Properties.VariableDescriptions{'TT_Mapping'} = tetrode_labels_description;

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
end

% Copy sorted rat data
TT_IO = sortrows(T, 'RowNames');

%% =========================== Save =======================================

% Save out table
save(fullfile(ioDir,'TT_IO'), 'TT_IO')

% Print saved changes
for i = 1:length(ratList)
    fprintf('FINISHED Adding Rat %s\n', char(ratList(i)));
end

end