function [] = START_GUI()
% Get pop-up position
sc = get(0,'MonitorPositions');
sc1 = sc(1,:);
sc2 = sc(2,:);

% Display question dialogue on first monitor
qstDlfPos = [(sc1(3)/2), (sc1(4)/2)];

% Exit MATLAB
choice = questdlgAWL('What do you want to run?', ...
    'RUN', 'ICR GUI', 'CT GUI', 'MATLAB', 'MATLAB', qstDlfPos);
% Handle response
switch choice
    case 'ICR GUI'
        drawnow; % force update UI
        % RUN ICR_BEHAVIOR.m
        ICR_GUI();
    case 'CT GUI'
        drawnow; % force update UI
        % RUN CT_GUI.m
        CT_GUI();
        drawnow; % force update UI
    case 'MATLAB'
        return;
end