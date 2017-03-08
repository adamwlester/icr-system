function [] = SesIOReformat_v3()


%% Changes for robot
% 16/10/12

% PAR.Dir.io = [pwd, '\IOfiles\SessionData'];
% 
% % Import session data
% S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
% SS_In_All = S.SS_In_All;
% S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
% SS_Out_ICR = S.SS_Out_ICR;
% clear S;
% 
% ratList = fieldnames(SS_Out_ICR);
% 
% % add condition
% SS_In_All.CT = categorical(repmat({'<undefined>'},height(SS_In_All) ,1), ...
%     {'Manual_Training', 'Behavior_Training', 'Implant_Training', 'Rotation'});
% SS_In_All.Properties.VariableNames{'CT'} = 'Session_Condition';
% 
% SS_In_All.Session_Condition(SS_In_All.ICRb) = 'Behavior_Training';
% 
% SS_In_All.ICRb = [];
% SS_In_All.ICRi = [];
% 
% SS_In_All.Finished_CT(:) = false;
% % change finish stuff
% SS_In_All.Properties.VariableNames{'Finished_CT'} = 'Finished_Manual_Training';
% SS_In_All.Properties.VariableNames{'Finished_ICRb'} = 'Finished_Behavior_Training';
% SS_In_All.Properties.VariableNames{'Finished_ICRi'} = 'Finished_Implant_Training';
% 
% temp1 = [SS_In_All.Session_ICRb_F, SS_In_All.Session_ICRb_F];
% temp2 = [SS_In_All.Session_ICRb_T, SS_In_All.Session_ICRb_F];
% temp3 = [SS_In_All.Session_ICRi_T, SS_In_All.Session_ICRi_F];
% SS_In_All.Session_CT_F = [];
% SS_In_All.Session_ICRb_F = [];
% SS_In_All.Session_ICRi_F = [];
% SS_In_All.Session_CT_T = temp1;
% SS_In_All.Properties.VariableNames{'Session_CT_T'} = 'Session_Manual_Training';
% SS_In_All.Session_ICRb_T = temp2;
% SS_In_All.Properties.VariableNames{'Session_ICRb_T'} = 'Session_Behavior_Training';
% SS_In_All.Session_ICRi_T = temp3;
% SS_In_All.Properties.VariableNames{'Session_ICRi_T'} = 'Session_Implant_Training';
% 
% % remove air condition
% SS_In_All.Air_Conditions = [];
% 
% % change sound stuff
% SS_In_All.Sound_Conditions = ...
%     [SS_In_All.Sound_Conditions(:,1), ...
%     SS_In_All.Sound_Conditions(:,2)];
% SS_In_All.Properties.VariableDescriptions{'Sound_Conditions'} = '[White, Reward]';
% 
% % change reward delay
% SS_In_All.Reward_Delay = renamecats(SS_In_All.Reward_Delay,{'0.1'},{'0.0'});
% 
% for z_rat = 1:length(ratList)
%     rat_field = ratList{z_rat};
%     
%     bind = SS_Out_ICR.(rat_field).ICRb;
%     rind = SS_Out_ICR.(rat_field).Rotation;
%     % add condtion
%     SS_Out_ICR.(rat_field).ICRb = categorical(repmat({'<undefined>'},height(SS_Out_ICR.(rat_field)) ,1), ...
%         {'Manual_Training', 'Behavior_Training', 'Implant_Training', 'Rotation'});
%     SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRb'} = 'Session_Condition';
%     
%     SS_Out_ICR.(rat_field).Session_Condition(bind) = 'Behavior_Training';
%     SS_Out_ICR.(rat_field).Session_Condition(rind) = 'Rotation';
%     
%     % add task
%     SS_Out_ICR.(rat_field).ICRi = categorical(repmat({'<undefined>'},height(SS_Out_ICR.(rat_field)) ,1), {'Track', 'Forage'});
%     SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRi'} = 'Session_Task';
%     SS_Out_ICR.(rat_field).Session_Task(:) = 'Track';
%     
%     temp1 = [SS_Out_ICR.(rat_field).Session_ICRb_F, SS_Out_ICR.(rat_field).Session_ICRb_F];
%     temp2 = [SS_Out_ICR.(rat_field).Session_ICRb_T, SS_Out_ICR.(rat_field).Session_ICRb_F];
%     temp3 = [SS_Out_ICR.(rat_field).Session_ICRi_T, SS_Out_ICR.(rat_field).Session_ICRi_F];
%     SS_Out_ICR.(rat_field).Track = [];
%     SS_Out_ICR.(rat_field).Forage = [];
%     SS_Out_ICR.(rat_field).Rotation = [];
%     SS_Out_ICR.(rat_field).Session_ICRi_F = [];
%     SS_Out_ICR.(rat_field).Session_ICRb_T = temp1;
%     SS_Out_ICR.(rat_field).Properties.VariableNames{'Session_ICRb_T'} = 'Session_Manual_Training';
%     SS_Out_ICR.(rat_field).Session_ICRb_F = temp2;
%     SS_Out_ICR.(rat_field).Properties.VariableNames{'Session_ICRb_F'} = 'Session_Behavior_Training';
%     SS_Out_ICR.(rat_field).Session_ICRi_T = temp3;
%     SS_Out_ICR.(rat_field).Properties.VariableNames{'Session_ICRi_T'} = 'Session_Implant_Training';
%     
%     % remove air
%     SS_Out_ICR.(rat_field).Air_Conditions = [];
%     
%     % change sound stuff
%     SS_Out_ICR.(rat_field).Sound_Conditions = ...
%         [SS_Out_ICR.(rat_field).Sound_Conditions(:,1), ...
%         SS_Out_ICR.(rat_field).Sound_Conditions(:,2)];
%     SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Sound_Conditions'} = '[White, Reward]';
%     
%     % change reward delay
%     SS_Out_ICR.(rat_field).Reward_Delay = renamecats(SS_Out_ICR.(rat_field).Reward_Delay,{'0.1'},{'0.0'});
%     
%     % change reversal to bulldoze
%     SS_Out_ICR.(rat_field).Properties.VariableNames{'Reversals'} = 'Bulldozings';
%     SS_Out_ICR.(rat_field).Bulldozings(:) = NaN;
%     
% end
% 
% save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')
% save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')

%% 16/10/22

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S;

session_number_description = '[Track, Forage]';
sound_conditions_description = '[White, Reward]';
rotations_per_session_description = '[2,4,6]';
laps_per_rotation_description = '[5:10]';
days_till_rotation_description = '[1:4]';

% Remove CT only rats
SS_In_All(SS_In_All.Session_Behavior_Training(:,1)==0 & SS_In_All.Session_Manual_Training(:,1)==0,:) = [];

ratList = fieldnames(SS_Out_ICR);

% change variable name
SS_In_All.Properties.VariableNames{'Rewards_Per_Rotation'} = 'Laps_Per_Rotation';

% change categories
cats = categorical({'<undefined>'}, ...
    {'NE', 'NW', 'SW', 'SE'});
SS_In_All.Reward_Delay = categorical(SS_In_All.Reward_Delay, ...
    {'0.0', '0.5', '1.0', '1.5', '2.0', '2.5', '3.0'}, 'Ordinal', true);

SS_In_All.Properties.VariableDescriptions{'Session_Manual_Training'} = session_number_description;
SS_In_All.Properties.VariableDescriptions{'Session_Behavior_Training'} = session_number_description;
SS_In_All.Properties.VariableDescriptions{'Session_Implant_Training'} = session_number_description;
SS_In_All.Properties.VariableDescriptions{'Sound_Conditions'} = sound_conditions_description;
SS_In_All.Properties.VariableDescriptions{'Rotations_Per_Session'} = rotations_per_session_description;
SS_In_All.Properties.VariableDescriptions{'Laps_Per_Rotation'} = laps_per_rotation_description;
SS_In_All.Properties.VariableDescriptions{'Days_Till_Rotation'} = days_till_rotation_description;

for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
    
    SS_In_All.Start_Quadrant{z_rat} = categorical(SS_In_All.Start_Quadrant{z_rat}, ...
        {'NE', 'NW', 'SW', 'SE'});
    SS_In_All.Rotation_Positions{z_rat,:} = categorical(SS_In_All.Rotation_Positions{z_rat,:}, ...
        {'90', '180', '270'});
    rotations = SS_In_All.Rotations_Per_Session{z_rat,:};
    rotations(rotations == '4') = '6';
    rotations(rotations == '2') = '4';
    rotations(rotations == '1') = '2';
    SS_In_All.Rotations_Per_Session{z_rat,:} = categorical(rotations, ...
        {'2', '4', '6'});
    
    SS_Out_ICR.(rat_field).Reward_Delay = categorical(SS_Out_ICR.(rat_field).Reward_Delay, ...
        {'0.0', '0.5', '1.0', '1.5', '2.0', '2.5', '3.0'}, 'Ordinal', true);
    SS_Out_ICR.(rat_field).Start_Quadrant = categorical(SS_Out_ICR.(rat_field).Start_Quadrant, ...
        {'NE', 'NW', 'SW', 'SE'});
    
    Rotation_Positions = SS_Out_ICR.(rat_field).Rotation_Positions;
    SS_Out_ICR.(rat_field).Rotation_Positions = cell(size(SS_Out_ICR.(rat_field).Rotation_Positions));
    SS_Out_ICR.(rat_field).Properties.VariableNames{'Rewards_Per_Rotation'} = 'Laps_Per_Rotation';
    for i = 1:size(SS_Out_ICR.(rat_field),1)
        x = [];
        if SS_Out_ICR.(rat_field).Session_Condition(i) == 'Rotation'
            x = str2num(str2mat(Rotation_Positions{i}))';
        else
            x = [];
        end
        SS_Out_ICR.(rat_field).Rotation_Positions{i} = x;
        x = [];
        if SS_Out_ICR.(rat_field).Session_Condition(i) == 'Rotation'
            x = SS_Out_ICR.(rat_field).Laps_Standard{i};
            for j = 1:max(length(SS_Out_ICR.(rat_field).Laps_40_Deg{i}), ...
                    length(SS_Out_ICR.(rat_field).Laps_0_Deg{i}))
                if length(SS_Out_ICR.(rat_field).Laps_40_Deg{i}) >= j
                    x = [x, SS_Out_ICR.(rat_field).Laps_40_Deg{i}(j)];
                end
                if length(SS_Out_ICR.(rat_field).Laps_0_Deg{i}) >= j
                    x = [x, SS_Out_ICR.(rat_field).Laps_0_Deg{i}(j)];
                end
            end
        else
            x = [];
        end
        SS_Out_ICR.(rat_field).Laps_Per_Rotation{i} = x;
    end
    
    SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Session_Manual_Training'} = session_number_description;
    SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Session_Behavior_Training'} = session_number_description;
    SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Session_Implant_Training'} = session_number_description;
    SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Sound_Conditions'} = sound_conditions_description;
    SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Rotations_Per_Session'} = rotations_per_session_description;
    SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Laps_Per_Rotation'} = laps_per_rotation_description;
    SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Days_Till_Rotation'} = days_till_rotation_description;
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')
save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')
