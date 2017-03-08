% Use this code to convert from old CSV formated session data
% Make sure to create a SS_In_All and SS_Out_ICR MAT file with at least one
% rat. This will be overwriten with imported rat data
function[] = SesIOReformat()

%% Set paramiters
OutDir = ...
    fullfile(pwd, '\IOfiles\SessionData');
InDir{1} = ...
    fullfile(pwd, '\ICR_ARENA\IOfiles\SessionData');
InDir{2} = ...
    fullfile(pwd, '\EXTERNAL_TRACK\IOfiles');

% Load in old session data
csvFi{1} = 'SesConds1.csv';
csvFi{2} = 'SesConds2.csv';
csvFi{3} = 'SesConds3.csv'; % vals [1, 2, 3] = [90, 180, 270]
csvFi{4} = 'SesConds.csv'; % CT dataset
% Get session info
SsDat{1} = CSV2Cell(fullfile(InDir{1}, csvFi{1}));
SsHed{1} = SsDat{1}(1,:); % get headers
SsDat{1} =  cell2mat(SsDat{1}(2:end,:)); % convert to double
SsDat{2} = CSV2Cell(fullfile(InDir{1}, csvFi{2}));
SsHed{2} = SsDat{2}(1,:); % get headers
SsDat{2} =  cell2mat(SsDat{2}(2:end,:)); % convert to double
SsDat{3} = CSV2Cell(fullfile(InDir{1}, csvFi{3}));
SsHed{3} = SsDat{3}(1,:); % get headers
SsDat{3} =  cell2mat(SsDat{3}(2:end,3:end)); % convert to double
SsDat{4} = CSV2Cell(fullfile(InDir{2}, csvFi{4}));
SsHed{4} =  SsDat{4}(1,:); % get headers
SsDat{4} =  cell2mat(SsDat{4}(2:end,:)); % convert to double

% Load in template new session MAT files
S = load(fullfile(OutDir, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
S = load(fullfile(OutDir, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S

% Get rat list
icrRatList = SsDat{1}(:,1);
ctRatList = SsDat{4}(:,1);
allRatList = sort([icrRatList; ctRatList]);
allRatList = unique(allRatList);

% Var strings
agestr = [{'Young'}, {'Old'}]; % age group
rdstr = [{'CCW'}, {''}, {'CW'}]; % rotation direction
sqstr = [{'NE'}, {'SE'}, {'SW'}, {'NW'}]; % start quadrant
rparr = [90, 180, 270]; % rotation position
custr = [{'None'}, {'Half'},{'All'}]; % cue condition

% Loop through each rat and copy over ICR data
for z_rat = 1:length(allRatList)
    rat_num = allRatList(z_rat);
    rat_lab = ['r',num2str(rat_num)];
    ranCT = any(ismember(ctRatList, rat_num));
    ranICR =  false;
    if any(ismember(icrRatList, rat_num))
        if SsDat{1}(ismember(icrRatList, rat_num),end) == 1
            ranICR = true;
        end
    end
    
    % Display current rat
    sprintf('RUNNING RAT %d\n%s\r\n', rat_num, datestr(clock,'HH:MM:SS'))
    
    % Get ICR data
    if ranICR
        icr_ind = find(icrRatList == rat_num);
        icrDat = CSV2Cell(fullfile(InDir{1}, [rat_lab(2:end),'.csv']));
        icrHed = icrDat(1,:);
        icrDat = icrDat(2:end,:);
        % Have to change indexing for some files missing Reversal column
        if ~strcmp(icrHed{12}, 'Reversals')
            coloff = 0;
        else
            coloff = 1;
        end
    end
    if ranCT
        ct_ind = find(ctRatList == rat_num);
        load(fullfile(InDir{2}, [rat_lab(2:end),'.mat']));
        ctDat = CT_Ses; clear CT_Ses
    end
    
    % Get data for fields
    Include = true;
    Rat = rat_lab;
    ICRi = false;
    CT_Forage_Sessions = 0;
    ICRb_Forage_Sessions = 0;
    ICRi_Track_Sessions = 0;
    ICRi_Forage_Sessions = 0;
    ICRi_Finished = false;
    Implant_Coordinates = [];
    Implant_Configuration = [];
    if ranCT
        Age_Group = agestr{SsDat{4}(ct_ind,2)};
        DOB = datestr(SsDat{4}(ct_ind,3), 'yyyy/mm/dd');
        Yoke_Mate = rat_lab;
        CT = true;
        ICRb = false;
        CT_Track_Sessions = size(ctDat,2);
        ICRb_Track_Sessions = 0;
        Rotation_Sessions = 0;
        CT_Finished = false;
        ICRb_Finished = false;
        Feeder_Condition = [];
        Rotation_Direction = [];
        Reward_Ratio = [];
        Reward_Delay = [];
        Cue_Condition = [];
        Pulse_Duration = [];
        Sound_Conditions = [];
        Smell_Conditions = [];
        Start_Quadrant = [];
        Rotation_Positions = [];
        Rotations_Per_Session = [];
        Rewards_Per_Rotation = [];
        Days_Till_Rotation = [];
    end
    if ranICR
        Age_Group = agestr{SsDat{1}(icr_ind,2)};
        DOB = datestr(SsDat{1}(icr_ind,3), 'yyyy/mm/dd');
        Yoke_Mate = ['r',num2str(SsDat{1}(SsDat{1}(icr_ind,6),1))];
        CT = false;
        ICRb = true;
        if ~ranCT
            CT_Track_Sessions = 0;
        end
        ICRb_Track_Sessions = size(icrDat,1);
        Rotation_Sessions = sum([icrDat{:,coloff+16}] > 0);
        CT_Finished = true;
        ICRb_Finished = true;
        Feeder_Condition = SsDat{1}(icr_ind,4);
        Rotation_Direction = rdstr{SsDat{1}(icr_ind,5)+2};
        Reward_Ratio = num2str(icrDat{end,7});
        Reward_Ratio = [repmat('1:',length(Reward_Ratio),1), Reward_Ratio];
        Reward_Delay = icrDat{end,8};
        Cue_Condition = icrDat{end,9};
        Pulse_Duration = icrDat{end,10};
        if length(icrDat{end,11}) == 1
            sn = icrDat{end,11};
            Smell_Conditions = false;
            if sn == 0
                Sound_Conditions = [false, false, false];
            elseif sn == 1
                Sound_Conditions = [true, false, false];
            elseif sn == 2
                Sound_Conditions = [false, true, false];
            elseif sn == 3
                Sound_Conditions = [true, true, false];
            end
        else
            Sound_Conditions = logical(icrDat{end,11}(1:3));
            Smell_Conditions = logical(icrDat{end,11}(4));
        end
        Start_Quadrant = sqstr(SsDat{2}(:,icr_ind+1));
        Rotation_Positions = rparr(reshape(SsDat{3}(:,icr_ind),9,[])');
        Rotation_Positions = [Rotation_Positions; NaN(100, 9)]; % add NaNs
        Rotations_Per_Session = [icrDat{[icrDat{:,coloff+16}]>0,coloff+16}]';
        Rotations_Per_Session = [Rotations_Per_Session; ...
            NaN(200-length(Rotations_Per_Session),1)]; % add NaNs
        Rewards_Per_Rotation = cell(200,9);
        Days_Till_Rotation = diff(find([icrDat{:,coloff+16}]>0 == 1))';
        Days_Till_Rotation = [Days_Till_Rotation; ...
            NaN(200-length(Days_Till_Rotation),1)]; % add NaNs
        Days_Till_Rotation = mat2cell([NaN(length(Days_Till_Rotation),1), ...
            Days_Till_Rotation],ones(1,length(Days_Till_Rotation)),2);
    end
    
    % Fill in fields for SS_In_All
    SS_In_All(z_rat).Include = Include;
    SS_In_All(z_rat).Rat = Rat;
    SS_In_All(z_rat).Age_Group = Age_Group;
    SS_In_All(z_rat).DOB = DOB;
    SS_In_All(z_rat).Yoke_Mate = Yoke_Mate;
    SS_In_All(z_rat).CT = CT;
    SS_In_All(z_rat).ICRb = ICRb;
    SS_In_All(z_rat).ICRi = ICRi;
    SS_In_All(z_rat).CT_Track_Sessions = CT_Track_Sessions;
    SS_In_All(z_rat).CT_Forage_Sessions = CT_Forage_Sessions;
    SS_In_All(z_rat).ICRb_Track_Sessions = ICRb_Track_Sessions;
    SS_In_All(z_rat).ICRb_Forage_Sessions = ICRb_Forage_Sessions;
    SS_In_All(z_rat).ICRi_Track_Sessions = ICRi_Track_Sessions;
    SS_In_All(z_rat).ICRi_Forage_Sessions = ICRi_Forage_Sessions;
    SS_In_All(z_rat).Rotation_Sessions = Rotation_Sessions;
    SS_In_All(z_rat).CT_Finished = CT_Finished;
    SS_In_All(z_rat).ICRb_Finished = ICRb_Finished;
    SS_In_All(z_rat).ICRi_Finished = ICRi_Finished;
    SS_In_All(z_rat).Feeder_Condition = Feeder_Condition;
    SS_In_All(z_rat).Rotation_Direction = Rotation_Direction;
    SS_In_All(z_rat).Reward_Ratio = Reward_Ratio;
    SS_In_All(z_rat).Reward_Delay = Reward_Delay;
    SS_In_All(z_rat).Cue_Condition = Cue_Condition;
    SS_In_All(z_rat).Pulse_Duration = Pulse_Duration;
    SS_In_All(z_rat).Sound_Conditions = Sound_Conditions;
    SS_In_All(z_rat).Smell_Conditions = Smell_Conditions;
    SS_In_All(z_rat).Start_Quadrant = Start_Quadrant;
    SS_In_All(z_rat).Rotation_Positions = Rotation_Positions;
    SS_In_All(z_rat).Rotations_Per_Session = Rotations_Per_Session;
    SS_In_All(z_rat).Rewards_Per_Rotation = Rewards_Per_Rotation;
    SS_In_All(z_rat).Days_Till_Rotation = Days_Till_Rotation;
    SS_In_All(z_rat).Implant_Coordinates = Implant_Coordinates;
    SS_In_All(z_rat).Implant_Configuration = Implant_Coordinates;
    
    % Get data for SS_Out_ICR
    if ranICR
        
        icrdays = find([icrDat{:,coloff+16}]>0);
        % Initialize rat specific structure
        nms = fieldnames(SS_Out_ICR);
        nms = fieldnames(SS_Out_ICR.(nms{1}));
        S = cell2struct(num2cell(NaN(length(nms), size(icrDat,1))), nms);
        
        % Pull out data
        Date = icrDat(:,2);
        Recording_Directory = icrDat(:,3);
        Start_Time = regexp(icrDat(:,2), '(?<=_)\d*.*', 'match');
        Start_Time = cellfun(@(x) x{:}, Start_Time, 'Uni', false);
        Start_Time = cellfun(@(x) datestr(datenum(x,'HH-MM-SS'),'HH:MM:SS'), Start_Time, 'Uni', false);
        Total_Time = cellfun(@(x) x/60, icrDat(:,4), 'Uni', false);
        ICRb_Track = num2cell(true(size(icrDat,1),1));
        ICRb_Forage = num2cell(false(size(icrDat,1),1));
        ICRi_Track = num2cell(false(size(icrDat,1),1));
        ICRi_Forage = num2cell(false(size(icrDat,1),1));
        ICR_Rotation = num2cell(false(size(icrDat,1),1));
        ICR_Rotation(icrdays) = {true};
        ICRb_Track_Session = icrDat(:,1);
        ICRb_Forage_Session = num2cell(zeros(size(icrDat,1),1));
        ICRi_Track_Session = num2cell(zeros(size(icrDat,1),1));
        ICRi_Forage_Session = num2cell(zeros(size(icrDat,1),1));
        Rotation_Session = num2cell(zeros(size(icrDat,1),1));
        Rotation_Session(icrdays) = num2cell(1:length(icrdays));
        Feeder_Condition = icrDat(:,5);
        Rotation_Direction = rdstr([icrDat{:,6}]+2);
        Reward_Ratio = num2str(num2str([icrDat{:,7}]'));
        Reward_Ratio = cellstr([repmat('1:',length(Reward_Ratio),1), Reward_Ratio]);
        Reward_Delay = icrDat(:,8);
        Cue_Condition = custr([icrDat{:,9}]+1)';
        Pulse_Duration = icrDat(:,10);
        Sound_Conditions = cell(size(icrDat,1),1);
        Smell_Conditions = cell(size(icrDat,1),1);
        for i = 1:size(icrDat,1)
            if length(icrDat{i,11}) == 1
                sn = icrDat{i,11};
                Smell_Conditions{i} = false;
                if sn == 0
                    Sound_Conditions{i} = [false, false, false];
                elseif sn == 1
                    Sound_Conditions{i} = [true, false, false];
                elseif sn == 2
                    Sound_Conditions{i} = [false, true, false];
                elseif sn == 3
                    Sound_Conditions{i} = [true, true, false];
                end
            else
                    Sound_Conditions{i} = logical(icrDat{i,11}(1:3));
                    Smell_Conditions{i} = logical(icrDat{i,11}(4));
            end
        end
        Start_Quadrant = Start_Quadrant(1:size(icrDat,1))';
        Rotation_Positions = cell(size(icrDat,1),1);
        Rotations_Per_Session = cell(size(icrDat,1),1);
        Rewards_Per_Rotation = cell(size(icrDat,1),1);
        Days_Till_Rotation = cell(size(icrDat,1),1);
        rp = rparr(reshape(SsDat{3}(:,icr_ind),9,[])');
        rps = [icrDat{[icrDat{:,coloff+16}]>0,coloff+16}]';
        rpr = cell(200,9);
        dtr =  diff(find([icrDat{:,coloff+16}]>0 == 1))';
        cnt = 0;
        for i = 1:length(icrdays)
            cnt = cnt+1;
            Rotation_Positions{icrdays(i)} = rp(icrdays(i),1:rps(cnt));
            Rotations_Per_Session{icrdays(i)} = rps(cnt);
            Rewards_Per_Rotation{icrdays(i)} =  rpr(cnt,1:rps(cnt));
            if i ~= length(icrdays)
                Days_Till_Rotation{icrdays(i)} = dtr(cnt);
            end
        end
        Reversals = cell(size(icrDat,1),1);
        if strcmp(icrHed{12}, 'Reversals')
            Reversals = num2cell(zeros(size(icrDat,1),1));
            ind = find([icrDat{:,12}]>0, 1, 'first');
            Reversals(ind:end) = icrDat(ind:end,12);
        end
        Standard_Rewards = icrDat(:,coloff+12);
        Rotated_Rewards = icrDat(:,coloff+13);
        Standard_Laps = icrDat(:,coloff+14);
        Rotated_Laps = icrDat(:,coloff+15);
        
        % Fill in fields for SS_Out_ICR
        [S.Date] = Date{:};
        [S.Recording_Directory] = Recording_Directory{:};
        [S.Start_Time] = Start_Time{:};
        [S.Total_Time] = Total_Time{:};
        [S.ICRb_Track] = ICRb_Track{:};
        [S.ICRb_Forage] = ICRb_Forage{:};
        [S.ICRi_Track] = ICRi_Track{:};
        [S.ICRi_Forage] = ICRi_Forage{:};
        [S.ICR_Rotation] = ICR_Rotation{:};
        [S.ICRb_Track_Session] = ICRb_Track_Session{:};
        [S.ICRb_Forage_Session] = ICRb_Forage_Session{:};
        [S.ICRi_Track_Session] = ICRi_Track_Session{:};
        [S.ICRi_Forage_Session] = ICRi_Forage_Session{:};
        [S.Rotation_Session] = Rotation_Session{:};
        [S.Feeder_Condition] = Feeder_Condition{:};
        [S.Rotation_Direction] = Rotation_Direction{:};
        [S.Reward_Ratio] = Reward_Ratio{:};
        [S.Reward_Delay] = Reward_Delay{:};
        [S.Cue_Condition] = Cue_Condition{:};
        [S.Pulse_Duration] = Pulse_Duration{:};
        [S.Sound_Conditions] = Sound_Conditions{:};
        [S.Smell_Conditions] = Smell_Conditions{:};
        [S.Start_Quadrant] = Start_Quadrant{:};
        [S.Rotation_Positions] = Rotation_Positions{:};
        [S.Rotations_Per_Session] = Rotations_Per_Session{:};
        [S.Rewards_Per_Rotation] = Rewards_Per_Rotation{:};
        [S.Days_Till_Rotation] = Days_Till_Rotation{:};
        [S.Reversals] = Reversals{:};
        [S.Standard_Rewards] = Standard_Rewards{:};
        [S.Rotated_Rewards] = Rotated_Rewards{:};
        [S.Standard_Laps] = Standard_Laps{:};
        [S.Rotated_Laps] = Rotated_Laps{:};
        
        % Add to main data structure
        SS_Out_ICR.(rat_lab) = S;
    end
    
end

% Remove existing rats from SS_Out_ICR struct
list = fieldnames(SS_Out_ICR);
listNum = char(list);
listNum = str2double(listNum(:,2:end));
SS_Out_ICR = rmfield(SS_Out_ICR, list(~ismember(listNum, icrRatList)));

% Save out data
save(fullfile(OutDir,'SS_In_All'), 'SS_In_All')
save(fullfile(OutDir,'SS_Out_ICR'), 'SS_Out_ICR')

% % Check SS_In_All field class
% flds = fieldnames(SS_In_All);
% rat = 1;
% for i = 1:length(flds)
%     sprintf('%s %s', flds{i}, class(SS_In_All(rat).(flds{i})))
%     pause
% end
% % Check SS_In_All field class
% rats = fieldnames(SS_Out_ICR);
% rat = 1;
% flds = fieldnames(SS_Out_ICR.(rats{rat}));
% for i = 1:length(flds)
%     sprintf('%s %s', flds{i}, class(SS_Out_ICR.(rats{rat})(1).(flds{i})))
%     pause
% end