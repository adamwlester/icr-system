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

% Rats to exclude from ICR data
% Get rat list
icrRatList_Num = SsDat{1}(:,1);
icrRatList_Str = cellstr([repmat('r',length(icrRatList_Num),1), num2str(icrRatList_Num)]);
ctRatList_Num = SsDat{4}(:,1);
ctRatList_Str = cellstr([repmat('r',length(ctRatList_Num),1), num2str(ctRatList_Num)]);
allRatList_Num = sort([icrRatList_Num; ctRatList_Num]);
allRatList_Num = unique(allRatList_Num);
allRatList_Str = cellstr([repmat('r',length(allRatList_Num),1), num2str(allRatList_Num)]);

% Var strings
agCtg = categories(SS_In_All.Age_Group); % feeder condition
fdCtg = categories(SS_In_All.Feeder_Condition); % feeder condition
rdCtg = categories(SS_In_All.Rotation_Direction); % rotation direction
rdCtg = [rdCtg{1},{''},rdCtg{2}];
rrCtg = categories(SS_In_All.Reward_Ratio); % reward ratio
rwdlCtg = categories(SS_In_All.Reward_Delay); % reward delay
ccCtg = categories(SS_In_All.Cue_Condition); % scue condition
ccCtg = flipud(ccCtg);
sqCtg = categories(SS_In_All.Start_Quadrant{1,1}); % start quadrant
rpCtg = categories(SS_In_All.Rotation_Positions{1,1}); % rotation position
rsCtg = categories(SS_In_All.Rotations_Per_Session{1,1}); % rotation per session
rprCtg = categories(SS_In_All.Rewards_Per_Rotation{1,1}); % rewards per rotation
dtrCtg = categories(SS_In_All.Days_Till_Rotation{1,1}); % days till rotation

% Preallocate new table
SS_In_All = SS_In_All(1,:);
SS_In_All.Properties.RowNames = {};
SS_In_All = repmat(SS_In_All, length(allRatList_Str), 1);
SS_In_All.Properties.RowNames = allRatList_Str;
flds = fieldnames(SS_Out_ICR);
S = struct;
for z_rat = 1:length(icrRatList_Str)
    S.(icrRatList_Str{z_rat}) = SS_Out_ICR.(flds{1});
end
SS_Out_ICR = S;

% Loop through each rat and copy over ICR data
for z_rat = 1:length(allRatList_Str)
    rat_num = str2double(allRatList_Str{z_rat}(2:end));
    rat_lab = ['r',num2str(rat_num)];
    ranCT = any(ismember(ctRatList_Num, rat_num));
    ranICR =  false;
    if any(ismember(icrRatList_Num, rat_num))
        if SsDat{1}(ismember(icrRatList_Num, rat_num),end) == 1
            ranICR = true;
        end
    end
    
    % Display current rat
    sprintf('RUNNING RAT %d\n%s\r\n', rat_num, datestr(clock,'HH:MM:SS'))
    
    % Get ICR data
    if ranICR
        icr_ind = find(icrRatList_Num == rat_num);
        icrDat = CSV2Cell(fullfile(InDir{1}, [rat_lab(2:end),'.csv']));
        icrHed = icrDat(1,:);
        icrDat = icrDat(2:end,:);
        % Have to change indexing for some files missing Reversal column
        if ~strcmp(icrHed{12}, 'Reversals')
            coloff = 0;
        else
            coloff = 1;
        end
        icrdays = find([icrDat{:,coloff+16}]>0);
    end
    if ranCT
        ct_ind = find(ctRatList_Num == rat_num);
        load(fullfile(InDir{2}, [rat_lab(2:end),'.mat']));
        ctDat = CT_Ses; clear CT_Ses;
    end
    
    % Get data for fields
    Include = true;
    ICRi = false;
    CT_Forage_Session = 0;
    ICRb_Forage_Session = 0;
    ICRi_Track_Session = 0;
    ICRi_Forage_Session = 0;
    ICRi_Finished = false;
    Implant_Coordinates = {[]};
    Implant_Configuration = {[]};
    Notes = {[]};
    if ranCT
        Age_Group = agCtg(SsDat{4}(ct_ind,2));
        DOB = {datestr(SsDat{4}(ct_ind,3), 'yyyy/mm/dd')};
        Yoke_Mate = {rat_lab};
        CT = true;
        ICRb = false;
        CT_Track_Session = size(ctDat,2);
        ICRb_Track_Session = 0;
        Rotation_Session = 0;
        CT_Finished = false;
        ICRb_Finished = false;
        Feeder_Condition = {'<undefined>'};
        Rotation_Direction = {'<undefined>'};
        Reward_Ratio = {'<undefined>'};
        Reward_Delay = {'<undefined>'};
        Cue_Condition = {'<undefined>'};
        Pulse_Duration = NaN;
        Sound_Conditions = [false, false, false];
        Air_Conditions = [false, false];
        Start_Quadrant = categorical(repmat({'<undefined>'},200,1), ...
            {'NE', 'SE', 'SW', 'NW'});
        Rotation_Positions = categorical(repmat({'<undefined>'},200,9), ...
            {'90', '180', '270'});
        Rotations_Per_Session = categorical(repmat({'<undefined>'},200,1), ...
            {'2', '4', '6'});
        Rewards_Per_Rotation = categorical(repmat({'<undefined>'},200,9), ...
            {'5:8', '6:9', '7:10'});
        Days_Till_Rotation = categorical(repmat({'<undefined>'},200,1), ...
            {'1:2', '2:3', '3:4'});
    end
    if ranICR
        Age_Group = agCtg(SsDat{1}(icr_ind,2));
        DOB = {datestr(SsDat{1}(icr_ind,3), 'yyyy/mm/dd')};
        Yoke_Mate = {['r',num2str(SsDat{1}(SsDat{1}(icr_ind,6),1))]};
        CT = false;
        ICRb = true;
        if ~ranCT
            CT_Track_Session = 0;
        end
        ICRb_Track_Session = size(icrDat,1);
        Rotation_Session = sum([icrDat{:,coloff+16}] > 0);
        CT_Finished = true;
        ICRb_Finished = true;
        Feeder_Condition = fdCtg(SsDat{1}(icr_ind,4));
        Rotation_Direction = rdCtg(SsDat{1}(icr_ind,5)+2);
        Reward_Ratio = num2str(icrDat{end,7});
        Reward_Ratio = {([ Reward_Ratio,repmat(':1',length(Reward_Ratio),1)])};
        Reward_Delay = {sprintf('%1.1f', icrDat{end,8})};
        if ~ismember(Reward_Delay, rwdlCtg)
            ind = find(str2double(char(rwdlCtg)) > str2double(char(Reward_Delay)), 1, 'first');
            SS_In_All.Reward_Delay = addcats(SS_In_All.Reward_Delay,Reward_Delay,'Before',rwdlCtg{ind});
        end
        Cue_Condition = ccCtg(icrDat{end,9}+1);
        Pulse_Duration = icrDat{end,10};
        if length(icrDat{end,11}) == 1
            sn = icrDat{end,11};
            Air_Conditions = [false, false];
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
            Air_Conditions = [logical(icrDat{end,11}(4)), false];
        end
        Start_Quadrant = categorical(sqCtg(SsDat{2}(:,icr_ind+1)), sqCtg);
        Rotation_Positions = rpCtg(reshape(SsDat{3}(:,icr_ind),9,[])');
        Rotation_Positions = categorical([Rotation_Positions; repmat({'<undefined>'},100, 9)], rpCtg);
        ind = [icrDat{[icrDat{:,coloff+16}]>0,coloff+16}]';
        if isempty(ind)
            Rotations_Per_Session = cell(0,0);
        else
            Rotations_Per_Session =    cellstr(num2str(ind));
        end
        ctg = unique([Rotations_Per_Session;rsCtg]);
        ctg(ismember(ctg,{'<undefined>'})) = [];
        Rotations_Per_Session = categorical([Rotations_Per_Session; ...
            repmat({'<undefined>'},200-length(Rotations_Per_Session),1)], ctg);
        Rewards_Per_Rotation = categorical(repmat({'<undefined>'},200, 9),rprCtg);
        if length(icrdays) > 1
            Days_Till_Rotation = cellstr(num2str(diff(find([icrDat{:,coloff+16}]>0 == 1))'));
            ctg = unique([Days_Till_Rotation;dtrCtg]);
            ctg(ismember(ctg,{'<undefined>'})) = [];
            Days_Till_Rotation = categorical([Days_Till_Rotation; ...
                repmat({'<undefined>'},200-length(Days_Till_Rotation),1)], ctg);
        else
            Days_Till_Rotation = categorical(repmat({'<undefined>'},200,1), ...
                {'1:2', '2:3', '3:4'});
        end
    end
    
    
    
    % Fill in fields for SS_In_All
    SS_In_All{rat_lab,{'Include'}} = Include;
    SS_In_All{rat_lab,{'Age_Group'}} = Age_Group;
    SS_In_All{rat_lab,{'DOB'}} = DOB;
    SS_In_All{rat_lab,{'Yoke_Mate'}} = Yoke_Mate;
    SS_In_All{rat_lab,{'CT'}} = CT;
    SS_In_All{rat_lab,{'ICRb'}} = ICRb;
    SS_In_All{rat_lab,{'ICRi'}} = ICRi;
    SS_In_All{rat_lab,{'CT_Track_Session'}} = CT_Track_Session;
    SS_In_All{rat_lab,{'CT_Forage_Session'}} = CT_Forage_Session;
    SS_In_All{rat_lab,{'ICRb_Track_Session'}} = ICRb_Track_Session;
    SS_In_All{rat_lab,{'ICRb_Forage_Session'}} = ICRb_Forage_Session;
    SS_In_All{rat_lab,{'ICRi_Track_Session'}} = ICRi_Track_Session;
    SS_In_All{rat_lab,{'ICRi_Forage_Session'}} = ICRi_Forage_Session;
    SS_In_All{rat_lab,{'Rotation_Session'}} = Rotation_Session;
    SS_In_All{rat_lab,{'CT_Finished'}} = CT_Finished;
    SS_In_All{rat_lab,{'ICRb_Finished'}} = ICRb_Finished;
    SS_In_All{rat_lab,{'ICRi_Finished'}} = ICRi_Finished;
    SS_In_All{rat_lab,{'Feeder_Condition'}} = Feeder_Condition;
    SS_In_All{rat_lab,{'Rotation_Direction'}} = Rotation_Direction;
    SS_In_All{rat_lab,{'Reward_Ratio'}} = Reward_Ratio;
    SS_In_All{rat_lab,{'Reward_Delay'}} = Reward_Delay;
    SS_In_All{rat_lab,{'Cue_Condition'}} = Cue_Condition;
    SS_In_All{rat_lab,{'Pulse_Duration'}} = Pulse_Duration;
    SS_In_All{rat_lab,{'Sound_Conditions'}} = Sound_Conditions;
    SS_In_All{rat_lab,{'Air_Conditions'}} = Air_Conditions;
    SS_In_All{rat_lab,{'Start_Quadrant'}}{:} = Start_Quadrant;
    SS_In_All{rat_lab,{'Rotation_Positions'}}{:} = Rotation_Positions;
    SS_In_All{rat_lab,{'Rotations_Per_Session'}}{:} = Rotations_Per_Session;
    SS_In_All{rat_lab,{'Rewards_Per_Rotation'}}{:} = Rewards_Per_Rotation;
    SS_In_All{rat_lab,{'Days_Till_Rotation'}}{:} = Days_Till_Rotation;
    SS_In_All{rat_lab,{'Implant_Coordinates'}} = Implant_Coordinates;
    SS_In_All{rat_lab,{'Implant_Configuration'}} = Implant_Configuration;
    SS_In_All{rat_lab,{'Notes'}} = Notes;
    
    % Get data for SS_Out_ICR
    if ranICR
        
        % Resize table
        SS_Out_ICR.(allRatList_Str{z_rat}) = ...
            repmat(SS_Out_ICR.(allRatList_Str{z_rat}), size(icrDat,1), 1);
        
        % Pull out data
        Include = true(size(icrDat,1),1);
        Date = icrDat(:,2);
        Recording_Directory = icrDat(:,3);
        Start_Time = regexp(icrDat(:,2), '(?<=_)\d*.*', 'match');
        Start_Time = cellfun(@(x) x{:}, Start_Time, 'Uni', false);
        Start_Time = cellfun(@(x) datestr(datenum(x,'HH-MM-SS'),'HH:MM:SS'), Start_Time, 'Uni', false);
        Total_Time = ([icrDat{:,4}])' / 60;
        ICRb_Track = true(size(icrDat,1),1);
        ICRb_Forage = false(size(icrDat,1),1);
        ICRi_Track = false(size(icrDat,1),1);
        ICRi_Forage = false(size(icrDat,1),1);
        ICR_Rotation = false(size(icrDat,1),1);
        ICR_Rotation(icrdays) = true;
        ICRb_Track_Session = ([icrDat{:,1}])';
        ICRb_Forage_Session = zeros(size(icrDat,1),1);
        ICRi_Track_Session = zeros(size(icrDat,1),1);
        ICRi_Forage_Session = zeros(size(icrDat,1),1);
        Rotation_Session = zeros(size(icrDat,1),1);
        Rotation_Session(icrdays) = (1:length(icrdays))';
        Feeder_Condition = categorical(fdCtg([icrDat{:,5}]), fdCtg);
        Rotation_Direction = categorical(rdCtg([icrDat{:,6}]+2), [rdCtg(1),rdCtg(end)])';
        Reward_Ratio = num2str(num2str([icrDat{:,7}]'));
        Reward_Ratio = cellstr([Reward_Ratio,repmat(':1',length(Reward_Ratio),1)]);
        ctg = unique([Reward_Ratio;rrCtg]);
        Reward_Ratio = categorical(Reward_Ratio, ctg);
        Reward_Delay = icrDat(:,8);
        Reward_Delay = cellfun(@(x) sprintf('%1.1f', x), Reward_Delay, 'Uni', false);
        ctg = unique([Reward_Delay;rwdlCtg]);
        Reward_Delay = categorical(Reward_Delay, ctg);
        Cue_Condition = ccCtg([icrDat{:,9}]+1)';
        Cue_Condition = categorical(Cue_Condition, flipud(ccCtg))';
        Pulse_Duration = ([icrDat{:,10}])';
        Sound_Conditions = cell(size(icrDat,1),1);
        Air_Conditions = cell(size(icrDat,1),1);
        for i = 1:size(icrDat,1)
            if length(icrDat{i,11}) == 1
                sn = icrDat{i,11};
                Air_Conditions{i} = [false, false];
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
                Air_Conditions{i} = [logical(icrDat{i,11}(4)), false];
            end
        end
        Sound_Conditions = cell2mat(Sound_Conditions);
        Air_Conditions = cell2mat(Air_Conditions);
        Start_Quadrant = Start_Quadrant(1:size(icrDat,1));
        rp = rpCtg(reshape(SsDat{3}(:,icr_ind),9,[])');
        rp = rp(1:length(icrdays),:);
        rps = [icrDat{[icrDat{:,coloff+16}]>0,coloff+16}]';
        dtmat = cellfun(@(x) x(1:10), icrDat(icrdays,2), 'Uni', false);
        dtmat = datenum(dtmat,'yyyy-mm-dd');
        %dtr =  diff(find([icrDat{:,coloff+16}]>0 == 1))';
        dtr =  diff(dtmat);
        Rotation_Positions = repmat({categorical({'<undefined>'}, ...
            rpCtg)}, size(icrDat,1),1);
        Rotations_Per_Session = NaN(size(icrDat,1),1);
        Rotations_Per_Session(icrdays) = rps;
        Rewards_Per_Rotation = SS_Out_ICR.(allRatList_Str{z_rat}).Rewards_Per_Rotation;
        Days_Till_Rotation = NaN(size(icrDat,1),1);
        if length(icrdays) > 1
            Days_Till_Rotation(icrdays(1:end-1)) = dtr;
        end
        cnt = 0;
        for i = icrdays
            cnt = cnt + 1;
            Rotation_Positions{i} = categorical(rp(1,1:rps(cnt)),rpCtg);
        end
        Reversals = NaN(size(icrDat,1),1);
        if strcmp(icrHed{12}, 'Reversals')
            Reversals = zeros(size(icrDat,1),1);
            ind = find([icrDat{:,12}]>0, 1, 'first');
            Reversals(ind:end) = [icrDat{ind:end,12}];
        end
        Rewards_0_Deg = ([icrDat{:,coloff+12}])';
        Rewards_40_Deg = ([icrDat{:,coloff+13}])';
        Laps_0_Deg = ([icrDat{:,coloff+14}])';
        Laps_40_Deg = ([icrDat{:,coloff+15}])';
        Notes = cell(size(icrDat,1),1);
        
        % Fill in fields for SS_Out_ICR
        SS_Out_ICR.(allRatList_Str{z_rat}).Include = Include;
        SS_Out_ICR.(allRatList_Str{z_rat}).Date = Date;
        SS_Out_ICR.(allRatList_Str{z_rat}).Recording_Directory = Recording_Directory;
        SS_Out_ICR.(allRatList_Str{z_rat}).Start_Time = Start_Time;
        SS_Out_ICR.(allRatList_Str{z_rat}).Total_Time = Total_Time;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICRb_Track = ICRb_Track;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICRb_Forage = ICRb_Forage;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICRi_Track = ICRi_Track;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICRi_Forage = ICRi_Forage;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICR_Rotation = ICR_Rotation;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICRb_Track_Session = ICRb_Track_Session;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICRb_Forage_Session = ICRb_Forage_Session;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICRi_Track_Session = ICRi_Track_Session;
        SS_Out_ICR.(allRatList_Str{z_rat}).ICRi_Forage_Session = ICRi_Forage_Session;
        SS_Out_ICR.(allRatList_Str{z_rat}).Rotation_Session = Rotation_Session;
        SS_Out_ICR.(allRatList_Str{z_rat}).Feeder_Condition = Feeder_Condition;
        SS_Out_ICR.(allRatList_Str{z_rat}).Rotation_Direction = Rotation_Direction;
        SS_Out_ICR.(allRatList_Str{z_rat}).Reward_Ratio = Reward_Ratio;
        SS_Out_ICR.(allRatList_Str{z_rat}).Reward_Delay = Reward_Delay;
        SS_Out_ICR.(allRatList_Str{z_rat}).Cue_Condition = Cue_Condition;
        SS_Out_ICR.(allRatList_Str{z_rat}).Pulse_Duration = Pulse_Duration;
        SS_Out_ICR.(allRatList_Str{z_rat}).Sound_Conditions = Sound_Conditions;
        SS_Out_ICR.(allRatList_Str{z_rat}).Air_Conditions = Air_Conditions;
        SS_Out_ICR.(allRatList_Str{z_rat}).Start_Quadrant = Start_Quadrant;
        SS_Out_ICR.(allRatList_Str{z_rat}).Rotation_Positions = Rotation_Positions;
        SS_Out_ICR.(allRatList_Str{z_rat}).Rotations_Per_Session = Rotations_Per_Session;
        SS_Out_ICR.(allRatList_Str{z_rat}).Rewards_Per_Rotation = Rewards_Per_Rotation;
        SS_Out_ICR.(allRatList_Str{z_rat}).Days_Till_Rotation = Days_Till_Rotation;
        SS_Out_ICR.(allRatList_Str{z_rat}).Reversals = Reversals;
        SS_Out_ICR.(allRatList_Str{z_rat}).Rewards_0_Deg = Rewards_0_Deg;
        SS_Out_ICR.(allRatList_Str{z_rat}).Rewards_40_Deg = Rewards_40_Deg;
        SS_Out_ICR.(allRatList_Str{z_rat}).Laps_0_Deg = Laps_0_Deg;
        SS_Out_ICR.(allRatList_Str{z_rat}).Laps_40_Deg = Laps_40_Deg;
        SS_Out_ICR.(allRatList_Str{z_rat}).Notes = Notes;
        
    end
    
end

% Save out data
save(fullfile(OutDir,'SS_In_All'), 'SS_In_All')
save(fullfile(OutDir,'SS_Out_ICR'), 'SS_Out_ICR')

%% FIX INCORRECT Rotation_Positions in SS_Out_ICR

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
clear S;

rpCtg = categories(SS_In_All.Rotation_Positions{1,1}); % rotation position

ratList = fieldnames(SS_Out_ICR);

for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
    icrdays = find(SS_Out_ICR.(rat_field).Rotation_Session > 0 == 1);
    rotPos = SS_In_All{rat_field, {'Rotation_Positions'}};
    rps = SS_Out_ICR.(rat_field){icrdays, {'Rotations_Per_Session'}};
    cnt = 0;
    for i = 1:length(icrdays)
        ind = icrdays(i);
        cnt = cnt+1;
        SS_Out_ICR.(rat_field).Rotation_Positions{ind,1} = ...
            rotPos{:}(cnt,1:rps(cnt));
    end
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')

%% FIX INCORRECT Age_Group SS_All_In

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
clear S;

oldInd = SS_In_All.Age_Group;
SS_In_All.Age_Group = renamecats(SS_In_All.Age_Group, ...
    categories(SS_In_All.Age_Group), ...
    {'Young', 'Old'}); 

save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')

%% Set sessions in SS_Out_ICR with laps < 5 to not inclue 

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S;

ratList = fieldnames(SS_Out_ICR);

for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
    laps = SS_Out_ICR.(rat_field).Laps_0_Deg + ...
        SS_Out_ICR.(rat_field).Laps_40_Deg;
    sesExc = laps < 5;
    % Set Include to false
    SS_Out_ICR.(rat_field){sesExc, {'Include'}} = false;
    % Add note
    SS_Out_ICR.(rat_field){sesExc, {'Notes'}} = {'Laps < 5'};
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')

%% Rename Standard and Rotated var names in SS_Out_ICR 

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S;

ratList = fieldnames(SS_Out_ICR);

for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
    % Set Include to false
    SS_Out_ICR.(rat_field).Properties.VariableNames{'Standard_Rewards'} = 'Rewards_0_Deg';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'Rotated_Rewards'} = 'Rewards_40_Deg';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'Standard_Laps'} = 'Laps_0_Deg';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'Rotated_Laps'} = 'Laps_40_Deg';
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')

%% Add standard lap count and compute for rotated sessionsSS_Out_ICR 

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];
PAR.Dir.dat = 'D:\BehaviorPilot';

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S;

ratList = fieldnames(SS_Out_ICR);

for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
%     % Set Include to false
%     SS_Out_ICR.(rat_field).Properties.VariableNames{'Rewards_0_Deg'} = 'Rewards_Standard';
%     SS_Out_ICR.(rat_field).Properties.VariableNames{'Laps_0_Deg'} = 'Laps_Standard';
%     SS_Out_ICR.(rat_field).Rewards_0_Deg = zeros(size(SS_Out_ICR.(rat_field),1),1);
%     SS_Out_ICR.(rat_field).Laps_0_Deg = zeros(size(SS_Out_ICR.(rat_field),1),1);
%     % Make all cells
%     SS_Out_ICR.(rat_field).Rewards_Standard = num2cell(SS_Out_ICR.(rat_field).Rewards_Standard);
%     SS_Out_ICR.(rat_field).Rewards_40_Deg = num2cell(SS_Out_ICR.(rat_field).Rewards_40_Deg);
%     SS_Out_ICR.(rat_field).Rewards_0_Deg = num2cell(SS_Out_ICR.(rat_field).Rewards_0_Deg);
%     SS_Out_ICR.(rat_field).Laps_Standard = num2cell(SS_Out_ICR.(rat_field).Laps_Standard);
%     SS_Out_ICR.(rat_field).Laps_40_Deg = num2cell(SS_Out_ICR.(rat_field).Laps_40_Deg);
%     SS_Out_ICR.(rat_field).Laps_0_Deg = num2cell(SS_Out_ICR.(rat_field).Laps_0_Deg);
%     % reorder
%     SS_Out_ICR.(rat_field) = [SS_Out_ICR.(rat_field)(:,1:31), ...
%         SS_Out_ICR.(rat_field)(:,35), ...
%         SS_Out_ICR.(rat_field)(:,32:33), ...
%         SS_Out_ICR.(rat_field)(:,36), ...
%         SS_Out_ICR.(rat_field)(:,34) ...
%         ];
    rotSes = find(SS_Out_ICR.(rat_field).Rotation_Session > 0);
    for z_ses = 1:length(rotSes)
        sesInd = rotSes(z_ses);
        sesFi = SS_Out_ICR.(rat_field).Date{sesInd};
        load(fullfile(PAR.Dir.dat, rat_field(2:end), sesFi, 'SS.mat'));
        
        imgList = [EV.Pro(EV.Pro.String == 'Lap', {'TS'}), ...
            EV.Pro(EV.Pro.String == 'Lap', {'Lap'}), ...
            EV.Pro(EV.Pro.String == 'Lap', {'Image'})];
        rewList = [EV.Pro(EV.Pro.String == 'Feeder_1_Open' | EV.Pro.String == 'Feeder_2_Open', {'TS'}), ...
            EV.Pro(EV.Pro.String == 'Feeder_1_Open' | EV.Pro.String == 'Feeder_2_Open', {'Lap'}), ...
            EV.Pro(EV.Pro.String == 'Feeder_1_Open' | EV.Pro.String == 'Feeder_2_Open', {'Image'})];
        
        icrTS = sort([EV.Pro.TS(EV.Pro.String == '0-Deg'); ...
            EV.Pro.TS(EV.Pro.String == '40-Deg')]);
        icrTS = [icrTS, [icrTS(2:end);EV.Pro.TS(end)]];
        
        % Laps
        SS_Out_ICR.(rat_field).Laps_Standard{sesInd} = sum(imgList.Image == 'Standard');
        % lap 40
        cnt = arrayfun(@(x,y) ...
            (imgList.TS(imgList.Image=='40-Deg') >= x & ...
            imgList.TS(imgList.Image=='40-Deg') < y), ...
            icrTS(:,1), icrTS(:,2), 'Uni', false);
        cnt = cell2mat(cnt');
        SS_Out_ICR.(rat_field).Laps_40_Deg{sesInd} = sum(cnt(:,1:2:end),1);
        % lap 0
        cnt = arrayfun(@(x,y) ...
            (imgList.TS(imgList.Image=='0-Deg') >= x & ...
            imgList.TS(imgList.Image=='0-Deg') < y), ...
            icrTS(:,1), icrTS(:,2), 'Uni', false);
        cnt = cell2mat(cnt');
        SS_Out_ICR.(rat_field).Laps_0_Deg{sesInd} = sum(cnt(:,2:2:end),1);
        
        % Rewards
        SS_Out_ICR.(rat_field).Rewards_Standard{sesInd} = sum(rewList.Image == 'Standard');
        % rew 40
        cnt = arrayfun(@(x,y) ...
            (rewList.TS(rewList.Image=='40-Deg') >= x & ...
            rewList.TS(rewList.Image=='40-Deg') < y), ...
            icrTS(:,1), icrTS(:,2), 'Uni', false);
        cnt = cell2mat(cnt');
        SS_Out_ICR.(rat_field).Rewards_40_Deg{sesInd} = sum(cnt(:,1:2:end),1);
        % rew 0
        cnt = arrayfun(@(x,y) ...
            (rewList.TS(rewList.Image=='0-Deg') >= x & ...
            rewList.TS(rewList.Image=='0-Deg') < y), ...
            icrTS(:,1), icrTS(:,2), 'Uni', false);
        cnt = cell2mat(cnt');
        SS_Out_ICR.(rat_field).Rewards_0_Deg{sesInd} = sum(cnt(:,2:2:end),1);
        
        % rewards per rotation
        SS_Out_ICR.(rat_field).Rewards_Per_Rotation{sesInd} = ...
            [SS_Out_ICR.(rat_field).Rewards_Standard{sesInd}, ...
            reshape([SS_Out_ICR.(rat_field).Rewards_40_Deg{sesInd};
            SS_Out_ICR.(rat_field).Rewards_0_Deg{sesInd}], 1, [])];
            
    end
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')

%% Change Yoke_Mate in SS_All_In

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
clear S;

SS_In_All.Yoke_Mate = addcats(SS_In_All.Yoke_Mate, 'None');
ratList = SS_In_All.Properties.RowNames;
for z_rat = 1:length(ratList)
    if SS_In_All.Yoke_Mate(z_rat) == ratList{z_rat}
        SS_In_All.Yoke_Mate(z_rat) = 'None';
    end
end

save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')

%% Copy over rotation parameters in SS_All_In for r9830 and r9857

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
clear S;

SS_In_All{'r9830','Rotations_Per_Session'}{:}(4:end) = ...
    SS_In_All{'r9989','Rotations_Per_Session'}{:}(4:end);
SS_In_All{'r9830','Rewards_Per_Rotation'}{:}(4:end) = ...
    SS_In_All{'r9989','Rewards_Per_Rotation'}{:}(4:end);
SS_In_All{'r9830','Days_Till_Rotation'}{:}(4:end) = ...
    SS_In_All{'r9989','Days_Till_Rotation'}{:}(4:end);

SS_In_All{'r9857','Rotations_Per_Session'}{:}(7:end) = ...
    SS_In_All{'r9990','Rotations_Per_Session'}{:}(7:end);
SS_In_All{'r9857','Rewards_Per_Rotation'}{:}(7:end) = ...
    SS_In_All{'r9990','Rewards_Per_Rotation'}{:}(7:end);
SS_In_All{'r9857','Days_Till_Rotation'}{:}(7:end) = ...
    SS_In_All{'r9990','Days_Till_Rotation'}{:}(7:end);

save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')

%% Change total time units and remove recording dir and reward ratio in SS_Out_ICR 

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S;

ratList = fieldnames(SS_Out_ICR);

% remove reward ratio
SS_In_All = ...
        [SS_In_All(:,1:19), ...
        SS_In_All(:,21:end)];

for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
    % change units
    SS_Out_ICR.(rat_field).Properties.VariableUnits{'Total_Time'} = 'min';
    % remove reward ratio and recording dir
    SS_Out_ICR.(rat_field) = ...
        [SS_Out_ICR.(rat_field)(:,1:2), ...
        SS_Out_ICR.(rat_field)(:,4:17), ...
        SS_Out_ICR.(rat_field)(:,19:end)];
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')
save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')

%% Change various variable names in all tables and set session = 0 to NaN
PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
S = load(fullfile(PAR.Dir.io, 'SS_Out_CT.mat'));
SS_Out_CT = S.SS_Out_CT;
clear S;

% Change SS_In_All
SS_In_All.Properties.VariableNames{'CT_Track_Session'} = 'Session_CT_T';
SS_In_All.Properties.VariableNames{'CT_Forage_Session'} = 'Session_CT_F';
SS_In_All.Properties.VariableNames{'ICRb_Track_Session'} = 'Session_ICRb_T';
SS_In_All.Properties.VariableNames{'ICRb_Forage_Session'} = 'Session_ICRb_F';
SS_In_All.Properties.VariableNames{'ICRi_Track_Session'} = 'Session_ICRi_T';
SS_In_All.Properties.VariableNames{'ICRi_Forage_Session'} = 'Session_ICRi_F';
SS_In_All.Properties.VariableNames{'Rotation_Session'} = 'Session_Rotation';
SS_In_All.Properties.VariableNames{'CT_Finished'} = 'Finished_CT';
SS_In_All.Properties.VariableNames{'ICRb_Finished'} = 'Finished_ICRb';
SS_In_All.Properties.VariableNames{'ICRi_Finished'} = 'Finished_ICRi';

% Change SS_Out_ICR
ratList = fieldnames(SS_Out_ICR);
for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRb_Track'} = 'ICRb';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRb_Forage'} = 'ICRi';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRi_Track'} = 'Track';
    SS_Out_ICR.(rat_field).Track = SS_Out_ICR.(rat_field).ICRb;
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRi_Forage'} = 'Forage';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICR_Rotation'} = 'Rotation';
    
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRb_Track_Session'} = 'Session_ICRb_T';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRb_Forage_Session'} = 'Session_ICRb_F';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRi_Track_Session'} = 'Session_ICRi_T';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'ICRi_Forage_Session'} = 'Session_ICRi_F';
    SS_Out_ICR.(rat_field).Properties.VariableNames{'Rotation_Session'} = 'Session_Rotation';
    
    % Change 0 to NaN
    SS_Out_ICR.(rat_field).Session_ICRb_F(SS_Out_ICR.(rat_field).Session_ICRb_F == 0) = NaN;
    SS_Out_ICR.(rat_field).Session_ICRi_T(SS_Out_ICR.(rat_field).Session_ICRi_T == 0) = NaN;
    SS_Out_ICR.(rat_field).Session_ICRi_F(SS_Out_ICR.(rat_field).Session_ICRi_F == 0) = NaN;
    SS_Out_ICR.(rat_field).Session_Rotation(SS_Out_ICR.(rat_field).Session_Rotation == 0) = NaN;
end

% Change SS_Out_CT
ratList = fieldnames(SS_Out_CT);
for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
    SS_Out_CT.(rat_field).Properties.VariableNames{'CT_Track'} = 'Track';
    SS_Out_CT.(rat_field).Properties.VariableNames{'CT_Forage'} = 'Forage';
    
    SS_Out_CT.(rat_field).Properties.VariableNames{'CT_Track_Session'} = 'Session_CT_T';
    SS_Out_CT.(rat_field).Properties.VariableNames{'CT_Forage_Session'} = 'Session_CT_F';
    % Change 0 to NaN
    SS_Out_CT.(rat_field).Session_CT_F(SS_Out_CT.(rat_field).Session_CT_F == 0) = NaN;
end

save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')
save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')
save(fullfile(PAR.Dir.io, 'SS_Out_CT.mat'), 'SS_Out_CT')


%% Add vacuum entry

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S;

ratList = fieldnames(SS_Out_ICR);

% add vac cond
SS_In_All.Air_Conditions = ...
        [SS_In_All.Air_Conditions(:,1), ...
        SS_In_All.Air_Conditions(:,2), ...
        SS_In_All.Air_Conditions(:,2)];
    % change units
SS_In_All.Properties.VariableDescriptions{'Air_Conditions'} = '[Pump, Vaccum, Ozone]';
    
for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
     
    % add vac cond
    SS_Out_ICR.(rat_field).Air_Conditions = ...
        [SS_Out_ICR.(rat_field).Air_Conditions(:,1), ...
        SS_Out_ICR.(rat_field).Air_Conditions(:,2), ...
        SS_Out_ICR.(rat_field).Air_Conditions(:,2)];
    
     % change units
    SS_Out_ICR.(rat_field).Properties.VariableDescriptions{'Air_Conditions'} = '[Pump, Vaccum, Ozone]';
 
    
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')
save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')

%% Change days till rotation

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S;

ratList = fieldnames(SS_Out_ICR);
    
for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};
     
    % add vac cond
    temp = categorical(SS_Out_ICR.(rat_field).Days_Till_Rotation);
    temp = addcats(temp,{'1:2', '2:3', '3:4'});
    SS_Out_ICR.(rat_field).Days_Till_Rotation = temp;
 
    
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')

%% Change include entries

PAR.Dir.io = [pwd, '\IOfiles\SessionData'];

% Import session data
S = load(fullfile(PAR.Dir.io, 'SS_In_All.mat'));
SS_In_All = S.SS_In_All;
S = load(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'));
SS_Out_ICR = S.SS_Out_ICR;
clear S;
S = load(fullfile(PAR.Dir.io, 'SS_Out_CT.mat'));
SS_Out_CT = S.SS_Out_CT;
clear S;

SS_In_All.Properties.VariableNames{'Include'} = 'Include_Run';
SS_In_All = [SS_In_All(:,1), ...
    table(true(size(SS_In_All,1),1), 'VariableNames', {'Include_Analysis'}), ...
    SS_In_All(:,2:end)];

ratList = fieldnames(SS_Out_ICR);

for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};

    SS_Out_ICR.(rat_field).Properties.VariableNames{'Include'} = 'Include_Analysis';
    
end

ratList = fieldnames(SS_Out_CT);

for z_rat = 1:length(ratList)
    rat_field = ratList{z_rat};

    SS_Out_CT.(rat_field).Properties.VariableNames{'Include'} = 'Include_Analysis';
    
end

save(fullfile(PAR.Dir.io, 'SS_Out_ICR.mat'), 'SS_Out_ICR')
save(fullfile(PAR.Dir.io, 'SS_In_All.mat'), 'SS_In_All')
save(fullfile(PAR.Dir.io, 'SS_Out_CT.mat'), 'SS_Out_CT')

