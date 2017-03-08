function[] = TT_TurnSetup()

% Directories
outdir = ...
    'C:\Users\lester\Documents\Research\BarnesLab\Study_ICR\MATLAB\IOfiles\SessionData';

% heading
head = [{'Session'}, {'Date'}, {'TT'}, {'Orientation'}, {'Turns'}, {'Depth'}, {'Notes'}];

% Rat numbers
rat = [{'9827'}; {'9859'}; {'9842'}];

% Tetrode setup for two bundles
% Relative tetrode locations
% rows go from L-M from top-bottom and A-P from left-right
% Note: 17 is R1; 18 is R2
ntt = [9,8];
% hipp bundle
ttpos{1} = [[{'09'}, {'10'}, {'11'}]', [{'12'}, {'13'}, {'14'}]', [{'15'}, {'16'}, {'R1'}]'];
% mec bundle
ttpos{2} = [[{'01'}, {'02'}]', [{'03'}, {'04'}]', [{'05'}, {'06'}]', [{'07'}, {'R2'}]'];

% session
ses = 0;

% date/time
date = datestr(clock, 'yyyy-mm-dd_HH-MM-SS', 'local');

% tt labels
ttlist{1} = sort(ttpos{1}(:)); 
ttlist{2} = sort(ttpos{2}(:));
lst = {[ttlist{1};ttlist{2}]};

% turns
trn = {zeros(sum(ntt), 1)};

% Starting screw orientation:
    % {'N'} {'NNE'} {'NE'} {'ENE'} 
    % {'E'} {'ESE'} {'SE'} {'SSE'} 
    % {'S'} {'SSW'} {'SW'} {'WSW'}
    % {'W'} {'WNW'} {'NW'} {'NNW'}
% will enter starting orientations from gui
stror = {repmat({''}, sum(ntt), 1)};

% turns
trn = {zeros(sum(ntt), 1)};

% depth
dpth = {zeros(sum(ntt), 1)};

% notes
nts = {repmat({''}, sum(ntt), 1)};

% Create first entry
x = [ses, date, lst, stror, trn, dpth, nts];

% Structure which will be saved
TTL.D = cell2struct(x', head);
TTL.P = ttpos;
TTL.L = ttlist;

for z_r = 1:length(rat)
    save(fullfile(outdir, [rat{z_r},'_ttl']), 'TTL')
end
