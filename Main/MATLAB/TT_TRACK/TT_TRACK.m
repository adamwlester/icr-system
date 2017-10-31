function TT_TRACK(D)

%% ======================== SET PARAMETERS ================================

% "TT" will be passed between all functions used
if nargin < 1
    D = struct;
    D.F.hide_tt_fig = false;
else
    D.F.hide_tt_fig = false;
end

% Debugging
D.DB.doAutoLoad = false;
D.DB.depthSet = 1; % (mm)

% Top directory
D.DIR.top = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running';

% IO dirs
D.DIR.ioTop = fullfile(D.DIR.top,'IOfiles');
D.DIR.log = fullfile(D.DIR.top,'IOfiles');
D.DIR.ioTT_Log = fullfile(D.DIR.ioTop, 'SessionData', 'TT_Log.mat');
% Image directory
D.DIR.img = fullfile(D.DIR.ioTop,'Images');
D.DIR.matFile = fullfile(D.DIR.img, 'Paxinos', 'procPax.mat');
D.DIR.pax{1} = fullfile(D.DIR.img, 'Paxinos', 'sag_lab');
D.DIR.pax{2} = fullfile(D.DIR.img, 'Paxinos', 'cor_lab');
D.DIR.pax{3} = fullfile(D.DIR.img, 'Paxinos', 'hor_lab');

% Number of bundles
D.PAR.nBundles = 2;

% Bundle angle (use negative for angled away from the nose)
D.UI.bndAng = [deg2rad(270), deg2rad(270+10)];

% Cannula spacing (mm)
D.PAR.canSp = 0.3175;

% TT diameter (mm)
D.PAR.ttDiam = 0.0254;

% Track lines offset (mm)
D.UI.linOfst = 0.025;

% Specify figure monitor
D.UI.figTTMon = 1;

% Set plot lims [A-P,M-L,D-V] (mm)
D.UI.mmPlotLims = [...
    0, -12; ...
    0, 8; ...
    0, -10];

% Paxinos image dims [y_mm, x_mm]
D.PAR.mmPaxSize{1} = [11,21];
D.PAR.mmPaxSize{2} = [11,8];
D.PAR.mmPaxSize{3} = [8,21];

% Paxinos image lims [x_min, x_max; y_min, y_max]
D.PAR.pxlPalLims{1} = [243, 3854; 420, 2311]; % sag
D.PAR.pxlPalLims{2} = [243, 2136; 247, 2842]; % cor
D.PAR.pxlPalLims{3} = [243, 3854; 250, 1631]; % hor

% Strange sized pax cor images
D.PAR.pxlPalLims{4} = [206, 2099; 210, 2812]; % cor dumb

% Image axis limits [A-P,M-L,D-V] (mm)
D.UI.mmPaxLims = [...
    6, -15; ...
    0, 8; ...
    0, -11];

% Paxinos view colors
D.UI.paxViewCol = [...
    255, 198, 198; ... % red
    198, 200, 255; ... % blue
    198, 255, 201] / 255; % green

% Active vs inactive color
D.UI.stateBtnCol = [...
    0.5, 0.5, 0.5; ...
    0.8, 0.8, 0.8];
D.UI.stateCol = [...
    0.5, 0.5, 0.5; ...
    0.8, 0.8, 0.8];

% Plot state settings
D.UI.ttLegMrkWdth = [1,4];
D.UI.ttFaceAlph = [0.05, 1];

%==========================================================================

%% ================== IMPORT/FORMAT PAXINOS IMAGES ========================

% Load session data
T = load(D.DIR.ioTT_Log);
D.TT_Log = T.TT_Log;
clear T;

% Get rat list
ind = D.TT_Log.Include_Run;
D.PAR.ratList = D.TT_Log.Properties.RowNames(ind);
D.PAR.ratList = regexprep(D.PAR.ratList, 'r', '');
D.PAR.ratList = [{'Select Rat'};D.PAR.ratList];

% Get image stuff
for z_view = 1:3
    
    % Get image list
    D.PAR.paxFiLabs{z_view} = dir(D.DIR.pax{z_view});
    D.PAR.paxFiLabs{z_view} = {D.PAR.paxFiLabs{z_view}(:).name};
    D.PAR.paxFiLabs{z_view} = ...
        D.PAR.paxFiLabs{z_view}(cell2mat(cellfun(@(x) ~isempty(x), strfind(D.PAR.paxFiLabs{z_view}, 'img'), 'uni', false)));
    
    % Setup image matrix [m, n, 3]
    D.UI.paxMat{z_view} = cell(1, length(D.PAR.paxFiLabs{z_view}));
    
    % Setup image handles
    D.UI.h_paxImg{z_view} = gobjects(1,length(D.PAR.paxFiLabs{z_view}));
    
    % Setup image coordinates
    D.TT.imgCoor{z_view} = NaN(1,length(D.PAR.paxFiLabs{z_view}));
end

% Check if we already have processed image data
if exist(D.DIR.matFile, 'file')
    load(D.DIR.matFile);
    D.UI.paxMat = IMG_MAT; %#ok<NODEF>
    D.TT.imgCoor = IMG_COORD; %#ok<NODEF>
    
else
    % Loop through paxinos directories
    for z_view = 1:3
        
        % Loop through each image
        for z_pax = 1:length(D.PAR.paxFiLabs{z_view})
            
            % Store
            D.UI.paxMat{z_view}{z_pax} = ...
                imread(fullfile(D.DIR.pax{z_view}, D.PAR.paxFiLabs{z_view}{z_pax}));
            
            % Handle stupid fucking pax bullshit
            if z_view == 2 && size(D.UI.paxMat{z_view}{z_pax},1) == 3300
                
                inc_x = D.PAR.pxlPalLims{4}(2,1):D.PAR.pxlPalLims{4}(2,2);
                inc_y = D.PAR.pxlPalLims{4}(1,1):D.PAR.pxlPalLims{4}(1,2);
                
            else
                inc_x = D.PAR.pxlPalLims{z_view}(2,1):D.PAR.pxlPalLims{z_view}(2,2);
                inc_y = D.PAR.pxlPalLims{z_view}(1,1):D.PAR.pxlPalLims{z_view}(1,2);
            end
            
            % Clip image
            D.UI.paxMat{z_view}{z_pax} = D.UI.paxMat{z_view}{z_pax}(inc_x, inc_y, :);
            
            % Get colored pixels
            color_ind = mean(diff(D.UI.paxMat{z_view}{z_pax},1,3),3) > 50;
            mono_ind = mean(D.UI.paxMat{z_view}{z_pax},3) < 250 & ~color_ind;
            
            % Create ind for each rgb colored pixels
            red_pxls = padarray(color_ind, [0,0,2], 'post');
            green_pxls = padarray(color_ind, [0,0,1], 'both');
            blue_pxls = padarray(color_ind, [0,0,2], 'pre');
            
            % Set monochrome cols to gray
            D.UI.paxMat{z_view}{z_pax}(repmat(mono_ind,[1,1,3])) = 127;
            
            % Set outline color
            D.UI.paxMat{z_view}{z_pax}(red_pxls) = D.UI.paxViewCol(z_view, 1)*255;
            D.UI.paxMat{z_view}{z_pax}(green_pxls) = D.UI.paxViewCol(z_view, 2)*255;
            D.UI.paxMat{z_view}{z_pax}(blue_pxls) = D.UI.paxViewCol(z_view, 3)*255;
            
            % Reseize image to 1pxl per 10 um
            D.UI.paxMat{z_view}{z_pax} = ...
                imresize(D.UI.paxMat{z_view}{z_pax}, D.PAR.mmPaxSize{z_view}*100);
            
            % Get coordinate info
            num_str = regexp(D.PAR.paxFiLabs{z_view}{z_pax}, 'lab_(-?\d?\d.\d\d).png', 'tokens');
            D.TT.imgCoor{z_view}(z_pax) = str2double(num_str{:});
            
            % Flip images
            D.UI.paxMat{z_view}{z_pax} = flip(D.UI.paxMat{z_view}{z_pax},1);
            D.UI.paxMat{z_view}{z_pax} = flip(D.UI.paxMat{z_view}{z_pax},2);
            
        end
        
        % Sort
        [D.TT.imgCoor{z_view}, sort_ind] = sort(D.TT.imgCoor{z_view});
        D.UI.paxMat{z_view} = D.UI.paxMat{z_view}(sort_ind);
    end
    
    % Save coordinate and image matrix
    IMG_MAT = D.UI.paxMat; %#ok<NASGU>
    IMG_COORD = D.TT.imgCoor; %#ok<NASGU>
    save(D.DIR.matFile,  'IMG_MAT', 'IMG_COORD');
    
end

%==========================================================================

%% ======================== SETUP UI FIGURE ===============================

% Border offset for all GUI features
D.UI.bordOff = 40; % pixels

% Monitor positions
sc = get(0,'MonitorPositions');
monPos(1,:) = sc(sc(:,1) == min(sc(:,1)),:);
monPos(2,:) = sc(sc(:,1) == max(sc(:,1)),:);

% Figure position
fig_wh = [1250, 1000];
fig_lft = monPos(D.UI.figTTMon,1) + (monPos(D.UI.figTTMon,3)-fig_wh(1))/2; 
fig_btm = monPos(D.UI.figTTMon,2) + (monPos(D.UI.figTTMon,4)-fig_wh(2))/2;
D.UI.figTTPos = [fig_lft, fig_btm, fig_wh(1), fig_wh(2)];

% Dialogue box
D.UI.dlgPos = [monPos(D.UI.figTTMon,1) + monPos(D.UI.figTTMon,3)/2, monPos(D.UI.figTTMon,4)/2];

% Main axis reserved space
D.UI.axeRng = ...
    [D.UI.bordOff, D.UI.bordOff, D.UI.figTTPos(4)-2*D.UI.bordOff, D.UI.figTTPos(4)-2*D.UI.bordOff];

% Main axis actual pos
D.UI.axePos =  ...
    [D.UI.axeRng(1)+D.UI.bordOff, D.UI.axeRng(2)+D.UI.bordOff,  D.UI.axeRng(3)-D.UI.bordOff, D.UI.axeRng(4)-D.UI.bordOff];

% Main figure
D.UI.figTT = figure(...
    'Units','Pixels',...
    'Position',D.UI.figTTPos,...
    'Color', [1 1 1],...
    'Name','TT Track',...
    'Tag','figure1',...
    'MenuBar', 'none',... % hide menu
    'Visible','off'); % hide figure durring creation

% TEMP
movegui('west')

% Label axes lims
lm_str = {...
    '|P|', '|A|'; ...
    '|M|', '|L|'; ...
    '|D|', '|V|'};
lm = NaN(3,2);
tk = cell(3,1);
tl = cell(3,1);

% Get axis stuff
for z_view = 1:3
    
    % Get axis limits
    lm(z_view,:) = [min(D.UI.mmPaxLims(z_view,:)), max(D.UI.mmPaxLims(z_view,:))]*100;
    
    % Get axis ticks
    tk{z_view} = linspace(lm(z_view,1),lm(z_view,2),sum(abs(D.UI.mmPaxLims(z_view,:)))+1)';
    
    % Make axis tick labels
    tl{z_view} = num2str(tk{z_view}/100);
    tl{z_view}  = [repmat(blanks(3-size(tl{z_view} ,2)), size(tl{z_view} ,1), 1), tl{z_view}];
    
    % Label axes lims
    tl{z_view}(tk{z_view}/100==min(D.UI.mmPlotLims(z_view,:)),:) = lm_str{z_view,1};
    tl{z_view}(tk{z_view}/100==max(D.UI.mmPlotLims(z_view,:)),:) = lm_str{z_view,2};
end

% Create axis
D.UI.axe3D = axes(...
    'Parent', D.UI.figTT,...
    'Units','Pixels',...
    'Position',D.UI.axePos,...
    'XDir', 'Reverse', ...
    'XLim', lm(1,:), ...
    'YLim', lm(2,:), ...
    'ZLim', lm(3,:), ...
    'XTick', tk{1}, ...
    'YTick', tk{2}, ...
    'ZTick', tk{3}, ...
    'XTickLabel', tl{1}, ...
    'YTickLabel', tl{2}, ...
    'ZTickLabel', tl{3}, ...
    'Color', 'None', ...
    'Visible', 'on');
hold on
box on

% Specify rotation mat
img_rot = [ ...
    deg2rad(90), 0, 0; ...
    deg2rad(90), deg2rad(90), 0;
    0, 0, 0
    ];

% Specify direction of translation
img_trans_x = [D.UI.mmPaxLims(1,2), 0, D.UI.mmPaxLims(1,2)] * 100;
img_trans_y = [D.UI.mmPaxLims(3,2), D.UI.mmPaxLims(3,2), 0] * 100;
img_trans_z = [-1, 1, 1];

% Show each image on correct plane
for z_view = 1:3
    for z_img = 1:length(D.PAR.paxFiLabs{z_view})
        
        % Get transform
        zt = D.TT.imgCoor{z_view}(z_img) * 100 * img_trans_z(z_view);
        p = hgtransform('Parent',D.UI.axe3D);
        rt = makehgtform(...
            'xrotate', img_rot(z_view,1), ...
            'yrotate', img_rot(z_view,2), ...
            'zrotate', img_rot(z_view,3), ...
            'translate',[img_trans_x(z_view), img_trans_y(z_view), zt]);
        h = hgtransform('Matrix',rt);
        set(h,'Parent',p)
        
        % Show image
        D.UI.h_paxImg{z_view}(z_img) =  ...
            image(D.UI.paxMat{z_view}{z_img}, ...
            'Parent', h, ...
            'Visible', 'off');
        
        % Set alpha
        D.UI.h_paxImg{z_view}(z_img).AlphaData = 0.75;
    end
end

% Set final plot lims
axis equal
set(D.UI.axe3D, ...
    'XLim',  100*[min(D.UI.mmPlotLims(1,:)), max(D.UI.mmPlotLims(1,:))], ...
    'YLim',  100*[min(D.UI.mmPlotLims(2,:)), max(D.UI.mmPlotLims(2,:))], ...
    'ZLim',  100*[min(D.UI.mmPlotLims(3,:)), max(D.UI.mmPlotLims(3,:))])

% Remove unused images
exc_sag = D.TT.imgCoor{1}<min(D.UI.mmPlotLims(2,:)) | ...
    D.TT.imgCoor{1}>max(D.UI.mmPlotLims(2,:));
exc_cor = D.TT.imgCoor{2}<min(D.UI.mmPlotLims(1,:)) | ...
    D.TT.imgCoor{2}>max(D.UI.mmPlotLims(1,:));
exc_hor = D.TT.imgCoor{3}<min(D.UI.mmPlotLims(3,:)) | ...
    D.TT.imgCoor{3}>max(D.UI.mmPlotLims(3,:));
D.TT.imgCoor{1}(exc_sag) = [];
D.TT.imgCoor{2}(exc_cor) = [];
D.TT.imgCoor{3}(exc_hor) = [];
D.UI.h_paxImg{1}(exc_sag) = [];
D.UI.h_paxImg{2}(exc_cor) = [];
D.UI.h_paxImg{3}(exc_hor) = [];

light_pos = [max(D.UI.axe3D.XLim), max(D.UI.axe3D.YLim), max(D.UI.axe3D.ZLim)];
D.UI.h_light(1) = light(D.UI.axe3D, ...
    'Style', 'local', ...
    'Position', light_pos);
light_pos = [min(D.UI.axe3D.XLim), min(D.UI.axe3D.YLim), min(D.UI.axe3D.ZLim)];
D.UI.h_light(2) = light(D.UI.axe3D, ...
    'Style', 'local', ...
    'Position', light_pos);

% Legend pos
wdht = 125; % width/hight of axis
lft = sum(D.UI.axePos([1,3])) - wdht*2 - D.UI.bordOff;
btom = D.UI.axePos(2)+D.UI.bordOff/2;
D.UI.leg_1_pos = [lft , btom, wdht, wdht];
D.UI.leg_2_pos = [sum(D.UI.leg_1_pos([1,3]))+D.UI.bordOff/2, D.UI.leg_1_pos(2), D.UI.leg_1_pos(3), D.UI.leg_1_pos(4)];

% Create Hipp legend axis
D.UI.axLeg(1) = axes(...
    'Parent', D.UI.figTT,...
    'Units','pixels',...
    'Position', D.UI.leg_1_pos,...
    'Color', [1 1 1],...
    'XTick', [], ...
    'YTick', [], ...
    'Visible', 'on');
box on
hold on

% Create MEX legend axis
D.UI.axLeg(2) = axes(...
    'Parent', D.UI.figTT,...
    'Units','pixels',...
    'Position', D.UI.leg_2_pos,...
    'Color', [1 1 1],...
    'XTick', [], ...
    'YTick', [], ...
    'Visible', 'on');
box on
hold on

% Create a backround axis
D.UI.axLegBack = copyobj(D.UI.axLeg(1),D.UI.figTT);
D.UI.axLegBack.Position = ...
    [D.UI.leg_1_pos(1)-D.UI.bordOff/2, D.UI.axePos(2), wdht*2 + D.UI.bordOff*1.5, wdht + D.UI.bordOff*1.5];

% Set titles
set(D.UI.axLeg(1), 'Title', ...
    text('String','    Hipp Bundle','FontSize',12,'FontWeight','bold','Color','k'))
set(D.UI.axLeg(2), 'Title', ...
    text('String','    MEC Bundle','FontSize',12,'FontWeight','bold','Color','k'))

% Orientation strings
D.PAR.dirVec = [{'N'},{'NNE'},{'NE'},{'ENE'},{'E'},{'ESE'},{'SE'},{'SSE'},{'S'},...
    {'SSW'},{'SW'},{'WSW'},{'W'},{'WNW'},{'NW'},{'NNW'}];

% Move legend to top
uistack(D.UI.axLeg,'top')

%==========================================================================

%% ======================= SETUP UI OBJECTS ===============================

% ------------------------------ DEFAULTS ---------------------------------

% default left pos
D.UI.dfltLft = sum(D.UI.axeRng([1,3]))+D.UI.bordOff;
% default width pos
D.UI.dfltWd = D.UI.figTTPos(3)-D.UI.dfltLft-D.UI.bordOff;
% default width pos
D.UI.dfltHt = 40;
% default object spacing
D.UI.dfltSp = 20;

% Fonts
D.UI.btnFont = 'MS Sans Serif';
D.UI.popFont = 'MS Sans Serif';
D.UI.txtFont = 'Monospaced'; %'Courier New';

% ------------------------ PAXINOS VIEW CONTROLS --------------------------

% Add Yaw (z axis) slider along bottom
D.UI.posSldYaw = ...
    [D.UI.axePos(1), D.UI.bordOff*0.25, D.UI.axePos(3), D.UI.bordOff*0.7];
D.UI.sldYaw = uicontrol('Style', 'slider',...
    'Parent',D.UI.figTT, ...
    'Units','Pixels', ...
    'Callback', {@Set3dView},...
    'UserData', 0, ...
    'Min',-90,'Max',90, ...
    'Value',0,...
    'SliderStep', [1/180,45/180], ...
    'Visible', 'on',...
    'Enable', 'on',...
    'Position',  D.UI.posSldYaw);

% Add Pitch (x axis) slider along left
D.UI.posSldPitch = ...
    [D.UI.bordOff*0.25, D.UI.axePos(2), D.UI.bordOff*0.7, D.UI.axePos(4)];
D.UI.sldPitch = uicontrol('Style', 'slider',...
    'Parent',D.UI.figTT, ...
    'Units','Pixels', ...
    'Callback', {@Set3dView},...
    'UserData', 0, ...
    'Min',-90,'Max',90, ...
    'Value',0,...
    'SliderStep', [1/180,45/180], ...
    'Visible', 'on',...
    'Enable', 'on',...
    'Position',  D.UI.posSldPitch);

% Add setview button
D.UI.posBtnSetView = [D.UI.bordOff*0.25, D.UI.bordOff*0.25, D.UI.bordOff*0.7, D.UI.bordOff*0.7];
D.UI.btnSetView(1) = uicontrol('style','toggle', ...
    'Parent', D.UI.figTT, ...
    'Enable', 'On', ...
    'UserData', 1, ...
    'Units','Pixels', ...
    'FontName',D.UI.btnFont,...
    'ForegroundColor', [0.1,0.1,0.1], ...
    'FontWeight','Bold',...
    'FontSize',14);

% Create additional set view buttons
D.UI.btnSetView(2) = copyobj(D.UI.btnSetView(1),D.UI.figTT);
D.UI.btnSetView(3) = copyobj(D.UI.btnSetView(1),D.UI.figTT);
D.UI.btnSetView(4) = copyobj(D.UI.btnSetView(1),D.UI.figTT);

% Change string and user data
set(D.UI.btnSetView(1), 'String', 'S', 'UserData', 1, 'BackgroundColor', D.UI.paxViewCol(1,:));
set(D.UI.btnSetView(2), 'String', 'C', 'UserData', 2, 'BackgroundColor', D.UI.paxViewCol(2,:));
set(D.UI.btnSetView(3), 'String', 'H', 'UserData', 3, 'BackgroundColor', D.UI.paxViewCol(3,:));
set(D.UI.btnSetView(4), 'String', 'O', 'UserData', 4, 'BackgroundColor', [0.8,0.8,0.8]);

% Change pos
set(D.UI.btnSetView, 'Position', D.UI.posBtnSetView);
D.UI.btnSetView(1).Position(1) = D.UI.posBtnSetView(1)+D.UI.posBtnSetView(3);
D.UI.btnSetView(3).Position(2) = D.UI.posBtnSetView(2)+D.UI.posBtnSetView(4);
D.UI.btnSetView(4).Position(1:2) = [D.UI.btnSetView(1).Position(1), D.UI.btnSetView(3).Position(2)];

% Set callback
set(D.UI.btnSetView, 'Callback', {@Set3dView});

% Set view
Set3dView(D.UI.btnSetView(4));

% ----------------------- PAXINOS IMAGE CONTROLS --------------------------

% Switch image button
swtch_txt_wdth = 75;
swtch_sld_wdth = 200;
swtch_ht = 25;
swtch_lft = D.UI.axePos(1) + D.UI.bordOff/2;
swtch_btm = D.UI.axePos(2)+D.UI.bordOff/2 + 25*2 + swtch_ht;

% Switch image text
pos = [swtch_lft , swtch_btm, swtch_txt_wdth, swtch_ht];
D.UI.txtImgCorr(1) = uicontrol('Style','text', ...
    'Parent',D.UI.figTT, ...
    'String',' ', ...
    'Units','Pixels', ...
    'Position', pos, ...
    'HorizontalAlignment', 'Left',...
    'ForegroundColor', [0.1,0.1,0.1], ...
    'FontName',D.UI.txtFont,...
    'FontWeight','Bold',...
    'FontSize', 14);

% Switch image slider
pos = [pos(1)+pos(3), pos(2), swtch_sld_wdth, swtch_ht];
D.UI.sldSwtchImg(1) = uicontrol('Style', 'slider',...
    'Parent',D.UI.figTT, ...
    'Units','Pixels', ...
    'Callback', {@Set3dView},...
    'Min',1, ...
    'Visible', 'on',...
    'Enable', 'on',...
    'Position',  pos);

% Create slider and text copies
D.UI.txtImgCorr(2) = copyobj(D.UI.txtImgCorr(1),D.UI.figTT);
D.UI.txtImgCorr(3) = copyobj(D.UI.txtImgCorr(1),D.UI.figTT);
D.UI.sldSwtchImg(2) = copyobj(D.UI.sldSwtchImg(1),D.UI.figTT);
D.UI.sldSwtchImg(3) = copyobj(D.UI.sldSwtchImg(1),D.UI.figTT);

% Change slider and text positions
D.UI.txtImgCorr(2).Position(2) = swtch_btm - (swtch_ht + swtch_ht/2);
D.UI.sldSwtchImg(2).Position(2) = swtch_btm - (swtch_ht + swtch_ht/2);
D.UI.txtImgCorr(3).Position(2) = swtch_btm - (swtch_ht + swtch_ht/2)*2;
D.UI.sldSwtchImg(3).Position(2) = swtch_btm - (swtch_ht + swtch_ht/2)*2;

% Loop through each view
for z_v = 1:3
    
    % Change text backround color
    set(D.UI.txtImgCorr(z_v), 'BackgroundColor', D.UI.paxViewCol(z_v,:))
    
    % Set user data
    set(D.UI.sldSwtchImg(z_v), 'UserData', z_v);
    
    % Set slider steps
    n_img = length(D.TT.imgCoor{z_v});
    set(D.UI.sldSwtchImg(z_v), ...
        'Max', n_img, ...
        'SliderStep', [1/n_img, 5*(1/n_img)], ...
        'Value', floor(n_img/2));
end

% Set callback
set(D.UI.sldSwtchImg, 'Callback', {@SldSwtchImg});

% Set to first image
SldSwtchImg(D.UI.sldSwtchImg(1));
SldSwtchImg(D.UI.sldSwtchImg(2));
SldSwtchImg(D.UI.sldSwtchImg(3));

% --------------------- SETUP, SAVE, QUIT CONTROLS ------------------------

% Load popup
D.UI.posPopLoad = [D.UI.dfltLft, D.UI.figTTPos(4)-D.UI.bordOff-D.UI.dfltHt, D.UI.dfltWd, D.UI.dfltHt];
D.UI.popLoadTT = uicontrol('Style','popupmenu', ...
    'Parent', D.UI.figTT, ...
    'Enable', 'On', ...
    'String',D.PAR.ratList,...
    'Callback', {@PopLoadRatTTInf},...
    'FontName',D.UI.popFont,...
    'UserData', 1, ...
    'Units','Pixels', ...
    'Position', D.UI.posPopLoad, ...
    'FontWeight','Bold',...
    'FontSize',14);

% Save button
D.UI.posBtnSave = [D.UI.dfltLft, D.UI.bordOff, D.UI.dfltWd/2-5, D.UI.dfltHt];
D.UI.btnSaveAll = uicontrol('style','push', ...
    'Parent', D.UI.figTT, ...
    'Enable', 'On', ...
    'String','SAVE',...
    'Callback', {@BtnSaveAll},...
    'UserData', 0, ...
    'Units','Pixels', ...
    'Position', D.UI.posBtnSave, ...
    'FontName',D.UI.btnFont,...
    'FontWeight','Bold',...
    'FontSize',14);

% Quit button
D.UI.posBtnQuit = [D.UI.dfltLft+D.UI.dfltWd/2+5, D.UI.bordOff, D.UI.dfltWd/2-5, D.UI.dfltHt];
D.UI.btnQuit = uicontrol('style','push', ...
    'Parent', D.UI.figTT, ...
    'Enable', 'On', ...
    'String','QUIT',...
    'Callback', {@BtnQuit},...
    'UserData', 0, ...
    'Units','Pixels', ...
    'Position', D.UI.posBtnQuit, ...
    'FontName',D.UI.btnFont,...
    'FontWeight','Bold',...
    'FontSize',14);

% area for tt buttons
D.UI.ttBtnRng = (D.UI.figTTPos(4) - D.UI.posPopLoad(4) - D.UI.posBtnSave(4) - D.UI.dfltSp - 2*D.UI.bordOff) * 0.425;
% area for tt pannel
D.UI.ttPanRng = (D.UI.figTTPos(4) - D.UI.posPopLoad(4) - D.UI.posBtnSave(4) - D.UI.dfltSp - 2*D.UI.bordOff) * 0.575;
D.UI.ttPanPos = [D.UI.dfltLft, D.UI.bordOff+D.UI.posBtnSave(4)+D.UI.dfltSp, D.UI.dfltWd, D.UI.ttPanRng];

% ------------------------- TT SELECT CONTROLS ----------------------------

% Bundle heading
ttBtnHead = [{'Hipp'}, {'MEC'}];
% Panels left (pixel)
ttGrpLft = [D.UI.dfltLft, D.UI.dfltLft+1+D.UI.dfltWd/2];
% Panels bottom (pixel)
ttGrpBtm = D.UI.posPopLoad(2) - D.UI.ttBtnRng;

% Loop through each bundle
for z_bndl = 1:D.PAR.nBundles
    
    % Add bundle pannel
    panPos = [ttGrpLft(z_bndl), ttGrpBtm, D.UI.dfltWd/D.PAR.nBundles-1, D.UI.ttBtnRng];
    D.UI.panBtnLoadTT(z_bndl) = uipanel(...
        'Parent',D.UI.figTT,...
        'Units','Pixels',...
        'BorderType','line',...
        'BorderWidth',4,...
        'FontSize',15,...
        'FontWeight','bold',...
        'HighlightColor',D.UI.stateCol(2,:),...
        'BackgroundColor',[1,1,1],...
        'Title',ttBtnHead{z_bndl},...
        'TitlePosition','centertop',...
        'UserData',[],...
        'Clipping','on',...
        'Position',panPos);
end

% ------------------------ TT HIDE SHOW TOGGLE ----------------------------

% Get eye icon
wtht = D.UI.bordOff/2;
eye_icon = imread(fullfile(D.DIR.img, 'Icons', 'eyeicon.png'));
eye_icon = imresize(eye_icon, [wtht-5, wtht-5]);
D.UI.btnHideTT(1) = uicontrol('style','toggle', ...
    'Parent', D.UI.figTT, ...
    'Enable', 'Off', ...
    'Units','Pixels', ...
    'FontName',D.UI.btnFont,...
    'ForegroundColor', [1,1,1], ...
    'FontWeight','Bold',...
    'Enable', 'off', ...
    'Value', 1, ...
    'cdata', eye_icon, ...
    'FontSize',14);

% Copy button
D.UI.btnHideTT(2) = copyobj(D.UI.btnHideTT(1), D.UI.figTT);

% Set values
set(D.UI.btnHideTT(1), ...
    'Position', [D.UI.leg_1_pos(1), D.UI.leg_1_pos(2)+ D.UI.leg_1_pos(4), wtht, wtht], ...
    'UserData', 1);
set(D.UI.btnHideTT(2), ...
    'Position', [D.UI.leg_2_pos(1), D.UI.leg_2_pos(2)+D.UI.leg_2_pos(4), wtht, wtht], ...
    'UserData', 2);

% Set callback
set(D.UI.btnHideTT, 'Callback', {@BtnHideShow});

% --------------------------- TT TURN CONTROLS ----------------------------

% TT settings pannel
D.UI.panTT = uipanel(...
    'Parent',D.UI.figTT,...
    'Units','Pixels',...
    'BorderType','line',...
    'BorderWidth',4,...
    'FontSize',15,...
    'FontWeight','bold',...
    'HighlightColor',D.UI.stateCol(2,:),...
    'BackgroundColor',[1,1,1],...
    'Title','TTxx',...
    'TitlePosition','centertop',...
    'Clipping','on',...
    'Position',D.UI.ttPanPos);

% TT save state
D.UI.ttStateStr{1} = sprintf('%sPrior\nOrientation & Depth', blanks(7));
D.UI.ttStateStr{2} = sprintf('%sNew\nOrientation & Depth', blanks(8));

% Plot default hight
dflt_ht = ((D.UI.ttPanRng-10)/10)*0.8;
% Plot defualt left from pan
dflt_lf = [5, D.UI.ttPanPos(3)/2];
% Plot defualt width
dflt_wd = D.UI.ttPanPos(3)/2-10;
% Plot default bottom ps vector
dflt_btm = linspace(D.UI.ttPanPos(4)-dflt_ht, 5, 11);

% Create orientation text
ps = [dflt_lf(1), dflt_btm(2), dflt_wd, dflt_ht];
D.UI.txtPanTT(1) = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.txtFont,...
    'FontSize',8,...
    'FontWeight','Bold',...
    'HorizontalAlignment','left',...
    'Position',ps, ...
    'String',sprintf('New Screw\r\nOrientation'),...
    'Style','text');

% Create enter orientation popup-menue object
ps = [dflt_lf(2), dflt_btm(2), dflt_wd, dflt_ht];
D.UI.popOr = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.popFont,...
    'BackgroundColor',[1 1 1],...
    'FontSize',11,...
    'Position',ps,...
    'String',D.PAR.dirVec,...
    'Style','popupmenu',...
    'Enable', 'off', ...
    'Value',1);

% Create enter number of rotations text
ps = [dflt_lf(1), dflt_btm(3), dflt_wd, dflt_ht];
D.UI.txtPanTT(2) = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.txtFont,...
    'FontSize',8,...
    'FontWeight','Bold',...
    'HorizontalAlignment','left',...
    'Position',ps,...
    'String',sprintf('Number of\r\nFull Turns'),...
    'Style','text');
% Create number of rotations popup-menue object
ps = [dflt_lf(2), dflt_btm(3), dflt_wd, dflt_ht];
D.UI.popTrn = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.popFont,...
    'BackgroundColor',[1 1 1],...
    'FontSize',11,...
    'Position',ps,...
    'String',num2cell((0:20)'),...
    'Style','popupmenu',...
    'Enable', 'off', ...
    'Value',1);

% Create direction text
ps = [dflt_lf(1), dflt_btm(4), dflt_wd, dflt_ht];
D.UI.txtPanTT(3) = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.txtFont,...
    'FontSize',8,...
    'FontWeight','Bold',...
    'HorizontalAlignment','left',...
    'Position',ps,...
    'String',sprintf('TT Lowering\r\nDirection'),...
    'Style','text');
% Create direction popup-menue object
ps = [dflt_lf(2), dflt_btm(4), dflt_wd, dflt_ht];
D.UI.popDir = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.popFont,...
    'BackgroundColor',[1 1 1],...
    'FontSize',11,...
    'Position',ps,...
    'String',{'Down'; 'Up'},...
    'Style','popupmenu',...
    'Enable', 'off', ...
    'Value',1);

% Create new notes text box object
ps = [dflt_lf(1), dflt_btm(7), D.UI.ttPanPos(3)-15, dflt_ht*1.5];
D.UI.editNewNoteTT = uicontrol(...
    'Style','edit',...
    'Max', 100, ...
    'Parent',D.UI.panTT,...
    'BackgroundColor',[1 1 1],...
    'HorizontalAlignment','left',...
    'Units','Pixels',...
    'FontSize',10,...
    'Position',ps);

% Create old notes text box object
ps = [dflt_lf(1), ps(2)+ps(4), D.UI.ttPanPos(3)-15, dflt_ht*1.5];
D.UI.editOldNoteTT = uicontrol(...
    'Style','edit',...
    'Max', 100, ...
    'Parent',D.UI.panTT,...
    'BackgroundColor',[1 1 1],...
    'ForegroundColor', [0.5,0.5,0.5], ...
    'HorizontalAlignment','left',...
    'Enable', 'inactive', ...
    'Units','Pixels',...
    'FontSize',10,...
    'Position',ps);

% Create notes heading
ps = [dflt_lf(1), ps(2)+ps(4), D.UI.ttPanPos(3)-15, 15];
D.UI.txtPanTT(4) = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.txtFont,...
    'FontSize',8,...
    'HorizontalAlignment','left',...
    'Position',ps,...
    'String','Notes',...
    'Style','text');

% Create save TT data button object
ps = [30, dflt_btm(8), D.UI.ttPanPos(3)-65, dflt_ht];
D.UI.btnSaveTT = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'Callback',{@BtnSaveTT},...
    'UserData',[0,0],...
    'FontSize',12,...
    'FontWeight','Bold',...
    'Position',ps,...
    'Enable', 'off', ...
    'String','Save xx');

% Create bottom sub-pannel
ps = [dflt_lf(1), dflt_btm(end), D.UI.ttPanPos(3)-15, dflt_ht*3];
D.UI.spanTT = uibuttongroup(... %give structure label based on TT number
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontSize',7,...
    'HighlightColor',[0,0,0],...
    'ShadowColor',[0.5,0.5,0.5],...
    'Title',blanks(0),...
    'Clipping','off',...
    'Position',ps,...
    'SelectedObject',[],...
    'SelectionChangeFcn',[],...
    'OldSelectedObject',[]);

% Create TT depth text object
ps = [dflt_lf(1)+5, ps(2)+5, D.UI.ttPanPos(3)-25, dflt_ht];
D.UI.txtPanTT(7) = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.txtFont,...
    'FontSize',20,...
    'FontWeight','Bold',...
    'HorizontalAlignment','center',...
    'Position',ps,...
    'String','xx',...
    'Style','text');

% Create TT orientation text object
ps = [dflt_lf(1)+5, ps(2)+dflt_ht*0.75, D.UI.ttPanPos(3)-25, dflt_ht];
D.UI.txtPanTT(6) = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'Callback',{@directionText_Callback},...
    'FontName',D.UI.txtFont,...
    'FontSize',20,...
    'FontWeight','Bold',...
    'HorizontalAlignment','center',...
    'Position',ps,...
    'String','xx',...
    'Style','text');

% Create "Prior Direction & Depth" text
ps = [dflt_lf(1)+5, ps(2)+dflt_ht, D.UI.ttPanPos(3)-25, dflt_ht];
D.UI.txtPanTT(5) = uicontrol(...
    'Parent',D.UI.panTT,...
    'Units','Pixels',...
    'FontName',D.UI.txtFont,...
    'FontSize',11,...
    'FontWeight','Bold',...
    'HorizontalAlignment','Center',...
    'Position',ps,...
    'String',D.UI.ttStateStr{1},...
    'Style','text');

% Set text colors
set(D.UI.txtPanTT, 'ForegroundColor', D.UI.stateCol(2,:))

% Show figure
if ~D.F.hide_tt_fig
    set(D.UI.figTT,'Visible','On')
end

%==========================================================================

%% ====================== GRAPHICS FUNCTIONS ==============================

% ----------------------------- PLOT TT PATHS -----------------------------
    function []  = PlotTTPath(bndl, tt)
        
        % Get tt data
        tt_fld = D.TT.ttFlds{bndl}{tt};
        
        % Pull out all depths for this tt
        depths_old = [D.TT.ttLogTable{1:end-1, [tt_fld,'_D']}];
        depths_old_mm = depths_old/1000; % convert to mm
        
        % Append new depth if tt has been updated
        user_date = get(D.UI.h_btnTT(bndl,tt), 'UserData');
        state = user_date(3);
        if state == 1
            depths_new = D.TT.ttLogTable{D.TT.Ses+1, [tt_fld,'_D']};
            depths_new_mm = depths_new/1000;
            depths_all_mm = [depths_old_mm; depths_new_mm];
        else
            depths_all_mm = depths_old_mm;
        end
        
        % Get position in bundle
        [ap, ml] = find(ismember(D.TT.ttMap{bndl}, tt_fld));
        
        % Determine position relative to bundle center
        center = [ap, ml] - ceil(size(D.TT.ttMap{bndl})/2);
        
        % Flip A-P center
        center(1) = center(1)*-1;
        
        % Get x start pos with offset as a function of z pos
        x_offset = center(2)*D.UI.linOfst;
        x = D.TT.ttCoords{bndl}(1) + center(1)*D.PAR.canSp + x_offset;
        
        % Get y pos
        y_offset = center(1)*D.UI.linOfst;
        y = D.TT.ttCoords{bndl}(2) + center(2)*D.PAR.canSp + y_offset;
        
        % Align z to bundle implant pos
        z = D.TT.ttCoords{bndl}(3);
        
        % Get implant pos based on bundle angle
        [xa, za] = pol2cart(D.UI.bndAng(bndl), depths_all_mm);
        
        % Align x and y to bundle implant pos
        px = 100 * (x + xa);
        py = 100 * (repmat(y, length(px), 1));
        pz = 100 * (z + abs(za)) * -1;
        
        % Delete existing plot
        delete(D.UI.ttTrkLin(bndl,tt));
        delete(D.UI.ttTrkMrk(bndl,tt,:));
        
        % Create cylinder tt for each
        [D.UI.ttTrkLin(bndl,tt), D.UI.ttTrkMrk(bndl,tt,1:length(px))] = ...
            Get3dTT([px, py, pz], D.PAR.ttDiam*100, 20, state, D.UI.axe3D);
        
        % Set rod color
        set(D.UI.ttTrkLin(bndl,tt), ...
            'FaceColor', D.UI.ttLabCol{bndl}(tt,:), ...
            'FaceAlpha', D.UI.ttFaceAlph(2), ...
            'Visible', 'on')
        
        % Set sphere color
        set(D.UI.ttTrkMrk(bndl,tt,1:length(px)), ...
            'FaceColor', D.UI.ttLabCol{bndl}(tt,:), ...
            'FaceAlpha', D.UI.ttFaceAlph(2), ...
            'Visible', 'on')
        
    end

% ------------------------- CREATE 3D TT GRAPHICS -------------------------
    function [tt_rod_h, tt_sphere_h] = Get3dTT(ttXmat,r,n,s,ax)
        
        % Pull out start and values
        X1 = ttXmat(1,:);
        X2 = ttXmat(end,:);
        
        % Calculating the length of the cylinder
        length_cyl=norm(X2-X1);
        
        % Creating a circle in the YZ plane
        t=linspace(0,2*pi,n)';
        x2=r*cos(t);
        x3=r*sin(t);
        
        % Creating the points in the X-Direction
        x1=[0 length_cyl];
        
        % Creating (Extruding) the cylinder points in the X-Directions
        xx1=repmat(x1,length(x2),1);
        xx2=repmat(x2,1,2);
        xx3=repmat(x3,1,2);
        
        % Plotting the cylinder along the X-Direction with required length starting
        % from Origin
        tt_rod_h=mesh(ax, xx1, xx2, xx3, 'Visible', 'on');
        
        % Defining Unit vector along the X-direction
        unit_Vx=[1 0 0];
        
        % Calulating the angle between the x direction and the required direction
        % of cylinder through dot product
        angle_X1X2=acos( dot( unit_Vx,(X2-X1) )/( norm(unit_Vx)*norm(X2-X1)) )*180/pi;
        
        % Finding the axis of rotation (single rotation) to roate the cylinder in
        % X-direction to the required arbitrary direction through cross product
        axis_rot=cross([1 0 0],(X2-X1) );
        
        % Rotating the plotted cylinder and the end plate circles to the required
        % angles
        if angle_X1X2~=0 % Rotation is not needed if required direction is along X
            rotate(tt_rod_h,axis_rot,angle_X1X2,[0 0 0])
        end
        
        % Till now cylinder has only been aligned with the required direction, but
        % position starts from the origin. so it will now be shifted to the right
        % position
        set(tt_rod_h,'XData',get(tt_rod_h,'XData')+X1(1))
        set(tt_rod_h,'YData',get(tt_rod_h,'YData')+X1(2))
        set(tt_rod_h,'ZData',get(tt_rod_h,'ZData')+X1(3))
        
        % Setup sphere handle array
        tt_sphere_h = gobjects(1,size(ttXmat,1));
        
        % Create spheres
        for z_sph = 1:size(ttXmat,1)
            
            % Create sphere
            [sx,sy,sz] = sphere;
            
            % Get scaling factor
            if s == 1 && z_sph == size(ttXmat,1)
                scale = r*4;
            else
                scale = r*2;
            end
            
            % Scale and position
            sx = sx*scale + ttXmat(z_sph,1);
            sy = sy*scale + ttXmat(z_sph,2);
            sz = sz*scale + ttXmat(z_sph,3);
            
            % Plot sphere
            tt_sphere_h(z_sph) = mesh(ax, sx, sy, sz, 'Visible', 'on');
        end
        
        % Set lighting stuff
        set([tt_rod_h, tt_sphere_h], ...
            'EdgeAlpha',0, ...
            'FaceLighting', 'gouraud', ...
            'EdgeLighting', 'gouraud', ...
            'DiffuseStrength', 0.9, ...
            'AmbientStrength', 0.5)
    end

% ------------------------- CREATE 3D TT GRAPHICS -------------------------
    function [] = ShowActiveTT(bndl, tt)
        
        % Handle input args
        if nargin == 0
            do_set_all = true;
        else
            do_set_all = false;
        end
        
        % Set all back to default
        for z_b = 1:D.PAR.nBundles
            for z_tt = 1:D.TT.nTT(z_b)
                
                % Set rod color
                set(D.UI.ttTrkLin(z_b,z_tt), ...
                    'FaceAlpha', D.UI.ttFaceAlph(do_set_all+1));
                
                % Set sphere color
                inc_ind = isgraphics(D.UI.ttTrkMrk(z_b,z_tt,:));
                set(D.UI.ttTrkMrk(z_b,z_tt,inc_ind), ...
                    'FaceAlpha', D.UI.ttFaceAlph(do_set_all+1));
                
                % Set legend marker
                set(D.UI.ttLegMrk, ...
                    'LineWidth', D.UI.ttLegMrkWdth(1))
            end
        end
        
        % Set current tt
        if ~do_set_all
            
            % Set active rod alpha
            set(D.UI.ttTrkLin(bndl,tt), ...
                'FaceAlpha', D.UI.ttFaceAlph(2));
            
            % Set active sphere alpha
            inc_ind = isgraphics(D.UI.ttTrkMrk(bndl,tt,:));
            set(D.UI.ttTrkMrk(bndl,tt,inc_ind), ...
                'FaceAlpha', D.UI.ttFaceAlph(2));
            
            % Set active legend marker
            set(D.UI.ttLegMrk(bndl,tt), ...
                'LineWidth', D.UI.ttLegMrkWdth(2))
            
        end
        
    end

% ------------------- CENTER PAXINOS IMAGE ON CURRENT TT ------------------
    function [] = SetPaxToTT(bndl, tt)
        
        % Get tt data
        tt_fld = D.TT.ttFlds{bndl}{tt};
        
        % Get position in bundle
        [ap, ml] = find(ismember(D.TT.ttMap{bndl}, tt_fld));
        
        % Determine position relative to bundle center
        center = [ap, ml] - ceil(size(D.TT.ttMap{bndl})/2);
        
        % Flip A-P center
        center(1) = center(1)*-1;
        
        % Get x start pos with offset as a function of z pos
        x = D.TT.ttCoords{bndl}(1) + center(1)*D.PAR.canSp;
        
        % Get y pos
        y = D.TT.ttCoords{bndl}(2) + center(2)*D.PAR.canSp;
        
        % Align z to bundle implant pos
        z = D.TT.ttCoords{bndl}(3);
        
        % Get last depth
        depth_mm = D.TT.ttLogTable{D.TT.Ses+1, [tt_fld,'_D']}/1000;
        
        % Get implant pos based on bundle angle
        [xa, za] = pol2cart(D.UI.bndAng(bndl), depth_mm);
        
        % Align x and y to bundle implant pos
        px = x + xa;
        py = y;
        pz = (z + abs(za)) * -1;
        
        % Set paxinos sag 
        img_ind = find(D.TT.imgCoor{1}' > py, 1, 'first');
        if isempty(img_ind)
            img_ind = length(D.TT.imgCoor{1});
        end
        set(D.UI.sldSwtchImg(1), 'Value', img_ind);
        SldSwtchImg(D.UI.sldSwtchImg(1));
        
        % Set paxinos cor 
        img_ind = find(D.TT.imgCoor{2}' < px, 1, 'last');
        if isempty(img_ind)
            img_ind = 1;
        end
        set(D.UI.sldSwtchImg(2), 'Value', img_ind);
        SldSwtchImg(D.UI.sldSwtchImg(2));
        
        % Set hor to deepest tt in first bundle
        img_ind = find(D.TT.imgCoor{3}' < pz, 1, 'last');
        if isempty(img_ind)
            img_ind = 1;
        end
        set(D.UI.sldSwtchImg(3), 'Value', img_ind);
        SldSwtchImg(D.UI.sldSwtchImg(3));
        
    end

%==========================================================================

%% ====================== CALLBACK FUNCTIONS ==============================

% ----------------------- LOAD ALL PREVEOUS TT DATA -----------------------
    function PopLoadRatTTInf(~, ~, ~)
        
        % Get dropdown value
        dd_val = get(gcbo, 'Value');
        
        % Bail if not valid selection
        if dd_val == 1
            return
        end
        
        % Get rat number as string
        D.TT.ratID = D.PAR.ratList{dd_val};
        
        % Get Rat label
        D.PAR.ratLab = ... % ('r####')
            ['r',D.TT.ratID(1:4)];
        
        % Get rat index in D.TT_Log
        D.TT.ratInd = ...
            find(ismember(D.TT_Log.Properties.RowNames, D.PAR.ratLab));
        
        % Get implant coordinates
        D.TT.ttCoords = D.TT_Log.Implant_Coordinates(D.TT.ratInd,:);
        
        % Get number of bundles
        D.PAR.nBundles = ~any(isnan(D.TT.ttCoords{2}))+1;
        
        % Get tt configs
        D.TT.ttConfig = D.TT_Log.Implant_Configuration(D.TT.ratInd,:);
        
        % Get tt map
        D.TT.ttMap = D.TT_Log.Tetrode_Mapping(D.TT.ratInd,:);
        
        % Remove unused entries
        for z_b = 1:2
            if D.PAR.nBundles==1 && z_b == 2
                D.TT.ttMap{z_b} = D.TT.ttMap{1}(1,1);
            else
                D.TT.ttMap{z_b} = D.TT.ttMap{z_b}(1:D.TT.ttConfig{z_b}(1),1:D.TT.ttConfig{z_b}(2));
            end
        end
        
        % Get thread pitch
        D.UI.umPerTurn = D.TT_Log.Thread_Pitch(D.TT.ratInd)*1000;
        
        % Store current log
        D.TT.ttLogTable = D.TT_Log.Turn_Log{D.TT.ratInd};
        
        % Get session number
        D.TT.Ses = D.TT.ttLogTable.Session(end)+1;
        
        % Copy last entry to new entry row
        D.TT.ttLogTable = [D.TT.ttLogTable; D.TT.ttLogTable(end,:)];
        
        % Add new ses and date
        D.TT.ttLogTable{D.TT.Ses+1, 'Session'} = D.TT.Ses;
        D.TT.ttLogTable{D.TT.Ses+1, 'Date'}{:} = datestr(clock, 'yyyy-mm-dd_HH-MM-SS', 'local');
        
        % Initialize arrays
        D.TT.ttFlds = cell(1,2);
        D.TT.ttLabs = cell(1,2);
        D.TT.nTT = zeros(1,2);
        D.UI.ttLabCol = cell(1,2);
        
        % Get tt string info
        for z_b = 1:D.PAR.nBundles
            
            % Store tt fields
            D.TT.ttFlds{z_b} =  cellstr(char(D.TT.ttMap{z_b}(~isundefined(D.TT.ttMap{z_b}))));
            
            % Sort tt fields
            D.TT.ttFlds{z_b} = sort(D.TT.ttFlds{z_b});
            
            % Remove 'TT'
            D.TT.ttLabs{z_b} = regexprep(D.TT.ttFlds{z_b}, 'TT', '');
            
            % Get number of tts in each bundle
            D.TT.nTT(z_b) = length(D.TT.ttFlds{z_b});
            
            % Specify tt colors
            D.UI.ttLabCol{z_b} = hsv(D.TT.nTT(z_b));
        end
        
        % Create tt plot handles
        D.UI.ttTrkLin = gobjects(D.PAR.nBundles,max(D.TT.nTT));
        D.UI.ttTrkMrk = gobjects(D.PAR.nBundles,max(D.TT.nTT),50);
        
        % Resize axis
        center = (max([size(D.TT.ttMap{2}),size(D.TT.ttMap{1})]) + 0.5)/2;
        
        % Loop throug TT and plot tt locs
        for z_b = 1:D.PAR.nBundles
            
            % Set axis lims
            set(D.UI.axLeg(z_b), ...
                'XLim', [D.TT.ttConfig{z_b}(1)/2 - center+ 0.5, D.TT.ttConfig{z_b}(1)/2 + center + 0.5], ...
                'YLim', [D.TT.ttConfig{z_b}(2)/2 - center+ 0.5, D.TT.ttConfig{z_b}(2)/2 + center + 0.5])
            
            % Show implant A-P coordinates on x axes
            set(D.UI.axLeg(z_b).XLabel, ...
                'Units', 'Normalized', ...
                'FontSize', 9, ...
                'String', sprintf('A-P %0.2fmm', D.TT.ttCoords{z_b}(1)));
            D.UI.axLeg(z_b).XLabel.Position(2) = D.UI.axLeg(z_b).XLabel.Position(2)+0.05;
            
            % Show implant M-L coordinates on y axes
            set(D.UI.axLeg(z_b).YLabel, ...
                'Units', 'Normalized', ...
                'FontSize', 9, ...
                'String', sprintf('M-L %0.2fmm', D.TT.ttCoords{z_b}(2)));
            D.UI.axLeg(z_b).YLabel.Position(1) = D.UI.axLeg(z_b).YLabel.Position(1)+0.05;
            
            for z_tt = 1:D.TT.nTT(z_b)
                
                % Find position of next tt
                tt_fld = D.TT.ttFlds{z_b}(z_tt);
                [x,y] = find(ismember(D.TT.ttMap{z_b}, tt_fld) == 1);
                
                % Bail if not found
                if isempty(x) || isempty(y)
                    continue
                end
                
                % Plot tt marker
                D.UI.ttLegMrk(z_b,z_tt) = ...
                    plot(x , y, 'o', 'Parent', D.UI.axLeg(z_b), ...
                    'MarkerEdgeColor', D.UI.ttLabCol{z_b}(z_tt,:), ...
                    'MarkerFaceColor', D.UI.ttLabCol{z_b}(z_tt,:)*0.5, ...
                    'LineWidth', D.UI.ttLegMrkWdth(1), ...
                    'MarkerSize', 16);
                text(x , y, D.TT.ttLabs{z_b}{z_tt}, 'Parent', D.UI.axLeg(z_b), ...
                    'Color', [1, 1, 1], ...
                    'HorizontalAlignment', 'Center', ...
                    'FontSize', 9, ...
                    'FontWeight', 'bold')
            end
        end
        
        % Button position stuff
        ttBtnWd = D.UI.dfltWd/11;
        ttBtnLft = [2, 2*ttBtnWd+5, 3*ttBtnWd+8, 4*ttBtnWd+4];
        ttBtnHt = round(D.UI.ttBtnRng/(max(D.TT.nTT)+1) * 0.8);
        ttBtnBtm = linspace(D.UI.ttBtnRng-(D.UI.ttBtnRng/(max(D.TT.nTT)+1)), 2, max(D.TT.nTT)+1);
        
        % Load sound icon
        sndicon = imread(fullfile(D.DIR.img, 'Icons', 'speakericon.png'));
        
        % Create hangle array
        D.UI.h_btnTT = gobjects(D.PAR.nBundles,max(D.TT.nTT));
        D.UI.h_radSnd = gobjects(2,max(D.TT.nTT),2);
        
        % Loop through each bundle
        for z_b = 1:D.PAR.nBundles
            
            % Looop through each TT
            for z_tt = 1:D.TT.nTT(z_b)
                
                % Add tt button
                btn_pos = [ttBtnLft(1), ttBtnBtm(z_tt+1), ttBtnWd*1.75, ttBtnHt];
                D.UI.h_btnTT(z_b,z_tt) = ...
                    uicontrol('style','toggle', ...
                    'Parent', D.UI.panBtnLoadTT(z_b), ...
                    'Enable', 'Off', ...
                    'BackgroundColor', D.UI.stateBtnCol(1,:), ...
                    'ForegroundColor', D.UI.ttLabCol{z_b}(z_tt,:), ...
                    'String',D.TT.ttLabs{z_b}{z_tt},...
                    'Callback', {@BtnLoadTT},...
                    'UserData', [z_b,z_tt,0], ... % (bundle, tt, saved)
                    'Units','Pixels', ...
                    'Value', 0, ...
                    'Position', btn_pos, ...
                    'FontName',D.UI.btnFont,...
                    'FontWeight','Bold',...
                    'FontSize',12);
                
                % Add sound icon
                icon_pos = [ttBtnLft(2), ttBtnBtm(z_tt+1), ttBtnWd, ttBtnHt];
                D.axSndIcn(z_b,z_tt) = ...
                    axes('Units', 'Pixels', ...
                    'Parent', D.UI.panBtnLoadTT(z_b), ...
                    'Position', icon_pos);
                image(sndicon)
                axis off
                axis image
                
                % Add sound left/right radial button
                for z_sn = 1:2
                    rad_pos = [ttBtnLft(z_sn+2), ttBtnBtm(z_tt+1), ttBtnWd, ttBtnHt];
                    D.UI.h_radSnd(z_b,z_tt,z_sn) = ...
                        uicontrol('Style','radiobutton',...
                        'Parent', D.UI.panBtnLoadTT(z_b), ...
                        'Enable', 'Off', ...
                        'String', [], ...
                        'Callback', {@RadSnd},...
                        'UserData', [z_b,z_tt,z_sn], ...
                        'Units', 'Pixels', ...
                        'Position', rad_pos, ...
                        'BackgroundColor',[1,1,1],...
                        'FontWeight','Normal',...
                        'FontSize', 10,...
                        'Value',0);
                end
                
                % Plot tt track
                PlotTTPath(z_b, z_tt);
            end
        end
        
        % Disable load button
        set(D.UI.popLoadTT, 'Enable', 'Off');
        
        % Enable other buttons
        set(D.UI.h_btnTT(isgraphics(D.UI.h_btnTT)), 'Enable', 'On')
        set(D.UI.h_radSnd(isgraphics(D.UI.h_radSnd)), 'Enable', 'On')
        set(D.UI.panBtnLoadTT,'HighlightColor',D.UI.stateCol(1,:))
        set(D.UI.btnHideTT, 'Enable', 'On')
        for z_b = 1:D.PAR.nBundles
            set(D.UI.btnHideTT(z_b), 'Enable', 'On')
        end
        
        % Set paxinos sag center of first bundle
        img_ind = knnsearch(D.TT.imgCoor{1}', D.TT.ttCoords{1}(2));
        set(D.UI.sldSwtchImg(1), 'Value', img_ind);
        SldSwtchImg(D.UI.sldSwtchImg(1));
        
        % Set paxinos cor center of first bundle
        img_ind = knnsearch(D.TT.imgCoor{2}', D.TT.ttCoords{1}(1));
        set(D.UI.sldSwtchImg(2), 'Value', img_ind);
        SldSwtchImg(D.UI.sldSwtchImg(2));
        
        % Set hor to deepest tt in first bundle
        dpth_max = 0;
        for z_tt = 1:D.TT.nTT(1)
            dpth_max = max(dpth_max, D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFlds{1}{z_tt},'_D']});
        end
        img_ind = knnsearch(D.TT.imgCoor{3}', -1*dpth_max/1000);
        set(D.UI.sldSwtchImg(3), 'Value', img_ind);
        SldSwtchImg(D.UI.sldSwtchImg(3));
        
        % Auto run GUI
        if (D.DB.doAutoLoad)
            
            % Get number of full turns
            turn_ind = ...
                find(ismember(cellstr(get(D.UI.popTrn, 'Str')), num2str(round((D.DB.depthSet/(D.UI.umPerTurn/1000))))));
            if isempty(turn_ind)
                turn_ind = length(get(D.UI.popTrn,'String'));
            end
            
            % Loop through each bundle
            for z_b = 1:D.PAR.nBundles
                
                % Looop through each TT
                for z_tt = 1:D.TT.nTT(z_b)
                    
                    % Load next TT
                    set(D.UI.h_btnTT(z_b,z_tt), 'Value', 1);
                    BtnLoadTT(D.UI.h_btnTT(z_b,z_tt));
                    
                    % Pause
                    pause(0.05);
                    
                    % Set turns to max
                    set(D.UI.popTrn, 'Value', turn_ind);
                    
                    % Set to random orientation
                    %set(D.UI.popOr,'Value', ceil(rand(1,1)*length(get(D.UI.popOr,'String'))))
                    
                    % Pause
                    pause(0.05);
                    
                    % Save
                    BtnSaveTT();
                end
            end
        end
        
    end

% --------------------------- PLAY SOUND FO TT ----------------------------
    function RadSnd(~, ~, ~)
    end

% ---------------- BUTTON PRESS TO LOAD TT DATA INTO PANEL ----------------
    function BtnLoadTT(hObject, ~, ~)
        
        % Get values
        val = get(hObject, 'Value');
        x = get(hObject, 'UserData');
        bndl = x(1);
        tt = x(2);
        state = x(3) + 1;
        
        % Get tt field
        D.TT.ttFldNow = D.TT.ttFlds{bndl}{tt};
        
        % Check if bunton disabled
        if (val == 0)
            
            % Reinable all buttons
            set(D.UI.h_btnTT, 'Enable', 'on');
            
            % Show all tts
            ShowActiveTT();
            
            % Bail
            return;
        end
        
        % Disable all buttons
        set(D.UI.h_btnTT, 'Enable', 'off');
        
        % Enable this button
        set(D.UI.h_btnTT(bndl,tt), 'Enable', 'on');
        
        % Reset objects
        set(D.UI.popTrn, 'Value', 1);
        set(D.UI.popOr, 'Value', 1);
        set(D.UI.popDir, 'Value', 1);
        set(D.UI.editNewNoteTT,'String','');
        set(D.UI.h_btnTT, 'Value', 0);
        
        % Set current toggle button value
        set(D.UI.h_btnTT(bndl,tt), 'Value', 1);
        
        % Show last note
        set(D.UI.editOldNoteTT,'String',D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_N']}{:})
        
        % Get tt index
        D.TT.ttIndNow = find(ismember([D.TT.ttFlds{1};D.TT.ttFlds{2}],D.TT.ttFldNow));
        
        % Update past bundle and tt
        D.UI.pstBndl = bndl;
        D.UI.pstTT = tt;
        
        % Show active tt
        ShowActiveTT(bndl,tt);
        
        % Get last orientation
        orientations_last = D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_O']};
        
        % Handle first ses
        if isundefined(orientations_last)
            
            % Set to specific string
            orientations_last = '1st Rec';
            
            % Change orientation vector
            orientation_list =  D.PAR.dirVec;
            
        else
            
            % Change to string
            orientations_last = char(orientations_last);
            
            % change cardinal direction list
            orind = find(ismember(D.PAR.dirVec, orientations_last) == 1);
            orientation_list = circshift(D.PAR.dirVec, [0,-(orind-1)]);
        end
        
        % Get last depth
        depths_last = D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_D']};
        
        % Enable objects
        set(D.UI.txtPanTT(5), 'String', D.UI.ttStateStr{state});
        set(D.UI.panTT,'HighlightColor',D.UI.stateCol(state,:))
        set(D.UI.txtPanTT, 'ForegroundColor', D.UI.stateCol(state,:))
        set(D.UI.popOr, 'Enable', 'on');
        set(D.UI.popTrn, 'Enable', 'on');
        set(D.UI.popDir, 'Enable', 'on');
        set(D.UI.editNewNoteTT, 'Enable', 'on');
        set(D.UI.btnSaveTT, 'Enable', 'on');
        
        % Update pannel title
        set(D.UI.panTT, ...
            'Title',D.TT.ttFldNow)
        
        % Update orientation
        set(D.UI.popOr, ...
            'String', orientation_list);
        
        % Update save TT data button objec
        set(D.UI.btnSaveTT, ...
            'UserData',[bndl,tt],...
            'String',['Save ', D.TT.ttFldNow]);
        
        % Update tt orientation text object
        set(D.UI.txtPanTT(6), ...
            'String', orientations_last);
        
        % Update tt depth text object
        set(D.UI.txtPanTT(7), ...
            'String', sprintf('%d %sm', depths_last, char(181)));
        
        % Update pax view
        SetPaxToTT(bndl, tt);
        
    end

% ---------------------- SLIDER SWITCH PAXANOS IMAGES ---------------------
    function SldSwtchImg(hObject, ~, ~)
        
        % Get user data
        user_dat = get(hObject, 'UserData');
        obj_ind = user_dat(1);
        img_ind = round(get(hObject, 'Value'));
        
        % Change image visibility
        [D.UI.h_paxImg{obj_ind}(:).Visible] = deal('off');
        D.UI.h_paxImg{obj_ind}(img_ind).Visible = 'on';
        
        % Get coord sine
        if D.TT.imgCoor{obj_ind}(img_ind) == abs(D.TT.imgCoor{obj_ind}(img_ind))
            sin_str = '+';
        else
            sin_str = '-';
        end
        
        % Update image coordinates
        print_str = sprintf('%s%0.2f', sin_str, abs(D.TT.imgCoor{obj_ind}(img_ind)));
        set(D.UI.txtImgCorr(obj_ind),'String',print_str)
        
        % Refresh
        drawnow;
        
    end

% ---------------------------- HIDE SHOW BUNDLE ---------------------------
    function BtnHideShow(hObject, ~, ~)
        
        % Get user data
        bndl_ind = get(hObject, 'UserData');
        
        % Handles
        rod_arr_h = D.UI.ttTrkLin(bndl_ind, isgraphics(D.UI.ttTrkLin(bndl_ind,:)));
        sph_arr_h = D.UI.ttTrkMrk(bndl_ind,isgraphics(D.UI.ttTrkMrk(bndl_ind,:,:)));
        
        % Set visibility
        if get(hObject, 'Value') == 1
            set(rod_arr_h, 'Visible', 'on')
            set(sph_arr_h, 'Visible', 'on')
        else
            set(rod_arr_h, 'Visible', 'off')
            set(sph_arr_h, 'Visible', 'off')
        end
        
    end

% -------------------- BUTTON PRESS TO SAVE TT ENTRIES --------------------
    function BtnSaveTT(~, ~, ~)
        
        % Update Orientation
        orientations_list = get(D.UI.popOr,'String');
        orientations_last = get(D.UI.txtPanTT(6), 'String');
        orientations_new = char(orientations_list(get(D.UI.popOr,'Value')));
        
        % Save orientation to struct
        D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_O']}(:) = orientations_new;
        
        % Get total turns
        turn_list = get(D.UI.popTrn,'String');
        turns = str2double(turn_list(get(D.UI.popTrn,'Value')));
        
        % Convert to um
        turn_um = turns*D.UI.umPerTurn;
        
        % Get turn direction
        direction_new = get(D.UI.popDir, 'Value');
        if direction_new == 1
            direction_new = 1;
        else
            direction_new = -1;
        end
        
        % Update change in orientation
        if any(ismember(orientations_list, orientations_last))
            
            % Compute delta orientation
            delta_orientation =  (find(ismember(orientations_list, orientations_new) == 1)-1) / 16;
            
            % Convert to microns
            delta_orientation = delta_orientation * D.UI.umPerTurn;
            
        else
            delta_orientation = 0;
        end
        
        % Add turns
        delta_depth = direction_new * (delta_orientation + turn_um);
        
        % Get last depth
        depths_last = D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_D']};
        
        % Compute total depth
        depth_new = delta_depth+depths_last;
        
        % Save depth to struct
        D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_D']} = depth_new;
        
        % Update Notes
        new_note = get(D.UI.editNewNoteTT,'String');
        if ~strcmp(new_note, '')
            new_note_str = ...
                [D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_N']}{:}, ...
                sprintf('%s: %s\n\n', ...
                datestr(clock, 'yyyy-mm-dd', 'local'), ...
                get(D.UI.editNewNoteTT,'String'))];
        else
            new_note_str = D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_N']}{:};
        end
        
        % Save note to struct
        D.TT.ttLogTable{D.TT.Ses+1, [D.TT.ttFldNow,'_N']}{:} = new_note_str;
        
        % Update tt select button vars
        x = get(D.UI.btnSaveTT, 'UserData');
        bndl = x(1); 
        tt = x(2);
        set(D.UI.h_btnTT(bndl,tt), ...
            'BackgroundColor', D.UI.stateBtnCol(2,:),...
            'UserData', [bndl, tt, 1])
        
        % Update pannel display
        set(D.UI.txtPanTT(6), 'String', orientations_new);
        txt = sprintf('%d %sm', depth_new, char(181));
        set(D.UI.txtPanTT(7), 'String', txt);
        set(D.UI.txtPanTT(5), 'String', D.UI.ttStateStr{2});
        
        % Disable objects
        set(D.UI.panTT,'HighlightColor',D.UI.stateCol(2,:))
        set(D.UI.txtPanTT, 'ForegroundColor', D.UI.stateCol(2,:))
        set(D.UI.popOr, 'Enable', 'off');
        set(D.UI.popTrn, 'Enable', 'off');
        set(D.UI.popDir, 'Enable', 'off');
        set(D.UI.editNewNoteTT, 'Enable', 'off');
        set(D.UI.btnSaveTT, 'Enable', 'off');
        set(D.UI.h_btnTT, 'Value', 0);
        
        % Update track plot
        PlotTTPath(bndl,tt);
        
        % Update pax view
        SetPaxToTT(bndl, tt);
        
        % Reset buttons
        BtnLoadTT(D.UI.h_btnTT(bndl,tt));
        
    end

% ------------------ BUTTON PRESS TO SAVE ALL TT ENTRIES ------------------
    function BtnSaveAll(~, ~, ~)
        
        % Save back to table
        D.TT_Log.Turn_Log{D.TT.ratInd} = D.TT.ttLogTable;
        
        % Save out data
        TT_Log = D.TT_Log; %#ok<NASGU>
        save(D.DIR.ioTT_Log, 'TT_Log');
        
        % Set user data to 1
        set(gcbo, 'UserData', 1)
        
        % Disable button
        set(D.UI.btnSaveAll, 'Enable', 'Off')
        
    end

% ------------------ BUTTON PRESS TO QUIT OUT OF TT_TRACK -----------------
    function BtnQuit(~, ~, ~)
        
        % Display warning if save not done
        if get(D.UI.btnSaveAll, 'UserData') ~= 1
            
            % Construct a questdlg with two options
            choice = dlgAWL('!!WARNING: QUIT WITHOUT SAVING?!!', ...
                'ABBORT RUN', ...
                'Yes', 'No', [], 'No', ...
                D.UI.dlgPos, ...
                'Warn');
            
            drawnow; % force update UI
            % Handle response
            switch choice
                case 'Yes'
                case 'No'
                    return
            end
        end
        
        % Close main UI
        close(D.UI.figTT)
        
    end

% --------------------------- SET 3D PLOT VIEW ----------------------------
    function Set3dView(hObject, ~, ~)
        
        % Get user data
        user_dat = get(hObject,'UserData');
        
        % Set view
        if user_dat == 1
            % Set to saggital
            set(D.UI.sldYaw, 'Value', 0);
            set(D.UI.sldPitch, 'Value', 0);
        elseif user_dat == 2
            % Set to corronal
            set(D.UI.sldYaw, 'Value', 90);
            set(D.UI.sldPitch, 'Value', 0);
        elseif user_dat == 3
            % Set to horizontal
            set(D.UI.sldYaw, 'Value', 90);
            set(D.UI.sldPitch, 'Value', -90);
        elseif user_dat == 4
            % Set to orthoginal
            set(D.UI.sldYaw, 'Value', 45);
            set(D.UI.sldPitch, 'Value', -45);
        end
        
        % Get values
        yaw = -1*get(D.UI.sldYaw, 'Value');
        pitch = -1*get(D.UI.sldPitch, 'Value');
        
        % Set toggle
        set(D.UI.btnSetView(1:4 ~= user_dat), 'Value', 0);
        
        % Set axis view
        view(D.UI.axe3D, [yaw, pitch]);
    end

%==========================================================================

end