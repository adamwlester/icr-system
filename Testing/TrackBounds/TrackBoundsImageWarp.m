function [] = TrackBoundsImageWarp()

%% Set parameters
% Image processing
% Monitor and projector resolution
P.PrjWdt = 1280; % proj res width
P.PrjHt = 800; % proj res hight
P.PD = 32; % proj distance (in)
P.ImgHt = 25.25; % proj img hight (in)
P.TopDst = 7/8; % projector offset (in) [1 + 5/8 = 1.375]
% Arena dimensions
P.R = 55/2 + 0.118; % arena radius (in)
P.QdWdt = 43.3; % wall width
P.QdHt = 25.125; % wall hight
P.A = 44.9; % cylinder angle from screen center

% Optional opperations
P.DoRot = true; % Create and save rotated version
P.DoTrg = true; % Create and save version with TTL trigger
P.DoFdCu = true; % Create and save version with feeder cues

% Directories
P.inDir = ... % input png images
    'C:\Users\lester\Documents\Research\BarnesLab\Study_ICR\MATLAB\ICR_ARENA\ArenaControl\Testing\TrackBounds\InputImages';
P.outDir = ... % output bmp images
    'C:\Users\lester\Documents\Research\BarnesLab\Study_ICR\MATLAB\ICR_ARENA\ArenaControl\Testing\TrackBounds\OutputImages';


%% Get mapped image paramiters
[M] = GetMapTrackBounds(P);

%% Transform images
% Loop through and create a figure for pre- and post-rotation images
H.Fig = figure('MenuBar','figure','Color',[0,0,0],'Visible','on',...
    'Units', 'pixels', 'Position', [0, 0, M.MonWth, P.PrjHt*M.MonCrt]);

for i_q = 1:4
    % GET REMAPPED IMAGE
    M.X = ... % get M.X corrected image
        M.ImgMat{1}(:,M.X_map,:,1);
    Xpad = ... % create padded copy
        [zeros(M.imgPad,M.Xp,3); M.X];
    ImgWarp = M.X;
    for i_x = 1:M.Xp % Loop to get Y corrected pixels
        ImgWarp(:,i_x,:) =  Xpad(M.Y_map(:,i_x),i_x,:);
    end
    
    % Invert image top/bottom
    ImgWarp = flipdim(ImgWarp,1);
    
    % Show warp corrected image
    H.AxWl(i_q) = axes('Units','normal', ...
        'Position',[i_q/4 - 1/4, 0, 1/4, 1], ...
        'ylim', [0,M.Yp]);
    hold on;
    imshow(ImgWarp,'Parent',H.AxWl(i_q));
    axis tight;
    
    % Compress image to image plane dims
    set(H.AxWl(i_q),'DataAspectRatio',M.IPaspr)
    
end
%fipath = fullfile(P.outDir, 'Tracker(AllWhite)');
fipath = fullfile(P.outDir, 'img_0');
export_fig(H.Fig, [fipath,'.bmp'], '-nocrop', M.Mag)

% Create feeder cue images
% Create matrix to be warped
CTmplt = zeros(size(Xpad,1),size(Xpad,2));
CueWdt = ceil(4.5 * M.In2Px); % cue width
CueHt = ceil(2 * M.In2Px); % cue hight; larger = ceil(4 * M.In2Px); small = ceil(2 * M.In2Px);
curPx = ceil(linspace(CueWdt/2, M.Xp-CueWdt/2, 32)); % pos to plot cue
bluInt = [linspace(0.4,0.2,16), linspace(0.2,0.4,16)];
redInt = [linspace(1,0.5,16), linspace(0.5,1,16)];
count = 0;
for j_pn = 1:4
    for i_cu = 1:32
        count = count + 1;
        
        cuepx = curPx(i_cu); % pos in pixels
        CueX = cat(3,cuepx - CueWdt/2, cuepx + CueWdt/2) + 1; % cue x bounds
        CueY = [M.Yp*0.99 - CueHt; M.Yp*0.99]; % cue y bounds
        CueX = round(CueX);
        CueY = round(CueY) + M.imgPad;
        
        C = CTmplt;
        C(CueY(1):CueY(2), ...
            CueX(1,1):CueX(1,2)) = 1;
        % Perform warp correction
        C = C(:,M.X_map);
        Cwrp = C(1:M.Yp,:);
        for i_x = 1:M.Xp % Loop to get Y corrected pixels
            Cwrp(:,i_x) =  C(M.Y_map(:,i_x),i_x);
        end
        Cwrp = flipdim(Cwrp,1); % flip same as image
        
        % Plot patch
        Cnt = contour(Cwrp,1,'LineColor','none'); % use contour to get patch coords
        Cnt = Cnt(:,2:end);
        % blue
        backPnts = min(Cnt(1,2:end)) + ((max(Cnt(1,2:end)) - min(Cnt(1,2:end))) *0.4);
        x1 = Cnt(1,(Cnt(1,2:end) <= backPnts));
        y1 = Cnt(2,(Cnt(1,2:end) <= backPnts));
        H.PtchCu1 = patch(x1, y1, [0, 0, bluInt(i_cu)], 'Parent', H.AxWl(j_pn)); 
        set(H.PtchCu1, 'EdgeColor', 'none')
        % red
        frontPnts = min(Cnt(1,2:end)) + ((max(Cnt(1,2:end)) - min(Cnt(1,2:end))) *0.6);
        x2 = Cnt(1,(Cnt(1,2:end) >= frontPnts));
        y2 = Cnt(2,(Cnt(1,2:end) >= frontPnts));
        H.PtchCu2 = patch(x2, y2, [redInt(i_cu), 0, 0], 'Parent', H.AxWl(j_pn)); 
        set(H.PtchCu2, 'EdgeColor', 'none')
        
        %     % Save figure
        fipath = fullfile(P.outDir, ['img_', num2str(count)]);
        export_fig(H.Fig, [fipath,'.bmp'], '-nocrop', M.Mag)
        drawnow
        % Delete patch
        set(H.PtchCu1,'FaceColor',[0,0,0])
        set(H.PtchCu2,'FaceColor',[0,0,0])
    end
end

% Close figures
close(H.Fig)






