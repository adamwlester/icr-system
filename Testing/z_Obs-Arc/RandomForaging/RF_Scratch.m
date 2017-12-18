function[] = RF_Scratch()
close all;

% Set params
datFi = 'D.P.pathMat.mat';
dir = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running\Testing\RandomForaging';
doLoadPath = false;
doPlotPathAvg = false;
doPlotPathAll = true;
doSimRat = true;
D.PAR.pathWdth = 25;
pixels = 400;
D.PAR.rfBins = 401;
D.PAR.pathDegDist = 5;
% Pos lims
% arena radius (cm)
D.UI.arnRad = 70;
% track width (cm)
D.UI.trkWdt = 10;
% random forrage radius (cm)
D.UI.rfRad = 60;
% rew zone width (cm)
D.UI.rfTargWdt = 15;

% Mouse vars
doNewTarg = true;
rewTarg = 0;
rewBnd = [];
ptchRew = gobjects(1,1);

% Setup
cd(dir);
arnRad = 140/2;
cm2pxl = (pixels/2)/arnRad;
bndWdth = 10;
D.PAR.rfBinEdgeX = linspace(0,pixels,D.PAR.rfBins+1);
D.PAR.rfBinEdgeY = D.PAR.rfBinEdgeX;
D.P.occMat = zeros(D.PAR.rfBins,D.PAR.rfBins);
img = [];
rewCnt = 0;

% Get screen dimensions
sc = get(0,'MonitorPositions');
sc2 = sc(1,:);
sc1 = sc(2,:);

% Set figure
fig = figure();
set(fig,...
    'MenuBar', 'none', ...
    'Color', [1, 1, 1]);
% Set figure pos
fg_wh = [pixels*2 pixels*2];
fg_pos = [ ...
    sc1(3) + (sc2(3)-fg_wh(1))/2, ...
    (sc2(4) - fg_wh(2))/2, ...
    fg_wh(1), ...
    fg_wh(2)];
set(fig,'Position',fg_pos);

% First axis
ax_1 = axes('Position', [0,0,1,1], ...
    'Color', 'None', ...
    'YDir', 'reverse', ...
    'XLim', [0,D.PAR.rfBins], ...
    'YLim', [0,D.PAR.rfBins]);
hold on;

% Second axis
ax_2 = axes('Position', [0,0,1,1], ...
    'Color', 'None', ...
    'XLim', [0,pixels], ...
    'YLim', [0,pixels], ...
    'XTick', 0:pixels, ...
    'YTick', 0:pixels);
grid on;
hold on;

% Plot outer bounds
circ = [0:.01:2*pi,0];
[X_out,Y_out] = pol2cart(circ, ones(1,length(circ)) * (pixels/2));
X_out = X_out+pixels/2;
Y_out = Y_out+pixels/2;
plot(X_out,Y_out,'k', ...
    'Parent', ax_2);

% Plot inner bounds
circ = [0:.01:2*pi,0];
in_bnd = (pixels - pixels*(bndWdth/arnRad))/2;
[X_in,Y_in] = pol2cart(circ, ones(1,length(circ)) * in_bnd);
X_in = X_in+pixels/2;
Y_in = Y_in+pixels/2;
plot(X_in,Y_in,'k', ...
    'Parent', ax_2);

% Compute trajectories
if (doLoadPath)
    % Load path
    load(fullfile(dir,datFi));
    D.P.pathMat = double(D.P.pathMat);
else
    % Deg bin vars
    D.PAR.nPaths = 45/D.PAR.pathDegDist*2 + 1;
    n_targs = 360/D.PAR.pathDegDist;
    
    % Setup path mat
    D.P.pathMat = zeros(D.PAR.rfBins,D.PAR.rfBins,D.PAR.nPaths,n_targs);
    
    % Setup temp path mat
    %path_bins = round(D.PAR.rfBins*((D.UI.rfRad-ceil(D.UI.rfTargWdt/2)) / D.UI.arnRad));
    path_bins = D.PAR.rfBins;
    path_width = D.PAR.pathWdth*2;
    mat_eye = eye(path_bins*2 + D.PAR.pathWdth);
    mat_p = zeros(size(mat_eye));
    ind = floor(path_width/2)*-1 : floor(path_width/2);
    for i = 1:path_width
        if ind(i) < 0
            mat_p = mat_p + padarray(mat_eye(abs(ind(i))+1:end,:),[abs(ind(i)),0],'post');
        elseif ind(i) > 0
            mat_p = mat_p + padarray(mat_eye(1:end-abs(ind(i)),:),[abs(ind(i)),0],'pre');
        else
            mat_p = mat_p + mat_eye;
        end
    end
    
    % Setup sim rat mat
    mat_r = mat_p + padarray(mat_eye(1+1:end,:),[1,0],'post');
    rat_mat = D.P.pathMat;
    
    path_ang_arr = linspace(0,90,90/D.PAR.pathDegDist+1);
    for c = 1:2
        if c==1
            mat_now = mat_p;
        else
            mat_now = mat_r;
        end
        
        for i = 1:D.PAR.nPaths
            
            deg = path_ang_arr(i);
            % Rotate
            mat_rot = imrotate(mat_now,deg,'bilinear','crop');
            % Cut and pad
            rind = floor(size(mat_eye,1)/2)-floor(path_bins/2):floor(size(mat_eye,1)/2)+floor(path_bins/2);
            cind = floor(size(mat_eye,2)/2):floor(size(mat_eye,1)/2)+path_bins;
            mat_rot = mat_rot(rind,cind);
            % Flip
            mat_rot = flip(mat_rot,2);
            imgH = imagesc(mat_rot, 'Parent', ax_1);
            pause(0.5);
            delete(imgH);
            continue
            
            % Cut and pad
            mat_rot = mat_rot(floor(size(mat_eye,1)/2) - ceil(path_width/2) : end, ...
                floor(size(mat_eye,2)/2) - ceil(path_width/2): end);
            mat_rot = mat_rot(1:path_bins,1:path_bins);
            mat_rot = padarray(mat_rot, [0,path_bins-size(mat_rot,2)],'pre');
            % Flip
            mat_rot = flip(flip(mat_rot',2),1);
            

            
            
            % Store
            for j = [1:n_targs]-1
                if c==1
                    D.P.pathMat(:,:,i+1,j+1) = imrotate(mat_rot,j*D.PAR.pathDegDist,'bilinear','crop');
                else
                    rat_mat(:,:,i+1,j+1) = imrotate(mat_rot,j*D.PAR.pathDegDist,'bilinear','crop');
                    rat_mat(:,:,D.PAR.nPaths-i,j+1) = imrotate(mat_mir,j*D.PAR.pathDegDist,'bilinear','crop');
                end
            end
        end
    end
    
    % Mask values outside circle
    [colNums, mat_rot] = meshgrid(1:path_bins, 1:path_bins);
    mask = (mat_rot - ceil(path_bins/2)).^2 ...
        + (colNums - ceil(path_bins/2)).^2 <= ceil(path_bins/2).^2;
    mask = repmat(mask,[1,1,D.PAR.nPaths,n_targs]);
    D.P.pathMat = D.P.pathMat.*mask;
    rat_mat = rat_mat.*mask;
    
    % Nomalize
    D.P.pathMat(D.P.pathMat>0) = 1;
    D.P.pathMat = D.P.pathMat ./ repmat(sum(sum(D.P.pathMat,1),2),[path_bins,path_bins,1,1]);
    rat_mat(rat_mat>0) = 1;
    
    % Save path
    D.P.pathMat = single(D.P.pathMat);
    %save(fullfile(dir,datFi),'D.P.pathMat');
    D.P.pathMat = double(D.P.pathMat);
    
    
    % Plot path averages
    if (doPlotPathAvg)
        % Plot accross paths
        ih = imagesc(sum(D.P.pathMat(:,:,:,1),3), 'Parent', ax_1);
        pause(1);
        delete(ih);
        % Plot accross start pos and paths
        ih = imagesc(sum(sum(D.P.pathMat(:,:,:,:),3),4), 'Parent', ax_1);
        pause(1);
        delete(ih);
    end
    
    % Plot each path
    if (doPlotPathAll)
        for j = 1:n_targs
            ih = imagesc(sum(D.P.pathMat(:,:,:,j),3), 'Parent', ax_1);
            pause(0.5);
        end
    end
    
end

% Get mouse position
if (~doSimRat)
    
    % Get first reward target
    GetNewTarg();
    doNewTarg = false;
    
    % Set callback
    set(fig, 'WindowButtonMotionFcn', @getVT);
end

% Simulate rat
if (doSimRat)
    while(true)
        GetNewTarg();
        pause(0.1);
    end
end



% GET MOUSE POS
    function getVT (~, ~)
        
        % Block callback re-entry
        s = dbstack();
        if numel(s) > 1
            return;
        end
        
        % Get new mouse data
        C = get (ax_2, 'CurrentPoint');
        
        % Get cart coord
        x = C(1,1);
        y = C(1,2);
        
        % Get pol coord
        [rad, roh] = cart2pol(x - (pixels/2), y - (pixels/2));
        rad = wrapTo2Pi(rad);
        
        % Bail if out of bound
        if roh > (pixels)/2
            return;
        end
        
        % Get new target
        if doNewTarg
            
            % Reset flag
            doNewTarg = false;
            
            % Track rewards
            rewCnt = rewCnt+1;
            fprintf('rewCnt=%d\r\n', rewCnt)
            
            % Get new target
            GetNewTarg();
            
            % Bail
            return;
        end
        
        % Plot trig bound dat in red
        if roh > ((pixels - pixels*(bndWdth/arnRad))/2)
            
            % Check if in reward bounds
            in_bnd = Check_Rad_Bnds(rad, rewBnd);
            
            % Handle targ reached
            if (in_bnd)
                
                % Change targ patch color
                set(ptchRew, 'FaceColor', [0,1,0]);
                
                % Set to find new target
                doNewTarg = true;
            end
            
        end
        
        % Histogram data
        N = histcounts2(y,x,D.PAR.rfBinEdgeY,D.PAR.rfBinEdgeX);
        D.P.occMat = D.P.occMat+flip(N,1);
        
        % Plot valuse
        delete(img);
        img = imagesc(D.P.occMat, ...
            'Parent', ax_1);
    end

% GET NEW TARG
    function [] = GetNewTarg()
        
        % Local vars
        D.PAR.pathTargArr = 0:D.PAR.pathDegDist:360-D.PAR.pathDegDist;
        path_arr = linspace(-45,45,45/D.PAR.pathDegDist*2 + 1);
        
        % Get inner product of current pos and occ
        D.I.targInd = find(D.PAR.pathTargArr == rewTarg);
        occ_prod = ...
            squeeze(sum(sum(D.P.pathMat(:,:,:,D.I.targInd).*repmat(D.P.occMat,[1,1,size(D.P.pathMat,3)]),1),2));
        path_ind = find(occ_prod == min(occ_prod));
        if length(path_ind) > 1
            path_ind = path_ind(ceil(rand(1,1)*length(path_ind)));
        end
        path_ang = path_arr(path_ind)*2;
        
        % Get new targ
        rew_last = rewTarg;
        rewTarg = 360/2 + rew_last + path_ang;
        if rewTarg >= 360
            rewTarg = rewTarg - 360;
        elseif rewTarg < 0
            rewTarg = rewTarg + 360;
        end
        
        fprintf('rew_last=%d path_ind=%d path_new=%d rewTarg=%d \r\n', rew_last, path_ind, path_ang, rewTarg)
        
        % Plot path
        if (~doSimRat)
            path_plot = D.P.pathMat(:,:,path_ind,D.I.targInd);
            path_plot(path_plot>0) = 1;
            img = imagesc(path_plot+D.P.occMat, ...
                'Parent', ax_1);
        end
        
        % Add rat path to OCC
        if (doSimRat)
            D.P.occMat = rat_mat(:,:,path_ind,D.I.targInd)+D.P.occMat;
            img = imagesc(D.P.occMat, ...
                'Parent', ax_1);
        end
        drawnow;
        
        % Plot targ patch
        rewBnd = [deg2rad(rewTarg-5), deg2rad(rewTarg+5)];
        [xbnd, ybnd] =  Get_Rad_Bnds(rewBnd);
        delete(ptchRew);
        ptchRew = ...
            patch([xbnd(1,:),fliplr(xbnd(2,:))], ...
            [ybnd(1,:),fliplr(ybnd(2,:))], ...
            [1,0,0], ...
            'FaceAlpha',0.9, ...
            'Parent',ax_2);
        drawnow;
    end

% CHECK RAD BOUNDS
    function [bool_arr] = Check_Rad_Bnds(rad_arr, polbnds)
        
        if all(isnan(rad_arr))
            bool_arr = false(size(rad_arr));
        else
            if polbnds(1) > polbnds(2)
                polbnds = wrapToPi(polbnds);
                rad_arr = wrapToPi(rad_arr);
            end
            bool_arr = rad_arr > polbnds(1) & rad_arr < polbnds(2);
        end
        
    end

% GET TRACK BOUNDS
    function [xbnd, ybnd] = Get_Rad_Bnds(polbnds)
        
        if (length(polbnds) == 2)
            if polbnds(1) > polbnds(2)
                polbnds = wrapToPi(polbnds);
            end
            radDist = min(2*pi - abs(polbnds(2)-polbnds(1)), abs(polbnds(2)-polbnds(1)));
            nPoints =  round(360 * (radDist/(2*pi))); % 360 pnts per 2*pi
            polbnds = linspace(polbnds(1), polbnds(2), nPoints);
        end
        % inner bounds 0 deg
        [x(1,:),y(1,:)] = pol2cart(polbnds, ones(1,length(polbnds)) * (arnRad-bndWdth));
        xbnd(1,:) = x(1,:)*cm2pxl + arnRad*cm2pxl;
        ybnd(1,:) = y(1,:)*cm2pxl + arnRad*cm2pxl;
        % outer bounds 0 deg
        [x(2,:),y(2,:)] = pol2cart(polbnds, ones(1,length(polbnds)) * arnRad);
        xbnd(2,:) = x(2,:)*cm2pxl + arnRad*cm2pxl;
        ybnd(2,:) = y(2,:)*cm2pxl + arnRad*cm2pxl;
        
    end

% ROTATE MATRIX
    function[M_rot] = rotMat(M,ang)
        
        % Check if not rotating
        if (ang==0)
            M_rot = M;
            return;
        end
        
        % Convert to rad
        theta = ang*pi/180;
        
        % Make grid
        [x y] = ndgrid(1:size(M,1), 1:size(M,2));
        
        % Calculate rotation matrix
        R = [ cos(theta) -sin(theta);
            sin(theta)  cos(theta)]; % just 2D case
        
        % calculate new positions of image indicies
        
        tmp = R*[x(:)' ; y(:)']; % 2 by numel(M)
        xi = reshape(tmp(1,:),size(x)); % new x-indicies
        yi = reshape(tmp(2,:),size(y)); % new y-indicies
        
        M_rot = interpn(x,y,M,xi,yi); % interpolate from old->new indicies
        M_rot(isnan(M_rot)) = 0;
        
        
    end

end



