function [M] = GetMapTrackBounds(P)

%% Calculate warp correction parameters

% Load images
M.X = zeros(400,2560,3);

% Get current monitor width
M.MonWth = get(0, 'MonitorPositions');
M.MonWth = M.MonWth(1,3); % first monitor width
M.MonCrt = ... % monitor resolution correction
    M.MonWth/(4*P.PrjWdt);
M.Mag = sprintf('-m%1.20g',1/M.MonCrt);

% Get image pixel dimensions for each quadrant
M.Xp = size(M.X,2)/4;
M.Yp = size(M.X,1);

% Get image pixel bounds in inches
% Arena Plane (made by connecting line between quad edges)
M.P1Xin = sin(deg2rad(P.A))*P.R*2;
M.P1Xinar = linspace(0, M.P1Xin, M.Xp);
% Y dimension is the same for all calculations
M.Yinar = linspace(0, P.ImgHt, M.Yp);

% Calculate inch to pixle conversion based on image Y npixels and wall
% height
M.In2Px = M.Yp/P.ImgHt;

% M.X COORDS
% Use trig to find boundery side lengths and angles
% Note: values refer to schematic "Projector_Project_Diagram.pptx"
% get paramiters to calculate angle B (projector angle)
M.b = P.R; % arena radius
M.c = P.PD+P.R; % side ajacent to projector angle
% ANGLES IN XZ PLANE
% angle B
M.Bar = ... % proj angle centered at screen center
    atand(M.P1Xinar(1:M.Xp/2) / (P.PD + (M.b - cos(deg2rad(P.A))*M.b)));
% angle C
M.Car = 180 - asind(M.c.*sin(deg2rad(M.Bar))./M.b);
% angle P.A
M.Aar = 180 - M.Bar - M.Car;

% M.X Map
% Get pixel corresponding to M.X pos on cylinder
M.X_map = ... % mirror ang P.A radians
    [-fliplr(deg2rad(M.Aar)), deg2rad(M.Aar)];
M.X_map = ... % make posetive
    M.X_map + max(M.X_map);
M.X_map = ... % scale to pixels
    ceil(M.X_map / max(M.X_map) * (M.Xp-1) + 1);

% Y COORDS
% Focal Plane
% Focal Plane (made by connecting line between quad edges)
M.P2Xin = tan(deg2rad(max(M.Bar)))*P.PD*2;
M.P2Xinar = linspace(0, M.P2Xin, M.Xp);
M.B2ar = ... % proj angle centered at screen center
    atand(M.P2Xinar(1:M.Xp/2) / P.PD);
M.FPar = ... % proj distance to focal plane by M.X pxl
    sqrt(P.PD^2 + (tan(deg2rad(M.B2ar))*P.PD).^2);
M.FPar = ... % mirror side P.A
    [fliplr(M.FPar), M.FPar];
M.FParp = ... % convert to pixels
    M.FPar * M.In2Px;

% Arena Plane
M.APar = ... % focal length to arena by ang P.A in M.X plane
    sqrt((sin(deg2rad(M.Aar))*M.b).^2 + ... % triangle hight
    (M.c - cos(deg2rad(M.Aar))*M.b).^2); % triangle base
M.APar = ... % mirror side P.A
    [fliplr(M.APar), M.APar];
M.AParp = ... % convert to pixels
    M.APar * M.In2Px;

% ANGLES IN ZY PLANE
M.D1ar = ... % D1 angle corrisponding to offset at each M.X pxl
    atand(P.TopDst ./ M.FPar);
M.Dmt = ... % D angle corrisponding to each Y by M.X pxl offset by P.TopDst
    atand(repmat((M.Yinar+P.TopDst)',1,M.Xp) ./ ...
    repmat(M.FPar,M.Yp,1));

% Y Map
% Get pixel corresponding to M.X pos on cylinder
M.Y_map = ... % get Y dim pxls for XbyY mat
    tan(deg2rad(M.Dmt)) .* repmat(M.AParp,M.Yp,1);
M.Y_map = round(M.Y_map); % round to nearest pixel
% Calculate padding for Y dim sampling
M.y_mins = [max(M.Y_map(1,:)), min(M.Y_map(1,:))];
M.imgPad = round(P.TopDst*M.In2Px + (M.y_mins(1) - M.y_mins(2)));
M.outInds = ... % find outlyer pixels
    M.Y_map <= M.imgPad | M.Y_map > M.Yp+M.imgPad;
M.Y_map(M.outInds) = M.Yp+M.imgPad; % set to pixel max

% Get rotated image

% pre-rotated matrix
M.ImgMat{1} = reshape(M.X,[M.Yp,M.Xp,4,3]);
M.ImgMat{1} = permute(M.ImgMat{1}, [1,2,4,3]);
% post-rotated matrix
M.imShft = floor(-40/90 * M.Xp); % shift by 40 deg
M.ImgMat{2} = circshift(M.X,[0,M.imShft,0,0]);
M.ImgMat{2} = reshape(M.ImgMat{2},[M.Yp,M.Xp,4,3]);
M.ImgMat{2} = permute(M.ImgMat{2}, [1,2,4,3]);

% Compress image to image plane dims
M.IP_X = (P.PD * tan(deg2rad(max(M.Bar))))*2;
M.FP_X = M.Xp/M.In2Px;
M.IPscl = M.FP_X / M.IP_X;
M.IPaspr = [M.IPscl, 1, 1]; % 1.19,1.28,1.3