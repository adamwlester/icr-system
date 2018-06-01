% Load config.json from repo root
cfg = jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
    '..', '..', '..', 'config.json')));

% Build full path to icon
fi = 'ArenaIcon_Small.bmp';
path = fullfile(cfg.PROJ_REPO_DIR, 'data', 'images', 'icons', fi);

% Read and convert icon
RGB = imread(path);
[icrIconData, icrIconMap] = rgb2ind(RGB, 256);