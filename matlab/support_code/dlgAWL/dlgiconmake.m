fi = 'ArenaIcon_Small.bmp';
path = fullfile('C:\Users\lester\repos\icr-system\data\images\icons',fi);
RGB = imread(path);
[icrIconData,icrIconMap] = rgb2ind(RGB,256);