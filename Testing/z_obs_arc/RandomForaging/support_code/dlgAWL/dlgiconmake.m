fi = 'ArenaIcon_Small.bmp';
path = fullfile('C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running\IOfiles\Images\Icons',fi);
RGB = imread(path);
[icrIconData,icrIconMap] = rgb2ind(RGB,256);