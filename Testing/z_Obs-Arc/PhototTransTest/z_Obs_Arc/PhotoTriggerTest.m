function [] = PhotoTriggerTest()
del1 = 0.5;
del2 = 5;
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Img1.bmp'))
pause(10)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Trigger_Spot_West.bmp'))
pause(del1)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Img1.bmp'))
pause(del2)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Trigger_Spot_North.bmp'))
pause(del1)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Img1.bmp'))
pause(del2)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Trigger_Spot_East.bmp'))
pause(del1)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Img1.bmp'))
pause(del2)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Trigger_Spot_South.bmp'))
pause(del1)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Img1.bmp'))
pause(del2)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Trigger_Spot_All.bmp'))
pause(del1)
fullscreenAWL(fullfile('\\ICR_CHEETAH\ArenaControlTesting\TestWallImages\Photo', 'Img1.bmp'))
pause(10)
closescreenAWL

% %% load handel1.mat;
% x = zeros(length(y),1);
% x = y;
% x = rand(1,length(y))/20
% x = x';
% 
% y = sin(2*pi*(1/Fs:1/Fs:length(y)/Fs)*500);
% y = y';
% while true
% sound([y,x], Fs);
% pause(length(y)/Fs)
% end