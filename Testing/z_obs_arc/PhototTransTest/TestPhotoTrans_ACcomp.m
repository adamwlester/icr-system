function [] = TestPhotoTrans_ACcomp()
del1 = 0.25;
del2 = 0.5;
topDir = '\\ICR_CHEETAH\Study_ICR\MATLAB\ICR_ARENA\ArenaControl\Testing\TestWallImages\Photo';
fullscreenAWL(fullfile(topDir, 'Img1.bmp'))
pause(5)

% trigger each wall seperatetly 10 times
for i = 1:10
    fullscreenAWL(fullfile(topDir, 'Trigger_Spot_West.bmp'))
    pause(del1)
    fullscreenAWL(fullfile(topDir, 'Img1.bmp'))
    pause(del2)
    fullscreenAWL(fullfile(topDir, 'Trigger_Spot_North.bmp'))
    pause(del1)
    fullscreenAWL(fullfile(topDir, 'Img1.bmp'))
    pause(del2)
    fullscreenAWL(fullfile(topDir, 'Trigger_Spot_East.bmp'))
    pause(del1)
    fullscreenAWL(fullfile(topDir, 'Img1.bmp'))
    pause(del2)
    fullscreenAWL(fullfile(topDir, 'Trigger_Spot_South.bmp'))
    pause(del1)
    fullscreenAWL(fullfile(topDir, 'Img1.bmp'))
    pause(del2)
end
pause(5)
% trigger each wall at the same time 10 times
for i = 1:10
    fullscreenAWL(fullfile(topDir, 'Trigger_Spot_All.bmp'))
    pause(del1)
    fullscreenAWL(fullfile(topDir, 'Img1.bmp'))
    pause(del2)
end
pause(5)
closescreenAWL

%% load handel1.mat;
x = zeros(length(y),1);
x = y;
x = rand(1,length(y))/20
x = x';

y = sin(2*pi*(1/Fs:1/Fs:length(y)/Fs)*500);
y = y';
while true
sound([y,x], Fs);
pause(length(y)/Fs)
end