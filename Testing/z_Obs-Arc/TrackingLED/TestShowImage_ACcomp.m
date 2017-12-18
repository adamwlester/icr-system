function [] = TestShowImage_ACcomp()

topDir = '\\ICR_CHEETAH\Study_ICR\MATLAB\ICR_ARENA\ArenaControl\WallImages';
imgFi = 'Img1_C1.bmp';
% loop to allow corrected image to be perioldicaly updated
while true
    try
        fullscreenAWL(fullfile(topDir, imgFi))
    catch
    end
    pause(1)
end