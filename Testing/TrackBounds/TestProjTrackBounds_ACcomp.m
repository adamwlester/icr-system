function [] = TestProjTrackBounds_ACcomp()
del = 0.001;
topDir = '\\ICR_CHEETAH\Study_ICR\MATLAB\ICR_ARENA\ArenaControl\Testing\TrackBounds\OutputImages';
fullscreenAWL(fullfile(topDir, 'img_0.bmp'))
pause(20)

% trigger each wall seperatetly 10 times
for j = 1:100
    for i = 1:128
        try fullscreenAWL(fullfile(topDir, ['img_', num2str(i), '.bmp']));
        catch 
        end
        pause(del)
    end
end

while true
    try fullscreenAWL(fullfile(topDir, ['img_', num2str(1), '.bmp']));
    catch
    end
    pause(0.1)
end
