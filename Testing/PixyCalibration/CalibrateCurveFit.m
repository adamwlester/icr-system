% Paist Y values from arduino
PX = [182,173,168,163,157,152,147,142,137,132,127,123,118,113,109,105,101,97,93,89,86,83,79,76,72,69,66,62,60]; 

% Format corresponding cm values
CM = 0:length(PX)-1;
CM = 37 + CM;

% Fit
P = polyfit(PX, CM, 3);

plot(CM, PX, '.-k');
axis square
hold on;

% Compute 4th order
%CM1 = P(1).*PX.^4 + P(2).*PX.^3 + P(3).*PX.^2 + P(4).*PX.^1 + P(5);
% Compute 3rd order
CM1 = P(1).*PX.^3 + P(2).*PX.^2 + P(3).*PX.^1 + P(4);

%PX1 = CM*p(1)^2 + CM*p(2) + p(3); 
plot(CM1, PX, '.-b');

% Copy these coeff values to arduino code
fprintf('%0.15f,\n', P);