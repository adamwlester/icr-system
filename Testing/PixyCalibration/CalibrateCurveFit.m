% Paist backpack Y values from arduino
%PX = [175,170,165,160,155,150,145,140,135,131,126,122,117,113,108,104,101,97,93,89,86,82,79,76,72,69,66,63,60,57,54,52,49,47,44,41]; 

% Paist cube tracker Y values from arduino
PX = [160,156,150,145,139,134,128,123,118,113,108,103,98,93,89,85,81,77,73,69,66,63,59,56,53,50,47,43,40,38,35,33,30,28,25,22];

% Format corresponding cm values
CM = linspace(40.5, 78.5, length(PX));

% Fit
P = polyfit(PX, CM, 3);

% Plot unfitted
figure;
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