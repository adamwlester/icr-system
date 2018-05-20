
% Paist tracker Y values from arduino
PX = [186,172.1,159,146,133,121,109,98,87,76.6,67,58,50,42,35,28,22,16,11,5.8];

% Format corresponding cm values
CM = linspace(21, 78, length(PX));

% Specify polynomial
polOrd = 4;

% Fit
P = polyfit(PX, CM, polOrd);

% Plot unfitted
figure;
plot(CM, PX, 'o-', ...
    'LineWidth', 2, ...
    'MarkerSize', 5, ...
    'Color', [0,0,0]);
axis square
hold on;

switch polOrd
    
    % Compute 2rd order
    case 2
        CM1 = P(1).*PX.^2 + P(2).*PX.^1 + P(3);
        
        % Compute 3rd order
    case 3
        CM1 = P(1).*PX.^3 + P(2).*PX.^2 + P(3).*PX.^1 + P(4);
        
        % Compute 4th order
    case 4
        CM1 = P(1).*PX.^4 + P(2).*PX.^3 + P(3).*PX.^2 + P(4).*PX.^1 + P(5);
        
end

%PX1 = CM*p(1)^2 + CM*p(2) + p(3);
plot(CM1, PX, 'o-', ...
    'LineWidth', 2, ...
    'MarkerSize', 5, ...
    'Color', [1,0,0]);

% Copy these coeff values to arduino code
fprintf('%0.15f,\n', P);