% Dir
inDir = 'C:\Users\lester\MeDocuments\Research\BarnesLab\Study_ICR\ICR_Code\ICR_Running\Testing\Output';
inFi = 'halt_error_6700.csv';
% Vel steps
V = (10:10:80)';

% Read in
halt_error = txt2cell(fullfile(inDir,inFi));
halt_error = cell2mat(halt_error(3:end,:));
CM = cell2mat(arrayfun(@(x) (mean(halt_error(halt_error(:,5)==x),1)), ...
    V, 'Uni', false));
figure;
ax = axes;
set(ax, 'XLim', [0,90])
%set(ax, 'YLim', [-5,25])
axis square
hold on

plot(halt_error(:,5),halt_error(:,1),'ko');
plot(V,CM,'ro-')

% Regression
[r,m,b] = regression(V',CM');
V2cm = (max(CM)-min(CM)) / 70;
plot(V,V*m + b,'bo-')

% Polyfit
P = polyfit(V, CM, 2);

CM1 = P(1).*V.^2 + P(2).*V.^1 + P(3);
plot(V, CM1, 'go-');

legend({'All','Average','Regression','Polyfit'},'Location','northwest')
% Copy these coeff values to arduino code
fprintf('%0.15f\n\r', P);

% Can check how well coeff work
velCoeff = [...
0.001830357142857, ...
0.131160714285714, ... 
- 2.425892857142854, ...
];
now_vel = 0:40;
h_error = ...
    velCoeff(1) .* (now_vel .* now_vel) + ...
    velCoeff(2) .* now_vel + ...
    velCoeff(3);
plot(now_vel, h_error, 'co-');