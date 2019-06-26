% From: http://mathcentral.uregina.ca/QQ/database/QQ.09.99/john2.html
% calculate cone params (mm)
h = 13;
r = 9;
s = (sqrt(h^2+r^2));
c = 2*pi*r;
t = (180*c)/(pi*s);

% calculate base hole perams (mm)
sr = 2;
th = rad2deg(sin(h/s));
sh = h * (sr/r);
ss = (sqrt(sh^2+sr^2));