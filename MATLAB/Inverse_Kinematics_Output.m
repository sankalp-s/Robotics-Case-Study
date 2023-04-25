
clc
clear

% DEFINE ALL VALUES OF x,y,z
[x, y, z] = deal(1, 2, 3);


N = [0, 0, 1]; % normal vector
N = N/mag(N); % converting vector 'n' to a unit vector

H = [0, 0, 4]; % center point (platform)
a(x) = 0.0001; % x component of point af

% input user (defined all lengths)
l0 = 2.875;
lf = 2.375;
d1 = 1;
d2 = 1.625;
m = 0.5;
p1 = 1;
p2 = 4.375;

%% MAIN CODE
output(N, H, a(x), l0, lf, d1, d2, m, p1, p2); % calcuates all theta values 
         