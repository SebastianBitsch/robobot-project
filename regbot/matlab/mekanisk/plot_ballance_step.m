% plot af data dra regbot
clear
close all
%%
data = load('ballance_late.txt');
%  1    time 0.002 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z: -0.069526 0.105488 9.836782
%  6  7  8 Gyro x,y,z: -0.038147 0.053406 0.030518
%  9 10 Motor velocity ref left, right: 370.61 370.60
% 11 12 Motor voltage [V] left, right: 11.9 11.9
% 13 14 Motor current left, right [A]: -0.006 0.032
% 15 16 Wheel velocity [r/s] left, right: -0.0051 0.0034
% 17 18 19 21 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.011639
% 21    Battery voltage [V]: 11.87
% 22 23 Get data time [us]: 490 +ctrl 830
% 24 25 26 27 Extra pt.: balE[0], balU[0], balUI[0], balUI[1]: -0.45 6.1391 0.0000 0.0000
%% plot wheel velocity- one sample
figure(1)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,20), 'b');
hold on
plot(data(1:n,1), data(1:n,7)/100, 'c');
plot(data(1:n,1), data(1:n,25), 'g');
plot(data(1:n,1), data(1:n,9)/100, 'r');
plot(data(1:n,1), data(1:n,11), 'b');
grid on
xlabel('time in sec');
legend('tilt', 'gyro-y/100','acc-x','motor ref/100 (left)','Ancher voltage',1)
%%
