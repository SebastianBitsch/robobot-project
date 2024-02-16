% plot af data dra regbot
clear
close all
%%
data_1 = load('/home/chr/svnregbot/matlab/position_step/posstep.txt');
%% logfile from robot Sofie (16)
%  1    time 0.004 sec
%  2    mission (7) state 2
%  3  4 Motor velocity ref left, right: 0.01 0.01
%  5  6 Motor voltage [V] left, right: 0.2 0.2
%  7  8 Motor current left, right [A]: -0.086 0.126
%  9 10 Wheel velocity [m/s] left, right: 0.0004 0.0000
% 11 12 13 14 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.645090
% 15    Battery voltage [V]: 12.20
% 16 17 Get data time [us]: 100 +ctrl 400
%%
figure(1)
data = data_1;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,3), 'b');
hold on
plot(data(1:n,1), data(1:n,5)/10, 'c');
plot(data(1:n,1), data(1:n,11), 'r');
%plot(data(1:n,1), data(1:n,14), 'y');
set(gca,'FontSize',14)
grid on
title('position step (1)')
xlabel('[sec]');
ylabel('m, m/s, V')
legend('Left vel ref', 'Left motor V/10','x-distance',2)
%%
data_2 = load('/home/chr/svnregbot/matlab/position_step/posstep_2.txt');
% logfile from robot Sofie (16)
%  1    time 0.004 sec
%  2    mission (7) state 2
%  3  4 Motor velocity ref left, right: 0.01 0.01
%  5  6 Motor voltage [V] left, right: 0.2 0.2
%  7  8 Motor current left, right [A]: -0.086 0.126
%  9 10 Wheel velocity [m/s] left, right: 0.0004 0.0000
% 11 12 13 14 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.645090
% 15    Battery voltage [V]: 12.20
% 16 17 Get data time [us]: 100 +ctrl 400
%%
figure(2)
data = data_2;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,3), 'b');
hold on
plot(data(1:n,1), data(1:n,5)/10, 'c');
plot(data(1:n,1), data(1:n,11), 'r');
%plot(data(1:n,1), data(1:n,14), 'y');
set(gca,'FontSize',14)
grid on
title('position step (1)')
xlabel('[sec]');
ylabel('m, m/s, V')
legend('Left vel ref', 'Left motor V/10','x-distance',2)
