%% plot_line_sensor log
close all
clear
%%
data1 = load('step_1_cm_kp01.txt');
data1d02 = load('step_1_d02_cm_kp01.txt');
data05 = load('step_05_cm_kp01.txt');
%
%% logfile from robot 20
%  1    time 0.004 sec
%  2    mission (5) state 2
%  3  4 Motor velocity ref left, right: 0.01 0.01
%  5  6 Motor voltage [V] left, right: 0.2 0.2
%  7  8 Wheel velocity [m/s] left, right: 0.0000 0.0000
%  9 10 11 12 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.912913
% 13 .. 27 Line sensor: left -0.081022 1, right 4.180000 0, values 527 622 628 465 201 147 114 93, white 0, used 1
% 29    Battery voltage [V]: 12.10
%% plot motor current/velocity
figure(20)
data = data1d02;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,13), 'b');
hold on
plot(data(1:n,1), data(1:n,14), 'r');
plot(data(1:n,1), data(1:n,3), 'r');
set(gca,'FontSize',14)
grid on
title('linesensor position')
xlabel('[sec]');
ylabel('cm')
legend('Left', 'right','motor voltage (left)',2)