% plot af data dra regbot
clear
close all
%  1    time 0.006 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z: 0.304478 -0.028770 -10.189209
%  6  7  8 Gyro x,y,z: 2.258301 -7.759094 4.684448
%  9 10 Motor velocity ref left, right: 0.08 -0.08
% 11 12 Motor voltage [V] left, right: -8.0 -8.0
% 13 14 Wheel velocity [r/s] left, right: -6.7613 0.0000
% 15 16 17 18 Pose x,y,h,tilt [m,m,rad,rad]: -0.0006 -0.0000 0.002595 -0.041514% 
% 19    Battery voltage [V]: 9.46
% 20 21 Get data time [us]: 490 +ctrl 800
% 22 23 24 25 Extra pt.: balE[0], balU[0], balUI[0], balUI[1]: 0.04 -99.0000 0.0000 0.0000
%% plot wheel velocity - velocity step
data = load('vel_step.txt');
figure(8)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,22), 'k');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,9), 'c');
%plot(data(1:n,1), data(1:n,15)*0.03, '.m');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=600 rad/s')
legend('tilt (deg)', 'bal error*100','wheel vel r/s', 'Wheel vel ref [r/s]',1)
%% gyro data - vel step
data = load('vel_step.txt');
figure(9)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,3), 'b');
hold on
plot(data(1:n,1), data(1:n,5), 'm');
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
plot(data(1:n,1), data(1:n,7), 'r');
%plot(data(1:n,1), data(1:n,8), 'c');
%plot(data(1:n,1), data(1:n,6), '.c');
grid on
xlabel('time in sec');
title('Vel step +40..-20 r/s')
legend('acc X', 'acc Z','tilt (deg)', 'gyro Y','gyro Z', 'gyro x',1)
%% plot ballance on floor
data = load('ballance_only_3.txt');
%  1    time 0.005 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z: -1.205923 -0.148643 9.714512
%  6  7  8 Gyro x,y,z: -17.021179 12.718201 -17.227173
%  9 10 Motor velocity ref left, right: -0.00 0.00
% 11 12 Motor voltage [V] left, right: 7.8 7.8
% 13 14 Wheel velocity [r/s] left, right: 0.0151 0.0000
% 15 16 17 18 Pose x,y,h,tilt [m,m,rad,rad]: 0.0004 -0.0000 0.000000 0.094861
% 19    Battery voltage [V]: 10.79
% 20 21 Get data time [us]: 490 +ctrl 810
% 22 23 24 25 Extra pt.: balE[0], balU[0], balUI[0], balUI[1]: -0.62 35.7788 0.0000 0.0000
figure(23)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi + 20, 'b');
hold on
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,9), 'c');
plot(data(1:n,1), data(1:n,23), '.-m');
plot(data(1:n,1), data(1:n,19), 'k');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=70 rad/s')
legend('tilt (deg)', 'wheel vel m/s', 'Wheel vel ref [m/s]','bal reg u(t)', 'bal I-term u(t)',1)
%% plot of raw data from ballance
figure(24)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*10, 'b');
hold on
plot(data(1:n,1), data(1:n,3), 'r');
plot(data(1:n,1), data(1:n,4), 'r');
plot(data(1:n,1), data(1:n,7)/10, '.-c');
grid on
xlabel('time in sec');
title('ballance control - U-limit=70 rad/s')
legend('tilt (rad*10)', 'acc X', 'acc Z', 'gyro y / 10',1)
