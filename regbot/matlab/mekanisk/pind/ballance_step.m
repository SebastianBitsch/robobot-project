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
%% plot wheel velocity- one sample
data = load('ballance_step.txt');
figure(8)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,22)*100, 'k');
plot(data(1:n,1), data(1:n,23), 'r');
plot(data(1:n,1), data(1:n,9), 'c');
plot(data(1:n,1), data(1:n,13)*0.03, '.m');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=600 rad/s')
legend('tilt (deg)', 'bal error*100','bal ctrl out (u)', 'Wheel vel ref [r/s]','wheel vel',1)
%% plot wheel velocity- one sample
data = load('ballance_step_2.txt');
figure(12)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,22)*100, 'k');
plot(data(1:n,1), data(1:n,23), 'r');
plot(data(1:n,1), data(1:n,10), 'c');
plot(data(1:n,1), data(1:n,13), 'c');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=600 rad/s')
legend('tilt (deg)', 'bal error*100','bal ctrl out (u)', 'Wheel vel ref [r/s]',1)
%% plot wheel velocity- one sample
data = load('ballance_step_3.txt');

figure(13)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,22)*100, 'k');
plot(data(1:n,1), data(1:n,23), 'r');
plot(data(1:n,1), data(1:n,10), 'c');
plot(data(1:n,1), data(1:n,13), 'c');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=600 rad/s')
legend('tilt (deg)', 'bal error*100','bal ctrl out (u)', 'Wheel vel ref [r/s]',1)
%% plot wheel velocity- one sample
data = load('ballance_step_4.txt');
figure(13)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,22)*100, 'k');
plot(data(1:n,1), data(1:n,23), 'r');
plot(data(1:n,1), data(1:n,9), 'c');
plot(data(1:n,1), data(1:n,13), 'm');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=600 rad/s')
legend('tilt (deg)', 'bal error*100','bal ctrl out (u)', 'Wheel vel ref [r/s]','wheel vel',1)

%% plot wheel velocity- one sample
data = load('ballance_step_5.txt');
%  1    time 0.002 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z: -0.441133 -0.088706 -10.198798
%  6  7  8 Gyro x,y,z: -1.678467 -19.546509 -2.403259
%  9 10 Motor velocity ref left, right: 0.00 -0.00
% 11 12 Motor voltage [V] left, right: -0.0 -0.0
% 13 14 Motor current left, right [A]: 0.861 0.887
% 15 16 Wheel velocity [r/s] left, right: 0.0051 -0.0030
% 17 18 19 20 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.007780
% 21    Battery voltage [V]: 11.91
% 22 23 Get data time [us]: 490 +ctrl 840
% 24 25 26 27 Extra pt.: balE[0], balU[0], balUI[0], balUI[1]: -0.01 0.9841 0.0000 0.0000
figure(13)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,20)*180/pi, 'b');
hold on
%plot(data(1:n,1), data(1:n,3), 'r');
plot(data(1:n,1), data(1:n,7)/10, 'k');
%plot(data(1:n,1), data(1:n,25), 'k');
plot(data(1:n,1), data(1:n,9), 'c');
plot(data(1:n,1), data(1:n,11), 'r');
plot(data(1:n,1), data(1:n,15), 'm');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=600 rad/s')
legend('tilt (deg)', 'gyro Y',  'Wheel vel ref [r/s]','motor volt','wheel vel',1)
%% plot wheel velocity- one sample
data = load('ballance_step_6.txt');
%  1    time 0.001 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z: -2.905723 -0.081514 -9.611421
%  6  7  8 Gyro x,y,z: -0.007629 1.007080 -0.015259
%  9 10 Motor velocity ref left, right: 20.75 20.75
% 11 12 Motor voltage [V] left, right: 9.0 9.0
% 13 14 Wheel velocity [r/s] left, right: 0.0268 -0.0055
% 15 16 17 18 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.294896
% 19    Battery voltage [V]: 11.91
% 20 21 Get data time [us]: 490 +ctrl 870
% 22 23 24 25 Extra pt.: balE[0], balU[0], balUI[0], balUI[1]: -0.29 20.7514 0.0000 0.0000
figure(13)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,7)/10, 'k');
%plot(data(1:n,1), data(1:n,25), 'k');
%plot(data(1:n,1), data(1:n,9), 'c');
%plot(data(1:n,1), data(1:n,11), 'c');
plot(data(1:n,1), data(1:n,3), 'r');
plot(data(1:n,1), data(1:n,13), 'm');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=600 rad/s')
legend('tilt (deg)', 'gyro Y',  'acc - x','wheel vel',1)
