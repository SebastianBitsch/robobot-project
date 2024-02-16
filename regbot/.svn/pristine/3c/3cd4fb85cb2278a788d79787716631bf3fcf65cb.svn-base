% plot af data dra regbot
clear
close all
%%
data = load('gyro_noise.txt');
%  1    time 0.000 sec
%  2    mission (3) line 0
%  3  4  5 Acc x,y,z: 4.749370 -0.294888 8.774707
%  6  7  8 Gyro x,y,z: 0.076294 -1.350403 -2.113342
%  9 10 Motor velocity ref left, right: -0.99 -0.99
% 11 12 Motor voltage [V] left, right: -0.8 -0.8
% 13 14 Motor current left, right [A]: 0.000 -0.013
% 15 16 Wheel velocity [r/s] left, right: 0.0062 0.0061
% 17 18 19 20 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 -0.498902
% 21    Battery voltage [V]: 11.91
% 22 23 Get data time [us]: 510 +ctrl 860
% 24 25 26 27 Extra pt.: balE[0], balU[0], balUI[0], balUI[1]: 0.09 -0.9862 0.0000 0.0000
%% plot wheel velocity- one sample
figure(1)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,20), 'b');
hold on
plot(data(1:n,1), data(1:n,7), 'c');
plot(data(1:n,1), data(1:n,5), 'g');
plot(data(1:n,1), -data(1:n,15)/10, 'r');
%plot(data(1:n,1), data(1:n,11), 'b');
grid on
xlabel('time in sec');
legend('tilt', 'gyro-y/100','acc-x','motor ref/100 (left)','Ancher voltage',1)
%%
%% plot wheel velocity- one sample
data = load('gyro_noise_4.txt');
figure(2)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,20), 'b');
hold on
plot(data(1:n,1), data(1:n,7), 'c');
plot(data(1:n,1), data(1:n,5), 'g');
plot(data(1:n,1), data(1:n,15)*0.03, 'r');
plot(data(1:n,1), data(1:n,9)*0.03, 'm');
%plot(data(1:n,1), data(1:n,11), 'Y');
grid on
xlabel('time in sec');
title('soft mount -window type')
legend('tilt [rad]', 'gyro-y [rad/s]','acc-x','wheel velocity m/s','wheel vel-ref m/s','anchor voltage [v]',1)
%% plot wheel velocity- one sample
data = load('gyro_noise_5.txt');
figure(3)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,20), 'b');
hold on
plot(data(1:n,1), data(1:n,3), 'c');
plot(data(1:n,1), data(1:n,5), 'g');
plot(data(1:n,1), data(1:n,15)*0.03, 'r');
plot(data(1:n,1), data(1:n,9)*0.03, 'm');
%plot(data(1:n,1), data(1:n,11), 'Y');
grid on
xlabel('time in sec');
title('soft mount of IMU')
legend('tilt [rad]', 'gyro-y [rad/s]','acc-x','wheel velocity m/s','wheel vel-ref m/s','anchor voltage [v]',1)
%% plot wheel velocity- one sample
data = load('gyro_noise_6.txt');
figure(4)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,20), 'b');
hold on
plot(data(1:n,1), data(1:n,7), 'c');
plot(data(1:n,1), data(1:n,5), 'g');
plot(data(1:n,1), data(1:n,15)*0.03, 'r');
plot(data(1:n,1), data(1:n,9)*0.03, 'm');
%plot(data(1:n,1), data(1:n,11), 'Y');
grid on
xlabel('time in sec');
title('soft IMU mount - burre - meget l{\o}s')
legend('tilt [rad]', 'gyro-y [rad/s]','acc-x','wheel velocity m/s','wheel vel-ref m/s','anchor voltage [v]',1)
%% plot wheel velocity- one sample
data = load('gyro_noise_7.txt');
figure(7)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,20), 'b');
hold on
plot(data(1:n,1), data(1:n,7), 'c');
plot(data(1:n,1), data(1:n,5), 'g');
plot(data(1:n,1), data(1:n,15)*0.03, 'r');
plot(data(1:n,1), data(1:n,9)*0.03, 'm');
%plot(data(1:n,1), data(1:n,11), 'Y');
grid on
xlabel('time in sec (hard mount)');
title('hard mount of IMU')
legend('tilt [rad]', 'gyro-y [rad/s]','acc-x','wheel velocity m/s','wheel vel-ref m/s','anchor voltage [v]',1)
