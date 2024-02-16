% plot af data dra regbot
clear
close all
%% plot wheel velocity- one sample
data = load('ballance_1.txt');
%  1    time 0.001 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z: -0.534634 -0.079116 -10.244350
%  6  7  8 Gyro x,y,z: 2.723694 8.972168 16.036987
%  9 10 Motor velocity ref left, right: -99.00 -99.00
% 11 12 Motor voltage [V] left, right: -9.0 -9.0
% 13 14 Wheel velocity [r/s] left, right: 0.0088 0.0142
% 15 16 17 18 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 -0.036552
% 19    Battery voltage [V]: 11.25
figure(100)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,7)/10, 'k');
%plot(data(1:n,1), data(1:n,25), 'k');
%plot(data(1:n,1), data(1:n,9), 'c');
%plot(data(1:n,1), data(1:n,11), 'c');
plot(data(1:n,1), data(1:n,3), 'r');
plot(data(1:n,1), data(1:n,19), 'm');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=600 rad/s')
legend('tilt (deg)', 'gyro Y',  'acc - x','wheel vel',1)
