% plot af data dra regbot
clear
close all
%% plot ballance on floor (free with wires - start flat)
data = load('b1/tilt_log_4.txt');
%  1    time 0.000 sec
%  2  3  4 Acc x,y,z [m/s2]: -2.637207 -1.026113 -10.699868
%  5  6  7 Gyro x,y,z [deg/s]: -9.765625 91.735840 -23.132324
%  8  9 Motor voltage [V] left, right: 3.0 3.0
% 10 11 Wheel velocity [r/s] left, right: 0.0000 0.0000
% 12 13 14 15 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.182945figure(86)
figure(86)
n = size(data,1) - 300;
hold off
plot(data(1:n,1), data(1:n,15), 'b');
hold on
plot(data(1:n,1), data(1:n,6)/100, 'r');
plot(data(1:n,1), data(1:n,2), 'c');
plot(data(1:n,1), data(1:n,4), 'k');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=70 rad/s')
legend('tilt (deg)','gyro y', 'acc-x','acc-tilt',1)
%% tilt calculation
tau = 1.0;
lp = tf(1,[tau 1]);
T = 0.002;
lpz = c2d(lp,T,'tustin')
taua = 0.01;
lpa = tf(1,[taua 1]);
Ta = 0.002;
lpza = c2d(lpa,Ta,'tustin')
lpz.num{1}(2);
figure(95)
hold off
tiltacc = atan2(-data(1:n,2), -data(1:n,4));
tiltaccfilt = filter(lpz.num{1}, lpz.den{1}, tiltacc,[0.48]);
tiltaccfilta = filter(lpza.num{1}, lpza.den{1}, tiltacc,[0.48]);
gyrofilt = filter(lpz.num{1}, lpz.den{1}, data(1:n,6)*pi/180)*tau;
tiltsum = tiltaccfilt - gyrofilt;
plot(data(1:n,1), data(1:n,15), 'm');
hold on
%plot(data(1:n,1), tiltacc, 'k');
plot(data(1:n,1), tiltaccfilt, 'b');
plot(data(1:n,1), data(1:n,6)*pi/180/10, 'r');
plot(data(1:n,1), gyrofilt, 'c');
plot(data(1:n,1), tiltsum, 'k');
plot(data(1:n,1), tiltaccfilta, 'y');
legend('robot tilt [rad]', 'acc tilt fitered [rad]', 'gyro-y/10 [rad/s]', 'gyro - filt', 'sum', 'acc-filt fast')%,'Location','SouthWest')
grid on
%% plot of raw data from ballance
figure(14)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,16)*10, 'b');
hold on
plot(data(1:n,1), data(1:n,3), 'r');
plot(data(1:n,1), data(1:n,5), 'm');
plot(data(1:n,1), data(1:n,7)/10, '.-c');
grid on
xlabel('time in sec');
title('ballance control - U-limit=70 rad/s')
legend('tilt error (rad*10)', 'acc X', 'acc Z', 'gyro y / 10',1)
%% CPU time
figure(15)
plot(data(1:n,1), data(1:n,21), 'r');
hold on
plot(data(1:n,1), data(1:n,20), 'b');
