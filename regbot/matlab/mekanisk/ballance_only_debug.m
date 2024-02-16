% plot af data dra regbot
clear
close all
%% plot ballance on floor (free with wires - start flat)
data = load('b1/regbot_log_5.txt');
%  1    time 0.001 sec
%  2    mission (2) state 3
%  3  4  5 Acc x,y,z [m/s2]: 2.730708 -0.246938 -10.016591
%  6  7  8 Gyro x,y,z [deg/s]: 0.053406 2.410889 0.106812
%  9 10 Motor velocity ref left, right: 70.00 70.00
% 11 12 Motor current left, right [A]: -0.291 -0.329
% 13 14 Wheel velocity [r/s] left, right: 0.0059 0.0058
% 15 16 17 18 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 -0.262720
% 19    Battery voltage [V]: 12.20
figure(85)
n = size(data,1) - 300;
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,7)/131*1*pi/180 * 1000, 'r');
plot(data(1:n,1), data(1:n,3)*10, 'c');
%plot(data(1:n,1), data(1:n,9), 'k');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=70 rad/s')
legend('tilt (deg)','gyro y', 'acc-x','acc-tilt',1)
%% tilt calculation
tau = 0.5;
lp = tf(1,[tau 1]);
T = 0.001;
lpz = c2d(lp,T,'tustin')
lpz.num{1}(2);
figure(95)
hold off
tiltacc = atan2(-data(1:n,3), -data(1:n,5));
tiltaccfilt = filter(lpz.num{1}, lpz.den{1}, tiltacc,[0.34]);
gyrofilt = filter(lpz.num{1}, lpz.den{1}, data(1:n,7)*pi/180)*tau;
tiltsum = tiltaccfilt - gyrofilt;
plot(data(1:n,1), data(1:n,18), 'm');
hold on
%plot(data(1:n,1), tiltacc, 'k');
plot(data(1:n,1), tiltaccfilt, 'b');
plot(data(1:n,1), data(1:n,7)/1000, 'r');
plot(data(1:n,1), gyrofilt, 'c');
plot(data(1:n,1), tiltsum, 'k');
legend('robot tilt', 'acc-based tilt fitered', 'gyro-y/1000', 'gyro - filt')
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
