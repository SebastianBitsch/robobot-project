% plot af data dra regbot
clear
close all
%% plot ballance on floor (free with wires - start flat)
data = load('ballance_vel_ctrl_1.txt');
%  1    time 0.003 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z: 9.709717 -0.275708 -2.023457
%  6  7  8 Gyro x,y,z: 1.533508 -5.737305 -6.271362
%  9 10 Motor velocity ref left, right: 0.00 -0.00
% 11 12 Motor voltage [V] left, right: -2.2 -2.2
% 13 14 Wheel velocity [r/s] left, right: -0.0060 -0.0062
% 15 16 17 18 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 1.384959
% 19    Battery voltage [V]: 3.65
% 20 21 Get data time [us]: 490 +ctrl 850
% 22 23 24 25 Extra pt.: balE[0], balU[0], balUI[0], balUI[1]: -1.30 -70.0000 0.0000 0.0000
figure(13)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,18)*180/pi, 'b');
hold on
plot(data(1:n,1), data(1:n,13)/10, 'r');
plot(data(1:n,1), data(1:n,9), 'c');
plot(data(1:n,1), data(1:n,22)*10, '-m');
plot(data(1:n,1), data(1:n,11), 'k');
%plot(data(1:n,1), data(1:n,23), 'y');
grid on
xlabel('time in sec');
title('ballance control - no U-limit=70 rad/s')
legend('tilt (deg)','wheel vel m/s', 'Wheel vel ref [m/s]','ballance error','motor volt',1)
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
