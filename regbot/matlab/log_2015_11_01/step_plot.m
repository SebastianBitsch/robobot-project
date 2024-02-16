% plot af data dra regbot
clear
close all
%%
data_1 = load('/home/chr/svnregbot/matlab/log_2015_11_01/bal_drive_1.log');
data_2 = load('/home/chr/svnregbot/matlab/log_2015_11_01/bal_drive_2.log');
data_3 = load('/home/chr/svnregbot/matlab/log_2015_11_01/bal_drive_3.log');
data_4 = load('/home/chr/svnregbot/matlab/log_2015_11_01/bal_drive_4.log');
data_5 = load('/home/chr/svnregbot/matlab/log_2015_11_01/bal_drive_5.log');
%  1    time 0.003 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z [m/s2]: -5.487788 0.064731 -8.429472
%  6  7  8 Gyro x,y,z [deg/s]: -0.091553 -0.122070 0.061035
%  9 10 Motor velocity ref left, right: 107.39 107.39
% 11 12 Motor voltage [V] left, right: 8.0 8.0
% 13 14 Motor current left, right [A]: 1.858 1.334
% 15 16 Wheel velocity [r/s] left, right: -0.0000 0.0000
% 17    Turnrate [r/s]: 0.0000
% 18 19 20 21 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.511352
% 22    Battery voltage [V]: 11.36
% 23 24 Get data time [us]: 500 +ctrl 770
% 25 26 27 28 29 Ballance pitch ref 0.000000 [rad], bal-out 107.385956 [rad],
%            I-out 5.113451 [m/s], D-out 102.272507 [rad], vel ref [m/s] 0.000000
%% plot motor balance drive step
figure(41)
data = data_1;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15), 'b');
hold on
plot(data(1:n,1), data(1:n,16), 'c');
%plot(data(1:n,1), data(1:n,17), 'r');
plot(data(1:n,1), data(1:n,21), 'r');
set(gca,'FontSize',14)
grid on
title('balance step - 0.1m/s')
xlabel('[sec]');
ylabel('m/s, rad')
legend('Left wheel', 'right wheel','tilt',2)
%% plot motor balance drive step
figure(42)
data = data_2;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15), 'b');
hold on
plot(data(1:n,1), data(1:n,16), 'c');
plot(data(1:n,1), data(1:n,25), 'm');
plot(data(1:n,1), data(1:n,21), 'r');
set(gca,'FontSize',14)
grid on
title('balance step - 0.1m/s')
xlabel('[sec]');
ylabel('m/s, rad')
legend('Left wheel', 'right wheel','tilt ref','tilt',2)
%% plot motor balance drive step
figure(43)
data = data_3;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15), 'b');
hold on
plot(data(1:n,1), data(1:n,16), 'c');
plot(data(1:n,1), data(1:n,25), 'm');
plot(data(1:n,1), data(1:n,21), 'r');
set(gca,'FontSize',14)
grid on
title('balance step - 0.1m/s Kpwv=5')
xlabel('[sec]');
ylabel('m/s, rad')
legend('Left wheel', 'right wheel','tilt ref','tilt',2)
%% plot motor balance drive step
figure(44)
data = data_4;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15), 'b');
hold on
plot(data(1:n,1), data(1:n,16), 'c');
plot(data(1:n,1), data(1:n,25), 'm');
plot(data(1:n,1), data(1:n,21), 'r');
set(gca,'FontSize',14)
grid on
title('balance step - 0.2m/s Kpwv=5')
xlabel('[sec]');
ylabel('m/s, rad')
legend('Left wheel', 'right wheel','tilt ref','tilt',2)
%% plot motor balance drive step
figure(45)
data = data_5;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15), 'b');
hold on
plot(data(1:n,1), data(1:n,16), 'c');
plot(data(1:n,1), data(1:n,25), 'm');
plot(data(1:n,1), data(1:n,21), 'r');
set(gca,'FontSize',14)
grid on
title('balance step - 0.15m/s Kpwv=5')
xlabel('[sec]');
ylabel('m/s, rad')
legend('Left wheel', 'right wheel','tilt ref','tilt',2)
%% plot motor voltage when in balance
figure(451)
hold off
plot(data(1:n,1), data(1:n,11), 'b');
hold on
plot(data(1:n,1), data(1:n,12), 'c');
set(gca,'FontSize',14)
grid on
title('balance step - 0.15m/s Kpwv=5')
xlabel('[sec]');
ylabel('m/s, rad')
legend('Left motor voltage', 'right motor',2)
