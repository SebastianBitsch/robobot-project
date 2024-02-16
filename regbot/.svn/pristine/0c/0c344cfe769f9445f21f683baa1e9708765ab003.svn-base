% plot af data dra regbot
clear
close all
%%
data_1 = load('/home/chr/svnregbot/matlab/log_2015_10_31/bal_velctrl_1.log');
data_2 = load('/home/chr/svnregbot/matlab/log_2015_10_31/bal_velctrl_2.log');
data_3 = load('/home/chr/svnregbot/matlab/log_2015_10_31/bal_velctrl_3.log');
data_5 = load('/home/chr/svnregbot/matlab/log_2015_10_31/bal_velctrl_5.log');
%  1    time 0.023 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z [m/s2]: -3.490703 0.227759 -15.624252
%  6  7  8 Gyro x,y,z [deg/s]: 9.796143 40.618896 1.159668
%  9 10 Motor velocity ref left, right: 139.02 139.02
% 11 12 Motor voltage [V] left, right: 8.0 8.0
% 13 14 Motor current left, right [A]: 2.250 1.594
% 15 16 Wheel velocity [r/s] left, right: 21.4605 22.9074
% 17    Turnrate [r/s]: 0.2773
% 18 19 20 21 Pose x,y,h,tilt [m,m,rad,rad]: 0.0096 0.0000 0.000000 0.503032
% 22    Battery voltage [V]: 11.42
% 23 24 Get data time [us]: 500 +ctrl 790%% plot motor current/velocity
%%
figure(31)
data = data_1;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15)*0.03, 'b');
hold on
plot(data(1:n,1), data(1:n,16)*0.03, 'c');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,14), 'y');
set(gca,'FontSize',14)
grid on
title('balance step 0-10cm/s')
xlabel('[sec]');
ylabel('m/s')
legend('Left wheel', 'right wheel','curent (left)','current (right)',2)
%% plot motor current/velocity
figure(32)
data = data_2;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15)*0.03, 'b');
hold on
plot(data(1:n,1), data(1:n,16)*0.03, 'c');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,14), 'y');
set(gca,'FontSize',14)
grid on
title('balance step 10cm/s - turn Kp=32')
xlabel('[sec]');
ylabel('m/s, Amps')
legend('Left wheel', 'right wheel','curent (left)','current (right)',2)
%% plot motor current/velocity
figure(33)
data = data_3;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15)*0.03, 'b');
hold on
plot(data(1:n,1), data(1:n,16)*0.03, 'c');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,14), 'y');
set(gca,'FontSize',14)
grid on
title('balance step - no vel ctrl - bal kp=-300')
xlabel('[sec]');
ylabel('m/s, Amps')
legend('Left wheel', 'right wheel','curent (left)','current (right)',2)
%% plot motor current/velocity
figure(35)
data = data_5;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15)*0.03, 'b');
hold on
plot(data(1:n,1), data(1:n,16)*0.03, 'c');
%plot(data(1:n,1), data(1:n,17), 'r');
plot(data(1:n,1), data(1:n,21)*180/pi, 'r');
set(gca,'FontSize',14)
grid on
title('balance step - no vel ctrl - bal kp=-300')
xlabel('[sec]');
ylabel('m/s, Amps')
legend('Left wheel', 'right wheel','tilt',2)
%%
%  1    time 0.007 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z [m/s2]: -5.475801 0.002397 -8.295215
%  6  7  8 Gyro x,y,z [deg/s]: -0.885010 -1.800537 0.457764
%  9 10 Motor velocity ref left, right: 115.94 115.94
% 11 12 Motor voltage [V] left, right: 8.0 8.0
% 13 14 Motor current left, right [A]: 2.271 2.204
% 15 16 Wheel velocity [r/s] left, right: 8.5463 9.7904
% 17    Turnrate [r/s]: 0.2384
% 18 19 20 21 Pose x,y,h,tilt [m,m,rad,rad]: 0.0012 -0.0000 0.000000 0.514637
% 22    Battery voltage [V]: 11.36
% 23 24 Get data time [us]: 500 +ctrl 790
% 25 26 27 28 29 Ballance pitch ref -0.009624 [rad], bal-out 115.941650 [rad], 
%       I-out 11.057992 [m/s], D-out 104.883659 [rad], vel ref [m/s] 0.000000
%%
data_6 = load('/home/chr/svnregbot/matlab/log_2015_10_31/bal_velctrl_6.log');
data_7 = load('/home/chr/svnregbot/matlab/log_2015_10_31/bal_velctrl_7.log');
data_8 = load('/home/chr/svnregbot/matlab/log_2015_10_31/bal_velctrl_8.log');
data_9 = load('/home/chr/svnregbot/matlab/log_2015_10_31/bal_velctrl_9.log');
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
%% plot motor current/velocity
figure(36)
data = data_6;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15)*0.03, 'b');
hold on
plot(data(1:n,1), data(1:n,16)*0.03, 'c');
%plot(data(1:n,1), data(1:n,17), 'r');
plot(data(1:n,1), data(1:n,21)*180/pi, 'r');
set(gca,'FontSize',14)
grid on
title('balance step - no vel ctrl - bal kp=-300')
xlabel('[sec]');
ylabel('m/s, Amps')
legend('Left wheel', 'right wheel','tilt',2)
%% plot motor current/velocity
figure(37)
data = data_7;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15)*0.03, 'b');
hold on
plot(data(1:n,1), data(1:n,16)*0.03, 'c');
%plot(data(1:n,1), data(1:n,17), 'r');
plot(data(1:n,1), data(1:n,21)*180/pi, 'r');
set(gca,'FontSize',14)
grid on
title('balance step - no vel ctrl - bal kp=-300')
xlabel('[sec]');
ylabel('m/s, Amps')
legend('Left wheel', 'right wheel','tilt',2)
%% plot motor current/velocity
figure(38)
data = data_8;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15)*0.03, 'b');
hold on
plot(data(1:n,1), data(1:n,16)*0.03, 'c');
%plot(data(1:n,1), data(1:n,17), 'r');
plot(data(1:n,1), data(1:n,21)*180/pi, 'r');
set(gca,'FontSize',14)
grid on
title('balance step - no vel ctrl - bal kp=-200')
xlabel('[sec]');
ylabel('m/s, deg')
legend('Left wheel', 'right wheel','tilt',2)
%% plot motor current/velocity
figure(39)
data = data_9;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15)*0.03, 'b');
hold on
plot(data(1:n,1), data(1:n,16)*0.03, 'c');
%plot(data(1:n,1), data(1:n,17), 'r');
plot(data(1:n,1), data(1:n,21)*180/pi, 'r');
set(gca,'FontSize',14)
grid on
title('balance step - no vel ctrl - bal kp=-200')
xlabel('[sec]');
ylabel('m/s, deg')
legend('Left wheel', 'right wheel','tilt',2)
