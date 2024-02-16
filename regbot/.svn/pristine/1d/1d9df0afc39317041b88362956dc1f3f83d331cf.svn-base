% plot af data dra regbot
clear
close all
%%
data20cm_1 = load('bal_vel_20cm.log');
data20cm_2 = load('bal_vel_20cm_2.log');
data20cm_3 = load('bal_vel_20cm_3.log');
data20cm_4 = load('bal_vel_20cm_4.log');
data20cm_5 = load('bal_vel_20cm_5.log');
%%  
%  1    time 0.004 sec
%  2    mission (2) state 2
%  3  4  5 Acc x,y,z [m/s2]: -5.061040 -0.011987 -8.391113
%  6  7  8 Gyro x,y,z [deg/s]: 0.427246 0.335693 -0.091553
%  9 10 Motor velocity ref left, right: 109.68 109.68
% 11 12 Motor voltage [V] left, right: 8.0 8.0
% 13 14 Wheel velocity [r/s] left, right: 0.0000 0.0000
% 15 16 17 18 Pose x,y,h,tilt [m,m,rad,rad]: 0.0002 -0.0000 -0.002591 0.497272
% 19    Battery voltage [V]: 12.26
% 20 21 Get data time [us]: 500 +ctrl 770
%% plot motor current/velocity
figure(20)
data1 = data20cm_1;
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,13)*0.03, 'b');
hold on
plot(data1(1:n,1), data1(1:n,14)*0.03, 'b');
set(gca,'FontSize',14)
grid on
title('balance step 0-20cm/s')
xlabel('[sec]');
ylabel('m/s')
legend('Left wheel', 'right wheel','motor voltage (left)',2)
%% plot motor current/velocity
figure(22)
data1 = data20cm_2;
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,13)*0.03, 'b');
hold on
plot(data1(1:n,1), data1(1:n,14)*0.03, 'b');
set(gca,'FontSize',14)
grid on
title('balance step 0-20cm/s')
xlabel('[sec]');
ylabel('m/s')
legend('Left wheel', 'right wheel','motor voltage (left)',2)
%% plot motor balance velocity
figure(22)
data1 = data20cm_3;
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,13)*0.03, 'b');
hold on
plot(data1(1:n,1), data1(1:n,14)*0.03, 'b');
set(gca,'FontSize',14)
grid on
title('balance step 0-20cm/s')
xlabel('[sec]');
ylabel('m/s')
legend('Left wheel', 'right wheel','motor voltage (left)',2)
%% plot motor balance velocity
figure(24)
data1 = data20cm_4;
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,13)*0.03, 'b');
hold on
plot(data1(1:n,1), data1(1:n,14)*0.03, 'b');
set(gca,'FontSize',14)
grid on
title('balance step 0-10cm/s')
xlabel('[sec]');
ylabel('m/s')
legend('Left wheel', 'right wheel','motor voltage (left)',2)
%% plot motor balance velocity
figure(25)
data1 = data20cm_5;
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,13)*0.03, 'b');
hold on
plot(data1(1:n,1), data1(1:n,14)*0.03, 'b');
set(gca,'FontSize',14)
grid on
title('balance step 0-10cm/s')
xlabel('[sec]');
ylabel('m/s')
legend('Left wheel', 'right wheel','motor voltage (left)',2)
