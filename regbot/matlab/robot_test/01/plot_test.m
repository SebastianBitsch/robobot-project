% plot af data dra regbot
clear
close all
%%
data = load('regbot_log.txt');
%  1    time 0.000 sec
%  2    mission (0) state 2
%  3  4  5 Acc x,y,z: 0.074321 0.014385 9.117543
%  6  7  8 Gyro x,y,z: 0.000000 0.114441 0.076294
%  9 10 Motor velocity ref left, right: 5.00 5.00
% 11 12 Motor voltage [V] left, right: 2.7 2.7
% 13 14 Motor current left, right [A]: -0.089 -0.285
% 15 16 Wheel velocity [r/s] left, right: 0.0000 0.0000
% 17    Turnrate [r/s]: 0.0000
% 18 19 20 21 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 -3.129814
% 22    Battery voltage [V]: 12.29
%
%% plot acceleration
figure(1)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,3), 'b');
hold on
plot(data(1:n,1), data(1:n,4), 'm');
plot(data(1:n,1), data(1:n,5)*1, 'g');
grid on
xlabel('time in sec');
ylabel('acceleration m/s^2')
legend('acc x', 'acc y', 'acc z',2)
title('ACC vel-step 5 to -30rad/s wheels up')
print -f1 -dpng regbot_acc.png
%
%% plot gyro and tilt
figure(2)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,6), 'b');
hold on
plot(data(1:n,1), data(1:n,7), 'm');
plot(data(1:n,1), data(1:n,8)*1, 'g');
plot(data(1:n,1), data(1:n,21)*1, 'r');
grid on
xlabel('time in sec');
ylabel('turn velocity deg/s')
legend('gyro x', 'gyro y', 'gyro z', 'tilt (rad)',2)
title('GYRO vel-step 5 to -30rad/s wheels up')
print -f2 -dpng regbot_gyro.png
%% plot wheel velocity and battery
figure(3)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,15), 'b');
hold on
plot(data(1:n,1), data(1:n,16), 'm');
plot(data(1:n,1), data(1:n,22)*1, 'g');
grid on
xlabel('time in sec');
ylabel('axle velocity rad/s')
legend('left', 'right', 'supply voltage [V]','Location','East')
title('AXLE VEL vel-step 5 to -30rad/s wheels up')
print -f3 -dpng regbot_axle_vel.png
%% plot motor voltage and current
figure(4)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,11), 'b');
hold on
plot(data(1:n,1), data(1:n,12), 'm');
plot(data(1:n,1), data(1:n,13), 'g');
plot(data(1:n,1), data(1:n,14), 'k');
grid on
xlabel('time in sec');
ylabel('Voltage [V], current [A]')
legend('motor volt left', 'motor volt right', 'cureent left','current right','Location','SouthEast')
title('MOTOR input vel-step 5 to -30rad/s wheels up')
print -f4 -dpng regbot_motor.png
