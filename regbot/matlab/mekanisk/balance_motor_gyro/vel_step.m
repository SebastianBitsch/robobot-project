% plot af data dra regbot
clear
close all
%%
data_kp001 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step_kp001.log');
data_kp005 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step_kp005.log');
data_kp01 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step_kp01.log');
data_kp045 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step_kp045.log');
data_kp05 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step.log');
data_kp065 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step_kp065.log');
data_kp07 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step_kp07.log');
data_kp08 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step_kp08.log');
data_kp09 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/vel_step_kp09.log');
%  1    time 0.000 sec
%  2    mission (0) state 2
%  3  4 Motor velocity ref left, right: -8.00 -8.00
%  5  6 Motor voltage [V] left, right: -8.0 -8.0
%  7  8 Wheel velocity [r/s] left, right: 0.0000 -0.0000
% 9    Turnrate [r/s]: -0.0000
% 10 11 12 13 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.544481
% 14    Battery voltage [V]: 12.13
%
%% plot wheel velocity- one sample
figure(7)
data = data_kp005;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,6), 'c');
plot(data(1:n,1), data(1:n,7), 'b');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',12)
grid on
axis([0,0.5,-18,2])
xlabel('Vel step kp0.05 - fast I-term + Lead, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
%% plot wheel velocity- one sample
figure(9)
data = data_kp01;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,6), 'c');
plot(data(1:n,1), data(1:n,7), 'b');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
axis([0,0.5,-18,2])
xlabel('Vel step - fast I-term, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
%% plot wheel velocity- one sample
figure(8)
data = data_kp001;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,6), 'c');
plot(data(1:n,1), data(1:n,7), 'b');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
axis([0,0.5,-18,2])
xlabel('Vel step - faster I-term + Lead, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
%% plot wheel velocity- one sample
figure(95)
data = data_kp045;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,6), 'c');
plot(data(1:n,1), data(1:n,7), 'b');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
axis([0,0.5,-14,2])
xlabel('Vel step kp=0.45 - no I-term fast lead, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
%% plot wheel velocity- one sample
figure(10)
data = data_kp05;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,6), 'c');
plot(data(1:n,1), data(1:n,7), 'b');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
axis([0,0.8,-14,2])
xlabel('Vel step kp=0.5 - no I-term, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
%% plot wheel velocity- one sample
figure(105)
data = data_kp065;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,6), 'c');
plot(data(1:n,1), data(1:n,7), 'b');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
axis([0,0.5,-14,2])
xlabel('Vel step - no I-term kp=0.65+lead=0.1/0.6, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
%% plot wheel velocity- one sample
figure(11)
data = data_kp07;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,6), 'c');
plot(data(1:n,1), data(1:n,7), 'b');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
axis([0,0.8,-14,2])
xlabel('Vel step - no I-term kp=0.7, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
%% plot wheel velocity- one sample
figure(125)
data = data_kp08;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,6), 'c');
plot(data(1:n,1), data(1:n,7), 'b');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
xlabel('Vel step - no I-term kp=0.8+Lead 0.003/0.1, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
