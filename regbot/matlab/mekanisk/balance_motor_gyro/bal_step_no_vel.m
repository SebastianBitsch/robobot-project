% plot af data dra regbot
clear
close all
%%
data_balNoV = load('/home/chr/svnregbot/matlab/rev_2015_10_25/bal_step_no_vel_ctrl.log');
data_balNoVTV8 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_no_vel_ctrl_TV8.log');
data_balNoVTV8I10ms = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_no_vel_ctrl_TV8_I10ms.log');
data_balNoVTV2I10ms = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_no_vel_ctrl_TV2_I10ms.log');
data_balNoVTV4I20ms = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_no_vel_ctrl_TV4_I20ms.log');
data_balNoVTV6I20ms = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_no_vel_ctrl_TV6_I20ms.log');
data_balNoVTV8I20ms = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_no_vel_ctrl_TV6_I20ms.log');
% logfile from robot 8
%  1    time 0.001 sec
%  2    mission (2) state 3
%  3  4 Motor velocity ref left, right: 109.01 79.01
%  5  6 Motor voltage [V] left, right: 8.0 8.0
%  7  8 Wheel velocity [r/s] left, right: 0.0000 0.0000
% 9    Turnrate [r/s]: 0.0000
% 10 11 12 13 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 0.539261
% 14    Battery voltage [V]: 11.79
% 15 16 17 18 19 Ballance pitch ref 0.000000 [rad], balTVref 0.000000 [rad], TV out 94.011734 [m/s], P-lead out 0.000000 [rad], bal-err [rad] -5.392607
%
%% plot tilt
figure(7)
data = data_balNoV;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5)/10, 'b');
hold on
plot(data(1:n,1), data(1:n,6)/10, 'c');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,16), 'g');
set(gca,'FontSize',12)
grid on
axis([0,1.5,-1,1])
xlabel('Balance TV12 - fast I-term, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
%% plot tilt
figure(8)
data = data_balNoVTV8;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5)/10, 'b');
hold on
plot(data(1:n,1), data(1:n,6)/10, 'c');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,16), 'g');
set(gca,'FontSize',12)
grid on
axis([0,1.5,-1,1])
xlabel('Balance TV8 - fast I-term, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
%% plot tilt
figure(9)
data = data_balNoVTV8I10ms;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5)/10, 'b');
hold on
plot(data(1:n,1), data(1:n,6)/10, 'c');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,16), 'g');
set(gca,'FontSize',12)
grid on
axis([0,1.5,-1,1])
xlabel('Balance TV8 - fast I-term 10ms, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
%% plot tilt
figure(10)
data = data_balNoVTV2I10ms;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5)/10, 'b');
hold on
plot(data(1:n,1), data(1:n,6)/10, 'c');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,16), 'g');
set(gca,'FontSize',12)
grid on
axis([0,1.5,-1,1])
xlabel('Balance TV2 - fast I-term 10ms, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
%% plot tilt
figure(12)
data = data_balNoVTV4I20ms;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5)/10, 'b');
hold on
plot(data(1:n,1), data(1:n,6)/10, 'c');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,16), 'g');
set(gca,'FontSize',12)
grid on
axis([0,1.5,-1,1])
xlabel('Balance TV4 - fast I-term 20ms, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
%% plot tilt
figure(14)
data = data_balNoVTV6I20ms;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5)/10, 'b');
hold on
plot(data(1:n,1), data(1:n,6)/10, 'c');
plot(data(1:n,1), data(1:n,3)/100, 'm');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,16), 'g');
set(gca,'FontSize',12)
grid on
axis([0,2.2,-1,1])
xlabel('Balance TV6 - fast I-term 20ms, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'motor vel ref', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
%% plot tilt
figure(16)
data = data_balNoVTV8I20ms;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5)/10, 'b');
hold on
plot(data(1:n,1), data(1:n,6)/10, 'c');
plot(data(1:n,1), data(1:n,3)/100, 'm');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,16), 'g');
set(gca,'FontSize',12)
grid on
axis([0,2.2,-1,1])
xlabel('Balance TV8 - fast I-term 20ms, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'motor vel ref', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
