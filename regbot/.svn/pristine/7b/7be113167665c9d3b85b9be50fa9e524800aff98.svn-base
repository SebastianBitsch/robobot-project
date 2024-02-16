% plot af data dra regbot
clear
close all
%%
data_balVTV8I20ms = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_vel_ctrl_TV8_I20ms.log');
data_balVTV8I20msv002 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_vel_ctrl_TV8_I20ms_v002.log');
data_balVTV8I20msv003 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_vel_ctrl_TV8_I20ms_v003.log');
data_balVTV8I20msv003 = load('/home/chr/svnregbot/matlab/rev_2015_10/bal/bal_step_vel_ctrl_TV8_I20ms_v004.log');
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
figure(16)
data = data_balVTV8I20ms;
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
%axis([0,2.2,-2,2])
xlabel('Balance TV8 - fast I-term 20ms, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'motor vel ref', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
%% plot tilt
figure(17)
data = data_balVTV8I20msv002;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5)/10, 'b');
hold on
plot(data(1:n,1), data(1:n,6)/10, 'c');
plot(data(1:n,1), data(1:n,3)/100, 'm');
plot(data(1:n,1), data(1:n,13), 'r');
plot(data(1:n,1), data(1:n,16), 'g');
plot(data(1:n,1), data(1:n,15), 'k');
set(gca,'FontSize',12)
grid on
axis([0,3.2,-1,1])
xlabel('Balance TV8 I20ms Kpv=0.02, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'motor vel ref', 'pitch [rad]', 'balTVref [r/s]','Pitch ref','Location','NorthEast')
%% plot tilt
figure(18)
data = data_balVTV8I20msv003;
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
axis([0,3.2,-1,1])
xlabel('Balance TV8 I20ms Kpv=0.03, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'motor vel ref', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
%% plot tilt
figure(18)
data = data_balVTV8I20msv004;
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
axis([0,3.2,-1,1])
xlabel('Balance TV8 I20ms Kpv=0.04, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V/10 left', 'motor V/10 right', 'motor vel ref', 'pitch [rad]', 'balTVref [r/s]','Location','NorthEast')
