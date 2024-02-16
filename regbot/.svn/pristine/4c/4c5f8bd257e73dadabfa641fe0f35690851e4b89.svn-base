%% plot_line_sensor log
close all
clear
%%
data1 = load('balance_line_1.txt');
data2 = load('balance_line_2.txt');
data3 = load('balance_line_3.txt');
%
%% logfile from robot 20
%  1    time 1.634 sec
%  2    mission (3) line 1
%  3  4 Wheel velocity [m/s] left, right: 0.5020 0.5149
%  5  6  7  8 Pose x,y,h,tilt [m,m,rad,rad]: 0.2052 -0.0019 -0.015547 0.537604
%  9 .. 23 Line sensor: left -4.180000 0, right -0.195887 1, values 1082 1191 1159 1100 926 767 708 619, white 1, used 1, LEDhigh=0
% 25    Battery voltage [V]: 12.14

%% plot motor current/velocity
figure(20)
data = data1;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,11), 'r');
plot(data(1:n,1), data(1:n,10), '.-b');
plot(data(1:n,1), data(1:n,12), '.-r');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
title('linesensor position')
xlabel('[sec]');
ylabel('cm')
legend('Left', 'right','left valid','right valid','tilt','Location','SouthEast')
%%
%  1    time 3.031 sec
%  2    mission (3) line 3
%  3  4 Motor velocity ref left, right: 0.21 0.26
%  5  6 Motor voltage [V] left, right: 1.3 1.1
%  7  8 Wheel velocity [m/s] left, right: 0.1450 0.1988
%  9 10 11 12 Pose x,y,h,tilt [m,m,rad,rad]: 0.6130 -0.0093 -0.020729 0.042940
% 13 .. 27 Line sensor: left -4.180000 0, right -0.198689 1, values 538 628 569 584 548 439 404 337, white 1, used 1, LEDhigh=0
% 29    Battery voltage [V]: 12.34
%% plot motor current/velocity
figure(21)
data = data2;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,13), 'b');
hold on
plot(data(1:n,1), data(1:n,15), 'r');
plot(data(1:n,1), data(1:n,14), '.-b');
plot(data(1:n,1), data(1:n,16), '.-r');
plot(data(1:n,1), data(1:n,12), 'g');
set(gca,'FontSize',14)
grid on
title('linesensor position')
xlabel('[sec]');
ylabel('cm')
legend('Left', 'right','left valid','right valid','tilt','Location','SouthEast')
%%
%  1    time 1.789 sec
%  2    mission (3) line 2
%  3  4 Wheel velocity [m/s] left, right: 0.2731 0.2462
%  5  6  7  8 Pose x,y,h,tilt [m,m,rad,rad]: 0.2836 -0.0055 -0.002591 -0.020486
%  9 .. 28 Line sensor: left -4.180000 0, right -0.780408 1, values 444 509 443 428 394 360 354 294, white 1, used 1, LEDhigh=0, xb=0 xw=0 xbc=0 xwc=% 28    Battery voltage [V]: 12.30
%%
%% plot motor current/velocity
figure(22)
data = data3;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,11), 'r');
plot(data(1:n,1), data(1:n,10), '.-b');
plot(data(1:n,1), data(1:n,12), '.-r');
plot(data(1:n,1), data(1:n,8), 'g');
set(gca,'FontSize',14)
grid on
title('linesensor position')
xlabel('[sec]');
ylabel('cm')
legend('Left', 'right','left valid','right valid','tilt','Location','SouthEast')
