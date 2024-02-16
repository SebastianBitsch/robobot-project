%% plot_line_sensor log
close all
clear
%%
data1 = load('vel_step.txt');
data2 = load('vel_step_2.txt');
data3 = load('vel_step_-2.txt');
data18 = load('vel_step_18.txt');
data19 = load('vel_step_19.txt');
data20 = load('vel_step_20.txt');
data21 = load('vel_step_21.txt');
%
%  1    time 0.000 sec
%  2    mission (0) state 2
%  3  4 Motor velocity ref left, right: 2.10 2.10
%  5  6 Motor voltage [V] left, right: 2.1 2.1
%  7  8 Motor current left, right [A]: 0.009 -0.002
%  9 10 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 11    Turnrate [r/s]: 0.0000
% 12 13 14 15 Pose x,y,h,tilt [m,m,rad,rad]: 0.0000 0.0000 0.000000 -2.069830
% 16 .. 35 Line sensor: left 0.000000 0, right 0.000000 0, values 25 23 -14 -18 11 5 5 18, white 0, used 0, LEDhigh=1, xb=0 xw=0 xbc=0 xwc=0
% 35    Battery voltage [V]: 12.17

%% plot motor current/velocity
figure(21)
data = data21;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #21')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(22)
data22 = load('vel_step_22.txt');
data = data22;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #22')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(23)
data23 = load('vel_step_23.txt');
data = data23;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #23')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(24)
data24 = load('vel_step_24.txt');
data = data24;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #24')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(26)
data26 = load('vel_step_26.txt');
data = data26;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #26')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(27)
data27 = load('vel_step_27.txt');
data = data27;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #27')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(28)
data28 = load('vel_step_28.txt');
data = data28;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #28')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(29)
data29 = load('vel_step_29.txt');
data = data29;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #29')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(30)
data30 = load('vel_step_30.txt');
data = data30;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #30')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(31)
data31 = load('vel_step_31.txt');
data = data31;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #31')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(32)
data32 = load('vel_step_32.txt');
data = data32;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #32')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(33)
data33 = load('vel_step_33.txt');
data = data33;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #33')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(34)
data34 = load('vel_step_34.txt');
data = data34;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #34')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(35)
data35 = load('vel_step_35.txt');
data = data35;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step #35')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(18)
data = data18;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(19)
data = data19;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(20)
data = data20;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,9), 'b');
hold on
plot(data(1:n,1), data(1:n,10), 'r');
plot(data(1:n,1), data(1:n,7), '.-b');
plot(data(1:n,1), data(1:n,8), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(21)
data = data2;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,17), 'b');
hold on
plot(data(1:n,1), data(1:n,18), 'r');
plot(data(1:n,1), data(1:n,13), '.-b');
plot(data(1:n,1), data(1:n,14), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
%% plot motor current/velocity
figure(22)
data = data3;
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,17), 'b');
hold on
plot(data(1:n,1), data(1:n,18), 'r');
plot(data(1:n,1), data(1:n,13), '.-b');
plot(data(1:n,1), data(1:n,14), '.-r');
set(gca,'FontSize',14)
grid on
title('velocity step  baglæns')
xlabel('[sec]');
ylabel('[m/s] og [A]')
legend('Left vel', 'right vel','left current','right current','Location','SouthEast')
