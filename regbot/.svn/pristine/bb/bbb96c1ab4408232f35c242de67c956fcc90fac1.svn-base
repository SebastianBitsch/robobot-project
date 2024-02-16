% edge debug
% clear
close all
%%
%% logfile from robot Ella (8)
% Ella (8)
%  1    time 0.006 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 2, line 0), events 0x1 (bit-flags)
%  6  7 Motor velocity ref left, right: 0.0120 0.0140
%  8  9 Wheel velocity [m/s] left, right: -0.0000 0.0000
% 10 11 12 13 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 0.505324
% 14 .. 35 Line sensor: left -4.180000 0, right 4.180000 0, values 769 902 844 1077 1603 2137 1898 1566, white 1, used 1, LEDhigh=0, xb=0 xw=0 xbc=0 xwc=0 lvl=0 lvr=0
data1 = load('elle_edge_01.txt');
data2 = load('elle_edge_02.txt');
data3 = load('ella_edge_03.txt');
data4 = load('ella_edge_04.txt');
data5 = load('ella_edge_05.txt');
data6 = load('susanne_edge_06.txt');
%%
data = data6;
figure(85)
hold off
plot(data(:,1), data(:,4), 'r');
hold on
plot(data(:,1), data(:,34),'--b');
plot(data(:,1), data(:,33),'b');
plot(data(:,1), data(:,32),'--m');
plot(data(:,1), data(:,31),':m');
plot(data(:,1), data(:,3), ':r');
plot(data(:,1), data(:,14), 'g');
plot(data(:,1), data(:,16), 'y');
grid on
legend('mission','right valid','left valid','crossing white', 'crossing black','thread');
%%
%%
data = data5;
figure(801)
hold off
plot(data(:,1), data(:,18));
hold on
plot(data(:,1), data(:,19));
plot(data(:,1), data(:,20));
plot(data(:,1), data(:,21));
plot(data(:,1), data(:,22));
plot(data(:,1), data(:,23));
plot(data(:,1), data(:,24));
plot(data(:,1), data(:,25));
grid on
legend('1','2','3','4', '5','6','7','8');
