% edge debug
clear
close all
%%
% Susanne (58)
%  1    time 0.003 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 0), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: 0.0000 0.0000
%  8  9 Motor voltage [V] left, right: 0.0 0.0
% 10 11 12 13 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 0.441798
% 14 .. 34 Line sensor: left 0.000000 0, right 0.000000 0, values 1114 1234 1316 1424 1423 1338 1124 1010, white 1, used 1, LEDhigh=0, xb=0 xw=0 xbc=0 xwc=0 lvl=0 lvr=0
% 35    Battery voltage [V]: 11.46
% 36 45 ctrl edge, ref=0, m=1.06452, m2=1.06452, uf=0, r2=0, ep=-0.425808,up=-0.143548, ui=0, u1=-0.143548, u=-0.143548
data6 = load('susanne_edge_06.txt');
data7 = load('susanne_edge_07.txt');
data8 = load('susanne_edge_08.txt');
data9 = load('susanne_edge_09.txt');
%%
data = data9;
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
plot(data(:,1), data(:,37), '--r');
grid on
axis([0,2.5,-5,21])
legend('mission','r valid','l valid','x white', 'x black','thread','le','re','measured');
%%
%%
data = data9;
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
