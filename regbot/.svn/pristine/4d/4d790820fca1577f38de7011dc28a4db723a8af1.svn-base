% edge debug
% clear
close all
%%
%% logfile from robot Ella (8)
% Liv (36)
%  1    time 0.011 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 0), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: 0.3028 0.2976
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: 0.00121704 5.30893e-07 0 0.128577
% 12 .. 32 Edge sensor: left -4.180000 0, right 1.722882 0, values 2387 2992 3016 2890 2273 2263 2057 1645, white 1, used 1, LEDhigh=1, xb=0 xw=0 xbc=0 xwc=0 lvl=0 lvr=0       whitelevel=[1464 1863 1969 2045 1757 1902 1713 1366]       blacklevel=[86 89 95 102 87 90 81 80]
% 33 34 Distance sensor [m]: 0.047 0.047
% 35    Battery voltage [V]: 12.07
data1 = load('liv_edge_01.txt');
data2 = load('liv_edge_02.txt');
data3 = load('liv_edge_03.txt');
data4 = load('liv_edge_04.txt');
data5 = load('liv_edge_05.txt');
data6 = load('liv_edge_06.txt');
data8 = load('liv_edge_08.txt');
data10 = load('liv_edge_10.txt');
data11 = load('liv_edge_11.txt');
data12 = load('liv_edge_12.txt');
data13 = load('liv_edge_13.txt');
data14 = load('liv_edge_14.txt');
%%
data = data14;
figure(94)
hold off
plot(data(:,1), data(:,4), ':r');
hold on
plot(data(:,1), data(:,12), 'r');
plot(data(:,1), data(:,13), 'b');
plot(data(:,1), data(:,10), 'g');
grid on
legend('mission','right pos','right valid','heading','location','southwest');
%%
data = data8;
%close(807)
figure(808)
hold off
%yyaxis right
plot(data(:,1), data(:,11));
hold on
plot(data(:,1), data(:,11));
plot(data(:,1), data(:,12));
plot(data(:,1), data(:,13));
plot(data(:,1), data(:,24),'g');
yyaxis right
plot(data(:,1), data(:,16),'m');
plot(data(:,1), data(:,17),'c');
plot(data(:,1), data(:,18));
plot(data(:,1), data(:,19));
plot(data(:,1), data(:,20));
plot(data(:,1), data(:,21));
plot(data(:,1), data(:,22));
plot(data(:,1), data(:,23));
grid on
%legend('mission','edge','valid','1','2','3','4', '5','6','7','8','white');
legend('tilt','white','1','2','3','4', '5','6','7','8');
