% plot af data dra regbot
clear
close all
%%
data1 = load('current_sensor_idle.txt');
data2 = load('current_sensor_idle2.txt');
data5 = load('current_sensor_3v_4v_1.txt');
data6 = load('current_sensor_2v_3v_2.txt');
data7 = load('current_sensor_2v_3v_3.txt');
data8 = load('current_sensor_2v_3v_4.txt');
data9 = load('current_sensor_2v_3v_5.txt');
data10 = (data6+data7+data8+data9)/4;
data11 = load('current_sensor_2v_3v_8.txt');
%%
data15 = load('current_sensor_3v_6v_1.txt');
data16 = load('current_sensor_-3v_6v_2.txt');
data17 = load('current_sensor_30rs_60rs_3.txt');
data18 = load('current_sensor_20rs_30rs_3.txt');
data19 = load('current_sensor_15rs_20rs_3.txt');

%  1    time 0.000 sec
%  2    mission (0) state 2
%  3  4 Motor velocity ref left, right: 2.00 2.00
%  5  6 Motor voltage [V] left, right: 2.0 2.0
%  7 10 Motor current left, right [A]: 0.002 -0.002 671 668
% 11 12 Encoder left, right: 0 0
% 13 14 Wheel velocity [r/s] left, right: 0.0000 0.0000
% 15    Battery voltage [V]: 3.15
%
%% plot motor current - idle no filter
figure(1)
n = size(data1,1);
hold off
plot(data1(1:n,1), (data1(1:n,9) - data1(1,10)) * 1.2/1024/.185, 'b');
hold on
plot(data1(1:n,1), (data1(1:n,10) - data1(1,10)) * 1.2/1024/.185, 'c');
set(gca,'FontSize',14)
grid on

xlabel('open loop step 2 to -2V, time in [sec]');
ylabel('rad/s and Volt')
legend('current left', 'current right',2)
%% plot motor current - no filter - 3v-4v open loop
figure(2)
n = size(data5,1);
hold off
plot(data5(1:n,1), (data5(1:n,9) - data5(1,9)) * 1.2/1024/.185, 'b');
hold on
plot(data5(1:n,1), -(data5(1:n,10) - data5(1,10)) * 1.2/1024/.185, 'c');
set(gca,'FontSize',14)
grid on
xlabel('open loop step 3 to 4V, time in [sec]');
ylabel('rad/s and Volt')
legend('current left', 'current right',2)
%% power spektrum af strømmåling
Fs = 1000; % sample frequency
n = size(data5,1)
x = data5(1:n,9) - data5(1,9);
x = data5(1:n,10) - data5(1,10);
xdft = fft(x);
xdft = xdft(1:n/2+1);
psdx = (1/(Fs*n)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:Fs/length(x):Fs/2;
figure(20)
plot(freq,10*log10(psdx))
grid on
title('Periodogram Using FFT')
xlabel('Frequency (Hz)')
ylabel('Power/Frequency (dB/Hz)')

%% plot motor current - 220nF filter - 2v-3v open loop
figure(3)
n = size(data6,1);
hold off
offset9 = mean(data11(:,9));
offset10 = mean(data11(:,10));
plot(data6(1:n,1), (data6(1:n,9) - offset9) * 1.2/1024/.185, 'b');
hold on
plot(data6(1:n,1), -(data6(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data7(1:n,1), -(data7(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data8(1:n,1), -(data8(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data9(1:n,1), -(data9(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data10(1:n,1), (data10(1:n,9) - offset9) * 1.2/1024/.185, 'r');
plot(data10(1:n,1), -(data10(1:n,10) - offset10) * 1.2/1024/.185, 'k');
set(gca,'FontSize',14)
grid on
xlabel('open loop step 2 to 3V, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V', 'current left (x100)', 'current right (x100)', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
%% plot motor current - 220nF filter - 3v-6v open loop
figure(4)
n = size(data6,1);
hold off
offset9 = mean(data11(:,9));
offset10 = mean(data11(:,10));
plot(data15(1:n,1), (data15(1:n,9) - offset9) * 1.2/1024/.185, 'b');
hold on
plot(data15(1:n,1), -(data15(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data15(1:n,1), data15(1:n,15)/10, 'k');
set(gca,'FontSize',14)
grid on
xlabel('open loop step 3 to 6V, time in [sec]');
ylabel('rad/s and Volt')
legend('current left', 'current right','batt voltage/10',2)
%% plot motor current - 220nF filter - 3v-6v open loop - and velocity
figure(4)
n = size(data16,1);
hold off
offset9 = mean(data11(:,9));
offset10 = mean(data11(:,10));
plot(data16(1:n,1), (data16(1:n,9) - offset9) * 1.2/1024/.185, 'b');
hold on
plot(data16(1:n,1), -(data16(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data16(1:n,1), data16(1:n,15)/10, 'k');
plot(data16(1:n,1), data16(1:n,13)/10, 'r');
plot(data16(1:n,1), data16(1:n,14)/10, 'g');
set(gca,'FontSize',14)
grid on
xlabel('open loop step -3 to 6V, time in [sec]');
ylabel('rad/s and Volt')
legend('current left', 'current right','batt voltage/10','vel-left/10','vel-right/10',2)
%% plot motor current - 220nF filter - 3v-6v open loop - and velocity
figure(5)
n = size(data17,1);
hold off
offset9 = mean(data11(:,9));
offset10 = mean(data11(:,10));
plot(data17(1:n,1), (data17(1:n,9) - offset9) * 1.2/1024/.185, 'b');
hold on
plot(data17(1:n,1), -(data17(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data17(1:n,1), data17(1:n,15)/10, 'k');
plot(data17(1:n,1), data17(1:n,13)/10, 'r');
plot(data17(1:n,1), data17(1:n,14)/10, 'g');
set(gca,'FontSize',14)
grid on
xlabel('closed loop step 30r/s to 60r/s, time in [sec]');
ylabel('rad/s and Volt')
legend('current left', 'current right','batt voltage/10','vel-left','vel-right',2)
%% plot motor current - 220nF filter - designed for not on ground
figure(6)
n = size(data17,1);
hold off
offset9 = mean(data11(:,9));
offset10 = mean(data11(:,10));
plot(data17(1:n,1), (data17(1:n,9) - offset9) * 1.2/1024/.185, 'b');
hold on
plot(data17(1:n,1), -(data17(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data17(1:n,1), data17(1:n,15)/10, 'k');
plot(data17(1:n,1), data17(1:n,13)/10, 'r');
plot(data17(1:n,1), data17(1:n,14)/10, 'g');
set(gca,'FontSize',14)
grid on
xlabel('closed loop step 20r/s to 30r/s, time in [sec]');
ylabel('rad/s and Volt')
legend('current left', 'current right','batt voltage/10','vel-left','vel-right',2)
%% plot motor current - 220nF filter - designed for not on ground
figure(7)
n = size(data17,1);
hold off
offset9 = mean(data11(:,9));
offset10 = mean(data11(:,10));
plot(data18(1:n,1), (data18(1:n,9) - offset9) * 1.2/1024/.185, 'b');
hold on
plot(data18(1:n,1), -(data18(1:n,10) - offset10) * 1.2/1024/.185, 'c');
plot(data18(1:n,1), data18(1:n,15)/10, 'k');
plot(data18(1:n,1), data18(1:n,13)/10, 'r');
plot(data18(1:n,1), data18(1:n,14)/10, 'g');
set(gca,'FontSize',14)
grid on
xlabel('closed loop step 15r/s to 20r/s, time in [sec]');
ylabel('rad/s and Volt')
legend('current left', 'current right','batt voltage/10','vel-left/10','vel-right/10',2)
