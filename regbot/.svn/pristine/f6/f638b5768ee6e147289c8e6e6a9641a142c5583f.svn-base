% plot af data dra regbot
clear
close all
%%
data = load('mission0_p2_p4.txt');
data1 = load('mission0_p2_p4_1.txt');
data2 = load('mission0_p6_p4_1.txt');
%% closed loop
data3 = load('mission0R_p6_p14_1.txt');
data4 = load('mission0R_p10_p20_1.txt');
data5 = load('mission0R_p10_p20_2.txt');
data6 = load('mission0R_p10_p20_3.txt');
%%
data7 = load('mission0R_p10_p20_4.txt');
data8 = load('mission0R_p10_p20_5.txt');
data9 = load('mission0R_p10_p20_6.txt');
data10 = load('mission0R_p20_p30_1.txt');
data11 = load('mission0R_p20_p30_2.txt');
data12 = load('mission0R_p20_p30_3.txt');
%
%  0    time 0.000 sec
%  1    mission (0) state 2
%  2  3 Motor velocity ref left, right: 2.00 2.00
%  4  5 Motor voltage [V] left, right: 2.0 2.0
%  6  7 Motor current left, right [A]: 0.002 0.002
%  8  9 Wheel velocity [r/s] left, right: -0.0041 0.0043
% 10    Turnrate [r/s]: -0.0003
% 11    Battery voltage [V]: 12.12
% 12 13 Get data time [us]: 560 +ctrl 840
%
%% plot wheel velocity- one sample
figure(1)
n = size(data,1);
hold off
plot(data(1:n,1), data(1:n,5), 'b');
hold on
plot(data(1:n,1), data(1:n,7)*100, 'c');
plot(data(1:n,1), data(1:n,8)*100, 'b');
plot(data(1:n,1), data(1:n,9)*1, 'g');
plot(data(1:n,1), data(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
xlabel('open loop step 2 to -2V, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V', 'current left (x100)', 'current right (x100)', 'wheel left [r/s]', 'wheel rigth [r/s]','Location','NorthWest')
%% plot wheel velocity- one sample
figure(2)
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,5), 'b');
hold on
plot(data1(1:n,1), data1(1:n,7)*100, 'c');
plot(data1(1:n,1), data1(1:n,8)*100, 'b');
plot(data1(1:n,1), data1(1:n,9)*1, 'g');
plot(data1(1:n,1), data1(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
xlabel('open loop step 2V to 4V [sec]');
ylabel('rad/s and Volt')
legend('motor V', 'current left (x100)', 'current right (x100)', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
%% plot wheel velocity- one sample
figure(3)
n = size(data2,1);
hold off
plot(data2(1:n,1), data2(1:n,5), 'b');
hold on
plot(data2(1:n,1), data2(1:n,7)*100, 'c');
plot(data2(1:n,1), data2(1:n,8)*100, 'b');
plot(data2(1:n,1), data2(1:n,9)*1, 'g');
plot(data2(1:n,1), data2(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
xlabel('open loop 6V to 4V, time in [sec]');
ylabel('rad/s and Volt')
legend('motor V', 'current left (x100)', 'current right (x100)', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
%% plot wheel velocity- closed loop [Kp=0.68 ti=0.016 limit=2.5 td=0]
figure(10)
n = size(data3,1);
hold off
plot(data3(1:n,1), data3(1:n,5), 'b');
hold on
plot(data3(1:n,1), data3(1:n,6), 'm');
% plot(data3(1:n,1), data3(1:n,7)*100, 'c');
% plot(data3(1:n,1), data3(1:n,8)*100, 'b');
plot(data3(1:n,1), data3(1:n,9)*1, 'g');
plot(data3(1:n,1), data3(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
xlabel('closed loop 6 r/s to 14 r/s (kp=0.68, ti=0.016), time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
%% plot wheel velocity- closed loop [Kp=0.68 ti=0.016 limit=5 td=0]
figure(11)
n = size(data4,1);
m = size(data5,1);
k = size(data6,1);
w = size(data7,1);
hold off
plot(data9(1:n,1), data9(1:n,5), 'c');
hold on
plot(data9(1:n,1), data9(1:n,6), 'm');
% plot(data4(1:n,1), data4(1:n,7)*100, 'c');
% plot(data4(1:n,1), data4(1:n,8)*100, 'b');
plot(data4(1:n,1), data4(1:n,9)*1, 'g');
plot(data4(1:n,1), data4(1:n,10)*1, 'y');
% plot(data5(1:m,1), data5(1:m,10)*1, 'r');
% plot(data6(1:k,1), data6(1:k,10)*1, 'b');
% plot(data7(1:w,1), data7(1:w,10)*1, 'c');
plot(data8(1:w,1), data8(1:w,10)*1, 'k');
plot(data9(1:w,1), data9(1:w,10)*1, 'r');
set(gca,'FontSize',14)
grid on
xlabel('Closed loop 10r/s to 20r/s (kp=0.68, ti=0.016), time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]', 'kp=05', 'kp=0.1', 'kp=0.56,ti=0.0267',2)
%% plot wheel velocity- closed loop [Kp=0.56 ti=0 td=0]
figure(13)
n = size(data10,1);
hold off
plot(data10(1:n,1), data10(1:n,5), 'b');
hold on
plot(data10(1:n,1), data10(1:n,6), 'm');
% plot(data3(1:n,1), data3(1:n,7)*100, 'c');
% plot(data3(1:n,1), data3(1:n,8)*100, 'b');
plot(data10(1:n,1), data10(1:n,9)*1, 'g');
plot(data10(1:n,1), data10(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
xlabel('closed loop 20 r/s to 30 r/s (kp=0.56, ti=0), time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
%% plot wheel velocity- closed loop [Kp=0.56 ti=0 td=0]
figure(14)
n = size(data11,1);
hold off
plot(data11(1:n,1), data11(1:n,5), 'b');
hold on
plot(data11(1:n,1), data11(1:n,6), 'm');
% plot(data3(1:n,1), data3(1:n,7)*100, 'c');
% plot(data3(1:n,1), data3(1:n,8)*100, 'b');
plot(data11(1:n,1), data11(1:n,9)*1, 'g');
plot(data11(1:n,1), data11(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
xlabel('closed loop 20 r/s to 30 r/s (kp=0.537, ti=0.034), time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
%% plot vel step PI-Lead [kp = 0.54, ti=0.021, td=0.0092, alpha=0.3]
figure(15)
n = size(data12,1);
hold off
plot(data11(1:n,1), data12(1:n,5), 'b');
hold on
plot(data12(1:n,1), data12(1:n,6), 'm');
% plot(data3(1:n,1), data3(1:n,7)*100, 'c');
% plot(data3(1:n,1), data3(1:n,8)*100, 'b');
plot(data12(1:n,1), data12(1:n,9)*1, 'g');
plot(data12(1:n,1), data12(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
xlabel('closed loop 20 r/s to 30 r/s (kp=0.54, ti=0.021, td=0.0092, al=0.3), time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]',2)