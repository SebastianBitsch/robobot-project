% plot af data dra regbot
clear
close all
%%
data2 = load('mission0Gulv_p2_p4_2.txt');
data4 = load('mission0Gulv_20_30_p2.txt');
data6 = load('mission0Gulv_20_30_pi.txt');
data8 = load('mission0Gulv_20_30_pi_1.txt');
%
%  1    time 0.000 sec
%  2    mission (0) state 2
%  3  4 Motor velocity ref left, right: 20.00 20.00
%  5  6 Motor voltage [V] left, right: 10.9 10.9
%  7  8 Motor current left, right [A]: 0.000 0.000
%  9 10 Wheel velocity [r/s] left, right: 0.0000 0.0000
% 11    Battery voltage [V]: 12.04
% 12 13 Get data time [us]: 560 +ctrl 730
%
%% plot wheel velocity- one sample
figure(50)
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

%%

%% plot vel step PI-Lead [kp = 2}
figure(51)
n = size(data4,1);
hold off
plot(data4(1:n,1), data4(1:n,5), 'b');
hold on
plot(data4(1:n,1), data4(1:n,6), 'm');
plot(data4(1:n,1), data4(1:n,9)*1, 'g');
plot(data4(1:n,1), data4(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
title('Gulv')
xlabel('closed loop 20 r/s to 30 r/s (kp=2), time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
%% plot vel step PI-Lead [kp = 1.27 ti=0.055}
figure(52)
n = size(data6,1);
hold off
plot(data6(1:n,1), data6(1:n,5), 'b');
hold on
plot(data6(1:n,1), data6(1:n,6), 'm');
plot(data6(1:n,1), data6(1:n,9)*1, 'g');
plot(data6(1:n,1), data6(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
title('Gulv')
xlabel('closed loop 20 r/s to 30 r/s (kp=1.27 ti=0.055), time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
%% plot vel step PI-Lead [kp = 0.537 ti=0.034}
figure(53)
n = size(data8,1);
hold off
plot(data8(1:n,1), data8(1:n,5), 'b');
hold on
plot(data8(1:n,1), data8(1:n,6), 'm');
plot(data8(1:n,1), data8(1:n,9)*1, 'g');
plot(data8(1:n,1), data8(1:n,10)*1, 'k');
set(gca,'FontSize',14)
grid on
title('Gulv')
xlabel('closed loop 20 r/s to 30 r/s (kp=0.537 ti=0.034), time in [sec]');
ylabel('rad/s and Volt')
legend('motor V left', 'motor V right', 'wheel left [r/s]', 'wheel rigth [r/s]',2)
