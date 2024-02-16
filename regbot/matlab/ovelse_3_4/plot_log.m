%% plot af data fra regbot
clear
close all
%%
data = load('log3.txt');
%
%  1    time 0.000 sec
%  2  3 Motor voltage [V] left, right: 2.0 2.0
%  4  5 Motor current left, right [A]: 0.006 0.006
%  6  7 Wheel velocity [r/s] left, right: 0.0000 0.0000
%  8    Battery voltage [V]: 12.17
%
% plot wheel velocity- one sample
figure(1)
n = size(data,1) / 2;
hold off
plot(data(1:n,1), data(1:n,2), 'b');
hold on
plot(data(1:n,1), data(1:n,4)*2, 'c');
plot(data(1:n,1), data(1:n,5)*2, 'g');
plot(data(1:n,1), data(1:n,6)*0.1, 'b');
plot(data(1:n,1), data(1:n,7)*0.1, 'm');
grid on
xlabel('time in sec');
%ylabel('rad/s and Volt')
legend('motor left V', 'current left *2 [A]',  'current right *2 [A]', 'wheel left *0.1 [rad/s]', 'wheel right *0.1 [rad/s]','Location', 'NorthWest')
