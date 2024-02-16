%% plot barometer reading
clear
close all
%%
% Liv (36)
%  1    time 0.000 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 0), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: 0.2000 0.2000
%  8  9 Motor voltage [V] left, right: 3.60 3.60
% 10 11 Motor current left, right [A]: -0.100 -0.073
% 12 13 Encoder left, right: 90279 95805
% 14 15 Wheel velocity [m/s] left, right: -0.0000 0.0000
% 16 17 Get data time [us]: 100 +ctrl 440
% fra 03
% Liv (36)
%  1    time 0.000 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 0), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: 0.2000 0.2000
%  8  9 Motor voltage [V] left, right: 2.82 2.82
% 10 11 Motor current left, right [A]: -0.079 -0.088
% 12 13 Wheel velocity [m/s] left, right: 0.0000 -0.0000
% 14 15 Get data time [us]: 100 +ctrl 400
data_01 = load('liv_enc_step.txt');
data_02 = load('liv_enc_step_02.txt');
data_03 = load('liv_enc_step_03.txt');
data_04 = load('liv_enc_step_04.txt');
data_05 = load('liv_enc_step_05.txt');
data_06 = load('liv_enc_step_06.txt');
data_07 = load('liv_enc_step_07.txt');
data_10 = load('liv_enc_step_10.txt');
data_11 = load('liv_enc_step_11.txt');
data_15 = load('liv_enc_step_15.txt');
data_16 = load('liv_enc_step_16.txt');
data_17 = load('liv_enc_step_17.txt');
data_18 = load('liv_enc_step_18.txt');
data_20 = load('liv_enc_step_20.txt'); % 0.2 to 2.5 m/s calib
data_21 = load('liv_enc_step_21.txt'); % 0.2 to 2.5 m/s not calib
data_24 = load('liv_enc_step_24.txt'); % slow kp=15 ti=0.05 ff=2
data_25 = load('liv_enc_step_25.txt'); % slow ff=4.5 only
%%
data = data_20; %06
figure(250)
hold off
plot(data(:,1), data(:,4)/10,'--r','linewidth',2);
hold on
plot(data(:,1), data(:,6), 'linewidth',2);
plot(data(:,1), data(:,12),'linewidth',3);
plot(data(:,1), data(:,13), 'linewidth',3);
plot(data(:,1), data(:,8)/10, 'linewidth',2);
grid on
grid MINOR
axis([0,0.55,0,1.6])
%
xlabel('time [seconds]')
ylabel('velocity [m/s]')
legend('mission/10','vel ref','left vel', 'right vel','left voltage/10','location','northwest');
%%
% 1 motoromdrejning i afstand
d1r = 2*pi/9.68*0.03
% time for 1 rot ved 20cm/s
1/(0.2/d1r)