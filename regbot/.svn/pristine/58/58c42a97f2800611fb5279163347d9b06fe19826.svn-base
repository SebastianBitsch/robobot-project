%% chirp
logInterval = 5;
endFrequency = 1 * 2 * pi;
logRowCnt = 1286;
chirpFrq = 2.0 * pi * 1000.0 / logInterval;
chirpFrqRate = exp(log(endFrequency/chirpFrq)/(logRowCnt));
%% test
low = chirpFrq*chirpFrqRate^(logRowCnt - 1:50:1200)

%% load from log
% Tania (57)
%  1    time 0.009 sec
%  2  3 Motor voltage [V] left, right: 1.84 1.84
%  4  5 Wheel velocity [m/s] left, right: 0.0000 0.0000
%  6  7  8  9 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 3.05368
% 10    Battery voltage [V]: 12.22
% 11 13 read sensor time [ms]: 0.093 and ctrl time 9.756, cycleEnd 0.183
% 14 15 Chirp frequency =417.524 rad/s, phase=0.417524 rad
data100 = load('chirp_000.txt');
data101 = load('chirp_001.txt');
data102 = load('chirp_002.txt');
data103 = load('chirp_003.txt');
data104 = load('chirp_004.txt');
data105 = load('chirp_005.txt');
data106 = load('chirp_006.txt');
data107 = load('chirp_007.txt');
data108 = load('chirp_008.txt');
data109 = load('chirp_009.txt');
data110 = load('chirp_010.txt');
%%
data = data110;
%
figure(1010)
hold off
plot(data(:,1), data(:,16)+0.2,'g');
hold on
plot(data(:,1), data(:,4),'r');
plot(data(:,1), data(:,5),'b');
grid on

%% Tania (57)
%  1    time 0.010 sec
%  2  3 Motor voltage [V] left, right: 4.75 4.75
%  4  5 Wheel velocity [m/s] left, right: 0.0000 -0.0000
%  6    Battery voltage [V]: 11.90
%  7  9 Chirp frequency =1256.1 rad/s, phase=1.2561 rad, value=0.95089
data100 = load('chirp_100.txt');
data101 = load('chirp_101.txt');
data102 = load('chirp_102.txt');
data103 = load('chirp_103.txt');
%%
data = data103;
%
figure(103)
hold off
plot(data(:,1), data(:,9),'g');
hold on
plot(data(:,1), data(:,4),'r');
plot(data(:,1), data(:,5),'b');
grid on
