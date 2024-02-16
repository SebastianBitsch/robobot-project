% edge debug
clear
close all
%%
% Alma (11)
%  1    time 2.007 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 1), events 0x0 (bit-flags)
%  6  7 Wheel velocity [m/s] left, right: 0.0119 0.0086
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: 0.0138559 0.000210471 0.00777337 -0.0148407
% 12 13 read sensor time [us]: 100 and ctrl time 420
% 14 23 ctrl bal pos, ref=0.5, m=0.013856, m2=0.013856, uf=0, r2=0.5, ep=0.486144,up=0.486144, ui=0, u1=0.486144, u=0.486144
data002 = load('bal_pos_002.txt');
data003 = load('bal_pos_003.txt'); % p=2
data004 = load('bal_pos_004.txt'); % p=3 lead tau=1 alfa=0.2
data005 = load('bal_pos_005.txt'); % p=2 lead tau=0.5 alfa=0.4
data006 = load('bal_pos_006.txt'); % p=2.5 lead tau=0.3 alfa=0.1
data007 = load('bal_pos_007.txt'); % p=2.5 lead tau=0.5 alfa=0.1
data008 = load('bal_pos_008.txt'); % p=2.2 lead tau=0.4 alfa=0.2
data009 = load('bal_pos_009.txt'); % p=2 lead tau=0.35 alfa=0.2
data010 = load('bal_pos_010.txt'); % p=2 lead tau=0.25 alfa=0.2
data011 = load('bal_pos_011.txt'); % p=1.8 lead tau=0.25 alfa=0.2
data012 = load('bal_pos_012.txt'); % p=1.8 lead tau=0.3 alfa=0.15
data013 = load('bal_pos_013.txt'); % p=1.8 lead tau=0.25 alfa=0.2, lag (s+1)/(2s+1)
data014 = load('bal_pos_014.txt'); % p=1.8 lead tau=0.4 alfa=0.125, lag (s+1)/(2s+1)
data015 = load('bal_pos_015.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1)
data016 = load('bal_pos_016.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 0.4m/s
data017 = load('bal_pos_017.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 0.8m/s
data018 = load('bal_pos_018.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 1m/s
data101 = load('bal_pos_101.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 1m/s
data102 = load('bal_pos_102.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 1.2m/s
data103 = load('bal_pos_103.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 1.2m/s (2.5m)
data104 = load('bal_pos_104.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 1m/s (2.5m)
data105 = load('bal_pos_105.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 0.9m/s, 2.5m
data106 = load('bal_pos_106.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 0.4m/s, 2.5m
data107 = load('bal_pos_107.txt'); % p=1.8 lead tau=0.5 alfa=0.1, lag (s+1)/(2s+1), 1.2m/s, 2.5m

%%
dd = data107;
fg = 107;
%
figure(fg)
hold off
plot(dd(:,1), dd(:,4)/10,'k')
grid on
hold on
plot(dd(:,1), dd(:,7),'b')
plot(dd(:,1), dd(:,8),'g')
plot(dd(:,1), dd(:,14),'r')
plot(dd(:,1), dd(:,15),'--r')
plot(dd(:,1), dd(:,23),'--b')

legend('mission','velocity - right','pos-x','ref','measured','u');
xlabel('seconds')
