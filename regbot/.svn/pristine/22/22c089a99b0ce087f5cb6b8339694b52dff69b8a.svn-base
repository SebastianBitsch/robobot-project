% Model System parameters - fra kørsel på gulv
clear
close all
RA = 3.3;   % stall 6V/2.2A; % from spec
            % - measured to between 2.4 and 3.0 Ohm
            % + Zth i driver (og strømforsyning)
NG = 9.68;  % gear omsætning
%
%% conclusion form plot
V_A = 4;   % ankerspænding [V] - measured SS
I_A = 0.06; % ankerstrøm steady state [A] - measured SS
W_A = 36.0;  % omdrejningshastighed aksel [rad/s] - measured SS
W_M = W_A * NG;
RPM4V = W_M/2/pi*60; % skal være ca. 3000 RPM
V_Rss = RA*I_A; % spænding over ankermodstand
Vemf = V_A-V_Rss; % beregnet elektromotorisk spænding
Kemf = Vemf/W_M; % beregnet motor-konstant
Ktau = Kemf;
TauAss = I_A*Ktau; % moment af anker steady state
bM = TauAss/W_M;    % dynamisk friktion
%%
% beregning af dynamiske poler
tau1 = 0.003; % effekt af ankerspole
tau2 = 0.160; % effekt af motor inerti
L = tau1*RA;  % induktion i anker
JA = tau2*(RA*bM+Kemf*Ktau)/RA;
%%
Rw = 0.06/2; % hjul radius
B  =  0.155; % Base - afstand mellem hjul
%% open loop motor model med gear
% motor model (inc Inerti)
GA = tf([Ktau],[L RA]);
GI = tf([1],[JA bM]);
GM = minreal(GA * GI / (1 + GA * GI * Kemf)/NG);
%% plot motor open loop bodeplot
figure(40)
margin(GM);
xlabel('Motor open loop med gear')
%% discrete model of motor - also L
DM2 = c2d(GM, 0.001, 'tustin')
%% discrete simplified model of motor - no L
GI0 = GI*Ktau/RA
GM0 = minreal(GI0 / (1 + GI0 * Kemf)/NG)
DM1 = c2d(GM0,0.001,'tustin')
[mn,md] = tfdata(DM1);
mnn=mn{1}
mdd=md{1}
%%
%% design PID - 2ms delay
% X ms delay
[Gdn,Gdd] = pade(0.002,4);
Gd = tf(Gdn, Gdd);
% plot open loop with delay
figure(41)
hold off
margin(GM*Gd)
hold on
margin(GM)
grid on
%% simuleret Pregulator
kp = 5.0 % ud fra graf ved 109 rad/s
%Golp = kp*Gd*GM;
Golp = kp*GM;
figure(42)
hold off
bode(Golp)
grid on
% closed loop
Gclp = Golp/(1+Golp)
hold on
bode(Gclp)
title('Gulv: P-regulator, kp=X')
legend('open-loop','closed loop')
bwp = bandwidth(Gclp)
%% step responce
figure(43)
step(Gclp)
title('Gulv: Step responce P=2')
grid on

%% regulator design - PI
Ni = 4;
pm = 70;
[sn sd] = tfdata(GM*Gd);
[wc, ti, kp] = solvepi(sn{1}, sd{1}, Ni, pm)
Gci = tf([ti 1],[ti 0]);
Gol = kp*Gci*GM*Gd;
% open loop plot
figure(44)
hold off
bode(Gol)
title('Open loop with PI regulator')
grid on
% closed loop
Gcl = Gol/(1+Gol);
hold on
bode(Gcl)
legend('open-loop','closed loop')
BWPI = bandwidth(Gcl)
%% step responce
figure(45)
step(Gcl)
title('Step responce PI regulator')
grid on

%% logged step responce with acc limit (5m/s 2)
data1 = load('v3velstep.txt');
%  1    time 0.001 sec
%  2    mission (0) state 2
%  3  4 Motor velocity ref left, right: 3.50 3.50
%  5  6 Motor voltage [V] left, right: 2.9 2.9
%  7  8 Motor current left, right [A]: 0.171 -0.082
%  9 10 Wheel velocity [r/s] left, right: 0.0000 0.0000
% 11    Turnrate [r/s]: 0.0000
% 12 13 14 Pose x,y,h [m,m,rad]: 0.0000 0.0000 0.000000
% 15    Battery voltage [V]: 12.29
% 16 17 Get data time [us]: 560 +ctrl 730figure(50)
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,3), 'b');
hold on
plot(data1(1:n,1), data1(1:n,4), 'r');
plot(data1(1:n,1), data1(1:n,5), 'c');
plot(data1(1:n,1), data1(1:n,6), 'm');
plot(data1(1:n,1), data1(1:n,9), 'y');
plot(data1(1:n,1), data1(1:n,10), 'g');
set(gca,'FontSize',14)
grid on
title('Velocity step')
xlabel('time in [sec]');
ylabel('Value')
legend('Motor ref left', 'motor ref right','motor voltage left', 'motor voltage right','wheel vel left','wheel vel right',2)
%
%% step with acc limit og 5m/s^2
data1 = load('v4turnstep.txt');
%  1    time 0.000 sec
%  2    mission (0) state 2
%  3  4 Motor velocity ref left, right: 0.17 0.17
%  5  6 Motor voltage [V] left, right: 0.8 0.8
%  7  8 Motor current left, right [A]: 0.000 -0.481
%  9 10 Wheel velocity [r/s] left, right: 0.0000 0.0000
% 11    Turnrate [r/s]: 0.0000
% 12 13 14 Pose x,y,h [m,m,rad]: 0.0000 0.0000 0.000000
% 15    Battery voltage [V]: 12.26
% 16 17 Get data time [us]: 560 +ctrl 860
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,3), 'b');
hold on
%plot(data1(1:n,1), data1(1:n,4), 'r');
plot(data1(1:n,1), data1(1:n,11), 'c');
plot(data1(1:n,1), data1(1:n,14)*10, 'm');
plot(data1(1:n,1), data1(1:n,9), 'y');
plot(data1(1:n,1), data1(1:n,10), 'g');
set(gca,'FontSize',14)
grid on
title('Turn step')
xlabel('time in [sec]');
ylabel('Value')
legend('Motor ref left', 'Turnrate * 10', 'Heading * 10','wheel vel left','wheel vel right',2)

%% step with acc limit og 5m/s^2 and turn regulator
data1 = load('v4turnRegStep.txt');
%  1    time 0.000 sec
%  2    mission (0) state 2
%  3  4 Motor velocity ref left, right: 0.17 0.17
%  5  6 Motor voltage [V] left, right: 0.8 0.8
%  7  8 Motor current left, right [A]: 0
%  9 10 Wheel velocity [r/s] left, righ
% 11    Turnrate [r/s]: 0.0000
% 12 13 14 Pose x,y,h [m,m,rad]: 0.0000 0.0000 0.000000
% 15    Battery voltage [V]: 12.26
% 16 17 Get data time [us]: 560 +ctrl 860
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,3), 'b');
hold on
%plot(data1(1:n,1), data1(1:n,4), 'r');
plot(data1(1:n,1), data1(1:n,11), 'c');
plot(data1(1:n,1), data1(1:n,14)*10, 'm');
plot(data1(1:n,1), data1(1:n,9), 'y');
plot(data1(1:n,1), data1(1:n,10), 'g');
set(gca,'FontSize',14)
grid on
title('Turn step with turn regulator')
xlabel('time in [sec]');
ylabel('Value')
legend('Motor ref left', 'Turnrate', 'Heading * 10','wheel vel left','wheel vel right',2)

