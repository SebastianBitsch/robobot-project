% Model System parameters
clear
close all
RA = 3.3;   % stall 6V/2.2A; % from spec
            % - measured to between 2.4 and 3.0 Ohm
            % + Zth i driver (og strømforsyning)
NG = 9.68;  % gear omsætning
%
%% conclusion form plot
V_A = 3;   % ankerspænding [V] - measured SS
I_A = 0.15; % ankerstrøm steady state [A] - measured SS
W_A = 24.0;  % omdrejningshastighed aksel [rad/s] - measured SS
W_M = W_A * NG;
RPM4V = W_M/2/pi*60; % skal være ca. 3000 RPM
V_Rss = RA*I_A; % spænding over ankermodstand
Vemf = V_A-V_Rss; % beregnet elektromotorisk spænding
Kemf = Vemf/W_M; % beregnet motor-konstant
Ktau = Kemf;
TauAss = I_A*Ktau; % moment af anker steady state
bM = TauAss/W_M;    % dynamisk friktion
bR = 0e-6;
bTot = bM + bR;
%%
% beregning af dynamiske poler
tau1 = 0.003; % effekt af ankerspole
tau2 = 0.037; % effekt af motor inerti
L = tau1*RA;  % induktion i anker
JA = tau2*(RA*bM+Kemf*Ktau)/RA;
JT = 2.5e-6;
J = JA+JT; % samlet Inerti
%%
Rw = 0.06/2; % hjul radius
B  =  0.155; % Base - afstand mellem hjul
%% open loop motor model med gear
% motor model (inc Inerti)
GA = tf([Ktau],[L RA]);
GI = tf([1],[JA bM]);
GM = minreal(GA * GI / (1 + GA * GI * Kemf)/NG);
%% plot motor open loop bodeplot
figure(20)
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
del = 0.002; % ms delay
[Gdn,Gdd] = pade(0.002,4);
Gd = tf(Gdn, Gdd);
% plot open loop with delay
figure(21)
margin(GM*Gd)
grid on
%% regulator design - PI
Ni = 4;
pm = 60;
[sn, sd] = tfdata(GM*Gd);
[wc, ti, kp] = solvepi(sn{1}, sd{1}, Ni, pm)
Gci = tf([ti 1],[ti 0]);
Gol = kp*Gci*GM*Gd;
% open loop plot
figure(22)
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
figure(23)
title('Step responce')
step(Gcl)
grid on

%% regulator design PI-Lead
Ni = 4
pm = 60
al = 0.3
[sn sd] = tfdata(GM*Gd);
[wc, td, ti, kp] = solvepid(sn{1}, sd{1}, al, Ni, pm)
Gci = tf([ti 1],[ti 0]);
Gcd = tf([td 1],[td*al 1]);
% open loop plot
Golpid  = Gci*Gcd*kp*GM*Gd;
figure(30)
hold off
title('PI-Lead open loop with controller')
bode(Golpid)
grid on
%closed loop
hold on
Gclpid = Golpid/(1+Golpid);
bode(Gclpid)
BWpid = bandwidth(Gclpid)
%% step responce PI-Lead
figure(31)
step(Gclpid)
title('step PI-Lead K_P=0.56, \tau_i=0.02, \alpha=0.3 \tau_d=0.009-sim')
grid on
