% Model System parameters
clear
close all
%%
RA = 3.5;   % stall 6V/2.2A; % from spec
            % - measured to between 2.4 and 3.0 Ohm
            % + Zth i driver (og strømforsyning)
NG = 9.68;  % gear omsætning
%
%% conclusion form plot - motor constants (motor alone)
V_A = 6;   % ankerspænding [V] - measured SS
I_A = 0.21; % ankerstrøm steady state [A] - measured SS
W_A = 58.8;  % omdrejningshastighed aksel [rad/s] - measured SS
W_M = W_A * NG; % motor velocity rad/s
RPM4V = W_M/2/pi*60; % skal være ca. 3000 RPM @ 4V
V_Rss = RA*I_A; % spænding over ankermodstand
Vemf = V_A-V_Rss; % beregnet elektromotorisk spænding
Kemf = Vemf/W_M % beregnet motor-konstant
Ktau = Kemf;
TauAss = I_A*Ktau % moment af anker steady state
bM = TauAss/W_M    % dynamisk friktion
%% inertia measurement for pendul
% Turn around vertical axis with one motor
% stop after 0.3 sec end speed 2.3r/s current 0.63A
t1 = 0.3; % stop time
v1 = 2.3; % wheel axel velocity [r/s] at stop time
i1 = 0.63; % anchor current - measured - rather constant
ta1 = i1*Ktau; % motor moment
ma1 = v1*NG/t1; % motor acceleration, assuming constant [rad/s2]
JM = ta1/ma1    % inertia moment on motor side
JR = JM*NG^2    % inertia on wheel axis Kg m^2
mR = 0.54;  % masse af robot (med kugle-hjul, uden ledninger)
lm = sqrt(JR/mR) % lille l i pendul model
%%
% opsummering (22/1 2015) for omvendt pendul parametre
% RA   = 3.5 Ohm - mest ud fra ankerspænding og ankerstrøm måling
% L    = 10.9 mH - ud fra tidskonstant nedenfor (tau1)
% Kemf = 0.0093  - ud fra steady state måling - samme tal som Ktau
% bM   = 3.4e-6 Nm/(rad/sek) - steady state fra -3 til +6V - se current_sensor_-3v_6v_2.txt
% JR   = 0.0074 Kgm^2 - inertimoment omkring hjulaksel - fra motor-step inertia_step_2.txt
% mR   = 0.54 Kg - robottens masse (køkkenvægt)
% l    = 0.105 m - omregnet til pendulmassens afstand fra aksel
%
%%
% beregning af dynamiske poler for motor alene
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
GI = tf([1],[JA bTot]);
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
[sn sd] = tfdata(GM*Gd);
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
title('step responce PI-Lead Kp=0.54, ti=0.021, td=0.0092 - simuleret')
step(Gclpid)
grid on
