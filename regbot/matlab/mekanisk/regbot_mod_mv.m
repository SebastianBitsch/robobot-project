%% mekanisk_model_regbot
close all
clear
%% parametre
% motor
RA = 3.3;    % ohm
JA = 1.3e-6; % motor inerti
LA = 6.6e-3; % ankerspole
BA = 3e-6;   % ankerfriktion
Kemf = 0.0105; % motorkonstant
% køretøj
NG = 9.69; % gear
WR = 0.03; % hjul radius
%
% model af balancerende pendul
mmotor = 0.25;  % samlet masse af motor og gear
mpendul = 0.25; % samlet masse af pendul (punktmasse)
mpdist =  0.1;   % afstand til pendulmasse
%
% motor controller
Kpmv = 0.537;
tmvi = 0.034;
%
%% temp params
%
% NB! cut loop before input
%     before linmod
% 
Kp = -30;
taud = 0.05;
taui = 0.1;
al=0.2;
Kpv = 1;
tauvd=1;
alv = 0.1;
%% linear model of robot in balance
balSwitch = -1;
velSwitch = 1;
[A,B,C,D] = linmod('regbot_1');
[num,den] = ss2tf(A,B,C,D)
%%
Gmw = minreal(tf(num(1,:),den));
Gmp = -minreal(tf(num(2,:),den));
Gmvx = minreal(tf(num(3,:),den));
Gmx = minreal(tf(num(4,:),den));
%%
%%
figure(6)
hold off
pzmap(Gmp)
hold on
pzmap(Gmw,'r')
%%
figure(7)
hold off
margin(Gmp)
grid on
legend('Gmp');
legend('From motor voltage to angle and angle velocity')
%% calculate ballance regulator
Ni =3;
gm = 50;
al = 0.2;
[Gsnum,Gsden] = tfdata(Gmp);
[wc, taud, taui, Kp, serr] = solvepid(Gsnum, Gsden, al, Ni, gm)
Kp = -Kp
taud
taui
wc
%% poles and zeros for angle and position
angle_poles = pole(Gmp)
angle_zeros = zero(Gmp)
poles_x = pole(Gmx)
zeros_x = zero(Gmx)
%% closed loop angle control bodeplot
Gc = Kp * tf([taui 1], [taui 0]) * tf([taud  1],[al*taud 1]);
Gmpol = Gc*Gmp;
Gmpcl = Gmpol/(1+Gmpol);
figure(9)
margin(Gmpol)
legend('angle open loop ctrl')
grid on
%%
figure(10)
bode(Gmpcl)
grid on
%
%% velocity control for robot
balSwitch = 1;
velSwitch = -1;
[A,B,C,D] = linmod('regbot_1');
[num,den] = ss2tf(A,B,C,D)
Gbvx = -minreal(tf(num(3,:),den))
%% bodeplot of 
figure(20)
hold off
margin(Gbvx)
hold on
grid on
legend('Gbvx open loop velocity with ballance');
%% velocity controller
gm = 90;
alv = 0.7;
[Gsnum,Gsden] = tfdata(Gbvx);
[wcv, tauvd, Kpv] = solvepd(Gsnum, Gsden, alv, gm, 100)
Kpv = Kpv;
%% open loop velocity control bodeplot
Gcv = Kpv * tf([tauvd  1],[alv*tauvd 1]);
Gbvol = Gcv*Gbvx;
figure(21)
margin(Gbvol)
legend('open loop velocity control')
grid on
%% ...and closed loop
Gbvcl = Gbvol/(1+Gbvol);
figure(22)
bode(Gbvcl)
legend('closed loop velocity control')
grid on
%% 
balSwitch = 1;
velSwitch = 1;
