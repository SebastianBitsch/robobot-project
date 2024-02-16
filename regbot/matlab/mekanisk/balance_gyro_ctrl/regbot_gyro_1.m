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
%% model af balancerende pendul
mmotor = 0.193;   % samlet masse af motor og gear
mframe = 0.32;    % samlet masse af ramme og print
mtopextra = 0.27; % extra masse på top
mtopextra2 =0.214; % 0.215; % extra masse på top - mast - not used
mpdist =  0.10;   % afstand til låg
%
%% temp params to make simulink run
% but all temp parameters are switched off
%
% motor velocity PI
Kpmv = 0.537
tmvi = 0.034
% mission velocity
Kpv = 1;
tvd=1;
alv = 0.1;
zeta=0.2;
tvi = 1;
% balance
Kpb = -30;
tbd = 0.05;
tbi = 0.1;
alb = 0.2;
%
%% 2. order zero-pole filter
% of tilt angle
% 
tz = 0.005;
zetaz = 0.12;
tp = 0.004;
zetap = 0.4;
tiltZ = [tz^2 2*tz*zetaz 1];
tiltP = [tp^2 2*tp*zetap 1];
Gef = tf(tiltZ,tiltP);
figure(1)
hold off
bode(Gef);
grid on
Gefz = c2d(Gef, 0.001, 'tustin')
hold on
bode(Gefz)
% not used
tiltZ = [1];
tiltP = [1];
%%
% zetaplp = 0.307;
% tplp = 0.005;
% tzplp = 0; % 1/142;
% zetazplp = 0.7;

%% linear model of robot in balance
switch_mot = 1; %  1 = no motor controller
balSwitch = -1; % -1 = motor input for linmod
velSwitch = 1;
pdSwitch = -1;
gyroSwitch = -1;
[A,B,C,D] = linmod('regbot_1g');
[num,den] = ss2tf(A,B,C,D)
%%
Gmw = minreal(tf(num(1,:),den));  %  pitch velocity
Gmp = minreal(tf(num(2,:),den)); %  pitch angle
Gmvx = minreal(tf(num(3,:),den)); % x-velocity
Gmx = minreal(tf(num(4,:),den));  % x position
%%
%%
figure(6)
hold off
pzmap(Gmp)
grid off
%%
figure(8)
hold off
ww = logspace(-3,3,300);
bode(-1*Gmp, ww)
hold on
bode(-1*Gmw, ww)
grid on
legend('Gmp - pitch angle', 'Gmw pitch velocity');
xlabel('From motor voltage to pitch')
%% calculate balance regulator pitch angle velocity
gmw = 60;
alw = 0.5;
[Gsnum,Gsden] = tfdata(-1*Gmw);
[wcw, tdw, Kpw] = solvepd(Gsnum, Gsden, alw, gmw, 100);
Kpw = -Kpw
tdw
wcw
% open loop balance angle control bodeplot
%Gc = Kpb * tf([tbi 1], [tbi 0]) * tf([tbd  1],[alb*tbd 1]);
Gcw = Kpw * tf([tdw  1],[alw*tdw 1]);
%c2d(Gc,0.001,'tustin')
GmwOL = Gcw*Gmw;
figure(9)
hold off
margin(-Gmw)
hold on
margin(GmwOL)
GmwCL = GmwOL/(1+GmwOL);
bode(GmwCL)
xlabel('Open loop balance tilt velocity controller');
legend('open loop', 'openLoop with ctrl', 'closed loop','Location','SouthWest')
grid on
%% new model with gyro velocity controller
balSwitch = -1;
gyroSwitch = 1;
[A,B,C,D] = linmod('regbot_1g');
[num,den] = ss2tf(A,B,C,D);
%
Gmw = minreal(tf(num(1,:),den))  %  pitch velocity
Gmp = minreal(tf(num(2,:),den)) %  pitch angle
Gmvx = minreal(tf(num(3,:),den)) % x-velocity
Gmx = minreal(tf(num(4,:),den))  % x position
% figure(106)
% hold off
% pzmap(Gmp)
% grid off
%
figure(108)
hold off
ww = logspace(-1,4,300);
bode(-1*Gmp, ww)
hold on
bode(-1*Gmw, ww)
grid on
legend('Gmp - pitch angle', 'Gmw pitch velocity');
xlabel('From motor voltage to pitch')
%% calculate balance regulator - pitch angle
gm = 67;
alb = 0.5;
[Gsnum,Gsden] = tfdata(1*Gmp);
[wc, tdb, Kpb] = solvepd(Gsnum, Gsden, alb, gm, 100);
Kpb = Kpb
tdb
wc
% open loop balance angle control bodeplot
%Gc = Kpb * tf([tbi 1], [tbi 0]) * tf([tbd  1],[alb*tbd 1]);
Gc = Kpb * tf([tdb  1],[alb*tdb 1]);
%c2d(Gc,0.001,'tustin')
Gmpol = Gc*Gmp;
figure(110)
hold off
bode(Gmp)
hold on
margin(Gmpol)
xlabel('Open loop balance with controller');
grid on
%%
Gmpcl = minreal(Gmpol/(1+Gmpol));
figure(112)
bode(Gmpcl,ww);
title('closed loop tilt control')
grid on
bandwidthTilt = bandwidth(Gmpcl)
tiltCtrlPoles = pole(Gmpcl)
%
%% change routing for simulation with balance, but not mission velocity
balSwitch = 1;
velSwitch = -1;
%% poles and zeros for balance angle and position
angle_poles = pole(Gmp)
angle_zeros = zero(Gmp)
%% positopn poles and zeros for x-velocity control
poles_x = pole(Gmx)
zeros_x = zero(Gmx)
%% balance in place, mission velocity control estimate for robot
balSwitch = 1;
velSwitch = -1;
[A,B,C,D] = linmod('regbot_1g');
[num,den] = ss2tf(A,B,C,D)
Gbvx = minreal(tf(num(3,:),den))
velSwitch = 1
%% bodeplot of tilt ref to x-velocity
figure(120)
hold off
bode(Gbvx,ww)
title('Open loop tilt-ref to x-velocity')
hold on
grid on
%legend('Gbvx open loop velocity with balance',3);
%% manuel velocity controller design from selected wC
velSwitch = 1
alv = 0.6;
Ni = 2.5;
phiI = -atan2(1,Ni) * 180/pi;
phiM = asin((1-alv)/(1+alv)) * 180/pi
angle_wC = -180 + gm - phiI - phiM 
% omegaC sat til 3 r/s
wC = 6;
zeta=0.5
tvi = Ni/wC
tvd = 1/(sqrt(alv)*wC)
alv2 = alv*0.93;
Gvdd = tf([tvd 1],[(alv2*tvd)^2 2*zeta*alv2*tvd 1])
Gvd = tf([tvd 1],[alv*tvd 1])
Gvi = tf([tvi 1],[tvi 0]);
KpSign = 1;
[mag,phase] = bode(KpSign*Gvd*Gbvx*Gvi, wC);
Kpv=1/mag*KpSign
[magd,phased] = bode(KpSign*Gvdd*Gbvx*Gvi, wC);
Kpvd=1/magd*KpSign
% figure(25)
% hold off
% margin(Kpv*Gvd*Gvi*Gbvx)
% xlabel('velocity control - PI Lead')
figure(126)
hold off
margin(Kpvd*Gvdd*Gbvx*Gvi)
grid on
xlabel('velocity control open loop - PI-complex-pole-lead')
% closed loop velocity system
GvelOL = Kpvd*Gvdd*Gbvx*Gvi
GvelCL = GvelOL/(1+GvelOL)
figure(127)
bode(GvelCL)
grid on
title('closed loop velocity control')
% for simulation
pdSwitch = -1;  % 1 = normal PD, -1 = complex-pole-PD
if (pdSwitch < 0)
    Kpv=Kpvd
end
%%
sim('regbot_1g', 5)
figure(30)
plot(x_pos);
grid on
title('Ballance robot with a push after 0.2 second')
ylabel('x-position [meter]')
%%
figure(31)
plot(pitch.time, pitch.data * 180/pi, 'r')
grid on
title('Ballance robot with a push after 0.2 second')
ylabel('Tilt angle [degrees]')

%% total closed loop transfer function
Gvol =  Kpvd*Gvdd*Gbvx*Gvi;
Gvcl = minreal(Gvol/(1 + Gvol))
pole(Gvcl)
% 
%%
T = 0.001;
c2d(Gvdd*Kpv,T,'tustin')
c2d(Gvi,T,'tustin')