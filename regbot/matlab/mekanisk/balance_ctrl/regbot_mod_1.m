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
[A,B,C,D] = linmod('regbot_1');
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
ww = logspace(-2,3,300);
bode(-1*Gmp, ww)
grid on
legend('Gmp - pitch angle', 'Gmw pitch velocity');
xlabel('From motor reference to pitch angle')
%% calculate balance regulator
gm = 60;
alb = 0.02;
[Gsnum,Gsden] = tfdata(-1*Gmp);
[wc, tbd, Kpb] = solvepd(Gsnum, Gsden, alb, gm, 100);
Kpb = -Kpb
tbd
wc
% open loop balance angle control bodeplot
%Gc = Kpb * tf([tbi 1], [tbi 0]) * tf([tbd  1],[alb*tbd 1]);
Gc = Kpb * tf([tbd  1],[alb*tbd 1]);
%c2d(Gc,0.001,'tustin')
Gmpol = Gc*Gmp;
figure(10)
margin(Gmpol)
hold on
xlabel('Open loop balance with controller');
grid on
%%
Gmpcl = minreal(Gmpol/(1+Gmpol));
figure(12)
bode(Gmpcl,ww);
title('closed loop tilt control')
grid on
bandwidth(Gmpcl)
pole(Gmpcl)
%
%% change routing for simulation with balance, but not mission velocity
balSwitch = 1;
velSwitch = -1;
%% poles and zeros for balance angle and position
angle_poles = pole(Gmp)
angle_zeros = zero(Gmp)
%% positopn poles and zeros
poles_x = pole(Gmx)
zeros_x = zero(Gmx)
%% balance in place, mission velocity control estimate for robot
balSwitch = 1;
velSwitch = -1;
[A,B,C,D] = linmod('regbot_1');
[num,den] = ss2tf(A,B,C,D)
Gbvx = minreal(tf(num(3,:),den))
velSwitch = 1
%% bodeplot of tilt ref to x-velocity
figure(20)
hold off
bode(Gbvx,ww)
title('Open loop tilt-ref to x-velocity')
hold on
grid on
%legend('Gbvx open loop velocity with balance',3);
%% manuel velocity controller design from selected wC
velSwitch = 1
alv = 0.7;
Ni = 2.5;
phiI = -atan2(1,Ni) * 180/pi;
phiM = asin((1-alv)/(1+alv)) * 180/pi
angle_wC = -180 + gm - phiI - phiM 
% omegaC sat til 3 r/s
wC = 7;
zeta=1
tvi = Ni/wC
tvd = 1/(sqrt(alv)*wC)
alv2 = alv*1;
Gvdd = tf([tvd 1],[(alv2*tvd)^2 2*zeta*alv2*tvd 1])
Gvd1 = tf([1],[alv*tvd 1])  
Gvd = tf([tvd 1],[alv*tvd 1])
Gvi = 1; %tf([tvi 1],[tvi 0]);
KpSign = 1;
[mag,phase] = bode(KpSign*Gvd*Gbvx*Gvi, wC);
Kpv=1/mag*KpSign
[magd,phased] = bode(KpSign*Gvdd*Gbvx*Gvi, wC);
Kpvd=1/magd*KpSign
% figure(25)
% hold off
% margin(Kpv*Gvd*Gvi*Gbvx)
% xlabel('velocity control - PI Lead')
figure(26)
hold off
margin(Kpvd*Gvdd*Gbvx*Gvi)
grid on
xlabel('velocity control open loop - PI-complex-pole-lead')
% closed loop velocity system
GvelOL = Kpvd*Gvdd*Gbvx*Gvi
GvelCL = GvelOL/(1+GvelOL)
figure(27)
bode(GvelCL)
grid on
title('closed loop velocity control')
% for simulation
pdSwitch = -1;  % 1 = normal PD, -1 = complex-pole-PD
if (pdSwitch < 0)
    Kpv=Kpvd
end
%%
sim('regbot_1', 5)
figure(30)
plot(x_pos);
grid on
title('Ballance robot start tilt 30 deg')
ylabel('x-position [meter]')
%%
figure(31)
plot(pitch.time, pitch.data * 180/pi, 'r')
grid on
title('Ballance robot - start tilt 30 deg')
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