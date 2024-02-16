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
mtopextra = 0.27; % 0.27; % extra masse på top
mpdist =  0.10;   % afstand til låg
pushDist = 0.1;
%
%% Sample time
Ts = 0.001;
%% temp params to make simulink run
% but all temp parameters are switched off
%
% motor velocity PI
Kpmv = 0.537
tmvi = 0.034
% mission velocity
Kpv = 1;
Kpv2 = 0;
tdv=1;
alv = 0.1;
zeta=0.2;
tiv = 1;
% balance
Kpb = -30;
tdb = 0.05;
tib = 0.1;
alb = 0.2;
% motor velocity ctrl
taudwv = 0.2;
tauiwv = 1;
wviSwitch = 1;
alphawv = 0.3;
Kpwv = 1;
alwv = 1;
tdwv = 1;
%pre filter
pre_pole = 1
% gyro velocity ctrl
Kpw = 1;
tdw = 1;
alw = 1;
tiw = 1;
taudb = 1;
tauib = 1;
tauiv = 1;
taudv = 0
%
%% linear model of robot in balance
startAngle = 0.5
switch_mot = 1; %  1 = no motor controller
balSwitch = 1; %  -1 = motor input for linmod
velSwitch = 1;
pdSwitch = -1;
gyroSwitch = -1;
motorSwitch = -1;
distSwitch = 1
[A,B,C,D] = linmod('regbot_1mg');
[num,den] = ss2tf(A,B,C,D)
%%
% Gmw = minreal(tf(num(1,:),den));  %  pitch velocity
% Gmp = minreal(tf(num(2,:),den)); %  pitch angle
% Gmvx = minreal(tf(num(3,:),den)); % x-velocity
% Gmx = minreal(tf(num(4,:),den));  % x position
Gmwv = minreal(tf(num(5,:),den));  % wheel axle velocity
%% motor controller
figure(201)
hold off
ww = logspace(-1,3,300);
bode(Gmwv,ww)
hold on
grid on
title('motor controller')
legend('open loop');
% motor velocity controller design
Kpwv = 15.0;
tauiwv = 0.002;
taudwv = 0.043;
alphawv = 0.99;
Ni = 3;
[Gsnum,Gsden] = tfdata(Gmwv);
%[wcw, taudwv, tauiwv, Kpwv] = solvepid(Gsnum, Gsden, alphawv, Ni, 100);
%tauiwv
Gwvi = 1; %tf([tauiwv 1],[tauiwv 0]);
[wcw, taudwv, Kpwv] = solvepd(Gsnum, Gsden, alphawv, 95,1000);
taudwv
Kpwv
wviSwitch = 1; % 1= uden i-led, -1 = med I-led
Kpwvtf = Kpwv * tf([taudwv 1],[alphawv*taudwv 1]);
margin(Kpwvtf* Gmwv * Gwvi)
% do not work well with I-term
GmwvCL = Kpwvtf*Gmwv*Gwvi/(1+Kpwvtf*Gmwv*Gwvi);
GmwvCLrev = Gmwv*Gwvi*Kpwvtf/(1+Kpwvtf*Gmwv*Gwvi);
% bode(GmwvCL);
bode(GmwvCLrev)
GmwvCL_BW=bandwidth(GmwvCL)
GmwvCL_rev_BW=bandwidth(GmwvCLrev)
xlabel('Motor controller open loop')
legend('open loop','open loop m controller','closed loop Lead-fwd','Closed loop Lead-rev');
%% implement motor controller
motorSwitch = 1;
balSwitch=-1;
startAngle = 5
[A,B,C,D] = linmod('regbot_1mg');
[num,den] = ss2tf(A,B,C,D)
%%
Gmw = minreal(tf(num(1,:),den));  %  pitch velocity
Gmp = minreal(tf(num(2,:),den)); %  pitch angle
Gmvx = minreal(tf(num(3,:),den)); % x-velocity
Gmx = minreal(tf(num(4,:),den));  % x position
%Gmwv = minreal(tf(num(5,:),den));  % wheel axle velocity

%%
figure(6)
hold off
pzmap(Gmp)
title('From motor ref to pitch')
grid off
%%
figure(81)
hold off
ww = logspace(-5,3,300);
bode(-1*Gmp, ww)
hold on
%bode(-1*Gmp/(1+Gmp), ww)
grid on
legend('Open loop', 'closed loop');
title('From motor ref to pitch')
%% calculate balance regulator - pitch angle
% integrator term
tauib = 0.07;
Gib = tf([tauib 1],[tauib 0]);
tauib2 = 0.07;
Gib2 = tf([tauib2 1],[tauib2 0]);
taudb = 0.3;
Gdb = tf([taudb 1],[0.01*taudb 1]);
taudb2 = 0.025;
Gdb2 = tf([taudb2 1],[0.0000001*taudb2 1]);
ww = logspace(-4.5,3,300);
%phasemargin = 22;
%[Gsnum,Gsden] = tfdata(-Gib*Gmp);
%[wc, Kpbp] = solvep(Gsnum, Gsden, phasemargin,150);
%Kpb = -Kpbp % too low value
Kpb = -7
figure(1101)
hold off
Gmpfwd = Kpb*Gib*Gmp;
Gmpfwd2 = Kpb*Gib2*Gmp;
bode(-Gmp,ww)
hold on
bode(Gib2)
bode(Gdb2)
margin(Gmpfwd2*Gdb2)
legend('Sys', 'i-led','d-led','ol',1)
grid on
hold on
%
figure(1102)
hold off
GmpCL = Gmpfwd/(1 + Gmpfwd*Gdb)
GmpCL2 = Gmpfwd2/(1 + Gmpfwd2*Gdb2)
ww = logspace(-2,1.5,300);
bode(GmpCL2,ww)
hold on
bode(GmpCL,ww)
grid on
legend('fast I (version 2)','0.07 sec I',3)
title('closed loop tilt')
tiltBW = bandwidth(GmpCL)
tiltBW2 = bandwidth(GmpCL2)
% test version 2
tauib = tauib2
taudb = taudb2
%%
% Gmpcl = minreal(Gmpol/(1+Gmpol));
% %figure(110)
% bode(Gmpcl,ww);
% title('closed loop tilt control')
% grid on
% bandwidthTilt = bandwidth(Gmpcl)
% tiltCtrlPoles = pole(Gmpcl)
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
pre_pole=0.07
satrtAngle=25
[A,B,C,D] = linmod('regbot_1mg');
[num,den] = ss2tf(A,B,C,D)
Gbvx = minreal(tf(num(3,:),den))
velSwitch = 1
%% bodeplot of tilt ref to x-velocity - including pre-filter
figure(119)
hold off
%bode(Gbvx,ww)
%hold on
ww = logspace(-1,2,300);
bode(Gbvx,ww)
title('Open loop tilt-ref to x-velocity')
hold on
grid on
%legend('Gbvx open loop velocity with balance',3);
%% manuel velocity controller design from selected wC
velSwitch = 1
alv = 0.99;
% Ni = 4;
% phiI = -atan2(1,Ni) * 180/pi;
% phiM = asin((1-alv)/(1+alv)) * 180/pi
% angle_wC = -180 + gm - phiI - phiM 
% omegaC sat til 3 r/s
wC = 2.5;
%zeta=1.0;
%tauiv = 2.5;
tdv = 1/(sqrt(alv)*wC)
alv2 = alv*1.0;
%Gvdd = tf([tdv 1],[(alv2*tdv)^2 2*zeta*alv2*tdv 1])
Gvp = tf([tdv 1],[alv*tdv 1]) % normal Lead
%Gvi = 1 %tf([tauiv 1],[tauiv 0]);
KpSign = 1;
% with 3-pole lead
[magd,phased] = bode(KpSign*Gvp*Gbvx, wC);
Kpvd=1/magd*KpSign;
Kpv = Kpvd
%Kpv = 0.3;
%figure(1191)
Gmvol = KpSign*Kpv*KpSign*Gvp*Gbvx; 
margin(Gmvol)
GmvCL = Gmvol/(1+Gmvol);
bode(GmvCL)
title('Open loop tilt-ref to x-velocity')
%[magd,phased] = bode(KpSign*Gvdd*Gvp*Gbvx*Gvi, wC);
%KpvdI=1/magd*KpSign
%% controller bode
% ww = logspace(-1,2.5,300);
% figure(120)
% hold off
% bode(Gbvx,ww)
% hold on
% grid on
% %bode(KpvdI*Gvdd*Gvp*Gvi)
% bode(Kpvd*Gvp,ww)
% %bode(KpvdI*Gvdd*Gvp*Gbvx*Gvi)
% GvelOL = Kpvd*Gvp*Gbvx*1
% margin(GvelOL)
% % GvelOLI = KpvdI*Gvdd*Gvp*Gbvx*Gvi
% % GvelCLI = GvelOL/(1+GvelOL)
% % bode(GvelCLI)
% GvelCL = GvelOL/(1+GvelOL)
% bode(GvelCL,ww)
% %legend('system open loop','Controller with I', 'controller no I', ...
% %    'open with controller', 'open loop w. ctrl, no I', ...
% %    'Closed loop', 'closed loop no I', ...
% %    'location','southEast')
% legend('system open loop','controller no I', ...
%     'open with controller', ...
%     'Closed loop no I', ...
%     'location','southEast')
%%
startAngle = 27
sim('regbot_1mg', 5)
figure(30)
plot(x_pos);
grid on
title('Measured position')
ylabel('x-position [meter]')
%%
figure(31)
plot(pitch.time, pitch.data * 180/pi, 'r')
grid on
title('Tilt angle')
ylabel('Tilt angle [degrees]')
%%
figure(32)
plot(x_vel.time, x_vel.data, 'r')
grid on
title('Measured velocity')
ylabel('Velocity [m/s]')
