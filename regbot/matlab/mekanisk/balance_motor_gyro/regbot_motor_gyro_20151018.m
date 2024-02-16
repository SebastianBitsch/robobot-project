%% mekanisk_model_regbot
%close all
%clear
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
Kpwv = 1;
alwv = 1;
tdwv = 1;
% gyro velocity ctrl
Kpw = 1;
tdw = 1;
alw = 1;
%
%% linear model of robot in balance
switch_mot = 1; %  1 = no motor controller
balSwitch = 1; %  -1 = motor input for linmod
velSwitch = 1;
pdSwitch = -1;
gyroSwitch = -1;
motorSwitch = -1;
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
bode(Gmwv)
hold on
grid on
title('motor controller')
legend('open loop');
%% motor velocity controller design
Kpwv = 5
GmwvCL = Kpwv*Gmwv/(1+Kpwv*Gmwv);
bode(GmwvCL);
legend('open loop','closed loop');
%% implement motor controller
motorSwitch = 1;
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
gmw = 70;
alw = 0.99;
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
motorSwitch = 1;
[A,B,C,D] = linmod('regbot_1mg');
[num,den] = ss2tf(A,B,C,D);
%
Gmw= minreal(tf(num(1,:),den))  %  pitch velocity
Gmp=minreal(tf(num(2,:),den)) %  pitch angle
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
xlabel('From gyro velocity ctrl to pitch')
%% calculate balance regulator - pitch angle
gm = 80;
alb = 0.9;
[Gsnum,Gsden] = tfdata(1*Gmp);
[wc, Kpb] = solvep(Gsnum, Gsden, gm);
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
[A,B,C,D] = linmod('regbot_1mg');
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
alv = 0.45;
Ni = 4;
phiI = -atan2(1,Ni) * 180/pi;
phiM = asin((1-alv)/(1+alv)) * 180/pi
angle_wC = -180 + gm - phiI - phiM 
% omegaC sat til 3 r/s
wC = 8;
zeta=1.0;
tiv = Ni/wC
tdv = 1/(sqrt(alv)*wC)
alv2 = alv*1.0;
Gvdd = tf([tdv 1],[(alv2*tdv)^2 2*zeta*alv2*tdv 1])
Gvp = tf([1],[alv*tdv 1])
Gvi = tf([tiv 1],[tiv 0]);
KpSign = 1;
% with 3-pole lead
[magd,phased] = bode(KpSign*Gvdd*Gvp*Gbvx, wC);
Kpvd=1/magd*KpSign
[magd,phased] = bode(KpSign*Gvdd*Gvp*Gbvx*Gvi, wC);
KpvdI=1/magd*KpSign
% controller bode
figure(120)
hold off
bode(Gbvx,ww)
hold on
grid on
%bode(KpvdI*Gvdd*Gvp*Gvi)
bode(Kpvd*Gvdd*Gvp)
%bode(KpvdI*Gvdd*Gvp*Gbvx*Gvi)
GvelOL = Kpvd*Gvdd*Gvp*Gbvx*1
margin(GvelOL)
% GvelOLI = KpvdI*Gvdd*Gvp*Gbvx*Gvi
% GvelCLI = GvelOL/(1+GvelOL)
% bode(GvelCLI)
GvelCL = GvelOL/(1+GvelOL)
bode(GvelCL)
%legend('system open loop','Controller with I', 'controller no I', ...
%    'open with controller', 'open loop w. ctrl, no I', ...
%    'Closed loop', 'closed loop no I', ...
%    'location','southEast')
legend('system open loop','controller no I', ...
    'open with controller', ...
    'Closed loop no I', ...
    'location','southEast')
%%
% xlabel('velocity control - PI Lead')
figure(126)
hold off
margin(GvelOL)
grid on
xlabel('velocity control open loop - PI-complex-pole-lead')
% closed loop velocity system
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
sim('regbot_1mg', 5)
figure(30)
plot(x_pos);
grid on
title('Ballance start at 30 degrees')
ylabel('x-position [meter]')
%%
figure(31)
plot(pitch.time, pitch.data * 180/pi, 'r')
grid on
title('Ballance start at 30 degrees')
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