%% Simscape multibody model og Regbot in balance
% initial setup with motor velocity controller 
% this is intended as simulation base for balance control.
%
close all
clear
%% Simulink model name
model='regbot_1mc';
fn = strcat(model,".txt")
debugPlot = 1;
%% parameters for REGBOT
% motor
RA = 3.3/2;    % ohm (2 motors)
JA = 1.3e-6*2; % motor inerti
LA = 6.6e-3/2; % rotor inductor (2 motors)
BA = 3e-6*2;   % rotor friktion
Kemf = 0.0105; % motor constant
Km = Kemf;
% køretøj
NG = 9.69; % gear
WR = 0.03; % wheel radius
Bw = 0.155; % wheel distance
% 
% model parts used in Simulink
mmotor = 0.193;   % total mass of motor and gear [kg]
mframe = 0.32;    % total mass of frame and base print [kg]
mtopextra = 0.97 - mframe - mmotor; % extra mass on top (charger and battery) [kg]
mpdist =  0.10;   % distance to lit [m]
% disturbance position (Z)
pushDist = 0.1; % relative to motor axle [m]

%% velocity controller (no balance) PI-regulator
% sample (usable) controller values
startAngle = 10;  % tilt in degrees at time zero
twvlp = 0.005;    % velocity noise low pass filter time constant (recommended)
% motor current control
timc = 1;
Kpmc = 0;
% balance
tibc = 1;
Kpbc = 0;
tdbc = 0;
% balance velocity
Kpv = 0;
tiv = 1;
tdv = 0;
alv = 1;
%balance position
Kpp = 0;
tip = 1;
tdp = 0;
alp = 0;
%% Estimate transfer function for base system using LINEARIZE
% Motor volatge to wheel velocity (wv)
load_system(model);
open_system(model);
% define points in model
ios(1) = linio(strcat(model,'/motor_voltage'),1,'openinput');
ios(2) = linio(strcat(model, '/current_filter'),1,'openoutput');
% attach to model
setlinio(model,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gmc = minreal(tf(num, den))
%% normalize for constant 1 in denominator
Gmc = norm1(Gmc) % not needed, comment out if you dont have the norm1 function
%% Bodeplot
h = figure(100)
bode(Gmc)
grid on
title('Transfer function from motor voltage to velocity')
%saveas(h, 'motor to velocity.png');

%% current control
gamma_mc = 60;
Ni_mc = 3.5;
[wcmc,Kpmc, timc] = findpi(Gmc, gamma_mc, Ni_mc)
% controller
Cimc = tf([timc 1],[timc 0]);
Golmc = Gmc*Kpmc*Cimc;
%% debug and save result
% function [C, Cd, h] = showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, Kff, tle)
showResult(debugPlot,1,fn, 'motor_current', Gmc, ...
           wcmc, Kpmc, timc, Ni_mc, 0, 1, ...
           gamma_mc, 0, 1, 'motor current control');
%% Balance
% define points in model
ios(1) = linio(strcat(model,'/cur_ref'),1,'openinput');
ios(2) = linio(strcat(model, '/pitch'),1,'openoutput');
% attach to model
setlinio(model,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gbc = minreal(tf(num, den))
%% balance angle design
tibc = 1/5;
Cibc = tf([tibc 1],[tibc 0]);
wc_bc = 100;
al_bc = 0.05;
tdbc = 1/(wc_bc*sqrt(al_bc));
Cdbc = tf([tdbc 1],[al_bc*tdbc 1]);
G = -Gbc*Cibc*Cdbc;
[Mbc,Pbc] = bode(G, wc_bc);
Kpbc = -1/Mbc;
Gbcol = Kpbc*Gbc*Cibc*Cdbc;
figure(300)
hold off
bode(Gbc)
hold on
bode(Gbcol)
grid on
margin(Gbcol)
% % Nyquist
% figure(310)
% hold off
% nyquist(-Gbc)
% hold on
% nyquist(Gbcol)
% grid on

showResult(debugPlot,1,fn, 'balance_angle', Gbc, ...
           wc_bc, Kpbc, tibc, 1, tdbc, al_bc, ...
           60, 0, 1, 'Balance control');

%% velocity in balance
% define points in model
ios(1) = linio(strcat(model,'/angle_ref'),1,'openinput');
ios(2) = linio(strcat(model, '/vel'),1,'openoutput');
% attach to model
setlinio(model,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gv = minreal(tf(num, den))
%% Bode for velocity control
figure(400)
hold off
bode(Gv)
grid on
% I-placement
wii = 10;
tau_ii = 1/wii;
Cii = tf([tau_ii 1],[tau_ii 0])
hold on
bode(Gv*Cii)
%% design velocity control
Ni_v = 2;
gm_v = 60;
w_lag = 30;
al_lag = 2.5;
taul = 1/(w_lag*sqrt(al_lag))
C_lag = 1;%tf([taul 1],[al_lag*taul 1])
Gv2 = Gv*C_lag;
[wc_v, Kpv, tiv] = findpi(Gv2,gm_v,Ni_v)
Civ = tf([tiv 1],[tiv 0]);
figure(410)
w = logspace(-0.5,3,100);
hold off
bode(Gv,w)
hold on
bode(Gv2,w)
bode(Gv2*Civ,w)
margin(Gv2*Civ*Kpv,w)
grid on
legend('sys','with lag','open-loop')
showResult(debugPlot,1,fn, 'velocity', Gv, ...
           wc_v, Kpv, tiv, Ni_v, 0, 1, ...
           gm_v, 0, 1, 'Velocity in balance');
%% Position control
% define points in model
ios(1) = linio(strcat(model,'/vel_ref'),1,'openinput');
ios(2) = linio(strcat(model, '/pos'),1,'openoutput');
% attach to model
setlinio(model,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gp = minreal(tf(num, den))
%% Bode for velocity control
figure(500)
hold off
bode(Gp)
grid on
%% design position control
gmp = 50;
alp = 0.25;
[wcp, Kpp, tdp] = findpd(Gp, gmp, alp)
showResult(debugPlot,1,fn, 'position', Gp, ...
           wcp, Kpp, -1, 0, tdp, alp, ...
           gmp, 0, 1, 'Position in balance');
%% run simulation
sim(model,10);