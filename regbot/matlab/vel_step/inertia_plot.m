% plot af data dra regbot
clear
close all
%%
data1 = load('inertia_step_1.txt');
data2 = load('inertia_step_2.txt');
%  1    time 0.000 sec
%  2    mission (0) state 2
%  3  4 Motor velocity ref left, right: 2.00 2.00
%  5  6 Motor voltage [V] left, right: 2.0 2.0
%  7  8 Motor current left, right [A]: -0.006 0.000
%  9 10 Encoder left, right: 6109 5772
% 11 12 Wheel velocity [r/s] left, right: -0.0072 0.0128
% 13    Battery voltage [V]: 12.02
%
%% constants
%---------------------- Measured Constants -------------------------------
r = 0.06/2;     % hjul radius
NG = 9.68;      % gear omsætning
mh = 0.012;     % masse af 2 hjul 
Jh = mh*r^2;    % Inerti af hjul
mR = 0.54;      % masse af robot (med kugle-hjul, uden ledninger)
m1 = 0.2;       % Kg - masse af motor (det der ikke er pendul)
m2 = mR - m1;   % Kg masse af pendul
RA = 3.5;       % Ohm ankermodstand
%-------------------------------------------------------------------------
% steady state velocity test
V_A = 3;        % ankerspænding [V] - measured SS
I_A = 0.15;     % ankerstrøm steady state [A] - measured SS
W_A = 24.0;     % omdrejningshastighed aksel [rad/s] - measured SS
% calculation from steady state Kemf, bm
W_M = W_A * NG;
RPM4V = W_M/2/pi*60; % skal være ca. 3000 RPM
V_Rss = RA*I_A; % spænding over ankermodstand
Vemf = V_A-V_Rss; % beregnet elektromotorisk spænding
Kemf = Vemf/W_M; % beregnet motor-konstant
Ktau = Kemf;
TauAss = I_A*Ktau; % moment af anker steady state
bm = TauAss/W_M;   % dynamisk friktion motor og gear alene
%
%% plot motor current/velocity
figure(1)
n = size(data1,1);
hold off
plot(data1(1:n,1), data1(1:n,7), 'b');
hold on
plot(data1(1:n,1), data1(1:n,8), 'c');
plot(data1(1:n,1), data1(1:n,11)/10, 'r');
plot(data1(1:n,1), data1(1:n,12)/10, 'm');
set(gca,'FontSize',14)
grid on
title('inertia step')
xlabel('open loop step 2V, time in [sec]');
ylabel('rad/s (1/10) and amps')
legend('current left', 'current right','velocity left/10 r/s', 'velocity right/10 r/s',2)
%% plot motor current/velocity
figure(2)
n = size(data1,1);
hold off
plot(data2(1:n,1), data2(1:n,7), 'b');
hold on
plot(data2(1:n,1), data2(1:n,8), 'c');
plot(data2(1:n,1), data2(1:n,11)/10, 'r');
plot(data2(1:n,1), data2(1:n,12)/10, 'm');
set(gca,'FontSize',14)
grid on
title('inertia step')
xlabel('open loop step 2.5V, time in [sec]');
ylabel('rad/s (1/10) and amps')
legend('current left', 'current right','velocity left/10 r/s', 'velocity right/10 r/s',2)
%%
%% conclusion (right wheel driving rotation only - figure 2)
iAvg=0.62; % A anchor current over 0.4 seconds
tAcc=0.3;  % sec acceleration time
vEnd=2.36; % r/s robot(pendulum) rotation velocity
tAvg=iAvg*Ktau*NG; % average torque on wheel axle
aAvg=vEnd/tAcc;    % average acceleration (asummed constant) over 0.3 sec
J2=tAvg/aAvg       % inertia of robot including one motor spinning, other fixed