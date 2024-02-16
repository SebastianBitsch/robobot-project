%% mekanisk_model_regbot
close all
clear
%% parametre
% motor
RA = 3.3;      % ohm
JA = 1.3e-6;   % motor inerti
LA = 6.6e-3;   % ankerspole
BA = 3e-6;     % ankerfriktion
Kemf = 0.0105; % motorkonstant
% køretøj
NG = 9.69; % gear
WR = 0.03; % hjul radius
%
%% model af balancerende pendul
mmotor = 0.193;   % samlet masse af motor og gear
mframe = 0.32;    % samlet masse af ramme og print
mtopextra = 0.27; % extra masse på top
%
%% ulineær simulering (i 4 sekunder)
sim('regbot_mekanisk_model', 4);
% som generer 2 datasæt x_pos [m] og pitch vinkel [rad]
figure(10)
plot(x_pos, 'b')
title (' Balancerende robot med et skub til tiden 0.5 sek')
ylabel('x-afstand [meter]')
grid on
% og vinkel
figure(11)
plot(pitch.time, pitch.data*180/pi , 'r')
grid on
title(' Balancerende robot med et skub til tiden 0.5 sek')
ylabel('Balancevinkel [grader]')
