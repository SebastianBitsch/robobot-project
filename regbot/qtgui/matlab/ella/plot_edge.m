% edge debug
clear
close all
%%
% Ella (8)
%  1    time 4.428 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 2), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: 0.2658 0.2658
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: 0.0971608 -9.82185e-06 -0.0026173 0.0184658
% 12 .. 32 Edge sensor: left -2.731363 1, right 1.175928 1, values 2103 2588 2818 2931 2599 2251 1835 1573, white 1, used 1, LEDhigh=1, xb=0 xw=1 xbc=0 xwc=9 lvl=5 lvr=5       whitelevel=[2276 2639 2892 2996 2872 2671 2521 2194]       blacklevel=[0 0 0 0 0 0 19 33]
% 33    Battery voltage [V]: 11.97
% 34 35 Get data time [us]: 100 +ctrl 640
% 36 45 ctrl edge, ref=0.5, m=0.5, m2=2.97525, uf=0, r2=0.5, ep=-0.495049,up=-0.165126, ui=0, u1=-0.165126, u=-0.165126
data100 = load('ella_edge_100.txt');
%%
data = data100;
figure(100)
hold off
plot(data(:,1), data(:,4),':m');
hold on
plot(data(:,1), data(:,14),'x-r');
plot(data(:,1), data(:,15),':r');
plot(data(:,1), data(:,30)/10,'g');
plot(data(:,1), data(:,31),'c');
plot(data(:,1), data(:,37),'b');
plot(data(:,1), data(:,38),'-om');
grid on
xlabel('mission time [s]')
legend('mission','right','valid','xw cnt','lvr','m','m2','location','southeast');
%%
% Ella (8)
%  1    time 3.944 sec
%  2 .. 22 Edge sensor: left -2.675530 1, right 2.660000 1, values 1995 2658 3047 3040 3042 2656 2444 2174, white 1, used 1, LEDhigh=1, xb=0 xw=1 xbc=0 xwc=19 lvl=5 lvr=5       whitelevel=[2276 2639 2892 2996 2872 2671 2521 2194]       blacklevel=[0 0 0 0 0 0 19 33]
data200 = load('ella_edge_200.txt');
%%
data = data200;
figure(200)
hold off
plot(data(:,1), data(:,4),':m');
hold on
plot(data(:,1), data(:,4),'x-r');
plot(data(:,1), data(:,5),':r');
plot(data(:,1), data(:,20)/10,'g');
plot(data(:,1), data(:,21),'c');
grid on
xlabel('mission time [s]')
legend('mission','right','valid','xw cnt','lvr','location','southeast');
%%
% Ella (8)
%  1    time 3.693 sec
%  2 .. 22 Edge sensor: left -2.666372 1, right 1.416858 1, values 1752 2494 2803 2904 2774 2210 1649 1239, white 1, used 1, LEDhigh=1, xb=0 xw=1 xbc=0 xwc=1 lvl=5 lvr=5       whitelevel=[2276 2639 2892 2996 2872 2671 2521 2194]       blacklevel=[0 0 0 0 0 0 19 33]
% 23 32 ctrl edge, ref=0.5, m=0.5, m2=1.48512, uf=0, r2=0.5, ep=-0.197025,up=-0.0658951, ui=0, u1=-0.0658951, u=-0.0658951
data301 = load('ella_edge_301.txt');
data302 = load('ella_edge_302.txt');
data303 = load('ella_edge_303.txt');
data304 = load('ella_edge_304.txt');
%%
data = data304;
figure(304)
hold off
plot(data(:,1), data(:,4),':m');
hold on
plot(data(:,1), data(:,5),'x-r');
plot(data(:,1), data(:,18),'vy');
plot(data(:,1), data(:,20)/10,':b');
plot(data(:,1), data(:,2),'c');
plot(data(:,1), data(:,24),'b');
plot(data(:,1), data(:,25),'-om');
grid on
xlabel('mission time [s]')
legend('right','valid','xw','xw cnt','left','m','m2','location','southeast');
%% balance velocity
% Ella (8)
%  1    time 2.002 sec
%  2  3 Motor velocity ref left, right: 0.0648 0.1673
%  4  5  6  7 Pose x,y,h,tilt [m,m,rad,rad]: 0.0125759 8.12261e-05 0.00523459 -0.00112881
%  8 17 ctrl bal vel, ref=0.2, m=0.101874, m2=0.145037, uf=0, r2=0.2, ep=0.00137408,up=0.00137408, ui=-0.0043939, u1=-0.00301982, u=-0.00301982
data310 = load('ella_edge_310.txt');
data311 = load('ella_edge_311.txt');
data312 = load('ella_edge_312.txt');
data313 = load('ella_edge_313.txt');
data314 = load('ella_edge_314.txt');
data315 = load('ella_edge_315.txt');
data316 = load('ella_edge_316.txt');
data317 = load('ella_edge_317.txt');
data318 = load('ella_edge_318.txt');
data319 = load('ella_edge_319.txt');
data320 = load('ella_edge_320.txt');
data321 = load('ella_edge_321.txt');
data322 = load('ella_edge_322.txt');
data324 = load('ella_edge_324.txt');
data325 = load('ella_edge_325.txt');
data326 = load('ella_edge_326.txt');
data327 = load('ella_edge_327.txt');
data328 = load('ella_edge_328.txt');
data329 = load('ella_edge_329.txt');
data330 = load('ella_edge_330.txt');
data331 = load('ella_edge_331.txt');
data332 = load('ella_edge_332.txt');
data333 = load('ella_edge_333.txt');
data334 = load('ella_edge_334.txt');
data335 = load('ella_edge_335.txt');
data336 = load('ella_edge_336.txt');
data337 = load('ella_edge_337.txt');
%%
data339 = load('ella_edge_339.txt');
data340 = load('ella_edge_340.txt');
data341 = load('ella_edge_341.txt');
data342 = load('ella_edge_342.txt');
data343 = load('ella_edge_343.txt');
data344 = load('ella_edge_344.txt');
data345 = load('ella_edge_345.txt');
data346 = load('ella_edge_346.txt');
data347 = load('ella_edge_347.txt');
data348 = load('ella_edge_348.txt');
data349 = load('ella_edge_349.txt');
data350 = load('ella_edge_350.txt');
data351 = load('ella_edge_351.txt');
data352 = load('ella_edge_352.txt');
data353 = load('ella_edge_353.txt');
data354 = load('ella_edge_354.txt');
data355 = load('ella_edge_355.txt');
data356 = load('ella_edge_356.txt');
data357 = load('ella_edge_357.txt');
data360 = load('ella_edge_360.txt');
data370 = load('ella_edge_370.txt');
data371 = load('ella_edge_371.txt');

%%
data = data371;
figure(371)
hold off
plot(data(:,1), data(:,6),':m');
hold on
plot(data(:,1), data(:,2),':r');
plot(data(:,1), data(:,3),':b');
plot(data(:,1), data(:,7),'g');
plot(data(:,1), data(:,12),'vy');
plot(data(:,1), data(:,9),'--b');
plot(data(:,1), data(:,10),'c');
plot(data(:,1), data(:,17),'r');
plot(data(:,1), data(:,15),'--k');
grid on
xlabel('mission time [s]')
legend('head','left vel','right vel','tilt','ref','m','m2','u','ui','location','southeast');
data380 = load('ella_edge_380.txt');

%%
% Ella (8)
%  1    time 0.000 sec
%  2  3 Motor velocity ref left, right: 0.7877 0.7877
%  4  5  6  7 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 0.310109
%  8 17 ctrl balance, ref=0, m=0.310109, m2=0.312828, uf=0, r2=0, ep=0.782069,up=0.782069, ui=0, u1=0.782069, u=0.787655
% 18 27 ctrl bal vel, ref=0, m=0, m2=0, uf=0, r2=0, ep=0,up=0, ui=0, u1=0, u=0
data380 = load('ella_edge_380.txt');
data381 = load('ella_edge_381.txt');
data382 = load('ella_edge_382.txt');
data383 = load('ella_edge_383.txt');
data384 = load('ella_edge_384.txt');
data385 = load('ella_edge_385.txt');

%%
data = data385;
figure(385)
hold off
plot(data(:,1), data(:,6),':m');
hold on
plot(data(:,1), data(:,2),':r');
plot(data(:,1), data(:,3),':b');
plot(data(:,1), data(:,7),'g');
plot(data(:,1), data(:,8),'vy');
plot(data(:,1), data(:,9),'--b');
plot(data(:,1), data(:,10),'c');
plot(data(:,1), data(:,17),'r');
plot(data(:,1), data(:,15),'--k');
grid on
xlabel('mission time [s]')
legend('head','left vel','right vel','tilt','ref','m','m2','u','ui','location','southeast');


