% plot af data fra regbot - new control log
clear
close all
%% plot ballance - hand held - partly on floor
% log is empty
% Silke (54)
%  1    time 2.003 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 1), events 0x0 (bit-flags)
%  6  7  8 Gyro x,y,z [deg/s]: 0.0305176 -2.2583 2.2583
%  9 10 Motor velocity ref left, right: 0.0338 0.0342
% 11 12 Wheel velocity [m/s] left, right: 0.0181 0.0125
% 13 14 15 16 Pose x,y,h,tilt [m,m,rad,rad]: 0.0355435 -3.92846e-05 0.00259112 -0.0107826
% 17 .. 37 Edge sensor: left 0.000000 0, right 0.000000 0, values 553 609 699 694 703 717 729 564, white 1, used 1, LEDhigh=0, xb=0 xw=0 xbc=0 xwc=0 lvl=0 lvr=0       whitelevel=[923 1039 1179 1162 1189 1192 1201 920]       blacklevel=[132 155 174 173 174 176 174 137]
dd1 = load('silke-line-follow.txt');
dd2 = load('silke-line-follow_02.txt');
dd3 = load('silke-line-follow_03.txt');
dd4 = load('silke-line-follow_04.txt');
%
whitelevel=[923 1039 1179 1162 1189 1192 1201 920];
blacklevel=[132 155 174 173 174 176 174 137];
gain=1./(whitelevel - blacklevel);
%  8  9 Get data time [us]: 100 +ctrl 460
%%
dd = dd4;
figure(6)
hold off
plot(dd(:,1), dd(:,4)/10,'r')
grid on
hold on
plot(dd(:,1), (dd(:,21) - blacklevel(1))*gain(1),'--r')
plot(dd(:,1), (dd(:,22) - blacklevel(2))*gain(2),'g')
plot(dd(:,1), (dd(:,23) - blacklevel(3))*gain(3),'c')
plot(dd(:,1), (dd(:,24) - blacklevel(4))*gain(4),'m')
plot(dd(:,1), (dd(:,25) - blacklevel(5))*gain(5),'y')
plot(dd(:,1), (dd(:,26) - blacklevel(6))*gain(6),':k')
plot(dd(:,1), (dd(:,27) - blacklevel(7))*gain(7),'r')
plot(dd(:,1), (dd(:,28) - blacklevel(8))*gain(8),'b')
% line valid
plot(dd(:,1), dd(:,18),'--r');
plot(dd(:,1), dd(:,20),'--b');
%%
figure(7)
hold off
plot(dd(:,1), dd(:,4),'r')
grid on
hold on
correct = 1-dd(:,16)*2
plot(dd(:,1), (dd(:,21) - blacklevel(1))*gain(1) .* correct,'--r')
plot(dd(:,1), (dd(:,22) - blacklevel(2))*gain(2).* correct,'g')
plot(dd(:,1), (dd(:,23) - blacklevel(3))*gain(3).* correct,'c')
plot(dd(:,1), (dd(:,24) - blacklevel(4))*gain(4).* correct,'m')
plot(dd(:,1), (dd(:,25) - blacklevel(5))*gain(5).* correct,'y')
plot(dd(:,1), (dd(:,26) - blacklevel(6))*gain(6).* correct,':k')
plot(dd(:,1), (dd(:,27) - blacklevel(7))*gain(7).* correct,'r')
plot(dd(:,1), (dd(:,28) - blacklevel(8))*gain(8).* correct,'b')
legend('left vel','edge 1','edge 2','edge 3','edge 4', 'edge 5', 'edge 6', ...
    'edge 7','edge 8','left valid','right valid','xb','xw','left valid cnt','right valid cnt')
title('silke edge detect')
%%
%plot(dd(:,1), dd(:,8),'r');
plot(dd(:,1), dd(:,9),'--r');
%plot(dd(:,1), dd(:,10),'b');
plot(dd(:,1), dd(:,11),'--b');
plot(dd(:,1), dd(:,25)/10,'b');
plot(dd(:,1), dd(:,26)/10,'r');
plot(dd(:,1), dd(:,27)/10,'k');
plot(dd(:,1), dd(:,28)/10,'m');

%% plot ballance - hand held - partly on floor
dd7 = load('edge_sensor_7.txt');
% Karla (10)
%  1    time 2.001 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 1), events 0x0 (bit-flags)
%  6  7 Wheel velocity [m/s] left, right: 0.1205 0.0912
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: 0.0329387 -0.000494256 4.65661e-10 -0.0527266
% 12 .. 32 Edge sensor: left -0.299283 1, right 4.100212 1, values 549 807 958 1119 1221 1218 1187 902, white 1, used 1, LEDhigh=0, xb=0 xw=0 xbc=0 xwc=0 lvl=20 lvr=2       whitelevel=[884 1280 1410 1427 1392 1268 1229 999]       blacklevel=[119 163 174 182 180 171 150 124]
% 12 13 Get data time [us]: 100 +ctrl 480
whitelevel=[884 1280 1410 1427 1392 1268 1229 999]; 
blacklevel=[119 163 174 182 180 171 150 124];
gain=1./(whitelevel - blacklevel);
%%
dd = dd7;
figure(7)
hold off
plot(dd(:,1), dd(:,4),':r')
grid on
hold on
plot(dd(:,1), (dd(:,16) - blacklevel(1))*gain(1),'--r')
plot(dd(:,1), (dd(:,17) - blacklevel(2))*gain(2),'g')
plot(dd(:,1), (dd(:,18) - blacklevel(3))*gain(3),'c')
plot(dd(:,1), (dd(:,19) - blacklevel(4))*gain(4),'m')
plot(dd(:,1), (dd(:,20) - blacklevel(5))*gain(5),'y')
plot(dd(:,1), (dd(:,21) - blacklevel(6))*gain(6),':k')
plot(dd(:,1), (dd(:,22) - blacklevel(7))*gain(7),'r')
plot(dd(:,1), (dd(:,23) - blacklevel(8))*gain(8),'b')
%
%plot(dd(:,1), dd(:,8),'r');
plot(dd(:,1), dd(:,13),'--r');
%plot(dd(:,1), dd(:,10),'b');
plot(dd(:,1), dd(:,15),'--b');
plot(dd(:,1), dd(:,29)/10,'b');
plot(dd(:,1), dd(:,30)/10,'r');
plot(dd(:,1), dd(:,31)/10,'k');
plot(dd(:,1), dd(:,32)/10,'m');

legend('left vel','edge 1','edge 2','edge 3','edge 4', 'edge 5', 'edge 6', ...
    'edge 7','edge 8','left valid','right valid','xb','xw','left valid cnt','right valid cnt')
title('Karla edge detect')
%%
%% plot ballance - hand held - partly on floor
dd8 = load('edge_sensor_8.txt');
dd9 = load('edge_sensor_9.txt');
dd10 = load('edge_sensor_10.txt');
% Karla (10)
%  1    time 2.007 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 1), events 0x0 (bit-flags)
%  6  7  8 Gyro x,y,z [deg/s]: 1.0376 5.67627 -6.98853
%  9 10 Wheel velocity [m/s] left, right: -0.0116 -0.0112
% 11 12 13 14 Pose x,y,h,tilt [m,m,rad,rad]: -0.00742998 -0.000129301 -0.00259112 0.00280174
% 15 .. 35 Edge sensor: left -4.180000 0, right -2.068738 0, values 585 831 893 902 867 780 734 580, white 1, used 1, LEDhigh=0, xb=0 xw=0 xbc=0 xwc=0 lvl=0 lvr=0       whitelevel=[884 1280 1410 1427 1392 1268 1229 999]       blacklevel=[119 163 174 182 180 171 150 124]
% 36 37 Get data time [us]: 100 +ctrl 480
whitelevel=[884 1280 1410 1427 1392 1268 1229 999]; 
blacklevel=[119 163 174 182 180 171 150 124];
gain=1./(whitelevel - blacklevel);
%%
dd = dd10;
figure(10)
hold off
plot(dd(:,1), dd(:,4)*0.1,':r','linewidth',3)
grid on
hold on
plot(dd(:,1), (dd(:,19) - blacklevel(1))*gain(1),'--r')
plot(dd(:,1), (dd(:,20) - blacklevel(2))*gain(2),'g')
plot(dd(:,1), (dd(:,21) - blacklevel(3))*gain(3),'c')
plot(dd(:,1), (dd(:,22) - blacklevel(4))*gain(4),'m')
plot(dd(:,1), (dd(:,23) - blacklevel(5))*gain(5),'y')
plot(dd(:,1), (dd(:,24) - blacklevel(6))*gain(6),':k')
plot(dd(:,1), (dd(:,25) - blacklevel(7))*gain(7),'r')
plot(dd(:,1), (dd(:,26) - blacklevel(8))*gain(8),'b')
%
%plot(dd(:,1), dd(:,8),'r');
plot(dd(:,1), dd(:,16),'--r');
%plot(dd(:,1), dd(:,10),'b');
plot(dd(:,1), dd(:,18),'--b');
plot(dd(:,1), dd(:,32)/10,'b');
plot(dd(:,1), dd(:,33)/10,'r');
%plot(dd(:,1), dd(:,34)/10,'k');
%plot(dd(:,1), dd(:,35)/10,'m');

legend('mission','edge 1','edge 2','edge 3','edge 4', 'edge 5', 'edge 6', ...
    'edge 7','edge 8','left valid','right valid','xb','xw','left valid cnt','right valid cnt')
title('Karla edge detect')
%plot((draw(:,1)-draw(1,1))/3600,(47800 - draw(:,9))/7,'b')
%legend('netatmo','(47800-raw)/7','Location','north')
%xlabel('hour')
%ylabel('CO2 ppm')