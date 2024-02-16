% plot af data dra regbot
clear
close all
%% plot ballance - hand held - partly on floor
dd = load('../regbot_log.txt');
% Mathilde (17)
%  1    time 0.000 sec
%  2  3  4   (mission 0), state 2, thread 1, line 0
%  5  6 Motor velocity ref left, right: 0.40 0.40
%  7  8 Motor voltage [V] left, right: 4.8 5.6
%  9 10 Motor current left, right [A]: 0.054 -0.056
% 11 12 Wheel velocity [m/s] left, right: 0.0000 0.0000
% 13 14 15 16 Pose x,y,h,tilt [m,m,rad,rad]: 0 0 0 1.39803
% 17    Battery voltage [V]: 12.45
% 18 19 Get data time [us]: 100 +ctrl 340
%%
figure(1)
hold off
plot(dd(:,1), dd(:,9),'r')
grid on
hold on
plot(dd(:,1), dd(:,10),'b')
%plot(dd(:,1), dd(:,10),'m')
plot(dd(:,1), dd(:,11),'g')
plot(dd(:,1), dd(:,12),'c')
legend('current left','current right','velocity left','velocity right')
%
figure(2)
hold off
plot(dd(:,1), dd(:,7),'r')
hold on
plot(dd(:,1), dd(:,8),'b')
grid on
legend('motor voltage left','motor voltage right')
%
dd2 = dd(240:350,:);
mvolt= [mean(dd2(:,7:8)) std(dd2(:,7:8))]
wvel = [mean(dd2(:,11:12)) std(dd2(:,11:12))]
%%
%plot((draw(:,1)-draw(1,1))/3600,(47800 - draw(:,9))/7,'b')
%legend('netatmo','(47800-raw)/7','Location','north')
%xlabel('hour')
%ylabel('CO2 ppm')