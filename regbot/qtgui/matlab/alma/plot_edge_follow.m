% edge debug
clear
close all
%%
% Alma (11)
%  1    time 1.512 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 1), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: -0.1275 -0.0553
%  8  9 10 11 Pose x,y,h,tilt [m,m,rad,rad]: 0.028716 0.000118374 -0.00777337 0.0014602
% 12 .. 35 Edge sensor: left 1.773112 1, right 2.194747 1, values 1717 2311 2530 2963 3322 3441 3374 2758, 
%    white 1, used 1, LEDhigh=1, xingVal=3.68 xlcnt=0 lvcnt=5,LineVal=0.94, lineLow=5, lineHi=6, edgeAngle=0.
whitelevel=[3006 3451 3494 3526 3547 3531 3470 2982];
blacklevel=[1404 1879 2038 2116 2154 2097 1922 1386];
% 36    Battery voltage [V]: 12.20
gain=1./(whitelevel - blacklevel);
%%
data400 = load('edge_400.txt');
%%
dd = data400;
fg = 400;
%
L1 = 16;
led1 = (dd(:,L1) - blacklevel(1))*gain(1);
led2 = (dd(:,L1+1) - blacklevel(2))*gain(2);
led3 = (dd(:,L1+2) - blacklevel(3))*gain(3);
led4 = (dd(:,L1+3) - blacklevel(4))*gain(4);
led5 = (dd(:,L1+4) - blacklevel(5))*gain(5);
led6 = (dd(:,L1+5) - blacklevel(6))*gain(6);
led7 = (dd(:,L1+6) - blacklevel(7))*gain(7);
led8 = (dd(:,L1+7) - blacklevel(8))*gain(8);
leds = [led1,led2,led3,led4,led5,led6,led7,led8];
%
%% plot before tilt compensation
figure(200 + fg)
hold off
plot(dd(:,1), dd(:,4)/10,'r')
grid on
hold on
plot(dd(:,1), leds(:,1),'--r')
plot(dd(:,1), leds(:,2))
plot(dd(:,1), leds(:,3))
plot(dd(:,1), leds(:,4))
plot(dd(:,1), leds(:,5))
plot(dd(:,1), leds(:,6))
plot(dd(:,1), leds(:,7))
plot(dd(:,1), leds(:,8))
% and detection
plot(dd(:,1), (dd(:,L1-4)/0.76 + 3.5)/10, '-+')
plot(dd(:,1), (dd(:,L1-2)/0.76 + 3.5)/10,'-v')
plot(dd(:,1), dd(:,L1-1),'--')
%plot(dd(:,1), dd(:,28),'--k')
plot(dd(:,1), dd(:,29)/10)
plot(dd(:,1), dd(:,30)/10)
%plot(dd(:,1), dd(:,25)./cos(dd(:,31)*1.2)/10,'--x');
%plot(dd(:,1), whiteLimit);
plot(dd(:,1), dd(:,35));
%plot(dd(:,1), dd(:,30)/10);
%plot(dd(:,1), nonWhite - 0.01,':');


legend('mission','LED1','LED2','LED3','LED4','LED5','LED6','LED7','LED8','left','right','valid','xingVal','xing cnt','angle');

