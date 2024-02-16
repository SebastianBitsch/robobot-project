% edge debug
clear
close all
% Alma (11)
%  1    time 1.514 sec
%  2  3  4  5   (mission 0), state 2, entered (thread 1, line 1), events 0x0 (bit-flags)
%  6  7 Motor velocity ref left, right: -0.0587 -0.0759
%  8 .. 30 Edge sensor: left 0.000000 1, right 0.000000 1, values 1813 2270 2293 2232 2344 2361 2151 1621, 
%    white 1, used 1, LEDhigh=1, xingVal=0.00 xlcnt=-1 lvcnt=-1,LineVal=1.00, lineLow=-31, lineHi=24.
whitelevel=[3006 3451 3494 3526 3547 3531 3470 2982];
blacklevel=[1404 1879 2038 2116 2154 2097 1922 1386];
% 31    Battery voltage [V]: 12.59
gain=1./(whitelevel - blacklevel);
%%
data100 = load('edge_001.txt');
data102 = load('edge_002.txt');
data103 = load('edge_003.txt');
data104 = load('edge_004.txt');
data200 = load('edge_200.txt');
data202 = load('edge_202.txt');
data203 = load('edge_203.txt');
data204 = load('edge_204.txt');
data205 = load('edge_205.txt');
data206 = load('edge_206.txt');
data207 = load('edge_207.txt');
data208 = load('edge_208.txt');
data209 = load('edge_209.txt');
data210 = load('edge_210.txt');
data300 = load('edge_300.txt');
data301 = load('edge_301.txt');
data302 = load('edge_302.txt');
data303 = load('edge_303.txt');
data304 = load('edge_304.txt');
data305 = load('edge_305.txt');
%%
dd = data305;
fg = 305;
%
L1 = 12;
led1 = (dd(:,L1) - blacklevel(1))*gain(1);
led2 = (dd(:,L1+1) - blacklevel(2))*gain(2);
led3 = (dd(:,L1+2) - blacklevel(3))*gain(3);
led4 = (dd(:,L1+3) - blacklevel(4))*gain(4);
led5 = (dd(:,L1+4) - blacklevel(5))*gain(5);
led6 = (dd(:,L1+5) - blacklevel(6))*gain(6);
led7 = (dd(:,L1+6) - blacklevel(7))*gain(7);
led8 = (dd(:,L1+7) - blacklevel(8))*gain(8);
leds = [led1,led2,led3,led4,led5,led6,led7,led8];
lineValidIdx = find(dd(:,L1- 1),1);
whiteValue = dd(:,L1+16);
% find line quality for limit
n = size(dd,1);
step = (dd(2,1) - dd(1,1))*1000;
whiteQ = zeros(n,1);
nonWhite = ones(n,1);
maxWhite = ones(n,1);
whiteLimit = zeros(n,1);
for i = lineValidIdx:n-1
  if (i > 1)
  if dd(i,L1- 1) > 0.5
    if (whiteQ(i-1) < 50)
      whiteQ(i) = whiteQ(i-1) + step;
      if whiteQ(i) > 50
          whiteQ(i) = 50;
      end
    else
      whiteQ(i) = 50;  
    end
  else
    if whiteQ(i-1) > 0
      whiteQ(i) = whiteQ(i-1) - step;
      if (whiteQ(i) < 0)
          whiteQ(i) = 0;
      end
    end
  end
  nonWhite(i) = leds(i,1);
  maxWhite(i) = leds(i,1);
  for k = 2:8
      if leds(i,k) < nonWhite(i)
          nonWhite(i) = leds(i,k);
      end
      if leds(i,k) > maxWhite(i)
          maxWhite(i) = leds(i,k);
      end
  end
  end
end
whiteLimit = whiteValue - 0.25./(1 + whiteQ);
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
plot(dd(:,1), (dd(:,8)/0.76 + 3.5)/10, '-+')
plot(dd(:,1), (dd(:,10)/0.76 + 3.5)/10,'-v')
plot(dd(:,1), dd(:,9),'--')
%plot(dd(:,1), dd(:,28),'--k')
plot(dd(:,1), dd(:,25)/10)
plot(dd(:,1), dd(:,26)/10)
%plot(dd(:,1), dd(:,25)./cos(dd(:,31)*1.2)/10,'--x');
%plot(dd(:,1), whiteLimit);
plot(dd(:,1), dd(:,31));
%plot(dd(:,1), dd(:,30)/10);
%plot(dd(:,1), nonWhite - 0.01,':');


legend('mission','LED1','LED2','LED3','LED4','LED5','LED6','LED7','LED8','left','right','valid','xingVal','xing cnt','angle');

,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,%%
missionTime = 2.764;
j = round((missionTime - dd(1,1)) / (step/1000)) + 1;
figure(300 + fg)
hold off
plot(leds(j,:))
grid on
hold on
leftIdx = dd(j,8)/0.76 + 3.5 + 1;
rightIdx = dd(j,10)/0.76 + 3.5 + 1;
plot([leftIdx, leftIdx], [0.5,1],'--')
plot([rightIdx, rightIdx], [0.5,1],'--')
plot([2,7],[dd(j,28), dd(j,28)])
plot([dd(j,30), dd(j,30)] + 0.98, [0.5,1])
plot([dd(j,31), dd(j,31)] + 0.98, [0.5,1])
plot([2,7],[whiteLimit(j), whiteLimit(j)])
%th = (whiteValue * 3 + nonWhite)/4;
%plot([2,7],[whiteLimit(j), whiteLimit(j)])
%plot([2,7],[th(j), th(j)]-0.01,'--')
plot([2,7],[maxWhite(j), maxWhite(j)]-0.03,'--o')

legend('LEDS','left edge','right edge','whiteVal','low idx','hi idx','white limit','th0.03','location','southeast')